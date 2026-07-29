/* TCP<->CAN 双向网关实现。
 * 下行：ESP8266 字节流 -> Packet 解析 -> QD4310 CAN 命令 -> ACK 回传。
 * 上行：周期 NOP 请求反馈 -> PollBus 收反馈帧 -> Packet 编码 -> TCP 发出。 */

#include "app_gateway.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "algorithm_level_current_control.h"
#include "algorithm_packet.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "core_config.h"
#include "device_esp8266.h"
#include "device_qd4310.h"

#define GATEWAY_ESP_UART_INDEX 1U  /* BSP transport 1 = USART3 */
#define GATEWAY_ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))

static esp8266_t s_wireless;
static Device_QD4310 s_motors[GATEWAY_MOTOR_COUNT];
static Device_QD4310 *s_motor_list[GATEWAY_MOTOR_COUNT];
static const uint8_t s_motor_ids[GATEWAY_MOTOR_COUNT] = GATEWAY_MOTOR_IDS;
static BSP_CAN_Handle s_can;
static Algorithm_LevelCurrentControl s_level_controller;

static Algorithm_Packet_Parser s_parser;
static uint8_t s_parser_storage[ALGORITHM_PACKET_PARSER_STORAGE_SIZE];

static uint32_t s_command_count;
static uint32_t s_command_errors;
static uint32_t s_next_feedback_ms;
static uint16_t s_tx_sequence;
static uint32_t s_last_feedback_sequence[GATEWAY_MOTOR_COUNT];
static uint32_t s_last_control_feedback_sequence;
static uint32_t s_level_control_fault_count;
static const float s_level_calibration_targets_deg[] =
    GATEWAY_LEVEL_CALIBRATION_TARGETS_DEG;
static size_t s_level_calibration_index;

static Device_QD4310 *find_motor(uint8_t motor_id)
{
    for (size_t index = 0U; index < GATEWAY_MOTOR_COUNT; ++index) {
        if (s_motors[index].config.motor_id == motor_id) {
            return &s_motors[index];
        }
    }
    return NULL;
}

static bool stop_primary_level_control(void)
{
    Device_QD4310 *motor = &s_motors[0];
    bool zero_sent = true;
    bool disabled = false;

    if (motor->initialized) {
        if (motor->command_enabled) {
            zero_sent = Device_QD4310_SetCurrent(motor, 0.0F);
        }
        disabled = Device_QD4310_Disable(motor);
    }
    Algorithm_LevelCurrentControl_Stop(&s_level_controller);
    return zero_sent && disabled;
}

static void run_primary_level_control(uint32_t now_ms)
{
    Device_QD4310 *motor = &s_motors[0];
    Device_QD4310_Feedback feedback;
    float current_a;

    if (!Algorithm_LevelCurrentControl_IsActive(&s_level_controller)) {
        return;
    }
    if (!Device_QD4310_GetFeedback(motor, now_ms, &feedback) ||
        !feedback.online ||
        ((uint32_t)(now_ms - feedback.received_at_ms) >=
            GATEWAY_LEVEL_FEEDBACK_TIMEOUT_MS)) {
        ++s_level_control_fault_count;
        (void)stop_primary_level_control();
        return;
    }
    if (feedback.sequence == s_last_control_feedback_sequence) {
        return;
    }
    s_last_control_feedback_sequence = feedback.sequence;
    if (!feedback.enabled ||
        !Algorithm_LevelCurrentControl_Update(
            &s_level_controller,
            feedback.single_turn_angle_rad,
            feedback.speed_rpm,
            &current_a) ||
        !Device_QD4310_SetCurrent(motor, current_a)) {
        ++s_level_control_fault_count;
        (void)stop_primary_level_control();
    }
}

/* 简易 ASCII 浮点解析（正负号+小数），避免拉入 strtod 的 locale 开销。 */
static bool parse_float(const char *text, size_t length, float *value)
{
    float result = 0.0F;
    float fraction = 0.1F;
    bool negative = false;
    bool in_fraction = false;
    bool any_digit = false;
    size_t index = 0U;

    if ((text == NULL) || (value == NULL) || (length == 0U)) {
        return false;
    }
    if ((text[0] == '-') || (text[0] == '+')) {
        negative = (text[0] == '-');
        index = 1U;
    }
    for (; index < length; ++index) {
        const char c = text[index];

        if ((c >= '0') && (c <= '9')) {
            if (in_fraction) {
                result += (float)(c - '0') * fraction;
                fraction *= 0.1F;
            }
            else {
                result = result * 10.0F + (float)(c - '0');
            }
            any_digit = true;
        }
        else if ((c == '.') && !in_fraction) {
            in_fraction = true;
        }
        else {
            return false;
        }
    }
    if (!any_digit) {
        return false;
    }
    *value = negative ? -result : result;
    return true;
}

static bool send_packet(
    const char *type, const uint8_t *payload, size_t payload_length)
{
    uint8_t frame[ALGORITHM_PACKET_MAX_FRAME_SIZE];
    size_t frame_length = 0U;

    if (Algorithm_Packet_Encode(
            type, s_tx_sequence, payload, payload_length,
            frame, sizeof(frame), &frame_length) !=
        ALGORITHM_PACKET_ENCODE_OK) {
        return false;
    }
    if (!esp8266_send(&s_wireless, frame, frame_length)) {
        return false;
    }
    ++s_tx_sequence;
    return true;
}

static void send_ack(uint16_t sequence, bool ok)
{
    char payload[16];
    size_t length = 0U;
    uint16_t value = sequence;
    char reverse[6];
    size_t count = 0U;

    do {
        reverse[count++] = (char)('0' + (value % 10U));
        value = (uint16_t)(value / 10U);
    } while (value > 0U);
    while (count > 0U) {
        payload[length++] = reverse[--count];
    }
    payload[length++] = ',';
    if (ok) {
        memcpy(&payload[length], "OK", 2U);
        length += 2U;
    }
    else {
        memcpy(&payload[length], "ERR", 3U);
        length += 3U;
    }
    (void)send_packet("ACK", (const uint8_t *)payload, length);
}

/* 把浮点格式化为 "-x.yyy"（3 位小数），返回写入长度。 */
static size_t format_float(char *out, float value)
{
    size_t length = 0U;
    int32_t scaled;
    uint32_t magnitude;
    uint32_t integer_part;
    uint32_t fraction_part;
    char reverse[10];
    size_t count = 0U;

    if (value < 0.0F) {
        out[length++] = '-';
        value = -value;
    }
    /* 限幅避免溢出：网关内的物理量都远小于 1e6。 */
    if (value > 999999.0F) {
        value = 999999.0F;
    }
    scaled = (int32_t)(value * 1000.0F + 0.5F);
    magnitude = (uint32_t)scaled;
    integer_part = magnitude / 1000U;
    fraction_part = magnitude % 1000U;

    do {
        reverse[count++] = (char)('0' + (integer_part % 10U));
        integer_part /= 10U;
    } while (integer_part > 0U);
    while (count > 0U) {
        out[length++] = reverse[--count];
    }
    out[length++] = '.';
    out[length++] = (char)('0' + (fraction_part / 100U));
    out[length++] = (char)('0' + ((fraction_part / 10U) % 10U));
    out[length++] = (char)('0' + (fraction_part % 10U));
    return length;
}

static void send_feedback(size_t motor_index, uint32_t now_ms)
{
    Device_QD4310_Feedback feedback;
    char payload[96];
    size_t length = 0U;

    if (!Device_QD4310_GetFeedback(
            &s_motors[motor_index], now_ms, &feedback)) {
        return;
    }
    if (feedback.sequence == s_last_feedback_sequence[motor_index]) {
        return;  /* 无新反馈帧 */
    }

    payload[length++] =
        (char)('0' + s_motors[motor_index].config.motor_id);
    payload[length++] = ',';
    payload[length++] = feedback.enabled ? '1' : '0';
    payload[length++] = ',';
    length += format_float(&payload[length], feedback.current_a);
    payload[length++] = ',';
    length += format_float(&payload[length], feedback.speed_rpm);
    payload[length++] = ',';
    length += format_float(&payload[length], feedback.single_turn_angle_rad);
    payload[length++] = ',';
    payload[length++] = feedback.online ? '1' : '0';

    if (send_packet("FB", (const uint8_t *)payload, length)) {
        s_last_feedback_sequence[motor_index] = feedback.sequence;
    }
}

static void make_primary_telemetry(
    uint32_t now_ms, APP_Gateway_Telemetry *telemetry)
{
    Device_QD4310_Feedback feedback;

    if (telemetry == NULL) {
        return;
    }
    *telemetry = (APP_Gateway_Telemetry){
        .motor_id = s_motors[0].config.motor_id,
    };
    if (!Device_QD4310_GetFeedback(
            &s_motors[0], now_ms, &feedback)) {
        return;
    }
    telemetry->enabled = feedback.enabled;
    telemetry->current_a = feedback.current_a;
    telemetry->speed_rpm = feedback.speed_rpm;
    telemetry->single_turn_angle_raw = feedback.single_turn_angle_raw;
    telemetry->single_turn_angle_rad = feedback.single_turn_angle_rad;
    telemetry->online = feedback.online;
}

/* 命令 payload："<motor_id>,<VERB>[,<value>]"，见头文件注释。 */
static bool execute_command(const Algorithm_Packet_View *view)
{
    char text[ALGORITHM_PACKET_MAX_PAYLOAD_LENGTH + 1U];
    char *verb;
    char *argument = NULL;
    char *comma;
    Device_QD4310 *motor;
    long motor_id;
    float value = 0.0F;

    if (view->payload_length > ALGORITHM_PACKET_MAX_PAYLOAD_LENGTH) {
        return false;
    }
    memcpy(text, view->payload, view->payload_length);
    text[view->payload_length] = '\0';

    comma = strchr(text, ',');
    if (comma == NULL) {
        return false;
    }
    *comma = '\0';
    verb = comma + 1;

    motor_id = strtol(text, NULL, 10);
    if ((motor_id < 0) || (motor_id > (long)DEVICE_QD4310_MOTOR_ID_MAX)) {
        return false;
    }
    motor = find_motor((uint8_t)motor_id);
    if (motor == NULL) {
        return false;
    }

    comma = strchr(verb, ',');
    if (comma != NULL) {
        *comma = '\0';
        argument = comma + 1;
    }

    /* The level controller is the sole owner of the primary motor while
     * active. A remote Disable performs the same zero-current cleanup; other
     * mode changes are rejected until the user stops level hold. */
    if ((motor == &s_motors[0]) &&
        Algorithm_LevelCurrentControl_IsActive(&s_level_controller)) {
        if (strcmp(verb, "DISABLE") == 0) {
            return stop_primary_level_control();
        }
        if (strcmp(verb, "NOP") != 0) {
            return false;
        }
    }

    if (strcmp(verb, "ENABLE") == 0) {
        return Device_QD4310_Enable(motor);
    }
    if (strcmp(verb, "DISABLE") == 0) {
        return Device_QD4310_Disable(motor);
    }
    if (strcmp(verb, "NOP") == 0) {
        return Device_QD4310_RequestFeedback(motor);
    }
    if ((argument == NULL) ||
        !parse_float(argument, strlen(argument), &value)) {
        return false;
    }
    if (strcmp(verb, "CURRENT") == 0) {
        return Device_QD4310_SetCurrent(motor, value);
    }
    if (strcmp(verb, "SPEED") == 0) {
        return Device_QD4310_SetSpeed(motor, value);
    }
    if (strcmp(verb, "LOWSPEED") == 0) {
        return Device_QD4310_SetLowSpeed(motor, value);
    }
    return false;
}

bool APP_Gateway_Init(uint32_t now_ms)
{
    BSP_UART_Handle esp_uart = BSP_UART_GetHandle(GATEWAY_ESP_UART_INDEX);
    const Algorithm_LevelCurrentControl_Config level_config = {
        .control_period_s =
            (float)GATEWAY_LEVEL_CONTROL_PERIOD_MS / 1000.0F,
        .target_deg = GATEWAY_LEVEL_TARGET_DEG,
        .target_limit_deg = GATEWAY_LEVEL_TARGET_LIMIT_DEG,
        .hard_limit_deg = GATEWAY_LEVEL_HARD_LIMIT_DEG,
        .kp_a_per_deg = GATEWAY_LEVEL_KP_A_PER_DEG,
        .ki_a_per_deg_s = GATEWAY_LEVEL_KI_A_PER_DEG_S,
        .kd_a_per_rpm = GATEWAY_LEVEL_KD_A_PER_RPM,
        .error_limit_deg = GATEWAY_LEVEL_ERROR_LIMIT_DEG,
        .deadband_deg = GATEWAY_LEVEL_DEADBAND_DEG,
        .integral_zone_deg = GATEWAY_LEVEL_INTEGRAL_ZONE_DEG,
        .integral_limit_a = GATEWAY_LEVEL_INTEGRAL_LIMIT_A,
        .current_limit_a = GATEWAY_LEVEL_CURRENT_LIMIT_A,
        .current_polarity = GATEWAY_LEVEL_CURRENT_POLARITY,
    };

    s_can = BSP_CAN_GetHandle(0U);
    if ((esp_uart == NULL) || (s_can == NULL) ||
        !isfinite(GATEWAY_LEVEL_START_MAX_SPEED_RPM) ||
        (GATEWAY_LEVEL_START_MAX_SPEED_RPM < 0.0F) ||
        (GATEWAY_LEVEL_FEEDBACK_TIMEOUT_MS <=
            GATEWAY_LEVEL_CONTROL_PERIOD_MS) ||
        (GATEWAY_LEVEL_CURRENT_LIMIT_A >
            GATEWAY_MOTOR_CURRENT_LIMIT_A) ||
        !Algorithm_LevelCurrentControl_Init(
            &s_level_controller, &level_config) ||
        !Algorithm_LevelCurrentControl_CaptureZero(
            &s_level_controller,
            Device_QD4310_AngleRawToRad(GATEWAY_LEVEL_ZERO_RAW))) {
        return false;
    }

    if (!Algorithm_Packet_Parser_Init(
            &s_parser, s_parser_storage, sizeof(s_parser_storage))) {
        return false;
    }

    for (size_t index = 0U; index < GATEWAY_MOTOR_COUNT; ++index) {
        Device_QD4310_Config config;

        if (!Device_QD4310_DefaultConfig(
                s_can, s_motor_ids[index], &config)) {
            return false;
        }
        config.send_timeout_ms = GATEWAY_CAN_SEND_TIMEOUT_MS;
        config.feedback_timeout_ms =
            GATEWAY_MOTOR_FEEDBACK_TIMEOUT_MS;
        config.current_limit_a = GATEWAY_MOTOR_CURRENT_LIMIT_A;
        config.speed_limit_rpm = GATEWAY_MOTOR_SPEED_LIMIT_RPM;
        if (!Device_QD4310_Init(&s_motors[index], &config)) {
            return false;
        }
        s_motor_list[index] = &s_motors[index];
        s_last_feedback_sequence[index] = 0U;
    }

    esp8266_init(&s_wireless, esp_uart, now_ms);
    s_command_count = 0U;
    s_command_errors = 0U;
    s_next_feedback_ms = now_ms + GATEWAY_FEEDBACK_PERIOD_MS;
    s_tx_sequence = 0U;
    s_last_control_feedback_sequence = 0U;
    s_level_control_fault_count = 0U;
    s_level_calibration_index = 0U;
    return true;
}

void APP_Gateway_Step(
    uint32_t now_ms, APP_Gateway_Telemetry *primary_telemetry)
{
    uint8_t byte;
    size_t processed = 0U;

    esp8266_poll(&s_wireless, now_ms);

    /* 下行：解析 TCP 字节流中的 Packet 帧并执行 */
    while (esp8266_read(&s_wireless, &byte)) {
        Algorithm_Packet_View view;

        if (Algorithm_Packet_Parser_Feed(&s_parser, byte, &view) !=
            ALGORITHM_PACKET_FEED_FRAME_READY) {
            continue;
        }
        if ((view.type_length == 3U) &&
            (memcmp(view.type, "CMD", 3U) == 0)) {
            const bool ok = execute_command(&view);

            ++s_command_count;
            if (!ok) {
                ++s_command_errors;
            }
            send_ack(view.sequence, ok);
        }
    }

    /* 上行：收 CAN 反馈帧并回传 */
    (void)Device_QD4310_PollBus(
        s_can, s_motor_list, GATEWAY_MOTOR_COUNT, now_ms,
        BSP_CAN_CLASSIC_MAX_DATA_LENGTH, &processed);
    run_primary_level_control(now_ms);
    for (size_t index = 0U; index < GATEWAY_MOTOR_COUNT; ++index) {
        send_feedback(index, now_ms);
    }

    /* 周期 NOP：应答式协议需要主动请求反馈 */
    if ((int32_t)(now_ms - s_next_feedback_ms) >= 0) {
        s_next_feedback_ms = now_ms + GATEWAY_FEEDBACK_PERIOD_MS;
        for (size_t index = 0U; index < GATEWAY_MOTOR_COUNT; ++index) {
            if ((index == 0U) &&
                Algorithm_LevelCurrentControl_IsActive(
                    &s_level_controller)) {
                continue;  /* CURRENT command already requests fresh feedback. */
            }
            (void)Device_QD4310_RequestFeedback(&s_motors[index]);
        }
    }
    make_primary_telemetry(now_ms, primary_telemetry);
}

bool APP_Gateway_TogglePrimaryMotor(uint32_t now_ms)
{
    Device_QD4310 *motor = &s_motors[0];
    Device_QD4310_Feedback feedback;

    if (!motor->initialized) {
        return false;
    }
    if (Algorithm_LevelCurrentControl_IsActive(&s_level_controller) ||
        motor->command_enabled) {
        return stop_primary_level_control();
    }
    if (!Device_QD4310_GetFeedback(
            motor, now_ms, &feedback) ||
        !feedback.online || feedback.enabled ||
        ((uint32_t)(now_ms - feedback.received_at_ms) >=
            GATEWAY_LEVEL_FEEDBACK_TIMEOUT_MS) ||
        !isfinite(feedback.speed_rpm) ||
        (fabsf(feedback.speed_rpm) >
            GATEWAY_LEVEL_START_MAX_SPEED_RPM) ||
        !Algorithm_LevelCurrentControl_Start(&s_level_controller)) {
        return false;
    }
    s_last_control_feedback_sequence = feedback.sequence;
    if (Device_QD4310_Enable(motor) &&
        Device_QD4310_SetCurrent(motor, 0.0F)) {
        return true;
    }
    (void)stop_primary_level_control();
    return false;
}

bool APP_Gateway_AdvancePrimaryMotorCalibration(uint32_t now_ms)
{
    Device_QD4310 *motor = &s_motors[0];
    const bool active =
        Algorithm_LevelCurrentControl_IsActive(&s_level_controller);

    if (!motor->initialized ||
        (GATEWAY_ARRAY_COUNT(s_level_calibration_targets_deg) == 0U)) {
        return false;
    }
    if (active || motor->command_enabled) {
        if (!active || !motor->command_enabled) {
            (void)stop_primary_level_control();
            return false;
        }
        if ((s_level_calibration_index + 1U) >=
            GATEWAY_ARRAY_COUNT(s_level_calibration_targets_deg)) {
            s_level_calibration_index = 0U;
            return stop_primary_level_control();
        }
        if (!Algorithm_LevelCurrentControl_SetTargetDeg(
                &s_level_controller,
                s_level_calibration_targets_deg[
                    s_level_calibration_index + 1U])) {
            return false;
        }
        ++s_level_calibration_index;
        return true;
    }

    if (!Algorithm_LevelCurrentControl_SetTargetDeg(
            &s_level_controller, s_level_calibration_targets_deg[0])) {
        return false;
    }
    s_level_calibration_index = 0U;
    return APP_Gateway_TogglePrimaryMotor(now_ms);
}

uint32_t APP_Gateway_CommandCount(void)
{
    return s_command_count;
}

uint32_t APP_Gateway_CommandErrorCount(void)
{
    return s_command_errors;
}

bool APP_Gateway_WirelessReady(void)
{
    return esp8266_status(&s_wireless) == ESP8266_SESSION_READY;
}
