/* TCP<->CAN 双向网关实现。
 * 下行：ESP8266 字节流 -> Packet 解析 -> QD4310 CAN 命令 -> ACK 回传。
 * 上行：周期 NOP 请求反馈 -> PollBus 收反馈帧 -> Packet 编码 -> TCP 发出。 */

#include "app_gateway.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "algorithm_ball_position_control.h"
#include "algorithm_ball_state_estimator.h"
#include "algorithm_chassis_acceleration_fusion.h"
#include "algorithm_level_current_control.h"
#include "algorithm_packet.h"
#include "algorithm_rod_angle_map.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "core_config.h"
#include "device_esp8266.h"
#include "device_qd4310.h"

#define GATEWAY_ESP_UART_INDEX 1U  /* BSP transport 1 = USART3 */
#define GATEWAY_ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))

typedef enum {
    GATEWAY_LEVEL_PHASE_STOPPED = 0,
    GATEWAY_LEVEL_PHASE_ZEROING,
    GATEWAY_LEVEL_PHASE_ACCELERATION_COMPENSATION,
} Gateway_LevelPhase;

static esp8266_t s_wireless;
static Device_QD4310 s_motors[GATEWAY_MOTOR_COUNT];
static Device_QD4310 *s_motor_list[GATEWAY_MOTOR_COUNT];
static const uint8_t s_motor_ids[GATEWAY_MOTOR_COUNT] = GATEWAY_MOTOR_IDS;
static BSP_CAN_Handle s_can;
static Algorithm_LevelCurrentControl s_level_controller;
static Algorithm_BallPositionControl s_ball_controller;
static Algorithm_BallStateEstimator s_ball_estimator;
static Algorithm_ChassisAccelerationFusion s_chassis_fusion;
static Algorithm_RodAngleMap s_rod_angle_map;
static const Algorithm_RodAngleMap_Point s_rod_angle_points[] =
    GATEWAY_ROD_ANGLE_CALIBRATION_POINTS;

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
static float s_rod_trim_angle_deg;
static float s_rod_minimum_angle_deg;
static float s_rod_maximum_angle_deg;
static float s_rod_command_deg;
static uint16_t s_motor_target_raw;
static float s_motor_command_current_a;
static int32_t s_selected_chassis_acceleration_mm_s2;
static int32_t s_planned_chassis_acceleration_mm_s2;
static bool s_last_chassis_motion_sequence_seen;
static uint16_t s_last_chassis_motion_sequence;
static uint32_t s_last_chassis_motion_sample_ms;
static bool s_chassis_fusion_trusted;
static bool s_last_ball_sequence_seen;
static uint16_t s_last_ball_sequence;
static bool s_ball_measurement_valid;
static bool s_chassis_acceleration_valid;
static bool s_rod_angle_saturated;
static bool s_motor_raw_limit_blocked;
static Gateway_LevelPhase s_level_phase;
static bool s_zero_entry_stable_seen;
static uint32_t s_zero_entry_stable_since_ms;
static float s_chassis_compensation_angle_deg;

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float raw_target_to_relative_motor_deg(float raw)
{
    float delta = raw - (float)GATEWAY_LEVEL_ZERO_RAW;

    if (delta > 32767.5F) {
        delta -= 65536.0F;
    }
    else if (delta < -32768.0F) {
        delta += 65536.0F;
    }
    return delta * (360.0F / 65535.0F);
}

static bool apply_rod_target_deg(float requested_rod_angle_deg)
{
    float target_raw;
    float motor_target_deg;

    if (!isfinite(requested_rod_angle_deg)) {
        return false;
    }
    s_rod_command_deg = clamp_float(
        requested_rod_angle_deg,
        s_rod_minimum_angle_deg,
        s_rod_maximum_angle_deg);
    s_rod_angle_saturated =
        s_rod_command_deg != requested_rod_angle_deg;
    if (!Algorithm_RodAngleMap_RodDegToRaw(
            &s_rod_angle_map, s_rod_command_deg, &target_raw)) {
        return false;
    }
    if ((target_raw < (float)GATEWAY_LEVEL_RAW_MIN_LIMIT) ||
        (target_raw > (float)GATEWAY_LEVEL_RAW_MAX_LIMIT)) {
        s_motor_raw_limit_blocked = true;
        return false;
    }
    motor_target_deg = raw_target_to_relative_motor_deg(target_raw);
    if (!Algorithm_LevelCurrentControl_SetTrackingTargetDeg(
            &s_level_controller, motor_target_deg)) {
        return false;
    }
    s_motor_target_raw = (uint16_t)(target_raw + 0.5F);
    return true;
}

static int32_t select_chassis_acceleration(
    uint32_t now_ms,
    const APP_Gateway_ControlInput *input)
{
    uint32_t elapsed_ms;
    uint32_t compensated_elapsed_ms;
    uint32_t remaining_ms;

    if ((input == NULL) || !input->chassis_acceleration_valid) {
        return 0;
    }
    elapsed_ms = now_ms - input->acceleration_received_at_ms;
    compensated_elapsed_ms = elapsed_ms + GATEWAY_ACC_TRANSPORT_BOUND_MS;
    if (compensated_elapsed_ms < elapsed_ms) {
        compensated_elapsed_ms = UINT32_MAX;
    }
    if (input->acceleration_next_after_ms == UINT16_MAX) {
        return input->acceleration_now_mm_s2;
    }
    remaining_ms =
        ((uint32_t)input->acceleration_next_after_ms >
            compensated_elapsed_ms) ?
        (uint32_t)input->acceleration_next_after_ms -
            compensated_elapsed_ms : 0U;
    return (remaining_ms <= GATEWAY_ACC_PREVIEW_LEAD_MS) ?
        input->acceleration_next_mm_s2 :
        input->acceleration_now_mm_s2;
}

static int32_t round_float_to_i32(float value)
{
    return (int32_t)(value + ((value >= 0.0F) ? 0.5F : -0.5F));
}

static void update_chassis_acceleration(
    uint32_t now_ms,
    const APP_Gateway_ControlInput *input)
{
    Algorithm_ChassisAccelerationFusion_State state;

    s_chassis_acceleration_valid =
        (input != NULL) && input->chassis_acceleration_valid;
    s_planned_chassis_acceleration_mm_s2 = select_chassis_acceleration(
        now_ms, input);
    s_selected_chassis_acceleration_mm_s2 =
        s_planned_chassis_acceleration_mm_s2;
    s_chassis_fusion_trusted = false;

    if (GATEWAY_CHASSIS_ACCEL_FUSION_ENABLE == 0U) {
        return;
    }
    if (!s_chassis_acceleration_valid) {
        Algorithm_ChassisAccelerationFusion_Reset(&s_chassis_fusion, now_ms);
        s_last_chassis_motion_sequence_seen = false;
        return;
    }
    if (!Algorithm_ChassisAccelerationFusion_Predict(
            &s_chassis_fusion,
            now_ms,
            (float)s_planned_chassis_acceleration_mm_s2)) {
        s_last_chassis_motion_sequence_seen = false;
        return;
    }
    if ((input != NULL) && input->chassis_motion_measurement_valid &&
        (!s_last_chassis_motion_sequence_seen ||
         (input->chassis_motion_sequence !=
            s_last_chassis_motion_sequence) ||
         (input->chassis_motion_sample_ms !=
            s_last_chassis_motion_sample_ms))) {
        if (input->measured_chassis_acceleration_valid ||
            input->measured_chassis_wheel_speed_valid) {
            (void)Algorithm_ChassisAccelerationFusion_Correct(
                &s_chassis_fusion,
                now_ms,
                input->measured_chassis_wheel_speed_valid,
                (float)input->measured_chassis_wheel_speed_mm_s,
                input->measured_chassis_acceleration_valid,
                (float)input->measured_chassis_acceleration_mm_s2);
        }
        s_last_chassis_motion_sequence = input->chassis_motion_sequence;
        s_last_chassis_motion_sample_ms = input->chassis_motion_sample_ms;
        s_last_chassis_motion_sequence_seen = true;
    }
    if (Algorithm_ChassisAccelerationFusion_IsTrusted(
            &s_chassis_fusion, now_ms) &&
        Algorithm_ChassisAccelerationFusion_GetState(
            &s_chassis_fusion, &state, NULL)) {
        s_selected_chassis_acceleration_mm_s2 =
            round_float_to_i32(state.acceleration_mm_s2);
        s_chassis_fusion_trusted = true;
    }
}

static bool get_estimator_rod_angle_deg(
    uint32_t now_ms,
    float *rod_angle_deg)
{
    Device_QD4310_Feedback feedback;

    return (rod_angle_deg != NULL) &&
        Device_QD4310_GetFeedback(&s_motors[0], now_ms, &feedback) &&
        feedback.online &&
        ((uint32_t)(now_ms - feedback.received_at_ms) <
            GATEWAY_LEVEL_FEEDBACK_TIMEOUT_MS) &&
        Algorithm_RodAngleMap_RawToRodDeg(
            &s_rod_angle_map,
            (float)feedback.single_turn_angle_raw,
            rod_angle_deg);
}

static void reset_estimated_ball_feedback(uint32_t now_ms)
{
    if (s_ball_estimator.state_valid ||
        (s_ball_estimator.history_count > 0U)) {
        Algorithm_BallStateEstimator_Reset(&s_ball_estimator, now_ms);
    }
    Algorithm_BallPositionControl_ResetMeasurement(&s_ball_controller);
    s_last_ball_sequence_seen = false;
}

static bool update_kalman_ball_feedback(
    uint32_t now_ms,
    const APP_Gateway_ControlInput *input,
    float *ball_angle_deg)
{
    Algorithm_BallStateEstimator_State state;
    float rod_angle_deg;

    if ((input == NULL) || !input->ball_measurement_valid ||
        !get_estimator_rod_angle_deg(now_ms, &rod_angle_deg)) {
        reset_estimated_ball_feedback(now_ms);
        return true;
    }
    if (!Algorithm_BallStateEstimator_Predict(
            &s_ball_estimator,
            now_ms,
            rod_angle_deg,
            (float)s_selected_chassis_acceleration_mm_s2)) {
        Algorithm_BallPositionControl_ResetMeasurement(&s_ball_controller);
        s_last_ball_sequence_seen = false;
        return true;
    }
    if (!s_last_ball_sequence_seen ||
        (input->ball_sequence != s_last_ball_sequence)) {
        if (!Algorithm_BallStateEstimator_CorrectPosition(
                &s_ball_estimator,
                now_ms,
                input->ball_measurement_age_ms,
                (float)input->ball_position_mm)) {
            reset_estimated_ball_feedback(now_ms);
            return true;
        }
        s_last_ball_sequence = input->ball_sequence;
        s_last_ball_sequence_seen = true;
    }
    if (!Algorithm_BallStateEstimator_IsTrusted(&s_ball_estimator, now_ms) ||
        !Algorithm_BallStateEstimator_GetState(
            &s_ball_estimator, &state, NULL)) {
        Algorithm_BallPositionControl_ResetMeasurement(&s_ball_controller);
        return true;
    }
    if (!Algorithm_BallPositionControl_SetEstimatedState(
            &s_ball_controller,
            state.position_mm,
            state.velocity_mm_s)) {
        reset_estimated_ball_feedback(now_ms);
        return false;
    }
    return Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &s_ball_controller, ball_angle_deg);
}

static bool update_primary_tracking_target(
    uint32_t now_ms,
    const APP_Gateway_ControlInput *input)
{
    float ball_angle_deg = 0.0F;
    float chassis_angle_deg = 0.0F;
    float requested_offset_deg;
    float limited_offset_deg;
    bool total_angle_saturated;
    bool target_applied;

    s_chassis_compensation_angle_deg = 0.0F;
    if (s_level_phase == GATEWAY_LEVEL_PHASE_ZEROING) {
        s_chassis_acceleration_valid = false;
        s_selected_chassis_acceleration_mm_s2 = 0;
        return apply_rod_target_deg(0.0F);
    }
    update_chassis_acceleration(now_ms, input);
    if (s_level_phase ==
            GATEWAY_LEVEL_PHASE_ACCELERATION_COMPENSATION) {
        if ((input != NULL) && input->chassis_motion_measurement_valid &&
            input->measured_chassis_acceleration_valid) {
            s_selected_chassis_acceleration_mm_s2 =
                input->measured_chassis_acceleration_mm_s2;
            s_chassis_acceleration_valid = true;
        }
    }
    else {
        s_chassis_acceleration_valid = false;
        s_selected_chassis_acceleration_mm_s2 = 0;
    }
    s_ball_measurement_valid =
        (GATEWAY_BALL_VISUAL_CONTROL_ENABLE != 0U) &&
        (input != NULL) && input->ball_measurement_valid;
    if (GATEWAY_BALL_VISUAL_CONTROL_ENABLE == 0U) {
        reset_estimated_ball_feedback(now_ms);
    }
    else if (GATEWAY_BALL_KALMAN_ENABLE != 0U) {
        if (!update_kalman_ball_feedback(
                now_ms, input, &ball_angle_deg)) {
            return false;
        }
    }
    else if (s_ball_measurement_valid) {
        if (!s_last_ball_sequence_seen ||
            (input->ball_sequence != s_last_ball_sequence)) {
            if (!Algorithm_BallPositionControl_UpdateMeasurement(
                    &s_ball_controller,
                    input->ball_sample_ms,
                    (float)input->ball_position_mm)) {
                return false;
            }
            s_last_ball_sequence = input->ball_sequence;
            s_last_ball_sequence_seen = true;
        }
        if (!Algorithm_BallPositionControl_ComputeRodAngleDeg(
                &s_ball_controller, &ball_angle_deg)) {
            return false;
        }
    }
    else if (s_ball_controller.measurement_seen) {
        Algorithm_BallPositionControl_ResetMeasurement(&s_ball_controller);
        s_last_ball_sequence_seen = false;
    }

    if (s_chassis_acceleration_valid) {
        chassis_angle_deg = GATEWAY_CHASSIS_FF_GAIN *
            GATEWAY_CHASSIS_FF_POLARITY *
            atan2f(
                (float)s_selected_chassis_acceleration_mm_s2,
            GATEWAY_BALL_GRAVITY_MM_S2) *
            (180.0F / 3.14159265358979323846F);
        chassis_angle_deg = clamp_float(
            chassis_angle_deg,
            -GATEWAY_CHASSIS_FF_ANGLE_LIMIT_DEG,
            GATEWAY_CHASSIS_FF_ANGLE_LIMIT_DEG);
    }
    s_chassis_compensation_angle_deg = chassis_angle_deg;
    requested_offset_deg = ball_angle_deg + chassis_angle_deg;
    limited_offset_deg = clamp_float(
        requested_offset_deg,
        -GATEWAY_TOTAL_ROD_COMMAND_LIMIT_DEG,
        GATEWAY_TOTAL_ROD_COMMAND_LIMIT_DEG);
    total_angle_saturated = limited_offset_deg != requested_offset_deg;
    target_applied = apply_rod_target_deg(
        s_rod_trim_angle_deg + limited_offset_deg);
    s_rod_angle_saturated =
        s_rod_angle_saturated || total_angle_saturated;
    return target_applied;
}

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

    s_level_phase = GATEWAY_LEVEL_PHASE_STOPPED;
    s_zero_entry_stable_seen = false;
    s_chassis_compensation_angle_deg = 0.0F;
    s_motor_command_current_a = 0.0F;
    if (motor->initialized) {
        if (motor->command_enabled) {
            zero_sent = Device_QD4310_SetCurrent(motor, 0.0F);
        }
        disabled = Device_QD4310_Disable(motor);
    }
    Algorithm_LevelCurrentControl_Stop(&s_level_controller);
    return zero_sent && disabled;
}

static void enforce_primary_motor_disabled_when_stopped(uint32_t now_ms)
{
    Device_QD4310 *motor = &s_motors[0];
    Device_QD4310_Feedback feedback;

    if (Algorithm_LevelCurrentControl_IsActive(&s_level_controller) ||
        !Device_QD4310_GetFeedback(motor, now_ms, &feedback) ||
        !feedback.online || !feedback.enabled) {
        return;
    }

    s_motor_command_current_a = 0.0F;
    if (motor->command_enabled) {
        (void)Device_QD4310_SetCurrent(motor, 0.0F);
    }
    (void)Device_QD4310_Disable(motor);
}

static void run_primary_level_control(uint32_t now_ms)
{
    Device_QD4310 *motor = &s_motors[0];
    Device_QD4310_Feedback feedback;
    float current_a;
    float relative_angle_deg;
    bool raw_recovery_active;

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
    if ((s_level_phase == GATEWAY_LEVEL_PHASE_ZEROING) &&
        Algorithm_LevelCurrentControl_GetRelativeAngleDeg(
            &s_level_controller,
            feedback.single_turn_angle_rad,
            &relative_angle_deg) &&
        (fabsf(relative_angle_deg) <=
            GATEWAY_LEVEL_ZERO_ENTRY_TOL_DEG) &&
        (fabsf(feedback.speed_rpm) <=
            GATEWAY_LEVEL_ZERO_ENTRY_MAX_RPM)) {
        if (!s_zero_entry_stable_seen) {
            s_zero_entry_stable_seen = true;
            s_zero_entry_stable_since_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - s_zero_entry_stable_since_ms) >=
            GATEWAY_LEVEL_ZERO_ENTRY_SETTLE_MS) {
            s_level_phase =
                GATEWAY_LEVEL_PHASE_ACCELERATION_COMPENSATION;
        }
    }
    else {
        s_zero_entry_stable_seen = false;
    }
    if (!feedback.enabled ||
        !Algorithm_LevelCurrentControl_ComputeRawRecoveryCurrent(
            &s_level_controller,
            feedback.single_turn_angle_raw,
            &current_a,
            &raw_recovery_active) ||
        (!raw_recovery_active &&
         !Algorithm_LevelCurrentControl_Update(
             &s_level_controller,
             feedback.single_turn_angle_rad,
             feedback.speed_rpm,
             &current_a))) {
        ++s_level_control_fault_count;
        (void)stop_primary_level_control();
        return;
    }
    if (raw_recovery_active) {
        s_motor_raw_limit_blocked = true;
    }
    if (((feedback.single_turn_angle_raw <=
                GATEWAY_LEVEL_RAW_MIN_LIMIT) &&
            ((current_a * GATEWAY_LEVEL_CURRENT_POLARITY) < 0.0F)) ||
        ((feedback.single_turn_angle_raw >=
                GATEWAY_LEVEL_RAW_MAX_LIMIT) &&
            ((current_a * GATEWAY_LEVEL_CURRENT_POLARITY) > 0.0F))) {
        current_a = 0.0F;
        s_motor_raw_limit_blocked = true;
    }
    s_motor_command_current_a = current_a;
    if (!Device_QD4310_SetCurrent(motor, current_a)) {
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
    Algorithm_BallStateEstimator_State estimator_state;
    Algorithm_BallStateEstimator_Covariance estimator_covariance;
    Algorithm_ChassisAccelerationFusion_State fusion_state;
    Algorithm_ChassisAccelerationFusion_Covariance fusion_covariance;
    const bool estimator_state_valid = Algorithm_BallStateEstimator_GetState(
        &s_ball_estimator, &estimator_state, &estimator_covariance);
    const bool fusion_state_valid =
        Algorithm_ChassisAccelerationFusion_GetState(
            &s_chassis_fusion, &fusion_state, &fusion_covariance);

    if (telemetry == NULL) {
        return;
    }
    *telemetry = (APP_Gateway_Telemetry){
        .motor_id = s_motors[0].config.motor_id,
        .rod_command_deg = s_rod_command_deg,
        .target_single_turn_angle_raw = s_motor_target_raw,
        .command_current_a = s_motor_command_current_a,
        .ball_position_mm = s_ball_controller.position_mm,
        .ball_velocity_mm_s = s_ball_controller.velocity_mm_s,
        .ball_acceleration_bias_mm_s2 = estimator_state_valid ?
            estimator_state.acceleration_bias_mm_s2 : 0.0F,
        .ball_position_variance_mm2 = estimator_state_valid ?
            estimator_covariance.p00 : 0.0F,
        .ball_velocity_variance_mm2_s2 = estimator_state_valid ?
            estimator_covariance.p11 : 0.0F,
        .chassis_acceleration_mm_s2 =
            s_selected_chassis_acceleration_mm_s2,
        .planned_chassis_acceleration_mm_s2 =
            s_planned_chassis_acceleration_mm_s2,
        .chassis_velocity_estimate_mm_s = fusion_state_valid ?
            fusion_state.velocity_mm_s : 0.0F,
        .chassis_imu_bias_estimate_mm_s2 = fusion_state_valid ?
            fusion_state.imu_bias_mm_s2 : 0.0F,
        .chassis_acceleration_variance_mm2_s4 = fusion_state_valid ?
            fusion_covariance.p11 : 0.0F,
        .ball_measurement_valid = s_ball_measurement_valid,
        .ball_velocity_valid = s_ball_controller.velocity_valid,
        .ball_estimator_trusted =
            (GATEWAY_BALL_KALMAN_ENABLE != 0U) &&
            Algorithm_BallStateEstimator_IsTrusted(
                &s_ball_estimator, now_ms),
        .chassis_acceleration_valid = s_chassis_acceleration_valid,
        .chassis_acceleration_fusion_trusted = s_chassis_fusion_trusted,
        .rod_angle_saturated = s_rod_angle_saturated,
        .motor_raw_limit_blocked = s_motor_raw_limit_blocked,
        .level_control_phase = (uint8_t)s_level_phase,
        .chassis_compensation_angle_deg =
            s_chassis_compensation_angle_deg,
        .ball_estimator_innovation_reject_count =
            s_ball_estimator.innovation_reject_count,
        .chassis_fusion_innovation_reject_count =
            s_chassis_fusion.wheel_innovation_reject_count +
            s_chassis_fusion.imu_innovation_reject_count,
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

    /* The KEY-owned, raw-limited level controller is the sole owner of the
     * primary motor. TCP may only request feedback or force a safe stop; raw
     * ENABLE/CURRENT/SPEED commands must never bypass the absolute limits. */
    if (motor == &s_motors[0]) {
        if (strcmp(verb, "DISABLE") == 0) {
            return Algorithm_LevelCurrentControl_IsActive(
                    &s_level_controller) ?
                stop_primary_level_control() :
                Device_QD4310_Disable(motor);
        }
        if (strcmp(verb, "NOP") == 0) {
            return Device_QD4310_RequestFeedback(motor);
        }
        return false;
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
    BSP_UART_Handle esp_uart = NULL;
    const Algorithm_LevelCurrentControl_Config level_config = {
        .control_period_s =
            (float)GATEWAY_LEVEL_CONTROL_PERIOD_MS / 1000.0F,
        .target_deg = 0.0F,
        .target_limit_deg = GATEWAY_LEVEL_TARGET_LIMIT_DEG,
        .hard_limit_deg = GATEWAY_LEVEL_HARD_LIMIT_DEG,
        .kp_a_per_deg = GATEWAY_LEVEL_KP_A_PER_DEG,
        .ki_a_per_deg_s = GATEWAY_LEVEL_KI_A_PER_DEG_S,
        .kd_a_per_rpm = GATEWAY_LEVEL_KD_A_PER_RPM,
        .feedforward_current_a =
            GATEWAY_LEVEL_FEEDFORWARD_CURRENT_A,
        .error_limit_deg = GATEWAY_LEVEL_ERROR_LIMIT_DEG,
        .deadband_deg = GATEWAY_LEVEL_DEADBAND_DEG,
        .integral_zone_deg = GATEWAY_LEVEL_INTEGRAL_ZONE_DEG,
        .integral_limit_a = GATEWAY_LEVEL_INTEGRAL_LIMIT_A,
        .current_limit_a = GATEWAY_LEVEL_CURRENT_LIMIT_A,
        .current_polarity = GATEWAY_LEVEL_CURRENT_POLARITY,
        .raw_min_limit = GATEWAY_LEVEL_RAW_MIN_LIMIT,
        .raw_max_limit = GATEWAY_LEVEL_RAW_MAX_LIMIT,
        .raw_recovery_current_a =
            GATEWAY_LEVEL_RAW_RECOVERY_CURRENT_A,
    };
    const Algorithm_BallPositionControl_Config ball_config = {
        .target_position_mm = GATEWAY_BALL_TARGET_MM,
        .measurement_polarity = GATEWAY_BALL_MEASUREMENT_POLARITY,
        .position_gain_per_s2 = GATEWAY_BALL_POSITION_KP_PER_S2,
        .velocity_gain_per_s = GATEWAY_BALL_VELOCITY_KD_PER_S,
        .velocity_filter_tau_s = GATEWAY_BALL_VELOCITY_FILTER_TAU_S,
        .gravity_mm_s2 = GATEWAY_BALL_GRAVITY_MM_S2,
        .inertia_ratio = GATEWAY_BALL_INERTIA_RATIO,
        .rod_angle_limit_deg = GATEWAY_BALL_ROD_ANGLE_LIMIT_DEG,
        .maximum_velocity_dt_ms = GATEWAY_BALL_MAX_VELOCITY_DT_MS,
    };
    const Algorithm_BallStateEstimator_Config estimator_config = {
        .measurement_polarity = GATEWAY_BALL_MEASUREMENT_POLARITY,
        .gravity_mm_s2 = GATEWAY_BALL_GRAVITY_MM_S2,
        .inertia_ratio = GATEWAY_BALL_INERTIA_RATIO,
        .position_measurement_variance_mm2 =
            GATEWAY_BALL_KALMAN_POSITION_R_MM2,
        .model_acceleration_noise_density_mm2_s3 =
            GATEWAY_BALL_KALMAN_MODEL_ACCEL_Q_MM2_S3,
        .bias_walk_variance_mm2_s4_per_s =
            GATEWAY_BALL_KALMAN_BIAS_WALK_Q_MM2_S4_S,
        .initial_position_variance_mm2 =
            GATEWAY_BALL_KALMAN_INITIAL_POSITION_P_MM2,
        .initial_velocity_variance_mm2_s2 =
            GATEWAY_BALL_KALMAN_INITIAL_VELOCITY_P_MM2_S2,
        .initial_bias_variance_mm2_s4 =
            GATEWAY_BALL_KALMAN_INITIAL_BIAS_P_MM2_S4,
        .innovation_gate_sigma =
            GATEWAY_BALL_KALMAN_INNOVATION_GATE_SIGMA,
        .maximum_absolute_position_mm =
            GATEWAY_BALL_KALMAN_MAX_POSITION_MM,
        .maximum_absolute_velocity_mm_s =
            GATEWAY_BALL_KALMAN_MAX_VELOCITY_MM_S,
        .maximum_absolute_bias_mm_s2 =
            GATEWAY_BALL_KALMAN_MAX_BIAS_MM_S2,
        .trusted_position_variance_mm2 =
            GATEWAY_BALL_KALMAN_TRUST_POSITION_P_MM2,
        .trusted_velocity_variance_mm2_s2 =
            GATEWAY_BALL_KALMAN_TRUST_VELOCITY_P_MM2_S2,
        .maximum_prediction_step_ms =
            GATEWAY_BALL_KALMAN_MAX_PREDICT_STEP_MS,
        .maximum_measurement_age_ms =
            VISION_LINK_MEASUREMENT_MAX_AGE_MS,
        .minimum_accepted_measurements =
            GATEWAY_BALL_KALMAN_MIN_ACCEPTED_VIS,
    };
    const Algorithm_ChassisAccelerationFusion_Config fusion_config = {
        .plan_gain = GATEWAY_CHASSIS_FUSION_PLAN_GAIN,
        .response_time_constant_s =
            GATEWAY_CHASSIS_FUSION_RESPONSE_TAU_S,
        .plan_delay_ms = GATEWAY_CHASSIS_FUSION_PLAN_DELAY_MS,
        .wheel_speed_measurement_variance_mm2_s2 =
            GATEWAY_CHASSIS_FUSION_WHEEL_R_MM2_S2,
        .imu_acceleration_measurement_variance_mm2_s4 =
            GATEWAY_CHASSIS_FUSION_IMU_R_MM2_S4,
        .velocity_noise_density_mm2_s3 =
            GATEWAY_CHASSIS_FUSION_VELOCITY_Q_MM2_S3,
        .acceleration_noise_density_mm2_s5 =
            GATEWAY_CHASSIS_FUSION_ACCEL_Q_MM2_S5,
        .imu_bias_walk_noise_density_mm2_s5 =
            GATEWAY_CHASSIS_FUSION_BIAS_Q_MM2_S5,
        .initial_velocity_variance_mm2_s2 =
            GATEWAY_CHASSIS_FUSION_INITIAL_VELOCITY_P_MM2_S2,
        .initial_acceleration_variance_mm2_s4 =
            GATEWAY_CHASSIS_FUSION_INITIAL_ACCEL_P_MM2_S4,
        .initial_imu_bias_variance_mm2_s4 =
            GATEWAY_CHASSIS_FUSION_INITIAL_BIAS_P_MM2_S4,
        .innovation_gate_sigma = GATEWAY_CHASSIS_FUSION_GATE_SIGMA,
        .maximum_absolute_velocity_mm_s =
            GATEWAY_CHASSIS_FUSION_MAX_VELOCITY_MM_S,
        .maximum_absolute_acceleration_mm_s2 =
            GATEWAY_CHASSIS_FUSION_MAX_ACCEL_MM_S2,
        .maximum_absolute_imu_bias_mm_s2 =
            GATEWAY_CHASSIS_FUSION_MAX_BIAS_MM_S2,
        .trusted_acceleration_variance_mm2_s4 =
            GATEWAY_CHASSIS_FUSION_TRUST_ACCEL_P_MM2_S4,
        .maximum_prediction_step_ms =
            GATEWAY_CHASSIS_FUSION_MAX_PREDICT_STEP_MS,
        .maximum_measurement_age_ms =
            GATEWAY_CHASSIS_FUSION_MAX_MEAS_AGE_MS,
        .minimum_accepted_wheel_measurements =
            GATEWAY_CHASSIS_FUSION_MIN_WHEEL_SAMPLES,
        .minimum_accepted_imu_measurements =
            GATEWAY_CHASSIS_FUSION_MIN_IMU_SAMPLES,
    };

    if (GATEWAY_ESP8266_ENABLE != 0U) {
        esp_uart = BSP_UART_GetHandle(GATEWAY_ESP_UART_INDEX);
    }
    s_can = BSP_CAN_GetHandle(0U);
    if (((GATEWAY_ESP8266_ENABLE != 0U) && (esp_uart == NULL)) ||
        (s_can == NULL) ||
        !isfinite(GATEWAY_LEVEL_START_MAX_SPEED_RPM) ||
        (GATEWAY_LEVEL_START_MAX_SPEED_RPM < 0.0F) ||
        (GATEWAY_LEVEL_FEEDBACK_TIMEOUT_MS <=
            GATEWAY_LEVEL_CONTROL_PERIOD_MS) ||
        (GATEWAY_LEVEL_CURRENT_LIMIT_A >
            GATEWAY_MOTOR_CURRENT_LIMIT_A) ||
        !isfinite(GATEWAY_LEVEL_ZERO_ENTRY_TOL_DEG) ||
        (GATEWAY_LEVEL_ZERO_ENTRY_TOL_DEG <= 0.0F) ||
        !isfinite(GATEWAY_LEVEL_ZERO_ENTRY_MAX_RPM) ||
        (GATEWAY_LEVEL_ZERO_ENTRY_MAX_RPM < 0.0F) ||
        (GATEWAY_LEVEL_ZERO_ENTRY_SETTLE_MS == 0U) ||
        !isfinite(GATEWAY_CHASSIS_FF_GAIN) ||
        (GATEWAY_CHASSIS_FF_GAIN < 0.0F) ||
        !isfinite(GATEWAY_CHASSIS_FF_POLARITY) ||
        ((GATEWAY_CHASSIS_FF_POLARITY != 1.0F) &&
         (GATEWAY_CHASSIS_FF_POLARITY != -1.0F)) ||
        !isfinite(GATEWAY_CHASSIS_FF_ANGLE_LIMIT_DEG) ||
        (GATEWAY_CHASSIS_FF_ANGLE_LIMIT_DEG <= 0.0F) ||
        !Algorithm_LevelCurrentControl_Init(
            &s_level_controller, &level_config) ||
        !Algorithm_LevelCurrentControl_CaptureZero(
            &s_level_controller,
            Device_QD4310_AngleRawToRad(GATEWAY_LEVEL_ZERO_RAW)) ||
        !Algorithm_BallPositionControl_Init(
            &s_ball_controller, &ball_config) ||
        !Algorithm_BallStateEstimator_Init(
            &s_ball_estimator, &estimator_config, now_ms) ||
        !Algorithm_ChassisAccelerationFusion_Init(
            &s_chassis_fusion, &fusion_config, now_ms) ||
        !Algorithm_RodAngleMap_Init(
            &s_rod_angle_map,
            s_rod_angle_points,
            GATEWAY_ARRAY_COUNT(s_rod_angle_points),
            GATEWAY_LEVEL_ZERO_RAW) ||
        !Algorithm_RodAngleMap_GetLimits(
            &s_rod_angle_map,
            &s_rod_minimum_angle_deg,
            &s_rod_maximum_angle_deg) ||
        !Algorithm_RodAngleMap_RawToRodDeg(
            &s_rod_angle_map,
            (float)GATEWAY_LEVEL_TRIM_RAW,
            &s_rod_trim_angle_deg) ||
        ((s_rod_trim_angle_deg - GATEWAY_TOTAL_ROD_COMMAND_LIMIT_DEG) <
            s_rod_minimum_angle_deg) ||
        ((s_rod_trim_angle_deg + GATEWAY_TOTAL_ROD_COMMAND_LIMIT_DEG) >
            s_rod_maximum_angle_deg) ||
        !apply_rod_target_deg(s_rod_trim_angle_deg)) {
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
    /* QD4310 can retain its previous enable/current mode across an MCU reset.
     * Issue a best-effort stop immediately; fresh feedback is checked again
     * every Step while the KEY-owned controller remains stopped. */
    (void)Device_QD4310_Disable(&s_motors[0]);

    if (GATEWAY_ESP8266_ENABLE != 0U) {
        esp8266_init(&s_wireless, esp_uart, now_ms);
    }
    s_command_count = 0U;
    s_command_errors = 0U;
    s_next_feedback_ms = now_ms + GATEWAY_FEEDBACK_PERIOD_MS;
    s_tx_sequence = 0U;
    s_last_control_feedback_sequence = 0U;
    s_level_control_fault_count = 0U;
    s_level_calibration_index = 0U;
    s_motor_command_current_a = 0.0F;
    s_selected_chassis_acceleration_mm_s2 = 0;
    s_planned_chassis_acceleration_mm_s2 = 0;
    s_last_chassis_motion_sequence_seen = false;
    s_last_chassis_motion_sequence = 0U;
    s_last_chassis_motion_sample_ms = 0U;
    s_chassis_fusion_trusted = false;
    s_last_ball_sequence_seen = false;
    s_ball_measurement_valid = false;
    s_chassis_acceleration_valid = false;
    s_motor_raw_limit_blocked = false;
    s_level_phase = GATEWAY_LEVEL_PHASE_STOPPED;
    s_zero_entry_stable_seen = false;
    s_zero_entry_stable_since_ms = 0U;
    s_chassis_compensation_angle_deg = 0.0F;
    return true;
}

void APP_Gateway_Step(
    uint32_t now_ms,
    const APP_Gateway_ControlInput *control_input,
    APP_Gateway_Telemetry *primary_telemetry)
{
    uint8_t byte;
    size_t processed = 0U;

    if (GATEWAY_ESP8266_ENABLE != 0U) {
        esp8266_poll(&s_wireless, now_ms);
    }

    if (!update_primary_tracking_target(now_ms, control_input)) {
        ++s_level_control_fault_count;
    }

    /* 下行：解析 TCP 字节流中的 Packet 帧并执行 */
    while ((GATEWAY_ESP8266_ENABLE != 0U) &&
        esp8266_read(&s_wireless, &byte)) {
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
    enforce_primary_motor_disabled_when_stopped(now_ms);
    run_primary_level_control(now_ms);
    if (GATEWAY_ESP8266_ENABLE != 0U) {
        for (size_t index = 0U; index < GATEWAY_MOTOR_COUNT; ++index) {
            send_feedback(index, now_ms);
        }
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
        !apply_rod_target_deg(0.0F) ||
        !Algorithm_LevelCurrentControl_Start(&s_level_controller)) {
        return false;
    }
    s_motor_raw_limit_blocked = false;
    s_level_phase = GATEWAY_LEVEL_PHASE_ZEROING;
    s_zero_entry_stable_seen = false;
    s_last_control_feedback_sequence = feedback.sequence;
    if (Device_QD4310_Enable(motor) &&
        Device_QD4310_SetCurrent(motor, 0.0F)) {
        s_motor_command_current_a = 0.0F;
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
    return (GATEWAY_ESP8266_ENABLE != 0U) &&
        (esp8266_status(&s_wireless) == ESP8266_SESSION_READY);
}
