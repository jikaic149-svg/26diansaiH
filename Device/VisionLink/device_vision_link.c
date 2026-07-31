#include "device_vision_link.h"

#include <limits.h>
#include <string.h>

#define VISION_LINK_FIELD_COUNT 4U
#define VISION_LINK_TUNING_FIELD_COUNT 3U
#define VISION_LINK_POSITION_ABS_MAX_MM INT32_C(125)
#define VISION_LINK_TARGET_ABS_MAX_0P1MM INT32_C(1250)
#define VISION_LINK_THETA_ABS_MAX_MDEG INT32_C(85000)

#define VISION_FLAG_BALL_SOURCE_MASK UINT8_C(0x03)
#define VISION_FLAG_BALL_YOLO_MEASURED UINT8_C(0x01)
#define VISION_FLAG_AXIS_SOURCE_MASK UINT8_C(0x0C)
#define VISION_FLAG_AXIS_MEASURED UINT8_C(0x04)
#define VISION_FLAG_AXIS_HELD UINT8_C(0x08)
#define VISION_FLAG_POSITION_VALID UINT8_C(0x10)
#define VISION_FLAG_CALIBRATION_VALID UINT8_C(0x20)
#define VISION_FLAG_POSITION_CLAMPED UINT8_C(0x40)

static void increment_saturating(uint32_t *value)
{
    if (*value < UINT32_MAX) {
        ++*value;
    }
}

static bool parse_u32_canonical(
    const uint8_t *text, size_t length, uint32_t *value)
{
    uint32_t result = 0U;

    if ((text == NULL) || (value == NULL) || (length == 0U) ||
        (length > 10U) || ((length > 1U) && (text[0] == '0'))) {
        return false;
    }
    for (size_t index = 0U; index < length; ++index) {
        uint32_t digit;

        if ((text[index] < '0') || (text[index] > '9')) {
            return false;
        }
        digit = (uint32_t)(text[index] - '0');
        if (result > ((UINT32_MAX - digit) / 10U)) {
            return false;
        }
        result = result * 10U + digit;
    }
    *value = result;
    return true;
}

static bool parse_i32_canonical(
    const uint8_t *text, size_t length, int32_t *value)
{
    uint32_t magnitude;

    if ((text == NULL) || (value == NULL) || (length == 0U)) {
        return false;
    }
    if (text[0] != '-') {
        if (!parse_u32_canonical(text, length, &magnitude) ||
            (magnitude > (uint32_t)INT32_MAX)) {
            return false;
        }
        *value = (int32_t)magnitude;
        return true;
    }
    if ((length <= 1U) ||
        !parse_u32_canonical(&text[1], length - 1U, &magnitude) ||
        (magnitude == 0U) ||
        (magnitude > ((uint32_t)INT32_MAX + 1U))) {
        return false;
    }
    *value = (magnitude == ((uint32_t)INT32_MAX + 1U)) ?
        INT32_MIN : -(int32_t)magnitude;
    return true;
}

static bool is_vis(const Algorithm_Packet_View *packet)
{
    return (packet->type_length == 3U) &&
        (memcmp(packet->type, "VIS", 3U) == 0);
}

static bool is_vtp(const Algorithm_Packet_View *packet)
{
    return (packet->type_length == 3U) &&
        (memcmp(packet->type, "VTP", 3U) == 0);
}

static bool is_vbt(const Algorithm_Packet_View *packet)
{
    return (packet->type_length == 3U) &&
        (memcmp(packet->type, "VBT", 3U) == 0);
}

static bool sequence_is_new(uint16_t next, uint16_t previous)
{
    const uint16_t difference = (uint16_t)(next - previous);

    return (difference > 0U) && (difference < UINT16_C(0x8000));
}

static bool flags_are_usable(uint8_t flags)
{
    const uint8_t axis = flags & VISION_FLAG_AXIS_SOURCE_MASK;

    return ((flags & VISION_FLAG_BALL_SOURCE_MASK) ==
            VISION_FLAG_BALL_YOLO_MEASURED) &&
        ((axis == VISION_FLAG_AXIS_MEASURED) ||
         (axis == VISION_FLAG_AXIS_HELD)) &&
        ((flags & VISION_FLAG_POSITION_VALID) != 0U) &&
        ((flags & VISION_FLAG_CALIBRATION_VALID) != 0U) &&
        ((flags & VISION_FLAG_POSITION_CLAMPED) == 0U);
}

static bool split_fields(
    const Algorithm_Packet_View *packet,
    size_t start[VISION_LINK_FIELD_COUNT],
    size_t length[VISION_LINK_FIELD_COUNT],
    size_t expected_count)
{
    size_t field_count = 0U;
    size_t field_start = 0U;

    for (size_t index = 0U; index <= packet->payload_length; ++index) {
        if ((index == packet->payload_length) ||
            (packet->payload[index] == ',')) {
            if ((field_count >= expected_count) ||
                (index == field_start)) {
                return false;
            }
            start[field_count] = field_start;
            length[field_count] = index - field_start;
            ++field_count;
            field_start = index + 1U;
        }
    }
    return field_count == expected_count;
}

static void consume_vis(
    Device_VisionLink *link,
    const Algorithm_Packet_View *packet,
    uint32_t now_ms)
{
    size_t start[VISION_LINK_FIELD_COUNT];
    size_t length[VISION_LINK_FIELD_COUNT];
    uint32_t sampled_at_ms;
    uint32_t processing_age_ms;
    int32_t position_mm;
    uint32_t flags;

    if (!split_fields(
            packet, start, length, VISION_LINK_FIELD_COUNT) ||
        !parse_u32_canonical(
            &packet->payload[start[0]], length[0], &sampled_at_ms) ||
        !parse_u32_canonical(
            &packet->payload[start[1]], length[1], &processing_age_ms) ||
        !parse_i32_canonical(
            &packet->payload[start[2]], length[2], &position_mm) ||
        !parse_u32_canonical(
            &packet->payload[start[3]], length[3], &flags) ||
        (processing_age_ms > UINT16_MAX) ||
        (position_mm < -VISION_LINK_POSITION_ABS_MAX_MM) ||
        (position_mm > VISION_LINK_POSITION_ABS_MAX_MM) ||
        (flags > UINT8_C(0x7F))) {
        increment_saturating(&link->status.vis_payload_error_count);
        return;
    }

    if (link->vis_sequence_seen &&
        ((uint32_t)(now_ms - link->status.last_vis_rx_ms) >=
            link->config.session_timeout_ms)) {
        link->vis_sequence_seen = false;
    }
    if (link->vis_sequence_seen) {
        const uint16_t difference =
            (uint16_t)(packet->sequence - link->last_vis_sequence);

        if (difference == 0U) {
            increment_saturating(
                &link->status.vis_duplicate_sequence_count);
            return;
        }
        if (!sequence_is_new(packet->sequence, link->last_vis_sequence)) {
            increment_saturating(&link->status.vis_old_sequence_count);
            return;
        }
    }

    link->last_vis_sequence = packet->sequence;
    link->vis_sequence_seen = true;
    link->status.last_vis_rx_ms = now_ms;
    link->measurement = (Device_VisionLink_Measurement){
        .sequence = packet->sequence,
        .sampled_at_ms = sampled_at_ms,
        .processing_age_ms = (uint16_t)processing_age_ms,
        .position_mm = (int16_t)position_mm,
        .flags = (uint8_t)flags,
        .received_at_ms = now_ms,
        .usable = flags_are_usable((uint8_t)flags),
        .valid = true,
    };
    increment_saturating(&link->status.vis_rx_count);
    if (link->measurement.usable) {
        increment_saturating(&link->status.vis_usable_count);
    }
}

static void consume_vtp(
    Device_VisionLink *link,
    const Algorithm_Packet_View *packet,
    uint32_t now_ms)
{
    size_t start[VISION_LINK_FIELD_COUNT];
    size_t length[VISION_LINK_FIELD_COUNT];
    uint32_t kp_milli;
    uint32_t ki_milli;
    uint32_t kd_milli;

    if (!split_fields(
            packet, start, length, VISION_LINK_TUNING_FIELD_COUNT) ||
        !parse_u32_canonical(
            &packet->payload[start[0]], length[0], &kp_milli) ||
        !parse_u32_canonical(
            &packet->payload[start[1]], length[1], &ki_milli) ||
        !parse_u32_canonical(
            &packet->payload[start[2]], length[2], &kd_milli) ||
        (kp_milli > UINT16_MAX) || (ki_milli > UINT16_MAX) ||
        (kd_milli > UINT16_MAX)) {
        increment_saturating(&link->status.vtp_payload_error_count);
        return;
    }

    if (link->vtp_sequence_seen &&
        ((uint32_t)(now_ms - link->status.last_vtp_rx_ms) >=
            link->config.session_timeout_ms)) {
        link->vtp_sequence_seen = false;
        link->pid_tuning_pending = false;
    }
    if (link->vtp_sequence_seen) {
        const uint16_t difference =
            (uint16_t)(packet->sequence - link->last_vtp_sequence);

        if (difference == 0U) {
            increment_saturating(
                &link->status.vtp_duplicate_sequence_count);
            return;
        }
        if (!sequence_is_new(packet->sequence, link->last_vtp_sequence)) {
            increment_saturating(&link->status.vtp_old_sequence_count);
            return;
        }
    }

    link->last_vtp_sequence = packet->sequence;
    link->vtp_sequence_seen = true;
    link->status.last_vtp_rx_ms = now_ms;
    link->pending_pid_tuning = (Device_VisionLink_PidTuningCommand){
        .sequence = packet->sequence,
        .kp_milli_per_s2 = (uint16_t)kp_milli,
        .ki_milli_per_s3 = (uint16_t)ki_milli,
        .kd_milli_per_s = (uint16_t)kd_milli,
    };
    link->pid_tuning_pending = true;
    increment_saturating(&link->status.vtp_rx_count);
}

static void consume_vbt(
    Device_VisionLink *link,
    const Algorithm_Packet_View *packet,
    uint32_t now_ms)
{
    int32_t target_0p1mm;

    if (!parse_i32_canonical(
            packet->payload, packet->payload_length, &target_0p1mm) ||
        (target_0p1mm < -VISION_LINK_TARGET_ABS_MAX_0P1MM) ||
        (target_0p1mm > VISION_LINK_TARGET_ABS_MAX_0P1MM)) {
        increment_saturating(&link->status.vbt_payload_error_count);
        return;
    }

    if (link->vbt_sequence_seen &&
        ((uint32_t)(now_ms - link->status.last_vbt_rx_ms) >=
            link->config.session_timeout_ms)) {
        link->vbt_sequence_seen = false;
        link->ball_target_pending = false;
    }
    if (link->vbt_sequence_seen) {
        const uint16_t difference =
            (uint16_t)(packet->sequence - link->last_vbt_sequence);

        if (difference == 0U) {
            increment_saturating(
                &link->status.vbt_duplicate_sequence_count);
            return;
        }
        if (!sequence_is_new(packet->sequence, link->last_vbt_sequence)) {
            increment_saturating(&link->status.vbt_old_sequence_count);
            return;
        }
    }

    link->last_vbt_sequence = packet->sequence;
    link->vbt_sequence_seen = true;
    link->status.last_vbt_rx_ms = now_ms;
    link->pending_ball_target = (Device_VisionLink_BallTargetCommand){
        .sequence = packet->sequence,
        .target_0p1mm = (int16_t)target_0p1mm,
    };
    link->ball_target_pending = true;
    increment_saturating(&link->status.vbt_rx_count);
}

static bool append_byte(
    uint8_t *payload, size_t capacity, size_t *length, uint8_t byte)
{
    if ((payload == NULL) || (length == NULL) || (*length >= capacity)) {
        return false;
    }
    payload[(*length)++] = byte;
    return true;
}

static bool append_u32(
    uint8_t *payload, size_t capacity, size_t *length, uint32_t value)
{
    uint8_t reverse[10];
    size_t count = 0U;

    do {
        reverse[count++] = (uint8_t)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);
    while (count > 0U) {
        if (!append_byte(payload, capacity, length, reverse[--count])) {
            return false;
        }
    }
    return true;
}

static bool append_i32(
    uint8_t *payload, size_t capacity, size_t *length, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        if (!append_byte(payload, capacity, length, '-')) {
            return false;
        }
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    }
    else {
        magnitude = (uint32_t)value;
    }
    return append_u32(payload, capacity, length, magnitude);
}

static bool transmit(
    Device_VisionLink *link,
    const char *type,
    uint16_t *sequence,
    size_t payload_length,
    uint32_t *sent_count,
    uint32_t *drop_count,
    uint32_t *error_count)
{
    size_t frame_length = 0U;
    BSP_UART_Result result;

    if (Algorithm_Packet_Encode(
            type, *sequence,
            link->tx_payload, payload_length,
            link->tx_frame, sizeof(link->tx_frame), &frame_length) !=
        ALGORITHM_PACKET_ENCODE_OK) {
        increment_saturating(error_count);
        return false;
    }
    result = BSP_UART_WriteBuffered(link->uart, link->tx_frame, frame_length);
    if (result == BSP_UART_RESULT_BUSY) {
        increment_saturating(drop_count);
        return true;
    }
    if (result != BSP_UART_RESULT_OK) {
        increment_saturating(error_count);
        return false;
    }
    *sequence = (uint16_t)(*sequence + 1U);
    increment_saturating(sent_count);
    return true;
}

bool Device_VisionLink_Init(
    Device_VisionLink *link,
    const Device_VisionLink_Config *config)
{
    BSP_UART_Info info;
    BSP_UART_Handle uart;

    if (link != NULL) {
        memset(link, 0, sizeof(*link));
    }
    if ((link == NULL) || (config == NULL)) {
        return false;
    }
    uart = BSP_UART_GetHandle(config->transport_index);
    if ((uart == NULL) || (config->maximum_rx_bytes_per_step == 0U) ||
        (config->maximum_rx_bytes_per_step > sizeof(link->rx_chunk)) ||
        (config->session_timeout_ms == 0U) ||
        (config->session_timeout_ms > (uint32_t)INT32_MAX) ||
        (config->maximum_measurement_age_ms == 0U) ||
        (config->maximum_measurement_age_ms >
            config->session_timeout_ms) ||
        !BSP_UART_GetInfo(uart, &info) ||
        (info.baud_rate != DEVICE_VISION_LINK_BAUD_RATE) ||
        !info.supports_buffered_tx || !info.supports_buffered_rx ||
        (info.tx_buffer_capacity < ALGORITHM_PACKET_MAX_FRAME_SIZE) ||
        (info.rx_buffer_capacity < ALGORITHM_PACKET_MAX_FRAME_SIZE) ||
        !Algorithm_Packet_Parser_Init(
            &link->parser, link->parser_storage,
            sizeof(link->parser_storage))) {
        return false;
    }
    link->config = *config;
    link->uart = uart;
    link->initialized = true;
    if (!BSP_UART_GetRxStatus(link->uart, &link->status.uart_rx)) {
        memset(link, 0, sizeof(*link));
        return false;
    }
    return true;
}

bool Device_VisionLink_Step(Device_VisionLink *link, uint32_t now_ms)
{
    size_t received = 0U;
    bool ok = true;

    if ((link == NULL) || !link->initialized) {
        return false;
    }
    if (BSP_UART_ReadAvailable(
            link->uart, link->rx_chunk,
            link->config.maximum_rx_bytes_per_step, &received) !=
        BSP_UART_RESULT_OK) {
        received = 0U;
        ok = false;
    }
    for (size_t index = 0U; index < received; ++index) {
        Algorithm_Packet_View packet;

        if ((Algorithm_Packet_Parser_Feed(
                &link->parser, link->rx_chunk[index], &packet) ==
                ALGORITHM_PACKET_FEED_FRAME_READY)) {
            if (is_vis(&packet)) {
                consume_vis(link, &packet, now_ms);
            }
            else if (is_vtp(&packet)) {
                consume_vtp(link, &packet, now_ms);
            }
            else if (is_vbt(&packet)) {
                consume_vbt(link, &packet, now_ms);
            }
        }
    }
    link->status.packet_parser_error_count =
        Algorithm_Packet_Parser_ErrorCount(&link->parser);
    if (!BSP_UART_GetRxStatus(link->uart, &link->status.uart_rx)) {
        ok = false;
    }
    if (!ok) {
        increment_saturating(&link->status.step_error_count);
    }
    return ok;
}

bool Device_VisionLink_TakePidTuningCommand(
    Device_VisionLink *link,
    Device_VisionLink_PidTuningCommand *command)
{
    if ((link == NULL) || !link->initialized || (command == NULL) ||
        !link->pid_tuning_pending) {
        return false;
    }
    *command = link->pending_pid_tuning;
    link->pid_tuning_pending = false;
    return true;
}

bool Device_VisionLink_TakeBallTargetCommand(
    Device_VisionLink *link,
    Device_VisionLink_BallTargetCommand *command)
{
    if ((link == NULL) || !link->initialized || (command == NULL) ||
        !link->ball_target_pending) {
        return false;
    }
    *command = link->pending_ball_target;
    link->ball_target_pending = false;
    return true;
}

bool Device_VisionLink_SendControlStatus(
    Device_VisionLink *link,
    const Device_VisionLink_ControlStatus *status)
{
    size_t length = 0U;

    if ((link == NULL) || !link->initialized || (status == NULL) ||
        (status->target_mm < -VISION_LINK_POSITION_ABS_MAX_MM) ||
        (status->target_mm > VISION_LINK_POSITION_ABS_MAX_MM) ||
        (status->theta_command_mdeg < -VISION_LINK_THETA_ABS_MAX_MDEG) ||
        (status->theta_command_mdeg > VISION_LINK_THETA_ABS_MAX_MDEG) ||
        ((status->competition_mode !=
          DEVICE_VISION_LINK_COMPETITION_MODE_UNKNOWN) &&
         ((status->competition_mode <
           DEVICE_VISION_LINK_COMPETITION_MODE_MIN) ||
          (status->competition_mode >
           DEVICE_VISION_LINK_COMPETITION_MODE_MAX))) ||
        ((status->status_flags & UINT16_C(0x4000)) != 0U) ||
        ((status->fault_bits & UINT16_C(0x8000)) != 0U)) {
        return false;
    }
#define APPEND_VST_UNSIGNED(value) \
    (append_u32(link->tx_payload, sizeof(link->tx_payload), &length, \
        (uint32_t)(value)))
#define APPEND_VST_SIGNED(value) \
    (append_i32(link->tx_payload, sizeof(link->tx_payload), &length, \
        (int32_t)(value)))
#define APPEND_VST_COMMA() \
    (append_byte(link->tx_payload, sizeof(link->tx_payload), &length, ','))
    if (!APPEND_VST_UNSIGNED(status->last_rx_vis_sequence) ||
        !APPEND_VST_COMMA() ||
        !APPEND_VST_UNSIGNED(status->last_used_vis_sequence) ||
        !APPEND_VST_COMMA() ||
        !APPEND_VST_UNSIGNED(status->measurement_age_ms) ||
        !APPEND_VST_COMMA() ||
        !APPEND_VST_SIGNED(status->estimate_mm) ||
        !APPEND_VST_COMMA() ||
        !APPEND_VST_SIGNED(status->velocity_mm_s) ||
        !APPEND_VST_COMMA() ||
        !APPEND_VST_SIGNED(status->target_mm) ||
        !APPEND_VST_COMMA() ||
        !APPEND_VST_SIGNED(status->theta_command_mdeg) ||
        !APPEND_VST_COMMA() ||
        !APPEND_VST_UNSIGNED(status->motor_target_raw) ||
        !APPEND_VST_COMMA() ||
        !APPEND_VST_UNSIGNED(status->motor_feedback_raw) ||
        !APPEND_VST_COMMA() ||
        !APPEND_VST_SIGNED(status->motor_command_ma) ||
        !APPEND_VST_COMMA() ||
        !APPEND_VST_SIGNED(status->motor_speed_centi_rpm) ||
        !APPEND_VST_COMMA() ||
        !APPEND_VST_SIGNED(status->stiction_kick_mdeg) ||
        !APPEND_VST_COMMA() ||
        !APPEND_VST_UNSIGNED(status->status_flags) ||
        !APPEND_VST_COMMA() ||
        !APPEND_VST_UNSIGNED(status->fault_bits) ||
        !APPEND_VST_COMMA() ||
        !APPEND_VST_UNSIGNED(status->competition_mode)) {
        increment_saturating(&link->status.vst_tx_error_count);
        return false;
    }
#undef APPEND_VST_UNSIGNED
#undef APPEND_VST_SIGNED
#undef APPEND_VST_COMMA
    return transmit(
        link, "VST", &link->vst_tx_sequence, length,
        &link->status.vst_tx_count,
        &link->status.vst_tx_drop_count,
        &link->status.vst_tx_error_count);
}

bool Device_VisionLink_SendPidTuningAck(
    Device_VisionLink *link,
    uint16_t request_sequence,
    bool accepted,
    uint16_t kp_milli_per_s2,
    uint16_t ki_milli_per_s3,
    uint16_t kd_milli_per_s)
{
    size_t length = 0U;

    if ((link == NULL) || !link->initialized ||
        !append_u32(link->tx_payload, sizeof(link->tx_payload), &length,
            request_sequence) ||
        !append_byte(link->tx_payload, sizeof(link->tx_payload), &length, ',') ||
        !append_u32(link->tx_payload, sizeof(link->tx_payload), &length,
            accepted ? 1U : 0U) ||
        !append_byte(link->tx_payload, sizeof(link->tx_payload), &length, ',') ||
        !append_u32(link->tx_payload, sizeof(link->tx_payload), &length,
            kp_milli_per_s2) ||
        !append_byte(link->tx_payload, sizeof(link->tx_payload), &length, ',') ||
        !append_u32(link->tx_payload, sizeof(link->tx_payload), &length,
            ki_milli_per_s3) ||
        !append_byte(link->tx_payload, sizeof(link->tx_payload), &length, ',') ||
        !append_u32(link->tx_payload, sizeof(link->tx_payload), &length,
            kd_milli_per_s)) {
        if ((link != NULL) && link->initialized) {
            increment_saturating(&link->status.vpa_tx_error_count);
        }
        return false;
    }
    return transmit(
        link, "VPA", &link->vpa_tx_sequence, length,
        &link->status.vpa_tx_count,
        &link->status.vpa_tx_drop_count,
        &link->status.vpa_tx_error_count);
}

bool Device_VisionLink_GetMeasurement(
    const Device_VisionLink *link,
    uint32_t now_ms,
    Device_VisionLink_Measurement *measurement)
{
    uint32_t age;
    uint32_t fixed_age;

    if ((link == NULL) || !link->initialized || (measurement == NULL) ||
        !link->measurement.valid || !link->measurement.usable) {
        return false;
    }
    fixed_age = link->config.camera_pipeline_cal_ms +
        link->config.transport_bound_ms;
    if (fixed_age < link->config.camera_pipeline_cal_ms) {
        fixed_age = UINT32_MAX;
    }
    age = now_ms - link->measurement.received_at_ms;
    if ((UINT32_MAX - age) < fixed_age) {
        age = UINT32_MAX;
    }
    else {
        age += fixed_age;
    }
    if ((UINT32_MAX - age) < link->measurement.processing_age_ms) {
        age = UINT32_MAX;
    }
    else {
        age += link->measurement.processing_age_ms;
    }
    if (age >= link->config.maximum_measurement_age_ms) {
        return false;
    }
    *measurement = link->measurement;
    measurement->estimated_age_ms = age;
    return true;
}
