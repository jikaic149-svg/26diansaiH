#include "device_vision_link.h"

#include <limits.h>
#include <string.h>

#define VISION_LINK_FIELD_COUNT 4U
#define VISION_LINK_POSITION_ABS_MAX_MM INT32_C(125)

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
    size_t length[VISION_LINK_FIELD_COUNT])
{
    size_t field_count = 0U;
    size_t field_start = 0U;

    for (size_t index = 0U; index <= packet->payload_length; ++index) {
        if ((index == packet->payload_length) ||
            (packet->payload[index] == ',')) {
            if ((field_count >= VISION_LINK_FIELD_COUNT) ||
                (index == field_start)) {
                return false;
            }
            start[field_count] = field_start;
            length[field_count] = index - field_start;
            ++field_count;
            field_start = index + 1U;
        }
    }
    return field_count == VISION_LINK_FIELD_COUNT;
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

    if (!split_fields(packet, start, length) ||
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
        !info.supports_buffered_rx ||
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
                ALGORITHM_PACKET_FEED_FRAME_READY) &&
            is_vis(&packet)) {
            consume_vis(link, &packet, now_ms);
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
