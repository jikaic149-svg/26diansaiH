#include "device_peer_link.h"

#include <limits.h>
#include <string.h>

#define DEVICE_PEER_LINK_MOTOR_ID_MAX UINT32_C(15)
#define DEVICE_PEER_LINK_CURRENT_ABS_MAX_MA INT32_C(10000)
#define DEVICE_PEER_LINK_SPEED_ABS_MAX_CENTI_RPM INT32_C(100000)
#define DEVICE_PEER_LINK_ANGLE_MAX_MRAD UINT32_C(6284)
#define DEVICE_PEER_LINK_TELEMETRY_FIELD_COUNT 7U
#define DEVICE_PEER_LINK_ENCODER_FIELD_COUNT 4U
#define DEVICE_PEER_LINK_F407_NODE_ID 1U
#define DEVICE_PEER_LINK_G3519_NODE_ID 2U

static void increment_saturating(uint32_t *value)
{
    if (*value < UINT32_MAX) {
        ++*value;
    }
}

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static bool node_id_valid(uint8_t node_id)
{
    return (node_id == 1U) || (node_id == 2U);
}

static size_t format_u32(uint8_t *output, size_t capacity, uint32_t value)
{
    uint8_t reverse[10];
    size_t count = 0U;

    do {
        reverse[count++] = (uint8_t)('0' + (value % 10U));
        value /= 10U;
    } while (value > 0U);
    if (count > capacity) {
        return 0U;
    }
    for (size_t index = 0U; index < count; ++index) {
        output[index] = reverse[count - index - 1U];
    }
    return count;
}

static size_t format_i32(uint8_t *output, size_t capacity, int32_t value)
{
    uint32_t magnitude;
    size_t length = 0U;

    if (value < 0) {
        if (capacity == 0U) {
            return 0U;
        }
        output[length++] = '-';
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    }
    else {
        magnitude = (uint32_t)value;
    }
    {
        const size_t digits = format_u32(
            &output[length], capacity - length, magnitude);

        return (digits == 0U) ? 0U : (length + digits);
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
    *value = (magnitude == ((uint32_t)INT32_MAX + 1U))
        ? INT32_MIN : -(int32_t)magnitude;
    return true;
}

static bool telemetry_values_valid(
    const Device_PeerLink_Telemetry *telemetry)
{
    return (telemetry != NULL) &&
        ((uint32_t)telemetry->motor_id <= DEVICE_PEER_LINK_MOTOR_ID_MAX) &&
        (telemetry->current_milliamps >=
            -DEVICE_PEER_LINK_CURRENT_ABS_MAX_MA) &&
        (telemetry->current_milliamps <=
            DEVICE_PEER_LINK_CURRENT_ABS_MAX_MA) &&
        (telemetry->speed_centi_rpm >=
            -DEVICE_PEER_LINK_SPEED_ABS_MAX_CENTI_RPM) &&
        (telemetry->speed_centi_rpm <=
            DEVICE_PEER_LINK_SPEED_ABS_MAX_CENTI_RPM) &&
        (telemetry->angle_milliradians <=
            DEVICE_PEER_LINK_ANGLE_MAX_MRAD);
}

static bool append_separator(
    uint8_t *payload, size_t capacity, size_t *length)
{
    if ((*length) >= capacity) {
        return false;
    }
    payload[(*length)++] = ',';
    return true;
}

static bool append_u32(
    uint8_t *payload, size_t capacity, size_t *length, uint32_t value)
{
    const size_t count = format_u32(
        &payload[*length], capacity - *length, value);

    if (count == 0U) {
        return false;
    }
    *length += count;
    return true;
}

static bool append_i32(
    uint8_t *payload, size_t capacity, size_t *length, int32_t value)
{
    const size_t count = format_i32(
        &payload[*length], capacity - *length, value);

    if (count == 0U) {
        return false;
    }
    *length += count;
    return true;
}

static bool is_heartbeat(const Algorithm_Packet_View *packet)
{
    return (packet->type_length == 2U) &&
           (memcmp(packet->type, "HB", 2U) == 0);
}

static bool is_telemetry(const Algorithm_Packet_View *packet)
{
    return (packet->type_length == 3U) &&
           (memcmp(packet->type, "TEL", 3U) == 0);
}

static bool is_absolute_encoder(const Algorithm_Packet_View *packet)
{
    return (packet->type_length == 3U) &&
           (memcmp(packet->type, "ENC", 3U) == 0);
}

static bool is_motor_disable_request(const Algorithm_Packet_View *packet)
{
    return (packet->type_length == 3U) &&
           (memcmp(packet->type, "DIS", 3U) == 0);
}

static void consume_heartbeat(
    Device_PeerLink *link,
    const Algorithm_Packet_View *packet,
    uint32_t now_ms)
{
    size_t comma = 0U;
    uint32_t node_id;
    uint32_t remote_uptime_ms;

    while ((comma < packet->payload_length) &&
           (packet->payload[comma] != ',')) {
        ++comma;
    }
    if ((comma == 0U) || ((comma + 1U) >= packet->payload_length) ||
        !parse_u32_canonical(packet->payload, comma, &node_id) ||
        !parse_u32_canonical(
            &packet->payload[comma + 1U],
            packet->payload_length - comma - 1U,
            &remote_uptime_ms)) {
        increment_saturating(&link->status.heartbeat_payload_error_count);
        return;
    }
    if (node_id != link->config.remote_node_id) {
        increment_saturating(&link->status.heartbeat_source_error_count);
        return;
    }

    if (link->heartbeat_rx_sequence_seen &&
        (packet->sequence == link->last_heartbeat_rx_sequence)) {
        increment_saturating(&link->status.duplicate_sequence_count);
    }
    else {
        link->last_heartbeat_rx_sequence = packet->sequence;
        link->heartbeat_rx_sequence_seen = true;
    }

    link->status.last_heartbeat_rx_ms = now_ms;
    link->status.last_remote_uptime_ms = remote_uptime_ms;
    link->status.online = true;
    increment_saturating(&link->status.heartbeat_rx_count);
}

static void consume_telemetry(
    Device_PeerLink *link,
    const Algorithm_Packet_View *packet,
    uint32_t now_ms)
{
    size_t start[DEVICE_PEER_LINK_TELEMETRY_FIELD_COUNT];
    size_t length[DEVICE_PEER_LINK_TELEMETRY_FIELD_COUNT];
    size_t field_count = 0U;
    size_t field_start = 0U;
    uint32_t node_id;
    uint32_t motor_id;
    uint32_t enabled;
    int32_t current_milliamps;
    int32_t speed_centi_rpm;
    uint32_t angle_milliradians;
    uint32_t online;

    for (size_t index = 0U; index <= packet->payload_length; ++index) {
        if ((index == packet->payload_length) ||
            (packet->payload[index] == ',')) {
            if ((field_count >= DEVICE_PEER_LINK_TELEMETRY_FIELD_COUNT) ||
                (index == field_start)) {
                increment_saturating(
                    &link->status.telemetry_payload_error_count);
                return;
            }
            start[field_count] = field_start;
            length[field_count] = index - field_start;
            ++field_count;
            field_start = index + 1U;
        }
    }
    if ((field_count != DEVICE_PEER_LINK_TELEMETRY_FIELD_COUNT) ||
        !parse_u32_canonical(
            &packet->payload[start[0]], length[0], &node_id)) {
        increment_saturating(&link->status.telemetry_payload_error_count);
        return;
    }
    if (node_id != link->config.remote_node_id) {
        increment_saturating(&link->status.telemetry_source_error_count);
        return;
    }
    if (!parse_u32_canonical(
            &packet->payload[start[1]], length[1], &motor_id) ||
        !parse_u32_canonical(
            &packet->payload[start[2]], length[2], &enabled) ||
        !parse_i32_canonical(
            &packet->payload[start[3]], length[3], &current_milliamps) ||
        !parse_i32_canonical(
            &packet->payload[start[4]], length[4], &speed_centi_rpm) ||
        !parse_u32_canonical(
            &packet->payload[start[5]], length[5], &angle_milliradians) ||
        !parse_u32_canonical(
            &packet->payload[start[6]], length[6], &online) ||
        (motor_id > DEVICE_PEER_LINK_MOTOR_ID_MAX) ||
        (enabled > 1U) ||
        (current_milliamps < -DEVICE_PEER_LINK_CURRENT_ABS_MAX_MA) ||
        (current_milliamps > DEVICE_PEER_LINK_CURRENT_ABS_MAX_MA) ||
        (speed_centi_rpm < -DEVICE_PEER_LINK_SPEED_ABS_MAX_CENTI_RPM) ||
        (speed_centi_rpm > DEVICE_PEER_LINK_SPEED_ABS_MAX_CENTI_RPM) ||
        (angle_milliradians > DEVICE_PEER_LINK_ANGLE_MAX_MRAD) ||
        (online > 1U)) {
        increment_saturating(&link->status.telemetry_payload_error_count);
        return;
    }

    if (link->telemetry_rx_sequence_seen &&
        (packet->sequence == link->last_telemetry_rx_sequence)) {
        increment_saturating(
            &link->status.telemetry_duplicate_sequence_count);
    }
    else {
        link->last_telemetry_rx_sequence = packet->sequence;
        link->telemetry_rx_sequence_seen = true;
    }
    link->remote_telemetry = (Device_PeerLink_Telemetry){
        .motor_id = (uint8_t)motor_id,
        .enabled = enabled != 0U,
        .current_milliamps = current_milliamps,
        .speed_centi_rpm = speed_centi_rpm,
        .angle_milliradians = angle_milliradians,
        .online = online != 0U,
        .received_at_ms = now_ms,
        .valid = true,
    };
    increment_saturating(&link->status.telemetry_rx_count);
}

static void consume_absolute_encoder(
    Device_PeerLink *link,
    const Algorithm_Packet_View *packet,
    uint32_t now_ms)
{
    size_t start[DEVICE_PEER_LINK_ENCODER_FIELD_COUNT];
    size_t length[DEVICE_PEER_LINK_ENCODER_FIELD_COUNT];
    size_t field_count = 0U;
    size_t field_start = 0U;
    uint32_t node_id;
    uint32_t motor_id;
    uint32_t angle_raw;
    uint32_t online;

    for (size_t index = 0U; index <= packet->payload_length; ++index) {
        if ((index == packet->payload_length) ||
            (packet->payload[index] == ',')) {
            if ((field_count >= DEVICE_PEER_LINK_ENCODER_FIELD_COUNT) ||
                (index == field_start)) {
                increment_saturating(
                    &link->status.encoder_payload_error_count);
                return;
            }
            start[field_count] = field_start;
            length[field_count] = index - field_start;
            ++field_count;
            field_start = index + 1U;
        }
    }
    if ((field_count != DEVICE_PEER_LINK_ENCODER_FIELD_COUNT) ||
        !parse_u32_canonical(
            &packet->payload[start[0]], length[0], &node_id)) {
        increment_saturating(&link->status.encoder_payload_error_count);
        return;
    }
    if ((link->config.local_node_id != DEVICE_PEER_LINK_G3519_NODE_ID) ||
        (node_id != DEVICE_PEER_LINK_F407_NODE_ID) ||
        (node_id != link->config.remote_node_id)) {
        increment_saturating(&link->status.encoder_source_error_count);
        return;
    }
    if (!parse_u32_canonical(
            &packet->payload[start[1]], length[1], &motor_id) ||
        !parse_u32_canonical(
            &packet->payload[start[2]], length[2], &angle_raw) ||
        !parse_u32_canonical(
            &packet->payload[start[3]], length[3], &online) ||
        (motor_id > DEVICE_PEER_LINK_MOTOR_ID_MAX) ||
        (angle_raw > UINT16_MAX) ||
        (online > 1U)) {
        increment_saturating(&link->status.encoder_payload_error_count);
        return;
    }

    if (link->encoder_rx_sequence_seen &&
        (packet->sequence == link->last_encoder_rx_sequence)) {
        increment_saturating(
            &link->status.encoder_duplicate_sequence_count);
    }
    else {
        link->last_encoder_rx_sequence = packet->sequence;
        link->encoder_rx_sequence_seen = true;
    }
    link->remote_encoder = (Device_PeerLink_AbsoluteEncoder){
        .motor_id = (uint8_t)motor_id,
        .angle_raw = (uint16_t)angle_raw,
        .online = online != 0U,
        .received_at_ms = now_ms,
        .valid = true,
    };
    increment_saturating(&link->status.encoder_rx_count);
}

static void consume_motor_disable_request(
    Device_PeerLink *link,
    const Algorithm_Packet_View *packet)
{
    uint32_t source_node_id;

    if (!parse_u32_canonical(
            packet->payload, packet->payload_length, &source_node_id)) {
        increment_saturating(
            &link->status.motor_disable_payload_error_count);
        return;
    }
    if ((link->config.local_node_id != DEVICE_PEER_LINK_F407_NODE_ID) ||
        (source_node_id != DEVICE_PEER_LINK_G3519_NODE_ID) ||
        (source_node_id != link->config.remote_node_id)) {
        increment_saturating(
            &link->status.motor_disable_source_error_count);
        return;
    }

    if (link->motor_disable_rx_sequence_seen &&
        (packet->sequence == link->last_motor_disable_rx_sequence)) {
        increment_saturating(
            &link->status.motor_disable_duplicate_sequence_count);
    }
    else {
        link->last_motor_disable_rx_sequence = packet->sequence;
        link->motor_disable_rx_sequence_seen = true;
    }
    link->motor_disable_pending = true;
    increment_saturating(&link->status.motor_disable_rx_count);
}

static bool send_heartbeat(Device_PeerLink *link, uint32_t now_ms)
{
    size_t payload_length;
    size_t length;
    size_t frame_length = 0U;
    BSP_UART_Result result;

    payload_length = format_u32(
        link->tx_payload, sizeof(link->tx_payload),
        link->config.local_node_id);
    if ((payload_length == 0U) ||
        (payload_length >= sizeof(link->tx_payload))) {
        increment_saturating(&link->status.heartbeat_tx_error_count);
        return false;
    }
    link->tx_payload[payload_length++] = ',';
    length = format_u32(
        &link->tx_payload[payload_length],
        sizeof(link->tx_payload) - payload_length,
        now_ms);
    if (length == 0U) {
        increment_saturating(&link->status.heartbeat_tx_error_count);
        return false;
    }
    payload_length += length;

    if (Algorithm_Packet_Encode(
            "HB", link->heartbeat_tx_sequence,
            link->tx_payload, payload_length,
            link->tx_frame, sizeof(link->tx_frame), &frame_length) !=
        ALGORITHM_PACKET_ENCODE_OK) {
        increment_saturating(&link->status.heartbeat_tx_error_count);
        return false;
    }

    result = BSP_UART_WriteBuffered(
        link->uart, link->tx_frame, frame_length);
    if (result == BSP_UART_RESULT_BUSY) {
        increment_saturating(&link->status.heartbeat_tx_drop_count);
        return true;
    }
    if (result != BSP_UART_RESULT_OK) {
        increment_saturating(&link->status.heartbeat_tx_error_count);
        return false;
    }

    link->heartbeat_tx_sequence =
        (uint16_t)(link->heartbeat_tx_sequence + 1U);
    increment_saturating(&link->status.heartbeat_tx_count);
    return true;
}

bool Device_PeerLink_Init(
    Device_PeerLink *link,
    const Device_PeerLink_Config *config,
    uint32_t now_ms)
{
    BSP_UART_Info info;
    BSP_UART_Handle uart;

    if ((link == NULL) || (config == NULL)) {
        return false;
    }
    memset(link, 0, sizeof(*link));
    uart = BSP_UART_GetHandle(config->transport_index);
    if ((uart == NULL) || !node_id_valid(config->local_node_id) ||
        !node_id_valid(config->remote_node_id) ||
        (config->local_node_id == config->remote_node_id) ||
        (config->heartbeat_period_ms == 0U) ||
        (config->heartbeat_period_ms > (uint32_t)INT32_MAX) ||
        (config->offline_timeout_ms < config->heartbeat_period_ms) ||
        (config->offline_timeout_ms > (uint32_t)INT32_MAX) ||
        (config->maximum_rx_bytes_per_step == 0U) ||
        (config->maximum_rx_bytes_per_step > sizeof(link->rx_chunk)) ||
        !BSP_UART_GetInfo(uart, &info) ||
        (info.baud_rate != DEVICE_PEER_LINK_BAUD_RATE) ||
        !info.supports_tx_dma || !info.supports_rx_dma ||
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
    link->next_heartbeat_tx_ms = now_ms;
    link->initialized = true;
    if (!BSP_UART_GetRxStatus(link->uart, &link->status.uart_rx)) {
        memset(link, 0, sizeof(*link));
        return false;
    }
    return true;
}

bool Device_PeerLink_Step(Device_PeerLink *link, uint32_t now_ms)
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

        if (Algorithm_Packet_Parser_Feed(
                &link->parser, link->rx_chunk[index], &packet) ==
            ALGORITHM_PACKET_FEED_FRAME_READY) {
            if (is_heartbeat(&packet)) {
                consume_heartbeat(link, &packet, now_ms);
            }
            else if (is_telemetry(&packet)) {
                consume_telemetry(link, &packet, now_ms);
            }
            else if (is_absolute_encoder(&packet)) {
                consume_absolute_encoder(link, &packet, now_ms);
            }
            else if (is_motor_disable_request(&packet)) {
                consume_motor_disable_request(link, &packet);
            }
        }
    }
    link->status.packet_parser_error_count =
        Algorithm_Packet_Parser_ErrorCount(&link->parser);

    if (link->status.online &&
        ((uint32_t)(now_ms - link->status.last_heartbeat_rx_ms) >=
         link->config.offline_timeout_ms)) {
        link->status.online = false;
    }

    if (time_reached(now_ms, link->next_heartbeat_tx_ms)) {
        link->next_heartbeat_tx_ms =
            now_ms + link->config.heartbeat_period_ms;
        if (!send_heartbeat(link, now_ms)) {
            ok = false;
        }
    }

    if (!BSP_UART_GetRxStatus(
            link->uart, &link->status.uart_rx)) {
        ok = false;
    }
    if (!ok) {
        increment_saturating(&link->status.step_error_count);
    }
    return ok;
}

bool Device_PeerLink_SendTelemetry(
    Device_PeerLink *link,
    const Device_PeerLink_Telemetry *telemetry)
{
    size_t payload_length = 0U;
    size_t frame_length = 0U;
    BSP_UART_Result result;

    if ((link == NULL) || !link->initialized ||
        !telemetry_values_valid(telemetry)) {
        if ((link != NULL) && link->initialized) {
            increment_saturating(&link->status.telemetry_tx_error_count);
        }
        return false;
    }
    if (!append_u32(link->tx_payload, sizeof(link->tx_payload),
            &payload_length, link->config.local_node_id) ||
        !append_separator(link->tx_payload, sizeof(link->tx_payload),
            &payload_length) ||
        !append_u32(link->tx_payload, sizeof(link->tx_payload),
            &payload_length, telemetry->motor_id) ||
        !append_separator(link->tx_payload, sizeof(link->tx_payload),
            &payload_length) ||
        !append_u32(link->tx_payload, sizeof(link->tx_payload),
            &payload_length, telemetry->enabled ? 1U : 0U) ||
        !append_separator(link->tx_payload, sizeof(link->tx_payload),
            &payload_length) ||
        !append_i32(link->tx_payload, sizeof(link->tx_payload),
            &payload_length, telemetry->current_milliamps) ||
        !append_separator(link->tx_payload, sizeof(link->tx_payload),
            &payload_length) ||
        !append_i32(link->tx_payload, sizeof(link->tx_payload),
            &payload_length, telemetry->speed_centi_rpm) ||
        !append_separator(link->tx_payload, sizeof(link->tx_payload),
            &payload_length) ||
        !append_u32(link->tx_payload, sizeof(link->tx_payload),
            &payload_length, telemetry->angle_milliradians) ||
        !append_separator(link->tx_payload, sizeof(link->tx_payload),
            &payload_length) ||
        !append_u32(link->tx_payload, sizeof(link->tx_payload),
            &payload_length, telemetry->online ? 1U : 0U) ||
        (Algorithm_Packet_Encode(
            "TEL", link->telemetry_tx_sequence,
            link->tx_payload, payload_length,
            link->tx_frame, sizeof(link->tx_frame), &frame_length) !=
         ALGORITHM_PACKET_ENCODE_OK)) {
        increment_saturating(&link->status.telemetry_tx_error_count);
        return false;
    }

    result = BSP_UART_WriteBuffered(
        link->uart, link->tx_frame, frame_length);
    if (result == BSP_UART_RESULT_BUSY) {
        increment_saturating(&link->status.telemetry_tx_drop_count);
        return true;
    }
    if (result != BSP_UART_RESULT_OK) {
        increment_saturating(&link->status.telemetry_tx_error_count);
        return false;
    }
    link->telemetry_tx_sequence =
        (uint16_t)(link->telemetry_tx_sequence + 1U);
    increment_saturating(&link->status.telemetry_tx_count);
    return true;
}

bool Device_PeerLink_SendMotorDisableRequest(Device_PeerLink *link)
{
    size_t payload_length;
    size_t frame_length = 0U;
    BSP_UART_Result result;

    if ((link == NULL) || !link->initialized ||
        (link->config.local_node_id != DEVICE_PEER_LINK_G3519_NODE_ID) ||
        (link->config.remote_node_id != DEVICE_PEER_LINK_F407_NODE_ID)) {
        if ((link != NULL) && link->initialized) {
            increment_saturating(
                &link->status.motor_disable_tx_error_count);
        }
        return false;
    }
    payload_length = format_u32(
        link->tx_payload, sizeof(link->tx_payload),
        link->config.local_node_id);
    if ((payload_length == 0U) ||
        (Algorithm_Packet_Encode(
            "DIS", link->motor_disable_tx_sequence,
            link->tx_payload, payload_length,
            link->tx_frame, sizeof(link->tx_frame), &frame_length) !=
         ALGORITHM_PACKET_ENCODE_OK)) {
        increment_saturating(
            &link->status.motor_disable_tx_error_count);
        return false;
    }

    result = BSP_UART_WriteBuffered(
        link->uart, link->tx_frame, frame_length);
    if (result == BSP_UART_RESULT_BUSY) {
        increment_saturating(
            &link->status.motor_disable_tx_drop_count);
        return true;
    }
    if (result != BSP_UART_RESULT_OK) {
        increment_saturating(
            &link->status.motor_disable_tx_error_count);
        return false;
    }
    link->motor_disable_tx_sequence =
        (uint16_t)(link->motor_disable_tx_sequence + 1U);
    increment_saturating(&link->status.motor_disable_tx_count);
    return true;
}

bool Device_PeerLink_TakeMotorDisableRequest(Device_PeerLink *link)
{
    if ((link == NULL) || !link->initialized ||
        !link->motor_disable_pending) {
        return false;
    }
    link->motor_disable_pending = false;
    return true;
}

bool Device_PeerLink_GetRemoteAbsoluteEncoder(
    const Device_PeerLink *link,
    uint32_t now_ms,
    uint32_t maximum_age_ms,
    Device_PeerLink_AbsoluteEncoder *encoder)
{
    if ((link == NULL) || !link->initialized || (encoder == NULL) ||
        !link->remote_encoder.valid ||
        ((uint32_t)(now_ms - link->remote_encoder.received_at_ms) >
            maximum_age_ms)) {
        return false;
    }
    *encoder = link->remote_encoder;
    return true;
}

bool Device_PeerLink_GetRemoteTelemetry(
    const Device_PeerLink *link,
    uint32_t now_ms,
    uint32_t maximum_age_ms,
    Device_PeerLink_Telemetry *telemetry)
{
    if ((link == NULL) || !link->initialized || (telemetry == NULL) ||
        !link->remote_telemetry.valid ||
        ((uint32_t)(now_ms - link->remote_telemetry.received_at_ms) >
            maximum_age_ms)) {
        return false;
    }
    *telemetry = link->remote_telemetry;
    return true;
}
