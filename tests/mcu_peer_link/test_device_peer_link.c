#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "device_peer_link.h"

#define FAKE_RX_CAPACITY 512U

struct BSP_UART_Resource {
    uint8_t token;
};

static struct BSP_UART_Resource s_uart_resource;
static BSP_UART_Info s_uart_info;
static BSP_UART_RxStatus s_uart_status;
static uint8_t s_rx_bytes[FAKE_RX_CAPACITY];
static size_t s_rx_length;
static uint8_t s_last_tx[ALGORITHM_PACKET_MAX_FRAME_SIZE];
static size_t s_last_tx_length;
static uint32_t s_tx_call_count;
static size_t s_last_read_capacity;
static bool s_tx_busy;

static void fake_reset(void)
{
    memset(&s_uart_status, 0, sizeof(s_uart_status));
    memset(s_rx_bytes, 0, sizeof(s_rx_bytes));
    memset(s_last_tx, 0, sizeof(s_last_tx));
    s_uart_info = (BSP_UART_Info){
        .baud_rate = DEVICE_PEER_LINK_BAUD_RATE,
        .supports_tx_dma = true,
        .supports_rx_dma = true,
        .supports_buffered_tx = true,
        .supports_buffered_rx = true,
        .tx_buffer_capacity = 511U,
        .rx_buffer_capacity = 511U,
    };
    s_uart_status.buffer_capacity = 511U;
    s_rx_length = 0U;
    s_last_tx_length = 0U;
    s_tx_call_count = 0U;
    s_last_read_capacity = 0U;
    s_tx_busy = false;
}

static void fake_queue_rx(const uint8_t *bytes, size_t length)
{
    assert(bytes != NULL);
    assert(length <= (sizeof(s_rx_bytes) - s_rx_length));
    memcpy(&s_rx_bytes[s_rx_length], bytes, length);
    s_rx_length += length;
}

size_t BSP_UART_Count(void)
{
    return 1U;
}

BSP_UART_Handle BSP_UART_GetHandle(size_t index)
{
    return (index == 0U) ? &s_uart_resource : NULL;
}

bool BSP_UART_GetInfo(BSP_UART_Handle uart, BSP_UART_Info *info)
{
    if ((uart != &s_uart_resource) || (info == NULL)) {
        return false;
    }
    *info = s_uart_info;
    return true;
}

BSP_UART_Result BSP_UART_WriteBuffered(
    BSP_UART_Handle uart, const uint8_t *data, size_t length)
{
    if ((uart != &s_uart_resource) || (data == NULL) ||
        (length > sizeof(s_last_tx))) {
        return BSP_UART_RESULT_INVALID_ARGUMENT;
    }
    ++s_tx_call_count;
    if (s_tx_busy) {
        return BSP_UART_RESULT_BUSY;
    }
    memcpy(s_last_tx, data, length);
    s_last_tx_length = length;
    return BSP_UART_RESULT_OK;
}

BSP_UART_Result BSP_UART_ReadAvailable(
    BSP_UART_Handle uart,
    uint8_t *data,
    size_t capacity,
    size_t *transferred)
{
    size_t count;

    if ((uart != &s_uart_resource) || (transferred == NULL) ||
        ((data == NULL) && (capacity != 0U))) {
        return BSP_UART_RESULT_INVALID_ARGUMENT;
    }
    s_last_read_capacity = capacity;
    count = (s_rx_length < capacity) ? s_rx_length : capacity;
    if (count > 0U) {
        memcpy(data, s_rx_bytes, count);
        memmove(s_rx_bytes, &s_rx_bytes[count], s_rx_length - count);
        s_rx_length -= count;
    }
    *transferred = count;
    return BSP_UART_RESULT_OK;
}

bool BSP_UART_GetRxStatus(
    BSP_UART_Handle uart, BSP_UART_RxStatus *status)
{
    if ((uart != &s_uart_resource) || (status == NULL)) {
        return false;
    }
    s_uart_status.buffered_bytes = s_rx_length;
    *status = s_uart_status;
    return true;
}

static Device_PeerLink_Config make_config(void)
{
    return (Device_PeerLink_Config){
        .transport_index = 0U,
        .local_node_id = 1U,
        .remote_node_id = 2U,
        .heartbeat_period_ms = 500U,
        .offline_timeout_ms = 1500U,
        .maximum_rx_bytes_per_step = 128U,
    };
}

static size_t encode_heartbeat(
    uint8_t node_id,
    uint32_t uptime_ms,
    uint16_t sequence,
    uint8_t *frame)
{
    char payload[16];
    int payload_length = snprintf(
        payload, sizeof(payload), "%u,%lu",
        (unsigned)node_id, (unsigned long)uptime_ms);
    size_t frame_length = 0U;

    assert(payload_length > 0);
    assert((size_t)payload_length < sizeof(payload));
    assert(Algorithm_Packet_Encode(
        "HB", sequence, (const uint8_t *)payload, (size_t)payload_length,
        frame, ALGORITHM_PACKET_MAX_FRAME_SIZE, &frame_length) ==
        ALGORITHM_PACKET_ENCODE_OK);
    return frame_length;
}

static size_t encode_telemetry(
    uint8_t node_id,
    uint8_t motor_id,
    unsigned enabled,
    int32_t current_milliamps,
    int32_t speed_centi_rpm,
    uint32_t angle_milliradians,
    unsigned online,
    uint16_t sequence,
    uint8_t *frame)
{
    char payload[48];
    int payload_length = snprintf(
        payload, sizeof(payload), "%u,%u,%u,%ld,%ld,%lu,%u",
        (unsigned)node_id, (unsigned)motor_id, enabled,
        (long)current_milliamps, (long)speed_centi_rpm,
        (unsigned long)angle_milliradians, online);
    size_t frame_length = 0U;

    assert(payload_length > 0);
    assert((size_t)payload_length < sizeof(payload));
    assert(Algorithm_Packet_Encode(
        "TEL", sequence, (const uint8_t *)payload, (size_t)payload_length,
        frame, ALGORITHM_PACKET_MAX_FRAME_SIZE, &frame_length) ==
        ALGORITHM_PACKET_ENCODE_OK);
    return frame_length;
}

static size_t encode_motor_disable_request(
    uint8_t node_id,
    uint16_t sequence,
    uint8_t *frame)
{
    char payload[4];
    int payload_length = snprintf(
        payload, sizeof(payload), "%u", (unsigned)node_id);
    size_t frame_length = 0U;

    assert(payload_length > 0);
    assert((size_t)payload_length < sizeof(payload));
    assert(Algorithm_Packet_Encode(
        "DIS", sequence, (const uint8_t *)payload, (size_t)payload_length,
        frame, ALGORITHM_PACKET_MAX_FRAME_SIZE, &frame_length) ==
        ALGORITHM_PACKET_ENCODE_OK);
    return frame_length;
}

static size_t encode_absolute_encoder(
    uint8_t node_id,
    uint8_t motor_id,
    uint32_t angle_raw,
    unsigned online,
    uint16_t sequence,
    uint8_t *frame)
{
    char payload[32];
    int payload_length = snprintf(
        payload, sizeof(payload), "%u,%u,%lu,%u",
        (unsigned)node_id, (unsigned)motor_id,
        (unsigned long)angle_raw, online);
    size_t frame_length = 0U;

    assert(payload_length > 0);
    assert((size_t)payload_length < sizeof(payload));
    assert(Algorithm_Packet_Encode(
        "ENC", sequence, (const uint8_t *)payload, (size_t)payload_length,
        frame, ALGORITHM_PACKET_MAX_FRAME_SIZE, &frame_length) ==
        ALGORITHM_PACKET_ENCODE_OK);
    return frame_length;
}

static void test_dma_required_and_first_frame(void)
{
    static const char expected[] = "@1|HB|0|3|1,0|CEC5\n";
    Device_PeerLink link;
    Device_PeerLink_Config config = make_config();

    fake_reset();
    s_uart_info.supports_rx_dma = false;
    assert(!Device_PeerLink_Init(&link, &config, 0U));
    s_uart_info.supports_rx_dma = true;
    assert(Device_PeerLink_Init(&link, &config, 0U));
    assert(Device_PeerLink_Step(&link, 0U));
    assert(s_last_tx_length == (sizeof(expected) - 1U));
    assert(memcmp(s_last_tx, expected, sizeof(expected) - 1U) == 0);
    assert(link.status.heartbeat_tx_count == 1U);
}

static void test_receive_duplicate_errors_and_timeout(void)
{
    Device_PeerLink link;
    Device_PeerLink_Config config = make_config();
    uint8_t frame[ALGORITHM_PACKET_MAX_FRAME_SIZE];
    size_t length;

    fake_reset();
    assert(Device_PeerLink_Init(&link, &config, 0U));
    length = encode_heartbeat(2U, 10U, 7U, frame);
    fake_queue_rx(frame, length);
    assert(Device_PeerLink_Step(&link, 10U));
    assert(link.status.online);
    assert(link.status.heartbeat_rx_count == 1U);

    fake_queue_rx(frame, length);
    assert(Device_PeerLink_Step(&link, 20U));
    assert(link.status.duplicate_sequence_count == 1U);
    assert(link.status.heartbeat_rx_count == 2U);

    length = encode_heartbeat(1U, 30U, 8U, frame);
    fake_queue_rx(frame, length);
    assert(Device_PeerLink_Step(&link, 30U));
    assert(link.status.heartbeat_source_error_count == 1U);
    assert(link.status.last_heartbeat_rx_ms == 20U);

    length = encode_heartbeat(2U, 40U, 9U, frame);
    frame[length - 2U] = (frame[length - 2U] == '0') ? '1' : '0';
    fake_queue_rx(frame, length);
    assert(Device_PeerLink_Step(&link, 40U));
    assert(link.status.packet_parser_error_count == 1U);
    assert(link.status.last_heartbeat_rx_ms == 20U);

    assert(Device_PeerLink_Step(&link, 1519U));
    assert(link.status.online);
    assert(Device_PeerLink_Step(&link, 1520U));
    assert(!link.status.online);
}

static void test_queue_full_does_not_advance_sequence(void)
{
    Device_PeerLink link;
    Device_PeerLink_Config config = make_config();
    Algorithm_Packet_Parser parser;
    Algorithm_Packet_View packet;
    uint8_t storage[ALGORITHM_PACKET_PARSER_STORAGE_SIZE];

    fake_reset();
    s_tx_busy = true;
    assert(Device_PeerLink_Init(&link, &config, 0U));
    assert(Device_PeerLink_Step(&link, 0U));
    assert(link.status.heartbeat_tx_drop_count == 1U);
    assert(link.status.heartbeat_tx_count == 0U);
    assert(s_tx_call_count == 1U);
    assert(Device_PeerLink_Step(&link, 499U));
    assert(s_tx_call_count == 1U);

    s_tx_busy = false;
    assert(Device_PeerLink_Step(&link, 500U));
    assert(link.status.heartbeat_tx_count == 1U);
    assert(Algorithm_Packet_Parser_Init(&parser, storage, sizeof(storage)));
    for (size_t index = 0U; index < s_last_tx_length; ++index) {
        Algorithm_Packet_FeedResult result = Algorithm_Packet_Parser_Feed(
            &parser, s_last_tx[index], &packet);

        if (index + 1U == s_last_tx_length) {
            assert(result == ALGORITHM_PACKET_FEED_FRAME_READY);
        }
    }
    assert(packet.sequence == 0U);
}

static void test_wrap_and_per_step_limit(void)
{
    Device_PeerLink link;
    Device_PeerLink_Config config = make_config();
    uint8_t frame[ALGORITHM_PACKET_MAX_FRAME_SIZE];
    uint8_t noise[200];
    size_t length;
    uint32_t received_at = UINT32_MAX - 2U;

    fake_reset();
    assert(Device_PeerLink_Init(&link, &config, UINT32_MAX - 4U));
    assert(Device_PeerLink_Step(&link, UINT32_MAX - 4U));
    length = encode_heartbeat(2U, UINT32_MAX, 0U, frame);
    fake_queue_rx(frame, length);
    assert(Device_PeerLink_Step(&link, received_at));
    assert(link.status.online);
    assert(Device_PeerLink_Step(&link, received_at + 1499U));
    assert(link.status.online);
    assert(Device_PeerLink_Step(&link, received_at + 1500U));
    assert(!link.status.online);

    memset(noise, '@', sizeof(noise));
    fake_queue_rx(noise, sizeof(noise));
    assert(Device_PeerLink_Step(&link, received_at + 1510U));
    assert(s_last_read_capacity == 128U);
    assert(s_rx_length == (sizeof(noise) - 128U));
}

static void test_telemetry_send_and_busy_sequence(void)
{
    Device_PeerLink link;
    Device_PeerLink_Config config = make_config();
    const Device_PeerLink_Telemetry telemetry = {
        .motor_id = 1U,
        .enabled = true,
        .current_milliamps = -250,
        .speed_centi_rpm = 12345,
        .angle_milliradians = 3142U,
        .online = true,
    };
    Algorithm_Packet_Parser parser;
    Algorithm_Packet_View packet;
    uint8_t storage[ALGORITHM_PACKET_PARSER_STORAGE_SIZE];

    fake_reset();
    assert(Device_PeerLink_Init(&link, &config, 0U));
    assert(Device_PeerLink_SendTelemetry(&link, &telemetry));
    assert(link.status.telemetry_tx_count == 1U);
    assert(Algorithm_Packet_Parser_Init(&parser, storage, sizeof(storage)));
    for (size_t index = 0U; index < s_last_tx_length; ++index) {
        const Algorithm_Packet_FeedResult result =
            Algorithm_Packet_Parser_Feed(
                &parser, s_last_tx[index], &packet);

        if (index + 1U == s_last_tx_length) {
            assert(result == ALGORITHM_PACKET_FEED_FRAME_READY);
        }
    }
    assert(packet.type_length == 3U);
    assert(memcmp(packet.type, "TEL", 3U) == 0);
    assert(packet.sequence == 0U);
    assert(packet.payload_length ==
        sizeof("1,1,1,-250,12345,3142,1") - 1U);
    assert(memcmp(packet.payload, "1,1,1,-250,12345,3142,1",
        packet.payload_length) == 0);

    s_tx_busy = true;
    assert(Device_PeerLink_SendTelemetry(&link, &telemetry));
    assert(link.status.telemetry_tx_drop_count == 1U);
    assert(link.status.telemetry_tx_count == 1U);
    s_tx_busy = false;
    assert(Device_PeerLink_SendTelemetry(&link, &telemetry));
    assert(link.status.telemetry_tx_count == 2U);
    Algorithm_Packet_Parser_Reset(&parser);
    for (size_t index = 0U; index < s_last_tx_length; ++index) {
        (void)Algorithm_Packet_Parser_Feed(
            &parser, s_last_tx[index], &packet);
    }
    assert(packet.sequence == 1U);

    assert(!Device_PeerLink_SendTelemetry(
        &link, &(Device_PeerLink_Telemetry){
            .motor_id = 1U,
            .angle_milliradians = 6285U,
        }));
    assert(link.status.telemetry_tx_error_count == 1U);
}

static void test_telemetry_receive_validation_and_age(void)
{
    Device_PeerLink link;
    Device_PeerLink_Config config = make_config();
    Device_PeerLink_Telemetry telemetry;
    uint8_t frame[ALGORITHM_PACKET_MAX_FRAME_SIZE];
    size_t length;

    fake_reset();
    assert(Device_PeerLink_Init(&link, &config, 0U));
    length = encode_telemetry(
        2U, 1U, 1U, -250, 12345, 3142U, 1U, 9U, frame);
    fake_queue_rx(frame, length);
    assert(Device_PeerLink_Step(&link, 100U));
    assert(link.status.telemetry_rx_count == 1U);
    assert(Device_PeerLink_GetRemoteTelemetry(
        &link, 300U, 200U, &telemetry));
    assert(telemetry.motor_id == 1U);
    assert(telemetry.enabled);
    assert(telemetry.current_milliamps == -250);
    assert(telemetry.speed_centi_rpm == 12345);
    assert(telemetry.angle_milliradians == 3142U);
    assert(telemetry.online);
    assert(!Device_PeerLink_GetRemoteTelemetry(
        &link, 301U, 200U, &telemetry));

    fake_queue_rx(frame, length);
    assert(Device_PeerLink_Step(&link, 110U));
    assert(link.status.telemetry_duplicate_sequence_count == 1U);

    length = encode_telemetry(
        1U, 1U, 1U, 0, 0, 0U, 1U, 10U, frame);
    fake_queue_rx(frame, length);
    assert(Device_PeerLink_Step(&link, 120U));
    assert(link.status.telemetry_source_error_count == 1U);

    {
        static const uint8_t invalid_payload[] = "2,1,1,-0,0,0,1";
        size_t frame_length = 0U;

        assert(Algorithm_Packet_Encode(
            "TEL", 11U, invalid_payload, sizeof(invalid_payload) - 1U,
            frame, sizeof(frame), &frame_length) ==
            ALGORITHM_PACKET_ENCODE_OK);
        fake_queue_rx(frame, frame_length);
        assert(Device_PeerLink_Step(&link, 130U));
        assert(link.status.telemetry_payload_error_count == 1U);
    }
}

static void test_motor_disable_request_send_receive_and_busy(void)
{
    Device_PeerLink receiver;
    Device_PeerLink sender;
    Device_PeerLink_Config receiver_config = make_config();
    Device_PeerLink_Config sender_config = make_config();
    Algorithm_Packet_Parser parser;
    Algorithm_Packet_View packet;
    uint8_t storage[ALGORITHM_PACKET_PARSER_STORAGE_SIZE];
    uint8_t frame[ALGORITHM_PACKET_MAX_FRAME_SIZE];
    size_t length;

    fake_reset();
    assert(Device_PeerLink_Init(&receiver, &receiver_config, 0U));
    length = encode_motor_disable_request(2U, 7U, frame);
    fake_queue_rx(frame, length);
    assert(Device_PeerLink_Step(&receiver, 10U));
    assert(receiver.status.motor_disable_rx_count == 1U);
    assert(Device_PeerLink_TakeMotorDisableRequest(&receiver));
    assert(!Device_PeerLink_TakeMotorDisableRequest(&receiver));

    fake_queue_rx(frame, length);
    assert(Device_PeerLink_Step(&receiver, 20U));
    assert(receiver.status.motor_disable_rx_count == 2U);
    assert(receiver.status.motor_disable_duplicate_sequence_count == 1U);
    assert(Device_PeerLink_TakeMotorDisableRequest(&receiver));

    length = encode_motor_disable_request(1U, 8U, frame);
    fake_queue_rx(frame, length);
    assert(Device_PeerLink_Step(&receiver, 30U));
    assert(receiver.status.motor_disable_source_error_count == 1U);
    assert(!Device_PeerLink_TakeMotorDisableRequest(&receiver));

    {
        static const uint8_t invalid_payload[] = "02";
        size_t frame_length = 0U;

        assert(Algorithm_Packet_Encode(
            "DIS", 9U, invalid_payload, sizeof(invalid_payload) - 1U,
            frame, sizeof(frame), &frame_length) ==
            ALGORITHM_PACKET_ENCODE_OK);
        fake_queue_rx(frame, frame_length);
        assert(Device_PeerLink_Step(&receiver, 40U));
        assert(receiver.status.motor_disable_payload_error_count == 1U);
    }
    assert(!Device_PeerLink_SendMotorDisableRequest(&receiver));
    assert(receiver.status.motor_disable_tx_error_count == 1U);

    fake_reset();
    sender_config.local_node_id = 2U;
    sender_config.remote_node_id = 1U;
    assert(Device_PeerLink_Init(&sender, &sender_config, 0U));
    assert(Device_PeerLink_SendMotorDisableRequest(&sender));
    assert(sender.status.motor_disable_tx_count == 1U);
    assert(Algorithm_Packet_Parser_Init(&parser, storage, sizeof(storage)));
    for (size_t index = 0U; index < s_last_tx_length; ++index) {
        (void)Algorithm_Packet_Parser_Feed(
            &parser, s_last_tx[index], &packet);
    }
    assert(packet.type_length == 3U);
    assert(memcmp(packet.type, "DIS", 3U) == 0);
    assert(packet.sequence == 0U);
    assert(packet.payload_length == 1U);
    assert(packet.payload[0] == '2');

    s_tx_busy = true;
    assert(Device_PeerLink_SendMotorDisableRequest(&sender));
    assert(sender.status.motor_disable_tx_drop_count == 1U);
    assert(sender.status.motor_disable_tx_count == 1U);
    s_tx_busy = false;
    assert(Device_PeerLink_SendMotorDisableRequest(&sender));
    assert(sender.status.motor_disable_tx_count == 2U);
    Algorithm_Packet_Parser_Reset(&parser);
    for (size_t index = 0U; index < s_last_tx_length; ++index) {
        (void)Algorithm_Packet_Parser_Feed(
            &parser, s_last_tx[index], &packet);
    }
    assert(packet.sequence == 1U);
}

static void test_absolute_encoder_receive_validation_and_age(void)
{
    Device_PeerLink link;
    Device_PeerLink_Config config = make_config();
    Device_PeerLink_AbsoluteEncoder encoder;
    uint8_t frame[ALGORITHM_PACKET_MAX_FRAME_SIZE];
    size_t length;

    fake_reset();
    config.local_node_id = 2U;
    config.remote_node_id = 1U;
    assert(Device_PeerLink_Init(&link, &config, 0U));
    length = encode_absolute_encoder(
        1U, 1U, UINT32_C(61047), 1U, 9U, frame);
    fake_queue_rx(frame, length);
    assert(Device_PeerLink_Step(&link, 100U));
    assert(link.status.encoder_rx_count == 1U);
    assert(Device_PeerLink_GetRemoteAbsoluteEncoder(
        &link, 300U, 200U, &encoder));
    assert(encoder.motor_id == 1U);
    assert(encoder.angle_raw == UINT16_C(61047));
    assert(encoder.online);
    assert(!Device_PeerLink_GetRemoteAbsoluteEncoder(
        &link, 301U, 200U, &encoder));

    fake_queue_rx(frame, length);
    assert(Device_PeerLink_Step(&link, 110U));
    assert(link.status.encoder_duplicate_sequence_count == 1U);

    length = encode_absolute_encoder(
        2U, 1U, UINT32_C(1234), 1U, 10U, frame);
    fake_queue_rx(frame, length);
    assert(Device_PeerLink_Step(&link, 120U));
    assert(link.status.encoder_source_error_count == 1U);

    length = encode_absolute_encoder(
        1U, 1U, UINT32_C(65536), 1U, 11U, frame);
    fake_queue_rx(frame, length);
    assert(Device_PeerLink_Step(&link, 130U));
    assert(link.status.encoder_payload_error_count == 1U);

    length = encode_absolute_encoder(
        1U, 15U, UINT16_MAX, 0U, 12U, frame);
    fake_queue_rx(frame, length);
    assert(Device_PeerLink_Step(&link, 140U));
    assert(link.status.encoder_rx_count == 3U);
    assert(Device_PeerLink_GetRemoteAbsoluteEncoder(
        &link, 140U, 200U, &encoder));
    assert(encoder.motor_id == 15U);
    assert(encoder.angle_raw == UINT16_MAX);
    assert(!encoder.online);

    {
        static const uint8_t invalid_payload[] = "1,1,01,1";
        size_t frame_length = 0U;

        assert(Algorithm_Packet_Encode(
            "ENC", 13U, invalid_payload, sizeof(invalid_payload) - 1U,
            frame, sizeof(frame), &frame_length) ==
            ALGORITHM_PACKET_ENCODE_OK);
        fake_queue_rx(frame, frame_length);
        assert(Device_PeerLink_Step(&link, 150U));
        assert(link.status.encoder_payload_error_count == 2U);
    }
}

int main(void)
{
    test_dma_required_and_first_frame();
    test_receive_duplicate_errors_and_timeout();
    test_queue_full_does_not_advance_sequence();
    test_wrap_and_per_step_limit();
    test_telemetry_send_and_busy_sequence();
    test_telemetry_receive_validation_and_age();
    test_motor_disable_request_send_receive_and_busy();
    test_absolute_encoder_receive_validation_and_age();
    puts("device_peer_link tests: PASS");
    return 0;
}
