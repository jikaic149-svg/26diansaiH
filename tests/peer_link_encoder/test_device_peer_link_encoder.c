#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "device_peer_link.h"

struct BSP_UART_Resource {
    uint8_t token;
};

static struct BSP_UART_Resource s_uart;
static uint8_t s_last_tx[ALGORITHM_PACKET_MAX_FRAME_SIZE];
static size_t s_last_tx_length;
static bool s_tx_busy;

BSP_UART_Handle BSP_UART_GetHandle(size_t index)
{
    return (index == 0U) ? &s_uart : NULL;
}

bool BSP_UART_GetInfo(BSP_UART_Handle uart, BSP_UART_Info *info)
{
    if ((uart != &s_uart) || (info == NULL)) {
        return false;
    }
    *info = (BSP_UART_Info){
        .baud_rate = DEVICE_PEER_LINK_BAUD_RATE,
        .supports_tx_dma = true,
        .supports_rx_dma = true,
        .supports_buffered_tx = true,
        .supports_buffered_rx = true,
        .tx_buffer_capacity = 511U,
        .rx_buffer_capacity = 511U,
    };
    return true;
}

BSP_UART_Result BSP_UART_WriteBuffered(
    BSP_UART_Handle uart, const uint8_t *data, size_t length)
{
    if ((uart != &s_uart) || (data == NULL) ||
        (length > sizeof(s_last_tx))) {
        return BSP_UART_RESULT_INVALID_ARGUMENT;
    }
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
    (void)data;
    (void)capacity;
    if ((uart != &s_uart) || (transferred == NULL)) {
        return BSP_UART_RESULT_INVALID_ARGUMENT;
    }
    *transferred = 0U;
    return BSP_UART_RESULT_OK;
}

bool BSP_UART_GetRxStatus(
    BSP_UART_Handle uart, BSP_UART_RxStatus *status)
{
    if ((uart != &s_uart) || (status == NULL)) {
        return false;
    }
    *status = (BSP_UART_RxStatus){
        .buffer_capacity = 511U,
    };
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

static Algorithm_Packet_View parse_last_frame(void)
{
    static Algorithm_Packet_Parser parser;
    static uint8_t storage[ALGORITHM_PACKET_PARSER_STORAGE_SIZE];
    Algorithm_Packet_View packet = {0};

    assert(Algorithm_Packet_Parser_Init(
        &parser, storage, sizeof(storage)));
    for (size_t index = 0U; index < s_last_tx_length; ++index) {
        const Algorithm_Packet_FeedResult result =
            Algorithm_Packet_Parser_Feed(
                &parser, s_last_tx[index], &packet);

        if ((index + 1U) == s_last_tx_length) {
            assert(result == ALGORITHM_PACKET_FEED_FRAME_READY);
        }
    }
    return packet;
}

static void test_encoder_payload_and_busy_sequence(void)
{
    Device_PeerLink link;
    const Device_PeerLink_Config config = make_config();
    const Device_PeerLink_AbsoluteEncoder encoder = {
        .motor_id = 1U,
        .angle_raw = UINT16_C(61047),
        .online = true,
    };
    Algorithm_Packet_View packet;

    s_last_tx_length = 0U;
    s_tx_busy = false;
    assert(Device_PeerLink_Init(&link, &config, 0U));
    assert(Device_PeerLink_SendAbsoluteEncoder(&link, &encoder));
    assert(link.status.encoder_tx_count == 1U);
    assert(link.status.encoder_tx_drop_count == 0U);
    assert(link.status.encoder_tx_error_count == 0U);
    packet = parse_last_frame();
    assert(packet.type_length == 3U);
    assert(memcmp(packet.type, "ENC", 3U) == 0);
    assert(packet.sequence == 0U);
    assert(packet.payload_length == sizeof("1,1,61047,1") - 1U);
    assert(memcmp(packet.payload, "1,1,61047,1",
        packet.payload_length) == 0);

    s_tx_busy = true;
    assert(Device_PeerLink_SendAbsoluteEncoder(&link, &encoder));
    assert(link.status.encoder_tx_drop_count == 1U);
    assert(link.status.encoder_tx_count == 1U);
    s_tx_busy = false;
    assert(Device_PeerLink_SendAbsoluteEncoder(&link, &encoder));
    packet = parse_last_frame();
    assert(packet.sequence == 1U);
    assert(link.status.encoder_tx_count == 2U);

    assert(!Device_PeerLink_SendAbsoluteEncoder(
        &link, &(Device_PeerLink_AbsoluteEncoder){
            .motor_id = 16U,
        }));
    assert(link.status.encoder_tx_error_count == 1U);
}

static void test_encoder_uint16_boundaries(void)
{
    Device_PeerLink link;
    const Device_PeerLink_Config config = make_config();
    Algorithm_Packet_View packet;

    assert(Device_PeerLink_Init(&link, &config, 0U));
    assert(Device_PeerLink_SendAbsoluteEncoder(
        &link, &(Device_PeerLink_AbsoluteEncoder){
            .motor_id = 15U,
            .angle_raw = UINT16_MAX,
            .online = false,
        }));
    packet = parse_last_frame();
    assert(packet.payload_length == sizeof("1,15,65535,0") - 1U);
    assert(memcmp(packet.payload, "1,15,65535,0",
        packet.payload_length) == 0);
}

static void test_existing_telemetry_format_is_unchanged(void)
{
    Device_PeerLink link;
    const Device_PeerLink_Config config = make_config();
    const Device_PeerLink_Telemetry telemetry = {
        .motor_id = 1U,
        .enabled = true,
        .current_milliamps = -250,
        .speed_centi_rpm = 12345,
        .angle_milliradians = 3142U,
        .online = true,
    };
    Algorithm_Packet_View packet;

    assert(Device_PeerLink_Init(&link, &config, 0U));
    assert(Device_PeerLink_SendTelemetry(&link, &telemetry));
    packet = parse_last_frame();
    assert(packet.type_length == 3U);
    assert(memcmp(packet.type, "TEL", 3U) == 0);
    assert(packet.sequence == 0U);
    assert(packet.payload_length ==
        sizeof("1,1,1,-250,12345,3142,1") - 1U);
    assert(memcmp(packet.payload, "1,1,1,-250,12345,3142,1",
        packet.payload_length) == 0);
}

int main(void)
{
    test_encoder_payload_and_busy_sequence();
    test_encoder_uint16_boundaries();
    test_existing_telemetry_format_is_unchanged();
    puts("peer link encoder tests: PASS");
    return 0;
}
