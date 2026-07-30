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
static uint8_t s_rx[ALGORITHM_PACKET_MAX_FRAME_SIZE];
static size_t s_rx_length;

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
    size_t count;

    if ((uart != &s_uart) || (transferred == NULL) ||
        ((data == NULL) && (capacity != 0U))) {
        return BSP_UART_RESULT_INVALID_ARGUMENT;
    }
    count = (s_rx_length < capacity) ? s_rx_length : capacity;
    if (count > 0U) {
        memcpy(data, s_rx, count);
    }
    if (count < s_rx_length) {
        memmove(s_rx, &s_rx[count], s_rx_length - count);
    }
    s_rx_length -= count;
    *transferred = count;
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

static void fake_reset(void)
{
    memset(s_last_tx, 0, sizeof(s_last_tx));
    s_last_tx_length = 0U;
    s_tx_busy = false;
    memset(s_rx, 0, sizeof(s_rx));
    s_rx_length = 0U;
}

static void queue_packet(
    const char *type, uint16_t sequence, const char *payload)
{
    size_t frame_length = 0U;

    assert(Algorithm_Packet_Encode(
        type, sequence,
        (const uint8_t *)payload, strlen(payload),
        s_rx, sizeof(s_rx), &frame_length) ==
        ALGORITHM_PACKET_ENCODE_OK);
    s_rx_length = frame_length;
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

static void test_heartbeat_only_tx_and_busy_sequence(void)
{
    Device_PeerLink link;
    const Device_PeerLink_Config config = make_config();
    Algorithm_Packet_View packet;

    fake_reset();
    assert(Device_PeerLink_Init(&link, &config, 0U));
    s_tx_busy = true;
    assert(Device_PeerLink_Step(&link, 0U));
    assert(link.status.heartbeat_tx_drop_count == 1U);
    assert(link.status.heartbeat_tx_count == 0U);
    s_tx_busy = false;
    assert(Device_PeerLink_Step(&link, 500U));
    packet = parse_last_frame();
    assert(packet.type_length == 2U);
    assert(memcmp(packet.type, "HB", 2U) == 0);
    assert(packet.sequence == 0U);
    assert(packet.payload_length == sizeof("1,500") - 1U);
    assert(memcmp(packet.payload, "1,500", packet.payload_length) == 0);
}

static void test_car_boundaries_validity_and_freshness(void)
{
    Device_PeerLink link;
    const Device_PeerLink_Config config = make_config();
    Device_PeerLink_CarState state;

    fake_reset();
    assert(Device_PeerLink_Init(&link, &config, 100U));
    queue_packet(
        "CAR", 10U, "2,1000,-5000,-50000,50000,100000,1");
    assert(Device_PeerLink_Step(&link, 100U));
    assert(Device_PeerLink_GetRemoteCarState(
        &link, 150U, 50U, &state));
    assert(state.sample_ms == 1000U);
    assert(state.speed_x_mm_per_second == -5000);
    assert(state.acceleration_x_mm_per_second2 == -50000);
    assert(state.acceleration_z_mm_per_second2 == 50000);
    assert(state.pitch_rate_centi_dps == 100000);
    assert(!Device_PeerLink_GetRemoteCarState(
        &link, 151U, 50U, &state));

    queue_packet("CAR", 11U, "2,1010,0,0,0,0,0");
    assert(Device_PeerLink_Step(&link, 110U));
    assert(link.status.car_state_rx_count == 2U);
    assert(!Device_PeerLink_GetRemoteCarState(
        &link, 110U, 50U, &state));
}

static void test_car_rejects_malformed_and_unknown_types(void)
{
    Device_PeerLink link;
    const Device_PeerLink_Config config = make_config();

    fake_reset();
    assert(Device_PeerLink_Init(&link, &config, 0U));
    queue_packet("TEL", 0U, "1,1,1,0,0,0,1");
    assert(Device_PeerLink_Step(&link, 1U));
    assert(link.status.car_state_rx_count == 0U);

    queue_packet("CAR", 0U, "1,1,0,0,0,0,1");
    assert(Device_PeerLink_Step(&link, 2U));
    assert(link.status.car_state_source_error_count == 1U);
    queue_packet("CAR", 0U, "2,01,0,0,0,0,1");
    assert(Device_PeerLink_Step(&link, 3U));
    queue_packet("CAR", 0U, "2,1,+1,0,0,0,1");
    assert(Device_PeerLink_Step(&link, 4U));
    queue_packet("CAR", 0U, "2,1,0,0,0,0,2");
    assert(Device_PeerLink_Step(&link, 5U));
    queue_packet("CAR", 0U, "2,1,1,0,0,0,0");
    assert(Device_PeerLink_Step(&link, 6U));
    queue_packet("CAR", 0U, "2,1,5001,0,0,0,1");
    assert(Device_PeerLink_Step(&link, 7U));
    assert(link.status.car_state_payload_error_count == 5U);
    assert(link.status.car_state_rx_count == 0U);
}

static void test_car_order_wrap_and_restart(void)
{
    Device_PeerLink link;
    const Device_PeerLink_Config config = make_config();
    Device_PeerLink_CarState state;

    fake_reset();
    assert(Device_PeerLink_Init(&link, &config, 0U));
    queue_packet("HB", 0U, "2,5000");
    assert(Device_PeerLink_Step(&link, 1U));
    queue_packet("CAR", UINT16_MAX, "2,4294967295,1,2,3,4,1");
    assert(Device_PeerLink_Step(&link, 2U));
    queue_packet("CAR", 0U, "2,0,5,6,7,8,1");
    assert(Device_PeerLink_Step(&link, 3U));
    assert(Device_PeerLink_GetRemoteCarState(
        &link, 3U, 50U, &state));
    assert(state.speed_x_mm_per_second == 5);

    queue_packet("CAR", 0U, "2,1,9,9,9,9,1");
    assert(Device_PeerLink_Step(&link, 4U));
    queue_packet("CAR", UINT16_MAX, "2,1,9,9,9,9,1");
    assert(Device_PeerLink_Step(&link, 5U));
    queue_packet("CAR", 1U, "2,4294967295,9,9,9,9,1");
    assert(Device_PeerLink_Step(&link, 6U));
    assert(link.status.car_state_duplicate_sequence_count == 1U);
    assert(link.status.car_state_old_sequence_count == 1U);
    assert(link.status.car_state_old_sample_count == 1U);

    queue_packet("HB", 1U, "2,5");
    assert(Device_PeerLink_Step(&link, 7U));
    assert(link.status.remote_session_reset_count == 1U);
    assert(!Device_PeerLink_GetRemoteCarState(
        &link, 7U, 50U, &state));
    queue_packet("CAR", 0U, "2,5,-1,-2,-3,-4,1");
    assert(Device_PeerLink_Step(&link, 8U));
    assert(Device_PeerLink_GetRemoteCarState(
        &link, 8U, 50U, &state));
    assert(state.speed_x_mm_per_second == -1);
}

static void test_local_freshness_wrap(void)
{
    Device_PeerLink link;
    const Device_PeerLink_Config config = make_config();
    Device_PeerLink_CarState state;
    const uint32_t received_at = UINT32_MAX - 2U;

    fake_reset();
    assert(Device_PeerLink_Init(&link, &config, received_at));
    queue_packet("CAR", 0U, "2,1,1,2,3,4,1");
    assert(Device_PeerLink_Step(&link, received_at));
    assert(Device_PeerLink_GetRemoteCarState(
        &link, received_at + 50U, 50U, &state));
    assert(!Device_PeerLink_GetRemoteCarState(
        &link, received_at + 51U, 50U, &state));
}

static Device_PeerLink_DebugTelemetry make_debug_telemetry(void)
{
    return (Device_PeerLink_DebugTelemetry){
        .chassis_acceleration_mm_s2 = -1234,
        .chassis_compensation_mdeg = -321,
        .rod_command_mdeg = 456,
        .motor_target_raw = UINT16_C(54329),
        .motor_feedback_raw = UINT16_C(54379),
        .motor_command_current_ma = -17,
        .motor_feedback_current_ma = 18,
        .motor_speed_centi_rpm = -125,
        .control_phase = 2U,
        .status_flags = DEVICE_PEER_LINK_DEBUG_STATUS_MASK,
    };
}

static void test_debug_payload_and_busy_sequence(void)
{
    Device_PeerLink link;
    const Device_PeerLink_Config config = make_config();
    const Device_PeerLink_DebugTelemetry debug = make_debug_telemetry();
    Algorithm_Packet_View packet;

    fake_reset();
    assert(Device_PeerLink_Init(&link, &config, 0U));
    assert(Device_PeerLink_SendDebugTelemetry(&link, &debug));
    assert(link.status.debug_tx_count == 1U);
    assert(link.status.debug_tx_drop_count == 0U);
    assert(link.status.debug_tx_error_count == 0U);
    packet = parse_last_frame();
    assert(packet.type_length == 3U);
    assert(memcmp(packet.type, "DBG", 3U) == 0);
    assert(packet.sequence == 0U);
    assert(packet.payload_length ==
        sizeof("1,-1234,-321,456,54329,54379,-17,18,-125,2,511") - 1U);
    assert(memcmp(packet.payload,
        "1,-1234,-321,456,54329,54379,-17,18,-125,2,511",
        packet.payload_length) == 0);

    s_tx_busy = true;
    assert(Device_PeerLink_SendDebugTelemetry(&link, &debug));
    assert(link.status.debug_tx_count == 1U);
    assert(link.status.debug_tx_drop_count == 1U);
    assert(link.debug_tx_sequence == 1U);
    s_tx_busy = false;
    assert(Device_PeerLink_SendDebugTelemetry(&link, &debug));
    packet = parse_last_frame();
    assert(packet.sequence == 1U);
    assert(link.status.debug_tx_count == 2U);
}

static void test_debug_boundaries_and_rejections(void)
{
    Device_PeerLink link;
    const Device_PeerLink_Config config = make_config();
    Device_PeerLink_DebugTelemetry debug = {
        .chassis_acceleration_mm_s2 = -60000,
        .chassis_compensation_mdeg = -60000,
        .rod_command_mdeg = -60000,
        .motor_target_raw = UINT16_MAX,
        .motor_feedback_raw = UINT16_MAX,
        .motor_command_current_ma = -2000,
        .motor_feedback_current_ma = -10000,
        .motor_speed_centi_rpm = -100000,
        .control_phase = 2U,
        .status_flags = DEVICE_PEER_LINK_DEBUG_STATUS_MASK,
    };
    Algorithm_Packet_View packet;

    fake_reset();
    assert(Device_PeerLink_Init(&link, &config, 0U));
    assert(Device_PeerLink_SendDebugTelemetry(&link, &debug));
    packet = parse_last_frame();
    assert(packet.payload_length < DEVICE_PEER_LINK_TX_PAYLOAD_CAPACITY);
    assert(memcmp(packet.payload,
        "1,-60000,-60000,-60000,65535,65535,-2000,-10000,-100000,2,511",
        packet.payload_length) == 0);

    debug.chassis_acceleration_mm_s2 = 60001;
    assert(!Device_PeerLink_SendDebugTelemetry(&link, &debug));
    debug = make_debug_telemetry();
    debug.motor_command_current_ma = 2001;
    assert(!Device_PeerLink_SendDebugTelemetry(&link, &debug));
    debug = make_debug_telemetry();
    debug.control_phase = 3U;
    assert(!Device_PeerLink_SendDebugTelemetry(&link, &debug));
    debug = make_debug_telemetry();
    debug.status_flags = UINT16_C(0x0200);
    assert(!Device_PeerLink_SendDebugTelemetry(&link, &debug));
    assert(link.status.debug_tx_error_count == 4U);
    assert(link.status.debug_tx_count == 1U);
}

int main(void)
{
    test_heartbeat_only_tx_and_busy_sequence();
    test_car_boundaries_validity_and_freshness();
    test_car_rejects_malformed_and_unknown_types();
    test_car_order_wrap_and_restart();
    test_local_freshness_wrap();
    test_debug_payload_and_busy_sequence();
    test_debug_boundaries_and_rejections();
    puts("device_peer_link CAR/DBG tests: PASS");
    return 0;
}
