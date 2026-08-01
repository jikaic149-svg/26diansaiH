#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "device_vision_link.h"

struct BSP_UART_Resource {
    uint8_t token;
};

static struct BSP_UART_Resource s_uart;
static uint8_t s_rx[ALGORITHM_PACKET_MAX_FRAME_SIZE];
static size_t s_rx_length;
static uint8_t s_tx[ALGORITHM_PACKET_MAX_FRAME_SIZE];
static size_t s_tx_length;
static BSP_UART_Result s_tx_result;

BSP_UART_Handle BSP_UART_GetHandle(size_t index)
{
    return (index == 2U) ? &s_uart : NULL;
}

bool BSP_UART_GetInfo(BSP_UART_Handle uart, BSP_UART_Info *info)
{
    if ((uart != &s_uart) || (info == NULL)) {
        return false;
    }
    *info = (BSP_UART_Info){
        .baud_rate = DEVICE_VISION_LINK_BAUD_RATE,
        .supports_buffered_tx = true,
        .supports_buffered_rx = true,
        .tx_buffer_capacity = 511U,
        .rx_buffer_capacity = 511U,
    };
    return true;
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
    memcpy(data, s_rx, count);
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
        .buffered_bytes = s_rx_length,
        .buffer_capacity = 511U,
    };
    return true;
}

BSP_UART_Result BSP_UART_WriteBuffered(
    BSP_UART_Handle uart, const uint8_t *data, size_t length)
{
    if ((uart != &s_uart) || ((data == NULL) && (length != 0U)) ||
        (length > sizeof(s_tx))) {
        return BSP_UART_RESULT_INVALID_ARGUMENT;
    }
    if (s_tx_result != BSP_UART_RESULT_OK) {
        return s_tx_result;
    }
    memcpy(s_tx, data, length);
    s_tx_length = length;
    return BSP_UART_RESULT_OK;
}

static Device_VisionLink_Config default_config(void)
{
    return (Device_VisionLink_Config){
        .transport_index = 2U,
        .maximum_rx_bytes_per_step = 128U,
        .session_timeout_ms = 100U,
        .maximum_measurement_age_ms = 100U,
        .camera_pipeline_cal_ms = 0U,
        .transport_bound_ms = 5U,
    };
}

static void queue_vis(uint16_t sequence, const char *payload)
{
    size_t frame_length = 0U;

    assert(Algorithm_Packet_Encode(
        "VIS", sequence,
        (const uint8_t *)payload, strlen(payload),
        s_rx, sizeof(s_rx), &frame_length) ==
        ALGORITHM_PACKET_ENCODE_OK);
    s_rx_length = frame_length;
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

static Algorithm_Packet_View parse_tx(
    Algorithm_Packet_Parser *parser, uint8_t *storage)
{
    Algorithm_Packet_View view = {0};

    assert(Algorithm_Packet_Parser_Init(
        parser, storage, ALGORITHM_PACKET_PARSER_STORAGE_SIZE));
    for (size_t index = 0U; index < s_tx_length; ++index) {
        const Algorithm_Packet_FeedResult result =
            Algorithm_Packet_Parser_Feed(parser, s_tx[index], &view);

        if (index + 1U == s_tx_length) {
            assert(result == ALGORITHM_PACKET_FEED_FRAME_READY);
        }
        else {
            assert(result == ALGORITHM_PACKET_FEED_IN_PROGRESS);
        }
    }
    return view;
}

static void test_valid_flags_and_age_boundary(void)
{
    Device_VisionLink link;
    Device_VisionLink_Measurement measurement;
    const Device_VisionLink_Config config = default_config();

    s_rx_length = 0U;
    assert(Device_VisionLink_Init(&link, &config));
    /* 53 = YOLO_MEASURED | AXIS_MEASURED | POSITION_VALID |
     * CALIBRATION_VALID. */
    queue_vis(0U, "1000,10,-25,53");
    assert(Device_VisionLink_Step(&link, 100U));
    assert(Device_VisionLink_GetMeasurement(&link, 184U, &measurement));
    assert(measurement.position_mm == -25);
    assert(measurement.estimated_age_ms == 99U);
    assert(!Device_VisionLink_GetMeasurement(&link, 185U, &measurement));
}

static void test_duplicate_and_old_do_not_replace_measurement(void)
{
    Device_VisionLink link;
    Device_VisionLink_Measurement measurement;
    const Device_VisionLink_Config config = default_config();

    assert(Device_VisionLink_Init(&link, &config));
    queue_vis(10U, "2000,0,10,53");
    assert(Device_VisionLink_Step(&link, 10U));
    queue_vis(10U, "2020,0,20,53");
    assert(Device_VisionLink_Step(&link, 20U));
    assert(link.status.vis_duplicate_sequence_count == 1U);
    assert(Device_VisionLink_GetMeasurement(&link, 20U, &measurement));
    assert(measurement.position_mm == 10);

    queue_vis(9U, "2040,0,30,53");
    assert(Device_VisionLink_Step(&link, 30U));
    assert(link.status.vis_old_sequence_count == 1U);
}

static void test_unusable_vis_holds_last_usable_until_age_limit(void)
{
    Device_VisionLink link;
    Device_VisionLink_Measurement measurement;
    const Device_VisionLink_Config config = default_config();

    assert(Device_VisionLink_Init(&link, &config));
    queue_vis(10U, "2000,0,10,53");
    assert(Device_VisionLink_Step(&link, 10U));
    assert(link.status.vis_rx_count == 1U);
    assert(link.status.vis_usable_count == 1U);

    queue_vis(11U, "2060,0,0,0");
    assert(Device_VisionLink_Step(&link, 40U));
    assert(link.last_vis_sequence == 11U);
    assert(link.status.last_vis_rx_ms == 40U);
    assert(link.status.vis_rx_count == 2U);
    assert(link.status.vis_usable_count == 1U);

    assert(Device_VisionLink_GetMeasurement(&link, 40U, &measurement));
    assert(measurement.sequence == 10U);
    assert(measurement.position_mm == 10);
    assert(measurement.received_at_ms == 10U);
    assert(measurement.estimated_age_ms == 35U);
    assert(Device_VisionLink_GetMeasurement(&link, 104U, &measurement));
    assert(measurement.estimated_age_ms == 99U);
    assert(!Device_VisionLink_GetMeasurement(&link, 105U, &measurement));

    queue_vis(12U, "2080,3,-20,53");
    assert(Device_VisionLink_Step(&link, 110U));
    assert(link.status.vis_rx_count == 3U);
    assert(link.status.vis_usable_count == 2U);
    assert(Device_VisionLink_GetMeasurement(&link, 110U, &measurement));
    assert(measurement.sequence == 12U);
    assert(measurement.position_mm == -20);
    assert(measurement.received_at_ms == 110U);
    assert(measurement.estimated_age_ms == 8U);
}

static void test_session_timeout_accepts_camera_restart(void)
{
    Device_VisionLink link;
    Device_VisionLink_Measurement measurement;
    const Device_VisionLink_Config config = default_config();

    assert(Device_VisionLink_Init(&link, &config));
    queue_vis(100U, "5000,0,5,53");
    assert(Device_VisionLink_Step(&link, 0U));
    queue_vis(0U, "0,0,-5,53");
    assert(Device_VisionLink_Step(&link, 100U));
    assert(Device_VisionLink_GetMeasurement(&link, 100U, &measurement));
    assert(measurement.sequence == 0U);
    assert(measurement.position_mm == -5);
}

static void test_pid_tuning_latest_value_and_ack(void)
{
    Device_VisionLink link;
    Device_VisionLink_PidTuningCommand command;
    Algorithm_Packet_Parser parser;
    Algorithm_Packet_View view;
    uint8_t storage[ALGORITHM_PACKET_PARSER_STORAGE_SIZE];
    char payload[64];
    const Device_VisionLink_Config config = default_config();

    s_rx_length = 0U;
    s_tx_length = 0U;
    s_tx_result = BSP_UART_RESULT_OK;
    assert(Device_VisionLink_Init(&link, &config));
    queue_packet("VTP", 7U, "1200,150,1800");
    assert(Device_VisionLink_Step(&link, 10U));
    assert(Device_VisionLink_TakePidTuningCommand(&link, &command));
    assert(command.sequence == 7U);
    assert(command.kp_milli_per_s2 == 1200U);
    assert(command.ki_milli_per_s3 == 150U);
    assert(command.kd_milli_per_s == 1800U);
    assert(!Device_VisionLink_TakePidTuningCommand(&link, &command));

    queue_packet("VTP", 7U, "2000,200,2000");
    assert(Device_VisionLink_Step(&link, 20U));
    assert(link.status.vtp_duplicate_sequence_count == 1U);
    assert(!Device_VisionLink_TakePidTuningCommand(&link, &command));

    assert(Device_VisionLink_SendPidTuningAck(
        &link, 7U, true, 1200U, 150U, 1800U));
    view = parse_tx(&parser, storage);
    assert((view.type_length == 3U) &&
        (memcmp(view.type, "VPA", 3U) == 0));
    assert(view.payload_length < sizeof(payload));
    memcpy(payload, view.payload, view.payload_length);
    payload[view.payload_length] = '\0';
    assert(strcmp(payload, "7,1,1200,150,1800") == 0);
}

static void test_ball_target_latest_value_sequence_and_payload(void)
{
    Device_VisionLink link;
    Device_VisionLink_BallTargetCommand command;
    const Device_VisionLink_Config config = default_config();

    s_rx_length = 0U;
    assert(Device_VisionLink_Init(&link, &config));

    queue_packet("VBT", 10U, "370");
    assert(Device_VisionLink_Step(&link, 10U));
    assert(Device_VisionLink_TakeBallTargetCommand(&link, &command));
    assert(command.sequence == 10U);
    assert(command.target_0p1mm == 370);

    queue_packet("VBT", 11U, "-1200");
    assert(Device_VisionLink_Step(&link, 20U));
    queue_packet("VBT", 12U, "400");
    assert(Device_VisionLink_Step(&link, 30U));
    assert(Device_VisionLink_TakeBallTargetCommand(&link, &command));
    assert(command.sequence == 12U);
    assert(command.target_0p1mm == 400);
    assert(link.status.vbt_rx_count == 3U);

    queue_packet("VBT", 12U, "500");
    assert(Device_VisionLink_Step(&link, 40U));
    assert(link.status.vbt_duplicate_sequence_count == 1U);
    queue_packet("VBT", 11U, "500");
    assert(Device_VisionLink_Step(&link, 50U));
    assert(link.status.vbt_old_sequence_count == 1U);
    assert(!Device_VisionLink_TakeBallTargetCommand(&link, &command));

    queue_packet("VBT", 13U, "+1");
    assert(Device_VisionLink_Step(&link, 60U));
    queue_packet("VBT", 13U, "01");
    assert(Device_VisionLink_Step(&link, 61U));
    queue_packet("VBT", 13U, "-0");
    assert(Device_VisionLink_Step(&link, 62U));
    queue_packet("VBT", 13U, "1251");
    assert(Device_VisionLink_Step(&link, 63U));
    queue_packet("VBT", 13U, "-1251");
    assert(Device_VisionLink_Step(&link, 64U));
    assert(link.status.vbt_payload_error_count == 5U);
    assert(!Device_VisionLink_TakeBallTargetCommand(&link, &command));

    /* Rejected payloads do not consume sequence 13. */
    queue_packet("VBT", 13U, "-1200");
    assert(Device_VisionLink_Step(&link, 70U));
    assert(Device_VisionLink_TakeBallTargetCommand(&link, &command));
    assert(command.sequence == 13U);
    assert(command.target_0p1mm == -1200);

    /* A new Maix session may restart its independent VBT sequence at zero. */
    queue_packet("VBT", 0U, "0");
    assert(Device_VisionLink_Step(&link, 170U));
    assert(Device_VisionLink_TakeBallTargetCommand(&link, &command));
    assert(command.sequence == 0U);
    assert(command.target_0p1mm == 0);
}

static void test_vst_encoding_and_busy_drop(void)
{
    Device_VisionLink link;
    Algorithm_Packet_Parser parser;
    Algorithm_Packet_View view;
    uint8_t storage[ALGORITHM_PACKET_PARSER_STORAGE_SIZE];
    char payload[96];
    const Device_VisionLink_Config config = default_config();
    const Device_VisionLink_ControlStatus status = {
        .last_rx_vis_sequence = 10U,
        .last_used_vis_sequence = 9U,
        .measurement_age_ms = 25U,
        .estimate_mm = -12,
        .velocity_mm_s = 34,
        .target_mm = 0,
        .theta_command_mdeg = -456,
        .motor_target_raw = 54329U,
        .motor_feedback_raw = 54340U,
        .motor_command_ma = -91,
        .motor_speed_centi_rpm = 325,
        .stiction_kick_mdeg = -450,
        .status_flags = UINT16_C(0xFFFF),
        .fault_bits = UINT16_C(0x0003),
        .competition_mode = 6U,
    };

    s_tx_length = 0U;
    s_tx_result = BSP_UART_RESULT_OK;
    assert(Device_VisionLink_Init(&link, &config));
    assert(Device_VisionLink_SendControlStatus(&link, &status));
    assert(link.status.vst_tx_count == 1U);
    view = parse_tx(&parser, storage);
    assert((view.type_length == 3U) &&
        (memcmp(view.type, "VST", 3U) == 0));
    assert(view.payload_length < sizeof(payload));
    memcpy(payload, view.payload, view.payload_length);
    payload[view.payload_length] = '\0';
    assert(strcmp(
        payload,
        "10,9,25,-12,34,0,-456,54329,54340,-91,325,-450,65535,3,6") == 0);

    s_tx_result = BSP_UART_RESULT_BUSY;
    assert(Device_VisionLink_SendControlStatus(&link, &status));
    assert(link.status.vst_tx_drop_count == 1U);
    assert(link.vst_tx_sequence == 1U);
}

int main(void)
{
    test_valid_flags_and_age_boundary();
    test_duplicate_and_old_do_not_replace_measurement();
    test_unusable_vis_holds_last_usable_until_age_limit();
    test_session_timeout_accepts_camera_restart();
    test_pid_tuning_latest_value_and_ack();
    test_ball_target_latest_value_sequence_and_payload();
    test_vst_encoding_and_busy_drop();
    puts("vision link tests: PASS");
    return 0;
}
