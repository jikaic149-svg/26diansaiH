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

static void test_duplicate_old_and_unusable_do_not_drive(void)
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

    queue_vis(11U, "2060,0,0,0");
    assert(Device_VisionLink_Step(&link, 40U));
    assert(!Device_VisionLink_GetMeasurement(&link, 40U, &measurement));
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

int main(void)
{
    test_valid_flags_and_age_boundary();
    test_duplicate_old_and_unusable_do_not_drive();
    test_session_timeout_accepts_camera_restart();
    puts("vision link tests: PASS");
    return 0;
}
