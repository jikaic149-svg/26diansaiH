#ifndef DEVICE_VISION_LINK_H
#define DEVICE_VISION_LINK_H

#include "algorithm_packet.h"
#include "bsp_uart.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DEVICE_VISION_LINK_BAUD_RATE UINT32_C(115200)
#define DEVICE_VISION_LINK_RX_CHUNK_CAPACITY ALGORITHM_PACKET_MAX_FRAME_SIZE

typedef struct {
    size_t transport_index;
    size_t maximum_rx_bytes_per_step;
    uint32_t session_timeout_ms;
    uint32_t maximum_measurement_age_ms;
    uint32_t camera_pipeline_cal_ms;
    uint32_t transport_bound_ms;
} Device_VisionLink_Config;

typedef struct {
    uint16_t sequence;
    uint32_t sampled_at_ms;
    uint16_t processing_age_ms;
    int16_t position_mm;
    uint8_t flags;
    uint32_t received_at_ms;
    uint32_t estimated_age_ms;
    bool usable;
    bool valid;
} Device_VisionLink_Measurement;

typedef struct {
    uint32_t packet_parser_error_count;
    uint32_t vis_rx_count;
    uint32_t vis_usable_count;
    uint32_t vis_payload_error_count;
    uint32_t vis_duplicate_sequence_count;
    uint32_t vis_old_sequence_count;
    uint32_t step_error_count;
    uint32_t last_vis_rx_ms;
    BSP_UART_RxStatus uart_rx;
} Device_VisionLink_Status;

typedef struct {
    Device_VisionLink_Config config;
    Device_VisionLink_Status status;
    BSP_UART_Handle uart;
    Algorithm_Packet_Parser parser;
    uint8_t parser_storage[ALGORITHM_PACKET_PARSER_STORAGE_SIZE];
    uint8_t rx_chunk[DEVICE_VISION_LINK_RX_CHUNK_CAPACITY];
    Device_VisionLink_Measurement measurement;
    uint16_t last_vis_sequence;
    bool vis_sequence_seen;
    bool initialized;
} Device_VisionLink;

/* VIS payload:
 *   sample_ms,proc_age_ms,position_mm,flags
 * Packet SEQ is the only sequence field. Parsing is non-blocking and owned by
 * one Task; the ISR only moves UART bytes into the BSP ring. */
bool Device_VisionLink_Init(
    Device_VisionLink *link,
    const Device_VisionLink_Config *config);
bool Device_VisionLink_Step(Device_VisionLink *link, uint32_t now_ms);

/* Returns only a real, calibrated, unclamped measurement whose conservative
 * age is strictly below maximum_measurement_age_ms. */
bool Device_VisionLink_GetMeasurement(
    const Device_VisionLink *link,
    uint32_t now_ms,
    Device_VisionLink_Measurement *measurement);

#endif
