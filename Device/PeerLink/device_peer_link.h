#ifndef DEVICE_PEER_LINK_H
#define DEVICE_PEER_LINK_H

#include "algorithm_packet.h"
#include "bsp_uart.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DEVICE_PEER_LINK_BAUD_RATE UINT32_C(115200)
#define DEVICE_PEER_LINK_RX_CHUNK_CAPACITY ALGORITHM_PACKET_MAX_FRAME_SIZE
#define DEVICE_PEER_LINK_TX_PAYLOAD_CAPACITY 40U

typedef struct {
    uint8_t motor_id;
    bool enabled;
    int32_t current_milliamps;
    int32_t speed_centi_rpm;
    uint32_t angle_milliradians;
    bool online;
    uint32_t received_at_ms;
    bool valid;
} Device_PeerLink_Telemetry;

typedef struct {
    uint8_t motor_id;
    /* QD4310 CAN feedback data[6..7], no unit, exact uint16 protocol value. */
    uint16_t angle_raw;
    bool online;
} Device_PeerLink_AbsoluteEncoder;

typedef struct {
    size_t transport_index;
    uint8_t local_node_id;
    uint8_t remote_node_id;
    uint32_t heartbeat_period_ms;
    uint32_t offline_timeout_ms;
    size_t maximum_rx_bytes_per_step;
} Device_PeerLink_Config;

typedef struct {
    bool online;
    uint32_t last_heartbeat_rx_ms;
    uint32_t last_remote_uptime_ms;
    uint32_t heartbeat_tx_count;
    uint32_t heartbeat_tx_drop_count;
    uint32_t heartbeat_tx_error_count;
    uint32_t heartbeat_rx_count;
    uint32_t packet_parser_error_count;
    uint32_t heartbeat_payload_error_count;
    uint32_t heartbeat_source_error_count;
    uint32_t duplicate_sequence_count;
    uint32_t telemetry_tx_count;
    uint32_t telemetry_tx_drop_count;
    uint32_t telemetry_tx_error_count;
    uint32_t telemetry_rx_count;
    uint32_t telemetry_payload_error_count;
    uint32_t telemetry_source_error_count;
    uint32_t telemetry_duplicate_sequence_count;
    uint32_t encoder_tx_count;
    uint32_t encoder_tx_drop_count;
    uint32_t encoder_tx_error_count;
    uint32_t step_error_count;
    BSP_UART_RxStatus uart_rx;
} Device_PeerLink_Status;

/* One Task owns each instance. UART/DMA ISR only fills/drains BSP rings;
 * protocol parsing and online-state updates happen in Step(). */
typedef struct {
    Device_PeerLink_Config config;
    Device_PeerLink_Status status;
    BSP_UART_Handle uart;
    Algorithm_Packet_Parser parser;
    uint8_t parser_storage[ALGORITHM_PACKET_PARSER_STORAGE_SIZE];
    uint8_t rx_chunk[DEVICE_PEER_LINK_RX_CHUNK_CAPACITY];
    uint8_t tx_payload[DEVICE_PEER_LINK_TX_PAYLOAD_CAPACITY];
    uint8_t tx_frame[ALGORITHM_PACKET_MAX_FRAME_SIZE];
    Device_PeerLink_Telemetry remote_telemetry;
    uint32_t next_heartbeat_tx_ms;
    uint16_t heartbeat_tx_sequence;
    uint16_t telemetry_tx_sequence;
    uint16_t encoder_tx_sequence;
    uint16_t last_heartbeat_rx_sequence;
    uint16_t last_telemetry_rx_sequence;
    bool heartbeat_rx_sequence_seen;
    bool telemetry_rx_sequence_seen;
    bool initialized;
} Device_PeerLink;

bool Device_PeerLink_Init(
    Device_PeerLink *link,
    const Device_PeerLink_Config *config,
    uint32_t now_ms);

/* Non-blocking periodic service. A false result records a local API/transport
 * error but never stops other product functions; call again next Task cycle. */
bool Device_PeerLink_Step(Device_PeerLink *link, uint32_t now_ms);

/* Queue one fixed-format F407 motor telemetry frame. BUSY drops the complete
 * frame without blocking or advancing its sequence and is reported in
 * status; false is reserved for invalid data or transport/API errors. */
bool Device_PeerLink_SendTelemetry(
    Device_PeerLink *link,
    const Device_PeerLink_Telemetry *telemetry);

/* Queue one F407 QD4310 absolute encoder frame:
 *   ENC=<node_id>,<motor_id>,<angle_raw>,<online>
 * BUSY drops the complete frame without blocking, retrying, or advancing
 * encoder_tx_sequence. */
bool Device_PeerLink_SendAbsoluteEncoder(
    Device_PeerLink *link,
    const Device_PeerLink_AbsoluteEncoder *encoder);

/* Copy the latest valid remote telemetry. Returns false when no frame has
 * arrived or its local receive age exceeds maximum_age_ms. */
bool Device_PeerLink_GetRemoteTelemetry(
    const Device_PeerLink *link,
    uint32_t now_ms,
    uint32_t maximum_age_ms,
    Device_PeerLink_Telemetry *telemetry);

#endif
