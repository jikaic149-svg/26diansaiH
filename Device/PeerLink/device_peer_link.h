#ifndef DEVICE_PEER_LINK_H
#define DEVICE_PEER_LINK_H

#include "algorithm_packet.h"
#include "bsp_uart.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DEVICE_PEER_LINK_BAUD_RATE UINT32_C(115200)
#define DEVICE_PEER_LINK_RX_CHUNK_CAPACITY ALGORITHM_PACKET_MAX_FRAME_SIZE
#define DEVICE_PEER_LINK_TX_PAYLOAD_CAPACITY 96U

#define DEVICE_PEER_LINK_CAR_SPEED_ABS_MAX_MM_PER_SECOND INT32_C(5000)
#define DEVICE_PEER_LINK_CAR_ACCEL_ABS_MAX_MM_PER_SECOND2 INT32_C(50000)
#define DEVICE_PEER_LINK_CAR_PLAN_ACCEL_ABS_MAX_MM_PER_SECOND2 INT32_C(5000)
#define DEVICE_PEER_LINK_CAR_WHEEL_ACCEL_ABS_MAX_MM_PER_SECOND2 INT32_C(5000)
#define DEVICE_PEER_LINK_CAR_PITCH_RATE_ABS_MAX_CENTI_DPS INT32_C(100000)
#define DEVICE_PEER_LINK_COMPETITION_MODE_MIN UINT8_C(2)
#define DEVICE_PEER_LINK_COMPETITION_MODE_MAX UINT8_C(6)
#define DEVICE_PEER_LINK_BALL_TARGET_ABS_MAX_0P1MM INT32_C(1250)

typedef struct {
    uint16_t sequence;
    uint32_t sample_ms;
    int32_t speed_x_mm_per_second;
    int32_t acceleration_x_mm_per_second2;
    int32_t acceleration_z_mm_per_second2;
    int32_t pitch_rate_centi_dps;
    int32_t planned_acceleration_x_mm_per_second2;
    int32_t filtered_wheel_acceleration_x_mm_per_second2;
    uint8_t competition_mode;
    bool competition_task_active;
    bool valid;
    bool planned_acceleration_valid;
    bool filtered_wheel_acceleration_valid;
    uint32_t received_at_ms;
} Device_PeerLink_CarState;

/* G3519 -> F407 ball command copies. Packet V1 and the packet Header SEQ
 * carry the explicit schema version and independent command sequence. */
typedef struct {
    int16_t target_0p1mm;
    uint16_t sequence;
    uint32_t received_at_ms;
} Device_PeerLink_BallTargetCommand;

typedef struct {
    uint16_t sequence;
    uint32_t received_at_ms;
} Device_PeerLink_BallStopCommand;

/* F407 -> G3519 fixed-point debug Latest Value for VOFA forwarding. */
typedef struct {
    int32_t ball_visual_position_0p1mm;
    int32_t ball_estimated_position_0p1mm;
    int32_t ball_target_command_0p1mm;
    int32_t ball_active_target_0p1mm;
    int32_t ball_velocity_0p1mm_s;
    uint16_t ball_measurement_age_ms;
    int32_t rod_command_mdeg;
    uint16_t motor_target_raw;
    uint16_t motor_feedback_raw;
    int32_t motor_command_current_ma;
    int32_t motor_feedback_current_ma;
    int32_t motor_speed_centi_rpm;
    uint8_t control_phase;
    uint16_t status_flags;
} Device_PeerLink_DebugTelemetry;

#define DEVICE_PEER_LINK_DEBUG_MOTOR_ONLINE        UINT16_C(0x0001)
#define DEVICE_PEER_LINK_DEBUG_MOTOR_ENABLED       UINT16_C(0x0002)
#define DEVICE_PEER_LINK_DEBUG_BALL_HOLD_ACTIVE    UINT16_C(0x0004)
#define DEVICE_PEER_LINK_DEBUG_ROD_SATURATED       UINT16_C(0x0008)
#define DEVICE_PEER_LINK_DEBUG_RAW_BLOCKED          UINT16_C(0x0010)
#define DEVICE_PEER_LINK_DEBUG_BALL_VALID           UINT16_C(0x0020)
#define DEVICE_PEER_LINK_DEBUG_PEER_FAULT           UINT16_C(0x0040)
#define DEVICE_PEER_LINK_DEBUG_VISION_FAULT         UINT16_C(0x0080)
#define DEVICE_PEER_LINK_DEBUG_TASK_FAULT           UINT16_C(0x0100)
#define DEVICE_PEER_LINK_DEBUG_STATUS_MASK          UINT16_C(0x01FF)

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
    uint32_t remote_session_reset_count;
    uint32_t car_state_rx_count;
    uint32_t car_state_payload_error_count;
    uint32_t car_state_source_error_count;
    uint32_t car_state_duplicate_sequence_count;
    uint32_t car_state_old_sequence_count;
    uint32_t car_state_old_sample_count;
    uint32_t ball_target_rx_count;
    uint32_t ball_target_payload_error_count;
    uint32_t ball_target_source_error_count;
    uint32_t ball_target_duplicate_sequence_count;
    uint32_t ball_target_old_sequence_count;
    uint32_t ball_stop_rx_count;
    uint32_t ball_stop_payload_error_count;
    uint32_t ball_stop_source_error_count;
    uint32_t ball_stop_duplicate_sequence_count;
    uint32_t ball_stop_old_sequence_count;
    uint32_t debug_tx_count;
    uint32_t debug_tx_drop_count;
    uint32_t debug_tx_error_count;
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
    Device_PeerLink_CarState remote_car_state;
    Device_PeerLink_BallTargetCommand pending_ball_target;
    Device_PeerLink_BallStopCommand pending_ball_stop;
    uint32_t next_heartbeat_tx_ms;
    uint16_t heartbeat_tx_sequence;
    uint16_t debug_tx_sequence;
    uint16_t last_heartbeat_rx_sequence;
    uint16_t last_car_state_rx_sequence;
    uint16_t last_ball_target_rx_sequence;
    uint16_t last_ball_stop_rx_sequence;
    uint32_t last_car_state_sample_ms;
    bool heartbeat_rx_sequence_seen;
    bool car_state_rx_sequence_seen;
    bool car_state_sample_seen;
    bool remote_car_state_seen;
    bool ball_target_rx_sequence_seen;
    bool ball_stop_rx_sequence_seen;
    bool ball_target_pending;
    bool ball_stop_pending;
    bool initialized;
} Device_PeerLink;

bool Device_PeerLink_Init(
    Device_PeerLink *link,
    const Device_PeerLink_Config *config,
    uint32_t now_ms);

/* Non-blocking periodic service. Unknown packet types are ignored. */
bool Device_PeerLink_Step(Device_PeerLink *link, uint32_t now_ms);

/* Copy the latest fresh CAR when either its physical-sensor branch or its
 * planned-acceleration branch is valid. Callers must inspect both flags.
 * Freshness uses the F407 local receive clock; maximum_age_ms is inclusive
 * and supports uint32_t wrap. */
bool Device_PeerLink_GetRemoteCarState(
    const Device_PeerLink *link,
    uint32_t now_ms,
    uint32_t maximum_age_ms,
    Device_PeerLink_CarState *state);

/* Copy the latest competition mode even when the physical CAR sample is
 * invalid. Mode 2..6 is informational only and expires on local receive age. */
bool Device_PeerLink_GetRemoteCompetitionMode(
    const Device_PeerLink *link,
    uint32_t now_ms,
    uint32_t maximum_age_ms,
    uint8_t *competition_mode);

/* Copy the periodic competition task state independently of physical CAR
 * sensor validity. Stale or never-seen CAR state returns false. */
bool Device_PeerLink_GetRemoteCompetitionTaskActive(
    const Device_PeerLink *link,
    uint32_t now_ms,
    uint32_t maximum_age_ms,
    bool *task_active);

/* BTG is a one-slot Latest Value. A newer accepted BTG replaces an untaken
 * value; Take copies the latest value once and clears the slot. */
bool Device_PeerLink_TakeBallTargetCommand(
    Device_PeerLink *link,
    Device_PeerLink_BallTargetCommand *command);

/* BST is a bounded discrete stop request. Multiple new BST frames received
 * before Take coalesce to the latest metadata because one stop is sufficient. */
bool Device_PeerLink_TakeBallStopRequest(
    Device_PeerLink *link,
    Device_PeerLink_BallStopCommand *command);

/* BUSY drops the complete DBG frame, returns true, and keeps its sequence. */
bool Device_PeerLink_SendDebugTelemetry(
    Device_PeerLink *link,
    const Device_PeerLink_DebugTelemetry *telemetry);


#endif
