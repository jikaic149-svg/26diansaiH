#ifndef DEVICE_VISION_LINK_H
#define DEVICE_VISION_LINK_H

#include "algorithm_packet.h"
#include "bsp_uart.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DEVICE_VISION_LINK_BAUD_RATE UINT32_C(115200)
#define DEVICE_VISION_LINK_RX_CHUNK_CAPACITY ALGORITHM_PACKET_MAX_FRAME_SIZE
#define DEVICE_VISION_LINK_COMPETITION_MODE_UNKNOWN UINT8_C(0)
#define DEVICE_VISION_LINK_COMPETITION_MODE_MIN UINT8_C(2)
#define DEVICE_VISION_LINK_COMPETITION_MODE_MAX UINT8_C(6)

#define DEVICE_VISION_LINK_STATUS_VIS_RX_SEEN          UINT16_C(0x0001)
#define DEVICE_VISION_LINK_STATUS_MEASUREMENT_USED     UINT16_C(0x0002)
#define DEVICE_VISION_LINK_STATUS_LAST_VIS_USABLE      UINT16_C(0x0004)
#define DEVICE_VISION_LINK_STATUS_ESTIMATE_VALID       UINT16_C(0x0008)
#define DEVICE_VISION_LINK_STATUS_ESTIMATE_FRESH       UINT16_C(0x0010)
#define DEVICE_VISION_LINK_STATUS_TARGET_VALID         UINT16_C(0x0020)
#define DEVICE_VISION_LINK_STATUS_CONTROL_ARMED        UINT16_C(0x0040)
#define DEVICE_VISION_LINK_STATUS_BALL_OUTER_ACTIVE    UINT16_C(0x0080)
#define DEVICE_VISION_LINK_STATUS_LEVEL_LOOP_ACTIVE    UINT16_C(0x0100)
#define DEVICE_VISION_LINK_STATUS_MOTOR_ONLINE         UINT16_C(0x0200)
#define DEVICE_VISION_LINK_STATUS_MOTOR_ENABLED        UINT16_C(0x0400)
#define DEVICE_VISION_LINK_STATUS_THETA_APPLIED        UINT16_C(0x0800)
#define DEVICE_VISION_LINK_STATUS_LINK_ONLINE          UINT16_C(0x1000)
#define DEVICE_VISION_LINK_STATUS_VELOCITY_VALID       UINT16_C(0x2000)
#define DEVICE_VISION_LINK_STATUS_COMPETITION_TASK_ACTIVE UINT16_C(0x4000)
#define DEVICE_VISION_LINK_STATUS_FAULT_PRESENT        UINT16_C(0x8000)

#define DEVICE_VISION_LINK_FAULT_LINK_TIMEOUT          UINT16_C(0x0001)
#define DEVICE_VISION_LINK_FAULT_MEASUREMENT_STALE     UINT16_C(0x0002)
#define DEVICE_VISION_LINK_FAULT_UART_RX_OVERFLOW      UINT16_C(0x0004)
#define DEVICE_VISION_LINK_FAULT_UART_HW_ERROR         UINT16_C(0x0008)
#define DEVICE_VISION_LINK_FAULT_PACKET_ERROR          UINT16_C(0x0010)
#define DEVICE_VISION_LINK_FAULT_VIS_PAYLOAD_ERROR     UINT16_C(0x0020)
#define DEVICE_VISION_LINK_FAULT_VIS_SEQUENCE_ERROR    UINT16_C(0x0040)
#define DEVICE_VISION_LINK_FAULT_ESTIMATOR_ERROR       UINT16_C(0x0080)
#define DEVICE_VISION_LINK_FAULT_TARGET_INVALID        UINT16_C(0x0100)
#define DEVICE_VISION_LINK_FAULT_ANGLE_MAPPING         UINT16_C(0x0200)
#define DEVICE_VISION_LINK_FAULT_LEVEL_CONTROL         UINT16_C(0x0400)
#define DEVICE_VISION_LINK_FAULT_MOTOR_OFFLINE         UINT16_C(0x0800)
#define DEVICE_VISION_LINK_FAULT_TX_DROP_ERROR         UINT16_C(0x1000)
#define DEVICE_VISION_LINK_FAULT_DEADLINE_MISS         UINT16_C(0x2000)
#define DEVICE_VISION_LINK_FAULT_CONFIG_INVALID        UINT16_C(0x4000)

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

/* Maix -> F407 runtime tuning. All gains use 0.001 fixed-point units. */
typedef struct {
    uint16_t sequence;
    uint16_t kp_milli_per_s2;
    uint16_t ki_milli_per_s3;
    uint16_t kd_milli_per_s;
} Device_VisionLink_PidTuningCommand;

/* Maix -> F407 ball-position target. The signed value uses 0.1 mm units. */
typedef struct {
    uint16_t sequence;
    int16_t target_0p1mm;
} Device_VisionLink_BallTargetCommand;

/* F407 -> Maix VST snapshot. Integer domains exactly match COOPERATION 11.3. */
typedef struct {
    uint16_t last_rx_vis_sequence;
    uint16_t last_used_vis_sequence;
    uint16_t measurement_age_ms;
    int16_t estimate_mm;
    int16_t velocity_mm_s;
    int16_t target_mm;
    int32_t theta_command_mdeg;
    uint16_t motor_target_raw;
    uint16_t motor_feedback_raw;
    int16_t motor_command_ma;
    int16_t motor_speed_centi_rpm;
    int16_t stiction_kick_mdeg;
    uint16_t status_flags;
    uint16_t fault_bits;
    uint8_t competition_mode;
} Device_VisionLink_ControlStatus;

typedef struct {
    uint32_t packet_parser_error_count;
    uint32_t vis_rx_count;
    uint32_t vis_usable_count;
    uint32_t vis_payload_error_count;
    uint32_t vis_duplicate_sequence_count;
    uint32_t vis_old_sequence_count;
    uint32_t vtp_rx_count;
    uint32_t vtp_payload_error_count;
    uint32_t vtp_duplicate_sequence_count;
    uint32_t vtp_old_sequence_count;
    uint32_t vbt_rx_count;
    uint32_t vbt_payload_error_count;
    uint32_t vbt_duplicate_sequence_count;
    uint32_t vbt_old_sequence_count;
    uint32_t vst_tx_count;
    uint32_t vst_tx_drop_count;
    uint32_t vst_tx_error_count;
    uint32_t vpa_tx_count;
    uint32_t vpa_tx_drop_count;
    uint32_t vpa_tx_error_count;
    uint32_t step_error_count;
    uint32_t last_vis_rx_ms;
    uint32_t last_vtp_rx_ms;
    uint32_t last_vbt_rx_ms;
    BSP_UART_RxStatus uart_rx;
} Device_VisionLink_Status;

typedef struct {
    Device_VisionLink_Config config;
    Device_VisionLink_Status status;
    BSP_UART_Handle uart;
    Algorithm_Packet_Parser parser;
    uint8_t parser_storage[ALGORITHM_PACKET_PARSER_STORAGE_SIZE];
    uint8_t rx_chunk[DEVICE_VISION_LINK_RX_CHUNK_CAPACITY];
    uint8_t tx_payload[ALGORITHM_PACKET_MAX_PAYLOAD_LENGTH];
    uint8_t tx_frame[ALGORITHM_PACKET_MAX_FRAME_SIZE];
    Device_VisionLink_Measurement measurement;
    Device_VisionLink_PidTuningCommand pending_pid_tuning;
    Device_VisionLink_BallTargetCommand pending_ball_target;
    uint16_t last_vis_sequence;
    uint16_t last_vtp_sequence;
    uint16_t last_vbt_sequence;
    uint16_t vst_tx_sequence;
    uint16_t vpa_tx_sequence;
    bool vis_sequence_seen;
    bool vtp_sequence_seen;
    bool vbt_sequence_seen;
    bool pid_tuning_pending;
    bool ball_target_pending;
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

/* Consume at most one static Latest Value tuning command. */
bool Device_VisionLink_TakePidTuningCommand(
    Device_VisionLink *link,
    Device_VisionLink_PidTuningCommand *command);

/* Consume at most one Latest Value VBT target command. */
bool Device_VisionLink_TakeBallTargetCommand(
    Device_VisionLink *link,
    Device_VisionLink_BallTargetCommand *command);

/* Both calls atomically enqueue one complete frame into the BSP TX ring.
 * BUSY drops the complete status frame and increments diagnostics; it is not
 * a control-path failure and no automatic retry is performed. */
bool Device_VisionLink_SendControlStatus(
    Device_VisionLink *link,
    const Device_VisionLink_ControlStatus *status);
bool Device_VisionLink_SendPidTuningAck(
    Device_VisionLink *link,
    uint16_t request_sequence,
    bool accepted,
    uint16_t kp_milli_per_s2,
    uint16_t ki_milli_per_s3,
    uint16_t kd_milli_per_s);

#endif
