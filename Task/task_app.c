/* 静态网关 Task，骨架来自 TI-Template：静态栈/TCB + vTaskDelayUntil 固定周期。
 * 本模板不含 MSPM0 看门狗；F407 WWDG/IWDG 未启用。 */

#include "task_app.h"

#include <limits.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_gateway.h"
#include "algorithm_chassis_imu_acceleration.h"
#include "bsp_key.h"
#include "bsp_spi.h"
#include "core_config.h"
#include "device_bmi088.h"
#include "device_peer_link.h"
#include "device_vision_link.h"

#define TASK_GATEWAY_PRIORITY \
    (tskIDLE_PRIORITY + GATEWAY_TASK_PRIORITY_OFFSET)
#define TASK_PEER_UART_INDEX 0U  /* BSP transport 0 = USART1 */
#define TASK_BMI088_MAX_DMA_COMPLETIONS_PER_PUMP 3U
#define TASK_BMI088_DMA_WAIT_TICKS              pdMS_TO_TICKS(1U)

typedef enum {
    TASK_VISION_LINK_FAULT_NONE = 0U,
    TASK_VISION_LINK_FAULT_CONFIG = 1U,
    TASK_VISION_LINK_FAULT_STEP = 2U,
    TASK_VISION_LINK_FAULT_TX = 3U,
} Task_VisionLinkFault;

static StaticTask_t g_TASK_GatewayTCB;
static StackType_t g_TASK_GatewayStack[GATEWAY_TASK_STACK_DEPTH];

static bool s_key_candidate_pressed;
static bool s_key_stable_pressed;
static uint32_t s_key_candidate_since_ms;
static uint32_t s_key_press_count;
static uint32_t s_key_command_errors;
static bool s_key2_candidate_pressed;
static bool s_key2_stable_pressed;
static uint32_t s_key2_candidate_since_ms;
static uint32_t s_key2_press_count;
static uint32_t s_key2_command_errors;
static Device_PeerLink s_peer_link;
static bool s_peer_link_ready;
static uint32_t s_peer_link_fault;
static Device_VisionLink s_vision_link;
static bool s_vision_link_ready;
static Task_VisionLinkFault s_vision_link_fault;
static uint32_t s_vision_link_started_ms;
static uint16_t s_last_used_vision_sequence;
static bool s_vision_measurement_used_seen;
static Device_BMI088 s_bmi088;
static bool s_bmi088_ready;
static uint32_t s_bmi088_fault;
static Algorithm_ChassisImuAcceleration s_chassis_imu_acceleration;
static uint32_t s_last_chassis_imu_sample_ms;
static bool s_chassis_imu_sample_seen;
static uint32_t s_next_debug_tx_ms;
static uint32_t s_next_vision_status_tx_ms;
static uint32_t s_bmi088_terminal_sequence_consumed;
static bool s_competition_task_active;

volatile uint32_t g_TASK_Heartbeat = 0U;
volatile uint32_t g_TASK_Fault = 0U;
volatile uint32_t g_TASK_Ready = 0U;
volatile uint32_t g_TASK_NowMs = 0U;
volatile uint32_t g_TASK_BMI088_Status = 0U;
volatile uint32_t g_TASK_BMI088_Sequence = 0U;
volatile int32_t g_TASK_BMI088_AccelX_mm_s2 = 0;
volatile int32_t g_TASK_BMI088_AccelY_mm_s2 = 0;
volatile int32_t g_TASK_BMI088_AccelZ_mm_s2 = 0;
volatile int32_t g_TASK_BMI088_ProcessedAccel_mm_s2 = 0;
volatile uint32_t g_TASK_BallTargetCommandErrors = 0U;
volatile uint32_t g_TASK_BallStopCommandErrors = 0U;
volatile uint32_t g_TASK_BallPidTuningErrors = 0U;

static void Task_KeyInit(uint32_t now_ms)
{
    const bool pressed = BSP_Key_IsPressed();
    const bool key2_pressed = BSP_Key2_IsPressed();

    s_key_candidate_pressed = pressed;
    s_key_stable_pressed = pressed;
    s_key_candidate_since_ms = now_ms;
    s_key_press_count = 0U;
    s_key_command_errors = 0U;
    s_key2_candidate_pressed = key2_pressed;
    s_key2_stable_pressed = key2_pressed;
    s_key2_candidate_since_ms = now_ms;
    s_key2_press_count = 0U;
    s_key2_command_errors = 0U;
}

static bool Task_Task6ModeActive(uint32_t now_ms)
{
    uint8_t competition_mode = 0U;

    return s_peer_link_ready &&
        Device_PeerLink_GetRemoteCompetitionMode(
            &s_peer_link,
            now_ms,
            PEER_LINK_OFFLINE_TIMEOUT_MS,
            &competition_mode) &&
        (competition_mode == GATEWAY_TASK6_COMPETITION_MODE);
}

static void Task_KeyStep(uint32_t now_ms)
{
    const bool pressed = BSP_Key_IsPressed();

    if (pressed != s_key_candidate_pressed) {
        s_key_candidate_pressed = pressed;
        s_key_candidate_since_ms = now_ms;
        return;
    }
    if ((pressed == s_key_stable_pressed) ||
        ((uint32_t)(now_ms - s_key_candidate_since_ms) <
            GATEWAY_KEY_DEBOUNCE_MS)) {
        return;
    }

    s_key_stable_pressed = pressed;
    if (pressed) {
        ++s_key_press_count;
#if GATEWAY_LEVEL_CALIBRATION_KEY_MODE == 1U
        if (!APP_Gateway_AdvancePrimaryMotorCalibration(now_ms)) {
#else
        if (!APP_Gateway_TogglePrimaryMotor(now_ms)) {
#endif
            ++s_key_command_errors;
        }
    }
}

static void Task_Key2Step(uint32_t now_ms)
{
    const bool pressed = BSP_Key2_IsPressed();

    if (pressed != s_key2_candidate_pressed) {
        s_key2_candidate_pressed = pressed;
        s_key2_candidate_since_ms = now_ms;
        return;
    }
    if ((pressed == s_key2_stable_pressed) ||
        ((uint32_t)(now_ms - s_key2_candidate_since_ms) <
            GATEWAY_KEY_DEBOUNCE_MS)) {
        return;
    }

    s_key2_stable_pressed = pressed;
    if (pressed && Task_Task6ModeActive(now_ms)) {
        ++s_key2_press_count;
        if (!APP_Gateway_AdvanceTask6LevelCalibration(now_ms)) {
            ++s_key2_command_errors;
        }
    }
}

static void Task_PeerLinkInit(uint32_t now_ms)
{
    const Device_PeerLink_Config config = {
        .transport_index = TASK_PEER_UART_INDEX,
        .local_node_id = PEER_LINK_LOCAL_NODE_ID,
        .remote_node_id = PEER_LINK_REMOTE_NODE_ID,
        .heartbeat_period_ms = PEER_LINK_HEARTBEAT_PERIOD_MS,
        .offline_timeout_ms = PEER_LINK_OFFLINE_TIMEOUT_MS,
        .maximum_rx_bytes_per_step = PEER_LINK_RX_BYTES_PER_STEP,
    };

    s_peer_link_fault = 0U;
    s_peer_link_ready = Device_PeerLink_Init(
        &s_peer_link, &config, now_ms);
    if (!s_peer_link_ready) {
        s_peer_link_fault = 1U;
    }
}

static void Task_TakePeerBallCommands(void)
{
    Device_PeerLink_BallStopCommand stop_command;
    Device_PeerLink_BallTargetCommand target_command;

    if (!s_peer_link_ready) {
        return;
    }
    if (Device_PeerLink_TakeBallStopRequest(
            &s_peer_link, &stop_command)) {
        if (!APP_Gateway_StopPrimaryMotor()) {
            if (g_TASK_BallStopCommandErrors < UINT32_MAX) {
                ++g_TASK_BallStopCommandErrors;
            }
        }
        /* STOP has priority over every BTG parsed in the same Task cycle.
         * Consume and discard that one-slot Latest Value so it cannot be
         * applied after a later local restart. */
        (void)Device_PeerLink_TakeBallTargetCommand(
            &s_peer_link, &target_command);
        return;
    }
    if (Device_PeerLink_TakeBallTargetCommand(
            &s_peer_link, &target_command)) {
        bool accepted = APP_Gateway_SetBallTargetMm(
            (float)target_command.target_0p1mm * 0.1F);
        if (!accepted && (g_TASK_BallTargetCommandErrors < UINT32_MAX)) {
            ++g_TASK_BallTargetCommandErrors;
        }
    }
}

static void Task_UpdateCompetitionTaskState(uint32_t now_ms)
{
    uint8_t competition_mode = 0U;
    bool task_active = false;

    s_competition_task_active =
        Device_PeerLink_GetRemoteCompetitionMode(
            &s_peer_link, now_ms, PEER_LINK_OFFLINE_TIMEOUT_MS,
            &competition_mode) &&
        Device_PeerLink_GetRemoteCompetitionTaskActive(
            &s_peer_link, now_ms, PEER_LINK_OFFLINE_TIMEOUT_MS,
            &task_active) &&
        (competition_mode == GATEWAY_TASK3_COMPETITION_MODE) && task_active;
}

static void Task_VisionLinkInit(uint32_t now_ms)
{
    const Device_VisionLink_Config config = {
        .transport_index = VISION_LINK_UART_INDEX,
        .maximum_rx_bytes_per_step = VISION_LINK_RX_BYTES_PER_STEP,
        .session_timeout_ms = VISION_LINK_SESSION_TIMEOUT_MS,
        .maximum_measurement_age_ms =
            VISION_LINK_MEASUREMENT_MAX_AGE_MS,
        .camera_pipeline_cal_ms = VISION_LINK_CAMERA_PIPELINE_CAL_MS,
        .transport_bound_ms = VISION_LINK_TRANSPORT_BOUND_MS,
    };

    s_vision_link_fault = TASK_VISION_LINK_FAULT_NONE;
    s_vision_link_started_ms = now_ms;
    s_last_used_vision_sequence = 0U;
    s_vision_measurement_used_seen = false;
    s_vision_link_ready = Device_VisionLink_Init(
        &s_vision_link, &config);
    if (!s_vision_link_ready) {
        s_vision_link_fault = TASK_VISION_LINK_FAULT_CONFIG;
    }
}

static uint16_t Task_GainToMilli(float gain)
{
    const float scaled = gain * 1000.0F;

    if (!isfinite(scaled) || (scaled <= 0.0F)) {
        return 0U;
    }
    if (scaled >= (float)UINT16_MAX) {
        return UINT16_MAX;
    }
    return (uint16_t)(scaled + 0.5F);
}

static void Task_TakeVisionPidTuning(void)
{
    Device_VisionLink_PidTuningCommand command;
    float kp;
    float ki;
    float kd;
    bool accepted;

    if (!s_vision_link_ready ||
        !Device_VisionLink_TakePidTuningCommand(
            &s_vision_link, &command)) {
        return;
    }
    accepted = APP_Gateway_SetBallPidGains(
        (float)command.kp_milli_per_s2 * 0.001F,
        (float)command.ki_milli_per_s3 * 0.001F,
        (float)command.kd_milli_per_s * 0.001F);
    if (!accepted && (g_TASK_BallPidTuningErrors < UINT32_MAX)) {
        ++g_TASK_BallPidTuningErrors;
    }
    if (!APP_Gateway_GetBallPidGains(&kp, &ki, &kd) ||
        !Device_VisionLink_SendPidTuningAck(
            &s_vision_link,
            command.sequence,
            accepted,
            Task_GainToMilli(kp),
            Task_GainToMilli(ki),
            Task_GainToMilli(kd))) {
        s_vision_link_fault = TASK_VISION_LINK_FAULT_TX;
    }
}

static void Task_TakeVisionBallTarget(void)
{
    Device_VisionLink_BallTargetCommand command;

    if (!s_vision_link_ready ||
        !Device_VisionLink_TakeBallTargetCommand(
            &s_vision_link, &command)) {
        return;
    }
    if (!APP_Gateway_SetBallTargetMm(
            (float)command.target_0p1mm * 0.1F) &&
        (g_TASK_BallTargetCommandErrors < UINT32_MAX)) {
        ++g_TASK_BallTargetCommandErrors;
    }
}

static void Task_BMI088DmaCompleteFromISR(void *context)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    const uint32_t transfer_sequence =
        Device_BMI088_GetTransferSequence(&s_bmi088);

    if ((context == NULL) || (transfer_sequence == 0U)) {
        return;
    }
    (void)xTaskNotifyFromISR(
        (TaskHandle_t)context,
        transfer_sequence,
        eSetValueWithOverwrite,
        &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void Task_BMI088Init(uint32_t now_ms)
{
    Device_BMI088_Config config = Device_BMI088_DefaultConfig();
    const BSP_SPI_Handle spi = BSP_SPI_GetHandle(0U);

    config.transport_index = 0U;
    config.expected_bit_rate_hz = UINT32_C(5250000);
    config.transfer_timeout_ms = UINT32_C(5);
    s_bmi088_fault = 0U;
    s_bmi088_terminal_sequence_consumed = 0U;
    if ((spi == NULL) ||
        !BSP_SPI_SetCompletionCallbackFromISR(
            spi,
            Task_BMI088DmaCompleteFromISR,
            xTaskGetCurrentTaskHandle())) {
        s_bmi088_fault = 1U;
        s_bmi088_ready = false;
        return;
    }
    s_bmi088_ready = Device_BMI088_Init(&s_bmi088, &config, now_ms);
    if (!s_bmi088_ready) {
        s_bmi088_fault = (uint32_t)Device_BMI088_GetStatus(&s_bmi088);
    }
}

static void Task_BMI088Step(uint32_t now_ms)
{
    if (s_bmi088_ready &&
        !Device_BMI088_Step(&s_bmi088, now_ms)) {
        s_bmi088_fault =
            (uint32_t)Device_BMI088_GetStatus(&s_bmi088);
        g_TASK_BMI088_Status = s_bmi088_fault;
        s_bmi088_ready = false;
    }
}

static uint32_t Task_CurrentMs(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void Task_BMI088PumpFrame(uint32_t now_ms)
{
    uint32_t completed_transfers = 0U;

    while (s_bmi088_ready &&
        (completed_transfers <
            TASK_BMI088_MAX_DMA_COMPLETIONS_PER_PUMP)) {
        uint32_t transfer_sequence;

        if (!Device_BMI088_IsTransferPending(&s_bmi088)) {
            uint32_t discarded_notification;

            /* No transaction is active, so any old notification is stale. */
            (void)xTaskNotifyWait(
                0U, UINT32_MAX, &discarded_notification, 0U);
            Task_BMI088Step(now_ms);
            if (!s_bmi088_ready ||
                !Device_BMI088_IsTransferPending(&s_bmi088)) {
                return;
            }
        }
        transfer_sequence =
            Device_BMI088_GetTransferSequence(&s_bmi088);
        if (s_bmi088_terminal_sequence_consumed != transfer_sequence) {
            uint32_t notified_sequence = 0U;

            if ((xTaskNotifyWait(
                    0U,
                    UINT32_MAX,
                    &notified_sequence,
                    TASK_BMI088_DMA_WAIT_TICKS) != pdTRUE) ||
                (notified_sequence != transfer_sequence)) {
                /* Finite timeout or stale token: let the Device poll/timeout
                 * state machine own the failure decision, then return. */
                Task_BMI088Step(Task_CurrentMs());
                return;
            }
            s_bmi088_terminal_sequence_consumed = transfer_sequence;
        }
        now_ms = Task_CurrentMs();
        Task_BMI088Step(now_ms);
        if (!s_bmi088_ready) {
            return;
        }
        if (Device_BMI088_IsTransferPending(&s_bmi088) &&
            (Device_BMI088_GetTransferSequence(&s_bmi088) ==
                transfer_sequence)) {
            /* DMA notified before the final SPI BSY bit cleared. Do not wait
             * for a second notification that cannot exist; poll next cycle. */
            return;
        }
        ++completed_transfers;
    }
}

static APP_Gateway_ControlInput Task_MakeControlInput(uint32_t now_ms)
{
    APP_Gateway_ControlInput input = {0};
    Device_BMI088_Snapshot bmi088 = {0};
    Device_PeerLink_CarState car_state = {0};
    Device_VisionLink_Measurement measurement;
    Algorithm_ChassisImuAcceleration_Output acceleration_output;
    uint8_t competition_mode = 0U;
    const bool competition_mode_valid =
        s_peer_link_ready &&
        Device_PeerLink_GetRemoteCompetitionMode(
            &s_peer_link,
            now_ms,
            PEER_LINK_OFFLINE_TIMEOUT_MS,
            &competition_mode);
    const bool acceleration_feedforward_blocked =
        !competition_mode_valid ||
        (competition_mode ==
            GATEWAY_ACCELERATION_FF_DISABLED_COMPETITION_MODE);

    input.competition_mode = competition_mode;

    if (s_bmi088_ready &&
        Device_BMI088_GetSnapshot(&s_bmi088, &bmi088)) {
        g_TASK_BMI088_Status = (uint32_t)bmi088.status;
        g_TASK_BMI088_Sequence = bmi088.sequence;
        g_TASK_BMI088_AccelX_mm_s2 = (int32_t)(
            bmi088.acceleration_m_s2[0] * 1000.0F);
        g_TASK_BMI088_AccelY_mm_s2 = (int32_t)(
            bmi088.acceleration_m_s2[1] * 1000.0F);
        g_TASK_BMI088_AccelZ_mm_s2 = (int32_t)(
            bmi088.acceleration_m_s2[2] * 1000.0F);
    }
    if (s_bmi088_ready && bmi088.online && (bmi088.sequence != 0U) &&
        ((uint32_t)(now_ms - bmi088.sample_ms) <=
            GATEWAY_BMI088_ACCEL_MAX_INPUT_AGE_MS)) {
        if (!s_chassis_imu_sample_seen ||
            (bmi088.sample_ms != s_last_chassis_imu_sample_ms)) {
            const Algorithm_ChassisImuAcceleration_Sample sample = {
                .sample_ms = bmi088.sample_ms,
                .longitudinal_specific_force_mm_s2 =
                    bmi088.acceleration_m_s2[1] * 1000.0F *
                        GATEWAY_BMI088_ACCEL_Y_SIGN,
            };

            if (Algorithm_ChassisImuAcceleration_Update(
                    &s_chassis_imu_acceleration, &sample)) {
                s_last_chassis_imu_sample_ms = bmi088.sample_ms;
                s_chassis_imu_sample_seen = true;
            }
        }
        if (Algorithm_ChassisImuAcceleration_GetOutput(
                &s_chassis_imu_acceleration, &acceleration_output) &&
            (acceleration_output.sample_ms == bmi088.sample_ms)) {
            const int32_t acceleration_mm_s2 = (int32_t)
                (acceleration_output.filtered_acceleration_mm_s2 +
                    ((acceleration_output.filtered_acceleration_mm_s2 >= 0.0F)
                        ? 0.5F : -0.5F));

            g_TASK_BMI088_ProcessedAccel_mm_s2 = acceleration_mm_s2;
            if (acceleration_output.bias_ready &&
                !acceleration_feedforward_blocked) {
                /* Only the F407-local BMI088 may drive chassis
                 * acceleration feedforward. G3519 CAR acceleration is
                 * intentionally excluded from the rod command. Unknown or
                 * stale competition mode and M3 keep sampling the BMI088 but
                 * do not submit it to control. */
                input.chassis_acceleration_valid = true;
                input.acceleration_now_mm_s2 = acceleration_mm_s2;
                input.acceleration_next_mm_s2 = acceleration_mm_s2;
                input.acceleration_next_after_ms = UINT16_MAX;
                input.acceleration_received_at_ms = bmi088.sample_ms;
                input.chassis_motion_measurement_valid = true;
                input.chassis_motion_sequence = (uint16_t)bmi088.sequence;
                input.chassis_motion_sample_ms = bmi088.sample_ms;
                input.chassis_motion_age_ms = now_ms - bmi088.sample_ms;
                input.measured_chassis_acceleration_mm_s2 =
                    acceleration_mm_s2;
                input.measured_chassis_acceleration_valid = true;
            }
        }
    }

    if (s_peer_link_ready &&
        Device_PeerLink_GetRemoteCarState(
            &s_peer_link, now_ms, PEER_LINK_CAR_MAX_AGE_MS, &car_state)) {
        /* G3519 measured acceleration never enters rod feedforward. The
         * heartbeat-confirmed M4/M5/M6 profile is the shared longitudinal
         * truth; M2 remains local-IMU-only and M3 blocks feedforward. */
        if (!acceleration_feedforward_blocked &&
            s_peer_link.status.online &&
            (car_state.competition_mode >=
                GATEWAY_PLANNED_ACCELERATION_COMPETITION_MODE_MIN) &&
            (car_state.competition_mode <=
                GATEWAY_PLANNED_ACCELERATION_COMPETITION_MODE_MAX) &&
            car_state.planned_acceleration_valid) {
            input.chassis_acceleration_valid = true;
            input.chassis_acceleration_plan_valid = true;
            input.acceleration_now_mm_s2 =
                car_state.planned_acceleration_x_mm_per_second2;
            input.acceleration_next_mm_s2 =
                car_state.planned_acceleration_x_mm_per_second2;
            input.acceleration_next_after_ms = UINT16_MAX;
            input.acceleration_received_at_ms = car_state.received_at_ms;
        }
        if (car_state.valid) {
            input.chassis_wheel_measurement_valid = true;
            input.chassis_wheel_sequence = car_state.sequence;
            input.chassis_wheel_sample_ms = car_state.sample_ms;
            input.measured_chassis_wheel_speed_mm_s =
                car_state.speed_x_mm_per_second;
        }
        if (car_state.filtered_wheel_acceleration_valid) {
            input.chassis_wheel_acceleration_measurement_valid = true;
            input.chassis_wheel_acceleration_sequence = car_state.sequence;
            input.chassis_wheel_acceleration_sample_ms = car_state.sample_ms;
            input.measured_chassis_wheel_acceleration_mm_s2 =
                car_state.filtered_wheel_acceleration_x_mm_per_second2;
        }
        if ((car_state.competition_mode >=
                GATEWAY_PLANNED_ACCELERATION_COMPETITION_MODE_MIN) &&
            (car_state.competition_mode <=
                GATEWAY_PLANNED_ACCELERATION_COMPETITION_MODE_MAX) &&
            !car_state.planned_acceleration_valid && car_state.valid &&
            (fabsf((float)car_state.speed_x_mm_per_second) <=
                GATEWAY_CHASSIS_STATIONARY_WHEEL_SPEED_MM_S)) {
            /* M4/M5/M6 publish a valid zero plan throughout an active run.
             * Once that plan is withdrawn and measured speed is stationary,
             * do not fall back to local BMI088 noise between laps. */
            input.chassis_acceleration_valid = false;
        }
    }

    if (s_vision_link_ready &&
        Device_VisionLink_GetMeasurement(
            &s_vision_link, now_ms, &measurement)) {
        input.ball_measurement_valid = true;
        input.ball_sequence = measurement.sequence;
        input.ball_sample_ms = measurement.sampled_at_ms;
        input.ball_position_mm = measurement.position_mm;
        input.ball_measurement_age_ms = measurement.estimated_age_ms;
    }
    return input;
}

static int32_t Task_ScaleFloatToI32(float value, float scale)
{
    float scaled;

    if (!isfinite(value) || !isfinite(scale)) {
        return 0;
    }
    scaled = value * scale;

    if (scaled >= (float)INT32_MAX) {
        return INT32_MAX;
    }
    if (scaled <= (float)INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)(scaled + ((scaled >= 0.0F) ? 0.5F : -0.5F));
}

static int16_t Task_ClampI32ToI16(int32_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static void Task_SendVisionControlStatus(
    uint32_t now_ms,
    const APP_Gateway_ControlInput *input,
    const APP_Gateway_Telemetry *gateway)
{
    Device_VisionLink_ControlStatus status = {0};
    uint16_t flags = 0U;
    uint16_t faults = 0U;
    bool link_online;

    if (!s_vision_link_ready || (gateway == NULL) ||
        ((int32_t)(now_ms - s_next_vision_status_tx_ms) < 0)) {
        return;
    }
    s_next_vision_status_tx_ms =
        now_ms + VISION_LINK_STATUS_TX_PERIOD_MS;
    link_online = (s_vision_link.status.vis_rx_count > 0U) &&
        s_vision_link.measurement.valid &&
        ((uint32_t)(now_ms - s_vision_link.status.last_vis_rx_ms) <
            VISION_LINK_SESSION_TIMEOUT_MS);

    if (gateway->ball_last_used_sequence_valid) {
        s_last_used_vision_sequence =
            gateway->ball_last_used_sequence;
        s_vision_measurement_used_seen = true;
    }

    if (s_vision_link.status.vis_rx_count > 0U) {
        flags |= DEVICE_VISION_LINK_STATUS_VIS_RX_SEEN;
        status.last_rx_vis_sequence = s_vision_link.last_vis_sequence;
    }
    if (s_vision_measurement_used_seen) {
        flags |= DEVICE_VISION_LINK_STATUS_MEASUREMENT_USED;
        status.last_used_vis_sequence =
            s_last_used_vision_sequence;
    }
    if (s_vision_link.measurement.valid &&
        s_vision_link.measurement.usable) {
        flags |= DEVICE_VISION_LINK_STATUS_LAST_VIS_USABLE;
    }
    if (gateway->ball_measurement_valid) {
        flags |= DEVICE_VISION_LINK_STATUS_ESTIMATE_VALID |
            DEVICE_VISION_LINK_STATUS_ESTIMATE_FRESH;
    }
    flags |= DEVICE_VISION_LINK_STATUS_TARGET_VALID;
    if (gateway->level_control_phase != 0U) {
        flags |= DEVICE_VISION_LINK_STATUS_CONTROL_ARMED |
            DEVICE_VISION_LINK_STATUS_LEVEL_LOOP_ACTIVE;
        if (!gateway->motor_raw_limit_blocked) {
            flags |= DEVICE_VISION_LINK_STATUS_THETA_APPLIED;
        }
    }
    if ((gateway->level_control_phase == 2U) &&
        gateway->ball_measurement_valid) {
        flags |= DEVICE_VISION_LINK_STATUS_BALL_OUTER_ACTIVE;
    }
    if (gateway->online) {
        flags |= DEVICE_VISION_LINK_STATUS_MOTOR_ONLINE;
    }
    if (gateway->enabled) {
        flags |= DEVICE_VISION_LINK_STATUS_MOTOR_ENABLED;
    }
    if (link_online) {
        flags |= DEVICE_VISION_LINK_STATUS_LINK_ONLINE;
    }
    if (gateway->ball_velocity_valid) {
        flags |= DEVICE_VISION_LINK_STATUS_VELOCITY_VALID;
    }
    if (s_competition_task_active) {
        flags |= DEVICE_VISION_LINK_STATUS_COMPETITION_TASK_ACTIVE;
    }

    if (!link_online &&
        ((uint32_t)(now_ms - s_vision_link_started_ms) >=
            VISION_LINK_SESSION_TIMEOUT_MS)) {
        faults |= DEVICE_VISION_LINK_FAULT_LINK_TIMEOUT;
    }
    if ((s_vision_link.status.vis_rx_count > 0U) &&
        !gateway->ball_measurement_valid) {
        faults |= DEVICE_VISION_LINK_FAULT_MEASUREMENT_STALE;
    }
    if (s_vision_link.status.uart_rx.software_overflow_count > 0U) {
        faults |= DEVICE_VISION_LINK_FAULT_UART_RX_OVERFLOW;
    }
    if ((s_vision_link.status.step_error_count > 0U) ||
        (s_vision_link.status.uart_rx.hardware_overrun_count > 0U) ||
        (s_vision_link.status.uart_rx.framing_error_count > 0U) ||
        (s_vision_link.status.uart_rx.parity_error_count > 0U) ||
        (s_vision_link.status.uart_rx.dma_error_count > 0U)) {
        faults |= DEVICE_VISION_LINK_FAULT_UART_HW_ERROR;
    }
    if (s_vision_link.status.packet_parser_error_count > 0U) {
        faults |= DEVICE_VISION_LINK_FAULT_PACKET_ERROR;
    }
    if (s_vision_link.status.vis_payload_error_count > 0U) {
        faults |= DEVICE_VISION_LINK_FAULT_VIS_PAYLOAD_ERROR;
    }
    if ((s_vision_link.status.vis_duplicate_sequence_count > 0U) ||
        (s_vision_link.status.vis_old_sequence_count > 0U)) {
        faults |= DEVICE_VISION_LINK_FAULT_VIS_SEQUENCE_ERROR;
    }
    if (g_TASK_BallTargetCommandErrors > 0U) {
        faults |= DEVICE_VISION_LINK_FAULT_TARGET_INVALID;
    }
    if (gateway->motor_raw_limit_blocked) {
        faults |= DEVICE_VISION_LINK_FAULT_ANGLE_MAPPING;
    }
    if (gateway->level_control_fault_count > 0U) {
        faults |= DEVICE_VISION_LINK_FAULT_LEVEL_CONTROL;
    }
    if (!gateway->online) {
        faults |= DEVICE_VISION_LINK_FAULT_MOTOR_OFFLINE;
    }
    if ((s_vision_link.status.vst_tx_drop_count > 0U) ||
        (s_vision_link.status.vst_tx_error_count > 0U) ||
        (s_vision_link.status.vpa_tx_drop_count > 0U) ||
        (s_vision_link.status.vpa_tx_error_count > 0U)) {
        faults |= DEVICE_VISION_LINK_FAULT_TX_DROP_ERROR;
    }
    /* CONFIG_INVALID is reserved for Device_VisionLink_Init() rejecting the
     * static transport/configuration contract. Runtime Step/TX failures have
     * dedicated UART, packet and TX fault bits above. */
    if (s_vision_link_fault == TASK_VISION_LINK_FAULT_CONFIG) {
        faults |= DEVICE_VISION_LINK_FAULT_CONFIG_INVALID;
    }
    if (faults != 0U) {
        flags |= DEVICE_VISION_LINK_STATUS_FAULT_PRESENT;
    }

    status.measurement_age_ms = ((input != NULL) &&
        input->ball_measurement_valid) ?
        (uint16_t)((input->ball_measurement_age_ms < UINT16_MAX) ?
            input->ball_measurement_age_ms : UINT16_C(65534)) : UINT16_MAX;
    status.estimate_mm = gateway->ball_measurement_valid ?
        Task_ClampI32ToI16(Task_ScaleFloatToI32(
            gateway->ball_position_mm, 1.0F)) : 0;
    status.velocity_mm_s = gateway->ball_velocity_valid ?
        Task_ClampI32ToI16(Task_ScaleFloatToI32(
            gateway->ball_velocity_mm_s, 1.0F)) : 0;
    status.target_mm = Task_ClampI32ToI16(Task_ScaleFloatToI32(
        gateway->ball_target_mm, 1.0F));
    status.theta_command_mdeg = (gateway->level_control_phase != 0U) ?
        Task_ScaleFloatToI32(gateway->rod_command_deg, 1000.0F) : 0;
    status.motor_target_raw = gateway->target_single_turn_angle_raw;
    status.motor_feedback_raw = gateway->single_turn_angle_raw;
    status.motor_command_ma = Task_ClampI32ToI16(Task_ScaleFloatToI32(
        gateway->command_current_a, 1000.0F));
    status.motor_speed_centi_rpm = Task_ClampI32ToI16(Task_ScaleFloatToI32(
        gateway->speed_rpm, 100.0F));
    status.stiction_kick_mdeg = Task_ClampI32ToI16(Task_ScaleFloatToI32(
        gateway->ball_stiction_kick_deg, 1000.0F));
    status.status_flags = flags;
    status.fault_bits = faults;
    (void)Device_PeerLink_GetRemoteCompetitionMode(
        &s_peer_link, now_ms, PEER_LINK_OFFLINE_TIMEOUT_MS,
        &status.competition_mode);
    if (!Device_VisionLink_SendControlStatus(&s_vision_link, &status)) {
        s_vision_link_fault = TASK_VISION_LINK_FAULT_TX;
    }
}

static void Task_SendDebugTelemetry(
    uint32_t now_ms,
    const APP_Gateway_ControlInput *input,
    const APP_Gateway_Telemetry *gateway)
{
    Device_PeerLink_DebugTelemetry debug;
    uint16_t flags = 0U;

    if (!s_peer_link_ready || (gateway == NULL) ||
        ((int32_t)(now_ms - s_next_debug_tx_ms) < 0)) {
        return;
    }
    s_next_debug_tx_ms = now_ms + PEER_LINK_DEBUG_PERIOD_MS;
    if (gateway->online) {
        flags |= DEVICE_PEER_LINK_DEBUG_MOTOR_ONLINE;
    }
    if (gateway->enabled) {
        flags |= DEVICE_PEER_LINK_DEBUG_MOTOR_ENABLED;
    }
    if (gateway->ball_hold_active) {
        flags |= DEVICE_PEER_LINK_DEBUG_BALL_HOLD_ACTIVE;
    }
    if (gateway->rod_angle_saturated) {
        flags |= DEVICE_PEER_LINK_DEBUG_ROD_SATURATED;
    }
    if (gateway->motor_raw_limit_blocked) {
        flags |= DEVICE_PEER_LINK_DEBUG_RAW_BLOCKED;
    }
    if (gateway->ball_measurement_valid) {
        flags |= DEVICE_PEER_LINK_DEBUG_BALL_VALID;
    }
    if (s_peer_link_fault != 0U) {
        flags |= DEVICE_PEER_LINK_DEBUG_PEER_FAULT;
    }
    if (s_vision_link_fault != 0U) {
        flags |= DEVICE_PEER_LINK_DEBUG_VISION_FAULT;
    }
    if (g_TASK_Fault != 0U) {
        flags |= DEVICE_PEER_LINK_DEBUG_TASK_FAULT;
    }
    debug = (Device_PeerLink_DebugTelemetry){
        .ball_visual_position_0p1mm =
            ((input != NULL) && input->ball_measurement_valid) ?
            Task_ScaleFloatToI32(
                GATEWAY_BALL_MEASUREMENT_POLARITY *
                    (float)input->ball_position_mm,
                10.0F) : 0,
        .ball_estimated_position_0p1mm = Task_ScaleFloatToI32(
            gateway->ball_position_mm, 10.0F),
        .ball_target_command_0p1mm = Task_ScaleFloatToI32(
            gateway->ball_target_mm, 10.0F),
        .ball_active_target_0p1mm = Task_ScaleFloatToI32(
            gateway->ball_active_target_mm, 10.0F),
        .ball_velocity_0p1mm_s = Task_ScaleFloatToI32(
            gateway->ball_velocity_mm_s, 10.0F),
        .ball_measurement_age_ms =
            ((input != NULL) && input->ball_measurement_valid) ?
            (uint16_t)((input->ball_measurement_age_ms < UINT16_MAX) ?
                input->ball_measurement_age_ms : UINT16_MAX) : UINT16_MAX,
        .rod_command_mdeg = Task_ScaleFloatToI32(
            gateway->rod_command_deg, 1000.0F),
        .motor_target_raw = gateway->target_single_turn_angle_raw,
        .motor_feedback_raw = gateway->single_turn_angle_raw,
        .motor_command_current_ma = Task_ScaleFloatToI32(
            gateway->command_current_a, 1000.0F),
        .motor_feedback_current_ma = Task_ScaleFloatToI32(
            gateway->current_a, 1000.0F),
        .motor_speed_centi_rpm = Task_ScaleFloatToI32(
            gateway->speed_rpm, 100.0F),
        .control_phase = gateway->level_control_phase,
        .status_flags = flags,
    };
    if (!Device_PeerLink_SendDebugTelemetry(&s_peer_link, &debug)) {
        s_peer_link_fault = 3U;
    }
}

static void Task_Gateway(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;
    {
        const Algorithm_ChassisImuAcceleration_Config imu_config = {
            .bias_calibration_samples =
                GATEWAY_BMI088_ACCEL_Y_BIAS_CALIBRATION_SAMPLES,
            .acceleration_lowpass_cutoff_frequency_hz =
                GATEWAY_BMI088_ACCEL_LOWPASS_CUTOFF_HZ,
            .notch_sample_frequency_hz =
                GATEWAY_BMI088_ACCEL_NOTCH_SAMPLE_HZ,
            .notch_center_frequency_hz =
                GATEWAY_BMI088_ACCEL_NOTCH_CENTER_HZ,
            .notch_quality_factor =
                GATEWAY_BMI088_ACCEL_NOTCH_QUALITY_FACTOR,
            .maximum_absolute_acceleration_mm_s2 =
                GATEWAY_BMI088_ACCEL_MAX_ABS_OUTPUT_MM_S2,
            .maximum_sample_step_ms =
                GATEWAY_BMI088_ACCEL_MAX_SAMPLE_STEP_MS,
        };

        if (!Algorithm_ChassisImuAcceleration_Init(
                &s_chassis_imu_acceleration, &imu_config)) {
            g_TASK_Fault = 3U;
            for (;;) {
                vTaskDelay(pdMS_TO_TICKS(1000U));
            }
        }
    }
    if (!APP_Gateway_Init(
            (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS))) {
        g_TASK_Fault = 2U;
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000U));
        }
    }
    Task_KeyInit(
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));
    Task_PeerLinkInit(
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));
    Task_BMI088Init(
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));
    s_next_debug_tx_ms =
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    s_next_vision_status_tx_ms =
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    Task_VisionLinkInit(
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));

    for (;;) {
        APP_Gateway_Telemetry gateway_telemetry;
        APP_Gateway_ControlInput control_input;

        g_TASK_NowMs =
            (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        ++g_TASK_Heartbeat;
        /* The DMA ISR only notifies this Task. Finite, yielding waits complete
         * ACC then GYRO in the same 1 ms cycle. A third healthy completion is
         * allowed only to flush an initialization or prior-cycle remainder. */
        Task_BMI088PumpFrame(g_TASK_NowMs);
        g_TASK_NowMs = Task_CurrentMs();
        Task_KeyStep(g_TASK_NowMs);
        if (s_peer_link_ready &&
            !Device_PeerLink_Step(&s_peer_link, g_TASK_NowMs)) {
            s_peer_link_fault = 2U;
        }
        Task_Key2Step(g_TASK_NowMs);
        Task_TakePeerBallCommands();
        Task_UpdateCompetitionTaskState(g_TASK_NowMs);
        if (s_vision_link_ready &&
            !Device_VisionLink_Step(&s_vision_link, g_TASK_NowMs)) {
            s_vision_link_fault = TASK_VISION_LINK_FAULT_STEP;
        }
        Task_TakeVisionPidTuning();
        Task_TakeVisionBallTarget();
        control_input = Task_MakeControlInput(g_TASK_NowMs);
        APP_Gateway_Step(
            g_TASK_NowMs, &control_input, &gateway_telemetry);
        Task_SendVisionControlStatus(
            g_TASK_NowMs, &control_input, &gateway_telemetry);
        Task_SendDebugTelemetry(
            g_TASK_NowMs, &control_input, &gateway_telemetry);
        vTaskDelayUntil(
            &last_wake, pdMS_TO_TICKS(GATEWAY_TASK_PERIOD_MS));
    }
}

bool Task_Init(void)
{
    TaskHandle_t task;

    g_TASK_Heartbeat = 0U;
    g_TASK_Fault = 0U;
    g_TASK_Ready = 0U;
    g_TASK_NowMs = 0U;
    g_TASK_BMI088_Status = 0U;
    g_TASK_BMI088_Sequence = 0U;
    g_TASK_BMI088_AccelX_mm_s2 = 0;
    g_TASK_BMI088_AccelY_mm_s2 = 0;
    g_TASK_BMI088_AccelZ_mm_s2 = 0;
    g_TASK_BMI088_ProcessedAccel_mm_s2 = 0;
    g_TASK_BallTargetCommandErrors = 0U;
    g_TASK_BallStopCommandErrors = 0U;
    g_TASK_BallPidTuningErrors = 0U;
    s_last_chassis_imu_sample_ms = 0U;
    s_chassis_imu_sample_seen = false;
    s_next_debug_tx_ms = 0U;
    s_next_vision_status_tx_ms = 0U;
    s_vision_link_started_ms = 0U;
    s_last_used_vision_sequence = 0U;
    s_vision_measurement_used_seen = false;
    s_bmi088_terminal_sequence_consumed = 0U;
    s_competition_task_active = false;

    task = xTaskCreateStatic(
        Task_Gateway,
        GATEWAY_TASK_NAME,
        GATEWAY_TASK_STACK_DEPTH,
        NULL,
        TASK_GATEWAY_PRIORITY,
        g_TASK_GatewayStack,
        &g_TASK_GatewayTCB);
    if (task == NULL) {
        g_TASK_Fault = 1U;
        return false;
    }

    g_TASK_Ready = 1U;
    return true;
}
