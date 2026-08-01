/* TCP<->CAN 双向网关实现。
 * 下行：ESP8266 字节流 -> Packet 解析 -> QD4310 CAN 命令 -> ACK 回传。
 * 上行：周期 NOP 请求反馈 -> PollBus 收反馈帧 -> Packet 编码 -> TCP 发出。 */

#include "app_gateway.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "algorithm_ball_model_interpolator.h"
#include "algorithm_ball_position_control.h"
#include "algorithm_ball_state_estimator.h"
#include "algorithm_chassis_acceleration_fusion.h"
#include "algorithm_level_current_control.h"
#include "algorithm_packet.h"
#include "algorithm_rod_angle_map.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "core_config.h"
#include "device_esp8266.h"
#include "device_qd4310.h"
#include "task3_rod_sequence.h"

#define GATEWAY_ESP_UART_INDEX 1U  /* BSP transport 1 = USART3 */
#define GATEWAY_ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))

typedef enum {
    GATEWAY_LEVEL_PHASE_STOPPED = 0,
    GATEWAY_LEVEL_PHASE_ZEROING,
    GATEWAY_LEVEL_PHASE_ACCELERATION_COMPENSATION,
} Gateway_LevelPhase;

typedef enum {
    GATEWAY_TASK6_LEVEL_CALIBRATION_NORMAL = 0,
    GATEWAY_TASK6_LEVEL_CALIBRATION_ZERO_CURRENT,
    GATEWAY_TASK6_LEVEL_CALIBRATION_CAPTURED,
} Gateway_Task6LevelCalibrationPhase;

static esp8266_t s_wireless;
static Device_QD4310 s_motors[GATEWAY_MOTOR_COUNT];
static Device_QD4310 *s_motor_list[GATEWAY_MOTOR_COUNT];
static const uint8_t s_motor_ids[GATEWAY_MOTOR_COUNT] = GATEWAY_MOTOR_IDS;
static BSP_CAN_Handle s_can;
static Algorithm_LevelCurrentControl s_level_controller;
static Algorithm_BallModelInterpolator s_ball_model_interpolator;
static Algorithm_BallPositionControl s_ball_controller;
static Algorithm_BallStateEstimator s_ball_estimator;
static Algorithm_ChassisAccelerationFusion s_chassis_fusion;
static Algorithm_RodAngleMap s_rod_angle_map;
static Task3_RodSequence s_task3_rod_sequence;
static const Algorithm_RodAngleMap_Point s_rod_angle_points[] =
    GATEWAY_ROD_ANGLE_CALIBRATION_POINTS;

static Algorithm_Packet_Parser s_parser;
static uint8_t s_parser_storage[ALGORITHM_PACKET_PARSER_STORAGE_SIZE];

static uint32_t s_command_count;
static uint32_t s_command_errors;
static uint32_t s_next_feedback_ms;
static uint16_t s_tx_sequence;
static uint32_t s_last_feedback_sequence[GATEWAY_MOTOR_COUNT];
static uint32_t s_last_control_feedback_sequence;
static uint32_t s_level_control_fault_count;
static uint32_t s_level_enable_requested_ms;
static size_t s_level_calibration_index;
static bool s_level_calibration_active;
static bool s_level_calibration_target_reached;
static bool s_level_calibration_stable_seen;
static uint32_t s_level_calibration_point_started_ms;
static uint32_t s_level_calibration_stable_since_ms;
static uint32_t s_level_calibration_hold_since_ms;
static float s_rod_trim_angle_deg;
static float s_rod_minimum_angle_deg;
static float s_rod_maximum_angle_deg;
static float s_rod_command_deg;
static uint16_t s_motor_target_raw;
static float s_motor_command_current_a;
static int32_t s_selected_chassis_acceleration_mm_s2;
static int32_t s_planned_chassis_acceleration_mm_s2;
static bool s_last_chassis_imu_sequence_seen;
static uint16_t s_last_chassis_imu_sequence;
static uint32_t s_last_chassis_imu_sample_ms;
static bool s_last_chassis_wheel_sequence_seen;
static uint16_t s_last_chassis_wheel_sequence;
static uint32_t s_last_chassis_wheel_sample_ms;
static bool s_last_chassis_wheel_acceleration_sequence_seen;
static uint16_t s_last_chassis_wheel_acceleration_sequence;
static uint32_t s_last_chassis_wheel_acceleration_sample_ms;
static bool s_simple_plan_compensation_active;
static bool s_chassis_fusion_trusted;
static bool s_last_ball_sequence_seen;
static uint16_t s_last_ball_sequence;
static bool s_ball_measurement_valid;
static bool s_chassis_acceleration_valid;
static bool s_rod_angle_saturated;
static bool s_motor_raw_limit_blocked;
static Gateway_LevelPhase s_level_phase;
static bool s_zero_entry_stable_seen;
static uint32_t s_zero_entry_stable_since_ms;
static float s_chassis_compensation_angle_deg;
static uint32_t s_last_ball_target_update_ms;
static uint8_t s_direct_visual_competition_mode;
static Gateway_Task6LevelCalibrationPhase
    s_task6_level_calibration_phase;
static bool s_task6_level_zero_valid;
static uint16_t s_task6_level_zero_raw;
static float s_task6_level_zero_rod_deg;

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float raw_target_to_relative_motor_deg(float raw)
{
    float delta = raw - (float)GATEWAY_LEVEL_ZERO_RAW;

    if (delta > 32767.5F) {
        delta -= 65536.0F;
    }
    else if (delta < -32768.0F) {
        delta += 65536.0F;
    }
    return delta * (360.0F / 65535.0F);
}

static bool task6_mode_active(const APP_Gateway_ControlInput *input)
{
    return (input != NULL) &&
        (input->competition_mode == GATEWAY_TASK6_COMPETITION_MODE);
}

static float active_horizontal_rod_deg(
    const APP_Gateway_ControlInput *input)
{
    return (task6_mode_active(input) && s_task6_level_zero_valid) ?
        s_task6_level_zero_rod_deg : s_rod_trim_angle_deg;
}

static void reset_task6_level_calibration(void)
{
    s_task6_level_calibration_phase =
        GATEWAY_TASK6_LEVEL_CALIBRATION_NORMAL;
    s_task6_level_zero_valid = false;
    s_task6_level_zero_raw = 0U;
    s_task6_level_zero_rod_deg = 0.0F;
}

static size_t level_calibration_one_way_point_count(void)
{
    return GATEWAY_LEVEL_CALIBRATION_ONE_WAY_POINTS;
}

static size_t level_calibration_total_point_count(void)
{
    return 2U * level_calibration_one_way_point_count() - 1U;
}

static uint16_t level_calibration_target_raw(size_t index)
{
    const size_t one_way_count =
        level_calibration_one_way_point_count();
    const size_t total_count =
        level_calibration_total_point_count();
    const size_t progress_index = (index < one_way_count) ?
        index : (total_count - 1U - index);
    const uint32_t interval_count = (uint32_t)(one_way_count - 1U);
    const uint32_t offset =
        (GATEWAY_LEVEL_CALIBRATION_SPAN_RAW *
            (uint32_t)progress_index + (interval_count / 2U)) /
        interval_count;

    return (uint16_t)(GATEWAY_LEVEL_CALIBRATION_START_RAW + offset);
}

static int32_t raw_signed_error(uint16_t target, uint16_t feedback)
{
    int32_t delta = (int32_t)target - (int32_t)feedback;

    if (delta > INT16_MAX) {
        delta -= INT32_C(65536);
    }
    else if (delta < INT16_MIN) {
        delta += INT32_C(65536);
    }
    return delta;
}

static uint16_t raw_distance(uint16_t first, uint16_t second)
{
    const int32_t delta = raw_signed_error(first, second);

    return (uint16_t)((delta < 0) ? -delta : delta);
}

static bool apply_rod_target_deg(float requested_rod_angle_deg)
{
    float target_raw;
    float motor_target_deg;

    if (!isfinite(requested_rod_angle_deg)) {
        return false;
    }
    s_rod_command_deg = clamp_float(
        requested_rod_angle_deg,
        s_rod_minimum_angle_deg,
        s_rod_maximum_angle_deg);
    s_rod_angle_saturated =
        s_rod_command_deg != requested_rod_angle_deg;
    if (!Algorithm_RodAngleMap_RodDegToRaw(
            &s_rod_angle_map, s_rod_command_deg, &target_raw)) {
        return false;
    }
    if ((target_raw < (float)GATEWAY_LEVEL_RAW_MIN_LIMIT) ||
        (target_raw > (float)GATEWAY_LEVEL_RAW_MAX_LIMIT)) {
        s_motor_raw_limit_blocked = true;
        return false;
    }
    motor_target_deg = raw_target_to_relative_motor_deg(target_raw);
    if (!Algorithm_LevelCurrentControl_SetTrackingTargetDeg(
            &s_level_controller, motor_target_deg)) {
        return false;
    }
    s_motor_target_raw = (uint16_t)(target_raw + 0.5F);
    return true;
}

static int32_t select_chassis_acceleration(
    uint32_t now_ms,
    const APP_Gateway_ControlInput *input)
{
    uint32_t elapsed_ms;
    uint32_t compensated_elapsed_ms;
    uint32_t remaining_ms;

    if ((input == NULL) || !input->chassis_acceleration_valid) {
        return 0;
    }
    elapsed_ms = now_ms - input->acceleration_received_at_ms;
    compensated_elapsed_ms = elapsed_ms + GATEWAY_ACC_TRANSPORT_BOUND_MS;
    if (compensated_elapsed_ms < elapsed_ms) {
        compensated_elapsed_ms = UINT32_MAX;
    }
    if (input->acceleration_next_after_ms == UINT16_MAX) {
        return input->acceleration_now_mm_s2;
    }
    remaining_ms =
        ((uint32_t)input->acceleration_next_after_ms >
            compensated_elapsed_ms) ?
        (uint32_t)input->acceleration_next_after_ms -
            compensated_elapsed_ms : 0U;
    return (remaining_ms <= GATEWAY_ACC_PREVIEW_LEAD_MS) ?
        input->acceleration_next_mm_s2 :
        input->acceleration_now_mm_s2;
}

static int32_t round_float_to_i32(float value)
{
    return (int32_t)(value + ((value >= 0.0F) ? 0.5F : -0.5F));
}

static bool select_level_tuning(
    const APP_Gateway_ControlInput *input)
{
    const bool task3 = (input != NULL) &&
        (input->competition_mode == GATEWAY_TASK3_COMPETITION_MODE);

    return Algorithm_LevelCurrentControl_SetTuning(
        &s_level_controller,
        task3 ? GATEWAY_TASK3_LEVEL_KP_A_PER_DEG :
            GATEWAY_LEVEL_KP_A_PER_DEG,
        task3 ? GATEWAY_TASK3_LEVEL_KI_A_PER_DEG_S :
            GATEWAY_LEVEL_KI_A_PER_DEG_S,
        task3 ? GATEWAY_TASK3_LEVEL_KD_A_PER_RPM :
            GATEWAY_LEVEL_KD_A_PER_RPM,
        task3 ? GATEWAY_TASK3_LEVEL_FEEDFORWARD_CURRENT_A :
            GATEWAY_LEVEL_FEEDFORWARD_CURRENT_A,
        task3 ? GATEWAY_TASK3_LEVEL_DEADBAND_DEG :
            GATEWAY_LEVEL_DEADBAND_DEG,
        task3 ? GATEWAY_TASK3_LEVEL_INTEGRAL_LIMIT_A :
            GATEWAY_LEVEL_INTEGRAL_LIMIT_A);
}

static void update_chassis_acceleration(
    uint32_t now_ms,
    const APP_Gateway_ControlInput *input)
{
    Algorithm_ChassisAccelerationFusion_State state;
    const bool task4_plan_compensation = (input != NULL) &&
        (input->competition_mode == GATEWAY_TASK4_COMPETITION_MODE);
    const bool simple_lap_compensation = (input != NULL) &&
        (input->competition_mode >=
            GATEWAY_LAP_COMPENSATION_COMPETITION_MODE_MIN) &&
        (input->competition_mode <= GATEWAY_TASK6_COMPETITION_MODE);
    const bool simple_plan_compensation =
        task4_plan_compensation || simple_lap_compensation;

    s_chassis_acceleration_valid =
        (input != NULL) && input->chassis_acceleration_valid;
    s_planned_chassis_acceleration_mm_s2 = select_chassis_acceleration(
        now_ms, input);
    s_selected_chassis_acceleration_mm_s2 =
        s_planned_chassis_acceleration_mm_s2;
    s_chassis_fusion_trusted = false;

    if (GATEWAY_CHASSIS_ACCEL_FUSION_ENABLE == 0U) {
        return;
    }
    if (!s_chassis_acceleration_valid || (input == NULL) ||
        !input->chassis_acceleration_plan_valid) {
        Algorithm_ChassisAccelerationFusion_Reset(&s_chassis_fusion, now_ms);
        s_last_chassis_imu_sequence_seen = false;
        s_last_chassis_wheel_sequence_seen = false;
        s_last_chassis_wheel_acceleration_sequence_seen = false;
        s_simple_plan_compensation_active = false;
        return;
    }
    if (simple_plan_compensation) {
        const bool chassis_stationary =
            (s_planned_chassis_acceleration_mm_s2 == 0) &&
            input->chassis_wheel_measurement_valid &&
            (fabsf((float)input->measured_chassis_wheel_speed_mm_s) <=
                GATEWAY_CHASSIS_STATIONARY_WHEEL_SPEED_MM_S);
        float plan_weight = fabsf(
            (float)s_planned_chassis_acceleration_mm_s2) /
            GATEWAY_CHASSIS_PLAN_DOMINANT_ACCEL_MM_S2;
        float selected = (float)s_planned_chassis_acceleration_mm_s2;

        plan_weight = clamp_float(plan_weight, 0.0F, 1.0F);
        if (task4_plan_compensation) {
            /* A->B is deterministic: keep IMU/Kalman delay and wheel noise
             * out of the zero-plan cruise and parking phases. */
            selected = (float)s_planned_chassis_acceleration_mm_s2;
        }
        else if (chassis_stationary) {
            selected = 0.0F;
        }
        else if (input->chassis_wheel_acceleration_measurement_valid) {
            selected += GATEWAY_CHASSIS_WHEEL_ACCEL_CRUISE_GAIN *
                (1.0F - plan_weight) *
                (float)input->measured_chassis_wheel_acceleration_mm_s2;
        }
        s_selected_chassis_acceleration_mm_s2 = round_float_to_i32(selected);
        if (!s_simple_plan_compensation_active) {
            Algorithm_ChassisAccelerationFusion_Reset(
                &s_chassis_fusion, now_ms);
            s_last_chassis_imu_sequence_seen = false;
            s_last_chassis_wheel_sequence_seen = false;
            s_last_chassis_wheel_acceleration_sequence_seen = false;
            s_simple_plan_compensation_active = true;
        }
        return;
    }
    if (s_simple_plan_compensation_active) {
        Algorithm_ChassisAccelerationFusion_Reset(&s_chassis_fusion, now_ms);
        s_last_chassis_imu_sequence_seen = false;
        s_last_chassis_wheel_sequence_seen = false;
        s_last_chassis_wheel_acceleration_sequence_seen = false;
        s_simple_plan_compensation_active = false;
    }
    if (!Algorithm_ChassisAccelerationFusion_Predict(
            &s_chassis_fusion,
            now_ms,
            (float)s_planned_chassis_acceleration_mm_s2)) {
        s_last_chassis_imu_sequence_seen = false;
        s_last_chassis_wheel_sequence_seen = false;
        s_last_chassis_wheel_acceleration_sequence_seen = false;
        return;
    }
    if ((input != NULL) && input->chassis_motion_measurement_valid &&
        input->measured_chassis_acceleration_valid &&
        (!s_last_chassis_imu_sequence_seen ||
         (input->chassis_motion_sequence !=
            s_last_chassis_imu_sequence) ||
         (input->chassis_motion_sample_ms !=
            s_last_chassis_imu_sample_ms))) {
        (void)Algorithm_ChassisAccelerationFusion_Correct(
            &s_chassis_fusion,
            now_ms,
            false,
            0.0F,
            true,
            (float)input->measured_chassis_acceleration_mm_s2);
        s_last_chassis_imu_sequence = input->chassis_motion_sequence;
        s_last_chassis_imu_sample_ms = input->chassis_motion_sample_ms;
        s_last_chassis_imu_sequence_seen = true;
    }
    if ((input != NULL) && input->chassis_wheel_measurement_valid &&
        (!s_last_chassis_wheel_sequence_seen ||
         (input->chassis_wheel_sequence !=
            s_last_chassis_wheel_sequence) ||
         (input->chassis_wheel_sample_ms !=
            s_last_chassis_wheel_sample_ms))) {
        (void)Algorithm_ChassisAccelerationFusion_Correct(
            &s_chassis_fusion,
            now_ms,
            true,
            (float)input->measured_chassis_wheel_speed_mm_s,
            false,
            0.0F);
        s_last_chassis_wheel_sequence = input->chassis_wheel_sequence;
        s_last_chassis_wheel_sample_ms = input->chassis_wheel_sample_ms;
        s_last_chassis_wheel_sequence_seen = true;
    }
    if ((input != NULL) &&
        input->chassis_wheel_acceleration_measurement_valid &&
        (!s_last_chassis_wheel_acceleration_sequence_seen ||
         (input->chassis_wheel_acceleration_sequence !=
            s_last_chassis_wheel_acceleration_sequence) ||
         (input->chassis_wheel_acceleration_sample_ms !=
            s_last_chassis_wheel_acceleration_sample_ms))) {
        (void)Algorithm_ChassisAccelerationFusion_CorrectWheelAcceleration(
            &s_chassis_fusion,
            now_ms,
            (float)input->measured_chassis_wheel_acceleration_mm_s2);
        s_last_chassis_wheel_acceleration_sequence =
            input->chassis_wheel_acceleration_sequence;
        s_last_chassis_wheel_acceleration_sample_ms =
            input->chassis_wheel_acceleration_sample_ms;
        s_last_chassis_wheel_acceleration_sequence_seen = true;
    }
    if (Algorithm_ChassisAccelerationFusion_IsTrusted(
            &s_chassis_fusion, now_ms) &&
        Algorithm_ChassisAccelerationFusion_GetState(
            &s_chassis_fusion, &state, NULL)) {
        s_selected_chassis_acceleration_mm_s2 =
            round_float_to_i32(state.acceleration_mm_s2);
        s_chassis_fusion_trusted = true;
    }
}

static bool get_estimator_rod_angle_deg(
    uint32_t now_ms,
    float *rod_angle_deg)
{
    Device_QD4310_Feedback feedback;

    return (rod_angle_deg != NULL) &&
        Device_QD4310_GetFeedback(&s_motors[0], now_ms, &feedback) &&
        feedback.online &&
        ((uint32_t)(now_ms - feedback.received_at_ms) <
            GATEWAY_LEVEL_FEEDBACK_TIMEOUT_MS) &&
        Algorithm_RodAngleMap_RawToRodDeg(
            &s_rod_angle_map,
            (float)feedback.single_turn_angle_raw,
            rod_angle_deg);
}

static void reset_estimated_ball_feedback(uint32_t now_ms)
{
    if (s_ball_model_interpolator.state_valid) {
        Algorithm_BallModelInterpolator_Reset(
            &s_ball_model_interpolator, now_ms);
    }
    if (s_ball_estimator.state_valid ||
        (s_ball_estimator.history_count > 0U)) {
        Algorithm_BallStateEstimator_Reset(&s_ball_estimator, now_ms);
    }
    Algorithm_BallPositionControl_ResetMeasurement(&s_ball_controller);
    s_last_ball_sequence_seen = false;
}

static bool update_model_interpolated_ball_feedback(
    uint32_t now_ms,
    const APP_Gateway_ControlInput *input,
    float *ball_angle_deg)
{
    Algorithm_BallModelInterpolator_State state;
    float rod_angle_deg;

    if (!get_estimator_rod_angle_deg(now_ms, &rod_angle_deg)) {
        reset_estimated_ball_feedback(now_ms);
        return true;
    }
    /* All ball-loop angles are offsets from the calibrated horizontal trim.
     * Remove that static trim before applying the physical rolling model. */
    rod_angle_deg -= active_horizontal_rod_deg(input);
    if (s_ball_model_interpolator.state_valid &&
        ((uint32_t)(now_ms -
            s_ball_model_interpolator.last_accepted_measurement_time_ms) >
            GATEWAY_BALL_MODEL_MAX_COAST_MS)) {
        Algorithm_BallModelInterpolator_Reset(
            &s_ball_model_interpolator, now_ms);
        Algorithm_BallPositionControl_ResetMeasurement(&s_ball_controller);
    }
    if (!Algorithm_BallModelInterpolator_Predict(
            &s_ball_model_interpolator,
            now_ms,
            rod_angle_deg,
            (float)s_selected_chassis_acceleration_mm_s2)) {
        Algorithm_BallPositionControl_ResetMeasurement(&s_ball_controller);
        return true;
    }
    if ((input != NULL) && input->ball_measurement_valid &&
        (!s_last_ball_sequence_seen ||
         (input->ball_sequence != s_last_ball_sequence))) {
        const float physical_position_mm =
            GATEWAY_BALL_MEASUREMENT_POLARITY *
            (float)input->ball_position_mm;

        if (!Algorithm_BallPositionControl_UpdateVisualVelocity(
                &s_ball_controller,
                input->ball_sample_ms,
                (float)input->ball_position_mm)) {
            reset_estimated_ball_feedback(now_ms);
            return true;
        }
        if (!Algorithm_BallModelInterpolator_CorrectPosition(
                &s_ball_model_interpolator,
                now_ms,
                input->ball_measurement_age_ms,
                physical_position_mm)) {
            reset_estimated_ball_feedback(now_ms);
            return true;
        }
        s_last_ball_sequence = input->ball_sequence;
        s_last_ball_sequence_seen = true;
    }
    if (!Algorithm_BallModelInterpolator_IsTrusted(
            &s_ball_model_interpolator, now_ms) ||
        !Algorithm_BallModelInterpolator_GetState(
            &s_ball_model_interpolator, &state)) {
        Algorithm_BallPositionControl_ResetMeasurement(&s_ball_controller);
        return true;
    }
    if (!Algorithm_BallPositionControl_SetEstimatedPosition(
            &s_ball_controller,
            now_ms,
            state.position_mm)) {
        reset_estimated_ball_feedback(now_ms);
        return false;
    }
    return Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &s_ball_controller, ball_angle_deg);
}

static bool update_kalman_ball_feedback(
    uint32_t now_ms,
    const APP_Gateway_ControlInput *input,
    float *ball_angle_deg)
{
    Algorithm_BallStateEstimator_State state;
    float rod_angle_deg;

    if (!get_estimator_rod_angle_deg(now_ms, &rod_angle_deg)) {
        reset_estimated_ball_feedback(now_ms);
        return true;
    }
    if (!Algorithm_BallStateEstimator_Predict(
            &s_ball_estimator,
            now_ms,
            rod_angle_deg,
            (float)s_selected_chassis_acceleration_mm_s2)) {
        Algorithm_BallPositionControl_ResetMeasurement(&s_ball_controller);
        s_last_ball_sequence_seen = false;
        return true;
    }
    if ((input != NULL) && input->ball_measurement_valid &&
        (!s_last_ball_sequence_seen ||
         (input->ball_sequence != s_last_ball_sequence))) {
        if (!Algorithm_BallStateEstimator_CorrectPosition(
                &s_ball_estimator,
                now_ms,
                input->ball_measurement_age_ms,
                (float)input->ball_position_mm)) {
            reset_estimated_ball_feedback(now_ms);
            return true;
        }
        s_last_ball_sequence = input->ball_sequence;
        s_last_ball_sequence_seen = true;
    }
    if (!Algorithm_BallStateEstimator_IsTrusted(&s_ball_estimator, now_ms) ||
        !Algorithm_BallStateEstimator_GetState(
            &s_ball_estimator, &state, NULL)) {
        Algorithm_BallPositionControl_ResetMeasurement(&s_ball_controller);
        return true;
    }
    if (!Algorithm_BallPositionControl_SetEstimatedState(
            &s_ball_controller,
            now_ms,
            state.position_mm,
            state.velocity_mm_s)) {
        reset_estimated_ball_feedback(now_ms);
        return false;
    }
    return Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &s_ball_controller, ball_angle_deg);
}

static bool task3_target_matches(float target_mm, float expected_mm)
{
    return fabsf(target_mm - expected_mm) <=
        GATEWAY_TASK3_TARGET_MATCH_TOLERANCE_MM;
}

static bool update_direct_visual_position_feedback(
    const APP_Gateway_ControlInput *input,
    float *ball_angle_deg)
{
    const bool new_measurement = (input != NULL) &&
        (!s_last_ball_sequence_seen ||
         (input->ball_sequence != s_last_ball_sequence));

    if ((input == NULL) || (ball_angle_deg == NULL)) {
        return false;
    }
    *ball_angle_deg = 0.0F;
    if (!s_ball_measurement_valid) {
        if (s_ball_controller.measurement_seen) {
            Algorithm_BallPositionControl_ResetMeasurement(
                &s_ball_controller);
        }
        s_last_ball_sequence_seen = false;
        return true;
    }
    if (new_measurement) {
        if (!Algorithm_BallPositionControl_UpdateMeasurement(
                &s_ball_controller,
                input->ball_sample_ms,
                (float)input->ball_position_mm)) {
            return false;
        }
        s_last_ball_sequence = input->ball_sequence;
        s_last_ball_sequence_seen = true;
    }
    if (!s_ball_controller.measurement_seen) {
        return true;
    }
    return Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &s_ball_controller, ball_angle_deg);
}

static bool update_task3_rod_target(
    uint32_t now_ms,
    const APP_Gateway_ControlInput *input,
    float *ball_angle_deg)
{
    Task3_RodSequence_Output sequence_output;
    bool new_measurement = false;
    const bool center_target = task3_target_matches(
        s_ball_controller.target_command_mm, GATEWAY_BALL_TARGET_MM);
    const bool outbound_target = task3_target_matches(
        s_ball_controller.target_command_mm,
        GATEWAY_TASK3_OUTBOUND_TARGET_MM);
    const bool return_target = task3_target_matches(
        s_ball_controller.target_command_mm,
        GATEWAY_TASK3_RETURN_TARGET_MM);

    if ((input == NULL) || (ball_angle_deg == NULL)) {
        return false;
    }
    *ball_angle_deg = 0.0F;
    if (s_level_phase != GATEWAY_LEVEL_PHASE_ACCELERATION_COMPENSATION) {
        Task3_RodSequence_Reset(&s_task3_rod_sequence, now_ms);
        return true;
    }

    if (s_ball_measurement_valid) {
        new_measurement = !s_last_ball_sequence_seen ||
            (input->ball_sequence != s_last_ball_sequence);
        if (new_measurement) {
            if (!Algorithm_BallPositionControl_UpdateMeasurement(
                    &s_ball_controller,
                    input->ball_sample_ms,
                    (float)input->ball_position_mm)) {
                return false;
            }
            s_last_ball_sequence = input->ball_sequence;
            s_last_ball_sequence_seen = true;
        }
    }
    else if (s_ball_controller.measurement_seen) {
        Algorithm_BallPositionControl_ResetMeasurement(&s_ball_controller);
        s_last_ball_sequence_seen = false;
    }

    if (center_target &&
        (s_task3_rod_sequence.state !=
            TASK3_ROD_SEQUENCE_WAIT_ORIGIN)) {
        Task3_RodSequence_Reset(&s_task3_rod_sequence, now_ms);
    }
    if (!Task3_RodSequence_Step(
            &s_task3_rod_sequence,
            now_ms,
            outbound_target,
            return_target,
            s_ball_measurement_valid,
            new_measurement,
            (float)input->ball_position_mm,
            &sequence_output)) {
        return false;
    }
    if (sequence_output.entered_position_return &&
        (!Algorithm_BallPositionControl_ResetTargetPositionMm(
                &s_ball_controller, GATEWAY_TASK3_RETURN_TARGET_MM) ||
         !Algorithm_BallPositionControl_RearmPositionLoop(
                &s_ball_controller))) {
        return false;
    }
    if ((sequence_output.mode ==
            TASK3_ROD_SEQUENCE_OUTPUT_POSITION_RETURN) ||
        ((sequence_output.mode == TASK3_ROD_SEQUENCE_OUTPUT_LEVEL) &&
         !outbound_target)) {
        if (!s_ball_controller.measurement_seen) {
            return true;
        }
        return Algorithm_BallPositionControl_ComputeRodAngleWithPdGainsDeg(
            &s_ball_controller,
            GATEWAY_TASK3_BALL_POSITION_KP_PER_S2,
            GATEWAY_TASK3_BALL_VELOCITY_KD_PER_S,
            ball_angle_deg);
    }
    *ball_angle_deg = sequence_output.rod_offset_deg;
    return true;
}

static bool update_primary_tracking_target(
    uint32_t now_ms,
    const APP_Gateway_ControlInput *input)
{
    float ball_angle_deg = 0.0F;
    float chassis_angle_deg = 0.0F;
    float limited_ball_angle_deg;
    float available_chassis_min_deg;
    float available_chassis_max_deg;
    float requested_offset_deg;
    float limited_offset_deg;
    bool total_angle_saturated;
    bool target_applied;
    const bool task3 = (input != NULL) &&
        (input->competition_mode == GATEWAY_TASK3_COMPETITION_MODE);
    const bool task4 = (input != NULL) &&
        (input->competition_mode == GATEWAY_TASK4_COMPETITION_MODE);
    const bool task6 = (input != NULL) &&
        (input->competition_mode == GATEWAY_TASK6_COMPETITION_MODE);
    const uint8_t direct_visual_mode = task3 ?
        GATEWAY_TASK3_COMPETITION_MODE :
        (task6 ? GATEWAY_TASK6_COMPETITION_MODE : UINT8_C(0));
    const float horizontal_rod_deg = active_horizontal_rod_deg(input);

    /* Task3 owns its two open-loop reference-video legs, then reuses the
     * launch-only speed PI if its final position return stalls. Task4 disables
     * static-friction assist while the chassis moves. Task6 keeps direct
     * visual speed PI active until the final handoff band. */
    if (!Algorithm_BallPositionControl_SetSpeedHandoffEnabled(
            &s_ball_controller, !task4)) {
        return false;
    }
    if (!Algorithm_BallPositionControl_SetSpeedHandoffMotionExitEnabled(
            &s_ball_controller, !task6)) {
        return false;
    }
    if (!Algorithm_BallPositionControl_SetMotionLimits(
            &s_ball_controller,
            task6 ? GATEWAY_TASK6_BALL_MAX_TARGET_VELOCITY_MM_S :
                GATEWAY_BALL_MAX_TARGET_VELOCITY_MM_S,
            task6 ? GATEWAY_TASK6_BALL_MAX_TARGET_ACCELERATION_MM_S2 :
                GATEWAY_BALL_MAX_TARGET_ACCELERATION_MM_S2)) {
        return false;
    }
    if (!Algorithm_BallPositionControl_SetSpeedTargetBrakingAcceleration(
            &s_ball_controller,
            task6 ? GATEWAY_TASK6_BALL_BRAKING_ACCELERATION_MM_S2 :
                GATEWAY_BALL_MAX_TARGET_ACCELERATION_MM_S2)) {
        return false;
    }
    if (!Algorithm_BallPositionControl_SetSpeedLaunchMinimumAngleDeg(
            &s_ball_controller,
            task6 ? GATEWAY_TASK6_BALL_LAUNCH_MINIMUM_ANGLE_DEG : 0.0F)) {
        return false;
    }
    if (!Algorithm_BallPositionControl_SetFinalPositionBands(
            &s_ball_controller,
            task6 ? GATEWAY_TASK6_BALL_SPEED_HANDOFF_ERROR_MM :
                GATEWAY_BALL_STICTION_MIN_ERROR_MM,
            GATEWAY_BALL_HOLD_POSITION_ERROR_MM)) {
        return false;
    }
    if (s_direct_visual_competition_mode != direct_visual_mode) {
        reset_estimated_ball_feedback(now_ms);
        Task3_RodSequence_Reset(&s_task3_rod_sequence, now_ms);
        if (task3 &&
            !Algorithm_BallPositionControl_ResetTargetPositionMm(
                &s_ball_controller, GATEWAY_BALL_TARGET_MM)) {
            return false;
        }
        s_direct_visual_competition_mode = direct_visual_mode;
    }
    s_chassis_compensation_angle_deg = 0.0F;
    if (s_task6_level_calibration_phase !=
            GATEWAY_TASK6_LEVEL_CALIBRATION_NORMAL) {
        s_chassis_acceleration_valid = false;
        s_selected_chassis_acceleration_mm_s2 = 0;
        s_ball_measurement_valid = false;
        return true;
    }
    if (s_level_calibration_active) {
        s_chassis_acceleration_valid = false;
        s_selected_chassis_acceleration_mm_s2 = 0;
        s_ball_measurement_valid = false;
        return true;
    }
    if (s_level_phase == GATEWAY_LEVEL_PHASE_ZEROING) {
        if (task3) {
            Task3_RodSequence_Reset(&s_task3_rod_sequence, now_ms);
        }
        s_chassis_acceleration_valid = false;
        s_selected_chassis_acceleration_mm_s2 = 0;
        return apply_rod_target_deg(
            (task6 && s_task6_level_zero_valid) ?
                horizontal_rod_deg : 0.0F);
    }
    update_chassis_acceleration(now_ms, input);
    if (s_level_phase !=
            GATEWAY_LEVEL_PHASE_ACCELERATION_COMPENSATION) {
        s_chassis_acceleration_valid = false;
        s_selected_chassis_acceleration_mm_s2 = 0;
    }
    s_ball_measurement_valid =
        (GATEWAY_BALL_VISUAL_CONTROL_ENABLE != 0U) &&
        (input != NULL) && input->ball_measurement_valid;
    if (GATEWAY_BALL_VISUAL_CONTROL_ENABLE == 0U) {
        reset_estimated_ball_feedback(now_ms);
    }
    else if (task3) {
        if (!update_task3_rod_target(now_ms, input, &ball_angle_deg)) {
            return false;
        }
    }
    else if (task6) {
        if (!update_direct_visual_position_feedback(
                input, &ball_angle_deg)) {
            return false;
        }
    }
    else if (GATEWAY_BALL_MODEL_INTERPOLATION_ENABLE != 0U) {
        if (!update_model_interpolated_ball_feedback(
                now_ms, input, &ball_angle_deg)) {
            return false;
        }
        s_ball_measurement_valid =
            Algorithm_BallModelInterpolator_IsTrusted(
                &s_ball_model_interpolator, now_ms);
    }
    else if (GATEWAY_BALL_KALMAN_ENABLE != 0U) {
        if (!update_kalman_ball_feedback(
                now_ms, input, &ball_angle_deg)) {
            return false;
        }
        s_ball_measurement_valid =
            Algorithm_BallStateEstimator_IsTrusted(
                &s_ball_estimator, now_ms);
    }
    else if (!update_direct_visual_position_feedback(
            input, &ball_angle_deg)) {
        return false;
    }

    if (s_chassis_acceleration_valid) {
        chassis_angle_deg = (task4 ? GATEWAY_TASK4_CHASSIS_FF_GAIN :
            GATEWAY_CHASSIS_FF_GAIN) *
            GATEWAY_CHASSIS_FF_POLARITY *
            atan2f(
                (float)s_selected_chassis_acceleration_mm_s2,
            GATEWAY_BALL_GRAVITY_MM_S2) *
            (180.0F / 3.14159265358979323846F);
        chassis_angle_deg = clamp_float(
            chassis_angle_deg,
            -GATEWAY_CHASSIS_FF_ANGLE_LIMIT_DEG,
            GATEWAY_CHASSIS_FF_ANGLE_LIMIT_DEG);
    }
    if ((input != NULL) && input->chassis_acceleration_plan_valid &&
        (s_planned_chassis_acceleration_mm_s2 < 0)) {
        /* The heartbeat-confirmed M4 deceleration plan supplies a small,
         * deterministic stopping baseline. Fusion and vision retain the
         * remaining correction authority; positive acceleration is unchanged. */
        chassis_angle_deg += GATEWAY_CHASSIS_STOP_PLAN_FF_GAIN *
            GATEWAY_CHASSIS_FF_POLARITY *
            atan2f(
                (float)s_planned_chassis_acceleration_mm_s2,
                GATEWAY_BALL_GRAVITY_MM_S2) *
            (180.0F / 3.14159265358979323846F);
        chassis_angle_deg = clamp_float(
            chassis_angle_deg,
            -GATEWAY_CHASSIS_FF_ANGLE_LIMIT_DEG,
            GATEWAY_CHASSIS_FF_ANGLE_LIMIT_DEG);
    }
    /* Translation feedforward is physically position-independent. For M6,
     * preserve the arbitrary-target ball loop first and let chassis damping
     * use only the rod-angle authority that remains. This avoids an unscaled
     * acceleration term taking control away near either end of the rod. */
    limited_ball_angle_deg = clamp_float(
        ball_angle_deg,
        GATEWAY_TOTAL_ROD_COMMAND_MIN_DEG,
        GATEWAY_TOTAL_ROD_COMMAND_MAX_DEG);
    available_chassis_min_deg =
        GATEWAY_TOTAL_ROD_COMMAND_MIN_DEG - limited_ball_angle_deg;
    available_chassis_max_deg =
        GATEWAY_TOTAL_ROD_COMMAND_MAX_DEG - limited_ball_angle_deg;
    s_chassis_compensation_angle_deg = clamp_float(
        chassis_angle_deg,
        available_chassis_min_deg,
        available_chassis_max_deg);
    requested_offset_deg = limited_ball_angle_deg +
        s_chassis_compensation_angle_deg;
    limited_offset_deg = requested_offset_deg;
    total_angle_saturated =
        (limited_ball_angle_deg != ball_angle_deg) ||
        (s_chassis_compensation_angle_deg != chassis_angle_deg);
    target_applied = apply_rod_target_deg(
        horizontal_rod_deg + limited_offset_deg);
    s_rod_angle_saturated =
        s_rod_angle_saturated || total_angle_saturated;
    return target_applied;
}

static Device_QD4310 *find_motor(uint8_t motor_id)
{
    for (size_t index = 0U; index < GATEWAY_MOTOR_COUNT; ++index) {
        if (s_motors[index].config.motor_id == motor_id) {
            return &s_motors[index];
        }
    }
    return NULL;
}

static bool stop_primary_level_control(void)
{
    Device_QD4310 *motor = &s_motors[0];
    bool zero_sent = true;
    bool disabled = false;

    s_level_phase = GATEWAY_LEVEL_PHASE_STOPPED;
    s_level_calibration_active = false;
    s_level_calibration_target_reached = false;
    s_level_calibration_stable_seen = false;
    s_zero_entry_stable_seen = false;
    s_level_enable_requested_ms = 0U;
    s_chassis_compensation_angle_deg = 0.0F;
    s_motor_command_current_a = 0.0F;
    reset_task6_level_calibration();
    if (motor->initialized) {
        if (motor->command_enabled) {
            zero_sent = Device_QD4310_SetCurrent(motor, 0.0F);
        }
        disabled = Device_QD4310_Disable(motor);
    }
    Algorithm_LevelCurrentControl_Stop(&s_level_controller);
    return zero_sent && disabled;
}

static bool apply_level_calibration_target(size_t index, uint32_t now_ms)
{
    const uint16_t target_raw = level_calibration_target_raw(index);

    if ((index >= level_calibration_total_point_count()) ||
        !Algorithm_LevelCurrentControl_SetTargetDeg(
            &s_level_controller,
            raw_target_to_relative_motor_deg((float)target_raw))) {
        return false;
    }
    s_level_calibration_index = index;
    s_motor_target_raw = target_raw;
    s_level_calibration_target_reached = false;
    s_level_calibration_stable_seen = false;
    s_level_calibration_point_started_ms = now_ms;
    s_level_phase = GATEWAY_LEVEL_PHASE_ZEROING;
    return true;
}

static bool update_level_calibration(
    uint32_t now_ms, const Device_QD4310_Feedback *feedback)
{
    const bool stable = (feedback != NULL) &&
        (raw_distance(
            feedback->single_turn_angle_raw, s_motor_target_raw) <=
            GATEWAY_LEVEL_CALIBRATION_STABLE_TOL_RAW) &&
        (fabsf(feedback->speed_rpm) <=
            GATEWAY_LEVEL_CALIBRATION_STABLE_MAX_RPM);

    if (!s_level_calibration_active) {
        return true;
    }
    if ((uint32_t)(now_ms - s_level_calibration_point_started_ms) >=
        GATEWAY_LEVEL_CALIBRATION_POINT_TIMEOUT_MS) {
        return false;
    }
    if (!stable) {
        s_level_calibration_stable_seen = false;
        s_level_phase = GATEWAY_LEVEL_PHASE_ZEROING;
        return true;
    }
    if (!s_level_calibration_stable_seen) {
        s_level_calibration_stable_seen = true;
        s_level_calibration_stable_since_ms = now_ms;
        return true;
    }
    if (s_level_phase == GATEWAY_LEVEL_PHASE_ZEROING) {
        if ((uint32_t)(now_ms - s_level_calibration_stable_since_ms) >=
            GATEWAY_LEVEL_CALIBRATION_STABLE_TIME_MS) {
            s_level_phase =
                GATEWAY_LEVEL_PHASE_ACCELERATION_COMPENSATION;
            s_level_calibration_hold_since_ms = now_ms;
        }
        return true;
    }
    if ((uint32_t)(now_ms - s_level_calibration_hold_since_ms) <
        GATEWAY_LEVEL_CALIBRATION_HOLD_TIME_MS) {
        return true;
    }
    if ((s_level_calibration_index + 1U) >=
        level_calibration_total_point_count()) {
        return stop_primary_level_control();
    }
    return apply_level_calibration_target(
        s_level_calibration_index + 1U, now_ms);
}

static void enforce_primary_motor_disabled_when_stopped(uint32_t now_ms)
{
    Device_QD4310 *motor = &s_motors[0];
    Device_QD4310_Feedback feedback;

    if (Algorithm_LevelCurrentControl_IsActive(&s_level_controller) ||
        !Device_QD4310_GetFeedback(motor, now_ms, &feedback) ||
        !feedback.online || !feedback.enabled) {
        return;
    }

    s_motor_command_current_a = 0.0F;
    if (motor->command_enabled) {
        (void)Device_QD4310_SetCurrent(motor, 0.0F);
    }
    (void)Device_QD4310_Disable(motor);
}

static void run_primary_level_control(uint32_t now_ms)
{
    Device_QD4310 *motor = &s_motors[0];
    Device_QD4310_Feedback feedback;
    float current_a;
    float relative_angle_deg;
    bool raw_recovery_active;

    if (!Algorithm_LevelCurrentControl_IsActive(&s_level_controller)) {
        return;
    }
    if (!Device_QD4310_GetFeedback(motor, now_ms, &feedback) ||
        !feedback.online ||
        ((uint32_t)(now_ms - feedback.received_at_ms) >=
            GATEWAY_LEVEL_FEEDBACK_TIMEOUT_MS)) {
        ++s_level_control_fault_count;
        (void)stop_primary_level_control();
        return;
    }
    if (feedback.sequence == s_last_control_feedback_sequence) {
        return;
    }
    s_last_control_feedback_sequence = feedback.sequence;
    if (s_task6_level_calibration_phase !=
            GATEWAY_TASK6_LEVEL_CALIBRATION_NORMAL) {
        if (!feedback.enabled) {
            ++s_level_control_fault_count;
            (void)stop_primary_level_control();
            return;
        }
        if (s_task6_level_calibration_phase ==
                GATEWAY_TASK6_LEVEL_CALIBRATION_ZERO_CURRENT) {
            s_motor_target_raw = feedback.single_turn_angle_raw;
        }
        else {
            s_motor_target_raw = s_task6_level_zero_raw;
        }
        s_chassis_acceleration_valid = false;
        s_selected_chassis_acceleration_mm_s2 = 0;
        s_chassis_compensation_angle_deg = 0.0F;
        s_motor_command_current_a = 0.0F;
        if (!Device_QD4310_SetCurrent(motor, 0.0F)) {
            ++s_level_control_fault_count;
            (void)stop_primary_level_control();
        }
        return;
    }
    if (s_level_calibration_active) {
        if (GATEWAY_LEVEL_CALIBRATION_MANUAL_MODE != 0U) {
            if (!feedback.enabled) {
                ++s_level_control_fault_count;
                (void)stop_primary_level_control();
                return;
            }
            s_motor_target_raw = feedback.single_turn_angle_raw;
            s_level_phase =
                ((uint32_t)(now_ms - s_level_calibration_point_started_ms) >=
                    GATEWAY_LEVEL_CALIBRATION_MANUAL_ARM_MS) ?
                GATEWAY_LEVEL_PHASE_ACCELERATION_COMPENSATION :
                GATEWAY_LEVEL_PHASE_ZEROING;
            s_motor_command_current_a = 0.0F;
            if (!Device_QD4310_SetCurrent(motor, 0.0F)) {
                ++s_level_control_fault_count;
                (void)stop_primary_level_control();
            }
            return;
        }
        const int32_t raw_error = raw_signed_error(
            s_motor_target_raw, feedback.single_turn_angle_raw);
        const uint16_t distance_raw = raw_distance(
            s_motor_target_raw, feedback.single_turn_angle_raw);
        const bool downward_leg = (s_level_calibration_index != 0U) &&
            (s_level_calibration_index <
                level_calibration_one_way_point_count());
        float control_current_a;

        current_a = 0.0F;
        if (!feedback.enabled) {
            ++s_level_control_fault_count;
            (void)stop_primary_level_control();
            return;
        }
        if (distance_raw <= GATEWAY_LEVEL_CALIBRATION_STABLE_TOL_RAW) {
            s_level_calibration_target_reached = true;
        }
        if (s_level_calibration_target_reached) {
            /* Closed-loop holding removes the old 0/0.20 A chatter at each
             * sample. The feedforward carries gravity while raw P and speed D
             * keep the actual sample close to its target. */
            control_current_a = GATEWAY_LEVEL_FEEDFORWARD_CURRENT_A +
                GATEWAY_LEVEL_CALIBRATION_HOLD_KP_A_PER_RAW *
                    (float)raw_error -
                GATEWAY_LEVEL_CALIBRATION_SPEED_KD_A_PER_RPM *
                    feedback.speed_rpm;
            control_current_a = clamp_float(
                control_current_a,
                -GATEWAY_LEVEL_CALIBRATION_MIN_CURRENT_A,
                GATEWAY_LEVEL_CALIBRATION_MIN_CURRENT_A);
        }
        else {
            const float approach_scale = clamp_float(
                (float)distance_raw /
                    (float)GATEWAY_LEVEL_CALIBRATION_SLOWDOWN_RAW,
                GATEWAY_LEVEL_CALIBRATION_APPROACH_MIN_CURRENT_A /
                    GATEWAY_LEVEL_CALIBRATION_MIN_CURRENT_A,
                1.0F);
            const float approach_current_a =
                GATEWAY_LEVEL_CALIBRATION_MIN_CURRENT_A * approach_scale;

            control_current_a = 0.0F;
            /* Raw decrease is upward. Index zero is upward-only; the first
             * scan leg is downward-only and the return leg is upward-only. */
            if (!downward_leg && (raw_error < 0)) {
                control_current_a = -approach_current_a -
                    GATEWAY_LEVEL_CALIBRATION_SPEED_KD_A_PER_RPM *
                        feedback.speed_rpm;
                if (control_current_a > 0.0F) {
                    control_current_a = 0.0F;
                }
            }
            else if (downward_leg && (raw_error > 0)) {
                control_current_a = approach_current_a -
                    GATEWAY_LEVEL_CALIBRATION_SPEED_KD_A_PER_RPM *
                        feedback.speed_rpm;
                if (control_current_a < 0.0F) {
                    control_current_a = 0.0F;
                }
            }
            control_current_a = clamp_float(
                control_current_a,
                -GATEWAY_LEVEL_CALIBRATION_MIN_CURRENT_A,
                GATEWAY_LEVEL_CALIBRATION_MIN_CURRENT_A);
        }
        current_a = control_current_a * GATEWAY_LEVEL_CURRENT_POLARITY;
        if (((feedback.single_turn_angle_raw <=
                    GATEWAY_LEVEL_RAW_MIN_LIMIT) &&
                (control_current_a < 0.0F)) ||
            ((feedback.single_turn_angle_raw >=
                    GATEWAY_LEVEL_RAW_MAX_LIMIT) &&
                (control_current_a > 0.0F))) {
            current_a = 0.0F;
            s_motor_raw_limit_blocked = true;
        }
        s_motor_command_current_a = current_a;
        if (!Device_QD4310_SetCurrent(motor, current_a) ||
            !update_level_calibration(now_ms, &feedback)) {
            ++s_level_control_fault_count;
            (void)stop_primary_level_control();
        }
        return;
    }
    if (!s_level_calibration_active &&
        (s_level_phase == GATEWAY_LEVEL_PHASE_ZEROING) &&
        Algorithm_LevelCurrentControl_GetRelativeAngleDeg(
            &s_level_controller,
            feedback.single_turn_angle_rad,
            &relative_angle_deg) &&
        (fabsf(relative_angle_deg) <=
            GATEWAY_LEVEL_ZERO_ENTRY_TOL_DEG) &&
        (fabsf(feedback.speed_rpm) <=
            GATEWAY_LEVEL_ZERO_ENTRY_MAX_RPM)) {
        if (!s_zero_entry_stable_seen) {
            s_zero_entry_stable_seen = true;
            s_zero_entry_stable_since_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - s_zero_entry_stable_since_ms) >=
            GATEWAY_LEVEL_ZERO_ENTRY_SETTLE_MS) {
            s_level_phase =
                GATEWAY_LEVEL_PHASE_ACCELERATION_COMPENSATION;
        }
    }
    else {
        s_zero_entry_stable_seen = false;
    }
    if (!feedback.enabled) {
        if (motor->command_enabled &&
            ((uint32_t)(now_ms - s_level_enable_requested_ms) <
                GATEWAY_LEVEL_ENABLE_CONFIRM_TIMEOUT_MS)) {
            s_motor_command_current_a = 0.0F;
            if (!Device_QD4310_RequestFeedback(motor)) {
                ++s_level_control_fault_count;
                (void)stop_primary_level_control();
            }
            return;
        }
        ++s_level_control_fault_count;
        (void)stop_primary_level_control();
        return;
    }
    if (!Algorithm_LevelCurrentControl_ComputeRawRecoveryCurrent(
            &s_level_controller,
            feedback.single_turn_angle_raw,
            &current_a,
            &raw_recovery_active) ||
        (!raw_recovery_active &&
         !Algorithm_LevelCurrentControl_Update(
             &s_level_controller,
             feedback.single_turn_angle_rad,
             feedback.speed_rpm,
             &current_a))) {
        ++s_level_control_fault_count;
        (void)stop_primary_level_control();
        return;
    }
    if (raw_recovery_active) {
        s_motor_raw_limit_blocked = true;
    }
    if (((feedback.single_turn_angle_raw <=
                GATEWAY_LEVEL_RAW_MIN_LIMIT) &&
            ((current_a * GATEWAY_LEVEL_CURRENT_POLARITY) < 0.0F)) ||
        ((feedback.single_turn_angle_raw >=
                GATEWAY_LEVEL_RAW_MAX_LIMIT) &&
            ((current_a * GATEWAY_LEVEL_CURRENT_POLARITY) > 0.0F))) {
        current_a = 0.0F;
        s_motor_raw_limit_blocked = true;
    }
    s_motor_command_current_a = current_a;
    if (!Device_QD4310_SetCurrent(motor, current_a) ||
        !update_level_calibration(now_ms, &feedback)) {
        ++s_level_control_fault_count;
        (void)stop_primary_level_control();
    }
}

/* 简易 ASCII 浮点解析（正负号+小数），避免拉入 strtod 的 locale 开销。 */
static bool parse_float(const char *text, size_t length, float *value)
{
    float result = 0.0F;
    float fraction = 0.1F;
    bool negative = false;
    bool in_fraction = false;
    bool any_digit = false;
    size_t index = 0U;

    if ((text == NULL) || (value == NULL) || (length == 0U)) {
        return false;
    }
    if ((text[0] == '-') || (text[0] == '+')) {
        negative = (text[0] == '-');
        index = 1U;
    }
    for (; index < length; ++index) {
        const char c = text[index];

        if ((c >= '0') && (c <= '9')) {
            if (in_fraction) {
                result += (float)(c - '0') * fraction;
                fraction *= 0.1F;
            }
            else {
                result = result * 10.0F + (float)(c - '0');
            }
            any_digit = true;
        }
        else if ((c == '.') && !in_fraction) {
            in_fraction = true;
        }
        else {
            return false;
        }
    }
    if (!any_digit) {
        return false;
    }
    *value = negative ? -result : result;
    return true;
}

static bool send_packet(
    const char *type, const uint8_t *payload, size_t payload_length)
{
    uint8_t frame[ALGORITHM_PACKET_MAX_FRAME_SIZE];
    size_t frame_length = 0U;

    if (Algorithm_Packet_Encode(
            type, s_tx_sequence, payload, payload_length,
            frame, sizeof(frame), &frame_length) !=
        ALGORITHM_PACKET_ENCODE_OK) {
        return false;
    }
    if (!esp8266_send(&s_wireless, frame, frame_length)) {
        return false;
    }
    ++s_tx_sequence;
    return true;
}

static void send_ack(uint16_t sequence, bool ok)
{
    char payload[16];
    size_t length = 0U;
    uint16_t value = sequence;
    char reverse[6];
    size_t count = 0U;

    do {
        reverse[count++] = (char)('0' + (value % 10U));
        value = (uint16_t)(value / 10U);
    } while (value > 0U);
    while (count > 0U) {
        payload[length++] = reverse[--count];
    }
    payload[length++] = ',';
    if (ok) {
        memcpy(&payload[length], "OK", 2U);
        length += 2U;
    }
    else {
        memcpy(&payload[length], "ERR", 3U);
        length += 3U;
    }
    (void)send_packet("ACK", (const uint8_t *)payload, length);
}

/* 把浮点格式化为 "-x.yyy"（3 位小数），返回写入长度。 */
static size_t format_float(char *out, float value)
{
    size_t length = 0U;
    int32_t scaled;
    uint32_t magnitude;
    uint32_t integer_part;
    uint32_t fraction_part;
    char reverse[10];
    size_t count = 0U;

    if (value < 0.0F) {
        out[length++] = '-';
        value = -value;
    }
    /* 限幅避免溢出：网关内的物理量都远小于 1e6。 */
    if (value > 999999.0F) {
        value = 999999.0F;
    }
    scaled = (int32_t)(value * 1000.0F + 0.5F);
    magnitude = (uint32_t)scaled;
    integer_part = magnitude / 1000U;
    fraction_part = magnitude % 1000U;

    do {
        reverse[count++] = (char)('0' + (integer_part % 10U));
        integer_part /= 10U;
    } while (integer_part > 0U);
    while (count > 0U) {
        out[length++] = reverse[--count];
    }
    out[length++] = '.';
    out[length++] = (char)('0' + (fraction_part / 100U));
    out[length++] = (char)('0' + ((fraction_part / 10U) % 10U));
    out[length++] = (char)('0' + (fraction_part % 10U));
    return length;
}

static void send_feedback(size_t motor_index, uint32_t now_ms)
{
    Device_QD4310_Feedback feedback;
    char payload[96];
    size_t length = 0U;

    if (!Device_QD4310_GetFeedback(
            &s_motors[motor_index], now_ms, &feedback)) {
        return;
    }
    if (feedback.sequence == s_last_feedback_sequence[motor_index]) {
        return;  /* 无新反馈帧 */
    }

    payload[length++] =
        (char)('0' + s_motors[motor_index].config.motor_id);
    payload[length++] = ',';
    payload[length++] = feedback.enabled ? '1' : '0';
    payload[length++] = ',';
    length += format_float(&payload[length], feedback.current_a);
    payload[length++] = ',';
    length += format_float(&payload[length], feedback.speed_rpm);
    payload[length++] = ',';
    length += format_float(&payload[length], feedback.single_turn_angle_rad);
    payload[length++] = ',';
    payload[length++] = feedback.online ? '1' : '0';

    if (send_packet("FB", (const uint8_t *)payload, length)) {
        s_last_feedback_sequence[motor_index] = feedback.sequence;
    }
}

static void make_primary_telemetry(
    uint32_t now_ms, APP_Gateway_Telemetry *telemetry)
{
    Device_QD4310_Feedback feedback;
    Algorithm_BallModelInterpolator_Diagnostics model_diagnostics;
    Algorithm_BallStateEstimator_State estimator_state;
    Algorithm_BallStateEstimator_Covariance estimator_covariance;
    Algorithm_ChassisAccelerationFusion_State fusion_state;
    Algorithm_ChassisAccelerationFusion_Covariance fusion_covariance;
    const bool estimator_state_valid = Algorithm_BallStateEstimator_GetState(
        &s_ball_estimator, &estimator_state, &estimator_covariance);
    const bool model_diagnostics_valid =
        Algorithm_BallModelInterpolator_GetDiagnostics(
            &s_ball_model_interpolator, &model_diagnostics);
    const bool fusion_state_valid =
        Algorithm_ChassisAccelerationFusion_GetState(
            &s_chassis_fusion, &fusion_state, &fusion_covariance);

    if (telemetry == NULL) {
        return;
    }
    *telemetry = (APP_Gateway_Telemetry){
        .motor_id = s_motors[0].config.motor_id,
        .rod_command_deg = s_rod_command_deg,
        .target_single_turn_angle_raw = s_motor_target_raw,
        .command_current_a = s_motor_command_current_a,
        .ball_position_mm = s_ball_controller.position_mm,
        .ball_velocity_mm_s = s_ball_controller.velocity_mm_s,
        .ball_integral_acceleration_mm_s2 =
            s_ball_controller.integral_acceleration_mm_s2,
        .ball_stiction_kick_deg =
            s_ball_controller.stiction_kick_angle_deg,
        .ball_target_mm = s_ball_controller.target_command_mm,
        .ball_active_target_mm = s_ball_controller.target_position_mm,
        .ball_acceleration_bias_mm_s2 = estimator_state_valid ?
            estimator_state.acceleration_bias_mm_s2 : 0.0F,
        .ball_position_variance_mm2 = estimator_state_valid ?
            estimator_covariance.p00 : 0.0F,
        .ball_velocity_variance_mm2_s2 = estimator_state_valid ?
            estimator_covariance.p11 : 0.0F,
        .chassis_acceleration_mm_s2 =
            s_selected_chassis_acceleration_mm_s2,
        .planned_chassis_acceleration_mm_s2 =
            s_planned_chassis_acceleration_mm_s2,
        .chassis_velocity_estimate_mm_s = fusion_state_valid ?
            fusion_state.velocity_mm_s : 0.0F,
        .chassis_imu_bias_estimate_mm_s2 = fusion_state_valid ?
            fusion_state.imu_bias_mm_s2 : 0.0F,
        .chassis_acceleration_variance_mm2_s4 = fusion_state_valid ?
            fusion_covariance.p11 : 0.0F,
        .ball_measurement_valid = s_ball_measurement_valid,
        .ball_velocity_valid = s_ball_controller.velocity_valid,
        .ball_hold_active = s_ball_controller.hold_active,
        .ball_last_used_sequence_valid = s_last_ball_sequence_seen,
        .ball_estimator_trusted =
            ((GATEWAY_BALL_MODEL_INTERPOLATION_ENABLE != 0U) &&
             Algorithm_BallModelInterpolator_IsTrusted(
                &s_ball_model_interpolator, now_ms)) ||
            ((GATEWAY_BALL_KALMAN_ENABLE != 0U) &&
             Algorithm_BallStateEstimator_IsTrusted(
                &s_ball_estimator, now_ms)),
        .chassis_acceleration_valid = s_chassis_acceleration_valid,
        .chassis_acceleration_fusion_trusted = s_chassis_fusion_trusted,
        .rod_angle_saturated = s_rod_angle_saturated,
        .motor_raw_limit_blocked = s_motor_raw_limit_blocked,
        .level_control_phase = (uint8_t)s_level_phase,
        .ball_last_used_sequence = s_last_ball_sequence,
        .chassis_compensation_angle_deg =
            s_chassis_compensation_angle_deg,
        .ball_estimator_innovation_reject_count =
            (GATEWAY_BALL_MODEL_INTERPOLATION_ENABLE != 0U) &&
                model_diagnostics_valid ?
            model_diagnostics.innovation_reject_count :
            s_ball_estimator.innovation_reject_count,
        .chassis_fusion_innovation_reject_count =
            s_chassis_fusion.wheel_innovation_reject_count +
            s_chassis_fusion.wheel_acceleration_innovation_reject_count +
            s_chassis_fusion.imu_innovation_reject_count,
        .level_control_fault_count = s_level_control_fault_count,
    };
    if (!Device_QD4310_GetFeedback(
            &s_motors[0], now_ms, &feedback)) {
        return;
    }
    telemetry->enabled = feedback.enabled;
    telemetry->current_a = feedback.current_a;
    telemetry->speed_rpm = feedback.speed_rpm;
    telemetry->single_turn_angle_raw = feedback.single_turn_angle_raw;
    telemetry->single_turn_angle_rad = feedback.single_turn_angle_rad;
    telemetry->online = feedback.online;
}

/* 命令 payload："<motor_id>,<VERB>[,<value>]"，见头文件注释。 */
static bool execute_command(const Algorithm_Packet_View *view)
{
    char text[ALGORITHM_PACKET_MAX_PAYLOAD_LENGTH + 1U];
    char *verb;
    char *argument = NULL;
    char *comma;
    Device_QD4310 *motor;
    long motor_id;
    float value = 0.0F;

    if (view->payload_length > ALGORITHM_PACKET_MAX_PAYLOAD_LENGTH) {
        return false;
    }
    memcpy(text, view->payload, view->payload_length);
    text[view->payload_length] = '\0';

    comma = strchr(text, ',');
    if (comma == NULL) {
        return false;
    }
    *comma = '\0';
    verb = comma + 1;

    motor_id = strtol(text, NULL, 10);
    if ((motor_id < 0) || (motor_id > (long)DEVICE_QD4310_MOTOR_ID_MAX)) {
        return false;
    }
    motor = find_motor((uint8_t)motor_id);
    if (motor == NULL) {
        return false;
    }

    comma = strchr(verb, ',');
    if (comma != NULL) {
        *comma = '\0';
        argument = comma + 1;
    }

    /* The KEY-owned, raw-limited level controller is the sole owner of the
     * primary motor. TCP may only request feedback or force a safe stop; raw
     * ENABLE/CURRENT/SPEED commands must never bypass the absolute limits. */
    if (motor == &s_motors[0]) {
        if (strcmp(verb, "DISABLE") == 0) {
            return Algorithm_LevelCurrentControl_IsActive(
                    &s_level_controller) ?
                stop_primary_level_control() :
                Device_QD4310_Disable(motor);
        }
        if (strcmp(verb, "NOP") == 0) {
            return Device_QD4310_RequestFeedback(motor);
        }
        return false;
    }

    if (strcmp(verb, "ENABLE") == 0) {
        return Device_QD4310_Enable(motor);
    }
    if (strcmp(verb, "DISABLE") == 0) {
        return Device_QD4310_Disable(motor);
    }
    if (strcmp(verb, "NOP") == 0) {
        return Device_QD4310_RequestFeedback(motor);
    }
    if ((argument == NULL) ||
        !parse_float(argument, strlen(argument), &value)) {
        return false;
    }
    if (strcmp(verb, "CURRENT") == 0) {
        return Device_QD4310_SetCurrent(motor, value);
    }
    if (strcmp(verb, "SPEED") == 0) {
        return Device_QD4310_SetSpeed(motor, value);
    }
    if (strcmp(verb, "LOWSPEED") == 0) {
        return Device_QD4310_SetLowSpeed(motor, value);
    }
    return false;
}

bool APP_Gateway_Init(uint32_t now_ms)
{
    BSP_UART_Handle esp_uart = NULL;
    const Algorithm_LevelCurrentControl_Config level_config = {
        .control_period_s =
            (float)GATEWAY_LEVEL_CONTROL_PERIOD_MS / 1000.0F,
        .target_deg = 0.0F,
        .target_limit_deg = GATEWAY_LEVEL_TARGET_LIMIT_DEG,
        .hard_limit_deg = GATEWAY_LEVEL_HARD_LIMIT_DEG,
        .kp_a_per_deg = GATEWAY_LEVEL_KP_A_PER_DEG,
        .ki_a_per_deg_s = GATEWAY_LEVEL_KI_A_PER_DEG_S,
        .kd_a_per_rpm = GATEWAY_LEVEL_KD_A_PER_RPM,
        .feedforward_current_a =
            GATEWAY_LEVEL_FEEDFORWARD_CURRENT_A,
        .error_limit_deg = GATEWAY_LEVEL_ERROR_LIMIT_DEG,
        .deadband_deg = GATEWAY_LEVEL_DEADBAND_DEG,
        .integral_zone_deg = GATEWAY_LEVEL_INTEGRAL_ZONE_DEG,
        .integral_limit_a = GATEWAY_LEVEL_INTEGRAL_LIMIT_A,
        .current_limit_a = GATEWAY_LEVEL_CURRENT_LIMIT_A,
        .current_polarity = GATEWAY_LEVEL_CURRENT_POLARITY,
        .raw_min_limit = GATEWAY_LEVEL_RAW_MIN_LIMIT,
        .raw_max_limit = GATEWAY_LEVEL_RAW_CONTROL_MAX,
        .raw_recovery_current_a =
            GATEWAY_LEVEL_RAW_RECOVERY_CURRENT_A,
    };
    const Algorithm_BallPositionControl_Config ball_config = {
        .target_position_mm = GATEWAY_BALL_TARGET_MM,
        .maximum_absolute_target_position_mm =
            GATEWAY_BALL_TARGET_LIMIT_MM,
        .target_slew_rate_mm_s = GATEWAY_BALL_TARGET_SLEW_MM_S,
        .maximum_target_velocity_mm_s =
            GATEWAY_BALL_MAX_TARGET_VELOCITY_MM_S,
        .maximum_target_acceleration_mm_s2 =
            GATEWAY_BALL_MAX_TARGET_ACCELERATION_MM_S2,
        .measurement_polarity = GATEWAY_BALL_MEASUREMENT_POLARITY,
        .position_gain_per_s2 = GATEWAY_BALL_POSITION_KP_PER_S2,
        .position_integral_gain_per_s3 =
            GATEWAY_BALL_POSITION_KI_PER_S3,
        .velocity_gain_per_s = GATEWAY_BALL_VELOCITY_KD_PER_S,
        .maximum_integral_acceleration_mm_s2 =
            GATEWAY_BALL_INTEGRAL_ACCEL_LIMIT_MM_S2,
        .velocity_filter_tau_s = GATEWAY_BALL_VELOCITY_FILTER_TAU_S,
        .velocity_estimation_window_ms =
            GATEWAY_BALL_VELOCITY_ESTIMATION_WINDOW_MS,
        .hold_position_error_mm =
            GATEWAY_BALL_HOLD_POSITION_ERROR_MM,
        .hold_maximum_velocity_mm_s =
            GATEWAY_BALL_HOLD_MAX_VELOCITY_MM_S,
        .hold_stable_duration_ms =
            GATEWAY_BALL_HOLD_STABLE_DURATION_MS,
        .hold_exit_position_error_mm =
            GATEWAY_BALL_HOLD_EXIT_POSITION_ERROR_MM,
        .hold_exit_maximum_velocity_mm_s =
            GATEWAY_BALL_HOLD_EXIT_MAX_VELOCITY_MM_S,
        .hold_exit_sample_count =
            GATEWAY_BALL_HOLD_EXIT_SAMPLE_COUNT,
        .near_target_error_mm =
            GATEWAY_BALL_NEAR_TARGET_ERROR_MM,
        .near_target_velocity_gain_multiplier =
            GATEWAY_BALL_NEAR_TARGET_KD_MULTIPLIER,
        .gravity_mm_s2 = GATEWAY_BALL_GRAVITY_MM_S2,
        .inertia_ratio = GATEWAY_BALL_INERTIA_RATIO,
        .rod_angle_limit_deg = GATEWAY_BALL_ROD_ANGLE_LIMIT_DEG,
        .maximum_velocity_dt_ms = GATEWAY_BALL_MAX_VELOCITY_DT_MS,
        .stiction_kick_angle_deg =
            GATEWAY_BALL_STICTION_KICK_ANGLE_DEG,
        .stiction_kick_step_angle_deg =
            GATEWAY_BALL_STICTION_KICK_STEP_ANGLE_DEG,
        .stiction_kick_maximum_angle_deg =
            GATEWAY_BALL_STICTION_KICK_MAX_ANGLE_DEG,
        .stiction_assist_velocity_gain_per_s =
            GATEWAY_BALL_STICTION_ASSIST_VELOCITY_KP_PER_S,
        .stiction_assist_velocity_integral_gain_per_s2 =
            GATEWAY_BALL_STICTION_ASSIST_VELOCITY_KI_PER_S2,
        .stiction_assist_exit_distance_mm =
            GATEWAY_BALL_STICTION_ASSIST_EXIT_DISTANCE_MM,
        .stiction_kick_minimum_error_mm =
            GATEWAY_BALL_STICTION_MIN_ERROR_MM,
        .stiction_kick_maximum_velocity_mm_s =
            GATEWAY_BALL_STICTION_MAX_VELOCITY_MM_S,
        .stiction_kick_duration_ms =
            GATEWAY_BALL_STICTION_DURATION_MS,
    };
    const Task3_RodSequence_Config task3_rod_sequence_config = {
        .origin_tolerance_mm = GATEWAY_TASK3_ORIGIN_TOLERANCE_MM,
        .positive_crossing_mm = GATEWAY_TASK3_POSITIVE_CROSSING_MM,
        .negative_crossing_mm =
            GATEWAY_TASK3_POSITION_RETURN_ENTRY_MM,
        .raise_rate_deg_s = GATEWAY_TASK3_RAISE_RATE_DEG_S,
        .maximum_raise_offset_deg =
            GATEWAY_TASK3_MAX_RAISE_OFFSET_DEG,
        .initial_reverse_offset_deg =
            GATEWAY_TASK3_INITIAL_REVERSE_OFFSET_DEG,
        .reverse_rate_deg_s = GATEWAY_TASK3_REVERSE_RATE_DEG_S,
        .maximum_reverse_offset_deg =
            GATEWAY_TASK3_MAX_REVERSE_OFFSET_DEG,
        .maximum_step_ms = GATEWAY_TASK3_MAX_RAISE_STEP_MS,
    };
    const Algorithm_BallModelInterpolator_Config model_interpolator_config = {
        .gravity_mm_s2 = GATEWAY_BALL_GRAVITY_MM_S2,
        .inertia_ratio = GATEWAY_BALL_INERTIA_RATIO,
        .position_alpha = GATEWAY_BALL_MODEL_POSITION_ALPHA,
        .velocity_beta = GATEWAY_BALL_MODEL_VELOCITY_BETA,
        .maximum_absolute_innovation_mm =
            GATEWAY_BALL_MODEL_MAX_INNOVATION_MM,
        .maximum_absolute_position_mm =
            GATEWAY_BALL_MODEL_MAX_POSITION_MM,
        .maximum_absolute_velocity_mm_s =
            GATEWAY_BALL_MODEL_MAX_VELOCITY_MM_S,
        .maximum_prediction_step_ms =
            GATEWAY_BALL_MODEL_MAX_PREDICT_STEP_MS,
        .maximum_measurement_age_ms =
            GATEWAY_BALL_MODEL_MAX_MEASUREMENT_AGE_MS,
        .maximum_coast_ms = GATEWAY_BALL_MODEL_MAX_COAST_MS,
        .minimum_accepted_measurements =
            GATEWAY_BALL_MODEL_MIN_ACCEPTED_VIS,
    };
    const Algorithm_BallStateEstimator_Config estimator_config = {
        .measurement_polarity = GATEWAY_BALL_MEASUREMENT_POLARITY,
        .gravity_mm_s2 = GATEWAY_BALL_GRAVITY_MM_S2,
        .inertia_ratio = GATEWAY_BALL_INERTIA_RATIO,
        .position_measurement_variance_mm2 =
            GATEWAY_BALL_KALMAN_POSITION_R_MM2,
        .model_acceleration_noise_density_mm2_s3 =
            GATEWAY_BALL_KALMAN_MODEL_ACCEL_Q_MM2_S3,
        .bias_walk_variance_mm2_s4_per_s =
            GATEWAY_BALL_KALMAN_BIAS_WALK_Q_MM2_S4_S,
        .initial_position_variance_mm2 =
            GATEWAY_BALL_KALMAN_INITIAL_POSITION_P_MM2,
        .initial_velocity_variance_mm2_s2 =
            GATEWAY_BALL_KALMAN_INITIAL_VELOCITY_P_MM2_S2,
        .initial_bias_variance_mm2_s4 =
            GATEWAY_BALL_KALMAN_INITIAL_BIAS_P_MM2_S4,
        .innovation_gate_sigma =
            GATEWAY_BALL_KALMAN_INNOVATION_GATE_SIGMA,
        .maximum_position_innovation_mm =
            GATEWAY_BALL_KALMAN_MAX_INNOVATION_MM,
        .maximum_absolute_position_mm =
            GATEWAY_BALL_KALMAN_MAX_POSITION_MM,
        .maximum_absolute_velocity_mm_s =
            GATEWAY_BALL_KALMAN_MAX_VELOCITY_MM_S,
        .maximum_absolute_bias_mm_s2 =
            GATEWAY_BALL_KALMAN_MAX_BIAS_MM_S2,
        .trusted_position_variance_mm2 =
            GATEWAY_BALL_KALMAN_TRUST_POSITION_P_MM2,
        .trusted_velocity_variance_mm2_s2 =
            GATEWAY_BALL_KALMAN_TRUST_VELOCITY_P_MM2_S2,
        .maximum_prediction_step_ms =
            GATEWAY_BALL_KALMAN_MAX_PREDICT_STEP_MS,
        .maximum_measurement_age_ms =
            GATEWAY_BALL_KALMAN_MAX_COAST_MS,
        .minimum_accepted_measurements =
            GATEWAY_BALL_KALMAN_MIN_ACCEPTED_VIS,
    };
    const Algorithm_ChassisAccelerationFusion_Config fusion_config = {
        .plan_gain = GATEWAY_CHASSIS_FUSION_PLAN_GAIN,
        .response_time_constant_s =
            GATEWAY_CHASSIS_FUSION_RESPONSE_TAU_S,
        .plan_delay_ms = GATEWAY_CHASSIS_FUSION_PLAN_DELAY_MS,
        .wheel_speed_measurement_variance_mm2_s2 =
            GATEWAY_CHASSIS_FUSION_WHEEL_R_MM2_S2,
        .wheel_acceleration_measurement_variance_mm2_s4 =
            GATEWAY_CHASSIS_FUSION_WHEEL_ACCEL_R_MM2_S4,
        .imu_acceleration_measurement_variance_mm2_s4 =
            GATEWAY_CHASSIS_FUSION_IMU_R_MM2_S4,
        .velocity_noise_density_mm2_s3 =
            GATEWAY_CHASSIS_FUSION_VELOCITY_Q_MM2_S3,
        .acceleration_noise_density_mm2_s5 =
            GATEWAY_CHASSIS_FUSION_ACCEL_Q_MM2_S5,
        .imu_bias_walk_noise_density_mm2_s5 =
            GATEWAY_CHASSIS_FUSION_BIAS_Q_MM2_S5,
        .initial_velocity_variance_mm2_s2 =
            GATEWAY_CHASSIS_FUSION_INITIAL_VELOCITY_P_MM2_S2,
        .initial_acceleration_variance_mm2_s4 =
            GATEWAY_CHASSIS_FUSION_INITIAL_ACCEL_P_MM2_S4,
        .initial_imu_bias_variance_mm2_s4 =
            GATEWAY_CHASSIS_FUSION_INITIAL_BIAS_P_MM2_S4,
        .innovation_gate_sigma = GATEWAY_CHASSIS_FUSION_GATE_SIGMA,
        .maximum_absolute_velocity_mm_s =
            GATEWAY_CHASSIS_FUSION_MAX_VELOCITY_MM_S,
        .maximum_absolute_acceleration_mm_s2 =
            GATEWAY_CHASSIS_FUSION_MAX_ACCEL_MM_S2,
        .maximum_absolute_imu_bias_mm_s2 =
            GATEWAY_CHASSIS_FUSION_MAX_BIAS_MM_S2,
        .trusted_acceleration_variance_mm2_s4 =
            GATEWAY_CHASSIS_FUSION_TRUST_ACCEL_P_MM2_S4,
        .maximum_prediction_step_ms =
            GATEWAY_CHASSIS_FUSION_MAX_PREDICT_STEP_MS,
        .maximum_measurement_age_ms =
            GATEWAY_CHASSIS_FUSION_MAX_MEAS_AGE_MS,
        .minimum_accepted_wheel_measurements =
            GATEWAY_CHASSIS_FUSION_MIN_WHEEL_SAMPLES,
        .minimum_accepted_imu_measurements =
            GATEWAY_CHASSIS_FUSION_MIN_IMU_SAMPLES,
    };

    if (GATEWAY_ESP8266_ENABLE != 0U) {
        esp_uart = BSP_UART_GetHandle(GATEWAY_ESP_UART_INDEX);
    }
    s_can = BSP_CAN_GetHandle(0U);
    if (((GATEWAY_ESP8266_ENABLE != 0U) && (esp_uart == NULL)) ||
        (s_can == NULL) ||
        !isfinite(GATEWAY_LEVEL_START_MAX_SPEED_RPM) ||
        (GATEWAY_LEVEL_START_MAX_SPEED_RPM < 0.0F) ||
        (GATEWAY_LEVEL_FEEDBACK_TIMEOUT_MS <=
            GATEWAY_LEVEL_CONTROL_PERIOD_MS) ||
        (GATEWAY_LEVEL_CURRENT_LIMIT_A >
            GATEWAY_MOTOR_CURRENT_LIMIT_A) ||
        !isfinite(GATEWAY_LEVEL_CALIBRATION_MIN_CURRENT_A) ||
        (GATEWAY_LEVEL_CALIBRATION_MIN_CURRENT_A <= 0.0F) ||
        (GATEWAY_LEVEL_CALIBRATION_MIN_CURRENT_A >
            GATEWAY_LEVEL_CURRENT_LIMIT_A) ||
        !isfinite(GATEWAY_LEVEL_CALIBRATION_APPROACH_MIN_CURRENT_A) ||
        (GATEWAY_LEVEL_CALIBRATION_APPROACH_MIN_CURRENT_A <= 0.0F) ||
        (GATEWAY_LEVEL_CALIBRATION_APPROACH_MIN_CURRENT_A >
            GATEWAY_LEVEL_CALIBRATION_MIN_CURRENT_A) ||
        (GATEWAY_LEVEL_CALIBRATION_SLOWDOWN_RAW <=
            GATEWAY_LEVEL_CALIBRATION_STABLE_TOL_RAW) ||
        !isfinite(GATEWAY_LEVEL_CALIBRATION_HOLD_KP_A_PER_RAW) ||
        (GATEWAY_LEVEL_CALIBRATION_HOLD_KP_A_PER_RAW <= 0.0F) ||
        !isfinite(GATEWAY_LEVEL_CALIBRATION_SPEED_KD_A_PER_RPM) ||
        (GATEWAY_LEVEL_CALIBRATION_SPEED_KD_A_PER_RPM < 0.0F) ||
        !isfinite(GATEWAY_LEVEL_CALIBRATION_START_MAX_SPEED_RPM) ||
        (GATEWAY_LEVEL_CALIBRATION_START_MAX_SPEED_RPM < 0.0F) ||
        !isfinite(GATEWAY_LEVEL_ZERO_ENTRY_TOL_DEG) ||
        (GATEWAY_LEVEL_ZERO_ENTRY_TOL_DEG <= 0.0F) ||
        !isfinite(GATEWAY_LEVEL_ZERO_ENTRY_MAX_RPM) ||
        (GATEWAY_LEVEL_ZERO_ENTRY_MAX_RPM < 0.0F) ||
        (GATEWAY_LEVEL_ZERO_ENTRY_SETTLE_MS == 0U) ||
        !isfinite(GATEWAY_CHASSIS_FF_GAIN) ||
        (GATEWAY_CHASSIS_FF_GAIN < 0.0F) ||
        !isfinite(GATEWAY_TASK4_CHASSIS_FF_GAIN) ||
        (GATEWAY_TASK4_CHASSIS_FF_GAIN < 0.0F) ||
        !isfinite(GATEWAY_CHASSIS_STOP_PLAN_FF_GAIN) ||
        (GATEWAY_CHASSIS_STOP_PLAN_FF_GAIN < 0.0F) ||
        !isfinite(GATEWAY_CHASSIS_WHEEL_ACCEL_CRUISE_GAIN) ||
        (GATEWAY_CHASSIS_WHEEL_ACCEL_CRUISE_GAIN < 0.0F) ||
        (GATEWAY_CHASSIS_WHEEL_ACCEL_CRUISE_GAIN > 1.0F) ||
        !isfinite(GATEWAY_CHASSIS_PLAN_DOMINANT_ACCEL_MM_S2) ||
        (GATEWAY_CHASSIS_PLAN_DOMINANT_ACCEL_MM_S2 <= 0.0F) ||
        !isfinite(GATEWAY_CHASSIS_STATIONARY_WHEEL_SPEED_MM_S) ||
        (GATEWAY_CHASSIS_STATIONARY_WHEEL_SPEED_MM_S <= 0.0F) ||
        !isfinite(GATEWAY_CHASSIS_FF_POLARITY) ||
        ((GATEWAY_CHASSIS_FF_POLARITY != 1.0F) &&
         (GATEWAY_CHASSIS_FF_POLARITY != -1.0F)) ||
        !isfinite(GATEWAY_CHASSIS_FF_ANGLE_LIMIT_DEG) ||
        (GATEWAY_CHASSIS_FF_ANGLE_LIMIT_DEG <= 0.0F) ||
        !Algorithm_LevelCurrentControl_Init(
            &s_level_controller, &level_config) ||
        !Algorithm_LevelCurrentControl_CaptureZero(
            &s_level_controller,
            Device_QD4310_AngleRawToRad(GATEWAY_LEVEL_ZERO_RAW)) ||
        !Algorithm_BallPositionControl_Init(
            &s_ball_controller, &ball_config) ||
        !Task3_RodSequence_Init(
            &s_task3_rod_sequence, &task3_rod_sequence_config, now_ms) ||
        !Algorithm_BallModelInterpolator_Init(
            &s_ball_model_interpolator,
            &model_interpolator_config,
            now_ms) ||
        !Algorithm_BallStateEstimator_Init(
            &s_ball_estimator, &estimator_config, now_ms) ||
        !Algorithm_ChassisAccelerationFusion_Init(
            &s_chassis_fusion, &fusion_config, now_ms) ||
        !Algorithm_RodAngleMap_Init(
            &s_rod_angle_map,
            s_rod_angle_points,
            GATEWAY_ARRAY_COUNT(s_rod_angle_points),
            GATEWAY_LEVEL_ZERO_RAW) ||
        !Algorithm_RodAngleMap_GetLimits(
            &s_rod_angle_map,
            &s_rod_minimum_angle_deg,
            &s_rod_maximum_angle_deg) ||
        !Algorithm_RodAngleMap_RawToRodDeg(
            &s_rod_angle_map,
            (float)GATEWAY_LEVEL_TRIM_RAW,
            &s_rod_trim_angle_deg) ||
        ((s_rod_trim_angle_deg + GATEWAY_TOTAL_ROD_COMMAND_MIN_DEG) <
            s_rod_minimum_angle_deg) ||
        ((s_rod_trim_angle_deg + GATEWAY_TOTAL_ROD_COMMAND_MAX_DEG) >
            s_rod_maximum_angle_deg) ||
        !apply_rod_target_deg(s_rod_trim_angle_deg)) {
        return false;
    }

    if (!Algorithm_Packet_Parser_Init(
            &s_parser, s_parser_storage, sizeof(s_parser_storage))) {
        return false;
    }

    for (size_t index = 0U; index < GATEWAY_MOTOR_COUNT; ++index) {
        Device_QD4310_Config config;

        if (!Device_QD4310_DefaultConfig(
                s_can, s_motor_ids[index], &config)) {
            return false;
        }
        config.send_timeout_ms = GATEWAY_CAN_SEND_TIMEOUT_MS;
        config.feedback_timeout_ms =
            GATEWAY_MOTOR_FEEDBACK_TIMEOUT_MS;
        config.current_limit_a = GATEWAY_MOTOR_CURRENT_LIMIT_A;
        config.speed_limit_rpm = GATEWAY_MOTOR_SPEED_LIMIT_RPM;
        if (!Device_QD4310_Init(&s_motors[index], &config)) {
            return false;
        }
        s_motor_list[index] = &s_motors[index];
        s_last_feedback_sequence[index] = 0U;
    }
    /* QD4310 can retain its previous enable/current mode across an MCU reset.
     * Issue a best-effort stop immediately; fresh feedback is checked again
     * every Step while the KEY-owned controller remains stopped. */
    (void)Device_QD4310_Disable(&s_motors[0]);

    if (GATEWAY_ESP8266_ENABLE != 0U) {
        esp8266_init(&s_wireless, esp_uart, now_ms);
    }
    s_command_count = 0U;
    s_command_errors = 0U;
    s_next_feedback_ms = now_ms + GATEWAY_FEEDBACK_PERIOD_MS;
    s_tx_sequence = 0U;
    s_last_control_feedback_sequence = 0U;
    s_level_control_fault_count = 0U;
    s_level_enable_requested_ms = 0U;
    s_level_calibration_index = 0U;
    s_level_calibration_active = false;
    s_level_calibration_target_reached = false;
    s_level_calibration_stable_seen = false;
    s_level_calibration_point_started_ms = 0U;
    s_level_calibration_stable_since_ms = 0U;
    s_level_calibration_hold_since_ms = 0U;
    s_motor_command_current_a = 0.0F;
    s_selected_chassis_acceleration_mm_s2 = 0;
    s_planned_chassis_acceleration_mm_s2 = 0;
    s_last_chassis_imu_sequence_seen = false;
    s_last_chassis_imu_sequence = 0U;
    s_last_chassis_imu_sample_ms = 0U;
    s_last_chassis_wheel_sequence_seen = false;
    s_last_chassis_wheel_sequence = 0U;
    s_last_chassis_wheel_sample_ms = 0U;
    s_last_chassis_wheel_acceleration_sequence_seen = false;
    s_last_chassis_wheel_acceleration_sequence = 0U;
    s_last_chassis_wheel_acceleration_sample_ms = 0U;
    s_simple_plan_compensation_active = false;
    s_chassis_fusion_trusted = false;
    s_last_ball_sequence_seen = false;
    s_ball_measurement_valid = false;
    s_chassis_acceleration_valid = false;
    s_motor_raw_limit_blocked = false;
    s_level_phase = GATEWAY_LEVEL_PHASE_STOPPED;
    s_zero_entry_stable_seen = false;
    s_zero_entry_stable_since_ms = 0U;
    s_chassis_compensation_angle_deg = 0.0F;
    s_last_ball_target_update_ms = now_ms;
    s_direct_visual_competition_mode = UINT8_C(0);
    reset_task6_level_calibration();
    return true;
}

void APP_Gateway_Step(
    uint32_t now_ms,
    const APP_Gateway_ControlInput *control_input,
    APP_Gateway_Telemetry *primary_telemetry)
{
    uint8_t byte;
    size_t processed = 0U;
    const uint32_t target_elapsed_ms =
        now_ms - s_last_ball_target_update_ms;

    /* Report raw limiting for the current control cycle. The protection
     * itself remains active; only the diagnostic state stops latching an
     * old recovery or rejected target forever. */
    s_motor_raw_limit_blocked = false;
    s_last_ball_target_update_ms = now_ms;
    if (!select_level_tuning(control_input)) {
        ++s_level_control_fault_count;
    }
    if (!Algorithm_BallPositionControl_AdvanceTarget(
            &s_ball_controller, target_elapsed_ms)) {
        ++s_level_control_fault_count;
    }

    if (GATEWAY_ESP8266_ENABLE != 0U) {
        esp8266_poll(&s_wireless, now_ms);
    }

    if (!update_primary_tracking_target(now_ms, control_input)) {
        ++s_level_control_fault_count;
    }

    /* 下行：解析 TCP 字节流中的 Packet 帧并执行 */
    while ((GATEWAY_ESP8266_ENABLE != 0U) &&
        esp8266_read(&s_wireless, &byte)) {
        Algorithm_Packet_View view;

        if (Algorithm_Packet_Parser_Feed(&s_parser, byte, &view) !=
            ALGORITHM_PACKET_FEED_FRAME_READY) {
            continue;
        }
        if ((view.type_length == 3U) &&
            (memcmp(view.type, "CMD", 3U) == 0)) {
            const bool ok = execute_command(&view);

            ++s_command_count;
            if (!ok) {
                ++s_command_errors;
            }
            send_ack(view.sequence, ok);
        }
    }

    /* 上行：收 CAN 反馈帧并回传 */
    (void)Device_QD4310_PollBus(
        s_can, s_motor_list, GATEWAY_MOTOR_COUNT, now_ms,
        BSP_CAN_CLASSIC_MAX_DATA_LENGTH, &processed);
    enforce_primary_motor_disabled_when_stopped(now_ms);
    run_primary_level_control(now_ms);
    if (GATEWAY_ESP8266_ENABLE != 0U) {
        for (size_t index = 0U; index < GATEWAY_MOTOR_COUNT; ++index) {
            send_feedback(index, now_ms);
        }
    }

    /* 周期 NOP：应答式协议需要主动请求反馈 */
    if ((int32_t)(now_ms - s_next_feedback_ms) >= 0) {
        s_next_feedback_ms = now_ms + GATEWAY_FEEDBACK_PERIOD_MS;
        for (size_t index = 0U; index < GATEWAY_MOTOR_COUNT; ++index) {
            if ((index == 0U) &&
                Algorithm_LevelCurrentControl_IsActive(
                    &s_level_controller)) {
                continue;  /* CURRENT command already requests fresh feedback. */
            }
            (void)Device_QD4310_RequestFeedback(&s_motors[index]);
        }
    }
    make_primary_telemetry(now_ms, primary_telemetry);
}

bool APP_Gateway_StartPrimaryMotor(uint32_t now_ms)
{
    Device_QD4310 *motor = &s_motors[0];
    Device_QD4310_Feedback feedback;

    if (!motor->initialized) {
        return false;
    }
    if (Algorithm_LevelCurrentControl_IsActive(&s_level_controller)) {
        return true;
    }
    if (motor->command_enabled) {
        return false;
    }
    if (!Device_QD4310_GetFeedback(
            motor, now_ms, &feedback) ||
        !feedback.online || feedback.enabled ||
        ((uint32_t)(now_ms - feedback.received_at_ms) >=
            GATEWAY_LEVEL_FEEDBACK_TIMEOUT_MS) ||
        !isfinite(feedback.speed_rpm) ||
        (fabsf(feedback.speed_rpm) >
            GATEWAY_LEVEL_START_MAX_SPEED_RPM) ||
        !apply_rod_target_deg(0.0F) ||
        !Algorithm_LevelCurrentControl_Start(&s_level_controller)) {
        return false;
    }
    s_motor_raw_limit_blocked = false;
    s_level_phase = GATEWAY_LEVEL_PHASE_ZEROING;
    s_zero_entry_stable_seen = false;
    s_last_control_feedback_sequence = feedback.sequence;
    s_level_enable_requested_ms = now_ms;
    if (Device_QD4310_Enable(motor) &&
        Device_QD4310_SetCurrent(motor, 0.0F)) {
        s_motor_command_current_a = 0.0F;
        return true;
    }
    (void)stop_primary_level_control();
    return false;
}

bool APP_Gateway_TogglePrimaryMotor(uint32_t now_ms)
{
    Device_QD4310 *motor = &s_motors[0];

    if (!motor->initialized) {
        return false;
    }
    if (Algorithm_LevelCurrentControl_IsActive(&s_level_controller) ||
        motor->command_enabled) {
        return stop_primary_level_control();
    }
    return APP_Gateway_StartPrimaryMotor(now_ms);
}

bool APP_Gateway_StopPrimaryMotor(void)
{
    return stop_primary_level_control();
}

bool APP_Gateway_AdvanceTask6LevelCalibration(uint32_t now_ms)
{
    Device_QD4310 *motor = &s_motors[0];
    Device_QD4310_Feedback feedback;
    float captured_rod_deg;

    if (!motor->initialized ||
        !Algorithm_LevelCurrentControl_IsActive(&s_level_controller) ||
        !motor->command_enabled ||
        !Device_QD4310_GetFeedback(motor, now_ms, &feedback) ||
        !feedback.online || !feedback.enabled ||
        ((uint32_t)(now_ms - feedback.received_at_ms) >=
            GATEWAY_LEVEL_FEEDBACK_TIMEOUT_MS)) {
        return false;
    }

    if (s_task6_level_calibration_phase ==
            GATEWAY_TASK6_LEVEL_CALIBRATION_NORMAL) {
        if (!Device_QD4310_SetCurrent(motor, 0.0F)) {
            return false;
        }
        s_motor_command_current_a = 0.0F;
        s_motor_target_raw = feedback.single_turn_angle_raw;
        s_task6_level_calibration_phase =
            GATEWAY_TASK6_LEVEL_CALIBRATION_ZERO_CURRENT;
        reset_estimated_ball_feedback(now_ms);
        return true;
    }

    if (s_task6_level_calibration_phase ==
            GATEWAY_TASK6_LEVEL_CALIBRATION_ZERO_CURRENT) {
        if (!Algorithm_RodAngleMap_RawToRodDeg(
                &s_rod_angle_map,
                (float)feedback.single_turn_angle_raw,
                &captured_rod_deg) ||
            ((captured_rod_deg + GATEWAY_TOTAL_ROD_COMMAND_MIN_DEG) <
                s_rod_minimum_angle_deg) ||
            ((captured_rod_deg + GATEWAY_TOTAL_ROD_COMMAND_MAX_DEG) >
                s_rod_maximum_angle_deg)) {
            return false;
        }
        s_task6_level_zero_raw = feedback.single_turn_angle_raw;
        s_task6_level_zero_rod_deg = captured_rod_deg;
        s_task6_level_zero_valid = true;
        s_motor_target_raw = s_task6_level_zero_raw;
        s_task6_level_calibration_phase =
            GATEWAY_TASK6_LEVEL_CALIBRATION_CAPTURED;
        return true;
    }

    if (!s_task6_level_zero_valid ||
        !Algorithm_LevelCurrentControl_SetTargetDeg(
            &s_level_controller,
            raw_target_to_relative_motor_deg(
                (float)s_task6_level_zero_raw)) ||
        !apply_rod_target_deg(s_task6_level_zero_rod_deg) ||
        !Algorithm_BallPositionControl_RearmPositionLoop(
            &s_ball_controller)) {
        return false;
    }
    reset_estimated_ball_feedback(now_ms);
    s_level_phase = GATEWAY_LEVEL_PHASE_ACCELERATION_COMPENSATION;
    s_zero_entry_stable_seen = false;
    s_chassis_acceleration_valid = false;
    s_selected_chassis_acceleration_mm_s2 = 0;
    s_chassis_compensation_angle_deg = 0.0F;
    s_motor_command_current_a = 0.0F;
    s_task6_level_calibration_phase =
        GATEWAY_TASK6_LEVEL_CALIBRATION_NORMAL;
    return true;
}

bool APP_Gateway_SetBallTargetMm(float target_mm)
{
    return Algorithm_BallPositionControl_SetTargetPositionMm(
        &s_ball_controller, target_mm);
}

bool APP_Gateway_SetBallPidGains(
    float kp_per_s2, float ki_per_s3, float kd_per_s)
{
    if (!isfinite(kp_per_s2) || !isfinite(ki_per_s3) ||
        !isfinite(kd_per_s) ||
        (kp_per_s2 < 0.0F) ||
        (kp_per_s2 > GATEWAY_BALL_TUNING_KP_MAX_PER_S2) ||
        (ki_per_s3 < 0.0F) ||
        (ki_per_s3 > GATEWAY_BALL_TUNING_KI_MAX_PER_S3) ||
        (kd_per_s < 0.0F) ||
        (kd_per_s > GATEWAY_BALL_TUNING_KD_MAX_PER_S)) {
        return false;
    }
    return Algorithm_BallPositionControl_SetGains(
        &s_ball_controller, kp_per_s2, ki_per_s3, kd_per_s);
}

bool APP_Gateway_GetBallPidGains(
    float *kp_per_s2, float *ki_per_s3, float *kd_per_s)
{
    if ((kp_per_s2 == NULL) || (ki_per_s3 == NULL) ||
        (kd_per_s == NULL) || !s_ball_controller.initialized) {
        return false;
    }
    *kp_per_s2 = s_ball_controller.config.position_gain_per_s2;
    *ki_per_s3 = s_ball_controller.config.position_integral_gain_per_s3;
    *kd_per_s = s_ball_controller.config.velocity_gain_per_s;
    return true;
}

bool APP_Gateway_AdvancePrimaryMotorCalibration(uint32_t now_ms)
{
    Device_QD4310 *motor = &s_motors[0];
    Device_QD4310_Feedback feedback;
    bool target_configured;

    if (!motor->initialized ||
        !isfinite(GATEWAY_LEVEL_CALIBRATION_STABLE_MAX_RPM) ||
        (GATEWAY_LEVEL_CALIBRATION_STABLE_MAX_RPM < 0.0F)) {
        return false;
    }
    if (s_level_calibration_active ||
        Algorithm_LevelCurrentControl_IsActive(&s_level_controller) ||
        motor->command_enabled) {
        return stop_primary_level_control();
    }
    if (!Device_QD4310_GetFeedback(motor, now_ms, &feedback) ||
        !feedback.online || feedback.enabled ||
        ((uint32_t)(now_ms - feedback.received_at_ms) >=
            GATEWAY_LEVEL_FEEDBACK_TIMEOUT_MS) ||
        !isfinite(feedback.speed_rpm) ||
        (fabsf(feedback.speed_rpm) >
            GATEWAY_LEVEL_CALIBRATION_START_MAX_SPEED_RPM)) {
        return false;
    }
    Algorithm_BallPositionControl_ResetMeasurement(&s_ball_controller);
    if (GATEWAY_LEVEL_CALIBRATION_MANUAL_MODE != 0U) {
        target_configured = Algorithm_LevelCurrentControl_SetTargetDeg(
            &s_level_controller,
            raw_target_to_relative_motor_deg(
                (float)feedback.single_turn_angle_raw));
        s_motor_target_raw = feedback.single_turn_angle_raw;
        s_level_calibration_point_started_ms = now_ms;
        s_level_phase = GATEWAY_LEVEL_PHASE_ZEROING;
    }
    else {
        target_configured = apply_level_calibration_target(0U, now_ms);
    }
    if (!target_configured ||
        !Algorithm_LevelCurrentControl_Start(&s_level_controller)) {
        (void)stop_primary_level_control();
        return false;
    }
    s_level_calibration_active = true;
    s_motor_raw_limit_blocked = false;
    s_zero_entry_stable_seen = false;
    s_last_control_feedback_sequence = feedback.sequence;
    if (Device_QD4310_Enable(motor) &&
        Device_QD4310_SetCurrent(motor, 0.0F)) {
        s_motor_command_current_a = 0.0F;
        return true;
    }
    (void)stop_primary_level_control();
    return false;
}

uint32_t APP_Gateway_CommandCount(void)
{
    return s_command_count;
}

uint32_t APP_Gateway_CommandErrorCount(void)
{
    return s_command_errors;
}

bool APP_Gateway_WirelessReady(void)
{
    return (GATEWAY_ESP8266_ENABLE != 0U) &&
        (esp8266_status(&s_wireless) == ESP8266_SESSION_READY);
}
