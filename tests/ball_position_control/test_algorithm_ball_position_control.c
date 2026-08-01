#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#undef assert
#define assert(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "assertion failed at line %d: %s\n", \
            __LINE__, #condition); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

#include "algorithm_ball_position_control.h"

static Algorithm_BallPositionControl_Config default_config(void)
{
    return (Algorithm_BallPositionControl_Config){
        .target_position_mm = 0.0F,
        .maximum_absolute_target_position_mm = 125.0F,
        .target_slew_rate_mm_s = 50.0F,
        .maximum_target_velocity_mm_s = 50.0F,
        .maximum_target_acceleration_mm_s2 = 1000.0F,
        .measurement_polarity = 1.0F,
        .position_gain_per_s2 = 1.0F,
        .position_integral_gain_per_s3 = 0.0F,
        .velocity_gain_per_s = 2.0F,
        .maximum_integral_acceleration_mm_s2 = 2.0F,
        .velocity_filter_tau_s = 0.0F,
        .velocity_estimation_window_ms = 1U,
        .gravity_mm_s2 = 9807.0F,
        .inertia_ratio = 0.4F,
        .rod_angle_limit_deg = 1.0F,
        .maximum_velocity_dt_ms = 100U,
    };
}

static bool close_enough(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static void test_position_and_velocity_sign(void)
{
    Algorithm_BallPositionControl controller;
    const Algorithm_BallPositionControl_Config config = default_config();
    float angle;

    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 1000U, 50.0F));
    assert(!controller.velocity_valid);
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(angle > 0.0F);

    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 1020U, 52.0F));
    assert(controller.velocity_valid);
    assert(close_enough(controller.velocity_mm_s, 100.0F, 0.001F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(angle > 0.0F);
}

static void test_real_dt_and_gap_reset(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = default_config();

    config.velocity_filter_tau_s = 0.020F;
    config.position_integral_gain_per_s3 = 0.5F;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, UINT32_MAX - 9U, 0.0F));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 10U, 2.0F));
    assert(controller.velocity_valid);
    assert(close_enough(controller.velocity_mm_s, 50.0F, 0.001F));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 200U, 3.0F));
    assert(!controller.velocity_valid);
    assert(close_enough(controller.velocity_mm_s, 0.0F, 0.001F));
    assert(close_enough(
        controller.integral_acceleration_mm_s2,
        0.0F,
        0.001F));
}

static void test_visual_velocity_uses_multi_frame_window(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = default_config();

    config.velocity_estimation_window_ms = 80U;
    config.velocity_filter_tau_s = 0.0F;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 0U, 0.0F));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 20U, 10.0F));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 40U, 0.0F));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 60U, 10.0F));
    assert(!controller.velocity_valid);
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 80U, 8.0F));
    assert(controller.velocity_valid);
    assert(close_enough(controller.velocity_mm_s, 100.0F, 0.001F));
}

static void test_polarity_limit_and_reset(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = default_config();
    float angle;

    config.measurement_polarity = -1.0F;
    config.rod_angle_limit_deg = 0.5F;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 1U, 125.0F));
    assert(close_enough(controller.position_mm, -125.0F, 0.001F));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 101U, 125.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(close_enough(angle, -0.5F, 0.0001F));
    Algorithm_BallPositionControl_ResetMeasurement(&controller);
    assert(!Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
}

static void test_external_state_estimate_bypasses_camera_polarity(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = default_config();
    float angle;

    config.measurement_polarity = -1.0F;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 10.0F, 20.0F));
    assert(close_enough(controller.position_mm, 10.0F, 0.001F));
    assert(close_enough(controller.velocity_mm_s, 20.0F, 0.001F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(angle > 0.0F);
}

static void test_external_state_velocity_filter_uses_real_dt(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = default_config();

    config.velocity_filter_tau_s = 0.100F;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 0.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 100U, 0.0F, 10.0F));
    assert(close_enough(controller.velocity_mm_s, 5.0F, 0.001F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 100U, 0.0F, -10.0F));
    assert(close_enough(controller.velocity_mm_s, 5.0F, 0.001F));
}

static void test_runtime_target_slew_and_state_preservation(void)
{
    Algorithm_BallPositionControl controller;
    const Algorithm_BallPositionControl_Config config = default_config();

    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 12.0F, -34.0F));
    assert(Algorithm_BallPositionControl_SetTargetPositionMm(
        &controller, 50.0F));
    assert(close_enough(controller.position_mm, 12.0F, 0.001F));
    assert(close_enough(controller.velocity_mm_s, -34.0F, 0.001F));
    assert(Algorithm_BallPositionControl_AdvanceTarget(&controller, 20U));
    assert(close_enough(controller.target_position_mm, 1.0F, 0.001F));
    assert(Algorithm_BallPositionControl_AdvanceTarget(&controller, 980U));
    assert(close_enough(controller.target_position_mm, 50.0F, 0.001F));

    assert(Algorithm_BallPositionControl_SetTargetPositionMm(
        &controller, -50.0F));
    assert(Algorithm_BallPositionControl_AdvanceTarget(&controller, 1000U));
    assert(close_enough(controller.target_position_mm, 0.0F, 0.001F));
    assert(Algorithm_BallPositionControl_AdvanceTarget(&controller, 1000U));
    assert(close_enough(controller.target_position_mm, -50.0F, 0.001F));
    assert(close_enough(controller.position_mm, 12.0F, 0.001F));
    assert(close_enough(controller.velocity_mm_s, -34.0F, 0.001F));
}

static void test_runtime_target_control_direction(void)
{
    Algorithm_BallPositionControl controller;
    const Algorithm_BallPositionControl_Config config = default_config();
    float angle;

    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 0.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetTargetPositionMm(
        &controller, 50.0F));
    assert(Algorithm_BallPositionControl_AdvanceTarget(&controller, 1000U));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 100U, 0.0F, 0.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(angle < 0.0F);

    assert(Algorithm_BallPositionControl_SetTargetPositionMm(
        &controller, -50.0F));
    assert(Algorithm_BallPositionControl_AdvanceTarget(&controller, 2000U));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 200U, 0.0F, 0.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(angle > 0.0F);
}

static void test_nonzero_target_equilibrium_and_direction(void)
{
    Algorithm_BallPositionControl controller;
    const Algorithm_BallPositionControl_Config config = default_config();
    float angle;

    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_ResetTargetPositionMm(
        &controller, 40.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 40.0F, 0.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(close_enough(angle, 0.0F, 0.0001F));

    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 1U, 20.0F, 0.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(angle < 0.0F); /* Accelerate from +20 mm toward +40 mm. */
}

static void test_runtime_target_rejects_invalid_values(void)
{
    Algorithm_BallPositionControl controller;
    const Algorithm_BallPositionControl_Config config = default_config();

    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(!Algorithm_BallPositionControl_SetTargetPositionMm(
        &controller, NAN));
    assert(!Algorithm_BallPositionControl_SetTargetPositionMm(
        &controller, INFINITY));
    assert(!Algorithm_BallPositionControl_SetTargetPositionMm(
        &controller, 125.001F));
    assert(!Algorithm_BallPositionControl_SetTargetPositionMm(
        &controller, -125.001F));
    assert(close_enough(controller.target_command_mm, 0.0F, 0.001F));
    assert(Algorithm_BallPositionControl_SetTargetPositionMm(
        &controller, 125.0F));
    assert(Algorithm_BallPositionControl_ResetTargetPositionMm(
        &controller, -125.0F));
    assert(close_enough(controller.target_position_mm, -125.0F, 0.001F));
}

static void test_integral_accumulates_clamps_and_resets(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = default_config();

    config.position_gain_per_s2 = 1.0F;
    config.position_integral_gain_per_s3 = 0.5F;
    config.velocity_gain_per_s = 2.0F;
    config.maximum_integral_acceleration_mm_s2 = 2.0F;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 0U, -10.0F));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 100U, -10.0F));
    assert(close_enough(
        controller.integral_acceleration_mm_s2, 0.5F, 0.001F));

    for (uint32_t sample_ms = 200U; sample_ms <= 1000U;
         sample_ms += 100U) {
        assert(Algorithm_BallPositionControl_UpdateMeasurement(
            &controller, sample_ms, -10.0F));
    }
    assert(close_enough(
        controller.integral_acceleration_mm_s2, 2.0F, 0.001F));
    assert(Algorithm_BallPositionControl_SetGains(
        &controller, 1.0F, 0.0F, 2.0F));
    assert(close_enough(
        controller.integral_acceleration_mm_s2, 0.0F, 0.001F));

    assert(Algorithm_BallPositionControl_SetGains(
        &controller, 1.0F, 0.5F, 2.0F));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 1100U, -10.0F));
    assert(controller.integral_acceleration_mm_s2 > 0.0F);
    Algorithm_BallPositionControl_ResetMeasurement(&controller);
    assert(close_enough(
        controller.integral_acceleration_mm_s2, 0.0F, 0.001F));
}

static void test_integral_stays_clear_while_target_is_slewing(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = default_config();

    config.position_integral_gain_per_s3 = 0.5F;
    config.target_slew_rate_mm_s = 10.0F;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetTargetPositionMm(
        &controller, 20.0F));
    assert(Algorithm_BallPositionControl_AdvanceTarget(&controller, 100U));
    assert(close_enough(controller.target_position_mm, 1.0F, 0.001F));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 0U, 0.0F));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 100U, 0.0F));
    assert(close_enough(
        controller.integral_acceleration_mm_s2, 0.0F, 0.001F));
}

static void test_position_pid_is_normal_motion_path(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = default_config();
    float angle;

    config.position_gain_per_s2 = 4.0F;
    config.velocity_gain_per_s = 2.0F;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_ResetTargetPositionMm(
        &controller, 100.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 0.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 100U, 0.0F, 0.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(close_enough(controller.target_velocity_mm_s, 0.0F, 0.001F));
    assert(close_enough(
        controller.desired_acceleration_mm_s2, 400.0F, 0.001F));
    assert(angle < 0.0F);

    assert(Algorithm_BallPositionControl_ResetTargetPositionMm(
        &controller, 5.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 200U, 0.0F, 5.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(close_enough(controller.target_velocity_mm_s, 0.0F, 0.001F));
    assert(close_enough(
        controller.desired_acceleration_mm_s2, 10.0F, 0.001F));

    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 300U, 5.0F, 5.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(close_enough(controller.target_velocity_mm_s, 0.0F, 0.001F));
    assert(close_enough(
        controller.desired_acceleration_mm_s2, -10.0F, 0.001F));

    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 400U, 5.0F, 0.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(close_enough(controller.target_velocity_mm_s, 0.0F, 0.001F));
    assert(close_enough(angle, 0.0F, 0.001F));
}

static void test_position_pid_rejects_invalid_configuration(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = default_config();

    config.maximum_target_velocity_mm_s = 0.0F;
    assert(!Algorithm_BallPositionControl_Init(&controller, &config));
    config = default_config();
    config.maximum_target_acceleration_mm_s2 = 0.0F;
    assert(!Algorithm_BallPositionControl_Init(&controller, &config));
    config = default_config();
    config.velocity_gain_per_s = 0.0F;
    assert(!Algorithm_BallPositionControl_Init(&controller, &config));

    config = default_config();
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(!Algorithm_BallPositionControl_SetGains(
        &controller, 1.0F, 0.0F, 0.0F));
    assert(close_enough(controller.config.velocity_gain_per_s, 2.0F, 0.001F));
}

static void test_stiction_kick_disabled_configuration(void)
{
    Algorithm_BallPositionControl controller;
    const Algorithm_BallPositionControl_Config config = default_config();

    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_ResetTargetPositionMm(
        &controller, 100.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 0.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 1000U, 0.0F, 0.0F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_IDLE);
    assert(close_enough(controller.stiction_kick_angle_deg, 0.0F, 0.001F));
}

static void test_quiet_hold_stays_until_target_changes(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = default_config();
    float angle;

    config.position_integral_gain_per_s3 = 1.0F;
    config.hold_position_error_mm = 2.0F;
    config.hold_maximum_velocity_mm_s = 5.0F;
    config.hold_stable_duration_ms = 300U;
    config.hold_exit_position_error_mm = 5.0F;
    config.hold_exit_maximum_velocity_mm_s = 12.0F;
    config.hold_exit_sample_count = 3U;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, -1.5F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 100U, -1.5F, 0.0F));
    assert(controller.integral_acceleration_mm_s2 > 0.0F);
    assert(!controller.hold_active);

    /* A single low-speed sample must not latch hold. */
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 200U, -1.5F, 6.0F));
    assert(!controller.hold_active);
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 300U, -1.5F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 400U, -1.5F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 500U, -1.5F, 0.0F));
    assert(!controller.hold_active);
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 600U, -1.5F, 0.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(close_enough(angle, 0.0F, 0.001F));
    assert(close_enough(controller.target_velocity_mm_s, 0.0F, 0.001F));
    assert(close_enough(
        controller.integral_acceleration_mm_s2, 0.0F, 0.001F));
    assert(controller.hold_active);

    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 700U, -1.5F, 6.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(close_enough(angle, 0.0F, 0.001F));
    assert(controller.hold_active);
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 800U, -20.0F, 20.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(close_enough(angle, 0.0F, 0.001F));
    assert(controller.hold_active);

    assert(Algorithm_BallPositionControl_SetTargetPositionMm(
        &controller, 10.0F));
    assert(!controller.hold_active);
    assert(Algorithm_BallPositionControl_AdvanceTarget(
        &controller, 100U));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 900U, -20.0F, 0.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(angle < 0.0F);

    assert(Algorithm_BallPositionControl_AdvanceTarget(
        &controller, 100U));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 1000U, 10.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 1100U, 10.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 1200U, 10.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 1300U, 10.0F, 0.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(close_enough(angle, 0.0F, 0.001F));
    assert(controller.hold_active);
    assert(Algorithm_BallPositionControl_SetTargetPositionMm(
        &controller, 10.0F));
    assert(controller.hold_active);
    Algorithm_BallPositionControl_ResetMeasurement(&controller);
    assert(!controller.hold_active);
}

static void test_quiet_hold_exits_at_nonzero_target_after_hysteresis(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = default_config();
    float angle;

    config.target_position_mm = 40.0F;
    config.hold_position_error_mm = 2.0F;
    config.hold_maximum_velocity_mm_s = 5.0F;
    config.hold_stable_duration_ms = 100U;
    config.hold_exit_position_error_mm = 5.0F;
    config.hold_exit_maximum_velocity_mm_s = 12.0F;
    config.hold_exit_sample_count = 3U;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 40.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 100U, 40.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 200U, 40.0F, 0.0F));
    assert(controller.hold_active);

    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 300U, 32.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 400U, 32.0F, 0.0F));
    assert(controller.hold_active);
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 500U, 32.0F, 0.0F));
    assert(!controller.hold_active);
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(angle < 0.0F);
}

static void test_invalid_quiet_hold_configuration(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = default_config();

    config.hold_position_error_mm = 2.0F;
    config.hold_maximum_velocity_mm_s = 0.0F;
    assert(!Algorithm_BallPositionControl_Init(&controller, &config));
    config = default_config();
    config.hold_position_error_mm = -0.1F;
    assert(!Algorithm_BallPositionControl_Init(&controller, &config));
    config = default_config();
    config.hold_position_error_mm = 2.0F;
    config.hold_maximum_velocity_mm_s = 5.0F;
    config.hold_stable_duration_ms = 100U;
    config.hold_exit_position_error_mm = 2.0F;
    config.hold_exit_maximum_velocity_mm_s = 10.0F;
    config.hold_exit_sample_count = 2U;
    assert(!Algorithm_BallPositionControl_Init(&controller, &config));
}

static void test_near_target_velocity_damping_gain_schedule(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = default_config();
    float angle;

    config.near_target_error_mm = 10.0F;
    config.near_target_velocity_gain_multiplier = 2.0F;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, -5.0F, 3.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 100U, -5.0F, 3.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(close_enough(controller.target_velocity_mm_s, 0.0F, 0.001F));
    assert(close_enough(
        controller.desired_acceleration_mm_s2, -7.0F, 0.001F));

    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 200U, -20.0F, 3.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(close_enough(controller.target_velocity_mm_s, 0.0F, 0.001F));
    assert(close_enough(
        controller.desired_acceleration_mm_s2, 14.0F, 0.001F));
}

static void test_invalid_near_target_gain_schedule(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = default_config();

    config.near_target_error_mm = 10.0F;
    config.near_target_velocity_gain_multiplier = 0.9F;
    assert(!Algorithm_BallPositionControl_Init(&controller, &config));
    config = default_config();
    config.near_target_velocity_gain_multiplier = 1.5F;
    assert(!Algorithm_BallPositionControl_Init(&controller, &config));
}

static Algorithm_BallPositionControl_Config stiction_config(void)
{
    Algorithm_BallPositionControl_Config config = default_config();

    config.position_gain_per_s2 = 0.0F;
    config.velocity_gain_per_s = 0.000001F;
    config.maximum_target_velocity_mm_s = 20.0F;
    config.maximum_target_acceleration_mm_s2 = 30.0F;
    config.stiction_assist_velocity_gain_per_s = 2.0F;
    config.stiction_assist_exit_distance_mm = 2.0F;
    config.stiction_kick_minimum_error_mm = 8.0F;
    config.stiction_kick_maximum_velocity_mm_s = 8.0F;
    config.stiction_kick_duration_ms = 120U;
    return config;
}

static void test_speed_assist_launches_then_position_loop_tracks(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = stiction_config();
    float angle;

    config.position_gain_per_s2 = 1.0F;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 20.0F, 0.0F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_ACTIVE);
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(close_enough(controller.target_velocity_mm_s, 0.0F, 0.001F));
    assert(close_enough(controller.desired_acceleration_mm_s2,
        0.0F, 0.001F));
    assert(close_enough(controller.stiction_kick_angle_deg, 0.0F, 0.001F));

    /* At 9 mm the distance model requests 7.746 mm/s, but the real 100 ms
     * interval limits the first velocity-target step to 3 mm/s. */
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 100U, 9.0F, -4.0F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_ACTIVE);
    assert(close_enough(controller.target_velocity_mm_s,
        -3.0F, 0.001F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(close_enough(controller.desired_acceleration_mm_s2,
        2.0F, 0.001F));

    /* Crossing the stiction speed threshold immediately hands tracking to
     * position PD and clears the launch PI state. */
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 200U, 8.0F, -9.0F));
    assert(controller.stiction_state ==
        ALGORITHM_BALL_STICTION_POSITION_LATCHED);
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 300U, 7.0F, -4.0F));
    assert(controller.stiction_state ==
        ALGORITHM_BALL_STICTION_POSITION_LATCHED);
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 400U, 20.0F, 0.0F));
    assert(controller.stiction_state ==
        ALGORITHM_BALL_STICTION_POSITION_LATCHED);
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(close_enough(controller.target_velocity_mm_s, 0.0F, 0.001F));
    assert(close_enough(controller.desired_acceleration_mm_s2,
        -20.0F, 0.001F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 500U, 20.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 520U, 20.0F, 0.0F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_ACTIVE);
}

static void test_new_target_rearms_using_final_command(void)
{
    Algorithm_BallPositionControl controller;
    const Algorithm_BallPositionControl_Config config = stiction_config();

    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 0.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetTargetPositionMm(
        &controller, 40.0F));
    assert(close_enough(controller.target_position_mm, 0.0F, 0.001F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 20U, 0.0F, 0.0F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_ACTIVE);
    assert(close_enough(controller.target_velocity_mm_s, 0.6F, 0.001F));
}

static void test_speed_target_uses_real_dt_acceleration_slew(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = stiction_config();

    config.maximum_target_velocity_mm_s = 50.0F;
    config.maximum_target_acceleration_mm_s2 = 10.0F;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 100.0F, 0.0F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_ACTIVE);
    assert(close_enough(controller.target_velocity_mm_s, 0.0F, 0.001F));

    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 40U, 100.0F, 0.0F));
    assert(close_enough(controller.target_velocity_mm_s, -0.4F, 0.001F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 100U, 100.0F, 0.0F));
    assert(close_enough(controller.target_velocity_mm_s, -1.0F, 0.001F));

    /* Real motion ends launch assistance immediately. */
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 150U, 8.0F, 9.0F));
    assert(controller.stiction_state ==
        ALGORITHM_BALL_STICTION_POSITION_LATCHED);
    assert(close_enough(controller.target_velocity_mm_s, 0.0F, 0.001F));
}

static void test_latched_position_reenters_after_sustained_stiction(void)
{
    Algorithm_BallPositionControl controller;
    const Algorithm_BallPositionControl_Config config = stiction_config();

    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 0.0F, 0.0F));
    assert(controller.stiction_state ==
        ALGORITHM_BALL_STICTION_POSITION_LATCHED);

    /* The entry band is 10 mm in this test configuration. Outside it, low
     * speed must persist for the configured 120 ms before launch re-arms. */
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 20U, -7.0F, 0.0F));
    assert(controller.stiction_state ==
        ALGORITHM_BALL_STICTION_POSITION_LATCHED);
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 40U, -12.0F, 0.0F));
    assert(controller.stiction_state ==
        ALGORITHM_BALL_STICTION_POSITION_LATCHED);
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 140U, -12.0F, 0.0F));
    assert(controller.stiction_state ==
        ALGORITHM_BALL_STICTION_POSITION_LATCHED);
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 160U, -12.0F, 0.0F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_ACTIVE);
    assert(controller.target_velocity_mm_s > 0.0F);
}

static void test_speed_assist_clears_integral_after_motion(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = stiction_config();

    config.stiction_kick_minimum_error_mm = 3.0F;
    config.stiction_assist_velocity_integral_gain_per_s2 = 10.0F;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 20.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 100U, 10.0F, 0.0F));
    assert(controller.stiction_assist_velocity_integral_acceleration_mm_s2 <
        0.0F);

    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 110U, 9.0F, -9.0F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_ACTIVE);
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 120U, 9.0F, -9.0F));
    assert(controller.stiction_state ==
        ALGORITHM_BALL_STICTION_POSITION_LATCHED);
    assert(close_enough(controller.target_velocity_mm_s, 0.0F, 0.001F));
    assert(close_enough(
        controller.stiction_assist_velocity_integral_acceleration_mm_s2,
        0.0F, 0.001F));
}

static void test_speed_pi_stops_after_motion(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = stiction_config();

    config.maximum_target_velocity_mm_s = 30.0F;
    config.maximum_target_acceleration_mm_s2 = 180.0F;
    config.stiction_assist_velocity_gain_per_s = 1.0F;
    config.stiction_assist_velocity_integral_gain_per_s2 = 10.0F;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 20.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 100U, 20.0F, 0.0F));
    assert(close_enough(
        controller.stiction_assist_velocity_integral_acceleration_mm_s2,
        -18.0F, 0.001F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 200U, 19.0F, -10.0F));
    assert(controller.stiction_state ==
        ALGORITHM_BALL_STICTION_POSITION_LATCHED);
    assert(close_enough(
        controller.stiction_assist_velocity_integral_acceleration_mm_s2,
        0.0F, 0.001F));
}

static void test_speed_handoff_runtime_gate_for_task3(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = stiction_config();
    float angle;

    config.position_gain_per_s2 = 1.0F;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetSpeedHandoffEnabled(
        &controller, false));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 20.0F, 0.0F));
    assert(controller.stiction_state ==
        ALGORITHM_BALL_STICTION_POSITION_LATCHED);
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(close_enough(controller.desired_acceleration_mm_s2,
        -20.0F, 0.001F));
    assert(close_enough(controller.stiction_assist_acceleration_mm_s2,
        0.0F, 0.001F));

    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 200U, 20.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 400U, 20.0F, 0.0F));
    assert(controller.stiction_state ==
        ALGORITHM_BALL_STICTION_POSITION_LATCHED);
    assert(controller.stiction_error_direction == 0);

    assert(Algorithm_BallPositionControl_SetSpeedHandoffEnabled(
        &controller, true));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 420U, 20.0F, 0.0F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_ACTIVE);
    assert(Algorithm_BallPositionControl_SetSpeedHandoffEnabled(
        &controller, false));
    assert(controller.stiction_state ==
        ALGORITHM_BALL_STICTION_POSITION_LATCHED);
}

static void test_runtime_motion_limits_validate_and_reset_trip(void)
{
    Algorithm_BallPositionControl controller;
    const Algorithm_BallPositionControl_Config config = stiction_config();
    float angle;
    float active_target_velocity_mm_s;

    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 20.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 100U, 20.0F, 0.0F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_ACTIVE);
    assert(controller.target_velocity_mm_s != 0.0F);

    assert(Algorithm_BallPositionControl_SetMotionLimits(
        &controller, 40.0F, 240.0F));
    assert(close_enough(controller.config.maximum_target_velocity_mm_s,
        40.0F, 0.001F));
    assert(close_enough(controller.config.maximum_target_acceleration_mm_s2,
        240.0F, 0.001F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_IDLE);
    assert(close_enough(controller.target_velocity_mm_s, 0.0F, 0.001F));
    assert(close_enough(
        controller.stiction_assist_velocity_integral_acceleration_mm_s2,
        0.0F, 0.001F));
    assert(close_enough(controller.desired_acceleration_mm_s2,
        0.0F, 0.001F));
    assert(!controller.hold_active);

    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 200U, 20.0F, 0.0F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_ACTIVE);
    active_target_velocity_mm_s = controller.target_velocity_mm_s;
    assert(active_target_velocity_mm_s != 0.0F);
    assert(Algorithm_BallPositionControl_SetMotionLimits(
        &controller, 40.0F, 240.0F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_ACTIVE);
    assert(close_enough(controller.target_velocity_mm_s,
        active_target_velocity_mm_s, 0.001F));

    assert(!Algorithm_BallPositionControl_SetMotionLimits(
        &controller, NAN, 240.0F));
    assert(!Algorithm_BallPositionControl_SetMotionLimits(
        &controller, 40.0F, 0.0F));
    assert(close_enough(controller.config.maximum_target_velocity_mm_s,
        40.0F, 0.001F));
    assert(close_enough(controller.config.maximum_target_acceleration_mm_s2,
        240.0F, 0.001F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_ACTIVE);

    assert(Algorithm_BallPositionControl_SetMotionLimits(
        &controller, 20.0F, 30.0F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_IDLE);
    assert(close_enough(controller.target_velocity_mm_s, 0.0F, 0.001F));
}

static void test_runtime_final_position_bands_validate_and_reset_trip(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = stiction_config();
    float active_target_velocity_mm_s;

    config.hold_position_error_mm = 3.0F;
    config.hold_maximum_velocity_mm_s = 8.0F;
    config.hold_stable_duration_ms = 60U;
    config.hold_exit_position_error_mm = 5.0F;
    config.hold_exit_maximum_velocity_mm_s = 12.0F;
    config.hold_exit_sample_count = 3U;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 0U, 1.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 100U, 1.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 200U, 1.0F, 0.0F));
    assert(controller.hold_active);

    assert(Algorithm_BallPositionControl_SetFinalPositionBands(
        &controller, 1.5F, 1.5F));
    assert(close_enough(controller.config.stiction_kick_minimum_error_mm,
        1.5F, 0.001F));
    assert(close_enough(controller.config.hold_position_error_mm,
        1.5F, 0.001F));
    assert(!controller.hold_active);
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_IDLE);

    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 300U, 20.0F, 0.0F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_ACTIVE);
    active_target_velocity_mm_s = controller.target_velocity_mm_s;
    assert(Algorithm_BallPositionControl_SetFinalPositionBands(
        &controller, 1.5F, 1.5F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_ACTIVE);
    assert(close_enough(controller.target_velocity_mm_s,
        active_target_velocity_mm_s, 0.001F));

    assert(!Algorithm_BallPositionControl_SetFinalPositionBands(
        &controller, NAN, 1.5F));
    assert(!Algorithm_BallPositionControl_SetFinalPositionBands(
        &controller, 1.5F, 0.0F));
    assert(!Algorithm_BallPositionControl_SetFinalPositionBands(
        &controller, 1.5F, 5.0F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_ACTIVE);

    assert(Algorithm_BallPositionControl_SetFinalPositionBands(
        &controller, 8.0F, 3.0F));
    assert(controller.stiction_state == ALGORITHM_BALL_STICTION_IDLE);
    assert(close_enough(controller.target_velocity_mm_s, 0.0F, 0.001F));
}

static void test_invalid_stiction_configuration(void)
{
    Algorithm_BallPositionControl controller;
    Algorithm_BallPositionControl_Config config = stiction_config();

    config.stiction_kick_angle_deg = 0.01F;
    assert(!Algorithm_BallPositionControl_Init(&controller, &config));
    config = stiction_config();
    config.stiction_kick_minimum_error_mm = 0.0F;
    assert(!Algorithm_BallPositionControl_Init(&controller, &config));
    config = stiction_config();
    config.stiction_kick_duration_ms = 0U;
    assert(!Algorithm_BallPositionControl_Init(&controller, &config));
    config = stiction_config();
    config.stiction_assist_velocity_gain_per_s = 0.0F;
    assert(!Algorithm_BallPositionControl_Init(&controller, &config));
    config = stiction_config();
    config.stiction_assist_velocity_integral_gain_per_s2 = -0.1F;
    assert(!Algorithm_BallPositionControl_Init(&controller, &config));
}

int main(void)
{
    test_position_and_velocity_sign();
    test_real_dt_and_gap_reset();
    test_visual_velocity_uses_multi_frame_window();
    test_polarity_limit_and_reset();
    test_external_state_estimate_bypasses_camera_polarity();
    test_external_state_velocity_filter_uses_real_dt();
    test_runtime_target_slew_and_state_preservation();
    test_runtime_target_control_direction();
    test_nonzero_target_equilibrium_and_direction();
    test_runtime_target_rejects_invalid_values();
    test_integral_accumulates_clamps_and_resets();
    test_integral_stays_clear_while_target_is_slewing();
    test_position_pid_is_normal_motion_path();
    test_position_pid_rejects_invalid_configuration();
    test_stiction_kick_disabled_configuration();
    test_quiet_hold_stays_until_target_changes();
    test_quiet_hold_exits_at_nonzero_target_after_hysteresis();
    test_invalid_quiet_hold_configuration();
    test_near_target_velocity_damping_gain_schedule();
    test_invalid_near_target_gain_schedule();
    test_speed_assist_launches_then_position_loop_tracks();
    test_new_target_rearms_using_final_command();
    test_speed_target_uses_real_dt_acceleration_slew();
    test_latched_position_reenters_after_sustained_stiction();
    test_speed_assist_clears_integral_after_motion();
    test_speed_pi_stops_after_motion();
    test_speed_handoff_runtime_gate_for_task3();
    test_runtime_motion_limits_validate_and_reset_trip();
    test_runtime_final_position_bands_validate_and_reset_trip();
    test_invalid_stiction_configuration();
    puts("ball position control tests: PASS");
    return 0;
}
