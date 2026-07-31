#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "algorithm_ball_position_control.h"

static Algorithm_BallPositionControl_Config default_config(void)
{
    return (Algorithm_BallPositionControl_Config){
        .target_position_mm = 0.0F,
        .maximum_absolute_target_position_mm = 125.0F,
        .target_slew_rate_mm_s = 50.0F,
        .measurement_polarity = 1.0F,
        .position_gain_per_s2 = 1.0F,
        .position_integral_gain_per_s3 = 0.0F,
        .velocity_gain_per_s = 2.0F,
        .maximum_integral_acceleration_mm_s2 = 2.0F,
        .velocity_filter_tau_s = 0.0F,
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
    assert(angle > 0.0F); /* Raise the positive end to accelerate toward -x. */

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
    float integral_before_gap;

    config.velocity_filter_tau_s = 0.020F;
    config.position_integral_gain_per_s3 = 0.5F;
    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, UINT32_MAX - 9U, 0.0F));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 10U, 2.0F));
    assert(controller.velocity_valid);
    assert(close_enough(controller.velocity_mm_s, 50.0F, 0.001F));
    integral_before_gap = controller.integral_acceleration_mm_s2;

    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 200U, 3.0F));
    assert(!controller.velocity_valid);
    assert(close_enough(controller.velocity_mm_s, 0.0F, 0.001F));
    assert(close_enough(
        controller.integral_acceleration_mm_s2,
        integral_before_gap,
        0.001F));
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
        &controller, 10.0F, 20.0F));
    assert(close_enough(controller.position_mm, 10.0F, 0.001F));
    assert(close_enough(controller.velocity_mm_s, 20.0F, 0.001F));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(angle > 0.0F);
}

static void test_runtime_target_slew_and_state_preservation(void)
{
    Algorithm_BallPositionControl controller;
    const Algorithm_BallPositionControl_Config config = default_config();

    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_SetEstimatedState(
        &controller, 12.0F, -34.0F));
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
        &controller, 0.0F, 0.0F));
    assert(Algorithm_BallPositionControl_SetTargetPositionMm(
        &controller, 50.0F));
    assert(Algorithm_BallPositionControl_AdvanceTarget(&controller, 1000U));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(angle < 0.0F);

    assert(Algorithm_BallPositionControl_SetTargetPositionMm(
        &controller, -50.0F));
    assert(Algorithm_BallPositionControl_AdvanceTarget(&controller, 2000U));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &angle));
    assert(angle > 0.0F);
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

    config.position_gain_per_s2 = 0.0F;
    config.position_integral_gain_per_s3 = 0.5F;
    config.velocity_gain_per_s = 0.0F;
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

int main(void)
{
    test_position_and_velocity_sign();
    test_real_dt_and_gap_reset();
    test_polarity_limit_and_reset();
    test_external_state_estimate_bypasses_camera_polarity();
    test_runtime_target_slew_and_state_preservation();
    test_runtime_target_control_direction();
    test_runtime_target_rejects_invalid_values();
    test_integral_accumulates_clamps_and_resets();
    puts("ball position control tests: PASS");
    return 0;
}
