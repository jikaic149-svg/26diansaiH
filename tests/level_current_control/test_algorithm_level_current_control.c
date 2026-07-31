#include "algorithm_level_current_control.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define TEST_PI      3.14159265358979323846F
#define TEST_TWO_PI  6.28318530717958647692F
#define TEST_DEG2RAD (TEST_PI / 180.0F)

static Algorithm_LevelCurrentControl_Config default_config(void)
{
    return (Algorithm_LevelCurrentControl_Config){
        .control_period_s = 0.010F,
        .target_deg = 0.0F,
        .target_limit_deg = 85.0F,
        .hard_limit_deg = 90.0F,
        .kp_a_per_deg = 0.010F,
        .ki_a_per_deg_s = 0.0002F,
        .kd_a_per_rpm = 0.0005F,
        .feedforward_current_a = 0.0F,
        .error_limit_deg = 8.0F,
        .deadband_deg = 0.20F,
        .integral_zone_deg = 1.0F,
        .integral_limit_a = 0.010F,
        .current_limit_a = 0.30F,
        .current_polarity = 1.0F,
        .raw_min_limit = 1000U,
        .raw_max_limit = 2000U,
        .raw_recovery_current_a = 0.10F,
    };
}

static bool close_enough(float actual, float expected)
{
    /* Float rad<->deg roundoff is below 0.0001 deg, still far below the
     * QD4310 feedback protocol's single-turn position quantum. */
    return fabsf(actual - expected) < 0.0001F;
}

static float wrap_degrees(float degrees)
{
    while (degrees >= 360.0F) {
        degrees -= 360.0F;
    }
    while (degrees < 0.0F) {
        degrees += 360.0F;
    }
    return degrees * TEST_DEG2RAD;
}

static void start_at(
    Algorithm_LevelCurrentControl *controller,
    const Algorithm_LevelCurrentControl_Config *config,
    float zero_degrees)
{
    assert(Algorithm_LevelCurrentControl_Init(controller, config));
    assert(Algorithm_LevelCurrentControl_CaptureZero(
        controller, wrap_degrees(zero_degrees)));
    assert(Algorithm_LevelCurrentControl_Start(controller));
}

static void test_boot_zero_is_captured_once(void)
{
    Algorithm_LevelCurrentControl controller;
    const Algorithm_LevelCurrentControl_Config config = default_config();
    float relative_deg = 0.0F;

    assert(Algorithm_LevelCurrentControl_Init(&controller, &config));
    assert(!Algorithm_LevelCurrentControl_HasZero(&controller));
    assert(!Algorithm_LevelCurrentControl_Start(&controller));
    assert(Algorithm_LevelCurrentControl_CaptureZero(&controller, 1.25F));
    assert(Algorithm_LevelCurrentControl_HasZero(&controller));
    assert(!Algorithm_LevelCurrentControl_CaptureZero(&controller, 2.0F));
    assert(Algorithm_LevelCurrentControl_GetRelativeAngleDeg(
        &controller, 1.25F, &relative_deg));
    assert(close_enough(relative_deg, 0.0F));
}

static void test_single_turn_wrap_and_proportional_current(void)
{
    Algorithm_LevelCurrentControl controller;
    const Algorithm_LevelCurrentControl_Config config = default_config();
    float relative_deg;
    float current_a;

    start_at(&controller, &config, 359.0F);
    assert(Algorithm_LevelCurrentControl_GetRelativeAngleDeg(
        &controller, wrap_degrees(1.0F), &relative_deg));
    assert(close_enough(relative_deg, 2.0F));
    assert(Algorithm_LevelCurrentControl_Update(
        &controller, wrap_degrees(1.0F), 0.0F, &current_a));
    assert(close_enough(current_a, -0.018F));

    start_at(&controller, &config, 1.0F);
    assert(Algorithm_LevelCurrentControl_GetRelativeAngleDeg(
        &controller, wrap_degrees(359.0F), &relative_deg));
    assert(close_enough(relative_deg, -2.0F));
    assert(Algorithm_LevelCurrentControl_Update(
        &controller, wrap_degrees(359.0F), 0.0F, &current_a));
    assert(close_enough(current_a, 0.018F));
}

static void test_deadband_integral_zone_and_integral_limit(void)
{
    Algorithm_LevelCurrentControl controller;
    Algorithm_LevelCurrentControl_Config config = default_config();
    float current_a;

    config.ki_a_per_deg_s = 0.0F;
    start_at(&controller, &config, 0.0F);
    assert(Algorithm_LevelCurrentControl_Update(
        &controller, wrap_degrees(0.1F), 0.0F, &current_a));
    assert(close_enough(current_a, 0.0F));

    config = default_config();
    start_at(&controller, &config, 0.0F);
    assert(Algorithm_LevelCurrentControl_Update(
        &controller, wrap_degrees(-0.5F), 0.0F, &current_a));
    assert(close_enough(current_a, 0.003001F));
    for (unsigned int count = 0U; count < 20000U; ++count) {
        assert(Algorithm_LevelCurrentControl_Update(
            &controller, wrap_degrees(-0.5F), 0.0F, &current_a));
    }
    assert(close_enough(controller.integral_current_a, 0.010F));
    assert(close_enough(current_a, 0.013F));

    assert(Algorithm_LevelCurrentControl_Update(
        &controller, wrap_degrees(-2.0F), 0.0F, &current_a));
    assert(close_enough(controller.integral_current_a, 0.0F));
    assert(close_enough(current_a, 0.018F));
}

static void test_speed_damping_current_limit_and_hard_limit(void)
{
    Algorithm_LevelCurrentControl controller;
    const Algorithm_LevelCurrentControl_Config config = default_config();
    float current_a;

    start_at(&controller, &config, 0.0F);
    assert(Algorithm_LevelCurrentControl_Update(
        &controller, 0.0F, 10.0F, &current_a));
    assert(close_enough(current_a, -0.005F));
    assert(Algorithm_LevelCurrentControl_Update(
        &controller, 0.0F, -1000.0F, &current_a));
    assert(close_enough(current_a, 0.30F));

    assert(!Algorithm_LevelCurrentControl_Update(
        &controller, wrap_degrees(90.1F), 0.0F, &current_a));
    assert(close_enough(current_a, 0.0F));
}

static void test_static_feedforward_and_output_limit(void)
{
    Algorithm_LevelCurrentControl controller;
    Algorithm_LevelCurrentControl_Config config = default_config();
    float current_a;

    config.feedforward_current_a = -0.090F;
    start_at(&controller, &config, 0.0F);
    assert(Algorithm_LevelCurrentControl_Update(
        &controller, 0.0F, 0.0F, &current_a));
    assert(close_enough(current_a, -0.090F));

    assert(Algorithm_LevelCurrentControl_Update(
        &controller, wrap_degrees(20.0F), 0.0F, &current_a));
    assert(close_enough(current_a, -0.168F));

    config = default_config();
    config.feedforward_current_a = 0.31F;
    assert(!Algorithm_LevelCurrentControl_Init(&controller, &config));
    config.feedforward_current_a = NAN;
    assert(!Algorithm_LevelCurrentControl_Init(&controller, &config));
}

static void test_opposing_feedforward_releases_continuously(void)
{
    Algorithm_LevelCurrentControl controller;
    Algorithm_LevelCurrentControl_Config config = default_config();
    const float raw_error_deg = 160.0F * 360.0F / 65535.0F;
    float current_a;

    config.kp_a_per_deg = 0.014F;
    config.ki_a_per_deg_s = 0.0F;
    config.kd_a_per_rpm = 0.0F;
    config.feedforward_current_a = -0.072F;
    start_at(&controller, &config, 0.0F);

    assert(Algorithm_LevelCurrentControl_Update(
        &controller, 0.0F, 0.0F, &current_a));
    assert(close_enough(current_a, -0.072F));

    assert(Algorithm_LevelCurrentControl_Update(
        &controller, wrap_degrees(-0.10F), 0.0F, &current_a));
    assert(close_enough(current_a, -0.036F));

    assert(Algorithm_LevelCurrentControl_Update(
        &controller, wrap_degrees(-0.20F), 0.0F, &current_a));
    assert(close_enough(current_a, 0.0F));

    assert(Algorithm_LevelCurrentControl_Update(
        &controller, wrap_degrees(-raw_error_deg), 0.0F, &current_a));
    assert(close_enough(
        current_a,
        0.014F * (raw_error_deg - config.deadband_deg)));
    assert(current_a > 0.0F);

    assert(Algorithm_LevelCurrentControl_Update(
        &controller, wrap_degrees(raw_error_deg), 0.0F, &current_a));
    assert(close_enough(
        current_a,
        -0.072F - 0.014F * (raw_error_deg - config.deadband_deg)));
}

static void test_runtime_target_update(void)
{
    Algorithm_LevelCurrentControl controller;
    const Algorithm_LevelCurrentControl_Config config = default_config();
    float current_a;

    start_at(&controller, &config, 0.0F);
    controller.integral_current_a = 0.005F;
    assert(Algorithm_LevelCurrentControl_SetTargetDeg(&controller, 2.0F));
    assert(close_enough(controller.config.target_deg, 2.0F));
    assert(close_enough(controller.integral_current_a, 0.0F));
    assert(Algorithm_LevelCurrentControl_Update(
        &controller, 0.0F, 0.0F, &current_a));
    assert(close_enough(current_a, 0.0180004F));
    assert(!Algorithm_LevelCurrentControl_SetTargetDeg(&controller, 86.0F));
    assert(!Algorithm_LevelCurrentControl_SetTargetDeg(&controller, NAN));
    assert(close_enough(controller.config.target_deg, 2.0F));
}

static void test_continuous_target_update_preserves_integral(void)
{
    Algorithm_LevelCurrentControl controller;
    const Algorithm_LevelCurrentControl_Config config = default_config();

    start_at(&controller, &config, 0.0F);
    controller.integral_current_a = 0.005F;
    assert(Algorithm_LevelCurrentControl_SetTrackingTargetDeg(
        &controller, 1.5F));
    assert(close_enough(controller.config.target_deg, 1.5F));
    assert(close_enough(controller.integral_current_a, 0.005F));

    assert(!Algorithm_LevelCurrentControl_SetTrackingTargetDeg(
        &controller, 86.0F));
    assert(!Algorithm_LevelCurrentControl_SetTrackingTargetDeg(
        &controller, NAN));
    assert(close_enough(controller.config.target_deg, 1.5F));
    assert(close_enough(controller.integral_current_a, 0.005F));
}

static void test_integral_rate_scales_to_500_hz(void)
{
    Algorithm_LevelCurrentControl slow;
    Algorithm_LevelCurrentControl fast;
    Algorithm_LevelCurrentControl_Config slow_config = default_config();
    Algorithm_LevelCurrentControl_Config fast_config = default_config();
    float current_a;

    slow_config.control_period_s = 0.010F;
    fast_config.control_period_s = 0.002F;
    start_at(&slow, &slow_config, 0.0F);
    start_at(&fast, &fast_config, 0.0F);

    assert(Algorithm_LevelCurrentControl_Update(
        &slow, wrap_degrees(-0.5F), 0.0F, &current_a));
    for (unsigned int count = 0U; count < 5U; ++count) {
        assert(Algorithm_LevelCurrentControl_Update(
            &fast, wrap_degrees(-0.5F), 0.0F, &current_a));
    }
    assert(close_enough(
        slow.integral_current_a, fast.integral_current_a));
}

static void test_raw_recovery_is_inward_only(void)
{
    Algorithm_LevelCurrentControl controller;
    Algorithm_LevelCurrentControl_Config config = default_config();
    float current_a;
    bool recovery_active;

    start_at(&controller, &config, 0.0F);
    controller.integral_current_a = 0.005F;
    assert(Algorithm_LevelCurrentControl_ComputeRawRecoveryCurrent(
        &controller, 999U, &current_a, &recovery_active));
    assert(recovery_active);
    assert(close_enough(current_a, 0.10F));
    assert(close_enough(controller.integral_current_a, 0.0F));

    assert(Algorithm_LevelCurrentControl_ComputeRawRecoveryCurrent(
        &controller, 1000U, &current_a, &recovery_active));
    assert(!recovery_active);
    assert(close_enough(current_a, 0.0F));
    assert(Algorithm_LevelCurrentControl_ComputeRawRecoveryCurrent(
        &controller, 2000U, &current_a, &recovery_active));
    assert(!recovery_active);

    assert(Algorithm_LevelCurrentControl_ComputeRawRecoveryCurrent(
        &controller, 2001U, &current_a, &recovery_active));
    assert(recovery_active);
    assert(close_enough(current_a, -0.10F));

    config.current_polarity = -1.0F;
    start_at(&controller, &config, 0.0F);
    assert(Algorithm_LevelCurrentControl_ComputeRawRecoveryCurrent(
        &controller, 999U, &current_a, &recovery_active));
    assert(recovery_active);
    assert(close_enough(current_a, -0.10F));
    assert(Algorithm_LevelCurrentControl_ComputeRawRecoveryCurrent(
        &controller, 2001U, &current_a, &recovery_active));
    assert(close_enough(current_a, 0.10F));

    Algorithm_LevelCurrentControl_Stop(&controller);
    assert(!Algorithm_LevelCurrentControl_ComputeRawRecoveryCurrent(
        &controller, 999U, &current_a, &recovery_active));
    assert(!recovery_active);
    assert(close_enough(current_a, 0.0F));
}

static void test_invalid_config_and_polarity(void)
{
    Algorithm_LevelCurrentControl controller;
    Algorithm_LevelCurrentControl_Config config = default_config();
    float current_a;

    config.target_deg = 86.0F;
    assert(!Algorithm_LevelCurrentControl_Init(&controller, &config));

    config = default_config();
    config.raw_max_limit = config.raw_min_limit;
    assert(!Algorithm_LevelCurrentControl_Init(&controller, &config));
    config = default_config();
    config.raw_recovery_current_a = 0.0F;
    assert(!Algorithm_LevelCurrentControl_Init(&controller, &config));
    config.raw_recovery_current_a = 0.31F;
    assert(!Algorithm_LevelCurrentControl_Init(&controller, &config));

    config = default_config();
    config.current_polarity = -1.0F;
    start_at(&controller, &config, 0.0F);
    assert(Algorithm_LevelCurrentControl_Update(
        &controller, wrap_degrees(2.0F), 0.0F, &current_a));
    assert(close_enough(current_a, 0.018F));
    Algorithm_LevelCurrentControl_Stop(&controller);
    assert(!Algorithm_LevelCurrentControl_IsActive(&controller));
    assert(close_enough(controller.integral_current_a, 0.0F));
}

static void test_minimum_current_preserves_direction(void)
{
    Algorithm_LevelCurrentControl controller;
    Algorithm_LevelCurrentControl_Config config = default_config();
    float current_a = -0.073F;

    start_at(&controller, &config, 0.0F);
    assert(Algorithm_LevelCurrentControl_ApplyMinimumCurrent(
        &controller, -75, 0.12F, &current_a));
    assert(close_enough(current_a, -0.12F));

    current_a = 0.0F;
    assert(Algorithm_LevelCurrentControl_ApplyMinimumCurrent(
        &controller, 75, 0.12F, &current_a));
    assert(close_enough(current_a, 0.12F));

    config.current_polarity = -1.0F;
    start_at(&controller, &config, 0.0F);
    current_a = 0.0F;
    assert(Algorithm_LevelCurrentControl_ApplyMinimumCurrent(
        &controller, 75, 0.12F, &current_a));
    assert(close_enough(current_a, -0.12F));
}

static void test_runtime_tuning_isolates_mode_parameters(void)
{
    Algorithm_LevelCurrentControl controller;
    Algorithm_LevelCurrentControl_Config config = default_config();

    start_at(&controller, &config, 0.0F);
    controller.integral_current_a = 0.005F;
    assert(Algorithm_LevelCurrentControl_SetTuning(
        &controller, 0.014F, 0.0010F, 0.0010F, -0.072F, 0.20F, 0.010F));
    assert(close_enough(controller.config.kp_a_per_deg, 0.014F));
    assert(close_enough(controller.config.ki_a_per_deg_s, 0.0010F));
    assert(close_enough(controller.config.kd_a_per_rpm, 0.0010F));
    assert(close_enough(controller.config.feedforward_current_a, -0.072F));
    assert(close_enough(controller.config.deadband_deg, 0.20F));
    assert(close_enough(controller.config.integral_limit_a, 0.010F));
    assert(close_enough(controller.integral_current_a, 0.0F));
    assert(Algorithm_LevelCurrentControl_IsActive(&controller));

    controller.integral_current_a = 0.004F;
    assert(Algorithm_LevelCurrentControl_SetTuning(
        &controller, 0.014F, 0.0010F, 0.0010F, -0.072F, 0.20F, 0.010F));
    assert(close_enough(controller.integral_current_a, 0.004F));

    assert(Algorithm_LevelCurrentControl_SetTuning(
        &controller, 0.014F, 0.0020F, 0.0010F, -0.090F, 0.05F, 0.020F));
    assert(close_enough(controller.config.kp_a_per_deg, 0.014F));
    assert(close_enough(controller.config.ki_a_per_deg_s, 0.0020F));
    assert(close_enough(controller.config.kd_a_per_rpm, 0.0010F));
    assert(close_enough(controller.config.feedforward_current_a, -0.090F));
    assert(close_enough(controller.config.deadband_deg, 0.05F));
    assert(close_enough(controller.config.integral_limit_a, 0.020F));
    assert(close_enough(controller.integral_current_a, 0.0F));
    assert(Algorithm_LevelCurrentControl_IsActive(&controller));

    assert(!Algorithm_LevelCurrentControl_SetTuning(
        &controller, 0.014F, 0.0010F, 0.0010F, -0.31F, 0.20F, 0.010F));
    assert(close_enough(controller.config.feedforward_current_a, -0.090F));
}

int main(void)
{
    test_boot_zero_is_captured_once();
    test_single_turn_wrap_and_proportional_current();
    test_deadband_integral_zone_and_integral_limit();
    test_speed_damping_current_limit_and_hard_limit();
    test_static_feedforward_and_output_limit();
    test_opposing_feedforward_releases_continuously();
    test_runtime_target_update();
    test_continuous_target_update_preserves_integral();
    test_integral_rate_scales_to_500_hz();
    test_raw_recovery_is_inward_only();
    test_invalid_config_and_polarity();
    test_minimum_current_preserves_direction();
    test_runtime_tuning_isolates_mode_parameters();
    puts("level current control tests: PASS");
    return 0;
}
