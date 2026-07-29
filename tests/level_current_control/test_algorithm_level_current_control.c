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
        .error_limit_deg = 8.0F,
        .deadband_deg = 0.20F,
        .integral_zone_deg = 1.0F,
        .integral_limit_a = 0.010F,
        .current_limit_a = 0.30F,
        .current_polarity = 1.0F,
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

static void test_invalid_config_and_polarity(void)
{
    Algorithm_LevelCurrentControl controller;
    Algorithm_LevelCurrentControl_Config config = default_config();
    float current_a;

    config.target_deg = 86.0F;
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

int main(void)
{
    test_boot_zero_is_captured_once();
    test_single_turn_wrap_and_proportional_current();
    test_deadband_integral_zone_and_integral_limit();
    test_speed_damping_current_limit_and_hard_limit();
    test_runtime_target_update();
    test_invalid_config_and_polarity();
    puts("level current control tests: PASS");
    return 0;
}
