#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "algorithm_ball_model_interpolator.h"

static bool close_enough(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static Algorithm_BallModelInterpolator_Config default_config(void)
{
    return (Algorithm_BallModelInterpolator_Config){
        .gravity_mm_s2 = 9800.0F,
        .inertia_ratio = 0.4F,
        .position_alpha = 0.5F,
        .velocity_beta = 0.2F,
        .maximum_absolute_innovation_mm = 12.0F,
        .maximum_absolute_position_mm = 500.0F,
        .maximum_absolute_velocity_mm_s = 1000.0F,
        .maximum_prediction_step_ms = 5U,
        .maximum_measurement_age_ms = 100U,
        .maximum_coast_ms = 150U,
        .minimum_accepted_measurements = 2U,
    };
}

static void predict_each_ms(
    Algorithm_BallModelInterpolator *interpolator,
    uint32_t first_ms,
    uint32_t last_ms,
    float rod_angle_deg,
    float chassis_acceleration_mm_s2)
{
    for (uint32_t now_ms = first_ms; now_ms <= last_ms; ++now_ms) {
        assert(Algorithm_BallModelInterpolator_Predict(
            interpolator,
            now_ms,
            rod_angle_deg,
            chassis_acceleration_mm_s2));
    }
}

static void seed_at_zero(Algorithm_BallModelInterpolator *interpolator)
{
    assert(Algorithm_BallModelInterpolator_CorrectPosition(
        interpolator, 0U, 0U, 0.0F));
}

static void test_solid_ball_model_prediction(void)
{
    Algorithm_BallModelInterpolator interpolator;
    Algorithm_BallModelInterpolator_State state;
    const Algorithm_BallModelInterpolator_Config config = default_config();

    assert(Algorithm_BallModelInterpolator_Init(
        &interpolator, &config, 0U));
    seed_at_zero(&interpolator);
    predict_each_ms(&interpolator, 1U, 100U, 0.0F, 140.0F);
    assert(Algorithm_BallModelInterpolator_GetState(
        &interpolator, &state));
    /* -(5/7)*140 = -100 mm/s^2 for a solid sphere. */
    assert(close_enough(state.position_mm, -0.5F, 0.0002F));
    assert(close_enough(state.velocity_mm_s, -10.0F, 0.0002F));

    Algorithm_BallModelInterpolator_Reset(&interpolator, 100U);
    assert(Algorithm_BallModelInterpolator_CorrectPosition(
        &interpolator, 100U, 0U, 0.0F));
    predict_each_ms(&interpolator, 101U, 110U, 30.0F, 0.0F);
    assert(Algorithm_BallModelInterpolator_GetState(
        &interpolator, &state));
    /* -(5/7)*9800*sin(30 deg) = -3500 mm/s^2. */
    assert(close_enough(state.position_mm, -0.175F, 0.0005F));
    assert(close_enough(state.velocity_mm_s, -35.0F, 0.001F));
}

static void test_delayed_correction_matches_on_time_constant_model(void)
{
    Algorithm_BallModelInterpolator reference;
    Algorithm_BallModelInterpolator delayed;
    Algorithm_BallModelInterpolator_State reference_state;
    Algorithm_BallModelInterpolator_State delayed_state;
    const Algorithm_BallModelInterpolator_Config config = default_config();

    assert(Algorithm_BallModelInterpolator_Init(&reference, &config, 0U));
    assert(Algorithm_BallModelInterpolator_Init(&delayed, &config, 0U));
    seed_at_zero(&reference);
    seed_at_zero(&delayed);

    predict_each_ms(&reference, 1U, 80U, 0.0F, 0.0F);
    assert(Algorithm_BallModelInterpolator_CorrectPosition(
        &reference, 80U, 0U, 10.0F));
    predict_each_ms(&reference, 81U, 100U, 0.0F, 0.0F);

    predict_each_ms(&delayed, 1U, 100U, 0.0F, 0.0F);
    assert(Algorithm_BallModelInterpolator_CorrectPosition(
        &delayed, 100U, 20U, 10.0F));

    assert(Algorithm_BallModelInterpolator_GetState(
        &reference, &reference_state));
    assert(Algorithm_BallModelInterpolator_GetState(
        &delayed, &delayed_state));
    assert(close_enough(reference_state.position_mm, 5.5F, 0.0001F));
    assert(close_enough(reference_state.velocity_mm_s, 25.0F, 0.0001F));
    assert(close_enough(
        delayed_state.position_mm, reference_state.position_mm, 0.0001F));
    assert(close_enough(
        delayed_state.velocity_mm_s, reference_state.velocity_mm_s,
        0.0001F));
    /* alpha<1 proves a normal frame does not hard-reset x to the sample. */
    assert(delayed_state.position_mm < 10.0F);
}

static void test_delayed_first_measurement_uses_model(void)
{
    Algorithm_BallModelInterpolator interpolator;
    Algorithm_BallModelInterpolator_State state;
    Algorithm_BallModelInterpolator_Config config = default_config();

    config.minimum_accepted_measurements = 1U;
    assert(Algorithm_BallModelInterpolator_Init(
        &interpolator, &config, 0U));
    assert(Algorithm_BallModelInterpolator_Predict(
        &interpolator, 20U, 0.0F, 140.0F));
    assert(Algorithm_BallModelInterpolator_CorrectPosition(
        &interpolator, 20U, 10U, 5.0F));
    assert(Algorithm_BallModelInterpolator_GetState(
        &interpolator, &state));
    assert(close_enough(state.position_mm, 4.995F, 0.0001F));
    assert(close_enough(state.velocity_mm_s, -1.0F, 0.0001F));
    assert(Algorithm_BallModelInterpolator_IsTrusted(&interpolator, 20U));
}

static void test_absolute_innovation_gate_is_non_destructive(void)
{
    Algorithm_BallModelInterpolator interpolator;
    Algorithm_BallModelInterpolator_State before;
    Algorithm_BallModelInterpolator_State after;
    Algorithm_BallModelInterpolator_Diagnostics diagnostics;
    const Algorithm_BallModelInterpolator_Config config = default_config();

    assert(Algorithm_BallModelInterpolator_Init(
        &interpolator, &config, 0U));
    /* The first measurement seeds even though its absolute value exceeds the
     * innovation gate; there is no established prediction to compare yet. */
    assert(Algorithm_BallModelInterpolator_CorrectPosition(
        &interpolator, 0U, 0U, 50.0F));
    predict_each_ms(&interpolator, 1U, 10U, 0.0F, 0.0F);
    assert(Algorithm_BallModelInterpolator_GetState(
        &interpolator, &before));

    assert(Algorithm_BallModelInterpolator_CorrectPosition(
        &interpolator, 10U, 0U, 63.0F));
    assert(Algorithm_BallModelInterpolator_GetState(
        &interpolator, &after));
    assert(close_enough(after.position_mm, before.position_mm, 0.0001F));
    assert(close_enough(after.velocity_mm_s, before.velocity_mm_s, 0.0001F));
    assert(Algorithm_BallModelInterpolator_GetDiagnostics(
        &interpolator, &diagnostics));
    assert(diagnostics.innovation_reject_count == 1U);
    assert(diagnostics.accepted_measurements_since_reset == 1U);
    assert(!diagnostics.last_correction_accepted);

    assert(Algorithm_BallModelInterpolator_CorrectPosition(
        &interpolator, 10U, 0U, 61.0F));
    assert(Algorithm_BallModelInterpolator_GetDiagnostics(
        &interpolator, &diagnostics));
    assert(diagnostics.innovation_reject_count == 1U);
    assert(diagnostics.accepted_measurements_since_reset == 2U);
    assert(diagnostics.last_correction_accepted);
}

static void test_stale_order_and_coast_trust(void)
{
    Algorithm_BallModelInterpolator interpolator;
    Algorithm_BallModelInterpolator_State before;
    Algorithm_BallModelInterpolator_State after;
    Algorithm_BallModelInterpolator_Diagnostics diagnostics;
    const Algorithm_BallModelInterpolator_Config config = default_config();

    assert(Algorithm_BallModelInterpolator_Init(
        &interpolator, &config, 0U));
    seed_at_zero(&interpolator);
    predict_each_ms(&interpolator, 1U, 20U, 0.0F, 0.0F);
    assert(Algorithm_BallModelInterpolator_CorrectPosition(
        &interpolator, 20U, 5U, 1.0F));
    assert(Algorithm_BallModelInterpolator_IsTrusted(&interpolator, 20U));
    assert(Algorithm_BallModelInterpolator_GetState(
        &interpolator, &before));

    /* Sample time 14 ms is older than the last accepted sample at 15 ms. */
    assert(Algorithm_BallModelInterpolator_CorrectPosition(
        &interpolator, 20U, 6U, 1.0F));
    assert(Algorithm_BallModelInterpolator_CorrectPosition(
        &interpolator, 20U, 101U, 1.0F));
    assert(Algorithm_BallModelInterpolator_GetState(
        &interpolator, &after));
    assert(close_enough(after.position_mm, before.position_mm, 0.0001F));
    assert(close_enough(after.velocity_mm_s, before.velocity_mm_s, 0.0001F));
    assert(Algorithm_BallModelInterpolator_GetDiagnostics(
        &interpolator, &diagnostics));
    assert(diagnostics.out_of_order_measurement_count == 1U);
    assert(diagnostics.stale_measurement_count == 1U);

    predict_each_ms(&interpolator, 21U, 165U, 0.0F, 0.0F);
    assert(Algorithm_BallModelInterpolator_IsTrusted(&interpolator, 165U));
    assert(Algorithm_BallModelInterpolator_Predict(
        &interpolator, 166U, 0.0F, 0.0F));
    assert(!Algorithm_BallModelInterpolator_IsTrusted(&interpolator, 166U));
    assert(Algorithm_BallModelInterpolator_GetState(
        &interpolator, &after));
}

static void test_gap_reset_and_state_clamps(void)
{
    Algorithm_BallModelInterpolator interpolator;
    Algorithm_BallModelInterpolator_State state;
    Algorithm_BallModelInterpolator_Diagnostics diagnostics;
    Algorithm_BallModelInterpolator_Config config = default_config();

    assert(Algorithm_BallModelInterpolator_Init(
        &interpolator, &config, 0U));
    seed_at_zero(&interpolator);
    assert(!Algorithm_BallModelInterpolator_Predict(
        &interpolator, 6U, 0.0F, 0.0F));
    assert(!Algorithm_BallModelInterpolator_GetState(
        &interpolator, &state));
    assert(Algorithm_BallModelInterpolator_GetDiagnostics(
        &interpolator, &diagnostics));
    assert(diagnostics.reset_count == 1U);

    config.maximum_absolute_position_mm = 0.2F;
    config.maximum_absolute_velocity_mm_s = 10.0F;
    assert(Algorithm_BallModelInterpolator_Init(
        &interpolator, &config, 0U));
    seed_at_zero(&interpolator);
    predict_each_ms(&interpolator, 1U, 20U, 0.0F, 28000.0F);
    assert(Algorithm_BallModelInterpolator_GetState(
        &interpolator, &state));
    assert(close_enough(state.position_mm, -0.2F, 0.0001F));
    assert(close_enough(state.velocity_mm_s, -10.0F, 0.0001F));
    assert(Algorithm_BallModelInterpolator_GetDiagnostics(
        &interpolator, &diagnostics));
    assert(diagnostics.state_clamp_count > 0U);
}

static void test_invalid_config_inputs_and_static_size(void)
{
    Algorithm_BallModelInterpolator interpolator;
    Algorithm_BallModelInterpolator_State state;
    Algorithm_BallModelInterpolator_Config config = default_config();

    config.position_alpha = 0.0F;
    assert(!Algorithm_BallModelInterpolator_Init(
        &interpolator, &config, 0U));
    config = default_config();
    config.velocity_beta = 1.01F;
    assert(!Algorithm_BallModelInterpolator_Init(
        &interpolator, &config, 0U));
    config = default_config();
    config.maximum_absolute_innovation_mm = 0.0F;
    assert(!Algorithm_BallModelInterpolator_Init(
        &interpolator, &config, 0U));

    config = default_config();
    assert(Algorithm_BallModelInterpolator_Init(
        &interpolator, &config, 0U));
    seed_at_zero(&interpolator);
    assert(!Algorithm_BallModelInterpolator_Predict(
        &interpolator, 1U, NAN, 0.0F));
    assert(!Algorithm_BallModelInterpolator_CorrectPosition(
        &interpolator, 0U, 0U, NAN));
    assert(Algorithm_BallModelInterpolator_GetState(
        &interpolator, &state));
    assert(sizeof(interpolator) <= 128U);
}

int main(void)
{
    test_solid_ball_model_prediction();
    test_delayed_correction_matches_on_time_constant_model();
    test_delayed_first_measurement_uses_model();
    test_absolute_innovation_gate_is_non_destructive();
    test_stale_order_and_coast_trust();
    test_gap_reset_and_state_clamps();
    test_invalid_config_inputs_and_static_size();
    puts("ball model interpolator tests: PASS");
    return 0;
}
