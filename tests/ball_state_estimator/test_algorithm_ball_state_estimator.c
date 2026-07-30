#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "algorithm_ball_state_estimator.h"

static Algorithm_BallStateEstimator_Config default_config(void)
{
    return (Algorithm_BallStateEstimator_Config){
        .measurement_polarity = 1.0F,
        .gravity_mm_s2 = 9807.0F,
        .inertia_ratio = 0.4F,
        .position_measurement_variance_mm2 = 4.0F,
        .model_acceleration_noise_density_mm2_s3 = 10000.0F,
        .bias_walk_variance_mm2_s4_per_s = 100.0F,
        .initial_position_variance_mm2 = 25.0F,
        .initial_velocity_variance_mm2_s2 = 90000.0F,
        .initial_bias_variance_mm2_s4 = 250000.0F,
        .innovation_gate_sigma = 5.0F,
        .maximum_absolute_position_mm = 200.0F,
        .maximum_absolute_velocity_mm_s = 3000.0F,
        .maximum_absolute_bias_mm_s2 = 3000.0F,
        .trusted_position_variance_mm2 = 100.0F,
        .trusted_velocity_variance_mm2_s2 = 100000.0F,
        .maximum_prediction_step_ms = 10U,
        .maximum_measurement_age_ms = 100U,
        .minimum_accepted_measurements = 3U,
    };
}

static bool close_enough(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static void seed(Algorithm_BallStateEstimator *estimator)
{
    assert(Algorithm_BallStateEstimator_CorrectPosition(
        estimator, 0U, 0U, 0.0F));
}

static void test_rigid_model_prediction(void)
{
    Algorithm_BallStateEstimator estimator;
    Algorithm_BallStateEstimator_State state;
    const Algorithm_BallStateEstimator_Config config = default_config();
    const float theta_deg = 1.0F;
    const float theta_rad = theta_deg *
        (3.14159265358979323846F / 180.0F);
    const float acceleration = -(5.0F / 7.0F) *
        config.gravity_mm_s2 * sinf(theta_rad);

    assert(Algorithm_BallStateEstimator_Init(&estimator, &config, 0U));
    seed(&estimator);
    assert(Algorithm_BallStateEstimator_Predict(
        &estimator, 10U, theta_deg, 0.0F));
    assert(Algorithm_BallStateEstimator_GetState(
        &estimator, &state, NULL));
    assert(close_enough(state.velocity_mm_s, acceleration * 0.010F, 0.001F));
    assert(close_enough(
        state.position_mm, 0.5F * acceleration * 0.010F * 0.010F,
        0.001F));
}

static void test_delayed_correction_matches_on_time_replay(void)
{
    Algorithm_BallStateEstimator on_time;
    Algorithm_BallStateEstimator delayed;
    Algorithm_BallStateEstimator_State on_time_state;
    Algorithm_BallStateEstimator_State delayed_state;
    Algorithm_BallStateEstimator_Covariance on_time_covariance;
    Algorithm_BallStateEstimator_Covariance delayed_covariance;
    const Algorithm_BallStateEstimator_Config config = default_config();

    assert(Algorithm_BallStateEstimator_Init(&on_time, &config, 0U));
    assert(Algorithm_BallStateEstimator_Init(&delayed, &config, 0U));
    seed(&on_time);
    seed(&delayed);

    for (uint32_t now_ms = 2U; now_ms <= 10U; now_ms += 2U) {
        assert(Algorithm_BallStateEstimator_Predict(
            &on_time, now_ms, 0.0F, 0.0F));
        assert(Algorithm_BallStateEstimator_Predict(
            &delayed, now_ms, 0.0F, 0.0F));
        if (now_ms == 4U) {
            assert(Algorithm_BallStateEstimator_CorrectPosition(
                &on_time, now_ms, 0U, 1.0F));
        }
    }
    assert(Algorithm_BallStateEstimator_CorrectPosition(
        &delayed, 10U, 6U, 1.0F));
    assert(Algorithm_BallStateEstimator_GetState(
        &on_time, &on_time_state, &on_time_covariance));
    assert(Algorithm_BallStateEstimator_GetState(
        &delayed, &delayed_state, &delayed_covariance));
    assert(close_enough(
        delayed_state.position_mm, on_time_state.position_mm, 0.00001F));
    assert(close_enough(
        delayed_state.velocity_mm_s, on_time_state.velocity_mm_s, 0.00001F));
    assert(close_enough(delayed_state.acceleration_bias_mm_s2,
        on_time_state.acceleration_bias_mm_s2, 0.00001F));
    assert(close_enough(
        delayed_covariance.p00, on_time_covariance.p00, 0.00001F));
    assert(close_enough(
        delayed_covariance.p11, on_time_covariance.p11, 0.001F));
}

static void test_mid_step_delayed_correction_matches_split_reference(void)
{
    Algorithm_BallStateEstimator reference;
    Algorithm_BallStateEstimator delayed;
    Algorithm_BallStateEstimator_State reference_state;
    Algorithm_BallStateEstimator_State delayed_state;
    Algorithm_BallStateEstimator_Covariance reference_covariance;
    Algorithm_BallStateEstimator_Covariance delayed_covariance;
    const Algorithm_BallStateEstimator_Config config = default_config();

    assert(Algorithm_BallStateEstimator_Init(&reference, &config, 0U));
    assert(Algorithm_BallStateEstimator_Init(&delayed, &config, 0U));
    seed(&reference);
    seed(&delayed);

    assert(Algorithm_BallStateEstimator_Predict(
        &reference, 2U, 0.0F, 50.0F));
    assert(Algorithm_BallStateEstimator_Predict(
        &reference, 3U, 0.0F, 50.0F));
    assert(Algorithm_BallStateEstimator_CorrectPosition(
        &reference, 3U, 0U, 1.0F));
    for (uint32_t now_ms = 4U; now_ms <= 10U; now_ms += 2U) {
        assert(Algorithm_BallStateEstimator_Predict(
            &reference, now_ms, 0.0F, 50.0F));
    }

    for (uint32_t now_ms = 2U; now_ms <= 10U; now_ms += 2U) {
        assert(Algorithm_BallStateEstimator_Predict(
            &delayed, now_ms, 0.0F, 50.0F));
    }
    assert(Algorithm_BallStateEstimator_CorrectPosition(
        &delayed, 10U, 7U, 1.0F));
    assert(Algorithm_BallStateEstimator_GetState(
        &reference, &reference_state, &reference_covariance));
    assert(Algorithm_BallStateEstimator_GetState(
        &delayed, &delayed_state, &delayed_covariance));
    assert(close_enough(
        delayed_state.position_mm, reference_state.position_mm, 0.00001F));
    assert(close_enough(
        delayed_state.velocity_mm_s, reference_state.velocity_mm_s, 0.00001F));
    assert(close_enough(delayed_covariance.p00,
        reference_covariance.p00, 0.0001F));
    assert(close_enough(delayed_covariance.p11,
        reference_covariance.p11, 0.01F));
}

static void test_delayed_first_measurement_is_propagated_to_now(void)
{
    Algorithm_BallStateEstimator estimator;
    Algorithm_BallStateEstimator_State state;
    const Algorithm_BallStateEstimator_Config config = default_config();

    assert(Algorithm_BallStateEstimator_Init(&estimator, &config, 0U));
    assert(Algorithm_BallStateEstimator_Predict(
        &estimator, 20U, 0.0F, 140.0F));
    assert(Algorithm_BallStateEstimator_CorrectPosition(
        &estimator, 20U, 10U, 5.0F));
    assert(Algorithm_BallStateEstimator_GetState(&estimator, &state, NULL));
    /* Forward-positive a_fwd gives a_ball=-(5/7)*140=-100 mm/s^2. */
    assert(close_enough(state.position_mm, 4.995F, 0.0001F));
    assert(close_enough(state.velocity_mm_s, -1.0F, 0.0001F));
    assert(estimator.history_count == 2U);
}

static void test_polarity_gate_and_trust(void)
{
    Algorithm_BallStateEstimator estimator;
    Algorithm_BallStateEstimator_State state;
    Algorithm_BallStateEstimator_Config config = default_config();

    config.measurement_polarity = -1.0F;
    assert(Algorithm_BallStateEstimator_Init(&estimator, &config, 0U));
    assert(Algorithm_BallStateEstimator_CorrectPosition(
        &estimator, 0U, 0U, 10.0F));
    assert(Algorithm_BallStateEstimator_GetState(&estimator, &state, NULL));
    assert(close_enough(state.position_mm, -10.0F, 0.001F));
    assert(!Algorithm_BallStateEstimator_IsTrusted(&estimator, 0U));

    assert(Algorithm_BallStateEstimator_Predict(
        &estimator, 2U, 0.0F, 0.0F));
    assert(Algorithm_BallStateEstimator_CorrectPosition(
        &estimator, 2U, 0U, 10.0F));
    assert(Algorithm_BallStateEstimator_Predict(
        &estimator, 4U, 0.0F, 0.0F));
    assert(Algorithm_BallStateEstimator_CorrectPosition(
        &estimator, 4U, 0U, 10.0F));
    assert(Algorithm_BallStateEstimator_IsTrusted(&estimator, 4U));

    assert(Algorithm_BallStateEstimator_Predict(
        &estimator, 6U, 0.0F, 0.0F));
    assert(Algorithm_BallStateEstimator_CorrectPosition(
        &estimator, 6U, 0U, -190.0F));
    assert(estimator.innovation_reject_count == 1U);
    assert(Algorithm_BallStateEstimator_GetState(&estimator, &state, NULL));
    assert(fabsf(state.position_mm + 10.0F) < 1.0F);
}

static void test_prediction_gap_and_stale_history(void)
{
    Algorithm_BallStateEstimator estimator;
    Algorithm_BallStateEstimator_Config config = default_config();

    config.maximum_measurement_age_ms = 200U;

    assert(Algorithm_BallStateEstimator_Init(&estimator, &config, 0U));
    seed(&estimator);
    assert(!Algorithm_BallStateEstimator_Predict(
        &estimator, 11U, 0.0F, 0.0F));
    assert(!estimator.state_valid);
    assert(estimator.reset_count == 1U);

    assert(Algorithm_BallStateEstimator_CorrectPosition(
        &estimator, 11U, 0U, 0.0F));
    for (uint32_t now_ms = 13U; now_ms <= 151U; now_ms += 2U) {
        assert(Algorithm_BallStateEstimator_Predict(
            &estimator, now_ms, 0.0F, 0.0F));
    }
    assert(Algorithm_BallStateEstimator_CorrectPosition(
        &estimator, 151U, 150U, 0.0F));
    assert(estimator.stale_measurement_count == 1U);
}

int main(void)
{
    test_rigid_model_prediction();
    test_delayed_correction_matches_on_time_replay();
    test_mid_step_delayed_correction_matches_split_reference();
    test_delayed_first_measurement_is_propagated_to_now();
    test_polarity_gate_and_trust();
    test_prediction_gap_and_stale_history();
    puts("ball state estimator tests: PASS");
    return 0;
}
