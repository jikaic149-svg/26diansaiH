#include "algorithm_chassis_imu_acceleration.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define TEST_PI 3.14159265358979323846F

static Algorithm_ChassisImuAcceleration_Config default_config(void)
{
    return (Algorithm_ChassisImuAcceleration_Config){
        .bias_calibration_samples = 300U,
        .notch_sample_frequency_hz = 1000.0F,
        .notch_center_frequency_hz = 14.0F,
        .notch_quality_factor = 0.7F,
        .acceleration_lowpass_cutoff_frequency_hz = 10.0F,
        .maximum_absolute_acceleration_mm_s2 = 40000.0F,
        .maximum_sample_step_ms = 5U,
    };
}

static void check_near(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

static void test_startup_mean_bias_and_output_gate(void)
{
    Algorithm_ChassisImuAcceleration processor;
    Algorithm_ChassisImuAcceleration_Output output;
    const Algorithm_ChassisImuAcceleration_Config config = default_config();

    assert(Algorithm_ChassisImuAcceleration_Init(&processor, &config));
    for (uint32_t sample_ms = 0U; sample_ms < 299U; ++sample_ms) {
        const float force = ((sample_ms & 1U) == 0U) ? 100.0F : 200.0F;

        assert(Algorithm_ChassisImuAcceleration_Update(
            &processor, &(Algorithm_ChassisImuAcceleration_Sample){
                .sample_ms = sample_ms,
                .longitudinal_specific_force_mm_s2 = force,
            }));
        assert(!Algorithm_ChassisImuAcceleration_GetOutput(
            &processor, &output));
    }
    assert(Algorithm_ChassisImuAcceleration_Update(
        &processor, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 299U,
            .longitudinal_specific_force_mm_s2 = 200.0F,
        }));
    assert(Algorithm_ChassisImuAcceleration_GetOutput(&processor, &output));
    assert(output.valid);
    assert(output.bias_ready);
    assert(output.bias_sample_count == 300U);
    assert(output.sample_ms == 299U);
    check_near(output.bias_mm_s2, 150.0F, 0.001F);
    check_near(output.unfiltered_acceleration_mm_s2, 50.0F, 0.001F);
    check_near(output.filtered_acceleration_mm_s2, 0.0F, 0.001F);
}

static void test_invalid_notch_configurations(void)
{
    Algorithm_ChassisImuAcceleration processor;
    Algorithm_ChassisImuAcceleration_Config config = default_config();

    config.notch_sample_frequency_hz = 0.0F;
    assert(!Algorithm_ChassisImuAcceleration_Init(&processor, &config));
    config = default_config();
    config.notch_sample_frequency_hz = NAN;
    assert(!Algorithm_ChassisImuAcceleration_Init(&processor, &config));

    config = default_config();
    config.notch_center_frequency_hz = 0.0F;
    assert(!Algorithm_ChassisImuAcceleration_Init(&processor, &config));
    config = default_config();
    config.notch_center_frequency_hz =
        0.5F * config.notch_sample_frequency_hz;
    assert(!Algorithm_ChassisImuAcceleration_Init(&processor, &config));
    config = default_config();
    config.notch_center_frequency_hz = NAN;
    assert(!Algorithm_ChassisImuAcceleration_Init(&processor, &config));

    config = default_config();
    config.notch_quality_factor = 0.0F;
    assert(!Algorithm_ChassisImuAcceleration_Init(&processor, &config));
    config = default_config();
    config.notch_quality_factor = NAN;
    assert(!Algorithm_ChassisImuAcceleration_Init(&processor, &config));

    config = default_config();
    config.acceleration_lowpass_cutoff_frequency_hz = 0.0F;
    assert(!Algorithm_ChassisImuAcceleration_Init(&processor, &config));
    config = default_config();
    config.acceleration_lowpass_cutoff_frequency_hz =
        0.5F * config.notch_sample_frequency_hz;
    assert(!Algorithm_ChassisImuAcceleration_Init(&processor, &config));
    config = default_config();
    config.acceleration_lowpass_cutoff_frequency_hz = NAN;
    assert(!Algorithm_ChassisImuAcceleration_Init(&processor, &config));

    config = default_config();
    config.maximum_sample_step_ms = 1000U;
    assert(!Algorithm_ChassisImuAcceleration_Init(&processor, &config));
}

static Algorithm_ChassisImuAcceleration_Output apply_single_step(
    float step_mm_s2)
{
    Algorithm_ChassisImuAcceleration processor;
    Algorithm_ChassisImuAcceleration_Output output;
    Algorithm_ChassisImuAcceleration_Config config = default_config();

    config.bias_calibration_samples = 1U;
    config.maximum_sample_step_ms = 5U;
    assert(Algorithm_ChassisImuAcceleration_Init(&processor, &config));
    assert(Algorithm_ChassisImuAcceleration_Update(
        &processor, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 0U,
            .longitudinal_specific_force_mm_s2 = 250.0F,
        }));
    assert(Algorithm_ChassisImuAcceleration_Update(
        &processor, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 1U,
            .longitudinal_specific_force_mm_s2 = 250.0F + step_mm_s2,
        }));
    assert(Algorithm_ChassisImuAcceleration_GetOutput(&processor, &output));
    return output;
}

static void test_positive_negative_steps_and_butterworth(void)
{
    const float notch_omega = 2.0F * TEST_PI * 14.0F / 1000.0F;
    const float notch_b0 = 1.0F /
        (1.0F + sinf(notch_omega) / (2.0F * 0.7F));
    const float lowpass_omega = 2.0F * TEST_PI * 10.0F / 1000.0F;
    const float lowpass_alpha = sinf(lowpass_omega) /
        (2.0F * 0.70710678118654752440F);
    const float lowpass_b0 = (1.0F - cosf(lowpass_omega)) /
        (2.0F * (1.0F + lowpass_alpha));
    const Algorithm_ChassisImuAcceleration_Output positive =
        apply_single_step(1000.0F);
    const Algorithm_ChassisImuAcceleration_Output negative =
        apply_single_step(-1000.0F);

    check_near(positive.bias_mm_s2, 250.0F, 0.001F);
    check_near(positive.unfiltered_acceleration_mm_s2, 1000.0F, 0.001F);
    check_near(positive.filtered_acceleration_mm_s2,
        1000.0F * notch_b0 * lowpass_b0, 0.01F);
    assert(positive.filtered_acceleration_mm_s2 > 0.0F);

    check_near(negative.bias_mm_s2, 250.0F, 0.001F);
    check_near(negative.unfiltered_acceleration_mm_s2, -1000.0F, 0.001F);
    check_near(negative.filtered_acceleration_mm_s2,
        -1000.0F * notch_b0 * lowpass_b0, 0.01F);
    assert(negative.filtered_acceleration_mm_s2 < 0.0F);
}

static void test_long_gap_reseeds_both_biquads(void)
{
    Algorithm_ChassisImuAcceleration processor;
    Algorithm_ChassisImuAcceleration_Output output;
    Algorithm_ChassisImuAcceleration_Config config = default_config();

    config.bias_calibration_samples = 1U;
    assert(Algorithm_ChassisImuAcceleration_Init(&processor, &config));
    assert(Algorithm_ChassisImuAcceleration_Update(
        &processor, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 0U,
            .longitudinal_specific_force_mm_s2 = 100.0F,
        }));
    assert(Algorithm_ChassisImuAcceleration_Update(
        &processor, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 1U,
            .longitudinal_specific_force_mm_s2 = 1100.0F,
        }));
    assert(Algorithm_ChassisImuAcceleration_Update(
        &processor, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 10U,
            .longitudinal_specific_force_mm_s2 = -400.0F,
        }));
    assert(Algorithm_ChassisImuAcceleration_GetOutput(&processor, &output));
    check_near(output.bias_mm_s2, 100.0F, 0.001F);
    check_near(output.unfiltered_acceleration_mm_s2, -500.0F, 0.001F);
    check_near(output.filtered_acceleration_mm_s2, -500.0F, 0.001F);

    assert(Algorithm_ChassisImuAcceleration_Update(
        &processor, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 11U,
            .longitudinal_specific_force_mm_s2 = -400.0F,
        }));
    assert(Algorithm_ChassisImuAcceleration_GetOutput(&processor, &output));
    check_near(output.filtered_acceleration_mm_s2, -500.0F, 0.01F);
}

static void test_sparse_update_matches_causal_dense_updates(void)
{
    Algorithm_ChassisImuAcceleration dense;
    Algorithm_ChassisImuAcceleration sparse;
    Algorithm_ChassisImuAcceleration_Output dense_output;
    Algorithm_ChassisImuAcceleration_Output sparse_output;
    Algorithm_ChassisImuAcceleration_Config config = default_config();

    config.bias_calibration_samples = 1U;
    assert(Algorithm_ChassisImuAcceleration_Init(&dense, &config));
    assert(Algorithm_ChassisImuAcceleration_Init(&sparse, &config));
    assert(Algorithm_ChassisImuAcceleration_Update(
        &dense, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 0U,
            .longitudinal_specific_force_mm_s2 = 0.0F,
        }));
    assert(Algorithm_ChassisImuAcceleration_Update(
        &sparse, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 0U,
            .longitudinal_specific_force_mm_s2 = 0.0F,
        }));
    for (uint32_t sample_ms = 1U; sample_ms < 5U; ++sample_ms) {
        assert(Algorithm_ChassisImuAcceleration_Update(
            &dense, &(Algorithm_ChassisImuAcceleration_Sample){
                .sample_ms = sample_ms,
                .longitudinal_specific_force_mm_s2 = 0.0F,
            }));
    }
    assert(Algorithm_ChassisImuAcceleration_Update(
        &dense, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 5U,
            .longitudinal_specific_force_mm_s2 = 1000.0F,
        }));
    assert(Algorithm_ChassisImuAcceleration_Update(
        &sparse, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 5U,
            .longitudinal_specific_force_mm_s2 = 1000.0F,
        }));
    assert(Algorithm_ChassisImuAcceleration_GetOutput(
        &dense, &dense_output));
    assert(Algorithm_ChassisImuAcceleration_GetOutput(
        &sparse, &sparse_output));
    check_near(sparse_output.filtered_acceleration_mm_s2,
        dense_output.filtered_acceleration_mm_s2, 0.001F);
    check_near(sparse.notch_state_one, dense.notch_state_one, 0.001F);
    check_near(sparse.notch_state_two, dense.notch_state_two, 0.001F);
    check_near(sparse.lowpass_state_one, dense.lowpass_state_one, 0.001F);
    check_near(sparse.lowpass_state_two, dense.lowpass_state_two, 0.001F);
}

static void test_notch_dc_gain(void)
{
    Algorithm_ChassisImuAcceleration processor;
    Algorithm_ChassisImuAcceleration_Output output;
    Algorithm_ChassisImuAcceleration_Config config = default_config();

    config.bias_calibration_samples = 1U;
    assert(Algorithm_ChassisImuAcceleration_Init(&processor, &config));
    assert(Algorithm_ChassisImuAcceleration_Update(
        &processor, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 0U,
            .longitudinal_specific_force_mm_s2 = 0.0F,
        }));
    for (uint32_t sample_ms = 1U; sample_ms <= 2000U; ++sample_ms) {
        assert(Algorithm_ChassisImuAcceleration_Update(
            &processor, &(Algorithm_ChassisImuAcceleration_Sample){
                .sample_ms = sample_ms,
                .longitudinal_specific_force_mm_s2 = 750.0F,
            }));
    }
    assert(Algorithm_ChassisImuAcceleration_GetOutput(&processor, &output));
    check_near(output.unfiltered_acceleration_mm_s2, 750.0F, 0.001F);
    check_near(output.filtered_acceleration_mm_s2, 750.0F, 0.1F);
}

static float measure_filter_sine_gain(
    float frequency_hz,
    float notch_center_frequency_hz,
    float notch_quality_factor,
    float lowpass_cutoff_frequency_hz)
{
    Algorithm_ChassisImuAcceleration processor;
    Algorithm_ChassisImuAcceleration_Output output;
    Algorithm_ChassisImuAcceleration_Config config = default_config();
    float input_energy = 0.0F;
    float output_energy = 0.0F;

    config.bias_calibration_samples = 1U;
    config.notch_center_frequency_hz = notch_center_frequency_hz;
    config.notch_quality_factor = notch_quality_factor;
    config.acceleration_lowpass_cutoff_frequency_hz =
        lowpass_cutoff_frequency_hz;
    config.maximum_sample_step_ms = 1U;
    assert(Algorithm_ChassisImuAcceleration_Init(&processor, &config));
    assert(Algorithm_ChassisImuAcceleration_Update(
        &processor, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 0U,
            .longitudinal_specific_force_mm_s2 = 0.0F,
        }));

    for (uint32_t sample_ms = 1U; sample_ms <= 12000U; ++sample_ms) {
        const float input = 1000.0F * sinf(
            2.0F * TEST_PI * frequency_hz *
            (float)sample_ms / config.notch_sample_frequency_hz);

        assert(Algorithm_ChassisImuAcceleration_Update(
            &processor, &(Algorithm_ChassisImuAcceleration_Sample){
                .sample_ms = sample_ms,
                .longitudinal_specific_force_mm_s2 = input,
            }));
        assert(Algorithm_ChassisImuAcceleration_GetOutput(
            &processor, &output));
        if (sample_ms > 4000U) {
            input_energy += input * input;
            output_energy += output.filtered_acceleration_mm_s2 *
                output.filtered_acceleration_mm_s2;
        }
    }
    assert(input_energy > 0.0F);
    return sqrtf(output_energy / input_energy);
}

static void test_notch_frequency_response(void)
{
    const float center_gain = measure_filter_sine_gain(
        14.0F, 14.0F, 0.7F, 10.0F);
    const float one_hz_gain = measure_filter_sine_gain(
        1.0F, 14.0F, 0.7F, 10.0F);
    const float three_hz_gain = measure_filter_sine_gain(
        3.0F, 14.0F, 0.7F, 10.0F);

    /* At least 20 dB attenuation at the configured center frequency. */
    assert(center_gain <= 0.10F);
    assert(one_hz_gain >= 0.95F);
    assert(one_hz_gain <= 1.05F);
    assert(three_hz_gain >= 0.90F);
    assert(three_hz_gain <= 1.05F);
}

static void test_butterworth_frequency_response(void)
{
    const float two_hz_gain = measure_filter_sine_gain(
        2.0F, 400.0F, 100.0F, 10.0F);
    const float cutoff_gain = measure_filter_sine_gain(
        10.0F, 400.0F, 100.0F, 10.0F);
    const float twenty_hz_gain = measure_filter_sine_gain(
        20.0F, 400.0F, 100.0F, 10.0F);

    assert(two_hz_gain >= 0.98F);
    assert(two_hz_gain <= 1.02F);
    assert(cutoff_gain >= 0.68F);
    assert(cutoff_gain <= 0.73F);
    assert(twenty_hz_gain >= 0.21F);
    assert(twenty_hz_gain <= 0.27F);
}

static void test_long_gap_restarts_bias_calibration(void)
{
    Algorithm_ChassisImuAcceleration processor;
    Algorithm_ChassisImuAcceleration_Output output;
    Algorithm_ChassisImuAcceleration_Config config = default_config();

    config.bias_calibration_samples = 3U;
    assert(Algorithm_ChassisImuAcceleration_Init(&processor, &config));
    assert(Algorithm_ChassisImuAcceleration_Update(
        &processor, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 0U,
            .longitudinal_specific_force_mm_s2 = 100.0F,
        }));
    assert(Algorithm_ChassisImuAcceleration_Update(
        &processor, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 1U,
            .longitudinal_specific_force_mm_s2 = 200.0F,
        }));
    assert(!Algorithm_ChassisImuAcceleration_GetOutput(
        &processor, &output));

    assert(Algorithm_ChassisImuAcceleration_Update(
        &processor, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 10U,
            .longitudinal_specific_force_mm_s2 = 300.0F,
        }));
    assert(Algorithm_ChassisImuAcceleration_Update(
        &processor, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 11U,
            .longitudinal_specific_force_mm_s2 = 300.0F,
        }));
    assert(!Algorithm_ChassisImuAcceleration_GetOutput(
        &processor, &output));
    assert(Algorithm_ChassisImuAcceleration_Update(
        &processor, &(Algorithm_ChassisImuAcceleration_Sample){
            .sample_ms = 12U,
            .longitudinal_specific_force_mm_s2 = 300.0F,
        }));
    assert(Algorithm_ChassisImuAcceleration_GetOutput(&processor, &output));
    assert(output.bias_ready);
    assert(output.bias_sample_count == 3U);
    check_near(output.bias_mm_s2, 300.0F, 0.001F);
    check_near(output.unfiltered_acceleration_mm_s2, 0.0F, 0.001F);
    check_near(output.filtered_acceleration_mm_s2, 0.0F, 0.001F);
}

int main(void)
{
    test_invalid_notch_configurations();
    test_startup_mean_bias_and_output_gate();
    test_positive_negative_steps_and_butterworth();
    test_long_gap_reseeds_both_biquads();
    test_sparse_update_matches_causal_dense_updates();
    test_notch_dc_gain();
    test_notch_frequency_response();
    test_butterworth_frequency_response();
    test_long_gap_restarts_bias_calibration();
    puts("chassis imu acceleration tests: PASS");
    return 0;
}
