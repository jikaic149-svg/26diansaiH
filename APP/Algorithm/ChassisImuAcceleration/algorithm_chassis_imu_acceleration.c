#include "algorithm_chassis_imu_acceleration.h"

#include <math.h>
#include <stddef.h>

#define CHASSIS_IMU_TWO_PI 6.28318530717958647692F
#define CHASSIS_IMU_BUTTERWORTH_Q 0.70710678118654752440F
#define CHASSIS_IMU_MAX_FILTER_STEPS 32.0F

static bool valid_config(
    const Algorithm_ChassisImuAcceleration_Config *config)
{
    return (config != NULL) &&
        (config->bias_calibration_samples > 0U) &&
        isfinite(config->notch_sample_frequency_hz) &&
        (config->notch_sample_frequency_hz > 0.0F) &&
        isfinite(config->acceleration_lowpass_cutoff_frequency_hz) &&
        (config->acceleration_lowpass_cutoff_frequency_hz > 0.0F) &&
        (config->acceleration_lowpass_cutoff_frequency_hz <
            (0.5F * config->notch_sample_frequency_hz)) &&
        isfinite(config->notch_center_frequency_hz) &&
        (config->notch_center_frequency_hz > 0.0F) &&
        (config->notch_center_frequency_hz <
            (0.5F * config->notch_sample_frequency_hz)) &&
        isfinite(config->notch_quality_factor) &&
        (config->notch_quality_factor > 0.0F) &&
        isfinite(config->maximum_absolute_acceleration_mm_s2) &&
        (config->maximum_absolute_acceleration_mm_s2 > 0.0F) &&
        (config->maximum_sample_step_ms > 0U) &&
        (config->notch_sample_frequency_hz *
            (float)config->maximum_sample_step_ms * 0.001F <=
            CHASSIS_IMU_MAX_FILTER_STEPS);
}

static bool configure_notch(Algorithm_ChassisImuAcceleration *processor)
{
    const float angular_frequency = CHASSIS_IMU_TWO_PI *
        processor->config.notch_center_frequency_hz /
        processor->config.notch_sample_frequency_hz;
    const float cosine = cosf(angular_frequency);
    const float alpha = sinf(angular_frequency) /
        (2.0F * processor->config.notch_quality_factor);
    const float denominator = 1.0F + alpha;

    if (!isfinite(cosine) || !isfinite(alpha) || (alpha <= 0.0F) ||
        !isfinite(denominator) || (denominator <= 0.0F)) {
        return false;
    }
    processor->notch_b0 = 1.0F / denominator;
    processor->notch_b1 = -2.0F * cosine / denominator;
    processor->notch_b2 = processor->notch_b0;
    processor->notch_a1 = processor->notch_b1;
    processor->notch_a2 = (1.0F - alpha) / denominator;
    return isfinite(processor->notch_b0) &&
        isfinite(processor->notch_b1) &&
        isfinite(processor->notch_b2) &&
        isfinite(processor->notch_a1) &&
        isfinite(processor->notch_a2) &&
        (fabsf(processor->notch_a2) < 1.0F);
}

static bool configure_lowpass(Algorithm_ChassisImuAcceleration *processor)
{
    const float angular_frequency = CHASSIS_IMU_TWO_PI *
        processor->config.acceleration_lowpass_cutoff_frequency_hz /
        processor->config.notch_sample_frequency_hz;
    const float cosine = cosf(angular_frequency);
    const float alpha = sinf(angular_frequency) /
        (2.0F * CHASSIS_IMU_BUTTERWORTH_Q);
    const float denominator = 1.0F + alpha;
    const float one_minus_cosine = 1.0F - cosine;

    if (!isfinite(cosine) || !isfinite(alpha) || (alpha <= 0.0F) ||
        !isfinite(denominator) || (denominator <= 0.0F) ||
        !isfinite(one_minus_cosine) || (one_minus_cosine <= 0.0F)) {
        return false;
    }
    processor->lowpass_b0 = 0.5F * one_minus_cosine / denominator;
    processor->lowpass_b1 = one_minus_cosine / denominator;
    processor->lowpass_b2 = processor->lowpass_b0;
    processor->lowpass_a1 = -2.0F * cosine / denominator;
    processor->lowpass_a2 = (1.0F - alpha) / denominator;
    return isfinite(processor->lowpass_b0) &&
        isfinite(processor->lowpass_b1) &&
        isfinite(processor->lowpass_b2) &&
        isfinite(processor->lowpass_a1) &&
        isfinite(processor->lowpass_a2) &&
        (fabsf(processor->lowpass_a2) < 1.0F);
}

static void seed_notch(
    Algorithm_ChassisImuAcceleration *processor,
    float acceleration_mm_s2)
{
    processor->notch_state_one =
        (1.0F - processor->notch_b0) * acceleration_mm_s2;
    processor->notch_state_two =
        (processor->notch_b2 - processor->notch_a2) *
        acceleration_mm_s2;
}

static void seed_lowpass(
    Algorithm_ChassisImuAcceleration *processor,
    float acceleration_mm_s2)
{
    processor->lowpass_state_one =
        (1.0F - processor->lowpass_b0) * acceleration_mm_s2;
    processor->lowpass_state_two =
        (processor->lowpass_b2 - processor->lowpass_a2) *
        acceleration_mm_s2;
}

static bool evaluate_filter_cascade(
    const Algorithm_ChassisImuAcceleration *processor,
    float previous_input_mm_s2,
    float current_input_mm_s2,
    uint32_t step_count,
    float *output_mm_s2,
    float *notch_state_one,
    float *notch_state_two,
    float *lowpass_state_one,
    float *lowpass_state_two)
{
    float local_notch_state_one;
    float local_notch_state_two;
    float local_lowpass_state_one;
    float local_lowpass_state_two;
    float output = current_input_mm_s2;

    if ((processor == NULL) || (output_mm_s2 == NULL) ||
        (notch_state_one == NULL) || (notch_state_two == NULL) ||
        (lowpass_state_one == NULL) || (lowpass_state_two == NULL) ||
        (step_count == 0U)) {
        return false;
    }
    local_notch_state_one = processor->notch_state_one;
    local_notch_state_two = processor->notch_state_two;
    local_lowpass_state_one = processor->lowpass_state_one;
    local_lowpass_state_two = processor->lowpass_state_two;
    for (uint32_t step = 0U; step < step_count; ++step) {
        const float input_mm_s2 = ((step + 1U) == step_count) ?
            current_input_mm_s2 : previous_input_mm_s2;
        const float notch_output =
            processor->notch_b0 * input_mm_s2 + local_notch_state_one;
        const float next_notch_state_one =
            processor->notch_b1 * input_mm_s2 -
            processor->notch_a1 * notch_output +
            local_notch_state_two;
        const float next_notch_state_two =
            processor->notch_b2 * input_mm_s2 -
            processor->notch_a2 * notch_output;
        const float lowpass_output =
            processor->lowpass_b0 * notch_output +
            local_lowpass_state_one;
        const float next_lowpass_state_one =
            processor->lowpass_b1 * notch_output -
            processor->lowpass_a1 * lowpass_output +
            local_lowpass_state_two;
        const float next_lowpass_state_two =
            processor->lowpass_b2 * notch_output -
            processor->lowpass_a2 * lowpass_output;

        if (!isfinite(notch_output) ||
            !isfinite(next_notch_state_one) ||
            !isfinite(next_notch_state_two) ||
            !isfinite(lowpass_output) ||
            !isfinite(next_lowpass_state_one) ||
            !isfinite(next_lowpass_state_two)) {
            return false;
        }
        output = lowpass_output;
        local_notch_state_one = next_notch_state_one;
        local_notch_state_two = next_notch_state_two;
        local_lowpass_state_one = next_lowpass_state_one;
        local_lowpass_state_two = next_lowpass_state_two;
    }
    *output_mm_s2 = output;
    *notch_state_one = local_notch_state_one;
    *notch_state_two = local_notch_state_two;
    *lowpass_state_one = local_lowpass_state_one;
    *lowpass_state_two = local_lowpass_state_two;
    return true;
}

bool Algorithm_ChassisImuAcceleration_Init(
    Algorithm_ChassisImuAcceleration *processor,
    const Algorithm_ChassisImuAcceleration_Config *config)
{
    if (processor != NULL) {
        *processor = (Algorithm_ChassisImuAcceleration){0};
    }
    if ((processor == NULL) || !valid_config(config)) {
        return false;
    }
    processor->config = *config;
    if (!configure_notch(processor) || !configure_lowpass(processor)) {
        *processor = (Algorithm_ChassisImuAcceleration){0};
        return false;
    }
    processor->initialized = true;
    return true;
}

void Algorithm_ChassisImuAcceleration_Reset(
    Algorithm_ChassisImuAcceleration *processor)
{
    Algorithm_ChassisImuAcceleration_Config config;

    if ((processor == NULL) || !processor->initialized) {
        return;
    }
    config = processor->config;
    (void)Algorithm_ChassisImuAcceleration_Init(processor, &config);
}

bool Algorithm_ChassisImuAcceleration_Update(
    Algorithm_ChassisImuAcceleration *processor,
    const Algorithm_ChassisImuAcceleration_Sample *sample)
{
    float longitudinal_specific_force;
    uint32_t delta_ms = 0U;
    bool sample_step_valid = false;

    if ((processor == NULL) || !processor->initialized ||
        (sample == NULL) ||
        !isfinite(sample->longitudinal_specific_force_mm_s2)) {
        if ((processor != NULL) && processor->initialized) {
            ++processor->invalid_sample_count;
        }
        return false;
    }

    longitudinal_specific_force =
        sample->longitudinal_specific_force_mm_s2;
    if (fabsf(longitudinal_specific_force) >
        processor->config.maximum_absolute_acceleration_mm_s2) {
        ++processor->invalid_sample_count;
        return false;
    }

    if (processor->sample_seen) {
        delta_ms = sample->sample_ms - processor->last_sample_ms;
        if (delta_ms == 0U) {
            ++processor->invalid_sample_count;
            return false;
        }
        sample_step_valid =
            delta_ms <= processor->config.maximum_sample_step_ms;
    }

    if (!processor->bias_ready) {
        if (!processor->sample_seen || !sample_step_valid) {
            processor->bias_sample_count = 1U;
            processor->bias_mm_s2 = longitudinal_specific_force;
        }
        else {
            ++processor->bias_sample_count;
            processor->bias_mm_s2 +=
                (longitudinal_specific_force - processor->bias_mm_s2) /
                (float)processor->bias_sample_count;
        }

        processor->last_sample_ms = sample->sample_ms;
        processor->sample_seen = true;
        ++processor->update_count;
        processor->bias_ready =
            processor->bias_sample_count >=
            processor->config.bias_calibration_samples;
        processor->output = (Algorithm_ChassisImuAcceleration_Output){
            .sample_ms = sample->sample_ms,
            .bias_mm_s2 = processor->bias_mm_s2,
            .bias_sample_count = processor->bias_sample_count,
            .bias_ready = processor->bias_ready,
            .valid = processor->bias_ready,
        };

        if (processor->bias_ready) {
            const float acceleration = longitudinal_specific_force -
                processor->bias_mm_s2;

            seed_notch(processor, 0.0F);
            seed_lowpass(processor, 0.0F);
            processor->output.unfiltered_acceleration_mm_s2 = acceleration;
            processor->output.filtered_acceleration_mm_s2 = 0.0F;
        }
        return true;
    }

    {
        const float acceleration = longitudinal_specific_force -
            processor->bias_mm_s2;
        float filtered;

        if (!isfinite(acceleration) ||
            (fabsf(acceleration) >
                processor->config.maximum_absolute_acceleration_mm_s2)) {
            ++processor->invalid_sample_count;
            return false;
        }

        if (sample_step_valid) {
            const float nominal_steps = (float)delta_ms * 0.001F *
                processor->config.notch_sample_frequency_hz;
            uint32_t filter_step_count =
                (uint32_t)(nominal_steps + 0.5F);
            float notch_state_one;
            float notch_state_two;
            float lowpass_state_one;
            float lowpass_state_two;

            if (filter_step_count == 0U) {
                filter_step_count = 1U;
            }
            if (!evaluate_filter_cascade(
                    processor,
                    processor->output.unfiltered_acceleration_mm_s2,
                    acceleration, filter_step_count, &filtered,
                    &notch_state_one, &notch_state_two,
                    &lowpass_state_one, &lowpass_state_two) ||
                (fabsf(filtered) >
                    processor->config.maximum_absolute_acceleration_mm_s2)) {
                ++processor->invalid_sample_count;
                return false;
            }
            processor->notch_state_one = notch_state_one;
            processor->notch_state_two = notch_state_two;
            processor->lowpass_state_one = lowpass_state_one;
            processor->lowpass_state_two = lowpass_state_two;
        }
        else {
            seed_notch(processor, acceleration);
            seed_lowpass(processor, acceleration);
            filtered = acceleration;
            ++processor->gap_reseed_count;
        }

        processor->output = (Algorithm_ChassisImuAcceleration_Output){
            .sample_ms = sample->sample_ms,
            .bias_mm_s2 = processor->bias_mm_s2,
            .unfiltered_acceleration_mm_s2 = acceleration,
            .filtered_acceleration_mm_s2 = filtered,
            .bias_sample_count = processor->bias_sample_count,
            .bias_ready = true,
            .valid = true,
        };
    }

    processor->last_sample_ms = sample->sample_ms;
    ++processor->update_count;
    return true;
}

bool Algorithm_ChassisImuAcceleration_GetOutput(
    const Algorithm_ChassisImuAcceleration *processor,
    Algorithm_ChassisImuAcceleration_Output *output)
{
    if ((processor == NULL) || !processor->initialized ||
        !processor->bias_ready || !processor->output.valid ||
        (output == NULL)) {
        return false;
    }
    *output = processor->output;
    return true;
}
