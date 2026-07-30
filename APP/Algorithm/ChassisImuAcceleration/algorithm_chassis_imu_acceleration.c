#include "algorithm_chassis_imu_acceleration.h"

#include <math.h>
#include <stddef.h>

#define CHASSIS_IMU_TWO_PI 6.28318530717958647692F

static bool valid_config(
    const Algorithm_ChassisImuAcceleration_Config *config)
{
    return (config != NULL) &&
        (config->bias_calibration_samples > 0U) &&
        isfinite(config->acceleration_filter_stage_time_constant_s) &&
        (config->acceleration_filter_stage_time_constant_s >= 0.0F) &&
        isfinite(config->notch_sample_frequency_hz) &&
        (config->notch_sample_frequency_hz > 0.0F) &&
        isfinite(config->notch_center_frequency_hz) &&
        (config->notch_center_frequency_hz > 0.0F) &&
        (config->notch_center_frequency_hz <
            (0.5F * config->notch_sample_frequency_hz)) &&
        isfinite(config->notch_quality_factor) &&
        (config->notch_quality_factor > 0.0F) &&
        isfinite(config->maximum_absolute_acceleration_mm_s2) &&
        (config->maximum_absolute_acceleration_mm_s2 > 0.0F) &&
        (config->maximum_sample_step_ms > 0U);
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

static bool evaluate_notch(
    const Algorithm_ChassisImuAcceleration *processor,
    float input_mm_s2,
    uint32_t step_count,
    float *output_mm_s2,
    float *state_one,
    float *state_two)
{
    float local_state_one;
    float local_state_two;
    float output = input_mm_s2;

    if ((processor == NULL) || (output_mm_s2 == NULL) ||
        (state_one == NULL) || (state_two == NULL) ||
        (step_count == 0U)) {
        return false;
    }
    local_state_one = processor->notch_state_one;
    local_state_two = processor->notch_state_two;
    for (uint32_t step = 0U; step < step_count; ++step) {
        const float next_output =
            processor->notch_b0 * input_mm_s2 + local_state_one;
        const float next_state_one =
            processor->notch_b1 * input_mm_s2 -
            processor->notch_a1 * next_output + local_state_two;
        const float next_state_two =
            processor->notch_b2 * input_mm_s2 -
            processor->notch_a2 * next_output;

        if (!isfinite(next_output) || !isfinite(next_state_one) ||
            !isfinite(next_state_two)) {
            return false;
        }
        output = next_output;
        local_state_one = next_state_one;
        local_state_two = next_state_two;
    }
    *output_mm_s2 = output;
    *state_one = local_state_one;
    *state_two = local_state_two;
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
    if (!configure_notch(processor)) {
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
            processor->acceleration_filter_stage_one_mm_s2 = 0.0F;
            processor->output.unfiltered_acceleration_mm_s2 = acceleration;
            processor->output.filtered_acceleration_mm_s2 = 0.0F;
        }
        return true;
    }

    {
        const float acceleration = longitudinal_specific_force -
            processor->bias_mm_s2;
        float stage_one;
        float filtered;

        if (!isfinite(acceleration) ||
            (fabsf(acceleration) >
                processor->config.maximum_absolute_acceleration_mm_s2)) {
            ++processor->invalid_sample_count;
            return false;
        }

        if (sample_step_valid) {
            const float dt_s = (float)delta_ms * 0.001F;
            const float nominal_steps = dt_s *
                processor->config.notch_sample_frequency_hz;
            uint32_t notch_step_count =
                (uint32_t)(nominal_steps + 0.5F);
            float notch_output;
            float notch_state_one;
            float notch_state_two;
            float filter_gain;

            if (notch_step_count == 0U) {
                notch_step_count = 1U;
            }
            if (!evaluate_notch(
                    processor, acceleration, notch_step_count,
                    &notch_output, &notch_state_one, &notch_state_two) ||
                (fabsf(notch_output) >
                    processor->config.maximum_absolute_acceleration_mm_s2)) {
                ++processor->invalid_sample_count;
                return false;
            }
            filter_gain =
                (processor->config.
                    acceleration_filter_stage_time_constant_s == 0.0F)
                ? 1.0F
                : 1.0F - expf(-dt_s / processor->config.
                    acceleration_filter_stage_time_constant_s);
            stage_one = processor->acceleration_filter_stage_one_mm_s2;
            filtered = processor->output.filtered_acceleration_mm_s2;
            stage_one += filter_gain * (notch_output - stage_one);
            filtered += filter_gain * (stage_one - filtered);
            if (!isfinite(stage_one) || !isfinite(filtered) ||
                (fabsf(filtered) >
                    processor->config.maximum_absolute_acceleration_mm_s2)) {
                ++processor->invalid_sample_count;
                return false;
            }
            processor->notch_state_one = notch_state_one;
            processor->notch_state_two = notch_state_two;
        }
        else {
            seed_notch(processor, acceleration);
            stage_one = acceleration;
            filtered = acceleration;
            ++processor->gap_reseed_count;
        }

        processor->acceleration_filter_stage_one_mm_s2 = stage_one;
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
