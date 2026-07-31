#include "algorithm_ball_position_control.h"

#include <math.h>
#include <stddef.h>

#define BALL_POSITION_CONTROL_RAD_TO_DEG \
    (180.0F / 3.14159265358979323846F)

static float limit_symmetric(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static bool valid_config(
    const Algorithm_BallPositionControl_Config *config)
{
    return (config != NULL) &&
        isfinite(config->target_position_mm) &&
        isfinite(config->maximum_absolute_target_position_mm) &&
        (config->maximum_absolute_target_position_mm > 0.0F) &&
        (fabsf(config->target_position_mm) <=
            config->maximum_absolute_target_position_mm) &&
        isfinite(config->target_slew_rate_mm_s) &&
        (config->target_slew_rate_mm_s > 0.0F) &&
        isfinite(config->measurement_polarity) &&
        ((config->measurement_polarity == 1.0F) ||
         (config->measurement_polarity == -1.0F)) &&
        isfinite(config->position_gain_per_s2) &&
        (config->position_gain_per_s2 >= 0.0F) &&
        isfinite(config->position_integral_gain_per_s3) &&
        (config->position_integral_gain_per_s3 >= 0.0F) &&
        isfinite(config->velocity_gain_per_s) &&
        (config->velocity_gain_per_s >= 0.0F) &&
        isfinite(config->maximum_integral_acceleration_mm_s2) &&
        (config->maximum_integral_acceleration_mm_s2 >= 0.0F) &&
        ((config->position_integral_gain_per_s3 == 0.0F) ||
         (config->maximum_integral_acceleration_mm_s2 > 0.0F)) &&
        isfinite(config->velocity_filter_tau_s) &&
        (config->velocity_filter_tau_s >= 0.0F) &&
        isfinite(config->gravity_mm_s2) &&
        (config->gravity_mm_s2 > 0.0F) &&
        isfinite(config->inertia_ratio) &&
        (config->inertia_ratio >= 0.0F) &&
        isfinite(config->rod_angle_limit_deg) &&
        (config->rod_angle_limit_deg > 0.0F) &&
        (config->rod_angle_limit_deg < 90.0F) &&
        (config->maximum_velocity_dt_ms > 0U) &&
        (config->maximum_velocity_dt_ms <= UINT32_C(1000));
}

bool Algorithm_BallPositionControl_Init(
    Algorithm_BallPositionControl *controller,
    const Algorithm_BallPositionControl_Config *config)
{
    if (controller != NULL) {
        *controller = (Algorithm_BallPositionControl){0};
    }
    if ((controller == NULL) || !valid_config(config)) {
        return false;
    }
    controller->config = *config;
    controller->target_position_mm = config->target_position_mm;
    controller->target_command_mm = config->target_position_mm;
    controller->initialized = true;
    return true;
}

bool Algorithm_BallPositionControl_SetTargetPositionMm(
    Algorithm_BallPositionControl *controller,
    float target_position_mm)
{
    if ((controller == NULL) || !controller->initialized ||
        !isfinite(target_position_mm) ||
        (fabsf(target_position_mm) >
            controller->config.maximum_absolute_target_position_mm)) {
        return false;
    }
    controller->target_command_mm = target_position_mm;
    return true;
}

bool Algorithm_BallPositionControl_AdvanceTarget(
    Algorithm_BallPositionControl *controller,
    uint32_t elapsed_ms)
{
    float delta_mm;
    float maximum_step_mm;

    if ((controller == NULL) || !controller->initialized) {
        return false;
    }
    delta_mm = controller->target_command_mm -
        controller->target_position_mm;
    maximum_step_mm = controller->config.target_slew_rate_mm_s *
        ((float)elapsed_ms / 1000.0F);
    if (!isfinite(delta_mm) || !isfinite(maximum_step_mm)) {
        return false;
    }
    if (delta_mm > maximum_step_mm) {
        controller->target_position_mm += maximum_step_mm;
    }
    else if (delta_mm < -maximum_step_mm) {
        controller->target_position_mm -= maximum_step_mm;
    }
    else {
        controller->target_position_mm =
            controller->target_command_mm;
    }
    return isfinite(controller->target_position_mm);
}

bool Algorithm_BallPositionControl_ResetTargetPositionMm(
    Algorithm_BallPositionControl *controller,
    float target_position_mm)
{
    if (!Algorithm_BallPositionControl_SetTargetPositionMm(
            controller, target_position_mm)) {
        return false;
    }
    controller->target_position_mm = target_position_mm;
    return true;
}

bool Algorithm_BallPositionControl_SetGains(
    Algorithm_BallPositionControl *controller,
    float position_gain_per_s2,
    float position_integral_gain_per_s3,
    float velocity_gain_per_s)
{
    if ((controller == NULL) || !controller->initialized ||
        !isfinite(position_gain_per_s2) ||
        !isfinite(position_integral_gain_per_s3) ||
        !isfinite(velocity_gain_per_s) ||
        (position_gain_per_s2 < 0.0F) ||
        (position_integral_gain_per_s3 < 0.0F) ||
        (velocity_gain_per_s < 0.0F) ||
        ((position_integral_gain_per_s3 > 0.0F) &&
         (controller->config.maximum_integral_acceleration_mm_s2 <= 0.0F))) {
        return false;
    }
    controller->config.position_gain_per_s2 = position_gain_per_s2;
    controller->config.position_integral_gain_per_s3 =
        position_integral_gain_per_s3;
    controller->config.velocity_gain_per_s = velocity_gain_per_s;
    if (position_integral_gain_per_s3 == 0.0F) {
        controller->integral_acceleration_mm_s2 = 0.0F;
    }
    return true;
}

bool Algorithm_BallPositionControl_UpdateMeasurement(
    Algorithm_BallPositionControl *controller,
    uint32_t sample_ms,
    float camera_position_mm)
{
    float position_mm;

    if ((controller == NULL) || !controller->initialized ||
        !isfinite(camera_position_mm)) {
        return false;
    }
    position_mm = camera_position_mm *
        controller->config.measurement_polarity;
    if (!isfinite(position_mm)) {
        return false;
    }

    if (controller->measurement_seen) {
        const uint32_t delta_ms = sample_ms - controller->last_sample_ms;

        if ((delta_ms > 0U) &&
            (delta_ms <= controller->config.maximum_velocity_dt_ms)) {
            const float dt_s = (float)delta_ms / 1000.0F;
            const float raw_velocity =
                (position_mm - controller->position_mm) / dt_s;
            const float alpha = (controller->config.velocity_filter_tau_s <=
                    0.0F) ? 1.0F :
                dt_s / (controller->config.velocity_filter_tau_s + dt_s);

            if (!isfinite(raw_velocity) || !isfinite(alpha)) {
                return false;
            }
            controller->velocity_mm_s +=
                alpha * (raw_velocity - controller->velocity_mm_s);
            controller->velocity_valid = true;
            controller->integral_acceleration_mm_s2 +=
                controller->config.position_integral_gain_per_s3 *
                (controller->target_position_mm - position_mm) * dt_s;
            controller->integral_acceleration_mm_s2 = limit_symmetric(
                controller->integral_acceleration_mm_s2,
                controller->config.maximum_integral_acceleration_mm_s2);
        }
        else {
            controller->velocity_mm_s = 0.0F;
            controller->velocity_valid = false;
        }
    }
    else {
        controller->velocity_mm_s = 0.0F;
        controller->integral_acceleration_mm_s2 = 0.0F;
        controller->velocity_valid = false;
    }

    controller->position_mm = position_mm;
    controller->last_sample_ms = sample_ms;
    controller->measurement_seen = true;
    return true;
}

bool Algorithm_BallPositionControl_SetEstimatedState(
    Algorithm_BallPositionControl *controller,
    float position_mm,
    float velocity_mm_s)
{
    if ((controller == NULL) || !controller->initialized ||
        !isfinite(position_mm) || !isfinite(velocity_mm_s)) {
        return false;
    }
    controller->position_mm = position_mm;
    controller->velocity_mm_s = velocity_mm_s;
    controller->measurement_seen = true;
    controller->velocity_valid = true;
    return true;
}

void Algorithm_BallPositionControl_ResetMeasurement(
    Algorithm_BallPositionControl *controller)
{
    if ((controller == NULL) || !controller->initialized) {
        return;
    }
    controller->position_mm = 0.0F;
    controller->velocity_mm_s = 0.0F;
    controller->integral_acceleration_mm_s2 = 0.0F;
    controller->desired_acceleration_mm_s2 = 0.0F;
    controller->last_sample_ms = 0U;
    controller->measurement_seen = false;
    controller->velocity_valid = false;
}

bool Algorithm_BallPositionControl_ComputeRodAngleDeg(
    Algorithm_BallPositionControl *controller,
    float *rod_angle_deg)
{
    float desired_acceleration;
    float normalized;
    float angle;

    if (rod_angle_deg != NULL) {
        *rod_angle_deg = 0.0F;
    }
    if ((controller == NULL) || !controller->initialized ||
        !controller->measurement_seen || (rod_angle_deg == NULL)) {
        return false;
    }

    desired_acceleration = controller->config.position_gain_per_s2 *
        (controller->target_position_mm - controller->position_mm) +
        controller->integral_acceleration_mm_s2;
    if (controller->velocity_valid) {
        desired_acceleration -= controller->config.velocity_gain_per_s *
            controller->velocity_mm_s;
    }
    if (!isfinite(desired_acceleration)) {
        return false;
    }
    controller->desired_acceleration_mm_s2 = desired_acceleration;

    normalized = (1.0F + controller->config.inertia_ratio) *
        desired_acceleration / controller->config.gravity_mm_s2;
    normalized = limit_symmetric(normalized, 1.0F);
    /* Positive rod angle raises the execution end and accelerates the ball
     * toward the fixed pivot, hence the minus sign in the inverse model. */
    angle = -asinf(normalized) * BALL_POSITION_CONTROL_RAD_TO_DEG;
    if (!isfinite(angle)) {
        return false;
    }
    *rod_angle_deg = limit_symmetric(
        angle, controller->config.rod_angle_limit_deg);
    return true;
}
