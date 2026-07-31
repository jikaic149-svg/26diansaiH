#include "algorithm_level_current_control.h"

#include <math.h>
#include <stddef.h>

#define LEVEL_CURRENT_CONTROL_PI      3.14159265358979323846F
#define LEVEL_CURRENT_CONTROL_TWO_PI  6.28318530717958647692F
#define LEVEL_CURRENT_CONTROL_RAD2DEG \
    (180.0F / LEVEL_CURRENT_CONTROL_PI)

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

static float absolute_value(float value)
{
    return (value < 0.0F) ? -value : value;
}

static float apply_deadband(float error, float deadband)
{
    if (error > deadband) {
        return error - deadband;
    }
    if (error < -deadband) {
        return error + deadband;
    }
    return 0.0F;
}

static float release_opposing_feedforward(
    float feedforward, float original_error, float deadband)
{
    float release_ratio;

    if ((feedforward * original_error) >= 0.0F) {
        return feedforward;
    }
    if ((deadband <= 0.0F) ||
        (absolute_value(original_error) >= deadband)) {
        return 0.0F;
    }

    release_ratio = absolute_value(original_error) / deadband;
    return feedforward * (1.0F - release_ratio);
}

static bool valid_single_turn_angle(float angle_rad)
{
    return isfinite(angle_rad) && (angle_rad >= 0.0F) &&
        (angle_rad <= LEVEL_CURRENT_CONTROL_TWO_PI);
}

static bool valid_config(
    const Algorithm_LevelCurrentControl_Config *config)
{
    return (config != NULL) &&
        isfinite(config->control_period_s) &&
        (config->control_period_s > 0.0F) &&
        isfinite(config->target_deg) &&
        isfinite(config->target_limit_deg) &&
        (config->target_limit_deg >= 0.0F) &&
        (config->target_limit_deg <= 180.0F) &&
        (absolute_value(config->target_deg) <= config->target_limit_deg) &&
        isfinite(config->hard_limit_deg) &&
        (config->hard_limit_deg > 0.0F) &&
        (config->hard_limit_deg <= 180.0F) &&
        (config->target_limit_deg <= config->hard_limit_deg) &&
        isfinite(config->kp_a_per_deg) &&
        (config->kp_a_per_deg >= 0.0F) &&
        isfinite(config->ki_a_per_deg_s) &&
        (config->ki_a_per_deg_s >= 0.0F) &&
        isfinite(config->kd_a_per_rpm) &&
        (config->kd_a_per_rpm >= 0.0F) &&
        isfinite(config->feedforward_current_a) &&
        isfinite(config->error_limit_deg) &&
        (config->error_limit_deg > 0.0F) &&
        isfinite(config->deadband_deg) &&
        (config->deadband_deg >= 0.0F) &&
        (config->deadband_deg <= config->error_limit_deg) &&
        isfinite(config->integral_zone_deg) &&
        (config->integral_zone_deg >= 0.0F) &&
        isfinite(config->integral_limit_a) &&
        (config->integral_limit_a >= 0.0F) &&
        isfinite(config->current_limit_a) &&
        (config->current_limit_a > 0.0F) &&
        (absolute_value(config->feedforward_current_a) <=
            config->current_limit_a) &&
        (config->integral_limit_a <= config->current_limit_a) &&
        isfinite(config->current_polarity) &&
        ((config->current_polarity == 1.0F) ||
         (config->current_polarity == -1.0F)) &&
        (config->raw_min_limit < config->raw_max_limit) &&
        isfinite(config->raw_recovery_current_a) &&
        (config->raw_recovery_current_a > 0.0F) &&
        (config->raw_recovery_current_a <= config->current_limit_a);
}

static void reset_runtime_state(Algorithm_LevelCurrentControl *controller)
{
    controller->integral_current_a = 0.0F;
}

bool Algorithm_LevelCurrentControl_Init(
    Algorithm_LevelCurrentControl *controller,
    const Algorithm_LevelCurrentControl_Config *config)
{
    if (controller != NULL) {
        *controller = (Algorithm_LevelCurrentControl){0};
    }
    if ((controller == NULL) || !valid_config(config)) {
        return false;
    }
    controller->config = *config;
    controller->initialized = true;
    return true;
}

bool Algorithm_LevelCurrentControl_CaptureZero(
    Algorithm_LevelCurrentControl *controller,
    float absolute_angle_rad)
{
    if ((controller == NULL) || !controller->initialized ||
        controller->zero_captured ||
        !valid_single_turn_angle(absolute_angle_rad)) {
        return false;
    }
    controller->zero_angle_rad =
        (absolute_angle_rad == LEVEL_CURRENT_CONTROL_TWO_PI) ?
        0.0F : absolute_angle_rad;
    controller->zero_captured = true;
    return true;
}

bool Algorithm_LevelCurrentControl_HasZero(
    const Algorithm_LevelCurrentControl *controller)
{
    return (controller != NULL) && controller->initialized &&
        controller->zero_captured;
}

bool Algorithm_LevelCurrentControl_Start(
    Algorithm_LevelCurrentControl *controller)
{
    if ((controller == NULL) || !controller->initialized ||
        !controller->zero_captured) {
        return false;
    }
    reset_runtime_state(controller);
    controller->active = true;
    return true;
}

bool Algorithm_LevelCurrentControl_SetTargetDeg(
    Algorithm_LevelCurrentControl *controller,
    float target_deg)
{
    if ((controller == NULL) || !controller->initialized ||
        !isfinite(target_deg) ||
        (absolute_value(target_deg) > controller->config.target_limit_deg)) {
        return false;
    }
    controller->config.target_deg = target_deg;
    reset_runtime_state(controller);
    return true;
}

bool Algorithm_LevelCurrentControl_SetTrackingTargetDeg(
    Algorithm_LevelCurrentControl *controller,
    float target_deg)
{
    if ((controller == NULL) || !controller->initialized ||
        !isfinite(target_deg) ||
        (absolute_value(target_deg) > controller->config.target_limit_deg)) {
        return false;
    }
    controller->config.target_deg = target_deg;
    return true;
}

void Algorithm_LevelCurrentControl_Stop(
    Algorithm_LevelCurrentControl *controller)
{
    if (controller == NULL) {
        return;
    }
    controller->active = false;
    reset_runtime_state(controller);
}

bool Algorithm_LevelCurrentControl_IsActive(
    const Algorithm_LevelCurrentControl *controller)
{
    return (controller != NULL) && controller->initialized &&
        controller->active;
}

bool Algorithm_LevelCurrentControl_GetRelativeAngleDeg(
    const Algorithm_LevelCurrentControl *controller,
    float absolute_angle_rad,
    float *relative_angle_deg)
{
    float delta_rad;

    if ((controller == NULL) || !controller->initialized ||
        !controller->zero_captured ||
        !valid_single_turn_angle(absolute_angle_rad) ||
        (relative_angle_deg == NULL)) {
        return false;
    }
    if (absolute_angle_rad == LEVEL_CURRENT_CONTROL_TWO_PI) {
        absolute_angle_rad = 0.0F;
    }
    delta_rad = absolute_angle_rad - controller->zero_angle_rad;
    if (delta_rad > LEVEL_CURRENT_CONTROL_PI) {
        delta_rad -= LEVEL_CURRENT_CONTROL_TWO_PI;
    }
    else if (delta_rad < -LEVEL_CURRENT_CONTROL_PI) {
        delta_rad += LEVEL_CURRENT_CONTROL_TWO_PI;
    }
    *relative_angle_deg = delta_rad * LEVEL_CURRENT_CONTROL_RAD2DEG;
    return true;
}

bool Algorithm_LevelCurrentControl_ComputeRawRecoveryCurrent(
    Algorithm_LevelCurrentControl *controller,
    uint16_t raw,
    float *current_a,
    bool *recovery_active)
{
    float raw_direction = 0.0F;

    if (current_a != NULL) {
        *current_a = 0.0F;
    }
    if (recovery_active != NULL) {
        *recovery_active = false;
    }
    if ((controller == NULL) || !controller->initialized ||
        !controller->active || (current_a == NULL) ||
        (recovery_active == NULL)) {
        return false;
    }

    if (raw < controller->config.raw_min_limit) {
        raw_direction = 1.0F;
    }
    else if (raw > controller->config.raw_max_limit) {
        raw_direction = -1.0F;
    }
    else {
        return true;
    }

    reset_runtime_state(controller);
    *current_a = raw_direction *
        controller->config.raw_recovery_current_a *
        controller->config.current_polarity;
    *recovery_active = true;
    return true;
}

bool Algorithm_LevelCurrentControl_ApplyMinimumCurrent(
    const Algorithm_LevelCurrentControl *controller,
    int32_t raw_error,
    float minimum_current_a,
    float *current_a)
{
    if ((controller == NULL) || !controller->initialized ||
        !isfinite(minimum_current_a) || (minimum_current_a <= 0.0F) ||
        (minimum_current_a > controller->config.current_limit_a) ||
        (current_a == NULL) || !isfinite(*current_a)) {
        return false;
    }
    if ((raw_error == 0) ||
        (absolute_value(*current_a) >= minimum_current_a)) {
        return true;
    }
    if (*current_a > 0.0F) {
        *current_a = minimum_current_a;
    }
    else if (*current_a < 0.0F) {
        *current_a = -minimum_current_a;
    }
    else {
        *current_a = ((raw_error > 0) ? 1.0F : -1.0F) *
            controller->config.current_polarity * minimum_current_a;
    }
    return true;
}

bool Algorithm_LevelCurrentControl_Update(
    Algorithm_LevelCurrentControl *controller,
    float absolute_angle_rad,
    float speed_rpm,
    float *current_a)
{
    float angle_deg;
    float original_error;
    float controlled_error;
    float applied_feedforward;
    float effort;

    if (current_a != NULL) {
        *current_a = 0.0F;
    }
    if ((controller == NULL) || !controller->initialized ||
        !controller->active || !isfinite(speed_rpm) ||
        (current_a == NULL) ||
        !Algorithm_LevelCurrentControl_GetRelativeAngleDeg(
            controller, absolute_angle_rad, &angle_deg)) {
        return false;
    }
    if (absolute_value(angle_deg) > controller->config.hard_limit_deg) {
        reset_runtime_state(controller);
        return false;
    }

    original_error = controller->config.target_deg - angle_deg;
    controlled_error = apply_deadband(
        limit_symmetric(original_error, controller->config.error_limit_deg),
        controller->config.deadband_deg);
    applied_feedforward = release_opposing_feedforward(
        controller->config.feedforward_current_a,
        original_error,
        controller->config.deadband_deg);

    if (absolute_value(original_error) <=
        controller->config.integral_zone_deg) {
        controller->integral_current_a +=
            controller->config.ki_a_per_deg_s * original_error *
            controller->config.control_period_s;
        controller->integral_current_a = limit_symmetric(
            controller->integral_current_a,
            controller->config.integral_limit_a);
    }
    else {
        reset_runtime_state(controller);
    }

    effort = applied_feedforward +
        controller->config.kp_a_per_deg * controlled_error +
        controller->integral_current_a -
        controller->config.kd_a_per_rpm * speed_rpm;
    effort = limit_symmetric(effort, controller->config.current_limit_a);

    if (((angle_deg >= controller->config.hard_limit_deg) &&
         (effort > 0.0F)) ||
        ((angle_deg <= -controller->config.hard_limit_deg) &&
         (effort < 0.0F))) {
        effort = 0.0F;
        reset_runtime_state(controller);
    }

    *current_a = effort * controller->config.current_polarity;
    return true;
}
