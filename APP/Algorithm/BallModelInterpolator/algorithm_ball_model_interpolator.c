#include "algorithm_ball_model_interpolator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define BALL_MODEL_INTERPOLATOR_DEG_TO_RAD \
    (3.14159265358979323846F / 180.0F)
#define BALL_MODEL_INTERPOLATOR_HALF_TIME_RANGE UINT32_C(0x80000000)

static bool finite_positive(float value)
{
    return isfinite(value) && (value > 0.0F);
}

static bool finite_nonnegative(float value)
{
    return isfinite(value) && (value >= 0.0F);
}

static void increment_saturated(uint32_t *value)
{
    if (*value < UINT32_MAX) {
        ++(*value);
    }
}

static bool valid_config(
    const Algorithm_BallModelInterpolator_Config *config)
{
    return (config != NULL) && finite_positive(config->gravity_mm_s2) &&
        finite_nonnegative(config->inertia_ratio) &&
        finite_positive(config->position_alpha) &&
        (config->position_alpha <= 1.0F) &&
        finite_positive(config->velocity_beta) &&
        (config->velocity_beta <= 1.0F) &&
        finite_positive(config->maximum_absolute_innovation_mm) &&
        finite_positive(config->maximum_absolute_position_mm) &&
        finite_positive(config->maximum_absolute_velocity_mm_s) &&
        (config->maximum_prediction_step_ms > 0U) &&
        (config->maximum_prediction_step_ms <= UINT32_C(1000)) &&
        (config->maximum_measurement_age_ms > 0U) &&
        (config->maximum_measurement_age_ms <
            BALL_MODEL_INTERPOLATOR_HALF_TIME_RANGE) &&
        (config->maximum_coast_ms > 0U) &&
        (config->maximum_coast_ms <
            BALL_MODEL_INTERPOLATOR_HALF_TIME_RANGE) &&
        (config->minimum_accepted_measurements > 0U);
}

static bool valid_state(
    const Algorithm_BallModelInterpolator *interpolator,
    const Algorithm_BallModelInterpolator_State *state)
{
    return isfinite(state->position_mm) && isfinite(state->velocity_mm_s) &&
        (fabsf(state->position_mm) <=
            interpolator->config.maximum_absolute_position_mm) &&
        (fabsf(state->velocity_mm_s) <=
            interpolator->config.maximum_absolute_velocity_mm_s);
}

static bool clamp_state(
    Algorithm_BallModelInterpolator *interpolator,
    Algorithm_BallModelInterpolator_State *state)
{
    bool clamped = false;

    if (!isfinite(state->position_mm) || !isfinite(state->velocity_mm_s)) {
        return false;
    }
    if (state->position_mm >
        interpolator->config.maximum_absolute_position_mm) {
        state->position_mm =
            interpolator->config.maximum_absolute_position_mm;
        clamped = true;
    }
    else if (state->position_mm <
        -interpolator->config.maximum_absolute_position_mm) {
        state->position_mm =
            -interpolator->config.maximum_absolute_position_mm;
        clamped = true;
    }
    if (state->velocity_mm_s >
        interpolator->config.maximum_absolute_velocity_mm_s) {
        state->velocity_mm_s =
            interpolator->config.maximum_absolute_velocity_mm_s;
        clamped = true;
    }
    else if (state->velocity_mm_s <
        -interpolator->config.maximum_absolute_velocity_mm_s) {
        state->velocity_mm_s =
            -interpolator->config.maximum_absolute_velocity_mm_s;
        clamped = true;
    }
    if (clamped) {
        increment_saturated(&interpolator->diagnostics.state_clamp_count);
    }
    return true;
}

static void reset_runtime(
    Algorithm_BallModelInterpolator *interpolator,
    uint32_t now_ms,
    bool count_reset)
{
    memset(&interpolator->state, 0, sizeof(interpolator->state));
    interpolator->last_prediction_ms = now_ms;
    interpolator->last_accepted_measurement_time_ms = now_ms;
    interpolator->last_model_acceleration_mm_s2 = 0.0F;
    interpolator->diagnostics.last_innovation_mm = 0.0F;
    interpolator->diagnostics.accepted_measurements_since_reset = 0U;
    interpolator->diagnostics.last_correction_accepted = false;
    interpolator->state_valid = false;
    interpolator->model_input_valid = false;
    if (count_reset) {
        increment_saturated(&interpolator->diagnostics.reset_count);
    }
}

static bool time_is_strictly_after(uint32_t time_ms, uint32_t reference_ms)
{
    const uint32_t delta_ms = time_ms - reference_ms;

    return (delta_ms > 0U) &&
        (delta_ms < BALL_MODEL_INTERPOLATOR_HALF_TIME_RANGE);
}

bool Algorithm_BallModelInterpolator_Init(
    Algorithm_BallModelInterpolator *interpolator,
    const Algorithm_BallModelInterpolator_Config *config,
    uint32_t now_ms)
{
    if (interpolator != NULL) {
        memset(interpolator, 0, sizeof(*interpolator));
    }
    if ((interpolator == NULL) || !valid_config(config)) {
        return false;
    }
    interpolator->config = *config;
    interpolator->initialized = true;
    reset_runtime(interpolator, now_ms, false);
    return true;
}

void Algorithm_BallModelInterpolator_Reset(
    Algorithm_BallModelInterpolator *interpolator,
    uint32_t now_ms)
{
    if ((interpolator == NULL) || !interpolator->initialized) {
        return;
    }
    reset_runtime(interpolator, now_ms, true);
}

bool Algorithm_BallModelInterpolator_Predict(
    Algorithm_BallModelInterpolator *interpolator,
    uint32_t now_ms,
    float rod_angle_deg,
    float chassis_acceleration_mm_s2)
{
    uint32_t delta_ms;
    float theta_rad;
    float rolling_ratio;
    float model_acceleration_mm_s2;
    float dt_s;
    Algorithm_BallModelInterpolator_State next;

    if ((interpolator == NULL) || !interpolator->initialized ||
        !isfinite(rod_angle_deg) ||
        !isfinite(chassis_acceleration_mm_s2)) {
        return false;
    }
    theta_rad = rod_angle_deg * BALL_MODEL_INTERPOLATOR_DEG_TO_RAD;
    rolling_ratio = 1.0F / (1.0F + interpolator->config.inertia_ratio);
    model_acceleration_mm_s2 = -rolling_ratio *
        (interpolator->config.gravity_mm_s2 * sinf(theta_rad) +
         chassis_acceleration_mm_s2 * cosf(theta_rad));
    if (!isfinite(theta_rad) || !isfinite(rolling_ratio) ||
        !isfinite(model_acceleration_mm_s2)) {
        return false;
    }

    delta_ms = now_ms - interpolator->last_prediction_ms;
    interpolator->last_model_acceleration_mm_s2 =
        model_acceleration_mm_s2;
    interpolator->model_input_valid = true;
    if (!interpolator->state_valid) {
        interpolator->last_prediction_ms = now_ms;
        return true;
    }
    if (delta_ms == 0U) {
        return true;
    }
    if (delta_ms > interpolator->config.maximum_prediction_step_ms) {
        reset_runtime(interpolator, now_ms, true);
        return false;
    }

    dt_s = (float)delta_ms / 1000.0F;
    next = interpolator->state;
    next.position_mm += next.velocity_mm_s * dt_s +
        0.5F * model_acceleration_mm_s2 * dt_s * dt_s;
    next.velocity_mm_s += model_acceleration_mm_s2 * dt_s;
    if (!clamp_state(interpolator, &next) ||
        !valid_state(interpolator, &next)) {
        reset_runtime(interpolator, now_ms, true);
        return false;
    }
    interpolator->state = next;
    interpolator->last_prediction_ms = now_ms;
    increment_saturated(&interpolator->diagnostics.prediction_count);
    return true;
}

bool Algorithm_BallModelInterpolator_CorrectPosition(
    Algorithm_BallModelInterpolator *interpolator,
    uint32_t now_ms,
    uint32_t measurement_age_ms,
    float measured_position_mm)
{
    uint32_t measurement_time_ms;
    float age_s;
    Algorithm_BallModelInterpolator_State next;

    if ((interpolator == NULL) || !interpolator->initialized ||
        !isfinite(measured_position_mm) ||
        (fabsf(measured_position_mm) >
            interpolator->config.maximum_absolute_position_mm) ||
        (now_ms != interpolator->last_prediction_ms)) {
        return false;
    }
    interpolator->diagnostics.last_correction_accepted = false;
    increment_saturated(&interpolator->diagnostics.correction_count);
    if (measurement_age_ms >
        interpolator->config.maximum_measurement_age_ms) {
        increment_saturated(
            &interpolator->diagnostics.stale_measurement_count);
        return true;
    }
    if ((measurement_age_ms > 0U) &&
        !interpolator->model_input_valid) {
        return false;
    }

    measurement_time_ms = now_ms - measurement_age_ms;
    age_s = (float)measurement_age_ms / 1000.0F;
    if (!interpolator->state_valid) {
        next = (Algorithm_BallModelInterpolator_State){
            .position_mm = measured_position_mm +
                0.5F * interpolator->last_model_acceleration_mm_s2 *
                    age_s * age_s,
            .velocity_mm_s =
                interpolator->last_model_acceleration_mm_s2 * age_s,
        };
        if (!clamp_state(interpolator, &next) ||
            !valid_state(interpolator, &next)) {
            return false;
        }
        interpolator->state = next;
        interpolator->last_accepted_measurement_time_ms =
            measurement_time_ms;
        interpolator->state_valid = true;
        interpolator->diagnostics.last_innovation_mm = 0.0F;
        interpolator->diagnostics.last_correction_accepted = true;
        increment_saturated(
            &interpolator->diagnostics.accepted_measurements_since_reset);
        return true;
    }

    if (!time_is_strictly_after(
            measurement_time_ms,
            interpolator->last_accepted_measurement_time_ms)) {
        increment_saturated(
            &interpolator->diagnostics.out_of_order_measurement_count);
        return true;
    }
    {
        const uint32_t measurement_interval_ms = measurement_time_ms -
            interpolator->last_accepted_measurement_time_ms;
        const float measurement_interval_s =
            (float)measurement_interval_ms / 1000.0F;
        const float delayed_position_mm = interpolator->state.position_mm -
            interpolator->state.velocity_mm_s * age_s +
            0.5F * interpolator->last_model_acceleration_mm_s2 *
                age_s * age_s;
        const float innovation_mm =
            measured_position_mm - delayed_position_mm;
        float velocity_correction_mm_s;

        if (!isfinite(measurement_interval_s) ||
            (measurement_interval_s <= 0.0F) ||
            !isfinite(delayed_position_mm) || !isfinite(innovation_mm)) {
            return false;
        }
        interpolator->diagnostics.last_innovation_mm = innovation_mm;
        if (fabsf(innovation_mm) >
            interpolator->config.maximum_absolute_innovation_mm) {
            increment_saturated(
                &interpolator->diagnostics.innovation_reject_count);
            return true;
        }

        velocity_correction_mm_s =
            interpolator->config.velocity_beta * innovation_mm /
            measurement_interval_s;
        next = interpolator->state;
        next.position_mm +=
            interpolator->config.position_alpha * innovation_mm +
            velocity_correction_mm_s * age_s;
        next.velocity_mm_s += velocity_correction_mm_s;
        if (!isfinite(velocity_correction_mm_s) ||
            !clamp_state(interpolator, &next) ||
            !valid_state(interpolator, &next)) {
            return false;
        }
    }

    interpolator->state = next;
    interpolator->last_accepted_measurement_time_ms = measurement_time_ms;
    interpolator->diagnostics.last_correction_accepted = true;
    increment_saturated(
        &interpolator->diagnostics.accepted_measurements_since_reset);
    return true;
}

bool Algorithm_BallModelInterpolator_GetState(
    const Algorithm_BallModelInterpolator *interpolator,
    Algorithm_BallModelInterpolator_State *state)
{
    if ((interpolator == NULL) || !interpolator->initialized ||
        !interpolator->state_valid || (state == NULL) ||
        !valid_state(interpolator, &interpolator->state)) {
        return false;
    }
    *state = interpolator->state;
    return true;
}

bool Algorithm_BallModelInterpolator_GetDiagnostics(
    const Algorithm_BallModelInterpolator *interpolator,
    Algorithm_BallModelInterpolator_Diagnostics *diagnostics)
{
    if ((interpolator == NULL) || !interpolator->initialized ||
        (diagnostics == NULL)) {
        return false;
    }
    *diagnostics = interpolator->diagnostics;
    return true;
}

bool Algorithm_BallModelInterpolator_IsTrusted(
    const Algorithm_BallModelInterpolator *interpolator,
    uint32_t now_ms)
{
    uint32_t coast_ms;

    if ((interpolator == NULL) || !interpolator->initialized ||
        !interpolator->state_valid ||
        !valid_state(interpolator, &interpolator->state) ||
        (interpolator->diagnostics.accepted_measurements_since_reset <
            interpolator->config.minimum_accepted_measurements)) {
        return false;
    }
    coast_ms = now_ms - interpolator->last_accepted_measurement_time_ms;
    return (coast_ms < BALL_MODEL_INTERPOLATOR_HALF_TIME_RANGE) &&
        (coast_ms <= interpolator->config.maximum_coast_ms);
}
