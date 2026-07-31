#include "algorithm_ball_state_estimator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define BALL_STATE_ESTIMATOR_DEG_TO_RAD \
    (3.14159265358979323846F / 180.0F)
#define BALL_STATE_ESTIMATOR_MIN_VARIANCE 0.000001F

enum CorrectionResult {
    CORRECTION_RESULT_ERROR = 0,
    CORRECTION_RESULT_ACCEPTED,
    CORRECTION_RESULT_REJECTED,
};

static bool finite_nonnegative(float value)
{
    return isfinite(value) && (value >= 0.0F);
}

static bool valid_config(const Algorithm_BallStateEstimator_Config *config)
{
    return (config != NULL) &&
        isfinite(config->measurement_polarity) &&
        ((config->measurement_polarity == 1.0F) ||
         (config->measurement_polarity == -1.0F)) &&
        isfinite(config->gravity_mm_s2) &&
        (config->gravity_mm_s2 > 0.0F) &&
        finite_nonnegative(config->inertia_ratio) &&
        isfinite(config->position_measurement_variance_mm2) &&
        (config->position_measurement_variance_mm2 > 0.0F) &&
        finite_nonnegative(config->model_acceleration_noise_density_mm2_s3) &&
        finite_nonnegative(config->bias_walk_variance_mm2_s4_per_s) &&
        isfinite(config->initial_position_variance_mm2) &&
        (config->initial_position_variance_mm2 > 0.0F) &&
        isfinite(config->initial_velocity_variance_mm2_s2) &&
        (config->initial_velocity_variance_mm2_s2 > 0.0F) &&
        isfinite(config->initial_bias_variance_mm2_s4) &&
        (config->initial_bias_variance_mm2_s4 > 0.0F) &&
        isfinite(config->innovation_gate_sigma) &&
        (config->innovation_gate_sigma >= 1.0F) &&
        isfinite(config->maximum_position_innovation_mm) &&
        (config->maximum_position_innovation_mm > 0.0F) &&
        isfinite(config->maximum_absolute_position_mm) &&
        (config->maximum_absolute_position_mm > 0.0F) &&
        isfinite(config->maximum_absolute_velocity_mm_s) &&
        (config->maximum_absolute_velocity_mm_s > 0.0F) &&
        isfinite(config->maximum_absolute_bias_mm_s2) &&
        (config->maximum_absolute_bias_mm_s2 > 0.0F) &&
        isfinite(config->trusted_position_variance_mm2) &&
        (config->trusted_position_variance_mm2 > 0.0F) &&
        isfinite(config->trusted_velocity_variance_mm2_s2) &&
        (config->trusted_velocity_variance_mm2_s2 > 0.0F) &&
        (config->maximum_prediction_step_ms > 0U) &&
        (config->maximum_prediction_step_ms <= UINT32_C(1000)) &&
        (config->maximum_measurement_age_ms > 0U) &&
        (config->maximum_measurement_age_ms < UINT32_C(0x80000000)) &&
        (config->minimum_accepted_measurements > 0U);
}

static bool valid_snapshot(
    const Algorithm_BallStateEstimator_Config *config,
    const Algorithm_BallStateEstimator_Snapshot *snapshot)
{
    const Algorithm_BallStateEstimator_State *state = &snapshot->state;
    const Algorithm_BallStateEstimator_Covariance *p = &snapshot->covariance;

    return isfinite(state->position_mm) &&
        isfinite(state->velocity_mm_s) &&
        isfinite(state->acceleration_bias_mm_s2) &&
        (fabsf(state->position_mm) <= config->maximum_absolute_position_mm) &&
        (fabsf(state->velocity_mm_s) <=
            config->maximum_absolute_velocity_mm_s) &&
        (fabsf(state->acceleration_bias_mm_s2) <=
            config->maximum_absolute_bias_mm_s2) &&
        isfinite(p->p00) && isfinite(p->p01) && isfinite(p->p02) &&
        isfinite(p->p11) && isfinite(p->p12) && isfinite(p->p22) &&
        (p->p00 >= 0.0F) && (p->p11 >= 0.0F) && (p->p22 >= 0.0F);
}

static void reset_runtime(
    Algorithm_BallStateEstimator *estimator,
    uint32_t now_ms,
    bool count_reset)
{
    memset(&estimator->state, 0, sizeof(estimator->state));
    memset(&estimator->covariance, 0, sizeof(estimator->covariance));
    memset(estimator->history, 0, sizeof(estimator->history));
    estimator->history_write_index = 0U;
    estimator->history_count = 0U;
    estimator->last_prediction_ms = now_ms;
    estimator->last_accepted_measurement_ms = now_ms;
    estimator->last_model_acceleration_mm_s2 = 0.0F;
    estimator->accepted_measurements_since_reset = 0U;
    estimator->state_valid = false;
    estimator->model_input_valid = false;
    if (count_reset && (estimator->reset_count < UINT32_MAX)) {
        ++estimator->reset_count;
    }
}

static size_t history_index(
    const Algorithm_BallStateEstimator *estimator,
    size_t chronological_index)
{
    return (estimator->history_write_index +
        ALGORITHM_BALL_STATE_ESTIMATOR_HISTORY_CAPACITY -
        estimator->history_count + chronological_index) %
        ALGORITHM_BALL_STATE_ESTIMATOR_HISTORY_CAPACITY;
}

static void history_push(
    Algorithm_BallStateEstimator *estimator,
    const Algorithm_BallStateEstimator_Snapshot *snapshot)
{
    estimator->history[estimator->history_write_index] = *snapshot;
    estimator->history_write_index = (estimator->history_write_index + 1U) %
        ALGORITHM_BALL_STATE_ESTIMATOR_HISTORY_CAPACITY;
    if (estimator->history_count <
        ALGORITHM_BALL_STATE_ESTIMATOR_HISTORY_CAPACITY) {
        ++estimator->history_count;
    }
}

static bool propagate_snapshot(
    const Algorithm_BallStateEstimator_Config *config,
    Algorithm_BallStateEstimator_Snapshot *snapshot,
    float dt_s,
    float model_acceleration_mm_s2)
{
    const float half_dt2 = 0.5F * dt_s * dt_s;
    const float dt2 = dt_s * dt_s;
    const float dt3 = dt2 * dt_s;
    const float total_acceleration = model_acceleration_mm_s2 +
        snapshot->state.acceleration_bias_mm_s2;
    const Algorithm_BallStateEstimator_Covariance old = snapshot->covariance;
    const float q_acc = config->model_acceleration_noise_density_mm2_s3;

    if (!isfinite(dt_s) || (dt_s <= 0.0F) ||
        !isfinite(model_acceleration_mm_s2) ||
        !isfinite(total_acceleration)) {
        return false;
    }

    snapshot->state.position_mm += snapshot->state.velocity_mm_s * dt_s +
        half_dt2 * total_acceleration;
    snapshot->state.velocity_mm_s += dt_s * total_acceleration;

    snapshot->covariance.p00 = old.p00 + 2.0F * dt_s * old.p01 +
        2.0F * half_dt2 * old.p02 + dt_s * dt_s * old.p11 +
        2.0F * dt_s * half_dt2 * old.p12 +
        half_dt2 * half_dt2 * old.p22 +
        q_acc * dt3 / 3.0F;
    snapshot->covariance.p01 = old.p01 + dt_s * old.p11 +
        half_dt2 * old.p12 + dt_s * old.p02 +
        dt_s * dt_s * old.p12 + dt_s * half_dt2 * old.p22 +
        q_acc * dt2 / 2.0F;
    snapshot->covariance.p02 = old.p02 + dt_s * old.p12 +
        half_dt2 * old.p22;
    snapshot->covariance.p11 = old.p11 + 2.0F * dt_s * old.p12 +
        dt_s * dt_s * old.p22 + q_acc * dt_s;
    snapshot->covariance.p12 = old.p12 + dt_s * old.p22;
    snapshot->covariance.p22 = old.p22 +
        config->bias_walk_variance_mm2_s4_per_s * dt_s;

    if (snapshot->covariance.p00 < BALL_STATE_ESTIMATOR_MIN_VARIANCE) {
        snapshot->covariance.p00 = BALL_STATE_ESTIMATOR_MIN_VARIANCE;
    }
    if (snapshot->covariance.p11 < BALL_STATE_ESTIMATOR_MIN_VARIANCE) {
        snapshot->covariance.p11 = BALL_STATE_ESTIMATOR_MIN_VARIANCE;
    }
    if (snapshot->covariance.p22 < BALL_STATE_ESTIMATOR_MIN_VARIANCE) {
        snapshot->covariance.p22 = BALL_STATE_ESTIMATOR_MIN_VARIANCE;
    }
    return valid_snapshot(config, snapshot);
}

static enum CorrectionResult correct_snapshot(
    const Algorithm_BallStateEstimator_Config *config,
    Algorithm_BallStateEstimator_Snapshot *snapshot,
    float measured_position_mm)
{
    const Algorithm_BallStateEstimator_Covariance old = snapshot->covariance;
    const float innovation = measured_position_mm -
        snapshot->state.position_mm;
    const float innovation_variance = old.p00 +
        config->position_measurement_variance_mm2;
    float gate;
    float gain0;
    float gain1;
    float gain2;

    if (!isfinite(innovation) || !isfinite(innovation_variance) ||
        (innovation_variance <= 0.0F)) {
        return CORRECTION_RESULT_ERROR;
    }
    if (fabsf(innovation) > config->maximum_position_innovation_mm) {
        return CORRECTION_RESULT_REJECTED;
    }
    gate = config->innovation_gate_sigma *
        config->innovation_gate_sigma * innovation_variance;
    if (!isfinite(gate)) {
        return CORRECTION_RESULT_ERROR;
    }
    if ((innovation * innovation) > gate) {
        return CORRECTION_RESULT_REJECTED;
    }

    gain0 = old.p00 / innovation_variance;
    gain1 = old.p01 / innovation_variance;
    gain2 = old.p02 / innovation_variance;
    snapshot->state.position_mm += gain0 * innovation;
    snapshot->state.velocity_mm_s += gain1 * innovation;
    snapshot->state.acceleration_bias_mm_s2 += gain2 * innovation;

    /* Scalar H=[1 0 0] covariance update, stored symmetrically. */
    snapshot->covariance.p00 = old.p00 - old.p00 * old.p00 /
        innovation_variance;
    snapshot->covariance.p01 = old.p01 - old.p00 * old.p01 /
        innovation_variance;
    snapshot->covariance.p02 = old.p02 - old.p00 * old.p02 /
        innovation_variance;
    snapshot->covariance.p11 = old.p11 - old.p01 * old.p01 /
        innovation_variance;
    snapshot->covariance.p12 = old.p12 - old.p01 * old.p02 /
        innovation_variance;
    snapshot->covariance.p22 = old.p22 - old.p02 * old.p02 /
        innovation_variance;

    if (snapshot->covariance.p00 < BALL_STATE_ESTIMATOR_MIN_VARIANCE) {
        snapshot->covariance.p00 = BALL_STATE_ESTIMATOR_MIN_VARIANCE;
    }
    if (snapshot->covariance.p11 < BALL_STATE_ESTIMATOR_MIN_VARIANCE) {
        snapshot->covariance.p11 = BALL_STATE_ESTIMATOR_MIN_VARIANCE;
    }
    if (snapshot->covariance.p22 < BALL_STATE_ESTIMATOR_MIN_VARIANCE) {
        snapshot->covariance.p22 = BALL_STATE_ESTIMATOR_MIN_VARIANCE;
    }
    return valid_snapshot(config, snapshot) ?
        CORRECTION_RESULT_ACCEPTED : CORRECTION_RESULT_ERROR;
}

bool Algorithm_BallStateEstimator_Init(
    Algorithm_BallStateEstimator *estimator,
    const Algorithm_BallStateEstimator_Config *config,
    uint32_t now_ms)
{
    if (estimator != NULL) {
        memset(estimator, 0, sizeof(*estimator));
    }
    if ((estimator == NULL) || !valid_config(config)) {
        return false;
    }
    estimator->config = *config;
    estimator->initialized = true;
    reset_runtime(estimator, now_ms, false);
    return true;
}

void Algorithm_BallStateEstimator_Reset(
    Algorithm_BallStateEstimator *estimator,
    uint32_t now_ms)
{
    if ((estimator == NULL) || !estimator->initialized) {
        return;
    }
    reset_runtime(estimator, now_ms, true);
}

bool Algorithm_BallStateEstimator_Predict(
    Algorithm_BallStateEstimator *estimator,
    uint32_t now_ms,
    float rod_angle_deg,
    float chassis_acceleration_mm_s2)
{
    uint32_t delta_ms;
    float theta_rad;
    float rolling_ratio;
    float model_acceleration;
    Algorithm_BallStateEstimator_Snapshot next;

    if ((estimator == NULL) || !estimator->initialized ||
        !isfinite(rod_angle_deg) ||
        !isfinite(chassis_acceleration_mm_s2)) {
        return false;
    }
    theta_rad = rod_angle_deg * BALL_STATE_ESTIMATOR_DEG_TO_RAD;
    rolling_ratio = 1.0F / (1.0F + estimator->config.inertia_ratio);
    model_acceleration = -rolling_ratio *
        (estimator->config.gravity_mm_s2 * sinf(theta_rad) +
         chassis_acceleration_mm_s2 * cosf(theta_rad));
    if (!isfinite(model_acceleration)) {
        reset_runtime(estimator, now_ms, true);
        return false;
    }
    estimator->last_model_acceleration_mm_s2 = model_acceleration;
    estimator->model_input_valid = true;
    delta_ms = now_ms - estimator->last_prediction_ms;
    if (!estimator->state_valid) {
        estimator->last_prediction_ms = now_ms;
        return true;
    }
    if (delta_ms == 0U) {
        return true;
    }
    if (delta_ms > estimator->config.maximum_prediction_step_ms) {
        reset_runtime(estimator, now_ms, true);
        return false;
    }

    next = (Algorithm_BallStateEstimator_Snapshot){
        .state = estimator->state,
        .covariance = estimator->covariance,
        .time_ms = now_ms,
        .step_model_acceleration_mm_s2 = model_acceleration,
    };
    if (!propagate_snapshot(
            &estimator->config, &next,
            (float)delta_ms / 1000.0F, model_acceleration)) {
        reset_runtime(estimator, now_ms, true);
        return false;
    }
    estimator->state = next.state;
    estimator->covariance = next.covariance;
    estimator->last_prediction_ms = now_ms;
    history_push(estimator, &next);
    if (estimator->prediction_count < UINT32_MAX) {
        ++estimator->prediction_count;
    }
    return true;
}

bool Algorithm_BallStateEstimator_CorrectPosition(
    Algorithm_BallStateEstimator *estimator,
    uint32_t now_ms,
    uint32_t measurement_age_ms,
    float camera_position_mm)
{
    float measured_position;
    uint32_t measurement_time_ms;
    size_t first_not_before = 0U;
    bool found = false;
    Algorithm_BallStateEstimator_Snapshot corrected;
    enum CorrectionResult result;

    if ((estimator == NULL) || !estimator->initialized ||
        !isfinite(camera_position_mm) ||
        (now_ms != estimator->last_prediction_ms) ||
        (measurement_age_ms >=
            estimator->config.maximum_measurement_age_ms)) {
        return false;
    }
    measured_position = camera_position_mm *
        estimator->config.measurement_polarity;
    if (!isfinite(measured_position) ||
        (fabsf(measured_position) >
            estimator->config.maximum_absolute_position_mm)) {
        return false;
    }
    measurement_time_ms = now_ms - measurement_age_ms;

    if (!estimator->state_valid) {
        corrected = (Algorithm_BallStateEstimator_Snapshot){
            .state = {
                .position_mm = measured_position,
                .velocity_mm_s = 0.0F,
                .acceleration_bias_mm_s2 = 0.0F,
            },
            .covariance = {
                .p00 = estimator->config.initial_position_variance_mm2,
                .p11 = estimator->config.initial_velocity_variance_mm2_s2,
                .p22 = estimator->config.initial_bias_variance_mm2_s4,
            },
            .time_ms = measurement_time_ms,
        };
        if (!valid_snapshot(&estimator->config, &corrected)) {
            return false;
        }
        estimator->history_write_index = 0U;
        estimator->history_count = 0U;
        history_push(estimator, &corrected);
        if (measurement_age_ms > 0U) {
            if (!estimator->model_input_valid ||
                !propagate_snapshot(
                    &estimator->config,
                    &corrected,
                    (float)measurement_age_ms / 1000.0F,
                    estimator->last_model_acceleration_mm_s2)) {
                estimator->history_write_index = 0U;
                estimator->history_count = 0U;
                return false;
            }
            corrected.time_ms = now_ms;
            corrected.step_model_acceleration_mm_s2 =
                estimator->last_model_acceleration_mm_s2;
            history_push(estimator, &corrected);
        }
        estimator->state = corrected.state;
        estimator->covariance = corrected.covariance;
        estimator->state_valid = true;
        estimator->last_accepted_measurement_ms = measurement_time_ms;
        estimator->accepted_measurements_since_reset = 1U;
        if (estimator->correction_count < UINT32_MAX) {
            ++estimator->correction_count;
        }
        return true;
    }

    if (estimator->history_count == 0U) {
        return false;
    }
    {
        const Algorithm_BallStateEstimator_Snapshot *oldest =
            &estimator->history[history_index(estimator, 0U)];

        if ((int32_t)(measurement_time_ms - oldest->time_ms) < 0) {
            if (estimator->stale_measurement_count < UINT32_MAX) {
                ++estimator->stale_measurement_count;
            }
            return true;
        }
    }

    for (size_t ordinal = 0U;
         ordinal < estimator->history_count;
         ++ordinal) {
        const size_t index = history_index(estimator, ordinal);

        if ((int32_t)(estimator->history[index].time_ms -
                measurement_time_ms) >= 0) {
            first_not_before = ordinal;
            found = true;
            break;
        }
    }
    if (!found) {
        if (estimator->stale_measurement_count < UINT32_MAX) {
            ++estimator->stale_measurement_count;
        }
        return false;
    }

    {
        const size_t first_index = history_index(
            estimator, first_not_before);
        const Algorithm_BallStateEstimator_Snapshot first =
            estimator->history[first_index];

        if (first.time_ms == measurement_time_ms) {
            corrected = first;
            result = correct_snapshot(
                &estimator->config, &corrected, measured_position);
            if (result == CORRECTION_RESULT_REJECTED) {
                if (estimator->innovation_reject_count < UINT32_MAX) {
                    ++estimator->innovation_reject_count;
                }
                return true;
            }
            if (result != CORRECTION_RESULT_ACCEPTED) {
                reset_runtime(estimator, now_ms, true);
                return false;
            }
            estimator->history[first_index] = corrected;
            ++first_not_before;
        }
        else {
            size_t before_index;
            uint32_t partial_ms;

            if (first_not_before == 0U) {
                if (estimator->stale_measurement_count < UINT32_MAX) {
                    ++estimator->stale_measurement_count;
                }
                return true;
            }
            before_index = history_index(estimator, first_not_before - 1U);
            corrected = estimator->history[before_index];
            partial_ms = measurement_time_ms - corrected.time_ms;
            if ((partial_ms > 0U) && !propagate_snapshot(
                    &estimator->config,
                    &corrected,
                    (float)partial_ms / 1000.0F,
                    first.step_model_acceleration_mm_s2)) {
                reset_runtime(estimator, now_ms, true);
                return false;
            }
            corrected.time_ms = measurement_time_ms;
            result = correct_snapshot(
                &estimator->config, &corrected, measured_position);
            if (result == CORRECTION_RESULT_REJECTED) {
                if (estimator->innovation_reject_count < UINT32_MAX) {
                    ++estimator->innovation_reject_count;
                }
                return true;
            }
            if (result != CORRECTION_RESULT_ACCEPTED) {
                reset_runtime(estimator, now_ms, true);
                return false;
            }
        }
    }

    for (size_t ordinal = first_not_before;
         ordinal < estimator->history_count;
         ++ordinal) {
        const size_t index = history_index(estimator, ordinal);
        const uint32_t destination_time = estimator->history[index].time_ms;
        const float model_acceleration =
            estimator->history[index].step_model_acceleration_mm_s2;
        const uint32_t replay_ms = destination_time - corrected.time_ms;

        if ((replay_ms == 0U) || !propagate_snapshot(
                &estimator->config,
                &corrected,
                (float)replay_ms / 1000.0F,
                model_acceleration)) {
            reset_runtime(estimator, now_ms, true);
            return false;
        }
        corrected.time_ms = destination_time;
        corrected.step_model_acceleration_mm_s2 = model_acceleration;
        estimator->history[index] = corrected;
    }

    {
        const size_t newest = history_index(
            estimator, estimator->history_count - 1U);

        estimator->state = estimator->history[newest].state;
        estimator->covariance = estimator->history[newest].covariance;
    }
    estimator->last_accepted_measurement_ms = measurement_time_ms;
    if (estimator->accepted_measurements_since_reset < UINT32_MAX) {
        ++estimator->accepted_measurements_since_reset;
    }
    if (estimator->correction_count < UINT32_MAX) {
        ++estimator->correction_count;
    }
    return true;
}

bool Algorithm_BallStateEstimator_GetState(
    const Algorithm_BallStateEstimator *estimator,
    Algorithm_BallStateEstimator_State *state,
    Algorithm_BallStateEstimator_Covariance *covariance)
{
    if ((estimator == NULL) || !estimator->initialized ||
        !estimator->state_valid || (state == NULL)) {
        return false;
    }
    *state = estimator->state;
    if (covariance != NULL) {
        *covariance = estimator->covariance;
    }
    return true;
}

bool Algorithm_BallStateEstimator_IsTrusted(
    const Algorithm_BallStateEstimator *estimator,
    uint32_t now_ms)
{
    return (estimator != NULL) && estimator->initialized &&
        estimator->state_valid &&
        (estimator->accepted_measurements_since_reset >=
            estimator->config.minimum_accepted_measurements) &&
        ((uint32_t)(now_ms - estimator->last_accepted_measurement_ms) <
            estimator->config.maximum_measurement_age_ms) &&
        (estimator->covariance.p00 <=
            estimator->config.trusted_position_variance_mm2) &&
        (estimator->covariance.p11 <=
            estimator->config.trusted_velocity_variance_mm2_s2);
}
