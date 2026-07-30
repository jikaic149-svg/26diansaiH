#include "algorithm_chassis_acceleration_fusion.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define CHASSIS_ACCELERATION_FUSION_MIN_VARIANCE 0.000001F

enum CorrectionResult {
    CORRECTION_RESULT_ERROR = 0,
    CORRECTION_RESULT_ACCEPTED,
    CORRECTION_RESULT_REJECTED,
};

static bool finite_positive(float value)
{
    return isfinite(value) && (value > 0.0F);
}

static bool finite_nonnegative(float value)
{
    return isfinite(value) && (value >= 0.0F);
}

static bool valid_config(
    const Algorithm_ChassisAccelerationFusion_Config *config)
{
    return (config != NULL) && finite_positive(config->plan_gain) &&
        finite_positive(config->response_time_constant_s) &&
        (config->plan_delay_ms < UINT32_C(0x80000000)) &&
        finite_positive(config->wheel_speed_measurement_variance_mm2_s2) &&
        finite_positive(
            config->imu_acceleration_measurement_variance_mm2_s4) &&
        finite_nonnegative(config->velocity_noise_density_mm2_s3) &&
        finite_nonnegative(config->acceleration_noise_density_mm2_s5) &&
        finite_nonnegative(config->imu_bias_walk_noise_density_mm2_s5) &&
        finite_positive(config->initial_velocity_variance_mm2_s2) &&
        finite_positive(config->initial_acceleration_variance_mm2_s4) &&
        finite_positive(config->initial_imu_bias_variance_mm2_s4) &&
        finite_positive(config->innovation_gate_sigma) &&
        (config->innovation_gate_sigma >= 1.0F) &&
        finite_positive(config->maximum_absolute_velocity_mm_s) &&
        finite_positive(config->maximum_absolute_acceleration_mm_s2) &&
        finite_positive(config->maximum_absolute_imu_bias_mm_s2) &&
        finite_positive(config->trusted_acceleration_variance_mm2_s4) &&
        (config->maximum_prediction_step_ms > 0U) &&
        (config->maximum_prediction_step_ms <= UINT32_C(1000)) &&
        (config->maximum_measurement_age_ms > 0U) &&
        (config->maximum_measurement_age_ms < UINT32_C(0x80000000)) &&
        (config->minimum_accepted_wheel_measurements > 0U) &&
        (config->minimum_accepted_imu_measurements > 0U);
}

static bool valid_state(const Algorithm_ChassisAccelerationFusion *fusion)
{
    const Algorithm_ChassisAccelerationFusion_State *state = &fusion->state;
    const Algorithm_ChassisAccelerationFusion_Covariance *p =
        &fusion->covariance;

    return isfinite(state->velocity_mm_s) &&
        isfinite(state->acceleration_mm_s2) &&
        isfinite(state->imu_bias_mm_s2) &&
        (fabsf(state->velocity_mm_s) <=
            fusion->config.maximum_absolute_velocity_mm_s) &&
        (fabsf(state->acceleration_mm_s2) <=
            fusion->config.maximum_absolute_acceleration_mm_s2) &&
        (fabsf(state->imu_bias_mm_s2) <=
            fusion->config.maximum_absolute_imu_bias_mm_s2) &&
        isfinite(p->p00) && isfinite(p->p01) && isfinite(p->p02) &&
        isfinite(p->p11) && isfinite(p->p12) && isfinite(p->p22) &&
        (p->p00 >= 0.0F) && (p->p11 >= 0.0F) && (p->p22 >= 0.0F);
}

static void clamp_covariance_diagonal(
    Algorithm_ChassisAccelerationFusion_Covariance *p)
{
    if (p->p00 < CHASSIS_ACCELERATION_FUSION_MIN_VARIANCE) {
        p->p00 = CHASSIS_ACCELERATION_FUSION_MIN_VARIANCE;
    }
    if (p->p11 < CHASSIS_ACCELERATION_FUSION_MIN_VARIANCE) {
        p->p11 = CHASSIS_ACCELERATION_FUSION_MIN_VARIANCE;
    }
    if (p->p22 < CHASSIS_ACCELERATION_FUSION_MIN_VARIANCE) {
        p->p22 = CHASSIS_ACCELERATION_FUSION_MIN_VARIANCE;
    }
}

static void plan_history_push(
    Algorithm_ChassisAccelerationFusion *fusion,
    uint32_t now_ms,
    float planned_acceleration_mm_s2)
{
    fusion->plan_history[fusion->plan_history_write_index] =
        (Algorithm_ChassisAccelerationFusion_PlanSample){
            .time_ms = now_ms,
            .planned_acceleration_mm_s2 = planned_acceleration_mm_s2,
        };
    fusion->plan_history_write_index =
        (fusion->plan_history_write_index + 1U) %
        ALGORITHM_CHASSIS_ACCELERATION_FUSION_PLAN_HISTORY_CAPACITY;
    if (fusion->plan_history_count <
        ALGORITHM_CHASSIS_ACCELERATION_FUSION_PLAN_HISTORY_CAPACITY) {
        ++fusion->plan_history_count;
    }
}

static float delayed_plan(
    const Algorithm_ChassisAccelerationFusion *fusion,
    uint32_t now_ms)
{
    const size_t capacity =
        ALGORITHM_CHASSIS_ACCELERATION_FUSION_PLAN_HISTORY_CAPACITY;
    const size_t oldest =
        (fusion->plan_history_write_index + capacity -
         fusion->plan_history_count) % capacity;
    float selected = fusion->plan_history[oldest].planned_acceleration_mm_s2;

    for (size_t order = 0U; order < fusion->plan_history_count; ++order) {
        const size_t index = (oldest + order) % capacity;
        const Algorithm_ChassisAccelerationFusion_PlanSample *sample =
            &fusion->plan_history[index];

        if ((uint32_t)(now_ms - sample->time_ms) <
            fusion->config.plan_delay_ms) {
            break;
        }
        selected = sample->planned_acceleration_mm_s2;
    }
    return selected;
}

static void reset_runtime(
    Algorithm_ChassisAccelerationFusion *fusion,
    uint32_t now_ms,
    bool count_reset)
{
    const Algorithm_ChassisAccelerationFusion_Config *config =
        &fusion->config;

    memset(&fusion->state, 0, sizeof(fusion->state));
    fusion->covariance = (Algorithm_ChassisAccelerationFusion_Covariance){
        .p00 = config->initial_velocity_variance_mm2_s2,
        .p11 = config->initial_acceleration_variance_mm2_s4,
        .p22 = config->initial_imu_bias_variance_mm2_s4,
    };
    memset(fusion->plan_history, 0, sizeof(fusion->plan_history));
    fusion->plan_history_write_index = 0U;
    fusion->plan_history_count = 0U;
    plan_history_push(fusion, now_ms, 0.0F);
    fusion->last_prediction_ms = now_ms;
    fusion->last_accepted_wheel_ms = now_ms;
    fusion->last_accepted_imu_ms = now_ms;
    fusion->accepted_wheel_measurement_count = 0U;
    fusion->accepted_imu_measurement_count = 0U;
    fusion->wheel_innovation_reject_count = 0U;
    fusion->imu_innovation_reject_count = 0U;
    fusion->prediction_count = 0U;
    if (count_reset && (fusion->reset_count < UINT32_MAX)) {
        ++fusion->reset_count;
    }
}

static enum CorrectionResult correct_scalar(
    Algorithm_ChassisAccelerationFusion *fusion,
    const float h[3],
    float innovation,
    float measurement_variance)
{
    const Algorithm_ChassisAccelerationFusion_Covariance old =
        fusion->covariance;
    const float p[3][3] = {
        { old.p00, old.p01, old.p02 },
        { old.p01, old.p11, old.p12 },
        { old.p02, old.p12, old.p22 },
    };
    float ph[3] = { 0.0F, 0.0F, 0.0F };
    float updated[3][3];
    float innovation_variance = measurement_variance;
    float gain[3];
    float gate;

    for (size_t row = 0U; row < 3U; ++row) {
        for (size_t column = 0U; column < 3U; ++column) {
            ph[row] += p[row][column] * h[column];
        }
        innovation_variance += h[row] * ph[row];
    }
    if (!isfinite(innovation) || !isfinite(innovation_variance) ||
        (innovation_variance <= 0.0F)) {
        return CORRECTION_RESULT_ERROR;
    }
    gate = fusion->config.innovation_gate_sigma *
        fusion->config.innovation_gate_sigma * innovation_variance;
    if (!isfinite(gate)) {
        return CORRECTION_RESULT_ERROR;
    }
    if ((innovation * innovation) > gate) {
        return CORRECTION_RESULT_REJECTED;
    }
    for (size_t index = 0U; index < 3U; ++index) {
        gain[index] = ph[index] / innovation_variance;
    }
    fusion->state.velocity_mm_s += gain[0] * innovation;
    fusion->state.acceleration_mm_s2 += gain[1] * innovation;
    fusion->state.imu_bias_mm_s2 += gain[2] * innovation;

    for (size_t row = 0U; row < 3U; ++row) {
        for (size_t column = 0U; column < 3U; ++column) {
            updated[row][column] = p[row][column] -
                gain[row] * ph[column];
        }
    }
    fusion->covariance = (Algorithm_ChassisAccelerationFusion_Covariance){
        .p00 = updated[0][0],
        .p01 = 0.5F * (updated[0][1] + updated[1][0]),
        .p02 = 0.5F * (updated[0][2] + updated[2][0]),
        .p11 = updated[1][1],
        .p12 = 0.5F * (updated[1][2] + updated[2][1]),
        .p22 = updated[2][2],
    };
    clamp_covariance_diagonal(&fusion->covariance);
    return valid_state(fusion) ?
        CORRECTION_RESULT_ACCEPTED : CORRECTION_RESULT_ERROR;
}

bool Algorithm_ChassisAccelerationFusion_Init(
    Algorithm_ChassisAccelerationFusion *fusion,
    const Algorithm_ChassisAccelerationFusion_Config *config,
    uint32_t now_ms)
{
    if (fusion != NULL) {
        memset(fusion, 0, sizeof(*fusion));
    }
    if ((fusion == NULL) || !valid_config(config)) {
        return false;
    }
    fusion->config = *config;
    fusion->initialized = true;
    reset_runtime(fusion, now_ms, false);
    return true;
}

void Algorithm_ChassisAccelerationFusion_Reset(
    Algorithm_ChassisAccelerationFusion *fusion,
    uint32_t now_ms)
{
    if ((fusion == NULL) || !fusion->initialized) {
        return;
    }
    reset_runtime(fusion, now_ms, true);
}

bool Algorithm_ChassisAccelerationFusion_Predict(
    Algorithm_ChassisAccelerationFusion *fusion,
    uint32_t now_ms,
    float planned_acceleration_mm_s2)
{
    uint32_t delta_ms;
    float dt_s;
    float phi;
    float coupling;
    float target_acceleration;
    Algorithm_ChassisAccelerationFusion_Covariance old;

    if ((fusion == NULL) || !fusion->initialized ||
        !isfinite(planned_acceleration_mm_s2)) {
        return false;
    }
    delta_ms = now_ms - fusion->last_prediction_ms;
    if (delta_ms == 0U) {
        return true;
    }
    if (delta_ms > fusion->config.maximum_prediction_step_ms) {
        reset_runtime(fusion, now_ms, true);
        return false;
    }
    plan_history_push(fusion, now_ms, planned_acceleration_mm_s2);
    target_acceleration = fusion->config.plan_gain *
        delayed_plan(fusion, now_ms);
    dt_s = (float)delta_ms / 1000.0F;
    phi = expf(-dt_s / fusion->config.response_time_constant_s);
    coupling = fusion->config.response_time_constant_s * (1.0F - phi);
    old = fusion->covariance;

    fusion->state.velocity_mm_s +=
        coupling * fusion->state.acceleration_mm_s2 +
        (dt_s - coupling) * target_acceleration;
    fusion->state.acceleration_mm_s2 =
        phi * fusion->state.acceleration_mm_s2 +
        (1.0F - phi) * target_acceleration;

    fusion->covariance.p00 = old.p00 + 2.0F * coupling * old.p01 +
        coupling * coupling * old.p11 +
        fusion->config.velocity_noise_density_mm2_s3 * dt_s;
    fusion->covariance.p01 = phi * (old.p01 + coupling * old.p11);
    fusion->covariance.p02 = old.p02 + coupling * old.p12;
    fusion->covariance.p11 = phi * phi * old.p11 +
        fusion->config.acceleration_noise_density_mm2_s5 * dt_s;
    fusion->covariance.p12 = phi * old.p12;
    fusion->covariance.p22 = old.p22 +
        fusion->config.imu_bias_walk_noise_density_mm2_s5 * dt_s;
    clamp_covariance_diagonal(&fusion->covariance);
    fusion->last_prediction_ms = now_ms;
    if (fusion->prediction_count < UINT32_MAX) {
        ++fusion->prediction_count;
    }
    if (!valid_state(fusion)) {
        reset_runtime(fusion, now_ms, true);
        return false;
    }
    return true;
}

bool Algorithm_ChassisAccelerationFusion_Correct(
    Algorithm_ChassisAccelerationFusion *fusion,
    uint32_t now_ms,
    bool wheel_speed_valid,
    float wheel_speed_mm_s,
    bool imu_acceleration_valid,
    float imu_acceleration_mm_s2)
{
    enum CorrectionResult result;

    if ((fusion == NULL) || !fusion->initialized ||
        (!wheel_speed_valid && !imu_acceleration_valid) ||
        (wheel_speed_valid && !isfinite(wheel_speed_mm_s)) ||
        (imu_acceleration_valid && !isfinite(imu_acceleration_mm_s2))) {
        return false;
    }
    if (wheel_speed_valid) {
        const float h[3] = { 1.0F, 0.0F, 0.0F };

        result = correct_scalar(
            fusion, h,
            wheel_speed_mm_s - fusion->state.velocity_mm_s,
            fusion->config.wheel_speed_measurement_variance_mm2_s2);
        if (result == CORRECTION_RESULT_ERROR) {
            reset_runtime(fusion, now_ms, true);
            return false;
        }
        if (result == CORRECTION_RESULT_ACCEPTED) {
            fusion->last_accepted_wheel_ms = now_ms;
            if (fusion->accepted_wheel_measurement_count < UINT32_MAX) {
                ++fusion->accepted_wheel_measurement_count;
            }
        }
        else if (fusion->wheel_innovation_reject_count < UINT32_MAX) {
            ++fusion->wheel_innovation_reject_count;
        }
    }
    if (imu_acceleration_valid) {
        const float h[3] = { 0.0F, 1.0F, 1.0F };
        const float predicted = fusion->state.acceleration_mm_s2 +
            fusion->state.imu_bias_mm_s2;

        result = correct_scalar(
            fusion, h, imu_acceleration_mm_s2 - predicted,
            fusion->config.imu_acceleration_measurement_variance_mm2_s4);
        if (result == CORRECTION_RESULT_ERROR) {
            reset_runtime(fusion, now_ms, true);
            return false;
        }
        if (result == CORRECTION_RESULT_ACCEPTED) {
            fusion->last_accepted_imu_ms = now_ms;
            if (fusion->accepted_imu_measurement_count < UINT32_MAX) {
                ++fusion->accepted_imu_measurement_count;
            }
        }
        else if (fusion->imu_innovation_reject_count < UINT32_MAX) {
            ++fusion->imu_innovation_reject_count;
        }
    }
    return valid_state(fusion);
}

bool Algorithm_ChassisAccelerationFusion_GetState(
    const Algorithm_ChassisAccelerationFusion *fusion,
    Algorithm_ChassisAccelerationFusion_State *state,
    Algorithm_ChassisAccelerationFusion_Covariance *covariance)
{
    if ((fusion == NULL) || !fusion->initialized || (state == NULL) ||
        !valid_state(fusion)) {
        return false;
    }
    *state = fusion->state;
    if (covariance != NULL) {
        *covariance = fusion->covariance;
    }
    return true;
}

bool Algorithm_ChassisAccelerationFusion_IsTrusted(
    const Algorithm_ChassisAccelerationFusion *fusion,
    uint32_t now_ms)
{
    return (fusion != NULL) && fusion->initialized && valid_state(fusion) &&
        (fusion->accepted_wheel_measurement_count >=
            fusion->config.minimum_accepted_wheel_measurements) &&
        (fusion->accepted_imu_measurement_count >=
            fusion->config.minimum_accepted_imu_measurements) &&
        ((uint32_t)(now_ms - fusion->last_accepted_wheel_ms) <
            fusion->config.maximum_measurement_age_ms) &&
        ((uint32_t)(now_ms - fusion->last_accepted_imu_ms) <
            fusion->config.maximum_measurement_age_ms) &&
        (fusion->covariance.p11 <=
            fusion->config.trusted_acceleration_variance_mm2_s4);
}
