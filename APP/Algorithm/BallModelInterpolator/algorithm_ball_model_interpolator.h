#ifndef ALGORITHM_BALL_MODEL_INTERPOLATOR_H
#define ALGORITHM_BALL_MODEL_INTERPOLATOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float position_mm;
    float velocity_mm_s;
} Algorithm_BallModelInterpolator_State;

typedef struct {
    float gravity_mm_s2;
    /* I/(mR^2): 0.4 for a homogeneous solid sphere. */
    float inertia_ratio;

    /* Fixed alpha-beta correction gains. Both must be in (0, 1]. */
    float position_alpha;
    float velocity_beta;
    /* Established states reject a camera innovation above this magnitude. */
    float maximum_absolute_innovation_mm;

    float maximum_absolute_position_mm;
    float maximum_absolute_velocity_mm_s;
    uint32_t maximum_prediction_step_ms;
    uint32_t maximum_measurement_age_ms;
    uint32_t maximum_coast_ms;
    uint32_t minimum_accepted_measurements;
} Algorithm_BallModelInterpolator_Config;

typedef struct {
    float last_innovation_mm;
    uint32_t accepted_measurements_since_reset;
    uint32_t prediction_count;
    uint32_t correction_count;
    uint32_t innovation_reject_count;
    uint32_t stale_measurement_count;
    uint32_t out_of_order_measurement_count;
    uint32_t state_clamp_count;
    uint32_t reset_count;
    bool last_correction_accepted;
} Algorithm_BallModelInterpolator_Diagnostics;

typedef struct {
    Algorithm_BallModelInterpolator_Config config;
    Algorithm_BallModelInterpolator_State state;
    Algorithm_BallModelInterpolator_Diagnostics diagnostics;
    uint32_t last_prediction_ms;
    uint32_t last_accepted_measurement_time_ms;
    float last_model_acceleration_mm_s2;
    bool state_valid;
    bool model_input_valid;
    bool initialized;
} Algorithm_BallModelInterpolator;

bool Algorithm_BallModelInterpolator_Init(
    Algorithm_BallModelInterpolator *interpolator,
    const Algorithm_BallModelInterpolator_Config *config,
    uint32_t now_ms);

void Algorithm_BallModelInterpolator_Reset(
    Algorithm_BallModelInterpolator *interpolator,
    uint32_t now_ms);

/* Predict to now_ms from the measured rod angle and forward-positive chassis
 * acceleration. The solid-ball rolling model is:
 *   a=-(1/(1+I/(mR^2)))*(g*sin(theta)+a_chassis*cos(theta)).
 * The intended call period is 1 ms. A gap above the configured maximum
 * invalidates the state instead of integrating across an unknown interval. */
bool Algorithm_BallModelInterpolator_Predict(
    Algorithm_BallModelInterpolator *interpolator,
    uint32_t now_ms,
    float rod_angle_deg,
    float chassis_acceleration_mm_s2);

/* Correct one new camera sample exactly once after Predict(now_ms, ...).
 * measurement_age_ms locates the observation in the recent past. A constant-
 * acceleration back-projection forms the innovation; the fixed alpha-beta
 * correction is then approximately propagated back to now. A stale,
 * duplicate, out-of-order, or innovation-gated sample returns true but leaves
 * the state unchanged; diagnostics distinguishes that result. */
bool Algorithm_BallModelInterpolator_CorrectPosition(
    Algorithm_BallModelInterpolator *interpolator,
    uint32_t now_ms,
    uint32_t measurement_age_ms,
    float measured_position_mm);

bool Algorithm_BallModelInterpolator_GetState(
    const Algorithm_BallModelInterpolator *interpolator,
    Algorithm_BallModelInterpolator_State *state);

bool Algorithm_BallModelInterpolator_GetDiagnostics(
    const Algorithm_BallModelInterpolator *interpolator,
    Algorithm_BallModelInterpolator_Diagnostics *diagnostics);

bool Algorithm_BallModelInterpolator_IsTrusted(
    const Algorithm_BallModelInterpolator *interpolator,
    uint32_t now_ms);

#endif
