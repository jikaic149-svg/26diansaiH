#ifndef ALGORITHM_BALL_STATE_ESTIMATOR_H
#define ALGORITHM_BALL_STATE_ESTIMATOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 128 snapshots preserve about 127 ms at the nominal 1 ms control period. */
#define ALGORITHM_BALL_STATE_ESTIMATOR_HISTORY_CAPACITY 128U

typedef struct {
    float position_mm;
    float velocity_mm_s;
    float acceleration_bias_mm_s2;
} Algorithm_BallStateEstimator_State;

typedef struct {
    float p00;
    float p01;
    float p02;
    float p11;
    float p12;
    float p22;
} Algorithm_BallStateEstimator_Covariance;

typedef struct {
    float measurement_polarity;
    float gravity_mm_s2;
    /* I/(mR^2): 0.4 for a homogeneous solid sphere. */
    float inertia_ratio;

    float position_measurement_variance_mm2;
    /* Continuous white acceleration spectral density, mm^2/s^3. */
    float model_acceleration_noise_density_mm2_s3;
    /* Acceleration-bias random-walk spectral density, mm^2/s^4 per second. */
    float bias_walk_variance_mm2_s4_per_s;
    float initial_position_variance_mm2;
    float initial_velocity_variance_mm2_s2;
    float initial_bias_variance_mm2_s4;
    float innovation_gate_sigma;

    float maximum_absolute_position_mm;
    float maximum_absolute_velocity_mm_s;
    float maximum_absolute_bias_mm_s2;
    float trusted_position_variance_mm2;
    float trusted_velocity_variance_mm2_s2;

    uint32_t maximum_prediction_step_ms;
    uint32_t maximum_measurement_age_ms;
    uint32_t minimum_accepted_measurements;
} Algorithm_BallStateEstimator_Config;

typedef struct {
    float last_innovation_mm;
    float last_innovation_variance_mm2;
    float last_normalized_innovation_squared;
    uint32_t accepted_measurements_since_reset;
    uint32_t prediction_count;
    uint32_t correction_count;
    uint32_t innovation_reject_count;
    uint32_t stale_measurement_count;
    uint32_t reset_count;
} Algorithm_BallStateEstimator_Diagnostics;

typedef struct {
    Algorithm_BallStateEstimator_State state;
    Algorithm_BallStateEstimator_Covariance covariance;
    uint32_t time_ms;
    /* Model acceleration applied from the preceding snapshot to this one. */
    float step_model_acceleration_mm_s2;
} Algorithm_BallStateEstimator_Snapshot;

typedef struct {
    Algorithm_BallStateEstimator_Config config;
    Algorithm_BallStateEstimator_State state;
    Algorithm_BallStateEstimator_Covariance covariance;
    Algorithm_BallStateEstimator_Snapshot
        history[ALGORITHM_BALL_STATE_ESTIMATOR_HISTORY_CAPACITY];
    size_t history_write_index;
    size_t history_count;
    uint32_t last_prediction_ms;
    uint32_t last_accepted_measurement_ms;
    float last_model_acceleration_mm_s2;
    float last_innovation_mm;
    float last_innovation_variance_mm2;
    float last_normalized_innovation_squared;
    uint32_t accepted_measurements_since_reset;
    uint32_t prediction_count;
    uint32_t correction_count;
    uint32_t innovation_reject_count;
    uint32_t stale_measurement_count;
    uint32_t reset_count;
    bool state_valid;
    bool model_input_valid;
    bool initialized;
} Algorithm_BallStateEstimator;

bool Algorithm_BallStateEstimator_Init(
    Algorithm_BallStateEstimator *estimator,
    const Algorithm_BallStateEstimator_Config *config,
    uint32_t now_ms);

void Algorithm_BallStateEstimator_Reset(
    Algorithm_BallStateEstimator *estimator,
    uint32_t now_ms);

/* Predict to now_ms from actual rod angle and forward-positive chassis
 * acceleration. The rolling model is:
 *   a_ball=-(1/(1+I/mR^2))*(g*sin(theta)+a_fwd*cos(theta))+bias.
 * A gap above maximum_prediction_step_ms resets the estimate. */
bool Algorithm_BallStateEstimator_Predict(
    Algorithm_BallStateEstimator *estimator,
    uint32_t now_ms,
    float rod_angle_deg,
    float chassis_acceleration_mm_s2);

/* Correct with one new real camera sample exactly once. measurement_age_ms
 * locates it in the fixed-lag history; an innovation outside the configured
 * sigma gate is counted and ignored without corrupting the current state. */
bool Algorithm_BallStateEstimator_CorrectPosition(
    Algorithm_BallStateEstimator *estimator,
    uint32_t now_ms,
    uint32_t measurement_age_ms,
    float camera_position_mm);

bool Algorithm_BallStateEstimator_GetState(
    const Algorithm_BallStateEstimator *estimator,
    Algorithm_BallStateEstimator_State *state,
    Algorithm_BallStateEstimator_Covariance *covariance);

bool Algorithm_BallStateEstimator_GetDiagnostics(
    const Algorithm_BallStateEstimator *estimator,
    Algorithm_BallStateEstimator_Diagnostics *diagnostics);

bool Algorithm_BallStateEstimator_IsTrusted(
    const Algorithm_BallStateEstimator *estimator,
    uint32_t now_ms);

#endif
