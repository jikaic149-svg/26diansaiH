#ifndef ALGORITHM_CHASSIS_ACCELERATION_FUSION_H
#define ALGORITHM_CHASSIS_ACCELERATION_FUSION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 64 samples retain about 126 ms of plan input at the nominal 2 ms rate. */
#define ALGORITHM_CHASSIS_ACCELERATION_FUSION_PLAN_HISTORY_CAPACITY 64U

/* State x=[v_chassis, a_chassis, b_imu]. */
typedef struct {
    float velocity_mm_s;
    float acceleration_mm_s2;
    float imu_bias_mm_s2;
} Algorithm_ChassisAccelerationFusion_State;

typedef struct {
    float p00;
    float p01;
    float p02;
    float p11;
    float p12;
    float p22;
} Algorithm_ChassisAccelerationFusion_Covariance;

typedef struct {
    /* a_dot=(plan_gain*a_plan_delayed-a)/response_time_constant_s. */
    float plan_gain;
    float response_time_constant_s;
    uint32_t plan_delay_ms;

    float wheel_speed_measurement_variance_mm2_s2;
    float imu_acceleration_measurement_variance_mm2_s4;
    float velocity_noise_density_mm2_s3;
    float acceleration_noise_density_mm2_s5;
    float imu_bias_walk_noise_density_mm2_s5;

    float initial_velocity_variance_mm2_s2;
    float initial_acceleration_variance_mm2_s4;
    float initial_imu_bias_variance_mm2_s4;
    float innovation_gate_sigma;

    float maximum_absolute_velocity_mm_s;
    float maximum_absolute_acceleration_mm_s2;
    float maximum_absolute_imu_bias_mm_s2;
    float trusted_acceleration_variance_mm2_s4;

    uint32_t maximum_prediction_step_ms;
    uint32_t maximum_measurement_age_ms;
    uint32_t minimum_accepted_wheel_measurements;
    uint32_t minimum_accepted_imu_measurements;
} Algorithm_ChassisAccelerationFusion_Config;

typedef struct {
    uint32_t time_ms;
    float planned_acceleration_mm_s2;
} Algorithm_ChassisAccelerationFusion_PlanSample;

typedef struct {
    Algorithm_ChassisAccelerationFusion_Config config;
    Algorithm_ChassisAccelerationFusion_State state;
    Algorithm_ChassisAccelerationFusion_Covariance covariance;
    Algorithm_ChassisAccelerationFusion_PlanSample
        plan_history[
            ALGORITHM_CHASSIS_ACCELERATION_FUSION_PLAN_HISTORY_CAPACITY];
    size_t plan_history_write_index;
    size_t plan_history_count;
    uint32_t last_prediction_ms;
    uint32_t last_accepted_wheel_ms;
    uint32_t last_accepted_imu_ms;
    uint32_t accepted_wheel_measurement_count;
    uint32_t accepted_imu_measurement_count;
    uint32_t wheel_innovation_reject_count;
    uint32_t imu_innovation_reject_count;
    uint32_t prediction_count;
    uint32_t reset_count;
    bool initialized;
} Algorithm_ChassisAccelerationFusion;

bool Algorithm_ChassisAccelerationFusion_Init(
    Algorithm_ChassisAccelerationFusion *fusion,
    const Algorithm_ChassisAccelerationFusion_Config *config,
    uint32_t now_ms);

void Algorithm_ChassisAccelerationFusion_Reset(
    Algorithm_ChassisAccelerationFusion *fusion,
    uint32_t now_ms);

/* Predict at the local control rate using the current planned acceleration.
 * The internal history applies the configured pure input delay. */
bool Algorithm_ChassisAccelerationFusion_Predict(
    Algorithm_ChassisAccelerationFusion *fusion,
    uint32_t now_ms,
    float planned_acceleration_mm_s2);

/* Correct each new ACC_MEAS source sample exactly once. At least one source
 * must be valid. The caller owns remote-age and source-flag validation. */
bool Algorithm_ChassisAccelerationFusion_Correct(
    Algorithm_ChassisAccelerationFusion *fusion,
    uint32_t now_ms,
    bool wheel_speed_valid,
    float wheel_speed_mm_s,
    bool imu_acceleration_valid,
    float imu_acceleration_mm_s2);

bool Algorithm_ChassisAccelerationFusion_GetState(
    const Algorithm_ChassisAccelerationFusion *fusion,
    Algorithm_ChassisAccelerationFusion_State *state,
    Algorithm_ChassisAccelerationFusion_Covariance *covariance);

bool Algorithm_ChassisAccelerationFusion_IsTrusted(
    const Algorithm_ChassisAccelerationFusion *fusion,
    uint32_t now_ms);

#endif
