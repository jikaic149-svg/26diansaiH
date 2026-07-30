#ifndef ALGORITHM_CHASSIS_IMU_ACCELERATION_H
#define ALGORITHM_CHASSIS_IMU_ACCELERATION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* Number of consecutive startup samples averaged as the stationary
     * longitudinal accelerometer bias. Control output stays invalid until
     * this calibration completes, then the bias is frozen. */
    uint32_t bias_calibration_samples;
    /* Each of the two cascaded acceleration EMA stages uses this time
     * constant, in seconds. Zero makes both stages track the input. */
    float acceleration_filter_stage_time_constant_s;
    /* Fixed-rate RBJ notch applied after bias removal and before both EMA
     * stages. Units are Hz; center must be below half the sample rate. */
    float notch_sample_frequency_hz;
    float notch_center_frequency_hz;
    float notch_quality_factor;
    float maximum_absolute_acceleration_mm_s2;
    uint32_t maximum_sample_step_ms;
} Algorithm_ChassisImuAcceleration_Config;

typedef struct {
    uint32_t sample_ms;
    /* Body-forward specific force after the fixed sensor-axis sign mapping,
     * in mm/s^2. No attitude or gravity projection is applied here. */
    float longitudinal_specific_force_mm_s2;
} Algorithm_ChassisImuAcceleration_Sample;

typedef struct {
    uint32_t sample_ms;
    float bias_mm_s2;
    float unfiltered_acceleration_mm_s2;
    float filtered_acceleration_mm_s2;
    uint32_t bias_sample_count;
    bool bias_ready;
    bool valid;
} Algorithm_ChassisImuAcceleration_Output;

typedef struct {
    Algorithm_ChassisImuAcceleration_Config config;
    Algorithm_ChassisImuAcceleration_Output output;
    uint32_t last_sample_ms;
    uint32_t update_count;
    uint32_t gap_reseed_count;
    uint32_t invalid_sample_count;
    uint32_t bias_sample_count;
    float bias_mm_s2;
    float notch_b0;
    float notch_b1;
    float notch_b2;
    float notch_a1;
    float notch_a2;
    float notch_state_one;
    float notch_state_two;
    float acceleration_filter_stage_one_mm_s2;
    bool sample_seen;
    bool bias_ready;
    bool initialized;
} Algorithm_ChassisImuAcceleration;

bool Algorithm_ChassisImuAcceleration_Init(
    Algorithm_ChassisImuAcceleration *processor,
    const Algorithm_ChassisImuAcceleration_Config *config);

void Algorithm_ChassisImuAcceleration_Reset(
    Algorithm_ChassisImuAcceleration *processor);

/* Startup samples establish and freeze the Sensor-Y stationary bias. After
 * calibration, Ay-bias passes through the fixed-rate notch and two exact EMA
 * stages. A long sample gap re-seeds all filter states from the current
 * bias-corrected sample so stale acceleration is never republished. */
bool Algorithm_ChassisImuAcceleration_Update(
    Algorithm_ChassisImuAcceleration *processor,
    const Algorithm_ChassisImuAcceleration_Sample *sample);

bool Algorithm_ChassisImuAcceleration_GetOutput(
    const Algorithm_ChassisImuAcceleration *processor,
    Algorithm_ChassisImuAcceleration_Output *output);

#endif
