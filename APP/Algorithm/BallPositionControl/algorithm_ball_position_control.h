#ifndef ALGORITHM_BALL_POSITION_CONTROL_H
#define ALGORITHM_BALL_POSITION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* Target and measurements use millimetres along the physical rod axis. */
    float target_position_mm;
    /* +1 or -1 maps camera-rule position into the physical rod coordinate. */
    float measurement_polarity;
    /* Desired ball acceleration: kp*(target-position)-kd*velocity. */
    float position_gain_per_s2;
    float velocity_gain_per_s;
    float velocity_filter_tau_s;
    float gravity_mm_s2;
    /* I/(mR^2): 0.4 for a homogeneous solid sphere. */
    float inertia_ratio;
    float rod_angle_limit_deg;
    uint32_t maximum_velocity_dt_ms;
} Algorithm_BallPositionControl_Config;

typedef struct {
    Algorithm_BallPositionControl_Config config;
    float position_mm;
    float velocity_mm_s;
    float desired_acceleration_mm_s2;
    uint32_t last_sample_ms;
    bool measurement_seen;
    bool velocity_valid;
    bool initialized;
} Algorithm_BallPositionControl;

bool Algorithm_BallPositionControl_Init(
    Algorithm_BallPositionControl *controller,
    const Algorithm_BallPositionControl_Config *config);

/* Feed each new real camera measurement exactly once. A gap outside
 * 1..maximum_velocity_dt_ms keeps position usable but resets velocity. */
bool Algorithm_BallPositionControl_UpdateMeasurement(
    Algorithm_BallPositionControl *controller,
    uint32_t sample_ms,
    float camera_position_mm);

/* Supply an already polarity-corrected physical state estimate. This is the
 * Kalman integration boundary; unlike UpdateMeasurement it does not derive
 * velocity or apply camera polarity. */
bool Algorithm_BallPositionControl_SetEstimatedState(
    Algorithm_BallPositionControl *controller,
    float position_mm,
    float velocity_mm_s);
void Algorithm_BallPositionControl_ResetMeasurement(
    Algorithm_BallPositionControl *controller);

/* Use the solid-ball rolling inverse model to convert position/velocity PD
 * acceleration into a rod-angle correction. */
bool Algorithm_BallPositionControl_ComputeRodAngleDeg(
    Algorithm_BallPositionControl *controller,
    float *rod_angle_deg);

#endif
