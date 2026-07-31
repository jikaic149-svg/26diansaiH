#ifndef ALGORITHM_BALL_POSITION_CONTROL_H
#define ALGORITHM_BALL_POSITION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* Initial target and measurements use millimetres along the physical rod
     * axis. Runtime targets are limited and slew-shaped by this controller. */
    float target_position_mm;
    float maximum_absolute_target_position_mm;
    float target_slew_rate_mm_s;
    /* +1 or -1 maps camera-rule position into the physical rod coordinate. */
    float measurement_polarity;
    /* Desired ball acceleration:
     * kp*(target-position) + integral_acceleration - kd*velocity. */
    float position_gain_per_s2;
    float position_integral_gain_per_s3;
    float velocity_gain_per_s;
    /* Symmetric anti-windup limit for the integral acceleration term. */
    float maximum_integral_acceleration_mm_s2;
    float velocity_filter_tau_s;
    float gravity_mm_s2;
    /* I/(mR^2): 0.4 for a homogeneous solid sphere. */
    float inertia_ratio;
    float rod_angle_limit_deg;
    uint32_t maximum_velocity_dt_ms;
    /* Optional bounded rod-angle pulse that breaks ball stiction. The first
     * pulse uses kick_angle, then each unsuccessful pulse adds step_angle up
     * to maximum_angle. Zero kick_angle disables the sequence. */
    float stiction_kick_angle_deg;
    float stiction_kick_step_angle_deg;
    float stiction_kick_maximum_angle_deg;
    float stiction_kick_minimum_error_mm;
    float stiction_kick_maximum_velocity_mm_s;
    uint32_t stiction_kick_stationary_ms;
    uint32_t stiction_kick_duration_ms;
    uint32_t stiction_kick_cooldown_ms;
} Algorithm_BallPositionControl_Config;

typedef enum {
    ALGORITHM_BALL_STICTION_IDLE = 0,
    ALGORITHM_BALL_STICTION_WAITING,
    ALGORITHM_BALL_STICTION_ACTIVE,
    ALGORITHM_BALL_STICTION_COOLDOWN,
} Algorithm_BallPositionControl_StictionState;

typedef struct {
    Algorithm_BallPositionControl_Config config;
    float target_position_mm;
    float target_command_mm;
    float position_mm;
    float velocity_mm_s;
    float integral_acceleration_mm_s2;
    float desired_acceleration_mm_s2;
    float stiction_kick_angle_deg;
    float stiction_next_kick_angle_deg;
    uint32_t last_sample_ms;
    uint32_t stiction_state_started_ms;
    int8_t stiction_error_direction;
    Algorithm_BallPositionControl_StictionState stiction_state;
    bool measurement_seen;
    bool velocity_valid;
    bool initialized;
} Algorithm_BallPositionControl;

bool Algorithm_BallPositionControl_Init(
    Algorithm_BallPositionControl *controller,
    const Algorithm_BallPositionControl_Config *config);

/* Set a bounded runtime target without clearing the measured position or
 * velocity state. AdvanceTarget moves the active target toward this command
 * at no more than target_slew_rate_mm_s. */
bool Algorithm_BallPositionControl_SetTargetPositionMm(
    Algorithm_BallPositionControl *controller,
    float target_position_mm);
bool Algorithm_BallPositionControl_AdvanceTarget(
    Algorithm_BallPositionControl *controller,
    uint32_t elapsed_ms);

/* Immediately align the command and active target. This only resets the
 * target trajectory; the measurement and velocity state are preserved. */
bool Algorithm_BallPositionControl_ResetTargetPositionMm(
    Algorithm_BallPositionControl *controller,
    float target_position_mm);

/* Apply finite, non-negative runtime gains. The integral state is represented
 * directly as acceleration, so changing Ki does not rescale or jump it.
 * Setting Ki to zero clears the integral contribution. */
bool Algorithm_BallPositionControl_SetGains(
    Algorithm_BallPositionControl *controller,
    float position_gain_per_s2,
    float position_integral_gain_per_s3,
    float velocity_gain_per_s);

/* Feed each new real camera measurement exactly once. A gap outside
 * 1..maximum_velocity_dt_ms keeps position and bounded integral state usable,
 * but resets velocity. ResetMeasurement remains the stale-link reset path. */
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

/* Use the solid-ball rolling inverse model to convert position PID
 * acceleration into a rod-angle correction. */
bool Algorithm_BallPositionControl_ComputeRodAngleDeg(
    Algorithm_BallPositionControl *controller,
    float *rod_angle_deg);

#endif
