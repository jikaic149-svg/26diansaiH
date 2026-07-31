#ifndef ALGORITHM_BALL_POSITION_CONTROL_H
#define ALGORITHM_BALL_POSITION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define ALGORITHM_BALL_POSITION_CONTROL_VELOCITY_HISTORY_CAPACITY 8U

typedef struct {
    /* Initial target and measurements use millimetres along the physical rod
     * axis. Runtime targets are limited and slew-shaped by this controller. */
    float target_position_mm;
    float maximum_absolute_target_position_mm;
    float target_slew_rate_mm_s;
    /* Speed-approach target and acceleration limit. A distant target command
     * starts one continuous speed-loop trip; the normal position loop takes
     * over once near the final command and remains latched for that trip. */
    float maximum_target_velocity_mm_s;
    float maximum_target_acceleration_mm_s2;
    /* +1 or -1 maps camera-rule position into the physical rod coordinate. */
    float measurement_polarity;
    /* Position PID acceleration:
     * kp*(target-position) + integral_acceleration - kd*velocity.
     * The field names remain compatible with the VTP wire API. */
    float position_gain_per_s2;
    float position_integral_gain_per_s3;
    float velocity_gain_per_s;
    /* Symmetric anti-windup limit for the integral acceleration term. */
    float maximum_integral_acceleration_mm_s2;
    float velocity_filter_tau_s;
    /* Minimum real-camera time baseline used for position differencing.
     * Several frames are retained so velocity is not derived from one noisy
     * inter-frame position step. */
    uint32_t velocity_estimation_window_ms;
    /* Optional sticky quiet hold: enter when both position error and estimated
     * speed are within their limits, then keep the ball-loop output at zero
     * until the target command changes or the measurement state is reset. */
    float hold_position_error_mm;
    float hold_maximum_velocity_mm_s;
    uint32_t hold_stable_duration_ms;
    /* Optional near-target damping schedule; zero values disable it. */
    float near_target_error_mm;
    float near_target_velocity_gain_multiplier;
    float gravity_mm_s2;
    /* I/(mR^2): 0.4 for a homogeneous solid sphere. */
    float inertia_ratio;
    float rod_angle_limit_deg;
    uint32_t maximum_velocity_dt_ms;
    /* Legacy discrete-angle pulse configuration. These values must remain
     * zero; stiction is now handled by the bounded speed assist below. */
    float stiction_kick_angle_deg;
    float stiction_kick_step_angle_deg;
    float stiction_kick_maximum_angle_deg;
    /* Legacy field names are kept for the VST/config integration. The speed
     * trip enters when command error exceeds minimum_error + exit_distance,
     * tapers across that band, and permanently hands off to the position loop
     * at minimum_error once ball speed is below maximum_velocity. A new target
     * or measurement reset rearms the trip. duration_ms shapes only the speed
     * loop acceleration limit; it does not gate loop activation. */
    float stiction_assist_velocity_gain_per_s;
    float stiction_assist_velocity_integral_gain_per_s2;
    float stiction_assist_exit_distance_mm;
    float stiction_kick_minimum_error_mm;
    float stiction_kick_maximum_velocity_mm_s;
    uint32_t stiction_kick_duration_ms;
} Algorithm_BallPositionControl_Config;

typedef enum {
    ALGORITHM_BALL_STICTION_IDLE = 0,
    ALGORITHM_BALL_STICTION_ACTIVE,
    ALGORITHM_BALL_STICTION_POSITION_LATCHED,
} Algorithm_BallPositionControl_StictionState;

typedef struct {
    Algorithm_BallPositionControl_Config config;
    float target_position_mm;
    float target_command_mm;
    float position_mm;
    float velocity_mm_s;
    float target_velocity_mm_s;
    float integral_acceleration_mm_s2;
    float desired_acceleration_mm_s2;
    float stiction_kick_angle_deg;
    float stiction_assist_acceleration_mm_s2;
    float stiction_assist_velocity_integral_acceleration_mm_s2;
    float velocity_history_position_mm[
        ALGORITHM_BALL_POSITION_CONTROL_VELOCITY_HISTORY_CAPACITY];
    uint32_t velocity_history_sample_ms[
        ALGORITHM_BALL_POSITION_CONTROL_VELOCITY_HISTORY_CAPACITY];
    uint32_t last_sample_ms;
    uint32_t hold_candidate_started_ms;
    uint32_t stiction_state_started_ms;
    int8_t stiction_error_direction;
    Algorithm_BallPositionControl_StictionState stiction_state;
    uint32_t velocity_history_write_index;
    uint32_t velocity_history_count;
    bool measurement_seen;
    bool velocity_valid;
    bool speed_handoff_enabled;
    bool hold_active;
    bool hold_candidate_active;
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

/* Runtime gate used by the Gateway competition mode. Disabling the handoff
 * immediately selects and latches the normal position loop without changing
 * the target or measurement state. Re-enabling rearms a fresh trip. */
bool Algorithm_BallPositionControl_SetSpeedHandoffEnabled(
    Algorithm_BallPositionControl *controller,
    bool enabled);

/* Apply finite, non-negative runtime gains. The integral state is represented
 * directly as acceleration, so changing Ki does not rescale or jump it.
 * Setting Ki to zero clears the integral contribution. */
bool Algorithm_BallPositionControl_SetGains(
    Algorithm_BallPositionControl *controller,
    float position_gain_per_s2,
    float position_integral_gain_per_s3,
    float velocity_gain_per_s);

/* Feed each new real camera measurement exactly once. Velocity is calculated
 * across the configured multi-frame time window and then low-pass filtered.
 * A gap outside 1..maximum_velocity_dt_ms resets that history. */
bool Algorithm_BallPositionControl_UpdateMeasurement(
    Algorithm_BallPositionControl *controller,
    uint32_t sample_ms,
    float camera_position_mm);

/* Supply an already polarity-corrected physical state estimate at sample_ms.
 * This is the Kalman integration boundary: velocity is supplied directly,
 * while the real elapsed time still advances integral and stiction state. */
bool Algorithm_BallPositionControl_SetEstimatedState(
    Algorithm_BallPositionControl *controller,
    uint32_t sample_ms,
    float position_mm,
    float velocity_mm_s);
void Algorithm_BallPositionControl_ResetMeasurement(
    Algorithm_BallPositionControl *controller);

/* Use the latched position loop or the exclusive continuous speed-approach
 * loop, then the solid-ball inverse model for rod angle. */
bool Algorithm_BallPositionControl_ComputeRodAngleDeg(
    Algorithm_BallPositionControl *controller,
    float *rod_angle_deg);

#endif
