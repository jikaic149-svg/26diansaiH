#ifndef ALGORITHM_LEVEL_CURRENT_CONTROL_H
#define ALGORITHM_LEVEL_CURRENT_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* Nominal controller sample time. The Gateway updates once per new
     * QD4310 feedback frame. */
    float control_period_s;
    /* Angles are relative to the absolute encoder position captured at boot. */
    float target_deg;
    float target_limit_deg;
    float hard_limit_deg;
    /* Current-loop outer-controller gains and limits. */
    float kp_a_per_deg;
    float ki_a_per_deg_s;
    float kd_a_per_rpm;
    /* Static current needed to carry the mechanism at the reference angle. */
    float feedforward_current_a;
    float error_limit_deg;
    float deadband_deg;
    float integral_zone_deg;
    float integral_limit_a;
    float current_limit_a;
    float current_polarity;
    /* Absolute single-turn raw recovery window. Outside it, normal PID is
     * suspended and only this bounded current toward the window is allowed. */
    uint16_t raw_min_limit;
    uint16_t raw_max_limit;
    float raw_recovery_current_a;
} Algorithm_LevelCurrentControl_Config;

typedef struct {
    Algorithm_LevelCurrentControl_Config config;
    float zero_angle_rad;
    float integral_current_a;
    bool zero_captured;
    bool active;
    bool initialized;
} Algorithm_LevelCurrentControl;

/* Pure absolute-encoder outer loop. It never accesses CAN, the QD4310 Device,
 * or the RTOS. CaptureZero() accepts one single-turn absolute position per
 * boot. Update() returns a CURRENT-mode command in amperes. */
bool Algorithm_LevelCurrentControl_Init(
    Algorithm_LevelCurrentControl *controller,
    const Algorithm_LevelCurrentControl_Config *config);
bool Algorithm_LevelCurrentControl_CaptureZero(
    Algorithm_LevelCurrentControl *controller,
    float absolute_angle_rad);
bool Algorithm_LevelCurrentControl_HasZero(
    const Algorithm_LevelCurrentControl *controller);
bool Algorithm_LevelCurrentControl_Start(
    Algorithm_LevelCurrentControl *controller);
/* Update a validated relative target at runtime and clear the accumulated I
 * term so a calibration step does not inherit effort from the previous one. */
bool Algorithm_LevelCurrentControl_SetTargetDeg(
    Algorithm_LevelCurrentControl *controller,
    float target_deg);
/* Continuous outer-loop tracking target. Unlike SetTargetDeg(), this keeps
 * the accumulated integral term and is safe to call for each ball-loop
 * update. */
bool Algorithm_LevelCurrentControl_SetTrackingTargetDeg(
    Algorithm_LevelCurrentControl *controller,
    float target_deg);
void Algorithm_LevelCurrentControl_Stop(
    Algorithm_LevelCurrentControl *controller);
bool Algorithm_LevelCurrentControl_IsActive(
    const Algorithm_LevelCurrentControl *controller);
bool Algorithm_LevelCurrentControl_GetRelativeAngleDeg(
    const Algorithm_LevelCurrentControl *controller,
    float absolute_angle_rad,
    float *relative_angle_deg);
/* Returns recovery_active=true and a bounded inward current when raw is
 * outside the configured window. Exact boundary values stay in normal PID.
 * The I term is cleared during recovery so it cannot wind up off-range. */
bool Algorithm_LevelCurrentControl_ComputeRawRecoveryCurrent(
    Algorithm_LevelCurrentControl *controller,
    uint16_t raw,
    float *current_a,
    bool *recovery_active);
bool Algorithm_LevelCurrentControl_Update(
    Algorithm_LevelCurrentControl *controller,
    float absolute_angle_rad,
    float speed_rpm,
    float *current_a);

#endif
