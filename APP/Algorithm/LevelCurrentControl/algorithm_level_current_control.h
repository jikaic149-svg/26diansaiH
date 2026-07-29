#ifndef ALGORITHM_LEVEL_CURRENT_CONTROL_H
#define ALGORITHM_LEVEL_CURRENT_CONTROL_H

#include <stdbool.h>

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
    float error_limit_deg;
    float deadband_deg;
    float integral_zone_deg;
    float integral_limit_a;
    float current_limit_a;
    float current_polarity;
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
void Algorithm_LevelCurrentControl_Stop(
    Algorithm_LevelCurrentControl *controller);
bool Algorithm_LevelCurrentControl_IsActive(
    const Algorithm_LevelCurrentControl *controller);
bool Algorithm_LevelCurrentControl_GetRelativeAngleDeg(
    const Algorithm_LevelCurrentControl *controller,
    float absolute_angle_rad,
    float *relative_angle_deg);
bool Algorithm_LevelCurrentControl_Update(
    Algorithm_LevelCurrentControl *controller,
    float absolute_angle_rad,
    float speed_rpm,
    float *current_a);

#endif
