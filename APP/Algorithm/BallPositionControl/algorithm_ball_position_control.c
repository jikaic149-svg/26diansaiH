#include "algorithm_ball_position_control.h"

#include <math.h>
#include <stddef.h>

#define BALL_POSITION_CONTROL_RAD_TO_DEG \
    (180.0F / 3.14159265358979323846F)

static float limit_symmetric(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static void reset_hold_candidate(
    Algorithm_BallPositionControl *controller)
{
    controller->hold_candidate_started_ms = 0U;
    controller->hold_candidate_active = false;
}

static void reset_hold_exit(Algorithm_BallPositionControl *controller)
{
    controller->hold_exit_consecutive_count = 0U;
}

static void reset_speed_loop(
    Algorithm_BallPositionControl *controller)
{
    controller->target_velocity_mm_s = 0.0F;
    controller->integral_acceleration_mm_s2 = 0.0F;
    controller->desired_acceleration_mm_s2 = 0.0F;
    controller->stiction_assist_acceleration_mm_s2 = 0.0F;
    controller->stiction_assist_velocity_integral_acceleration_mm_s2 = 0.0F;
    controller->stiction_kick_angle_deg = 0.0F;
    reset_hold_candidate(controller);
}

static void reset_velocity_history(
    Algorithm_BallPositionControl *controller)
{
    controller->velocity_history_write_index = 0U;
    controller->velocity_history_count = 0U;
}

static void push_velocity_sample(
    Algorithm_BallPositionControl *controller,
    uint32_t sample_ms,
    float position_mm)
{
    const uint32_t index = controller->velocity_history_write_index;

    controller->velocity_history_position_mm[index] = position_mm;
    controller->velocity_history_sample_ms[index] = sample_ms;
    controller->velocity_history_write_index = (index + 1U) %
        ALGORITHM_BALL_POSITION_CONTROL_VELOCITY_HISTORY_CAPACITY;
    if (controller->velocity_history_count <
        ALGORITHM_BALL_POSITION_CONTROL_VELOCITY_HISTORY_CAPACITY) {
        ++controller->velocity_history_count;
    }
}

static bool calculate_window_velocity(
    const Algorithm_BallPositionControl *controller,
    uint32_t sample_ms,
    float position_mm,
    float *velocity_mm_s)
{
    uint32_t offset;

    if ((velocity_mm_s == NULL) ||
        (controller->velocity_history_count < 2U)) {
        return false;
    }
    for (offset = 1U;
         offset < controller->velocity_history_count;
         ++offset) {
        const uint32_t index =
            (controller->velocity_history_write_index +
             ALGORITHM_BALL_POSITION_CONTROL_VELOCITY_HISTORY_CAPACITY -
             1U - offset) %
            ALGORITHM_BALL_POSITION_CONTROL_VELOCITY_HISTORY_CAPACITY;
        const uint32_t elapsed_ms = sample_ms -
            controller->velocity_history_sample_ms[index];

        if (elapsed_ms >=
            controller->config.velocity_estimation_window_ms) {
            const float elapsed_s = (float)elapsed_ms / 1000.0F;
            const float velocity =
                (position_mm -
                 controller->velocity_history_position_mm[index]) /
                elapsed_s;

            if (!isfinite(velocity)) {
                return false;
            }
            *velocity_mm_s = velocity;
            return true;
        }
    }
    return false;
}

static void clear_speed_handoff_output(
    Algorithm_BallPositionControl *controller)
{
    controller->stiction_kick_angle_deg = 0.0F;
    controller->stiction_assist_acceleration_mm_s2 = 0.0F;
    controller->stiction_assist_velocity_integral_acceleration_mm_s2 = 0.0F;
    controller->target_velocity_mm_s = 0.0F;
}

static void reset_speed_handoff_trip(
    Algorithm_BallPositionControl *controller)
{
    clear_speed_handoff_output(controller);
    controller->stiction_state_started_ms = 0U;
    controller->stiction_error_direction = 0;
    controller->stiction_state = ALGORITHM_BALL_STICTION_IDLE;
}

static void latch_position_loop(
    Algorithm_BallPositionControl *controller)
{
    clear_speed_handoff_output(controller);
    controller->stiction_state_started_ms = 0U;
    controller->stiction_error_direction = 0;
    controller->stiction_state = ALGORITHM_BALL_STICTION_POSITION_LATCHED;
}

static int8_t error_direction(float error_mm)
{
    if (error_mm > 0.0F) {
        return 1;
    }
    if (error_mm < 0.0F) {
        return -1;
    }
    return 0;
}

static void begin_speed_handoff(
    Algorithm_BallPositionControl *controller,
    uint32_t sample_ms,
    int8_t direction)
{
    clear_speed_handoff_output(controller);
    controller->stiction_state = ALGORITHM_BALL_STICTION_ACTIVE;
    controller->stiction_state_started_ms = sample_ms;
    controller->stiction_error_direction = direction;
}

static void update_speed_handoff(
    Algorithm_BallPositionControl *controller,
    uint32_t sample_ms,
    uint32_t delta_ms)
{
    const float error_mm = controller->target_command_mm -
        controller->position_mm;
    const float absolute_error_mm = fabsf(error_mm);
    const float absolute_velocity_mm_s = fabsf(controller->velocity_mm_s);
    const int8_t direction = error_direction(error_mm);
    const bool configured =
        controller->config.stiction_assist_velocity_gain_per_s > 0.0F;
    const float exit_error_mm =
        controller->config.stiction_kick_minimum_error_mm;
    const float entry_error_mm = exit_error_mm +
        controller->config.stiction_assist_exit_distance_mm;
    const bool handoff_velocity_qualifies = controller->velocity_valid &&
        (absolute_velocity_mm_s <=
            controller->config.stiction_kick_maximum_velocity_mm_s);
    const bool minimum_assist_time_elapsed =
        (uint32_t)(sample_ms - controller->stiction_state_started_ms) >=
            controller->config.stiction_kick_duration_ms;

    if (!configured) {
        reset_speed_handoff_trip(controller);
        return;
    }
    if (!controller->speed_handoff_enabled) {
        latch_position_loop(controller);
        return;
    }
    if (!controller->velocity_valid) {
        clear_speed_handoff_output(controller);
        if (controller->stiction_state ==
                ALGORITHM_BALL_STICTION_POSITION_LATCHED) {
            controller->stiction_state_started_ms = 0U;
            controller->stiction_error_direction = 0;
        }
        return;
    }
    if (controller->stiction_state ==
            ALGORITHM_BALL_STICTION_POSITION_LATCHED) {
        clear_speed_handoff_output(controller);
        if ((controller->target_position_mm !=
                controller->target_command_mm) ||
            (absolute_error_mm <= entry_error_mm) ||
            !handoff_velocity_qualifies || (direction == 0)) {
            controller->stiction_state_started_ms = 0U;
            controller->stiction_error_direction = 0;
            return;
        }
        if (controller->stiction_error_direction != direction) {
            controller->stiction_state_started_ms = sample_ms;
            controller->stiction_error_direction = direction;
            return;
        }
        if ((uint32_t)(sample_ms -
                controller->stiction_state_started_ms) <
            controller->config.stiction_kick_duration_ms) {
            return;
        }
        begin_speed_handoff(controller, sample_ms, direction);
    }
    if (controller->stiction_state == ALGORITHM_BALL_STICTION_ACTIVE) {
        /* Speed PI is launch assistance only, but keep it long enough to
         * produce useful motion. After the minimum assist time, real motion
         * above the stiction threshold hands tracking to position PD. */
        if ((!handoff_velocity_qualifies && minimum_assist_time_elapsed) ||
            (direction == 0) ||
            (direction != controller->stiction_error_direction) ||
            (absolute_error_mm <= exit_error_mm)) {
            latch_position_loop(controller);
            return;
        }
    }
    else if ((absolute_error_mm > entry_error_mm) && (direction != 0)) {
        begin_speed_handoff(controller, sample_ms, direction);
    }
    else {
        /* This command starts inside the stiction entry band, so position PD
         * owns tracking unless a later sustained low-speed stall rearms the
         * one-shot launch assistance above. */
        latch_position_loop(controller);
        return;
    }

    if (controller->stiction_state == ALGORITHM_BALL_STICTION_ACTIVE) {
        const float braking_distance_mm = fmaxf(
            0.0F, absolute_error_mm - exit_error_mm);
        float planned_target_velocity_mm_s;
        float target_speed_magnitude_mm_s = sqrtf(
            2.0F * controller->config.maximum_target_acceleration_mm_s2 *
            braking_distance_mm);

        if (target_speed_magnitude_mm_s >
            controller->config.maximum_target_velocity_mm_s) {
            target_speed_magnitude_mm_s =
                controller->config.maximum_target_velocity_mm_s;
        }
        planned_target_velocity_mm_s = (float)direction *
            target_speed_magnitude_mm_s;
        if ((delta_ms > 0U) &&
            (delta_ms <= controller->config.maximum_velocity_dt_ms)) {
            const float maximum_velocity_step_mm_s =
                controller->config.maximum_target_acceleration_mm_s2 *
                ((float)delta_ms / 1000.0F);
            const float velocity_step_mm_s =
                planned_target_velocity_mm_s -
                controller->target_velocity_mm_s;

            if (velocity_step_mm_s > maximum_velocity_step_mm_s) {
                controller->target_velocity_mm_s +=
                    maximum_velocity_step_mm_s;
            }
            else if (velocity_step_mm_s < -maximum_velocity_step_mm_s) {
                controller->target_velocity_mm_s -=
                    maximum_velocity_step_mm_s;
            }
            else {
                controller->target_velocity_mm_s =
                    planned_target_velocity_mm_s;
            }
            controller->stiction_assist_velocity_integral_acceleration_mm_s2 +=
                controller->config.
                    stiction_assist_velocity_integral_gain_per_s2 *
                (controller->target_velocity_mm_s -
                    controller->velocity_mm_s) *
                ((float)delta_ms / 1000.0F);
            controller->stiction_assist_velocity_integral_acceleration_mm_s2 =
                limit_symmetric(
                    controller->
                        stiction_assist_velocity_integral_acceleration_mm_s2,
                    controller->config.maximum_target_acceleration_mm_s2);
        }
    }
}

static bool valid_config(
    const Algorithm_BallPositionControl_Config *config)
{
    return (config != NULL) &&
        isfinite(config->target_position_mm) &&
        isfinite(config->maximum_absolute_target_position_mm) &&
        (config->maximum_absolute_target_position_mm > 0.0F) &&
        (fabsf(config->target_position_mm) <=
            config->maximum_absolute_target_position_mm) &&
        isfinite(config->target_slew_rate_mm_s) &&
        (config->target_slew_rate_mm_s > 0.0F) &&
        isfinite(config->maximum_target_velocity_mm_s) &&
        (config->maximum_target_velocity_mm_s > 0.0F) &&
        isfinite(config->maximum_target_acceleration_mm_s2) &&
        (config->maximum_target_acceleration_mm_s2 > 0.0F) &&
        isfinite(config->measurement_polarity) &&
        ((config->measurement_polarity == 1.0F) ||
         (config->measurement_polarity == -1.0F)) &&
        isfinite(config->position_gain_per_s2) &&
        (config->position_gain_per_s2 >= 0.0F) &&
        isfinite(config->position_integral_gain_per_s3) &&
        (config->position_integral_gain_per_s3 >= 0.0F) &&
        isfinite(config->velocity_gain_per_s) &&
        (config->velocity_gain_per_s > 0.0F) &&
        isfinite(config->maximum_integral_acceleration_mm_s2) &&
        (config->maximum_integral_acceleration_mm_s2 >= 0.0F) &&
        ((config->position_integral_gain_per_s3 == 0.0F) ||
         (config->maximum_integral_acceleration_mm_s2 > 0.0F)) &&
        isfinite(config->velocity_filter_tau_s) &&
        (config->velocity_filter_tau_s >= 0.0F) &&
        (config->velocity_estimation_window_ms > 0U) &&
        (config->velocity_estimation_window_ms <= UINT32_C(1000)) &&
        isfinite(config->hold_position_error_mm) &&
        (config->hold_position_error_mm >= 0.0F) &&
        isfinite(config->hold_maximum_velocity_mm_s) &&
        (config->hold_maximum_velocity_mm_s >= 0.0F) &&
        isfinite(config->hold_exit_position_error_mm) &&
        (config->hold_exit_position_error_mm >= 0.0F) &&
        isfinite(config->hold_exit_maximum_velocity_mm_s) &&
        (config->hold_exit_maximum_velocity_mm_s >= 0.0F) &&
        (((config->hold_position_error_mm == 0.0F) &&
          (config->hold_maximum_velocity_mm_s == 0.0F) &&
          (config->hold_exit_position_error_mm == 0.0F) &&
          (config->hold_exit_maximum_velocity_mm_s == 0.0F) &&
          (config->hold_exit_sample_count == 0U)) ||
         ((config->hold_position_error_mm > 0.0F) &&
          (config->hold_maximum_velocity_mm_s > 0.0F) &&
          (config->hold_stable_duration_ms > 0U) &&
          (config->hold_exit_position_error_mm >
            config->hold_position_error_mm) &&
          (config->hold_exit_maximum_velocity_mm_s >
            config->hold_maximum_velocity_mm_s) &&
          (config->hold_exit_sample_count > 0U))) &&
        isfinite(config->near_target_error_mm) &&
        (config->near_target_error_mm >= 0.0F) &&
        isfinite(config->near_target_velocity_gain_multiplier) &&
        (((config->near_target_error_mm == 0.0F) &&
          (config->near_target_velocity_gain_multiplier == 0.0F)) ||
         ((config->near_target_error_mm > 0.0F) &&
          (config->near_target_velocity_gain_multiplier >= 1.0F))) &&
        isfinite(config->gravity_mm_s2) &&
        (config->gravity_mm_s2 > 0.0F) &&
        isfinite(config->inertia_ratio) &&
        (config->inertia_ratio >= 0.0F) &&
        isfinite(config->rod_angle_limit_deg) &&
        (config->rod_angle_limit_deg > 0.0F) &&
        (config->rod_angle_limit_deg < 90.0F) &&
        (config->maximum_velocity_dt_ms > 0U) &&
        (config->maximum_velocity_dt_ms <= UINT32_C(1000)) &&
        isfinite(config->stiction_kick_angle_deg) &&
        (config->stiction_kick_angle_deg == 0.0F) &&
        isfinite(config->stiction_kick_step_angle_deg) &&
        (config->stiction_kick_step_angle_deg == 0.0F) &&
        isfinite(config->stiction_kick_maximum_angle_deg) &&
        (config->stiction_kick_maximum_angle_deg == 0.0F) &&
        isfinite(config->stiction_assist_velocity_gain_per_s) &&
        (config->stiction_assist_velocity_gain_per_s >= 0.0F) &&
        isfinite(config->stiction_assist_velocity_integral_gain_per_s2) &&
        (config->stiction_assist_velocity_integral_gain_per_s2 >= 0.0F) &&
        isfinite(config->stiction_assist_exit_distance_mm) &&
        (config->stiction_assist_exit_distance_mm >= 0.0F) &&
        isfinite(config->stiction_kick_minimum_error_mm) &&
        (config->stiction_kick_minimum_error_mm >= 0.0F) &&
        isfinite(config->stiction_kick_maximum_velocity_mm_s) &&
        (config->stiction_kick_maximum_velocity_mm_s >= 0.0F) &&
        (((config->stiction_assist_velocity_gain_per_s == 0.0F) &&
          (config->stiction_kick_minimum_error_mm == 0.0F) &&
          (config->stiction_kick_maximum_velocity_mm_s == 0.0F) &&
          (config->stiction_assist_exit_distance_mm == 0.0F) &&
          (config->stiction_kick_duration_ms == 0U)) ||
         ((config->stiction_kick_minimum_error_mm > 0.0F) &&
          (config->stiction_kick_maximum_velocity_mm_s > 0.0F) &&
          (config->stiction_assist_velocity_gain_per_s > 0.0F) &&
          (config->stiction_assist_exit_distance_mm > 0.0F) &&
          (config->stiction_kick_duration_ms > 0U)));
}

bool Algorithm_BallPositionControl_Init(
    Algorithm_BallPositionControl *controller,
    const Algorithm_BallPositionControl_Config *config)
{
    if (controller != NULL) {
        *controller = (Algorithm_BallPositionControl){0};
    }
    if ((controller == NULL) || !valid_config(config)) {
        return false;
    }
    controller->config = *config;
    controller->target_position_mm = config->target_position_mm;
    controller->target_command_mm = config->target_position_mm;
    controller->speed_handoff_enabled = true;
    reset_speed_handoff_trip(controller);
    controller->initialized = true;
    return true;
}

bool Algorithm_BallPositionControl_SetTargetPositionMm(
    Algorithm_BallPositionControl *controller,
    float target_position_mm)
{
    if ((controller == NULL) || !controller->initialized ||
        !isfinite(target_position_mm) ||
        (fabsf(target_position_mm) >
            controller->config.maximum_absolute_target_position_mm)) {
        return false;
    }
    if (target_position_mm != controller->target_command_mm) {
        controller->hold_active = false;
        reset_hold_exit(controller);
        controller->integral_acceleration_mm_s2 = 0.0F;
        reset_hold_candidate(controller);
        reset_speed_handoff_trip(controller);
    }
    controller->target_command_mm = target_position_mm;
    return true;
}

static void update_position_loop_state(
    Algorithm_BallPositionControl *controller,
    uint32_t sample_ms,
    uint32_t delta_ms)
{
    const float position_error_mm = controller->target_position_mm -
        controller->position_mm;
    float dt_s;
    bool hold_qualifies;

    if (!controller->velocity_valid || (delta_ms == 0U) ||
        (delta_ms > controller->config.maximum_velocity_dt_ms)) {
        reset_speed_loop(controller);
        return;
    }
    if (controller->hold_active) {
        const bool exit_exceeded =
            (fabsf(controller->target_command_mm - controller->position_mm) >=
                controller->config.hold_exit_position_error_mm) ||
            (fabsf(controller->velocity_mm_s) >=
                controller->config.hold_exit_maximum_velocity_mm_s);

        if (exit_exceeded) {
            if (controller->hold_exit_consecutive_count < UINT32_MAX) {
                ++controller->hold_exit_consecutive_count;
            }
        }
        else {
            reset_hold_exit(controller);
        }
        if (controller->hold_exit_consecutive_count <
            controller->config.hold_exit_sample_count) {
            reset_speed_loop(controller);
            return;
        }
        controller->hold_active = false;
        controller->integral_acceleration_mm_s2 = 0.0F;
        reset_hold_exit(controller);
        reset_hold_candidate(controller);
        latch_position_loop(controller);
    }
    if (controller->stiction_state == ALGORITHM_BALL_STICTION_ACTIVE) {
        controller->integral_acceleration_mm_s2 = 0.0F;
        reset_hold_candidate(controller);
        return;
    }

    if (controller->target_position_mm != controller->target_command_mm) {
        controller->integral_acceleration_mm_s2 = 0.0F;
    }
    else {
        dt_s = (float)delta_ms / 1000.0F;
        controller->integral_acceleration_mm_s2 +=
            controller->config.position_integral_gain_per_s3 *
            position_error_mm * dt_s;
        controller->integral_acceleration_mm_s2 = limit_symmetric(
            controller->integral_acceleration_mm_s2,
            controller->config.maximum_integral_acceleration_mm_s2);
    }

    hold_qualifies =
        (controller->config.hold_position_error_mm > 0.0F) &&
        (controller->target_position_mm == controller->target_command_mm) &&
        (fabsf(controller->target_command_mm - controller->position_mm) <=
            controller->config.hold_position_error_mm) &&
        (fabsf(controller->velocity_mm_s) <=
            controller->config.hold_maximum_velocity_mm_s);
    if (!hold_qualifies) {
        reset_hold_candidate(controller);
    }
    else if (!controller->hold_candidate_active) {
        controller->hold_candidate_active = true;
        controller->hold_candidate_started_ms = sample_ms;
    }
    else if ((uint32_t)(sample_ms -
            controller->hold_candidate_started_ms) >=
            controller->config.hold_stable_duration_ms) {
        controller->hold_active = true;
        reset_hold_exit(controller);
        reset_speed_loop(controller);
    }
}

bool Algorithm_BallPositionControl_AdvanceTarget(
    Algorithm_BallPositionControl *controller,
    uint32_t elapsed_ms)
{
    float delta_mm;
    float maximum_step_mm;

    if ((controller == NULL) || !controller->initialized) {
        return false;
    }
    delta_mm = controller->target_command_mm -
        controller->target_position_mm;
    maximum_step_mm = controller->config.target_slew_rate_mm_s *
        ((float)elapsed_ms / 1000.0F);
    if (!isfinite(delta_mm) || !isfinite(maximum_step_mm)) {
        return false;
    }
    if (delta_mm > maximum_step_mm) {
        controller->target_position_mm += maximum_step_mm;
    }
    else if (delta_mm < -maximum_step_mm) {
        controller->target_position_mm -= maximum_step_mm;
    }
    else {
        controller->target_position_mm =
            controller->target_command_mm;
    }
    return isfinite(controller->target_position_mm);
}

bool Algorithm_BallPositionControl_ResetTargetPositionMm(
    Algorithm_BallPositionControl *controller,
    float target_position_mm)
{
    if (!Algorithm_BallPositionControl_SetTargetPositionMm(
            controller, target_position_mm)) {
        return false;
    }
    controller->target_position_mm = target_position_mm;
    return true;
}

bool Algorithm_BallPositionControl_SetSpeedHandoffEnabled(
    Algorithm_BallPositionControl *controller,
    bool enabled)
{
    if ((controller == NULL) || !controller->initialized) {
        return false;
    }
    if (controller->speed_handoff_enabled == enabled) {
        return true;
    }
    controller->speed_handoff_enabled = enabled;
    controller->integral_acceleration_mm_s2 = 0.0F;
    if (enabled) {
        reset_speed_handoff_trip(controller);
    }
    else {
        latch_position_loop(controller);
    }
    return true;
}

bool Algorithm_BallPositionControl_SetMotionLimits(
    Algorithm_BallPositionControl *controller,
    float maximum_target_velocity_mm_s,
    float maximum_target_acceleration_mm_s2)
{
    if ((controller == NULL) || !controller->initialized ||
        !isfinite(maximum_target_velocity_mm_s) ||
        (maximum_target_velocity_mm_s <= 0.0F) ||
        !isfinite(maximum_target_acceleration_mm_s2) ||
        (maximum_target_acceleration_mm_s2 <= 0.0F)) {
        return false;
    }
    if ((controller->config.maximum_target_velocity_mm_s ==
            maximum_target_velocity_mm_s) &&
        (controller->config.maximum_target_acceleration_mm_s2 ==
            maximum_target_acceleration_mm_s2)) {
        return true;
    }
    controller->config.maximum_target_velocity_mm_s =
        maximum_target_velocity_mm_s;
    controller->config.maximum_target_acceleration_mm_s2 =
        maximum_target_acceleration_mm_s2;
    controller->hold_active = false;
    reset_hold_exit(controller);
    reset_speed_loop(controller);
    reset_speed_handoff_trip(controller);
    return true;
}

bool Algorithm_BallPositionControl_SetFinalPositionBands(
    Algorithm_BallPositionControl *controller,
    float speed_handoff_error_mm,
    float hold_position_error_mm)
{
    if ((controller == NULL) || !controller->initialized ||
        !isfinite(speed_handoff_error_mm) ||
        !isfinite(hold_position_error_mm) ||
        (speed_handoff_error_mm <= 0.0F) ||
        (hold_position_error_mm <= 0.0F) ||
        (hold_position_error_mm >=
            controller->config.hold_exit_position_error_mm)) {
        return false;
    }
    if ((controller->config.stiction_kick_minimum_error_mm ==
            speed_handoff_error_mm) &&
        (controller->config.hold_position_error_mm ==
            hold_position_error_mm)) {
        return true;
    }
    controller->config.stiction_kick_minimum_error_mm =
        speed_handoff_error_mm;
    controller->config.hold_position_error_mm =
        hold_position_error_mm;
    controller->hold_active = false;
    reset_hold_exit(controller);
    reset_speed_loop(controller);
    reset_speed_handoff_trip(controller);
    return true;
}

bool Algorithm_BallPositionControl_SetGains(
    Algorithm_BallPositionControl *controller,
    float position_gain_per_s2,
    float position_integral_gain_per_s3,
    float velocity_gain_per_s)
{
    if ((controller == NULL) || !controller->initialized ||
        !isfinite(position_gain_per_s2) ||
        !isfinite(position_integral_gain_per_s3) ||
        !isfinite(velocity_gain_per_s) ||
        (position_gain_per_s2 < 0.0F) ||
        (position_integral_gain_per_s3 < 0.0F) ||
        (velocity_gain_per_s <= 0.0F) ||
        ((position_integral_gain_per_s3 > 0.0F) &&
         (controller->config.maximum_integral_acceleration_mm_s2 <= 0.0F))) {
        return false;
    }
    controller->config.position_gain_per_s2 = position_gain_per_s2;
    controller->config.position_integral_gain_per_s3 =
        position_integral_gain_per_s3;
    controller->config.velocity_gain_per_s = velocity_gain_per_s;
    if (position_integral_gain_per_s3 == 0.0F) {
        controller->integral_acceleration_mm_s2 = 0.0F;
    }
    return true;
}

bool Algorithm_BallPositionControl_UpdateMeasurement(
    Algorithm_BallPositionControl *controller,
    uint32_t sample_ms,
    float camera_position_mm)
{
    float position_mm;
    uint32_t delta_ms = 0U;
    float raw_velocity_mm_s = 0.0F;
    bool velocity_available;

    if ((controller == NULL) || !controller->initialized ||
        !isfinite(camera_position_mm)) {
        return false;
    }
    position_mm = camera_position_mm *
        controller->config.measurement_polarity;
    if (!isfinite(position_mm)) {
        return false;
    }

    if (controller->measurement_seen) {
        delta_ms = sample_ms - controller->last_sample_ms;

        if ((delta_ms == 0U) ||
            (delta_ms > controller->config.maximum_velocity_dt_ms)) {
            reset_velocity_history(controller);
            controller->velocity_mm_s = 0.0F;
            controller->velocity_valid = false;
            reset_speed_loop(controller);
        }
    }
    else {
        reset_velocity_history(controller);
        controller->velocity_mm_s = 0.0F;
        reset_speed_loop(controller);
        controller->velocity_valid = false;
    }

    push_velocity_sample(controller, sample_ms, position_mm);
    velocity_available = calculate_window_velocity(
        controller, sample_ms, position_mm, &raw_velocity_mm_s);
    if (velocity_available) {
        const float dt_s = (float)delta_ms / 1000.0F;
        const float alpha =
            (controller->config.velocity_filter_tau_s <= 0.0F) ?
            1.0F : dt_s /
            (controller->config.velocity_filter_tau_s + dt_s);

        if (!isfinite(alpha)) {
            return false;
        }
        controller->velocity_mm_s +=
            alpha * (raw_velocity_mm_s - controller->velocity_mm_s);
        controller->velocity_valid = true;
    }
    else {
        controller->velocity_mm_s = 0.0F;
        controller->velocity_valid = false;
        reset_speed_loop(controller);
    }
    controller->position_mm = position_mm;
    update_position_loop_state(controller, sample_ms, delta_ms);
    update_speed_handoff(controller, sample_ms, delta_ms);
    controller->last_sample_ms = sample_ms;
    controller->measurement_seen = true;
    return true;
}

bool Algorithm_BallPositionControl_SetEstimatedState(
    Algorithm_BallPositionControl *controller,
    uint32_t sample_ms,
    float position_mm,
    float velocity_mm_s)
{
    uint32_t delta_ms = 0U;
    float filtered_velocity_mm_s;

    if ((controller == NULL) || !controller->initialized ||
        !isfinite(position_mm) || !isfinite(velocity_mm_s)) {
        return false;
    }
    filtered_velocity_mm_s = velocity_mm_s;
    if (controller->measurement_seen) {
        delta_ms = sample_ms - controller->last_sample_ms;
        if ((delta_ms > 0U) &&
            (delta_ms <= controller->config.maximum_velocity_dt_ms)) {
            const float dt_s = (float)delta_ms / 1000.0F;
            const float alpha =
                (controller->config.velocity_filter_tau_s <= 0.0F) ?
                1.0F : dt_s /
                (controller->config.velocity_filter_tau_s + dt_s);

            filtered_velocity_mm_s = controller->velocity_mm_s +
                alpha * (velocity_mm_s - controller->velocity_mm_s);
        }
        else if (delta_ms == 0U) {
            filtered_velocity_mm_s = controller->velocity_mm_s;
        }
    }
    else {
        reset_speed_loop(controller);
    }
    controller->position_mm = position_mm;
    controller->velocity_mm_s = filtered_velocity_mm_s;
    controller->measurement_seen = true;
    controller->velocity_valid = true;
    update_position_loop_state(controller, sample_ms, delta_ms);
    update_speed_handoff(controller, sample_ms, delta_ms);
    controller->last_sample_ms = sample_ms;
    return true;
}

void Algorithm_BallPositionControl_ResetMeasurement(
    Algorithm_BallPositionControl *controller)
{
    if ((controller == NULL) || !controller->initialized) {
        return;
    }
    controller->position_mm = 0.0F;
    controller->velocity_mm_s = 0.0F;
    reset_speed_loop(controller);
    controller->last_sample_ms = 0U;
    reset_velocity_history(controller);
    reset_speed_handoff_trip(controller);
    controller->measurement_seen = false;
    controller->velocity_valid = false;
    controller->hold_active = false;
    reset_hold_exit(controller);
}

bool Algorithm_BallPositionControl_ComputeRodAngleDeg(
    Algorithm_BallPositionControl *controller,
    float *rod_angle_deg)
{
    float measured_velocity;
    float velocity_gain;
    float position_acceleration;
    float speed_acceleration;
    float desired_acceleration;
    float assist_acceleration_limit;
    float assist_ramp_fraction;
    float normalized;
    float angle;

    if (rod_angle_deg != NULL) {
        *rod_angle_deg = 0.0F;
    }
    if ((controller == NULL) || !controller->initialized ||
        !controller->measurement_seen || (rod_angle_deg == NULL)) {
        return false;
    }

    if (controller->hold_active) {
        controller->target_velocity_mm_s = 0.0F;
        controller->integral_acceleration_mm_s2 = 0.0F;
        controller->desired_acceleration_mm_s2 = 0.0F;
        *rod_angle_deg = 0.0F;
        return true;
    }
    measured_velocity = controller->velocity_valid ?
        controller->velocity_mm_s : 0.0F;
    velocity_gain = controller->config.velocity_gain_per_s;
    if ((controller->config.near_target_error_mm > 0.0F) &&
        (fabsf(controller->target_position_mm - controller->position_mm) <=
            controller->config.near_target_error_mm)) {
        velocity_gain *=
            controller->config.near_target_velocity_gain_multiplier;
    }
    position_acceleration = controller->config.position_gain_per_s2 *
        (controller->target_position_mm - controller->position_mm) +
        controller->integral_acceleration_mm_s2 -
        velocity_gain * measured_velocity;
    desired_acceleration = position_acceleration;
    controller->stiction_assist_acceleration_mm_s2 = 0.0F;
    controller->stiction_kick_angle_deg = 0.0F;
    if (controller->stiction_state == ALGORITHM_BALL_STICTION_ACTIVE) {
        assist_ramp_fraction = (float)(controller->last_sample_ms -
            controller->stiction_state_started_ms) /
            (float)controller->config.stiction_kick_duration_ms;
        assist_ramp_fraction = limit_symmetric(
            assist_ramp_fraction, 1.0F);
        if (assist_ramp_fraction < 0.0F) {
            assist_ramp_fraction = 0.0F;
        }
        assist_acceleration_limit =
            controller->config.maximum_target_acceleration_mm_s2 *
            (0.5F + 0.5F * assist_ramp_fraction);
        speed_acceleration = limit_symmetric(
            controller->config.stiction_assist_velocity_gain_per_s *
                (controller->target_velocity_mm_s - measured_velocity) +
            controller->
                stiction_assist_velocity_integral_acceleration_mm_s2,
            assist_acceleration_limit);
        controller->stiction_assist_acceleration_mm_s2 =
            speed_acceleration;
        /* Speed control exclusively owns the distant trip. No position-loop
         * blend and no discrete stiction kick are added. */
        desired_acceleration = speed_acceleration;
    }
    if (!isfinite(desired_acceleration)) {
        return false;
    }
    controller->desired_acceleration_mm_s2 = desired_acceleration;

    normalized = (1.0F + controller->config.inertia_ratio) *
        desired_acceleration / controller->config.gravity_mm_s2;
    normalized = limit_symmetric(normalized, 1.0F);
    /* Positive rod angle raises the execution end and accelerates the ball
     * toward the fixed pivot, hence the minus sign in the inverse model. */
    angle = -asinf(normalized) * BALL_POSITION_CONTROL_RAD_TO_DEG;
    if (!isfinite(angle)) {
        return false;
    }
    *rod_angle_deg = limit_symmetric(
        angle, controller->config.rod_angle_limit_deg);
    return true;
}
