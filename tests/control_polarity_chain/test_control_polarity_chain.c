#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "algorithm_ball_position_control.h"
#include "algorithm_chassis_imu_acceleration.h"
#include "algorithm_level_current_control.h"
#include "algorithm_rod_angle_map.h"
#include "core_config.h"

#define TEST_PI       3.14159265358979323846F
#define TEST_DEG2RAD  (TEST_PI / 180.0F)

static const Algorithm_RodAngleMap_Point s_points[] =
    GATEWAY_ROD_ANGLE_CALIBRATION_POINTS;

static float raw_to_relative_motor_deg(float raw)
{
    return (raw - (float)GATEWAY_LEVEL_ZERO_RAW) * (360.0F / 65535.0F);
}

static float current_for_rod_target(float rod_target_deg)
{
    Algorithm_RodAngleMap map;
    Algorithm_LevelCurrentControl level;
    Algorithm_LevelCurrentControl_Config level_config = {
        .control_period_s =
            (float)GATEWAY_LEVEL_CONTROL_PERIOD_MS * 0.001F,
        .target_deg = 0.0F,
        .target_limit_deg = GATEWAY_LEVEL_TARGET_LIMIT_DEG,
        .hard_limit_deg = GATEWAY_LEVEL_HARD_LIMIT_DEG,
        .kp_a_per_deg = GATEWAY_LEVEL_KP_A_PER_DEG,
        .ki_a_per_deg_s = 0.0F,
        .kd_a_per_rpm = 0.0F,
        .feedforward_current_a = 0.0F,
        .error_limit_deg = GATEWAY_LEVEL_ERROR_LIMIT_DEG,
        .deadband_deg = 0.0F,
        .integral_zone_deg = GATEWAY_LEVEL_INTEGRAL_ZONE_DEG,
        .integral_limit_a = GATEWAY_LEVEL_INTEGRAL_LIMIT_A,
        .current_limit_a = GATEWAY_LEVEL_CURRENT_LIMIT_A,
        .current_polarity = GATEWAY_LEVEL_CURRENT_POLARITY,
        .raw_min_limit = GATEWAY_LEVEL_RAW_MIN_LIMIT,
        .raw_max_limit = GATEWAY_LEVEL_RAW_MAX_LIMIT,
        .raw_recovery_current_a =
            GATEWAY_LEVEL_RAW_RECOVERY_CURRENT_A,
    };
    float target_raw;
    float current_a;

    assert(Algorithm_RodAngleMap_Init(
        &map, s_points, sizeof(s_points) / sizeof(s_points[0]),
        GATEWAY_LEVEL_ZERO_RAW));
    assert(Algorithm_RodAngleMap_RodDegToRaw(
        &map, rod_target_deg, &target_raw));
    assert(Algorithm_LevelCurrentControl_Init(&level, &level_config));
    assert(Algorithm_LevelCurrentControl_CaptureZero(
        &level, (float)GATEWAY_LEVEL_ZERO_RAW *
            (2.0F * TEST_PI / 65535.0F)));
    assert(Algorithm_LevelCurrentControl_SetTrackingTargetDeg(
        &level, raw_to_relative_motor_deg(target_raw)));
    assert(Algorithm_LevelCurrentControl_Start(&level));
    assert(Algorithm_LevelCurrentControl_Update(
        &level,
        (float)GATEWAY_LEVEL_ZERO_RAW * (2.0F * TEST_PI / 65535.0F),
        0.0F, &current_a));
    return current_a;
}

static float visual_rod_target(float camera_position_mm)
{
    Algorithm_BallPositionControl controller;
    const Algorithm_BallPositionControl_Config config = {
        .target_position_mm = GATEWAY_BALL_TARGET_MM,
        .maximum_absolute_target_position_mm =
            GATEWAY_BALL_TARGET_LIMIT_MM,
        .target_slew_rate_mm_s = GATEWAY_BALL_TARGET_SLEW_MM_S,
        .maximum_target_velocity_mm_s =
            GATEWAY_BALL_MAX_TARGET_VELOCITY_MM_S,
        .maximum_target_acceleration_mm_s2 =
            GATEWAY_BALL_MAX_TARGET_ACCELERATION_MM_S2,
        .measurement_polarity = GATEWAY_BALL_MEASUREMENT_POLARITY,
        .position_gain_per_s2 = GATEWAY_BALL_POSITION_KP_PER_S2,
        .position_integral_gain_per_s3 =
            GATEWAY_BALL_POSITION_KI_PER_S3,
        .velocity_gain_per_s = GATEWAY_BALL_VELOCITY_KD_PER_S,
        .maximum_integral_acceleration_mm_s2 =
            GATEWAY_BALL_INTEGRAL_ACCEL_LIMIT_MM_S2,
        .velocity_filter_tau_s = GATEWAY_BALL_VELOCITY_FILTER_TAU_S,
        .velocity_estimation_window_ms =
            GATEWAY_BALL_VELOCITY_ESTIMATION_WINDOW_MS,
        .hold_position_error_mm =
            GATEWAY_BALL_HOLD_POSITION_ERROR_MM,
        .hold_maximum_velocity_mm_s =
            GATEWAY_BALL_HOLD_MAX_VELOCITY_MM_S,
        .hold_stable_duration_ms =
            GATEWAY_BALL_HOLD_STABLE_DURATION_MS,
        .hold_exit_position_error_mm =
            GATEWAY_BALL_HOLD_EXIT_POSITION_ERROR_MM,
        .hold_exit_maximum_velocity_mm_s =
            GATEWAY_BALL_HOLD_EXIT_MAX_VELOCITY_MM_S,
        .hold_exit_sample_count =
            GATEWAY_BALL_HOLD_EXIT_SAMPLE_COUNT,
        .near_target_error_mm =
            GATEWAY_BALL_NEAR_TARGET_ERROR_MM,
        .near_target_velocity_gain_multiplier =
            GATEWAY_BALL_NEAR_TARGET_KD_MULTIPLIER,
        .gravity_mm_s2 = GATEWAY_BALL_GRAVITY_MM_S2,
        .inertia_ratio = GATEWAY_BALL_INERTIA_RATIO,
        .rod_angle_limit_deg = GATEWAY_BALL_ROD_ANGLE_LIMIT_DEG,
        .maximum_velocity_dt_ms = GATEWAY_BALL_MAX_VELOCITY_DT_MS,
    };
    float rod_target_deg;

    assert(Algorithm_BallPositionControl_Init(&controller, &config));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 1U, camera_position_mm));
    assert(Algorithm_BallPositionControl_UpdateMeasurement(
        &controller, 101U, camera_position_mm));
    assert(Algorithm_BallPositionControl_ComputeRodAngleDeg(
        &controller, &rod_target_deg));
    return rod_target_deg;
}

static float processed_chassis_acceleration(float sensor_y_specific_force)
{
    Algorithm_ChassisImuAcceleration processor;
    Algorithm_ChassisImuAcceleration_Output output;
    const Algorithm_ChassisImuAcceleration_Config config = {
        .bias_calibration_samples = 1U,
        .notch_sample_frequency_hz =
            GATEWAY_BMI088_ACCEL_NOTCH_SAMPLE_HZ,
        .notch_center_frequency_hz =
            GATEWAY_BMI088_ACCEL_NOTCH_CENTER_HZ,
        .notch_quality_factor =
            GATEWAY_BMI088_ACCEL_NOTCH_QUALITY_FACTOR,
        .acceleration_lowpass_cutoff_frequency_hz =
            GATEWAY_BMI088_ACCEL_LOWPASS_CUTOFF_HZ,
        .maximum_absolute_acceleration_mm_s2 =
            GATEWAY_BMI088_ACCEL_MAX_ABS_OUTPUT_MM_S2,
        .maximum_sample_step_ms =
            GATEWAY_BMI088_ACCEL_MAX_SAMPLE_STEP_MS,
    };
    const Algorithm_ChassisImuAcceleration_Sample level_sample = {
        .sample_ms = 100U,
        .longitudinal_specific_force_mm_s2 = 0.0F,
    };
    const Algorithm_ChassisImuAcceleration_Sample accelerated_sample = {
        .sample_ms = 101U,
        .longitudinal_specific_force_mm_s2 = sensor_y_specific_force *
            GATEWAY_BMI088_ACCEL_Y_SIGN,
    };

    assert(Algorithm_ChassisImuAcceleration_Init(&processor, &config));
    assert(Algorithm_ChassisImuAcceleration_Update(
        &processor, &level_sample));
    assert(Algorithm_ChassisImuAcceleration_Update(
        &processor, &accelerated_sample));
    assert(Algorithm_ChassisImuAcceleration_GetOutput(
        &processor, &output));
    return output.filtered_acceleration_mm_s2;
}

static void test_visual_to_motor_chain(void)
{
    const float camera_left_rod_deg = visual_rod_target(-50.0F);
    const float camera_right_rod_deg = visual_rod_target(50.0F);

    assert(GATEWAY_BALL_VISUAL_CONTROL_ENABLE == 1U);
    /* Camera left is physical +x: raise the execution end, decrease raw,
     * and command the already-proven negative motor current. */
    assert(camera_left_rod_deg > 0.0F);
    assert(current_for_rod_target(camera_left_rod_deg) < 0.0F);
    /* Camera right is the fixed-pivot side and must produce the inverse. */
    assert(camera_right_rod_deg < 0.0F);
    assert(current_for_rod_target(camera_right_rod_deg) > 0.0F);
}

static void test_imu_and_rod_map_chain(void)
{
    Algorithm_RodAngleMap map;
    float raised_angle_deg;
    float forward_push_acceleration;
    float opposite_acceleration;
    float chassis_rod_deg;

    assert(Algorithm_RodAngleMap_Init(
        &map, s_points, sizeof(s_points) / sizeof(s_points[0]),
        GATEWAY_LEVEL_ZERO_RAW));
    assert(Algorithm_RodAngleMap_RawToRodDeg(
        &map, (float)GATEWAY_LEVEL_ZERO_RAW - 100.0F,
        &raised_angle_deg));
    /* JY61P+chassis-IMU scan evidence: raw down means indicated rod angle up. */
    assert(raised_angle_deg > 0.0F);

    forward_push_acceleration = processed_chassis_acceleration(500.0F);
    opposite_acceleration = processed_chassis_acceleration(-500.0F);
    /* Two motor-disabled forward pushes confirmed Sensor Y and the processed
     * longitudinal acceleration both go positive first. */
    assert(forward_push_acceleration > 0.0F);
    assert(opposite_acceleration < 0.0F);

    chassis_rod_deg = GATEWAY_CHASSIS_FF_GAIN *
        GATEWAY_CHASSIS_FF_POLARITY * atan2f(
            forward_push_acceleration, GATEWAY_BALL_GRAVITY_MM_S2) *
        (180.0F / TEST_PI);
    /* The execution end is at the vehicle front. Forward acceleration must
     * lower it: negative theta increases raw and requires positive current. */
    assert(chassis_rod_deg < 0.0F);
    assert(current_for_rod_target(chassis_rod_deg) > 0.0F);
}

int main(void)
{
    test_visual_to_motor_chain();
    test_imu_and_rod_map_chain();
    puts("control polarity chain tests: PASS");
    return 0;
}
