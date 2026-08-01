#ifndef APP_GATEWAY_H
#define APP_GATEWAY_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t motor_id;
    bool enabled;
    float current_a;
    float speed_rpm;
    uint16_t single_turn_angle_raw;
    uint16_t target_single_turn_angle_raw;
    float single_turn_angle_rad;
    float command_current_a;
    bool online;
    float rod_command_deg;
    float ball_position_mm;
    float ball_velocity_mm_s;
    float ball_integral_acceleration_mm_s2;
    float ball_stiction_kick_deg;
    float ball_target_mm;
    float ball_active_target_mm;
    float ball_acceleration_bias_mm_s2;
    float ball_position_variance_mm2;
    float ball_velocity_variance_mm2_s2;
    int32_t chassis_acceleration_mm_s2;
    int32_t planned_chassis_acceleration_mm_s2;
    float chassis_velocity_estimate_mm_s;
    float chassis_imu_bias_estimate_mm_s2;
    float chassis_acceleration_variance_mm2_s4;
    bool ball_measurement_valid;
    bool ball_velocity_valid;
    bool ball_hold_active;
    bool ball_last_used_sequence_valid;
    bool ball_estimator_trusted;
    bool chassis_acceleration_valid;
    bool chassis_acceleration_fusion_trusted;
    bool rod_angle_saturated;
    bool motor_raw_limit_blocked;
    uint8_t level_control_phase;
    uint16_t ball_last_used_sequence;
    float chassis_compensation_angle_deg;
    uint32_t ball_estimator_innovation_reject_count;
    uint32_t chassis_fusion_innovation_reject_count;
    uint32_t level_control_fault_count;
} APP_Gateway_Telemetry;

typedef struct {
    /* Fresh G3519 competition mode forwarded by the Task. Zero means the
     * mode is unavailable/offline for this control cycle. */
    uint8_t competition_mode;
    bool ball_measurement_valid;
    uint16_t ball_sequence;
    uint32_t ball_sample_ms;
    int16_t ball_position_mm;
    uint32_t ball_measurement_age_ms;

    bool chassis_acceleration_valid;
    /* True only when the primary acceleration input is a fresh trajectory
     * plan. Local BMI088-only fallback must not drive the fusion predictor. */
    bool chassis_acceleration_plan_valid;
    int32_t acceleration_now_mm_s2;
    int32_t acceleration_next_mm_s2;
    uint16_t acceleration_next_after_ms;
    uint32_t acceleration_received_at_ms;

    /* Local IMU acceleration and remote wheel speed are asynchronous
     * observations. Keep independent sample identities so neither Latest
     * Value is corrected more than once by the optional fusion filter. */
    bool chassis_motion_measurement_valid;
    uint16_t chassis_motion_sequence;
    uint32_t chassis_motion_sample_ms;
    uint32_t chassis_motion_age_ms;
    int32_t measured_chassis_acceleration_mm_s2;
    bool measured_chassis_acceleration_valid;

    bool chassis_wheel_measurement_valid;
    uint16_t chassis_wheel_sequence;
    uint32_t chassis_wheel_sample_ms;
    int32_t measured_chassis_wheel_speed_mm_s;

    bool chassis_wheel_acceleration_measurement_valid;
    uint16_t chassis_wheel_acceleration_sequence;
    uint32_t chassis_wheel_acceleration_sample_ms;
    int32_t measured_chassis_wheel_acceleration_mm_s2;
} APP_Gateway_ControlInput;

/* TCP<->CAN 双向网关。
 *
 * 下行（TCP 客户端 -> CAN/QD4310），Packet 帧 TYPE=CMD，payload 为 ASCII：
 *   "<motor_id>,ENABLE"          使能（仅非主电机）
 *   "<motor_id>,DISABLE"         失能
 *   "<motor_id>,CURRENT,<A>"     电流模式，单位 A（仅非主电机）
 *   "<motor_id>,SPEED,<rpm>"     速度模式，单位 rpm（仅非主电机）
 *   "<motor_id>,LOWSPEED,<rpm>"  低速模式，单位 rpm（仅非主电机）
 *   "<motor_id>,NOP"             仅请求一帧反馈
 * 主电机只接受NOP/DISABLE；使能和控制只能由带绝对raw保护的KEY闭环发出。
 *
 * 上行（CAN 反馈 -> TCP），TYPE=FB，payload：
 *   "<motor_id>,<en>,<current_A>,<speed_rpm>,<angle_rad>,<online>"
 * 命令执行结果 TYPE=ACK，payload："<seq>,<OK|ERR>"。
 *
 * 每周期由网关 Task 调用 APP_Gateway_Step()。 */

bool APP_Gateway_Init(uint32_t now_ms);
void APP_Gateway_Step(
    uint32_t now_ms,
    const APP_Gateway_ControlInput *control_input,
    APP_Gateway_Telemetry *primary_telemetry);

/* Toggle the first configured QD4310 between absolute-angle outer-loop hold
 * and Disable. Level hold commands only QD4310 CURRENT mode. The first start
 * uses the configured fixed absolute zero and requires fresh, stopped,
 * disabled feedback. Any single-turn raw may start; an off-range start first
 * uses bounded inward-only recovery. Stopping sends zero before Disable. */
bool APP_Gateway_TogglePrimaryMotor(uint32_t now_ms);

/* Idempotently start the same protected primary loop used by the local KEY.
 * An already-active loop remains active; unsafe or stale feedback is rejected. */
bool APP_Gateway_StartPrimaryMotor(uint32_t now_ms);

/* Unconditionally end the primary level/ball loop through the existing safe
 * path: command zero current before disabling the QD4310. This never starts
 * the motor when already stopped. */
bool APP_Gateway_StopPrimaryMotor(void);

/* Advance the Task6-only manual horizontal calibration while the protected
 * level loop is already enabled. KEY2 calls perform: continuous zero current,
 * capture the fresh QD4310 raw as the runtime horizontal reference, then
 * clear control state and resume the visual position loop. Task owns the
 * competition-mode/KEY2 gate. Stop/fault discards the runtime reference. */
bool APP_Gateway_AdvanceTask6LevelCalibration(uint32_t now_ms);

/* During the active visual-control phase and only after a usable visual
 * measurement in the preceding Gateway step, request a physical ball
 * position within the configured safe range. The active reference moves at
 * the configured slew rate; changing it does not clear ball state. */
bool APP_Gateway_SetBallTargetMm(float target_mm);

/* Apply bounded runtime ball-position PID gains without starting the motor or
 * changing the target. Product limits remain owned by core_config.h. */
bool APP_Gateway_SetBallPidGains(
    float kp_per_s2, float ki_per_s3, float kd_per_s);
bool APP_Gateway_GetBallPidGains(
    float *kp_per_s2, float *ki_per_s3, float *kd_per_s);

/* Temporary automatic bidirectional mechanism calibration. The first call
 * prepositions upward to the contracted upper endpoint, then starts the
 * configured down/up raw sweep. A second call stops immediately through
 * zero-current then Disable. The final held point also stops automatically. */
bool APP_Gateway_AdvancePrimaryMotorCalibration(uint32_t now_ms);

/* 遥测：命令计数/错误计数，调试用。 */
uint32_t APP_Gateway_CommandCount(void);
uint32_t APP_Gateway_CommandErrorCount(void);
bool APP_Gateway_WirelessReady(void);

#endif
