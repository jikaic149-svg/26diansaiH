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
    float single_turn_angle_rad;
    bool online;
} APP_Gateway_Telemetry;

/* TCP<->CAN 双向网关。
 *
 * 下行（TCP 客户端 -> CAN/QD4310），Packet 帧 TYPE=CMD，payload 为 ASCII：
 *   "<motor_id>,ENABLE"          使能
 *   "<motor_id>,DISABLE"         失能
 *   "<motor_id>,CURRENT,<A>"     电流模式，单位 A
 *   "<motor_id>,SPEED,<rpm>"     速度模式，单位 rpm
 *   "<motor_id>,LOWSPEED,<rpm>"  低速模式，单位 rpm
 *   "<motor_id>,NOP"             仅请求一帧反馈
 *
 * 上行（CAN 反馈 -> TCP），TYPE=FB，payload：
 *   "<motor_id>,<en>,<current_A>,<speed_rpm>,<angle_rad>,<online>"
 * 命令执行结果 TYPE=ACK，payload："<seq>,<OK|ERR>"。
 *
 * 每周期由网关 Task 调用 APP_Gateway_Step()。 */

bool APP_Gateway_Init(uint32_t now_ms);
void APP_Gateway_Step(
    uint32_t now_ms, APP_Gateway_Telemetry *primary_telemetry);

/* Toggle the first configured QD4310 between absolute-angle outer-loop hold
 * and Disable. Level hold commands only QD4310 CURRENT mode. The first start
 * uses the configured fixed absolute zero and requires fresh, stopped,
 * disabled feedback; stopping sends zero current before Disable. */
bool APP_Gateway_TogglePrimaryMotor(uint32_t now_ms);

/* 遥测：命令计数/错误计数，调试用。 */
uint32_t APP_Gateway_CommandCount(void);
uint32_t APP_Gateway_CommandErrorCount(void);
bool APP_Gateway_WirelessReady(void);

#endif
