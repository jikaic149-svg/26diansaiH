#ifndef CORE_CONFIG_H
#define CORE_CONFIG_H

/* F407-Gateway 核心可调参数唯一入口。
 * 只收录源码实际消费、需按产品整定的默认值。
 * 引脚/时钟/中断映射属于 BSP，FreeRTOS 内核参数属于 FreeRTOSConfig.h，
 * QD4310 协议常量属于 Device/QD4310。
 */

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Task 与调度                                                          */
/* ------------------------------------------------------------------ */

/* 网关控制 Task：周期 ms、tskIDLE_PRIORITY 之上的偏移、栈深(StackType_t)。 */
#define GATEWAY_TASK_PERIOD_MS            UINT32_C(10)
#define GATEWAY_TASK_PRIORITY_OFFSET      2U
#define GATEWAY_TASK_STACK_DEPTH          512U
#define GATEWAY_TASK_NAME                 "gateway"

/* ------------------------------------------------------------------ */
/* ESP8266 无线（AP + TCP 服务器）                                       */
/* ------------------------------------------------------------------ */

/* ESP8266 建热点的 SSID / 密码（密码至少 8 字符，WPA2）。 */
/* 本机可在被 Git 忽略的 local_secrets.h 中覆盖以下默认值。 */
#if defined(__has_include)
#if __has_include("local_secrets.h")
#include "local_secrets.h"
#endif
#endif
#ifndef ESP8266_AP_SSID
#define ESP8266_AP_SSID                   "F407-Gateway"
#endif
#ifndef ESP8266_AP_PASSWORD
#define ESP8266_AP_PASSWORD               "change-me"
#endif
/* AP 信道 1..13，热点最大同时接入 4 个客户端（固件上限）。 */
#define ESP8266_AP_CHANNEL                5

/* TCP 服务器监听端口，客户端（手机/电脑）连接此端口收发帧。 */
#define ESP8266_TCP_SERVER_PORT           3333

/* AT 交互超时与重连节奏，单位 ms。 */
#define ESP8266_BOOT_WAIT_MS              UINT32_C(1500)
#define ESP8266_AT_TIMEOUT_MS             UINT32_C(2000)
#define ESP8266_RECONNECT_INTERVAL_MS     UINT32_C(3000)

/* ------------------------------------------------------------------ */
/* TCP<->CAN 网关                                                       */
/* ------------------------------------------------------------------ */

/* 总线上 QD4310 电机数量与各自 CAN ID（0x00..0x0F，总线内唯一）。 */
#define GATEWAY_MOTOR_COUNT               1U
#define GATEWAY_MOTOR_IDS                 { 0x01U }

/* 板载 KEY1(PA0，高有效)的消抖时间，单位 ms；稳定按下只触发一次。 */
#define GATEWAY_KEY_DEBOUNCE_MS           UINT32_C(30)

/* 临时五点机构标定模式：1时KEY依次切换水平附近5个电机相对角，第6次执行0 A + Disable；
 * 0时恢复正常的一次启动、一次停止。角度单位deg，均须位于TARGET_LIMIT内。
 * 以零点54379换算，五点约为raw=53579/53979/54379/54779/55179。 */
#define GATEWAY_LEVEL_CALIBRATION_KEY_MODE 1U
#define GATEWAY_LEVEL_CALIBRATION_TARGETS_DEG \
    { -4.395F, -2.197F, 0.0F, 2.197F, 4.395F }

/* 周期性向电机发 NOP 请求反馈并把反馈帧回传 TCP 的周期，单位 ms。
 * QD4310 是应答式：只有收到控制/NOP 帧才回反馈帧。 */
#define GATEWAY_FEEDBACK_PERIOD_MS        UINT32_C(100)

/* CAN 发送等待空邮箱的上限，单位 ms；网关任务周期 10ms，不允许长阻塞。 */
#define GATEWAY_CAN_SEND_TIMEOUT_MS       UINT32_C(2)

/* QD4310 产品侧命令限幅：电流 A、速度 rpm。不得超过 Device 登记的 2 A / 800 rpm
 * 物理峰值；首轮保持额定值 1 A / 500 rpm。 */
#define GATEWAY_MOTOR_CURRENT_LIMIT_A     1.0F
#define GATEWAY_MOTOR_SPEED_LIMIT_RPM     500.0F

/* 反馈在线超时，单位 ms。必须大于反馈请求周期并留出 Task/CAN 抖动余量。 */
#define GATEWAY_MOTOR_FEEDBACK_TIMEOUT_MS UINT32_C(300)

/* ------------------------------------------------------------------ */
/* 水管绝对角外环 -> QD4310 CURRENT 电流环                              */
/* ------------------------------------------------------------------ */

/* 控制器名义周期和新反馈超时，单位 ms。控制只在收到一帧新反馈时更新一次；
 * 名义周期参与积分计算，必须与当前 Gateway Task 周期一致。 */
#define GATEWAY_LEVEL_CONTROL_PERIOD_MS    UINT32_C(10)
#define GATEWAY_LEVEL_FEEDBACK_TIMEOUT_MS UINT32_C(100)

/* 最新VOFA最后连续使能段末100帧raw均为54379、标准差0、转速0；用户确认后将其设为新的机械水平零点。
 * 原始值范围0..65535；改变安装关系后必须重新标定。上电不再以当前位置覆盖该值。 */
#define GATEWAY_LEVEL_ZERO_RAW             UINT16_C(54379)

/* KEY启动闭环时允许的最大绝对转速，单位rpm；必须>=0。 */
#define GATEWAY_LEVEL_START_MAX_SPEED_RPM  1.0F

/* 相对固定机械零点的目标角、目标允许范围和机械硬限位，单位deg。
 * 固定零点标定为54379后，按用户要求将运行目标再减小约50count至54329；
 * |TARGET|<=TARGET_LIMIT<=HARD_LIMIT<=180。 */
#define GATEWAY_LEVEL_TARGET_DEG           -0.275F
#define GATEWAY_LEVEL_TARGET_LIMIT_DEG     85.0F
#define GATEWAY_LEVEL_HARD_LIMIT_DEG       90.0F

/* 实机水平闭环参数：角度 P、电流积分 I、转速反馈阻尼 D。
 * 单位依次为 A/deg、A/(deg*s)、A/rpm；增益均需>=0。
 * KP=0.030和0.020实机均振动明显；按用户要求从0.015小幅提高到0.017，KD保持不变。 */
#define GATEWAY_LEVEL_KP_A_PER_DEG          0.017F
#define GATEWAY_LEVEL_KI_A_PER_DEG_S        0.0010F
#define GATEWAY_LEVEL_KD_A_PER_RPM          0.0010F

/* 角误差限幅、死区和积分作用区，单位 deg。先限幅再扣除死区；仅在
 * |原始误差|<=INTEGRAL_ZONE 时积分，离开该区立即清积分。 */
#define GATEWAY_LEVEL_ERROR_LIMIT_DEG        8.0F
#define GATEWAY_LEVEL_DEADBAND_DEG           0.20F
#define GATEWAY_LEVEL_INTEGRAL_ZONE_DEG      1.0F

/* 积分分量和最终电流的对称限幅，单位 A。控制电流限幅不得超过 Device
 * 产品侧电流限幅；POLARITY 只能是 +1 或 -1，用于实机确认电机方向。 */
#define GATEWAY_LEVEL_INTEGRAL_LIMIT_A       0.010F
#define GATEWAY_LEVEL_CURRENT_LIMIT_A        0.30F
#define GATEWAY_LEVEL_CURRENT_POLARITY       1.0F

#if GATEWAY_LEVEL_ZERO_RAW > UINT16_MAX
#error "Level zero raw value must fit uint16_t"
#endif

#if GATEWAY_LEVEL_CALIBRATION_KEY_MODE > 1U
#error "Level calibration key mode must be 0 or 1"
#endif

#if GATEWAY_LEVEL_CONTROL_PERIOD_MS != GATEWAY_TASK_PERIOD_MS
#error "Level controller period must match the Gateway Task period"
#endif

/* ------------------------------------------------------------------ */
/* MCU 板间 USART1 心跳                                                  */
/* ------------------------------------------------------------------ */

/* F407 为节点 1，对端 MCU 为节点 2；节点 ID 必须非零且不同。 */
#define PEER_LINK_LOCAL_NODE_ID           UINT8_C(1)
#define PEER_LINK_REMOTE_NODE_ID          UINT8_C(2)

/* 心跳发送周期与离线超时，单位 ms。超时应不小于一个心跳周期。 */
#define PEER_LINK_HEARTBEAT_PERIOD_MS     UINT32_C(500)
#define PEER_LINK_OFFLINE_TIMEOUT_MS      UINT32_C(1500)

/* 现有 10 ms Task 每周期最多解析的 USART1 字节数，范围 1..128。 */
#define PEER_LINK_RX_BYTES_PER_STEP       128U

/* 主 QD4310 遥测送往对端的周期，单位 ms；BUSY 时整帧丢弃。 */
#define PEER_LINK_TELEMETRY_PERIOD_MS     UINT32_C(20)

#endif
