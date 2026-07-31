# F407 与 G3519 板间 UART 协议

当前协议交换双向心跳、G3519→F407车体状态和F407→G3519只读调试遥测。F407不再发送
旧`TEL/ENC`，旧`ACC/ACC_MEAS`也不再发送或解析。视觉继续使用独立USART2。

## 物理链路

- 115200 8N1，TX/RX交叉并共地。
- F407：USART1 PA9 TX / PA10 RX，DMA2 Stream7/Stream2 Channel4，节点1。
- G3519：UART1 PB4 TX / PB5 RX，DMA_CH1/DMA_CH0，节点2。
- 两端静态缓冲、非阻塞发送；BUSY整帧丢弃，不立即重试、不推进该 TYPE 序号。

## Packet V1

```text
@1,TYPE,SEQ,LEN,PAYLOAD,CRC16\n
```

沿用现有 `Algorithm_Packet` 的规范十进制、长度和 CRC16 校验。未知 TYPE 忽略。

## HB

```text
HB=<node_id>,<uptime_ms>
```

双方每500 ms发送，1500 ms无合法心跳判定离线。两端各用自己的本地无符号毫秒时钟；
uptime 非回绕倒退表示对端重启，接收端清除旧 CAR 会话。

## CAR

```text
CAR=2,<sample_ms>,<speed_x_mm_s>,<specific_force_x_mm_s2>,
      <specific_force_z_mm_s2>,<pitch_rate_centi_dps>,<valid>,<competition_mode>
```

- G3519以100 Hz发送；F407只接收节点2。
- `sample_ms` 与 G3519 HB 时钟同源，只用于排序，不能与 F407 本地时钟相减。
- `speed_x_mm_s`：左右轮平均实测车速，车头方向为正，范围 `-5000..5000`。
- `specific_force_x/z_mm_s2`：ICM45686 X/Z 加速度计比力，范围 `-50000..50000`。
- `pitch_rate_centi_dps`：零偏校正后的 Pitch/Y 角速度，单位 `0.01 deg/s`，范围
  `-100000..100000`。
- `valid` 只能为0或1；为0时四个物理量必须全0。
- `competition_mode`独立于valid，只接受G3519当前赛题号`2..6`。F407只保存并转发该
  信息，不用它启动、停止或切换滚球控制。
- CAR使用独立16位序号。重复/旧序号或非递增样本时间不覆盖Latest Value；序号和
  `sample_ms`均支持自然回绕。
- F407以本地接收时间判新鲜度：年龄不超过50 ms有效，51 ms失效。

当前CAR轮速不参与补偿；X/Z比力和Pitch角速度在F407经去重力和20 ms低通，只在KEY先回
`raw=54379`并稳定300 ms后进入ICM补偿。当前增益为0.50，底盘融合和球Kalman仍关闭。
G3519的JY61P、灰度、里程和VOFA不跨板；F407的QD4310/CAN控制仍只在F407本地执行。

G3519 不再发送停车事件；F407 的 QD4310失能和故障保护由本地逻辑负责。
若后续要让DIS实际失能，必须另行冻结可靠性与消费顺序并补测试。

## DBG

```text
DBG=1,<a_icm_mm_s2>,<theta_imu_mdeg>,<theta_cmd_mdeg>,
      <target_raw>,<feedback_raw>,<command_current_ma>,<feedback_current_ma>,
      <speed_centi_rpm>,<phase>,<status_flags>
```

- F407以50 Hz发送，采样点位于本周期`APP_Gateway_Step()`之后；只读，不参与控制。
- `theta_imu_mdeg`是ICM补偿角，`theta_cmd_mdeg`是加trim后的最终杆角命令；1 mdeg=0.001°。
- 电流单位mA，速度单位0.01 rpm，raw为QD4310单圈编码器count。
- `phase`：0=STOPPED，1=ZEROING，2=ACCELERATION_COMPENSATION。
- `status_flags`：bit0 motor_online、bit1 motor_enabled、bit2 acceleration_valid、
  bit3 rod_saturated、bit4 raw_blocked、bit5 ball_measurement_valid、bit6 peer_fault、
  bit7 vision_fault、bit8 task_fault；其余位必须为0。
- DBG使用独立16位序号。BUSY整帧丢弃并保持序号，无ACK、无重发；最坏payload为61 B，
  仍在72 B静态缓冲和Packet V1的128 B整帧上限内。
- F407发送API成功只表示进入F407 TX队列。G3519现已完成`DBG`来源、字段、序号和
  100 ms新鲜度校验，并将十个值转发到VOFA CH22～CH31。G3519在CH31额外使用bit9
  表示DBG新鲜（该位不在板间payload中）；101 ms起CH22～CH31全0。端到端仍须用
  接收/丢弃/解析计数和实际32列串口抓取验收，不能把发送计数冒充显示成功。

## 验证边界

源码和构建通过不等于硬件链路已经验证。烧录双方后仍需抓取100 Hz CAR和50 Hz DBG，
确认COM5每行严格32列，验证50/51 ms CAR边界、100/101 ms DBG边界、BUSY丢帧、单端
重启、链路断开、ICM轴符号，以及F407本地QD4310掉线保护。
