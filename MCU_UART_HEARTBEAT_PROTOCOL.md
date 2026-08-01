# F407 与 G3519 板间 UART 协议

当前协议交换双向心跳、G3519→F407车体状态/规划加速度和F407→G3519只读调试遥测。
F407不再发送旧`TEL/ENC`，旧`ACC/ACC_MEAS`也不再发送或解析；规划量兼容扩展在现有
`CAR`尾部，避免增加100 Hz独立帧的串口占用。视觉继续使用独立USART2。

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
# 旧八字段，继续兼容
CAR=2,<sample_ms>,<speed_x_mm_s>,<specific_force_x_mm_s2>,
      <specific_force_z_mm_s2>,<pitch_rate_centi_dps>,<valid>,<competition_mode>

# 新十字段，在尾部追加规划纵向加速度及其独立有效位
CAR=2,<sample_ms>,<speed_x_mm_s>,<specific_force_x_mm_s2>,
      <specific_force_z_mm_s2>,<pitch_rate_centi_dps>,<valid>,<competition_mode>,
      <planned_acceleration_x_mm_s2>,<plan_valid>
```

- G3519以100 Hz发送；F407只接收节点2。
- `sample_ms` 与 G3519 HB 时钟同源，只用于排序，不能与 F407 本地时钟相减。
- `speed_x_mm_s`：左右轮平均实测车速，车头方向为正，范围 `-5000..5000`。
- `specific_force_x/z_mm_s2`：ICM45686 X/Z 加速度计比力，范围 `-50000..50000`。
- `pitch_rate_centi_dps`：零偏校正后的 Pitch/Y 角速度，单位 `0.01 deg/s`，范围
  `-100000..100000`。
- `valid` 只能为0或1；为0时四个物理量必须全0。
- `competition_mode`独立于valid，只接受G3519当前赛题号`2..6`。F407保存并转发该信息，
  不用它启动、停止或切换滚球控制；mode 3仅用于禁止底盘加速度前馈。
- `planned_acceleration_x_mm_s2`是G3519轨迹规划器输出的车身纵向线加速度参考，车头方向
  为正，协议范围`-5000..5000 mm/s²`。它不是ICM比力、轮速差分测量或加速度闭环误差项。
- `plan_valid`只存在于十字段格式且只能为0或1；为0时规划加速度必须为0。它与物理传感器
  `valid`相互独立，因此IMU临时无效时仍可提交有效规划，零加速度匀速也必须以`0,1`表示。
- F407同时接受严格8/9/10/11/13字段格式；过渡期14字段也兼容接收，但末尾旧段号只做
  `0..3`合法性检查并立即丢弃，不参与控制。只有心跳在线、CAR年龄不
  超过50 ms、`competition_mode=4/5/6`且`plan_valid=1`时，规划量成为主前馈输入；G3519
  实测加速度不进入控制。M3禁用全部底盘加速度前馈；M2及旧八字段使用F407本地BMI088。
- CAR使用独立16位序号。重复/旧序号或非递增样本时间不覆盖Latest Value；序号和
  `sample_ms`均支持自然回绕。
- F407以本地接收时间判新鲜度：年龄不超过50 ms有效，51 ms失效。

F407只在上述M4/M5/M6门控成立时直接使用规划量；CAR平均轮速与本地BMI088处理加速度仍作为独立
观测接到三状态融合框架。轮速不做差分，两个异步观测按各自序号只校正一次。
`GATEWAY_CHASSIS_ACCEL_FUSION_ENABLE=1`，规划失鲜时复位融合器并立即回退本地BMI088。

### M4/M5/M6规划契约

- M4/M5/M6：起步和结尾统一使用`100 mm/s²`加减速度上限与`400 mm/s³`jerk；三题都以
  G3519实际轮速规划器输出作为唯一规划前馈，F407不依据mode重新规划或修改该量。
- M4保持`320 mm/s`巡航及B后`50 mm`目标，并继续验证8 s内通过B。M5/M6保持`234 mm/s`
  巡航、`110 mm/s`终点接近速度及横杠后`180 mm`目标。
- 除起步和结尾过渡外，M5/M6直线与半圆弧均保持相同纵向速度，所有切点规划纵向加速度
  必须为0；题面单圈中心线长度约`6141.6 mm`，时间和停车位置仍须以实车VOFA确认。

G3519 不再发送停车事件；F407 的 QD4310失能和故障保护由本地逻辑负责。
若后续要让DIS实际失能，必须另行冻结可靠性与消费顺序并补测试。

## DBG

```text
DBG=1,<a_selected_mm_s2>,<theta_chassis_mdeg>,<theta_cmd_mdeg>,
      <target_raw>,<feedback_raw>,<command_current_ma>,<feedback_current_ma>,
      <speed_centi_rpm>,<phase>,<status_flags>
```

- F407以50 Hz发送，采样点位于本周期`APP_Gateway_Step()`之后；只读，不参与控制。
- `a_selected_mm_s2`是F407本周期最终采用的纵向加速度；`theta_chassis_mdeg`是对应底盘
  补偿角，`theta_cmd_mdeg`是加trim和球反馈后的最终杆角命令；1 mdeg=0.001°。
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

G3519按赛题优化VOFA时应至少保留统一的`mode/路段/用时、规划速度、实测平均轮速、速度
误差、规划加速度、IMU纵向加速度、加速度环输出、左右轮目标/实测、循迹误差`。M4重点看
A到B用时和加减速峰值；M5/M6重点看起步、终点减速及直线/圆弧切点，切点处规划纵向
加速度应连续保持0。F407现有CH22～CH31继续显示最终采用加速度、补偿角、总杆角和电机量，
用于与G3519本地规划/实测量按时间对齐。

## 验证边界

源码和构建通过不等于硬件链路已经验证。烧录双方后仍需抓取100 Hz CAR和50 Hz DBG，
确认COM5每行严格32列，验证50/51 ms CAR边界、100/101 ms DBG边界、BUSY丢帧、单端
重启、链路断开、ICM轴符号，以及F407本地QD4310掉线保护。

## 2026-08-01：滤波轮速加速度扩展（当前有效）

G3519 在完整 11 字段 CAR 尾部追加滤波轮速加速度及独立有效位：

```text
CAR=2,<sample_ms>,<speed_x_mm_s>,<specific_force_x_mm_s2>,
      <specific_force_z_mm_s2>,<pitch_rate_centi_dps>,<valid>,
      <competition_mode>,<task_active>,
      <planned_acceleration_x_mm_s2>,<plan_valid>,
      <filtered_wheel_acceleration_x_mm_s2>,<wheel_accel_valid>
```

- 新 F407 继续接受旧 8/9/10/11 字段以及新 13 字段；启用 13 字段的 G3519 必须配套本版 F407，旧 F407 解析器会拒绝 13 字段。
- 13 字段只在 M4/M5/M6 规划有效且轮速估计器完成预热时发送；否则退回 11 或 9 字段。
- 滤波轮速加速度来自左右轮平均速度的 13 点最小二乘斜率（标称约 120 ms），再经过 100 ms 一阶低通与 `±1000 mm/s²` 限幅；它不是原有单步轮速差分 CH12。
- `wheel_accel_valid=0` 时对应值必须为 0；`wheel_accel_valid=1` 必须同时满足 `plan_valid=1`。
- M4仍将该量以`H=[0,1,0]`加入`[v,a,b_imu]`融合器。M5/M6不再使用该Kalman输出，
  而按下述最终静态融合直接使用该低通轮速加速度。
- 每个 CAR 序号/采样时间的轮速和轮速加速度各只校正一次。旧协议没有该字段时不会阻断融合，也不会改变 M2/M3 门控。
- 平移补偿理论上不按 M6 球位置放大。合成杆角时球位控制先占用 `[-2.00,+0.75] deg` 权限，底盘补偿只使用剩余余量。

## 2026-08-01：M5/M6最终静态加速度融合（当前有效）

```text
w_plan = clamp(abs(a_plan) / 50, 0, 1)
a_selected = a_plan + 0.50 * (1 - w_plan) * a_wheel_filtered
```

- 起步/刹车规划达到`50 mm/s²`后由jerk限幅规划完全主导，不叠加轮速或BMI088噪声。
- 规划接近0的直线和半圆保留0.50倍低通轮速加速度补偿；最终杆角仍经过既有0.25前馈
  增益和`±0.60°`限幅，因此匀速小噪声不会抢走视觉位置环控制权。
- M5/M6不再按AB/BC/CD/DA切段，不复位融合状态，不使用BMI088/Kalman输出。M4原融合、
  M3禁用、M2本地BMI088回退以及负规划停车基准保持不变。
