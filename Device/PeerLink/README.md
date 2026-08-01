# PeerLink

PeerLink 由现有 1 ms Gateway Task 单一拥有，通过 USART1 与 G3519 交换 Packet V1。
当前处理双向 `HB`、G3519→F407 的九/十一字段 `CAR` Latest Value，以及F407→G3519的
50 Hz只读`DBG`；F407不再发送QD4310/CAN `TEL/ENC`，也不接收旧`ACC/ACC_MEAS`：

```text
CAR=<node_id>,<sample_ms>,<speed_x_mm_s>,<specific_force_x_mm_s2>,
    <specific_force_z_mm_s2>,<pitch_rate_centi_dps>,<valid>,<competition_mode>,
    <task_active>
CAR=<上述九字段>,<planned_acceleration_x_mm_s2>,<plan_valid>
```

F407 只接受节点 2、规范十进制字段、车速 `±5000 mm/s`、X/Z 比力
`±50000 mm/s²`、Pitch/Y 角速度 `±100000`（单位 `0.01 deg/s`）和合法 valid 组合。
`valid=0` 时四个物理量必须全为0。X/Z 是加速度计比力；静止水平且轴符号正确时 Z
约为 `+9807 mm/s²`，不是去重力线加速度。
`competition_mode`独立于valid且只接受G3519赛题号`2..6`；F407按本地接收年龄保存并
转发给Maix，不据此启动、停止或切换滚球控制；mode 3仅用于禁止底盘加速度前馈。
`task_active`独立于物理valid；F407只把mode 3的活动状态映射到VST bit14，不调用
QD4310 Enable/Disable。十一字段尾部规划加速度是车头方向为正的纵向线加速度参考，范围
`±5000 mm/s²`；`plan_valid=0`时该值必须为0。旧八/十字段继续兼容并视task inactive。

独立 Header 序号与远端 `sample_ms` 都按前半区规则前进；重复、倒序或旧样本不刷新本地
接收时刻。对端 HB 离线恢复或 uptime 非回绕倒退时清序号/样本会话，使 G3519 独立重启
后可从0恢复。getter 使用 F407 本地 `received_at_ms`；50 ms 仍有效，51 ms 失效。

只有心跳在线、CAR年龄不超过50 ms、`competition_mode=4/5/6`且`plan_valid=1`时，十一字段
规划量才作为`[v,a,b_imu]`融合器的预测输入；CAR平均轮速与本地BMI088处理加速度按各自
独立样本身份校正。G3519实测加速度始终不进入前馈；M3禁用全部底盘加速度前馈，
M2、旧格式或规划过期时复位融合器并继续使用F407本地BMI088。

```text
DBG=<node_id>,<visual_position_0p1mm>,<estimated_position_0p1mm>,
    <target_command_0p1mm>,<active_target_0p1mm>,<velocity_0p1mm_s>,
    <measurement_age_ms>,<theta_cmd_mdeg>,<target_raw>,<feedback_raw>,
    <command_current_ma>,<feedback_current_ma>,<speed_centi_rpm>,
    <phase>,<status_flags>
```

`DBG`每20 ms在`APP_Gateway_Step()`之后尝试一次，使用独立16位序号。`phase`为
0=STOPPED、1=ZEROING、2=ACCELERATION_COMPENSATION；`status_flags`从bit0依次表示
motor_online、motor_enabled、ball_hold_active、rod_saturated、raw_blocked、
ball_measurement_valid、peer_fault、vision_fault、task_fault。数值均为ASCII定点单位，
不发送原始C结构体。UART BUSY时整帧丢弃、返回非故障且不推进序号；无ACK、无重发。
该帧只读，不作为任何控制输入。F407发送成功只表示整帧进入本地TX队列，不证明G3519
已经接收、转发或显示。当前G3519接收端已实现来源、范围、序号和100 ms新鲜度校验，
并转发为15列位置环专用VOFA；仍须以双端计数和实际串口抓取完成硬件验收。

G3519→F407新增Packet V1控制类型：`BTG=2,<target_0p1mm>`（协议域±1250，独立
16位序号、单槽Latest Value）和`BST=2`（独立序号、单槽停止事件）。Task在PeerLink
Step后消费，BST优先并丢弃同周期BTG，执行原有`0 A -> Disable`；BTG只在phase 2且
上一周期视觉测量有效时SetTarget，不隐式启动。产品目标安全域为±1200（0.1 mm）。

## 13 字段轮速加速度扩展

当前完整 CAR 为“原 11 字段 + `filtered_wheel_acceleration_x_mm_s2,wheel_accel_valid`”。解析器兼容 8/9/10/11/13 字段；13 字段中的轮速加速度范围为 `±5000 mm/s²`，无效时值必须为 0，且有效轮速加速度必须伴随有效规划。Gateway 按 CAR 序号和远端采样时间独立去重，并把它作为不含 IMU 零偏的加速度观测送入融合器。
