# PeerLink

PeerLink 由现有 1 ms Gateway Task 单一拥有，通过 USART1 与 G3519 交换 Packet V1。
当前处理双向 `HB`、G3519→F407 的七字段 `CAR` Latest Value，以及F407→G3519的
50 Hz只读`DBG`；F407不再发送QD4310/CAN `TEL/ENC`，也不接收旧`ACC/ACC_MEAS`：

```text
CAR=<node_id>,<sample_ms>,<speed_x_mm_s>,<specific_force_x_mm_s2>,
    <specific_force_z_mm_s2>,<pitch_rate_centi_dps>,<valid>
```

F407 只接受节点 2、规范十进制字段、车速 `±5000 mm/s`、X/Z 比力
`±50000 mm/s²`、Pitch/Y 角速度 `±100000`（单位 `0.01 deg/s`）和合法 valid 组合。
`valid=0` 时四个物理量必须全为0。X/Z 是加速度计比力；静止水平且轴符号正确时 Z
约为 `+9807 mm/s²`，不是去重力线加速度。

独立 Header 序号与远端 `sample_ms` 都按前半区规则前进；重复、倒序或旧样本不刷新本地
接收时刻。对端 HB 离线恢复或 uptime 非回绕倒退时清序号/样本会话，使 G3519 独立重启
后可从0恢复。getter 使用 F407 本地 `received_at_ms`；50 ms 仍有效，51 ms 失效。

当前CAR的轮速不参与补偿；X/Z比力和Pitch角速度经F407去重力、20 ms低通后，只在KEY回零
稳定并进入补偿phase时以`GATEWAY_CHASSIS_FF_GAIN=0.50`生成ICM角。底盘融合和球Kalman仍
关闭。

```text
DBG=<node_id>,<a_icm_mm_s2>,<theta_imu_mdeg>,<theta_cmd_mdeg>,
    <target_raw>,<feedback_raw>,<command_current_ma>,<feedback_current_ma>,
    <speed_centi_rpm>,<phase>,<status_flags>
```

`DBG`每20 ms在`APP_Gateway_Step()`之后尝试一次，使用独立16位序号。`phase`为
0=STOPPED、1=ZEROING、2=ACCELERATION_COMPENSATION；`status_flags`从bit0依次表示
motor_online、motor_enabled、acceleration_valid、rod_saturated、raw_blocked、
ball_measurement_valid、peer_fault、vision_fault、task_fault。数值均为ASCII定点单位，
不发送原始C结构体。UART BUSY时整帧丢弃、返回非故障且不推进序号；无ACK、无重发。
该帧只读，不作为任何控制输入。F407发送成功只表示整帧进入本地TX队列，不证明G3519
已经接收、转发或显示。当前G3519接收端已实现来源、范围、序号和100 ms新鲜度校验，
并转发到VOFA CH22～CH31；仍须以双端计数和实际32列串口抓取完成硬件验收。
