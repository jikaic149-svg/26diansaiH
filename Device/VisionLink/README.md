# VisionLink

VisionLink由现有1 ms Gateway Task单一拥有，通过候选USART2（PA2 TX、PA3 RX，
115200 8N1）非阻塞接收MaixCAM Pro测量。当前BSP使用字节中断环形缓冲；板端连续性、
3.3 V接线和相机发送端尚未实测。

复用Packet V1，TYPE=`VIS`，Header `SEQ`为独立16位序号，Payload固定为：

```text
<sample_ms>,<proc_age_ms>,<position_mm>,<flags>
```

- `sample_ms`是相机采样时钟，仅用于同一相机会话内的真实速度差分；
- `proc_age_ms`为形成帧时已经发生的处理年龄；
- `position_mm`范围`-125..125`；
- flags仅接受YOLO真实测量、杆轴已测/保持、位置有效、标定有效且未clamp的组合。

重复/旧序号不覆盖Latest Value。100 ms无新帧后允许相机从新序号会话恢复。保守年龄为
本地接收年龄、`proc_age_ms`、相机固定流水线校准值和UART传输上界之和；达到100 ms即
不可用于控制。当前固定流水线校准仍为0 ms，47 B在115200 8N1的串行上界暂取5 ms，
所以尚不能声称端到端年龄指标通过。

## PID 调参与确认

Maix端可发送独立序号的`VTP` Latest Value：

```text
TYPE=VTP
Payload=<kp_milli_per_s2>,<ki_milli_per_s3>,<kd_milli_per_s>
```

三个字段都是`0..65535`无符号整数，分别以`0.001 1/s^2`、
`0.001 1/s^3`、`0.001 1/s`表示滚球位置PID的Kp/Ki/Kd。Packet合法只表示进入静态
单槽；Gateway还会按`core_config.h`中的产品上限检查。调参不启动电机、不修改目标，
也不改变杆角、电流和raw保护。F407对每个被Task消费的VTP发送一次`VPA`：

```text
TYPE=VPA
Payload=<request_seq>,<accepted>,<kp_milli>,<ki_milli>,<kd_milli>
```

`request_seq`回显VTP Header序号，`accepted`为0或1；后三项总是返回实际保留的增益，
因此拒绝后可确认旧值未变。VTP、VPA和VIS/VST各自使用独立16位序号。BUSY时VPA整帧
丢弃且不推进VPA序号，不做ACK重发；`vpa_tx_drop/error`保留诊断。

## 球位置目标

Maix端可发送独立序号的`VBT` Latest Value：

```text
TYPE=VBT
Payload=<target_0p1mm>
```

Payload是规范十进制有符号整数，协议域为`-1250..+1250`，单位`0.1 mm`；Gateway产品域
继续限制为`-1200..+1200`。VBT拥有独立序号、重复/旧帧抑制和100 ms会话重开，Task每个
1 ms周期最多消费一个最新值。命令只修改小球目标，可在停机或回零phase 1预设并跨启停
保留，不启动电机、不绕过视觉有效门、杆角/raw/电流保护，也不替代F407内的杆球模型和
底盘加速度前馈。协议不新增逐命令ACK；Maix用周期VST中的整数`target_mm`确认整数毫米目标。
VBT和VST目标字段都使用F407杆球物理坐标；当前Maix画面左负右正，而该物理坐标左正右负，
所以触屏层以`-1`做双向转换，协议接收层不重复应用VIS测量极性。

## VST 控制状态

F407每20 ms最多发送一帧异步Latest Value，不逐帧确认VIS：

```text
TYPE=VST
Payload=<last_rx_vis_seq>,<last_used_vis_seq>,<measurement_age_ms>,
        <estimate_mm>,<velocity_mm_s>,<target_mm>,<theta_cmd_mdeg>,
        <status_flags>,<fault_bits>
```

- `last_rx_vis_seq`是最后一个Packet/Payload合法且序号新的VIS；b0有效；
- `last_used_vis_seq`是最后真正进入球状态更新的VIS，跨暂时失效保持；b1有效；
- `measurement_age_ms`为当前可用测量的保守年龄，`65535`表示当前无可用测量；
- 估计、速度、目标单位分别为mm、mm/s、mm，杆角命令单位为mdeg；无效动态量发0；
- VST Header序号仅在整帧进入本地TX环后增加；BUSY整帧丢弃、不重试。

`status_flags`：

| 位 | 名称 | 当前语义 |
|---|---|---|
| b0 | `VIS_RX_SEEN` | 启动后至少收到过一个合法且序号新的VIS |
| b1 | `MEASUREMENT_USED_SEEN` | 至少一个VIS真正进入过球状态更新，粘滞到本机重启 |
| b2 | `LAST_VIS_USABLE` | 最后合法VIS的flags组合可用于控制 |
| b3/b4 | `ESTIMATE_VALID/FRESH` | 当前有小于100 ms的可用球反馈 |
| b5 | `TARGET_VALID` | Gateway已持有有界球目标 |
| b6/b8 | `CONTROL_ARMED/LEVEL_LOOP_ACTIVE` | 主杆环处于ZEROING或phase 2 |
| b7 | `BALL_OUTER_ACTIVE` | phase 2且当前球反馈有效 |
| b9/b10 | `MOTOR_ONLINE/ENABLED` | QD4310最新反馈状态 |
| b11 | `THETA_COMMAND_APPLIED` | 杆环活动且当前未触发raw映射阻断 |
| b12 | `VIS_LINK_ONLINE` | 100 ms内收到新且Payload合法的VIS |
| b13 | `VELOCITY_VALID` | 当前球差分速度有效 |
| b15 | `FAULT_PRESENT` | `fault_bits`的b0..b14至少一位非0 |

`fault_bits`：b0链路超时、b1当前无新鲜可用测量、b2 RX环溢出、b3 UART/Step硬件
错误、b4 Packet版本/长度/CRC错误、b5 VIS Payload错误、b6 VIS重复/旧序号、b7估计器
数值错误、b8目标拒绝、b9角度/raw映射阻断、b10杆环错误、b11电机离线、b12 VST/VPA
TX丢弃或错误、b13控制Deadline丢失、b14 `CONFIG_INVALID`。错误计数类位保持到F407
重启；链路在线、测量新鲜和电机在线等当前位每次快照重算。

`CONFIG_INVALID`只表示`Device_VisionLink_Init()`拒绝静态UART/缓冲/超时配置，运行期
`Step()`失败映射到UART错误，VST/VPA发送失败映射到TX错误，不得借用b14。初始化失败时
VisionLink不可用，通常也无法在线发出VST；该位仍保留为严格的配置诊断语义。
