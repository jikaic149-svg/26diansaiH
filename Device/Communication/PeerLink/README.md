# MCU PeerLink

`PeerLink` 实现 F407 与 G3519 之间的 UART 心跳、主 QD4310 遥测、绝对编码器原始值
和一次性主电机 Disable 请求。两端均为 115200 8N1，
使用 BSP 静态收发环和 DMA；DMA/ISR 只搬运字节，现有 10 ms Task 调用
`Device_PeerLink_Step()` 完成 Packet 解析、500 ms 心跳和 1500 ms 离线判定；F407
另以 `TEL` 每 20 ms 提交 ID/使能/电流/速度/单圈角度/在线，并以独立 `ENC` 提交
QD4310 CAN data[6..7] 的原始 `uint16_t` 绝对值。G3519 分别保存两个 Latest Value。

G3519 端固定为节点 2，UART1 使用 PB4 TX、PB5 RX，RX/TX 分别使用 DMA_CH0、
DMA_CH1。对端 F407 固定为节点 1，使用 USART1 PA9 TX、PA10 RX。发送队列满时
整帧丢弃，不阻塞、不立即重试，也不增加对应 TYPE 的独立序号。

Agent A 侧新增独立 `DIS`：Payload 固定为来源节点 `2`，仅允许 G3519 在用户按键进入
停车状态时调用 `Device_PeerLink_SendMotorDisableRequest()`。它请求 F407 Disable
主 QD4310，使用独立 16 位序号；队列满时整帧丢弃，没有 ACK 或自动重试。F407
通过 `Device_PeerLink_TakeMotorDisableRequest()` 消费一次已校验事件。当前 G3519
发送端已经实现；F407 Task 的实际消费和 QD4310 Disable 仍由 Agent B 接入，完成前
不能把按键到无刷电机的动作写成双端实测通过。

完整线上格式、接线、错误处理和联调验收以 F407 工程根目录的
`MCU_UART_HEARTBEAT_PROTOCOL.md` 为唯一冻结协议契约；Agent B 接入 `DIS` 时必须同步
更新该契约。

`ENC` Payload 固定为 `<node_id>,<motor_id>,<angle_raw>,<online>`，仅接受 F407 节点 1
发往 G3519 节点 2。`angle_raw` 范围 `0..65535`、无单位；它只是 QD4310 CAN 协议
原始字段，不代表物理编码器分辨率。G3519 保存接收时间和独立序号/诊断，调用
`Device_PeerLink_GetRemoteAbsoluteEncoder()` 读取；VOFA 只在数据不超过 200 ms 且
`online=1` 时显示，否则 CH20 置零。

## 实机状态

2026-07-29 最新已烧录基线为 `HB+TEL`。同步启动
后连续运行约 62.5 s：G3519 与 F407 的心跳发送/接收均为 `126/126`，F407 发送
`3142` 帧 `TEL`，G3519 接收 `3142` 帧，双方在线。两端 Task/PeerLink 故障、发送
丢弃、Packet/CRC/节点/重复序号、UART 溢出/帧错误和 DMA 错误均为 0。蓝牙旧版
20 路稳态实收 4.006 s、198/198 帧格式正确、49.42 Hz。新增 `ENC`/CH20 尚未烧录或
实物验证；故意断线的 1500 ms 边界和两个 200 ms 置零边界也尚待验证。
