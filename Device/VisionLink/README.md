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
