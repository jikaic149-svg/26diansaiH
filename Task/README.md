# Task

模板只提供一个 10 ms 静态 FreeRTOS 控制任务骨架，用于验证调度器、移植层和全局
看门狗调用链，不包含具体产品 Task 或 Device/Mode 编排。
任务名称、周期、栈深度和优先级偏移集中定义在
[`config/core_config.h`](../config/core_config.h) 的“Task 与调度”分区。

`task_app.c` 是全局 250 ms WWDT1 的唯一运行期喂狗者，只在当前
10 ms 控制周期完整返回且 APP 健康策略满足后喂狗。禁止把喂狗移入 Tick/Idle/ISR，
也禁止其他 Task 独立喂狗；新增关键 Task 时按
[`docs/WATCHDOG.md`](../docs/WATCHDOG.md) 汇总当前周期健康状态。

`g3519-bringup-debug/release` 额外编译静态 `Task_BringupChassis`：周期和 deadline
均为 10 ms、优先级 `tskIDLE_PRIORITY+1`、栈深 512 个 `StackType_t`。第一次按下
PB31 KEY 后启用芯流 T8 八路模拟循迹：UART 每 20 ms 查询一次 0～4095 ADC，新帧先做
八路共同基线消除和模拟位置估计，再由 `Mode_Car_AnalogTracking` 围绕 250 mm/s
前进目标生成左右差速轮速，现有双轮速度控制器继续作为 10 ms 内环。通道从左到右
权重为 `-0.70,-0.45,-0.20,0,0,0.20,0.45,0.70`，该产品 Task 单独使用循迹
`Kp=0.70`；丢线、置信度低于 0.10 或样本超过
100 ms 时立即清零两个电机命令，后续收到有效线帧可自动恢复。第二次按下立即清零并
锁定两个底盘电机 Device，同时向 F407 非阻塞提交一次独立 `DIS` 主电机 Disable
请求。`DIS` 无 ACK 和自动重试，队列满会整帧丢弃并计数。

蓝牙每 20 ms 最多提交一行固定 21 列 FireWater 遥测：CH0 为归一化模拟线误差
（右偏为正，样本过期为 0），CH1/CH2 为左右轮速 mm/s，CH3/CH4 为两路电机命令，
CH5 为 yaw，CH6～CH13 为 T8 八路 ADC；CH14～CH19 为 F407 主 QD4310 的 ID、使能、
电流 A、速度 rpm、单圈角度 rad、在线，CH20 为 CAN 反馈 data[6..7] 的原始
`uint16_t` 绝对值。BUSY 时允许丢帧。当前 WCET、最大调度抖动和栈高水位尚未上板
测量；该 Task 不参与全局喂狗决策。

Bring-up Task 同时持有一个 `Device_PeerLink`：UART1 PB4/PB5 使用 DMA_CH1/CH0，
作为节点 2 每 500 ms 与 F407 节点 1 双向交换心跳，并接收 F407 每 20 ms 的独立
`TEL` 和 `ENC`；1500 ms 未收到合法心跳时离线，200 ms 未收到 `TEL` 时蓝牙
CH14～CH19 置零，`ENC` 超过 200 ms 或 `online=0` 时 CH20 置零。
链路由同一 10 ms 周期非阻塞推进，故障只记录在 `g_peer_link_fault`，不停止底盘
Device。完整协议以 F407 工程的 `MCU_UART_HEARTBEAT_PROTOCOL.md` 为准。

Bring-up 启动后还会在自身低优先级 Task 中初始化一次 G3519 的 ST7735S，并将
128×160 全屏填充为红色作为最小显示自检。初始化与填充为约 460 ms 的有界阻塞启动
操作，但发生在调度器启动后；更高优先级的 10 ms `Task_Control` 仍可抢占并喂
WWDT1。红屏自检后进入 10 行状态页：灰度线状态/误差、发车/停车、PeerLink 在线与本地故障、QD4310
在线/使能/ID、电流 A、转速 rpm、单圈角度 rad、绝对编码器原始值，以及心跳接收数和
底盘主故障。界面使用 Task 私有 5×7 字模和 4 KiB 静态行缓冲，每 20 ms 最多写一条
128×16 行，约 200 ms 检查一轮页面；文字和颜色未变化的行不会重复发送，不分配堆
内存。显示失败只写
`g_BRINGUP_DisplayFault`（1 默认配置、2 初始化、3 尺寸、4 红屏填充、5 状态行更新），
不改变 `g_BRINGUP_Fault`，也不停止底盘、蓝牙或 PeerLink。
