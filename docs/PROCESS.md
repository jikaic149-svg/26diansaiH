# 工程变更记录

## 2026-07-29：Agent A 接收 QD4310 原始绝对值并追加 VOFA CH20

- 修改原因：用户要求读取协作文档并完成交接 `20260729-01`，把 F407 已发送的
  QD4310 绝对编码器原始值加入 VOFA 回传。
- 最终决策：G3519 PeerLink 新增独立 `ENC` 接收：Payload 固定为
  `<node_id>,<motor_id>,<angle_raw>,<online>`，只接受 F407 节点 1 发往 G3519
  节点 2。`angle_raw` 原样保存为 `uint16_t 0..65535`、无单位，并使用独立序号、
  Latest Value、接收时间和 Payload/来源/重复诊断。蓝牙保持 CH0～CH19 原顺序与单位，
  仅在末尾追加 CH20；`ENC` 超过 200 ms 或 `online=0` 时 CH20 置 0。
- 架构边界：没有修改 UART/DMA/ISR/引脚、10 ms Task 周期或 20 ms VOFA 周期；没有
  新增 Task、动态内存、ACK、重发或控制动作。保留 Agent A 已实现的 `DIS`，并与
  Agent B 的 `ENC` 增量合并。静态 FireWater 缓冲仍为 224 B。
- 涉及文件：`Device/Communication/PeerLink/`、`Task/task_bringup_chassis.*`、
  `config/core_config.h`、PeerLink host 测试、Task/PeerLink README、桌面
  `COOPERATION.md` 和本文档。
- 验证方式和结果：Auto 增量验证通过；模板架构、`git diff HEAD --check`、
  Communication host 测试和 7 个通信源 Cortex-M0+ 严格编译均通过。测试覆盖原始值
  `61047`、`65535` 边界、非法 `65536`、规范十进制、来源、重复序号和 200/201 ms
  新鲜度。G3519 Bring-up Debug/Release 完整链接通过，text/data/bss 分别为
  `50552/5956/7572 B`、`47064/5956/7564 B`，BIN 为 `56440/52952 B`；相对前一版
  静态 RAM 增加 32 B。同步复核 F407 当前 Debug/Release 构建通过，BIN 为
  `20900/17648 B`；Debug SHA-256 为
  `8D7DC1F20A563476A97AEE2FF96382D6BA1F0DA0AD31DAB23C7D3B77DEB65BDA`，G3519
  Debug SHA-256 为
  `EA097067C7D7ADB1F8137F2191B44E6AE50ACAA45B1E86BDC597339574DD8D36`。
- 已知限制：本轮未烧录；F407 `ENC` 发送端也尚未烧录，因此 21 列 FireWater、原始值
  跨零和 200 ms 置零尚未完成双端实物验证。原始 16 位字段不代表物理编码器分辨率。
- 待确认事项：双端烧录后在 COM5/VOFA 检查每行固定 21 列；转动 QD4310 验证 CH20
  在 `0..65535` 跨零、静止值稳定；断开 F407 TX 后确认 CH14～CH20 不超过 220 ms
  置零且 M0 CH0～CH13 继续更新。

## 2026-07-29：Agent A 停车 KEY 提交 F407 主电机 Disable 请求

- 修改原因：用户要求 G3519 主 KEY 进入停车状态时，不仅停车，还要 Disable 滚球侧
  主电机。
- 最终决策：保留第二次 KEY 按下时两只底盘电机原有 `Stop + SetUnlocked(false)`；
  随后由同一 10 ms Task 非阻塞提交一次独立 Packet `DIS`。Payload 固定为来源节点
  `2`，仅允许 G3519 节点 2 发往 F407 节点 1，使用独立 16 位序号。重复合法帧仍产生
  幂等 Disable 事件；发送队列满整帧丢弃并计数，不阻塞、不自动重试、无 ACK。
- 架构边界：Agent A 只实现 G3519 发送、共享 PeerLink 编解码/诊断和接收事件 API；
  没有修改 F407 滚球业务。Agent B 需在 F407 Task 的 PeerLink Step 后消费事件并调用
  主 QD4310 `0 A + Disable`。`HB`、`TEL`、`ENC`、Task 周期、DMA/ISR 和 VOFA 字段
  不变。
- 涉及文件：`Device/Communication/PeerLink/`、`Task/task_bringup_chassis.*`、
  PeerLink host 测试、Task/PeerLink README、桌面 `COOPERATION.md` 和本文档。
- 验证方式和结果：Auto 增量验证通过；模板架构和 `git diff HEAD --check`、
  Communication host 测试、7 个通信源 Cortex-M0+ 严格编译均通过。G3519 Bring-up
  Debug/Release 完整链接通过，text/data/bss 分别为 `49992/5956/7540 B`、
  `46576/5956/7532 B`，FLASH/SRAM 分别为 `55880/13488 B`、`52464/13480 B`。
- 已知限制：本轮未烧录。F407 还未消费 `DIS`，因此尚未形成 KEY 到 QD4310 Disable
  的双端实物闭环；无 ACK/自动重试意味着队列满时该次请求会被丢弃并留在诊断计数中。
- 待确认事项：Agent B 完成 `20260729-02` 交接后，先使能 QD4310，再按 G3519 停车
  KEY，确认两只底盘轮电机锁定且 `TEL.enabled` 变为 0，双方 `DIS` 错误/丢弃计数为 0。

## 2026-07-29：F407 与 M0 数据合并到 G3519 蓝牙固定 20 路 VOFA

- 修改原因：用户要求 F4 数据和 M0 数据都从蓝牙串口回传。
- 最终决策：保留 G3519 UART4 PB10/PB11 为唯一蓝牙出口；F407 每 20 ms 经现有
  USART1 DMA 链路发送独立 Packet `TEL`，Payload 为主 QD4310 的节点/电机 ID、使能、
  电流 mA、速度 0.01 rpm、单圈角度 mrad、在线。`TEL` 与 500 ms `HB` 使用独立序号；
  队列满整帧丢弃，不阻塞、不重试、不推进序号。G3519 保存最新 `TEL`，在原 M0 14 路
  FireWater 后追加 ID/使能/电流 A/速度 rpm/角度 rad/在线，蓝牙固定输出 20 路；超过
  200 ms 无新 `TEL` 时末 6 路置零。F407 的 QD4310 周期 NOP 不再依赖 ESP8266 Ready，
  因而没有 Wi-Fi 客户端时蓝牙仍可获得电机反馈。
- 架构边界：没有修改 UART/DMA/ISR/引脚，没有新增 Task、ACK、重发、状态机或动态
  内存；ISR/DMA 仍只搬运字节，双端现有 10 ms Task 解析和组帧。`HB` 在线语义不变，
  `TEL` 新鲜度只影响显示值，不触发控制或恢复动作。
- 涉及文件：双端 `Device/PeerLink`，F407 `APP/Gateway`、Task/config/协议/README，
  G3519 `Task/task_bringup_chassis.*`、config、PeerLink 测试/README 和本文档。
- 验证方式和结果：Communication host 测试和 Cortex-M0+ 7 个通信源严格编译通过；
  模板架构与 `git diff HEAD --check` 通过；G3519 Bring-up Debug/Release 完整链接，
  FLASH/SRAM 分别为 `55456/13448 B`、`52144/13440 B`。F407 Debug/Release 完整链接，
  FLASH/RAM 分别为 `18460/12248 B`、`15632/12240 B`。双端 PeerLink 源文件 SHA-256
  保持一致。D 盘 OpenOCD 烧录最新 Debug BIN，并完整校验 F407 `18460` 字节、G3519
  `55456` 字节。同步启动约 62.5 s 后双方心跳均为 `126/126`，F407 `TEL` 发送
  `3142`、G3519 接收 `3142`；双方在线，所有发送丢弃、协议、UART、DMA、Task/
  PeerLink 故障计数均为 0。
- 已知限制：调试快照中的最新 `TEL` 有效且电机 ID 为 1，但 QD4310 在线位为 0；
  20 路蓝牙实收、200 ms 置零边界和 Task WCET/栈高水位仍待硬件验证。
- 待确认事项：恢复 QD4310 CAN 反馈后在 VOFA 核对固定 20 列及末 6 列单位，再断开
  F407 TX 验证 M0 前 14 路继续更新、末 6 路在不超过 220 ms 内置零。

## 2026-07-29：G3519 与 F407 双向 UART DMA 心跳实机闭环

- 修改原因：用户要求两块 MCU 继续收发数据，并双端调试到心跳通信稳定。
- 最终决策：G3519 使用 UART1 PB4 TX/PB5 RX、DMA_CH1/CH0、节点 2；F407 使用
  USART1 PA9 TX/PA10 RX、DMA2 Stream7/Stream2 Channel4、节点 1。双方保持
  115200 8N1、500 ms `HB`、1500 ms 离线超时、Packet CRC16 协议；各自由现有
  10 ms Task 非阻塞推进，不新增 Task、重发或动态内存。
- 涉及文件：`Device/Communication/PeerLink/`、`Task/task_bringup_chassis.c`、
  G3519 UART/DMA BSP 与 SysConfig、`config/core_config.h`、相关 CMake/测试及本文档；
  F407 对端实现和唯一线上协议位于 `C:\Users\rlyoulo\Desktop\F407-Gateway`。
- 构建与烧录：当前 Bring-up Debug ELF 为 text/data/bss `48512/5956/7420`，BIN
  54400 字节；DAPLink 烧录后的全长 readback 与构建 BIN 的 SHA-256 一致。
- 双端实机验证：分别锁定 G3519 DAPLink `0D28:0204` 和 F407 HID CMSIS-DAP
  `0416:5051`，同步复位后连续运行约 46 s。G3519 发送/接收 `92/92`，F407
  `94/92`，双方在线；两端 Task/PeerLink 故障、TX 丢弃、Packet/CRC/节点/重复、
  UART 溢出/帧错误和 DMA 错误全部为 0。
- 已知限制：当前只定义 `HB`，不承载底盘、滚球、视觉或电机业务数据；尚未执行
  故意断线后的 1500 ms 精确边界和长时间压力测试。单端长时间 SWD halt 会使持续运行
  的另一端 UART 产生调试扰动，因此最终验收采用双端同步复位和同时取样。
- 待确认事项：后续需要传输业务命令时先扩展唯一协议契约并定义 Topic/幂等语义；
  现阶段心跳链路无需调整。

## 2026-07-29：G3519 SPI0 改接板载 ST7735S TFT，并新增 Display Device

- 修改原因：用户要求判断 G3519 能否直接接入 1.8 英寸 TFT，并明确不需要 BMI088，
  允许将 SPI0 分配给 TFT；随后授权烧录。
- 最终决策：两套 G3519 SysConfig 均删除 BMI088 的 SPI0/双 CS 分配，改为板载显示
  SPI0 Mode 0、MSB first、10 MHz、仅发送：PB3 SCLK、PB2 MOSI、PC9 CS（低有效）、
  PC8 DC、PB23 RESET（低有效）、PA30 背光（高有效）；SPI1 板载 ICM-45686 保持不变。
  新增 `BSP_Display` 和 TX-only SPI 能力描述，新增 `Device_ST7735S` 的约 420 ms 有界
  初始化、四向旋转、RGB565 区域写入/填充/像素和背光接口。绘图只用 64 像素临时块，
  不新增字体、UI、整屏 framebuffer、DMA、Task 或产品状态机。模块只允许 3.0–3.3 V。
- 涉及文件：G3519 两份 SysConfig、BSP resource map/SPI/Display/init/manifest、
  `Device/Display/ST7735S`、根/Device CMake 与 feature-smoke preset、Display host/ARM
  测试、验证映射和板级/参数说明。
- 验证方式和结果：TB6612、AT8236 两份 SysConfig 生成通过，并确认 SPI0/PB3/PB2及
  四个控制 GPIO 宏；ST7735S 假 BSP 主机测试和 Cortex-M0+ `-Werror` 严格编译通过；
  `tools/project.ps1 -Action Verify -Scope Auto` 通过，覆盖架构/差异检查、相关 host/ARM
  测试及 G3507、G3519 TB6612/AT8236、G3519 feature-smoke Debug/Release 完整链接。
  显式选择 MSPM0G3519 + DAPLink，烧录 G3519 TB6612 Debug 17832 字节并完整回读，
  SHA-256 与 BIN 一致
  （`9C3869841FA8D02DD5A3ED170D62E0A9AD648B32EF9390862F8299AF2B2134C2`），随后
  `reset run`。
- 已知限制：烧录验证只证明映射后的基础固件可写入并运行；当前没有 TFT 调用者，
  BSP 初始化会保持 RESET 有效且背光关闭，因此屏幕不会自动点亮。尚未用实物验证
  10 MHz 波形、颜色、GRAM 偏移、背光极性或面板初始化；未启用显示 DMA。
- 待确认事项：需要点屏时由用户明确授权最小显示测试入口，再调用
  `Device_ST7735S_Init()` 和纯色填充，上板核对红/绿/蓝、四角坐标和旋转方向。

## 2026-07-29：角度锁外环 Kp 从 3.0 提高到 4.0

- 修改原因：用户确认角度纠偏极性正确。对 2010 个 VOFA 样本分析后，锁定目标为
  `-0.87°`，扰动误差约达 ±34°，回正过程没有持续振荡；最大左右电机命令分别为
  5876/6803，未达到 8000 限幅，但多次在约 1.3°～4° 残差处轮速归零并缓慢累积
  电机命令，表明小角度比例输出克服静摩擦偏慢。
- 最终决策：遵循单变量整定，只将绝对角度外环 `Kp` 从 `3.0` 提高到 `4.0`；保持
  `Ki=0.05`、`Kd=1.0`、轮速内环、±160 mm/s 外环轮速限幅和 80% 最终命令钳位不变。
- 涉及文件：`config/core_config.h`、本记录。
- 验证方式和结果：`tools/project.ps1 -Action Verify -Scope Auto` 通过，本次按影响映射
  执行 Navigation/LineTracking 两个组件；另行完整链接 Bring-up Debug/Release，分别
  使用 FLASH 9.62%/9.12%、SRAM_BANK0 18.71%/18.70%。显式选择 MSPM0G3519 +
  DAPLink，烧录 Bring-up Debug 50416 字节；从主 Flash `0x00000000` 全长回读后
  SHA-256 与构建 BIN 一致
  （`79461DBD357C17DA60D931897EF20357EE94A7CA2F35DFCA05424E22AE455687`），随后执行
  `reset run`。
- 已知限制：Kp=4.0 的实物回正时间、残差和过冲尚未复测；六轴 yaw 漂移和机械静摩擦
  仍会影响长时间角度保持。
- 待确认事项：采集新的 ±5°、±10° 扰动数据；若仍无回摆且回正偏软，再评估 Kp=4.5，
  若出现回摆则保持 Kp 并提高 Kd。

## 2026-07-29：Bring-up 从定时轮速阶跃切换为按键保持角度锁

- 修改原因：300 mm/s 双轮速度闭环测试完成后，用户要求测试绝对角度闭环：第一次
  按 KEY 锁定当前角度并保持原地，第二次按 KEY 退出。
- 最终决策：保留同一 10 ms 静态 Bring-up Task、双轮速度内环和 80% 最终命令钳位；
  删除 3 s 定时定速行为。第一次消抖按下且 ICM-45686 姿态/陀螺 bias 有效时，记录
  当前 Mahony yaw；现有角度外环（`Kp/Ki/Kd=3/0.05/1`、轮速目标±160 mm/s）输出
  等大反向左右轮速目标，再由双轮速度环产生电机命令。锁定持续到下一次按下，退出时
  电机清零并重新锁定 Device。当前 IMU 样本无效时不沿用旧 PWM，本周期输出零且保留
  用户选择的锁定状态。未新增 Task、超时、自动退出或硬件配置。
- 遥测语义：FireWater 保持 14 列和 20 ms 周期；CH0 改为锁定 yaw 目标度数（空闲为
  0），CH1/2 仍为左右轮速，CH3/4 为电机命令，CH5 为实测 yaw，CH6..13 为灰度 ADC。
- 涉及文件：`Task/task_bringup_chassis.*`、`Task/README.md`、根 CMake、
  `config/core_config.h`、本记录。
- 验证方式和结果：`tools/project.ps1 -Action Verify -Scope Auto` 通过，共执行九个组件；
  模板架构、差异检查、相关 host 测试、Cortex-M0+ 严格编译以及 Bring-up Debug/Release
  完整链接通过。Debug 使用 FLASH 9.62%、SRAM_BANK0 18.71%；Release 使用 FLASH
  9.12%、SRAM_BANK0 18.70%。已显式选择 MSPM0G3519 + DAPLink，烧录角度锁定版
  Bring-up Debug 50416 字节；从主 Flash `0x00000000` 全长回读后，SHA-256 与构建 BIN
  一致（`44D59625A7FA0BCD1CF2510812D76B117627F025D60ACBC321FE569D11DDF0C2`），随后
  执行 `reset run`。角度纠偏方向仍待实车确认。
- 已知限制：六轴 Mahony 没有绝对 yaw 参考，长时间锁定会跟随陀螺积分漂移；机械打滑
  也不能保证位置绝对不平移。Task 的 WCET、调度抖动和栈高水位仍未上板测量。
- 待确认事项：烧录后等待约 2 s 校准完成再按 KEY，小角度手推验证纠偏符号；若方向
  正确，再采集目标 yaw、实测 yaw、轮速和电机命令，整定角度环刚度与阻尼。

## 2026-07-29：提高底盘轮速环比例增益并将 Bring-up 输出上限扩大到 80%

- 修改原因：首轮 300 mm/s VOFA 阶跃中，左右轮上升偏慢且后段命令持续达到原 55%
  上限；用户要求加大 Kp，并明确将实际 PWM 输出上限扩大到 80%。
- 最终决策：保持轮速环 Ki、Kd、前馈和目标平滑不变；低速段 Kp 从 `0.15` 提高到
  `0.25`，高速段 Kp 按相同比例从 `0.12` 提高到 `0.20`。速度控制器最大 effort
  从 55% 提高到 80%，Bring-up 最终电机命令钳位同步从 6000 提高到 8000，避免两层
  限幅不一致。当前 300 mm/s 测试因平滑目标严格不大于 300 mm/s，实际使用低速段参数。
- 涉及文件：`config/core_config.h`、本记录。
- 验证方式和结果：`tools/project.ps1 -Action Verify -Scope Auto` 通过；模板架构、
  `git diff HEAD --check`、相关 host 测试、Cortex-M0+ 严格编译，以及 bring-up
  Debug/Release 完整链接均通过。Debug 使用 FLASH 9.28%、SRAM_BANK0 18.66%；
  Release 使用 FLASH 8.77%、SRAM_BANK0 18.65%。已显式选择 DAPLink 烧录 Bring-up
  Debug 48680 字节，完整回读主 Flash 后 SHA-256 与构建 BIN 一致，并 `reset run`。
  用户回传的 3 s 阶跃中，末帧左右轮为 289/287 mm/s，峰值命令 5824/6131，无超调、
  振荡或 80% 饱和；末段仍缓慢上升，说明速度环稳定但现有前馈偏小。
- 已知限制：80% PWM 不是电流限流；MG513X 12 V 堵转电流约 3.2 A，高于 TB6612
  单桥 1.2 A 连续额定值，堵转或重载会显著增加驱动器风险。MG513X 输出轴 CPR 仍需
  实测确认。
- 待确认事项：MG513X 输出轴 CPR 和有效滚动轮周仍需实测；后续按多个速度工作点重新
  标定前馈，减少积分建立时间。

## 2026-07-29：bring-up 接入模板轮速闭环；灰度无数据定位为物理链路

- 修改原因：用户要求测试速度闭环；同时报告 VOFA 灰度通道恒 0。
- 最终决策：bring-up 运行段从固定 PWM 改为模板 `Mode_Car_SpeedController`
  双频段 PID + 前馈 + 目标平滑闭环，目标
  `TEMPLATE_BRINGUP_TARGET_SPEED_MM_PER_SECOND = 300 mm/s`（两轮同目标，参数
  沿用 `TEMPLATE_CAR_SPEED_*` 既有标定），effort 百分比 ×100 转电机命令后仍经
  ±6000 限幅；`TEMPLATE_BRINGUP_MOTOR_COMMAND` 已删除。遥测 CH0 改发平滑后
  目标速度；新增故障码 9=速度控制器。CMake bring-up 档自动启用
  `MODE_CAR`（连带 `ALGORITHM_PID`）。
- 灰度诊断：DAPLink 在线读 `g_grayscale` 统计，accepted/rejected/checksum/
  format/discarded 全为 0，故障码 0——UART3 RX 从未收到任何字节，排除波特率与
  协议原因，判定为物理链路问题（T8 供电、TX/RX 未交叉或未共地）。文档记录
  T8 在既往测试工程 115200 下工作过。
- 涉及文件：`Task/task_bringup_chassis.*`、`Task/CMakeLists.txt`、根 CMake、
  `config/core_config.h`、本记录。
- 验证方式和结果：`g3519-bringup-debug` 构建通过（FLASH 9.28%、SRAM_BANK0
  18.66%），DAPLink 烧录 48680 字节成功并复位运行；闭环阶跃响应待实车 VOFA
  确认。
- 已知限制：假设电机 i 与编码器 i 同侧配对；若实际交叉，两轮闭环会互相耦合振荡，
  现象出现时需交换电机或编码器索引。首次闭环测试需悬空驱动轮。
- 待确认事项：300 mm/s 阶跃的超调与稳态误差；灰度物理链路排查结果；右轮编码器
  取反后 CH1/CH2 是否同为正。

## 2026-07-29：bring-up 升级为 VOFA+ 回传并接入 IMU/串口灰度，撤销受控电源误用

- 修改原因：用户要求回传改 VOFA+ 分通道并增加目标速度、实际速度、PWM、yaw、
  8 路灰度；要求 PWM 输出限幅保护 TB6612；并纠正 12 V/5 V 受控电源与 TB6612
  无关（电机供电走电池，不经受控轨），撤销上一版在 bring-up 中打开受控电源的
  行为。
- 最终决策：`task_bringup_chassis` 重写为 VOFA+ 回传，经蓝牙 UART4 每 20 ms 一帧，
  14 个数值：`0` 目标速度占位（速度环未实现，恒 0）、`1/2` 左右轮速 mm/s
  （`Device_Encoder`，500 线×4×28:1，名义轮周 204.20 mm）、`3/4` 电机命令、
  `5` yaw 度（板载 ICM-45686 SPI1 + Mahony，上电静止校零偏 200 样本）、
  `6..13` 芯流 T8 八路 ADC（UART3 PC6/PC7 115200，每 20 ms 发 0x19 查询）。
  首版 JustFloat 二进制帧在用户端出现乱码/解析错误，用户要求改用模板组件：现按
  模板约定接入 `Mode_Communicate_BluetoothPC`（策略：原始字节流，上层格式由产品
  Task 决定），帧格式改为 VOFA+ FireWater 纯 ASCII 行（逗号分隔、`\n` 结尾、
  整数化 snprintf，yaw 定点两位小数），任何串口助手可直接读，便于区分波特率
  问题与协议问题。所有电机命令统一经 `bringup_command_motor()` 限幅
  `TEMPLATE_BRINGUP_MOTOR_COMMAND_LIMIT=6000`（60%）。不再初始化 PowerSwitch
  Device；故障码 6 改为 IMU、8 为灰度串口。按键固定 PWM 3 s 行为不变。
- 涉及文件：`Task/task_bringup_chassis.*`、`Task/CMakeLists.txt`、根 CMake、
  `config/core_config.h`、本记录。
- 验证方式和结果：FireWater 版 `g3519-bringup-debug` 构建通过（FLASH 8.70%、
  SRAM_BANK0 18.53%），DAPLink 烧录 45632 字节成功并 `reset run`；实收波形
  待用户 VOFA 确认。
- 已知限制：IMU 初始化失败会红灯停机（探测不到 ICM-45686 时无降级路径）；芯流
  T8 具体波特率按 115200 假设，需实物确认；yaw 为 Mahony 融合值，长时间静止后
  仍会缓慢漂移。
- 待确认事项：DAPLink 重新接入后完成烧录；VOFA 实收帧率；灰度模块响应与波特率；
  电机转向/编码器符号（沿用上一条待确认）。
- 2026-07-29 实测更新：右轮（B 路，PB6/PB7）编码器前进计数为负，已在
  `Device_Encoder` 配置将 `TEMPLATE_BRINGUP_ENCODER_B_DIRECTION` 定为 `-1`
  （A 路保持 `+1`），FireWater 版重新编译烧录（45640 字节）。电机转向未报告
  问题，视为正确。

## 2026-07-29：H 题底盘第一步 bring-up Task 完成编译并烧录

- 修改原因：H 题起步测试需要最小闭环——按键触发固定 PWM、GMR 原始计数蓝牙回传、
  RGB 状态指示，用于核对电机/编码器方向并采集首批标定数据（16.1 节步骤 1）。
- 最终决策：新增可选 `Task/task_bringup_chassis.c(.h)`，由
  `TEMPLATE_ENABLE_TASK_BRINGUP_CHASSIS` 开关和 `g3519-bringup-debug/release`
  preset 编译，自动带入 Key/Motor/RGB/PowerSwitch/Encoder/Serial Device。行为：
  上电全部 Device 初始化并打开 12 V/5 V 受控电源（已被 2026-07-29 后续记录撤销，
  受控电源与 TB6612 供电无关），绿色=待机，按 PB31 按键后两电机
  以 +2000（20%）开环运行 3 s（蓝色），每 100 ms 经蓝牙 UART4（PB10 TX/PB11 RX,
  115200）发送 `B,ms,run,cmd,cntA,cntB` 帧，结束发送 `END,dA,dB` 并回待机；任何
  Device 失败红灯并写 `g_BRINGUP_Fault`（1 RGB/2 按键/3 电机/4 编码器/5 串口/
  6 电源/7 任务）。电机型号确认为 MG513X（500 线 GMR，28:1），TB6612 驱动。
  参数集中在 `core_config.h` 的 `TEMPLATE_BRINGUP_` 前缀分区，验证清单 Firmware
  组件已覆盖该前缀和两个 bring-up preset。
- 涉及文件：`Task/task_bringup_chassis.*`、`Task/CMakeLists.txt`、根 CMake/Presets、
  `main.c`、`config/core_config.h`、`config/verification/components.json`、本记录。
- 验证方式和结果：`g3519-bringup-debug` 完整构建通过（FLASH 5.92%、SRAM_BANK0
  17.74%）；OpenOCD DAPLink 烧录 31024 字节成功并 `reset run`。
- 已知限制：实物行为（电机转向、编码器计数极性、蓝牙输出、按键、RGB）尚未回读
  确认；开环 20% 占空比要求首次测试将驱动轮悬空。
- 待确认事项：左右电机正方向与编码器符号对应关系；轮径与轮距实测；蓝牙模块波特
  率是否为 115200（模块出厂常见 9600，需要用 AT 命令核对或改配置）。

## 2026-07-28：区分芯流 T8 模拟循迹与感为数字循迹 Device

- 修改原因：用户纠正第二种循迹不是 UART，而是感为 DATA+CLK 同步串行数字量模块，
  并确认 Mode 必须保留数字循迹和模拟循迹两个独立入口。
- 最终决策：新增具体 `Device_Ganwei`，只绑定恰好八路数字通道的现有
  `BSP_LineSensor` 后端，并始终调用 `Device_LineTracking_Update()` 的逐通道阈值、
  位图和交叉口判定路径；后续对应 `Mode_Car_DigitalTracking`。芯流 T8 继续由
  `Device_XinliuT8` 解析 UART ADC，并调用 `Device_LineTracking_UpdateAnalogRaw()`
  的共同基线消除和模拟置信度路径，后续对应 `Mode_Car_AnalogTracking`。两者只复用
  通用位置估计和 PID 核心。新增独立 CMake 开关，启用感为时自动带入通用循迹 Device；
  G3519 feature-smoke Debug/Release 同时编译两个具体 Device。PB8 DAT、PB9 CLK、八个
  时钟的 BSP 读取时序、SysConfig、Task 和现有 T8 行为均未修改。
- 涉及文件：`Device/Sensor/LineTracking/Ganwei`、根/Device CMake 与 Presets、循迹
  host/ARM 测试、BSP/Device/根说明、设备参数、验证组件描述和本记录。
- 验证方式和结果：`tools/project.ps1 -Action Verify -Scope Auto` 通过；模板架构检查、
  `git diff HEAD --check`、循迹 host 测试和 7 个循迹源文件 Cortex-M0+ 严格编译通过；
  G3519 feature-smoke Debug/Release 完整构建通过并生成 `device_ganwei` 静态库。由于根
  构建输入变化，增量验证实际执行九个组件，均通过。
- 已知限制：本次未新增产品循迹 Task、未烧录，也未用逻辑分析仪验证感为 DAT/CLK
  波形、位序和极性；编译和 host 测试不能替代实物验证。
- 待确认事项：感为具体型号、供电与 DATA 电平、黑线有效极性、机械安装、模块数量
  和产品调用周期。

## 2026-07-28：修复增量验证并补齐可选组件覆盖

- 修改原因：模板需要为 AI 辅助和日常开发提供可信且省时的验证入口；原增量缓存会因
  任一核心参数或 Markdown 改动扩大编译范围，验证清单中的错误引用也可能导致漏检，
  Windows PowerShell 5.1 还存在模块编码和子脚本参数传递问题；提交前审计又发现 AI
  上下文、构建复用证据、可选源码严格编译和 clean 基线判断存在遗漏。
- 最终决策：验证清单升级为 schema 2，`core_config.h` 保持单文件，并按
  `TEMPLATE_CAR_`、`TEMPLATE_GIMBAL_`、`TEMPLATE_IMU_FUSION_` 等宏前缀只使所属
  组件缓存失效；变更中的 Markdown 会进入 AI 优先文件清单，但不参与编译缓存哈希。
  架构检查会校验清单输入、依赖、host/ARM scope、构建 preset 和核心参数前缀覆盖。
  统一工具使用 UTF-8 BOM 兼容 Windows PowerShell 5.1，以命名参数向子脚本传递
  `Scope`，保留复用 preset 名称，并要求所有构建组件同时具有 fresh、clean 和完整
  preset 证据后才宣布基线就绪。日常入口固定为
  `tools/project.ps1 -Action Verify -Scope Auto`；首次建立缓存或准备基线才运行
  `-Action Verify -Scope All -Clean`。
- 构建覆盖：新增 G3519 `feature-smoke` Debug/Release preset，同时打开当前所有可选
  Algorithm、Device 和 Mode。可选静态库没有产品 Task 引用，因此只将结果表述为组件
  ARM 构建通过，不表述为产品固件集成完成。
- 涉及文件：`AGENTS.md`、根/Mode/config 说明、`CMakePresets.json`、
  `config/verification/components.json`、ARM 编译与模板检查脚本、统一项目工具及本记录；
  未增加 Task、状态机、硬件配置或运行期行为。
- 验证方式和结果：全部 host GCC 测试通过；ARM `All` 共 62 个源文件通过，并验证
  Watchdog、Communication、IMU、Motor、Navigation、LineTracking、Gimbal、Optional
  独立 scope；
  G3519 FeatureSmoke Debug/Release 均完成 142 步干净构建。九个增量组件全量验证通过后，
  `Verify Auto` 只执行全局架构与 `git diff HEAD --check`，九个组件全部命中缓存；模板检查
  和最终差异空白检查通过。
- 已知限制：本次未烧录、未上板，也未验证传感器、电机、通信或控制参数的实物行为；
  FeatureSmoke 不证明可选组件已被某个产品 Task 调用。
- 待确认事项：无；后续新增核心参数、组件或 preset 时必须同步验证清单。

## 2026-07-28：核心整定参数集中到单一分类配置

- 修改原因：用户要求把 PID、Kalman、滤波以及常调的 Task 周期/优先级/栈、阈值、
  限幅和超时汇总到 `config/`，配置必须有注释和分类，同时明确不要按算法或模块拆得
  过细；同一要求还需固化到嵌入式工作流 skill。
- 最终决策：新增单一 `config/core_config.h`，在文件内按 Task、车辆控制、循迹、云台、
  Mahony、双 IMU 融合/Kalman 六个大类分区。现有默认配置函数直接消费这些宏，所有
  既有数值和启用状态保持不变；特别保留导航 slip gate 默认关闭。通用一维 Kalman 和
  Filter API 没有业务默认值，继续由调用方显式传参，不创建未使用的空参数表。固定
  公式、公共 API 结构容量、协议常量、寄存器值、引脚/时钟和构建开关继续留在原 owner。
- Skill 规则：`embedded-firmware-workflow` 新增“核心参数与配置”约束，要求少量配置
  文件、按 owner/领域分区、注释用途/单位/约束/影响、默认函数消费单一真值，并禁止
  未使用占位参数和破坏硬件/FreeRTOS/Device/CMake 的事实边界。
- 涉及文件：`config/core_config.h` 与说明/验证映射，Task、Car/Gimbal Mode、
  LineTracking/Mahony/IMUFusion Algorithm 的默认配置入口和私有 include 路径，以及
  对应 README、测试脚本和本记录；未新增产品 Task、状态机或保护逻辑。
- 验证方式和结果：全部 host GCC 测试、Cortex-M0+ 严格编译、模板架构检查和
  `git diff --check` 通过；启用 Car、Gimbal、LineTracking、BMI088 和 ICM45686 的
  G3519 Debug/Release 均完成 SysConfig 生成、ARM 编译和完整链接。Skill 经
  skill-creator `quick_validate.py` 校验通过。
- 已知限制：本次只验证 host、ARM 编译和完整链接，未烧录或执行实物参数整定；
  配置迁移不代表这些通用默认值已经适配任一具体机械系统。

## 2026-07-28：从通用模板删除循迹产品 Task

- 修改原因：用户确认通用 Template 不应自带 `LineTrackingReadout` 和
  `AnalogTracking` 产品任务，明确要求直接删除。
- 最终决策：删除 `Task/LineTracking` 及专用接入文档，移除两个 Product Task CMake
  选项、四组循迹 configure/build preset 和 `mspm0.ps1 -TaskProfile` 入口。模板只保留
  10 ms 静态 `Task_Control` 骨架及其全局看门狗唯一喂狗职责；同步删除仅服务于产品
  Task 的输出许可、锁定和 Mode 故障调试全局量。通用控制 Task 的名称、周期、栈深度
  和优先级偏移集中到 `config/task_config.h`。可复用的 LineTracking Algorithm、Device、
  Car Tracking Mode、参数事实和 host 测试继续保留，由具体产品自行编排。
- 涉及文件：`Task`、根 CMake/presets、`tools/mspm0.ps1`、验证组件清单、架构检查配置
  白名单、根/Task/Watchdog/XinliuT8/config 文档；历史记录不改写。
- 验证方式和结果：全部 host 测试、Cortex-M0+ 严格编译、模板架构检查和
  `git diff --check` 通过；剩余 6 个 G3507/G3519 TB6612/AT8236 Debug/Release preset
  均以 `--fresh` 配置，完成 TI SysConfig 生成、ARM 编译和完整链接。
- 已知限制：本次未烧录或执行实物验证；删除后模板不再提供 T8 查询、循迹遥测或电机
  循迹产品行为。
- 待确认事项：后续产品如需循迹，应在具体产品工程新增 Task 编排，不再把产品 Task
  回填通用模板。

## 2026-07-27：加入全局 WWDT1 健康监督链路

- 修改原因：用户要求模板提供全局硬件看门狗，并确认 APP 负责全局健康策略，后续
  合作者必须能明确看到唯一喂狗者和硬件分层约束。
- 最终决策：G3507/G3519 三份 SysConfig 均启用应用级 WWDT1，LFCLK 32.768 kHz、
  除以 8、`2^10` 计数、250 ms、0% closed window、睡眠继续计数；BSP 初始化把
  Core Halt 行为设为 STOP。`APP/System/Watchdog` 只做 required/completed 位掩码
  判定，`Task/task_app.c` 是唯一喂狗决策者，调用经 `Device/System/Watchdog` 到 BSP。
  控制周期完整返回即代表执行仍推进，Mode 安全故障不会造成无意义复位循环。
- 架构约束：APP 不依赖 MCU/Device/RTOS，Task 不绕过 Device；架构检查新增唯一
  喂狗者规则，禁止其他 Task、Mode、ISR 或 Device 旁路喂狗。完整扩展说明写入
  `docs/WATCHDOG.md`，根目录及 APP/BSP/Device/Task README 均建立入口。
- 涉及文件：`APP/System/Watchdog`、`Device/System/Watchdog`、两套 BSP 的
  SysConfig/resource map/manifest/init/watchdog backend、`Task/task_app.c`、根 CMake、
  DriverLib manifest、host/ARM 测试、架构检查和文档。
- 验证方式和结果：APP Watchdog host 单元测试、全部 host 回归、Cortex-M0+ 严格
  编译和模板架构检查通过。10 个正式 preset 均以 `--fresh` 重新生成 SysConfig 并
  完整链接通过；生成代码确认 WWDT1 为 250 ms、Window0/1 关闭、睡眠继续计数。
  AnalogTracking Debug/Release 的 text/data/bss 为 `51488/5412/7212`、
  `49168/5412/7212`。
- 已知限制：尚未上板故意停止喂狗测量 250 ms 复位和复位原因；WWDT 在超时前不能
  立即关闭电机，也不能发现仍在运行的控制逻辑错误。TB6612 STBY 复位期间的默认
  安全状态仍需确认硬件下拉，致命异常立即拉低 STBY 需要独立保护路径。
- 待确认事项：实物测试初始化到首次喂狗小于 250 ms、调试暂停不复位、停止喂狗后
  WWDT1 产生 SYSRST，以及复位期间 TB6612 输出确实失能。

## 2026-07-27：分离模拟循迹产品配置与 Task 控制流程

- 修改原因：模拟循迹 Task 原先把设备逻辑索引、机械参数、已验证方向和控制参数直接
  定义在 `task_analog_tracking.c`，产品配置与周期控制流程混在同一实现文件中。
- 最终决策：新增 Task 私有声明式配置 `task_analog_tracking_config.h`，集中保存上述产品
  绑定与参数；Task 继续负责实例化 Device 并向 Mode 提交数据，Mode 不依赖 Device/BSP。
  编码器每轮 1040 计数、轮周 150.796447 mm、左右方向及所有运行行为均保持不变。
- 验证边界：本次不改变调度超时策略，也不把 KEY 当作 HardFault、断言或栈溢出时的
  电机保护；致命异常安全停机需要独立的异常钩子或硬件失能链路。

## 2026-07-27：同步已验证的芯流 T8 模拟循迹链路

- 修改原因：用户完成当前 `test` 工程实车调试后，明确要求把现行循迹代码同步回
  `D:\TI MSPM0\TI Template`。
- 最终决策：仅增量同步循迹链路，不覆盖模板已有惯导、导航、Communicate、电机和
  功能底板改动，也不恢复已删除的视觉循迹。新增芯流 T8 `0x19` 八路 ADC UART Device、
  `Device_LineTracking_UpdateRaw/UpdateAnalogRaw`、独立 Digital/Analog Tracking Mode，
  以及互斥的 `LineTrackingReadout` 和 `AnalogTracking` 产品 Task profile。
- 模拟量算法：每帧减去八路共同最小值，不做逐通道阈值或黑白标定；有线判定和置信度
  使用八路总差值，方向使用加权差比和。T8 产品权重为
  `-1,-0.60,-0.25,0,0,0.25,0.60,1`，中心两路只确认有线，边缘权重更大。
- 控制行为：10 ms 静态 Task。`AnalogTracking` 为循迹 PID（Kp 1.5）→ yaw 角度 PID
  （Kp 10.0）→ 双轮轮速 PI 三级串级；基础速度 250 mm/s，相对 yaw 最大 ±25°，
  单轮目标限幅 ±800 mm/s。KEY 启停并以当前 yaw 为基准，RGB 指示状态，蓝牙约 25 Hz
  输出固定 14 列。来源工程已由用户确认轮速环、角度环和循迹极性。
- 硬件资源：G3519 两份 SysConfig 都增加 UART3，PC6 TX、PC7 RX、115200 8N1；
  BSP UART 逻辑索引 3，新增 `UART3_IRQHandler`。蓝牙继续使用 UART4/PB10/PB11。
- 涉及文件：`APP/Algorithm/LineTracking`、`BSP/MSPM0G3519`、
  `Device/Sensor/LineTracking`、`Mode/Car/Tracking`、`Task/LineTracking`、根/子目录
  CMake 与 presets、`tools/mspm0.ps1`、循迹测试和文档。`vofa+.csv` 未同步。
- 验证方式和结果：Template 全部 host 测试通过，包含 T8 解析、模拟相对差、中心有线
  判定、非线性权重和两个 Mode。AnalogTracking Debug/Release 完整 ARM 构建通过，
  text/data/bss 为 `51176/5412/7212`、`48864/5412/7212`；LineTrackingReadout
  Debug/Release 为 `26744/5412/6420`、`26008/5412/6412`，HEX 范围检查通过。
- 已知限制：本次只同步并构建 Template，未重新烧录 Template 产物；T8 供电/UART
  电平、机械高度和间距仍需按实物确认。
- 待确认事项：第二种八路数字量模块的品牌、协议和硬件绑定；它继续走独立 Digital Mode。

## 2026-07-27：同步 PC 与双 MCU Bluetooth Communicate Mode

- 修改原因：用户要求中央模板提供单 MCU 对电脑与双 MCU 两种蓝牙通信模式，供具体
  产品 Task 按需求选择。
- 最终决策：模板新增 `Mode/Communicate/Bluetooth`，公开 `BLUETOOTH_MODE_PC` 和
  `BLUETOOTH_MODE_DUAL_MCU`。严格沿用现有 Mode 结构：两个具体模式各有独立源文件和
  `Context/Init/Update`，公共类型集中在 `include/mode_communicate.h`。Mode 不访问
  UART、BSP、Device、FreeRTOS 或 Task，不持有协议实例，不选择或切换模式。模板只
  提供通用组件；双车专用 Task 没有复制进模板。
- 旧方向：单一 `mode_communicate.c` 加通用 `GetPolicy()` 的特殊结构已被用户否决并删除。
- 涉及文件：`Mode/Communicate`、根 CMake/Presets、`Mode/README.md`、host/ARM
  测试脚本、`tests/mode_communicate` 和本记录。同步为最小增量，模板原有未提交修改保留。
- 验证方式和结果：JSON 与 PowerShell 语法、全部 host 测试、Cortex-M0+ 严格编译和
  架构检查通过。显式启用 `TEMPLATE_ENABLE_MODE_COMMUNICATE=ON` 后，G3507
  Debug/Release 与 G3519 Debug/Release 均完整构建通过，text/data/bss 分别为
  `5208/124/3444`、`4712/124/3444`、`11832/4140/3804`、
  `11416/4140/3796`。
- 已知限制：模板没有选择通信模式的产品 Task，也没有蓝牙模块硬件映射；UART、波特率、
  帧收发和协议实例必须由派生产品提供。
- 待确认事项：无；派生产品按自身需求在 Task 初始化时请求其中一个模式。

## 2026-07-22：MSPM0G3519 功能底板硬件映射

- 修改原因：按功能底板原理图释放 PA12/PA13 给 CAN，并增加第二路舵机与受控 5 V。
- 最终决策：CANFD0 使用 PA12/PA13；舵机使用 TIMG12_CCP0 PB13 和 CCP1 PA16；受控 12 V/5 V 分别使用 PA15/PB26；蜂鸣器按用户最终确认改为 PA7 GPIO。
- 涉及文件：`BSP/MSPM0G3519/MSPM0G3519.syscfg`、`bsp_resource_map.h`、`bsp_servo.c`、`bsp_power_switch.c`、BSP 构建清单及板级文档。
- 验证方式和结果：TI SysConfig 1.27.1 生成成功；43 个显式分配引脚无重复；MSPM0G3519 Debug 完整构建成功。TIMG12 生成时钟 10 MHz、周期 200000 ticks。
- 已知限制：CAN 仅完成 PinMux、控制器和 DriverLib 配置；`BSP_CAN` backend 仍未实现，因此能力标志保持关闭。蜂鸣器仅完成 GPIO 配置，尚无公共 BSP 接口。
- 待确认事项：CAN 收发器 CRX 的 3.3 V 电平兼容性；两只舵机堵转电流与 5 V 电源容量；受控 5 V 外部开关的默认关闭电路；实物 PA7 蜂鸣器网络是否已同步改板。

## 2026-07-22：暂时停用 CAN，补齐电源与蜂鸣器接口

- 当前决策：暂不启用 CAN；移除 G3519 SysConfig 的 MCAN 实例和 `mcan` DriverLib 模块，PA12/PA13 暂不分配。
- 接口调整：受控 12 V/5 V 统一使用 `BSP_PowerSwitch_*` 和 `Device_PowerSwitch_*`；PA7 增加 `BSP_Buzzer_*` 公共控制接口。
- 验证方式和结果：G3519 Debug/Release、G3507 Debug，以及 Servo + PowerSwitch + QD4310 全功能构建通过；架构检查通过。

## 2026-07-24：同步 Vision 与单 IMU 可复用模型；撤回 Cannon 状态机

- 修改原因：用户指定将 `vision_track`、单 ICM-45686、单 BMI088 解算，以及
  `summer_jixun3` 的继电器/LCD控制同步回中央模板；随后明确否决未经批准新增继电器/
  LCD 状态机。
- 最终决策：`vision_track_mode` 进入 `Mode/Car`，保持 400 mm/s 基础速度、横向与
  航向误差合成、yaw 外环和双轮速度内环，丢线立即归零。ICM-45686 与 BMI088
  分别通过 `Device_*_UpdateIMU()` 提交统一 `Device_IMU`，复用右手轴映射、静止
  零偏标定和六轴 Mahony；任一陀螺轴达到 ±32000 count 时不提交姿态。此前擅自新增的
  `Mode_Cannon`、CMake 选项、测试和文档已全部删除；继电器/LCD 同步方式待用户明确。
- 构建与依赖：启用任一 ICM45686 或 BMI088 选项会自动带入 `Device_IMU` 与
  `Algorithm_Mahony`。没有修改 BSP、SysConfig、Task 周期、优先级或启动行为。
- 涉及文件：`Mode/Car/Tracking/vision_track_mode.c`、对应公开头文件与 CMake，
  BMI088/ICM45686 Device/CMake/README，根 CMake/README、Device/参数/决策文档，
  以及 `tests/`。
- 验证方式和结果：Vision、单 ICM/BMI 两组 host 测试通过；新增/修改源在
  Cortex-M0+ 上以 `-Wall -Wextra -Wpedantic -Werror` 严格编译通过；模板架构检查和
  JSON 解析通过。全功能 G3519 Debug/Release 分别为 text/data/bss
  `11664/4140/3804`、`11328/4140/3796`，G3507 Debug 为 `5208/124/3444`；
  ICM-only 与 BMI-only 两套 G3519 Debug 均独立完整构建通过。
- 已知限制：本次未烧录。两颗 IMU 的 SPI backend、安装轴和实物静止零偏仍需产品
  配置；视觉转向符号和增益仍需车辆实测。继电器/LCD没有同步任何新增实现。
- 待确认事项：用户明确继电器/LCD希望同步的具体文件和边界后再处理；IMU 产品接入时
  登记安装轴与采样时序，再执行静止标定和低风险视觉转向测试。

## 2026-07-24：按实际算法整理目录与名称

- 修改原因：用户要求去除模糊名称、占位算法和不需要的串口命令解析器，并拆开混合的滤波实现。
- 最终决策：`Attitude` 完整改名为 `Mahony`，`Control` 完整改名为 `PID`；删除
  `APP/Algorithm/Balance` 和不需要的串口命令解析组件；`Filter` 按低通、
  滑动平均、三点中值、Biquad、通用一维 Kalman 拆成独立头文件和源文件。
- 涉及文件：根 CMake/Presets/README、`APP/Algorithm`、`Device/Sensor/IMU`、
  `Mode/Car`、测试脚本和对应文档。
- 验证方式和结果：Vision 与单 IMU host 测试通过；Cortex-M0+ 严格编译和架构检查通过；
  G3519 Debug 启用 Car、Filter、ICM45686、BMI088 后完整构建通过，六个 Filter 源文件均进入编译。
- 已知限制：这是公共 API 改名，不保留旧 `Algorithm_Attitude`/`Algorithm_Control` 兼容层。
- 待确认事项：`LineTracking` 是否保留由用户另行决定；本次未修改其代码。

## 2026-07-24：精简 Kalman 并删除 Balance Mode

- 修改原因：用户只保留一个可由任意调用方使用的 Kalman，并明确删除 `Mode/Balance`。
- 最终决策：删除角度专用 Kalman，将标量实现改为通用一维
  `Algorithm_Filter_Kalman_*`；删除 `Mode/Balance` 及其 CMake 选项和文档入口。
- 涉及文件：`APP/Algorithm/Filter`、根 CMake/Presets/README、`Mode/Balance` 和对应文档。
- 验证方式和结果：Vision、单 IMU host 测试通过；Cortex-M0+ 严格编译、架构检查和
  JSON 解析通过；G3519 Debug 启用 Car、Filter、ICM45686、BMI088 后完整构建通过，
  编译链中只有 `algorithm_filter_kalman.c` 一份 Kalman 实现。
- 已知限制：不保留 AngleKalman/ScalarKalman 兼容 API；删除内容无法从当前未跟踪仓库恢复。
- 待确认事项：310 与 513x 电机参数由用户后续提供；参数到齐前不拆 Motor、不创建占位目录。

## 2026-07-24：同步 AT8236/TB6612 profile 与两种 GMR 电机参数

- 修改原因：用户指定参考 `C:\Users\rlyoulo\Desktop\test` 的已验证 MSPM0
  电机实现，并确认实际使用 MG310 GMR版和 MG513X GMR版，两者均为500线。
- 最终决策：保留一个 `Device_Motor` 公共接口；G3519 在编译期通过
  `MSPM0_MOTOR_DRIVER=TB6612|AT8236` 选择互斥 SysConfig 与 BSP backend。
  TB6612 保持默认和原有引脚；AT8236 使用 TIMA0 四路同步 20 kHz PWM：
  PC2/CCP0、PC4/CCP1、PB20/CCP2、PA25/CCP3，输出为正转 `PWM/0`、
  反转 `0/PWM`、滑行 `0/0`、刹车 `1/1`。没有新增状态机或保护机制。
- 用户纠正：先前建立独立 `Device_AT8236` 的方向被参考工程否决，该目录、API、
  CMake选项和测试已经删除，不再代表当前实现。
- 电机型号：新增 `Motor/MG310` 与 `Motor/MG513X` 参数目录。MG310 使用实际
  减速比20.409，并按已确认的QEI四倍频记录输出轴每圈`40818`计数；MG513X使用
  减速比28和GMR 500线原始标称。先前把MG513X直接固定为`500×28×4`的方向已被
  否决，其“500线”边沿定义、QEI倍率解释和输出轴CPR需手转一圈实测确认。
- 蓝牙串口：G3519的TB6612/AT8236两套profile均配置UART4（PB10 TX、PB11 RX、
  115200 8N1、RX DMA_CH2）。`main -> BSP_Init -> BSP_UART_Init`会在上电时完成
  初始化并启动接收DMA；G3519 Debug/Release preset均启用现有Serial Device。
  没有新增蓝牙状态机、Task或回传协议。
- 涉及文件：根 CMake/Presets/README、`tools/mspm0.ps1`、G3519 SysConfig、
  Motor/PWM BSP与资源映射、Motor型号目录、Motor真值表/型号测试及硬件文档。
- 验证方式和结果：TB6612与AT8236两套host真值表、型号参数测试、ARM严格编译、
  JSON/PowerShell语法和模板架构检查通过；TI SysConfig 1.27.1 对两套profile均
  生成成功。G3519 Debug/Release完整构建通过：TB6612 text/data/bss为
  `11832/4140/3804`、`11416/4140/3796`，AT8236为`11760/4156/3796`、
  `11400/4156/3788`；四套构建的HEX范围检查均通过；G3507 Debug回归构建通过。
- 已知限制：本次未烧录，未对四路PWM波形、电机方向、电流和温升做实物验证。
- 待确认事项：编码器接口电平、所在轴、计数正方向，以及MG310/MG513X分别搭配
  哪一种驱动profile。MG513X输出轴CPR仍需手转一圈实测确认。

## 2026-07-26：同步两套 SPI IMU、棱科 M0 与 MG513X 轮径

- 修改原因：用户要求模板收录后车 BMI088 SPI、前车 ICM45686 SPI、棱科 M0，以及
  当前 MG513X 的已确认轮胎尺寸。
- 最终决策：BMI088 增加复用逻辑 BSP SPI handle 的双 CS 适配；ICM45686 增加单设备
  SPI 适配；棱科 M0 作为可选 UART IMU Device 加入根 CMake/Presets。三者保持来源
  工程行为，不新增 Task 或状态机。MG513X 记录轮径 `65 mm`、名义轮周 `204.20 mm`；
  只保留500线和28减速比，不写死未确认的输出轴CPR。
- 硬件资源：G3519 BMI088 为 SPI0 10 MHz Mode 0、PC0/PC1 双软件 CS；ICM45686 为
  SPI1 10 MHz Mode 0、PB12 软件 CS，INT1/INT2仍未启用。棱科 M0 的 UART 由产品绑定。
- 涉及文件：根 CMake/Presets/README、`Device/Sensor/IMU` 三个设备目录、MG513X
  参数目录、设备参数/决策文档和主机测试。
- 验证方式和结果：各驱动核心源码与对应来源工程 SHA-256 一致；主机测试全部通过。
  G3519 AT8236 Debug 同时启用 BMI088、ICM45686、棱科 M0 后，使用 `cmake --fresh`、
  `--clean-first` 完整构建成功，三套 Device 及两套 SPI 适配均进入编译清单。
- 已知限制：本次未烧录，未读取实物 Chip ID、检查 SPI/UART 波形或确认安装轴；
  `204.20 mm` 是由直径计算的名义轮周，有效滚动轮周仍需实测。
- 待确认事项：MG513X输出轴CPR、两颗IMU安装轴和棱科M0产品UART绑定。

## 2026-07-26：产品控制 Mode 不进入通用模板

- 修改原因：用户确认双车跟随、视觉循迹和融合导航仍需继续优化，不能作为稳定通用
  Mode 写入模板。
- 最终决策：先前把视觉循迹 Mode 同步回模板的方向已被否决。模板删除该 Mode 的源码、
  公开头文件、CMake入口、专用 host/ARM 测试和现行文档入口；双车逻辑与融合导航原本
  就未进入模板。通用 `Mode/Car` 角度环、轮速环和基础循迹 Mode 保留。前后车产品代码
  与 Task 行为均不修改。
- 涉及文件：`Mode/Car`、`tests/run_gcc_tests.ps1`、`tests/run_arm_compile.ps1`、
  `docs/PROJECT_DECISIONS.md` 和本记录。
- 验证方式和结果：排除历史记录后，模板源码、测试及全新构建目录中无三类产品逻辑
  标识或文件。删后 host 测试和 Cortex-M0+ ARM 严格编译通过；G3519 AT8236 Debug
  同时启用通用 Car、BMI088、ICM45686、棱科 M0，使用 `cmake --fresh`、
  `--clean-first` 完整构建成功，编译清单只含通用 Car Mode。
- 已知限制：未烧录；本次只验证模板构建边界，不代表三套产品 Mode 已完成优化。
- 待确认事项：三套产品 Mode 分别在对应车辆工程内优化和验证，稳定前不再同步模板。

## 2026-07-27：仅将通用融合惯导 Mode 同步回模板

- 修改原因：用户明确要求只把当前 `test` 中的惯导融合能力同步回模板，其他产品功能
  继续在当前工程测试。
- 最终决策：此前“融合导航暂不进入模板”的方向已被本次用户指令覆盖。模板仅新增
  `Mode_Car_FusedNavigation`：消费标准左右轮距离/速度和已校正机体 Z 轴角速度，完成
  局部 `x/y/yaw` 融合、路径前视曲率、yaw-rate 与双轮速度串级计算；不绑定具体 IMU、
  编码器、UART、RTOS 或 MCU。模拟循迹、13 PPR 轮速参数、蓝牙、RGB 和产品 Task 均未同步。
- 涉及文件：`Mode/Car/Navigation/`、`Mode/Car/include/mode_car_fused_navigation.h`、
  `Mode/Car/CMakeLists.txt`、`Mode/Car/README.md`、对应主机单元测试和本记录。
- 验证方式和结果：模板源码通过现有 FusedNavigation 主机单元测试，并以
  `-Wall -Wextra -Wpedantic -Werror` 编译；G3519/TB6612 Debug、Release 均使用
  `cmake --fresh`、`--clean-first` 完整构建成功，编译清单包含 FusedNavigation 源码。
- 已知限制：未新增或启用产品 Task，未烧录，也未验证 ICM45686、轮编码器方向和实车轨迹。
- 待确认事项：其余车辆功能继续在 `C:\Users\rlyoulo\Desktop\test` 实测，用户确认后再
  决定是否同步模板。

## 2026-07-27：加入通用双 IMU 融合算法

- 修改原因：用户要求将当前工程的 `APP/Algorithm/IMUFusion` 加入通用模板。
- 最终决策：模板原样加入纯算法 `Algorithm_IMUFusion` 及公共 API；输入为已换算单位的
  两颗 IMU 加速度、角速度、有效标志和采样周期，算法负责机体系轴映射、逆方差与预测
  置信度融合、杆臂补偿、可选启动/运行时陀螺零偏处理、六状态协方差和重力方向修正。
  BMI088 与 ICM45686 同时启用时自动构建该算法及 Mahony 类型依赖。未新增产品 Task、
  Mode 或默认 preset，现有上电行为不变。
- 涉及文件：`APP/Algorithm/IMUFusion/`、根目录和 Algorithm CMake/README、IMUFusion
  主机测试、ARM 严格编译入口及本记录。
- 验证方式和结果：算法与测试源码的规范化内容和来源工程一致；全部主机测试、
  Cortex-M0+ 严格编译和模板架构检查通过。独立目录中的 G3519 Debug/Release 完整构建
  均通过，并确认 `algorithm_imu_fusion.c`、`device_dual_imu.c`、BMI088/ICM45686 SPI
  适配四个编译单元全部进入构建；尺寸分别为 text/data/bss `12088/5412/3852` 和
  `11600/5412/3844`。Cortex-M0+ O2 的静态栈报告中 `Algorithm_IMUFusion_Init` 为
  `184 B`、`Algorithm_IMUFusion_Update` 为 `1392 B`。
- 已知限制：本次没有把 Device 样本送入算法，没有创建周期调用者，也未烧录或做双 IMU
  实物验证；六轴输入不能提供绝对 yaw 可观测性。后续 Task 必须为 1392 B 更新路径及其
  调用链预留栈余量，并上板检查高水位。
- 待确认事项：产品调用方仍需配置两颗 IMU 的安装轴、相对位置、采样周期、有效状态及
  可选轮速静止判据，再进行实车参数验证。

## 2026-07-27：同步蓝牙透传策略与 LoRa 半双工通信组件

- 修改原因：用户要求把当前 `test` 中已验证的 LoRa 和蓝牙无线透传能力同步回中央模板，
  并确认 LoRa 不应绑定某一颗 MCU。
- 最终决策：模板既有 `Device_Serial` 与来源工程逐字一致，继续作为普通蓝牙无线透传的
  非阻塞 UART Device，不新增无协议意义的 Bluetooth Device。Bluetooth PC/双 MCU Mode
  输出新增 `duplex_policy=UNRESTRICTED`，只表示软件不互斥 TX/RX，不宣称无线 PHY 已
  验证物理全双工。新增 `Device_LoRa`，按实测格式原子发送
  `[目标地址高][目标地址低][目标信道]+payload`，并透明接收字节流；新增纯
  `Mode_Communicate_LoRaHalfDuplex` 管理互补轮次。LoRa 选项自动带入 Device_Serial。
  没有同步双板测试 Task、产品消息协议、默认 preset、ACK、超时重发或自动恢复。
- 硬件与身份边界：已登记模块地址 `0x7302/0x1007`、信道 `0x4A/0x30`、115200 8N1、
  空口参数和定点路由头。模块地址属于 LoRa 模块内部配置，不属于 MCU、开发板或
  DAPLink；换板只需让产品 `local_side` 与所接模块地址一致，并提供满足能力的 UART
  transport。G3519 PB10/PB11 是已实测映射，不是通用 Device 的固定依赖。
- 涉及文件：`Device/Communication/LoRa`、既有 Serial 说明、`Mode/Communicate`、根/
  Device CMake 与 Presets、对应 host/ARM 测试、设备参数和本记录。模板既有 Watchdog、
  BSP/SysConfig、Task 周期、产品状态机和默认启动行为未修改。
- 验证方式和结果：LoRa Device、Bluetooth/LoRa Communicate Mode 及模板全部 host 回归
  通过；Cortex-M0+ `-Wall -Wextra -Wpedantic -Werror` 严格编译和架构检查通过。显式
  打开 LoRa+Communicate 的 G3519 Debug/Release 完整构建成功，确认 Serial、LoRa 和
  三个通信 Mode 源进入编译链，Flash/SRAM 分别为 `17736/9256 B` 和
  `17264/9248 B`。原 AnalogTracking Debug/Release 回归构建通过，证明既有蓝牙调用
  兼容，Flash/SRAM 分别为 `56816/12616 B` 和 `54512/12616 B`。LoRa 核心源码和
  host 测试与来源工程规范化内容一致；来源工程此前已完成双板空口连续序号/CRC 实测，
  本次模板未重复烧录。
- 已知限制：`Device_LoRa_Send()` 的 `OK` 只代表 UART 接受完整数据；Mode 没有 ACK、
  重发、超时或失步恢复，任一轮次消息丢失会停在等待。模块完整型号、最大 RF 单包、
  VCC 和 AUX/空口完成语义仍未知。
- 待确认事项：具体产品接入时定义消息边界、最大 payload、两端初始轮次和所需可靠性；
  模块换接板卡后按实际模块地址更新 `local_side`，不要按 MCU 或探针串号猜测。

## 2026-07-29：部署双 MCU `ENC`/21 路 VOFA 联调固件

- 修改原因：用户要求烧录已经完成构建验证的双端 Debug 固件，使 F407 的 QD4310
  绝对编码器原始值能经板间串口进入 G3519，并追加到蓝牙 VOFA CH20。
- 最终决策：F407 继续使用独立 `ENC=<node_id>,<motor_id>,<angle_raw>,<online>`；
  G3519 在不改变 CH0～CH19 的前提下固定输出 21 列，新鲜且在线时 CH20 输出
  `angle_raw`，超过 200 ms 或离线时输出 0。G3519 停车键边沿发送的 `DIS` 仍只是
  发送端实现；F407 尚未消费该命令，不能视为滚球电机已随小车停车键 Disable。
- 烧录与回读：显式选择 VID:PID `0416:5051` 的 F407 HID CMSIS-DAP，以及序列号
  `000100011a5a5bb60000000000045c0aa5a5a5a597969908` 的 G3519 DAPLink。分别以 BIN
  写入 F407 Debug `20900 B` 和 G3519 Bring-up Debug `56440 B`，按精确长度完整回读；
  SHA-256 分别与源固件一致：`8D7DC1F20A563476A97AEE2FF96382D6BA1F0DA0AD31DAB23C7D3B77DEB65BDA`
  和 `EA097067C7D7ADB1F8137F2191B44E6AE50ACAA45B1E86BDC597339574DD8D36`。
  两端随后均已复位运行。
- 验证边界：本轮证明的是目标识别、Flash 写入、完整回读和复位运行；VOFA+ 进程与
  COM5 仍存在，但没有重新独占串口采集，因此不能宣称当前 21 列帧率、CH20 动态值、
  跨零行为或 200 ms 置零已经实物验证。
- 待确认事项：恢复 QD4310 在线反馈后在 VOFA 核对固定 21 列和 CH20；断开 F407 TX
  验证置零边界。由滚球 Agent 消费 `DIS` 后，再验证小车停车键同时锁定底盘和 Disable
  QD4310。

## 2026-07-29：G3519 Bring-up 启用 ST7735S 红屏自检

- 修改原因：用户纠正 TFT 应由 MSPM0G3519 驱动，不是 F407；现有 G3519 显示 BSP、
  SPI0 和 ST7735S Device 虽已具备，但 bring-up profile 与 Task 没有启用或调用它，
  所以屏幕保持黑色。
- 最终决策：F407 TFT 接入方向已撤回。`g3519-bringup-debug/release` 自动启用并链接
  `device_st7735s`；`Task_BringupChassis` 在调度器启动后执行一次约 420 ms 面板初始化
  和约 40 ms 全屏红色填充。该 Task 优先级为 `tskIDLE_PRIORITY+1`，初始化期间
  `tskIDLE_PRIORITY+2` 的 10 ms `Task_Control` 可继续抢占并喂 250 ms WWDT1，避免把
  初始化放在调度器启动前造成持续复位。显示错误单独写
  `g_BRINGUP_DisplayFault=1..4`，不改变底盘主故障，也不停止蓝牙或 PeerLink；不新增
  Task、重试、状态机或周期界面刷新。
- 硬件映射：沿用当前 G3519 SysConfig 真值，SPI0 Mode 0/MSB first/10 MHz，SCL PB3、
  MOSI PB2、CS PC9、DC PC8、低有效 RESET PB23、高有效背光 PA30；面板 VCC 为 3.3 V。
  SPI1/PB12 CS 继续专用于 ICM-45686，BMI088 未启用。
- 涉及文件：根/Task CMake、`Task/task_bringup_chassis.*`、Task README、验证组件清单
  和本记录；未修改 SysConfig 或 BSP 引脚。
- 验证方式和结果：`tools/project.ps1 -Action Verify -Scope Auto` 通过，包含 ST7735S
  host 测试、ARM 严格编译、模板架构检查、`git diff HEAD --check` 以及 G3519
  Bring-up Debug/Release 完整链接。Debug/Release Flash 分别为 `58344/54544 B`，
  SRAM_BANK0 分别为 `13552/13544 B`；两个链接均确认编译并链接
  `device_st7735s.c` 与 `task_bringup_chassis.c`。
- 烧录与回读：显式选择序列号
  `000100011a5a5bb60000000000045c0aa5a5a5a597969908` 的 G3519 DAPLink，写入
  Bring-up Debug `58344 B` 并按主 Flash 完整回读；源 BIN 与回读 SHA-256 均为
  `D6A4BDF3EBF0B69986398AF3F70DE119A394D01D050B49349268F2CBFFE2735D`。目标随后
  已执行 `reset run`；F407 探针未被选择。
- 已知限制：写入与回读一致不能替代屏幕实物观察；约 460 ms 启动段的实际时长、
  任务抖动和栈高水位尚未上板测量。当前显示只做一次红屏，不显示运行数据。
- 待确认事项：观察红屏、RGB、蓝牙和双 MCU 通信是否同时正常；若
  `g_BRINGUP_DisplayFault` 非零，按对应阶段检查 SPI/控制线和 3.3 V 供电。

## 2026-07-29：G3519 TFT 增加低开销运行状态页

- 修改原因：用户确认红屏自检成功，并要求 TFT 显示 QD4310、双 MCU 通信和发车
  状态，同时明确保持低开销。
- 最终决策：状态页归 `Task_BringupChassis` 产品编排所有，不给通用 ST7735S Device
  增加字体或产品字段。页面固定 10 行：标题、`CAR` 发车/停车、PeerLink 在线和本地
  故障、QD4310 在线、ID/使能、电流 A、转速 rpm、单圈角度 rad、绝对编码器原始值、
  心跳接收数和底盘主故障。Task 私有 5×7 大写字模按 8×16 字符格渲染，使用一个
  4 KiB 静态行缓冲，无全屏 framebuffer、动态内存或新增 Task。
- 刷新策略：现有低优先级 Bring-up Task 每 20 ms 最多检查一行，约 200 ms 扫描全页；
  每行缓存上次文字和颜色，内容不变时不发 SPI。显示调度独立于底盘 Device 是否全部
  初始化成功，所以主故障时仍能显示状态；单行写失败记录独立显示故障码 5，不停止
  控制、蓝牙或 PeerLink。
- 涉及文件：`Task/task_bringup_chassis.*`、`Task/README.md` 和本记录；未修改 Device
  公共 API、BSP、SysConfig、Task 周期或优先级。
- 验证方式和结果：`tools/project.ps1 -Action Verify -Scope Auto` 通过模板架构检查、
  `git diff HEAD --check` 和 Firmware 组件 Debug/Release 完整链接；最终针对性重链也
  通过。Bring-up Debug/Release Flash 分别为 `60416/56272 B`，SRAM_BANK0 分别为
  `17848/17840 B`（约 27.2%）。
- 烧录与回读：显式选择序列号
  `000100011a5a5bb60000000000045c0aa5a5a5a597969908` 的 G3519 DAPLink，写入
  状态页 Debug `60416 B` 并按主 Flash 全长回读；源 BIN 与回读 SHA-256 均为
  `5B7108B8564A53E981C5D3BC653CC6C5646F6A28C1DE9DDC26A50B2F0AA8BF9B`，随后
  已执行 `reset run`。F407 探针未被选择。
- 已知限制：单行 SPI 写入的实测 WCET、Bring-up Task 抖动和栈高水位仍待硬件测量。
  字体仅覆盖当前页面需要的 ASCII 大写、数字和标点。
- 待确认事项：观察约 200 ms 页面刷新、数值可读性、PeerLink/QD4310 断开变红以及
  发车/停车颜色切换，同时确认蓝牙和控制环不受影响。
