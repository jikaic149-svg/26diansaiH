# Process

## 2026-07-29：QD4310绝对编码器原始值回传接口

- 修改原因：用户要求VOFA回传电机绝对编码器原始值，用于机械水平零点标定；现有
  `TEL.angle_mrad` 的1 mrad量化不能无损还原CAN原始字段。
- 最终决策：Device从QD4310反馈data[6..7]小端保留 `uint16_t single_turn_angle_raw`，
  Gateway快照原样透传。为不破坏已验证的 `TEL` 和VOFA CH0～CH19，PeerLink新增独立
  `ENC=<node_id>,<motor_id>,<angle_raw>,<online>`；F407每20 ms最多发送一帧，使用
  独立16位序号和发送/丢弃/错误计数，UART BUSY时整帧丢弃，不阻塞、不重发、无ACK。
- 架构边界：未修改USART1/DMA/ISR、Task周期/优先级/栈、HB/TEL或控制逻辑；未新增
  Task、动态内存或自动恢复。小车侧只按桌面 `COOPERATION.md` 交接20260729-01处理，
  本轮未越界修改 `TI Template`。
- 涉及文件：`Device/QD4310/`、`APP/Gateway/app_gateway.[ch]`、
  `Device/PeerLink/device_peer_link.[ch]`、`Task/task_app.c`、协议、测试、README和本记录。
- 验证方式和结果：MSYS2 GCC 16.1以 `-Wall -Wextra -Wpedantic -Werror` 执行主机测试
  通过，覆盖CAN原始值精确保留、`ENC` Payload、0/65535、独立序号、BUSY丢弃和
  原TEL格式不变。F407
  Debug/Release完整链接零警告，FLASH/RAM分别为 `20900/12344 B`、`17648/12336 B`。
- 烧录与运行：使用D盘xPack OpenOCD和VID:PID `0416:5051`锁定F407探针，确认
  DBGMCU DEV_ID `0x413`、512 KiB Flash；最新Debug ELF（SHA-256
  `95BFCB07EE6CC3F61C0492858D9606AAFE2F0145EF89E60EBE04DDA992DE8E4E`）编程、
  校验和复位成功。约1.35 s时主程序/Task故障为0、Task心跳136、Ready=1、PeerLink
  Ready=1且故障0、滚球控制故障0。随后 `ENC` 已成功排队1699帧，丢弃/错误均为0。
- 已知限制：当前QD4310反馈序号0、`online=0`、`angle_raw=0`，因此尚不能验证原始值；
  小车Agent的软件已消费 `ENC` 并追加CH20，但G3519新版尚未烧录，实物仍输出旧20路。
- 待确认：小车Agent烧录G3519新版后，恢复QD4310 CAN反馈；先转动电机核对CH20与
  CAN data[6..7]一致，再断开F407 TX验证不超过220 ms置零。

## 2026-07-29：上电绝对零点的 QD4310 外部电流闭环

- 修改原因：用户要求用绝对编码器精确保持水管水平，每次上电以当前点为零点，并明确
  否决 QD4310 内部位置环，要求控制 `CURRENT`；初值参考旧工程参数。
- 最终决策：保留现有 10 ms 静态 Gateway Task 和 KEY1 30 ms 单次消抖。电机失能、
  反馈在线且 |转速|≤1 rpm 时只锁存一次上电绝对角；KEY 首次按下发送 `Enable + 0 A`，
  后续每帧新反馈运行绝对角外环并发送电流，第二次按下发送 `0 A + Disable`。控制器沿用
  旧 `PITCH_HOLD_*` 的 P/I/转速阻尼 D、误差/积分/电流限幅和 ±90° 硬限位；参数集中在
  `config/core_config.h`。反馈 100 ms 超时、失能反馈、硬限位、计算或 CAN 发送失败时
  清积分并尝试 `0 A + Disable`，不自动重启。主电机闭环运行时拒绝远程模式抢占，仅
  允许 NOP/Disable；Gateway 不再开放 ANGLE/STEP。此前临时 `SetAngle()` 方案已否决。
- 架构边界：纯算法位于 `APP/Algorithm/LevelCurrentControl`，无 MCU/Device/RTOS 依赖；
  Gateway 持有零点捕获、反馈序号、启停和 Device 调用；Task 只传入当前 ms 并保持原
  周期/优先级/栈。未新增 Task、动态内存、ISR、DMA、引脚、自动重试或自动恢复。
- 涉及文件：`APP/Algorithm/LevelCurrentControl/`、`APP/Gateway/app_gateway.[ch]`、
  `Task/task_app.c`、`config/core_config.h`、`CMakeLists.txt`、测试、README 和本记录。
- 验证方式和结果：arm-none-eabi-gcc 10.3 Debug/Release 完整链接零警告；FLASH/RAM
  分别为 `20560/12320 B`、`17340/12312 B`。算法与测试源均经 `-Wall -Wextra -Werror`
  目标编译；静态检索确认 APP/Task 不再调用 `Device_QD4310_SetAngle/StepAngle`。
- 后续验证补充：MSYS2 GCC 16.1主机测试已覆盖上电零点、0/360°回绕、死区、积分、
  速度阻尼、电流限幅和硬限位并通过。测试使用0.0001°浮点容差；实测换算舍入约
  0.000016°，远小于QD4310反馈量化。
- 已知限制：包含该控制器的最新F407固件已烧录并通过非运动运行检查，但未按KEY使能；
  不能把主机测试和烧录成功视为控制方向、绝对角单位、实时反馈或机械安全验证。
- 待确认：先悬空或解除负载，以低电流确认 `CURRENT_POLARITY`；再验证上电零点、
  0/360° 回绕、KEY 两次启停、100 ms 断反馈出口和 ±90° 硬限位，最后再逐级调 P/D/I。

## 2026-07-29：F407 QD4310 遥测经 M0 蓝牙统一回传

- 修改原因：用户要求 F4 数据和 M0 数据都从 G3519 蓝牙串口回传。
- 最终决策：在已通过实机验证的 USART1 DMA PeerLink 上新增独立 `TEL`，每 20 ms
  报告主 QD4310 的 ID、使能、电流 mA、速度 0.01 rpm、单圈角度 mrad、在线；`TEL`
  和 `HB` 分别维护序号，UART BUSY 时整帧丢弃且不重试。`APP_Gateway_Step()` 将当次
  主电机快照交给现有 10 ms Task，Task 定点化后提交 PeerLink。QD4310 周期 NOP 不再
  依赖 ESP8266 Ready，确保蓝牙作为唯一上位机链路时仍能刷新反馈。
- 架构边界：没有修改 USART1 DMA/ISR、CAN/KEY/ESP8266/TFT 硬件资源，没有新增 Task、
  ACK、重发、状态机或动态内存；协议解析仍在 Task，DMA/ISR 只搬运字节。
- 涉及文件：`Device/PeerLink/device_peer_link.[ch]`、`APP/Gateway/app_gateway.[ch]`、
  `Task/task_app.c`、`config/core_config.h`、协议、README 和本记录。
- 验证方式和结果：共享 PeerLink 的 Communication host 测试和 Cortex-M0+ 严格编译
  通过；F407 Debug/Release 完整链接零警告，FLASH/RAM 分别为 `18460/12248 B`、
  `15632/12240 B`；G3519 Bring-up Debug/Release 同时完整链接通过。D 盘 OpenOCD
  烧录最新 Debug BIN，并完整校验 F407 `18460` 字节、G3519 `55456` 字节。同步启动
  约 62.5 s 后双方心跳均为 `126/126`，F407 `TEL` 发送 `3142`、G3519 接收 `3142`；
  双方在线，所有发送丢弃、协议、UART、DMA、Task/PeerLink 故障计数均为 0。
- 已知限制：本轮调试快照中的 `TEL` 有效且电机 ID 为 1，但 QD4310 在线位为 0；尚未
  用蓝牙/VOFA 实收固定 20 路，也未验证 200 ms 置零边界和 Task 栈高水位。
- 待确认：恢复 QD4310 CAN 反馈后核对电流/速度/角度单位；在 VOFA 核对固定 20 列，
  再断开 F407 TX 验证末 6 路在不超过 220 ms 内置零。

## 2026-07-29：实现 F407 端 MCU 串口心跳通信

- 修改原因：用户明确要求完成 MCU 与 MCU 之间的直接通信；此前已冻结双方心跳协议。
- 最终决策：使用当前空闲的 USART1（PA9 TX / PA10 RX）、3.3 V TTL、115200 8N1；
  F407 为节点 1、对端为节点 2。双方每 500 ms 发送 Packet `HB` 帧，1500 ms 无合法
  对端心跳判定离线；发送队列满则整帧丢弃并等待下个周期，不阻塞、不立即重试。
- 架构边界：F407 复用静态 UART 环和 Packet 组件，USART1 RX/TX 分别使用 DMA2
  Stream2/Stream7 Channel4，由现有 10 ms Task 推进，
  不新增 Task；通信初始化/运行失败只记录链路故障，不会停止 KEY、QD4310 或 ESP8266。
  USART3/ESP8266、KEY1、CAN1/QD4310 和 TFT 停用状态保持不变。
- 涉及文件：`Device/PeerLink/device_peer_link.[ch]`、`Task/task_app.c`、
  `BSP/bsp_f407.c`、`config/core_config.h`、`CMakeLists.txt`、
  `MCU_UART_HEARTBEAT_PROTOCOL.md`、`README.md`、`process.md`。
- 构建验证：arm-none-eabi-gcc 10.3 Debug/Release 编译链接零警告；Debug 为
  FLASH 3.24%、RAM 9.28%，Release 为 FLASH 2.75%、RAM 9.27%。静态检查确认
  PeerLink 使用静态缓冲区、每 10 ms 最多解析 128 字节，未引入动态内存、HAL、
  阻塞式 UART、额外 Task 或 TFT 构建项。
- 缺陷修复：USART1 DMA 使用 direct mode 且未使能 FIFO error interrupt，原 ISR 却把
  FEIF2/FEIF7 当作致命错误，导致每次有效传输都累计伪 DMA 错误。错误判定现只保留
  已使能的 DMEIF/TEIF；修复后双端 DMA 错误保持 0。
- 烧录与实机验证：通过 VID:PID `0416:5051` 的 HID CMSIS-DAP 锁定 F407，D 盘
  OpenOCD 识别 DEV_ID `0x413`、512 KiB Flash，Debug ELF 编程、校验、复位成功。
  F407 与 G3519 同步复位后连续运行约 46 s：F407 发送/接收 `94/92`，G3519
  `92/92`，双方在线；Task/PeerLink 故障、TX 丢弃、Packet/CRC/节点/重复、UART
  溢出/帧错误及 DMA 错误全部为 0。
- 已知限制：当前协议只定义 `HB` 心跳，不承载视觉、小车或电机业务数据；尚未执行
  人为断线后的 1500 ms 边界计时和长时间压力测试。当前 CAN1 ESR `0x00800033` 且
  QD4310 无反馈，KEY→电机闭环需在电机/CAN 重新接通后单独回归。
- 待确认：检查 QD4310 供电、CANH/CANL、共地和终端电阻；串口心跳链路无需改动。

## 2026-07-29：停用 TFT 初始化并恢复电机按键

- 修改原因：用户明确不再需要 TFT，要求关闭初始化。
- 最终决策：ST7735S 与 DisplayTest 源码保留但退出当前 CMake 目标；BSP 删除 TFT
  资源、SPI1/DMA2 时钟、GPIO 复用和 DMA2 Stream3 强中断实现。当前启动过程不会
  访问 PA4/PA5/PA7/PB0/PB1，也不会发送 TFT 复位、命令或像素数据。
- 行为变化：KEY1 恢复此前实机验证的 30 ms 消抖和 QD4310 Enable/Disable 交替控制；
  网关 Task 仍为 10 ms，ESP8266、CAN1 和 QD4310 行为不变。
- 涉及文件：`BSP/bsp_f407.c`、`BSP/bsp_init.h`、`Task/task_app.[ch]`、
  `config/core_config.h`、`CMakeLists.txt`、`README.md`。
- 构建验证：arm-none-eabi-gcc 10.3 Debug/Release 零警告；Debug 为 FLASH 2.77%、
  RAM 8.80%，Release 为 FLASH 2.36%、RAM 8.80%。当前 ELF 不含 TFT BSP、Device、
  APP 或 DMA2 Stream3 强实现，仅保留启动文件的弱默认中断符号。
- 实机验证：D 盘 OpenOCD 对更新后的 Debug ELF 编程/校验/复位成功；运行约 25.7 s
  时 Task 心跳 2569、故障码 0、按键错误 0。RCC AHB1ENR 的 DMA2EN=0、APB2ENR
  的 SPI1EN=0，PA4/PA5/PA7/PB0/PB1 保持未配置；SPI1 CR1/CR2 与 DMA2 Stream3
  CR 均为 0，确认未发生 TFT 初始化或传输。
- 已知限制：本轮未再次物理按压 KEY1；恢复的按键路径与此前实机验证版本一致。

## 2026-07-29：ST7735S SPI 屏与 KEY1 纯色测试

- 修改原因：用户先独立测试 1.8 寸八针 SPI TFT，不接视觉和小车，并要求 KEY1 控制。
- 最终决策：用户资料确认模组为 ST7735S、128×160、4 线 SPI，八针顺序为
  GND/VCC/SCL/SDA/RES/DC/CS/BL；VCC 只接 3.3 V。使用 SPI1 Mode 0、5.25 MHz，
  PA5 SCK、PA7 MOSI、PA4 CS、PB0 DC、PB1 RES，BL 固定接 3.3 V。SPI1 TX 使用
  DMA2 Stream3 Channel3；DMA ISR 只记录终态，Task 后续周期等待 BSY 清除并结束片选。
- 架构边界：BSP 持有 SPI/DMA/IRQ/引脚；Device/ST7735S 持有资料原始初始化序列、
  X+2/Y+1 显存偏移和 RGB565 分段刷屏；APP/DisplayTest 只保留最新颜色请求；现有
  10 ms 静态 Task 负责按键消抖与推进。无 HAL、软件模拟 SPI、动态内存或忙等待。
- 行为变化：屏幕初始化完成后显示红色；KEY1 每次稳定按下依次切换绿、蓝、白、黑、
  黄、红。KEY1 暂停控制 QD4310；TCP 下发 QD4310 命令的网关行为保持不变。
- 涉及文件：`BSP/bsp_tft.h`、`BSP/bsp_f407.c`、`Device/ST7735S/`、
  `APP/DisplayTest/`、`Task/task_app.c`、`config/core_config.h`、`CMakeLists.txt`、
  `README.md`。
- 验证结果：arm-none-eabi-gcc 10.3 Debug/Release 零警告；Debug 为 FLASH 3.19%、
  RAM 9.62%，Release 为 FLASH 2.71%、RAM 9.61%；ELF 中 DMA2 Stream3 强中断及
  BSP/Device/APP 符号存在。静态检查未发现 HAL、动态内存、忙等待或上层硬件引脚泄漏。
  D 盘 OpenOCD 对 Debug ELF 编程/校验/复位成功；目标运行约 19.8 s 时 Task 心跳
  1981、故障码 0，ST7735S 为 READY、160 行完成，DMA busy/done/error 均为 0，
  SPI1 与 PA4/PA5/PA7/PB0/PB1 寄存器映射符合配置。
- 已知限制：当前只支持资料指定的竖屏方向与纯色全屏测试，不提供字体、图像或局部 UI。
- 待确认：实屏颜色顺序、方向、四边完整性和 KEY1 切换尚未接线观察；若边缘偏移不符，
  只按实测调整 ST7735S 的 X/Y 模组偏移。

## 2026-07-29：QD4310 ID 绑定与 KEY1 Enable/Disable 翻转

- 修改原因：实机烧录测试发现配置 ID 0 无反馈；用户要求每次按 KEY 只发送一次命令，
  首次 Enable、再次 Disable，之后交替。
- 最终决策：QD4310 CAN 驱动保持 TI Template 实现；实测 NOP 扫描仅 ID 1 返回反馈，
  因此产品绑定改为 ID 1。KEY1 使用厂家 v5 板资料确认的 PA0、输入下拉、按下高有效。
- 执行边界：BSP 只配置并读取 KEY1 原始电平；现有网关 Task 保持 10 ms 周期，执行
  30 ms 非阻塞消抖且只在稳定按下时触发；APP 根据首台电机的 Device 命令门调用
  Enable 或 Disable。未新增按键中断、Task、动态内存或自动重试。
- 涉及文件：`BSP/bsp_key.h`、`BSP/bsp_f407.c`、`Task/task_app.c`、
  `APP/Gateway/app_gateway.[ch]`、`config/core_config.h`、`README.md`。
- 构建验证：arm-none-eabi-gcc 10.3 Debug/Release 零警告；Debug 为 FLASH 2.77%、
  RAM 8.80%，Release 为 FLASH 2.36%、RAM 8.80%。
- 实机验证：CMSIS-DAP + OpenOCD 识别 DEV_ID `0x413`、512 KiB Flash；Debug ELF
  编程和校验通过。单一调试会话运行约 1 s 时 Task 心跳增加 102、毫秒计数增加
  1020（包含恢复/暂停开销），符合 10 ms 周期；主程序与 Task 故障码均为 0。
  ID 1 周期反馈连续增长且 CAN ESR 为 0。
- 按键闭环验证：调试器将 PA0 等效置高/恢复两次；第一次按下计数 1、命令错误 0、
  `command_enabled=1`、电机反馈 Enabled；第二次按下计数 2、命令错误 0、
  `command_enabled=0`、电机反馈 Disabled。测试后 PA0 MODER/PUPDR/ODR 已恢复。
- 物理按键验证：恢复 GPIO 后，目标端按键计数由 2 增至 8，新增 6 次物理 KEY1
  事件均无命令错误；偶数次翻转后命令门和电机反馈均为 Disabled，确认按帽和走线有效。
- 已知限制：本轮只验证 Enable/Disable/NOP，不下发电流、速度或角度运动命令。

## 2026-07-29：与 G3519 同步部署 `ENC` 联调固件

- 执行原因：用户要求烧录双端当前 Debug 固件，使已经完成软件验证的 QD4310 原始
  绝对值 `ENC` 链路进入 G3519 的 21 路 VOFA 输出。
- F407 烧录：使用 VID:PID `0416:5051` 明确锁定 HID CMSIS-DAP；OpenOCD 识别
  Cortex-M4、DEV_ID `0x413` 和 512 KiB Flash。以 BIN 写入当前 Debug 固件
  `20900 B`，按 `0x08000000` 起始地址完整回读 20900 字节；回读 SHA-256 与固件
  `8D7DC1F20A563476A97AEE2FF96382D6BA1F0DA0AD31DAB23C7D3B77DEB65BDA` 完全一致，
  随后复位运行。
- 对端烧录：G3519 使用序列号
  `000100011a5a5bb60000000000045c0aa5a5a5a597969908` 的 DAPLink 写入 Bring-up Debug
  `56440 B`；完整回读 SHA-256 为
  `EA097067C7D7ADB1F8137F2191B44E6AE50ACAA45B1E86BDC597339574DD8D36`，一致后复位运行。
- 验证边界：本轮未按 KEY、未触发电机动作，也未重新采集 VOFA；因此只确认固件部署，
  不把 QD4310 在线、CH20 动态值或 200 ms 置零写成实物通过。G3519 已发送 `DIS` 的
  软件路径不代表 F407 已消费；滚球侧仍需实现接收事件后的 `0 A + Disable`。


## 2026-07-29：三份 VOFA 数据标定固定绝对零点

- 修改原因：用户要求读取桌面三份VOFA记录，经数学运算后作为QD4310无刷电机的绝对
  水平零点，不再使用“每次上电当前位置归零”。
- 最终决策：只使用 `I19=1` 且 `I20` 位于 `0..65535` 的有效样本。三份记录有效样本数
  为1028、1007、1039，各自I20中位数为55357、55183、55081；再取三者中位数，固定
  `GATEWAY_LEVEL_ZERO_RAW=55183`，对应303.133898°/5.290684593 rad。两级中位数让
  每次记录权重相同，并抑制记录内缓慢漂移和停留时长偏置。旧的上电当前位置零点方案
  已被本次固定机械零点覆盖。
- 架构边界：标定值只放在 `config/core_config.h`。Device新增原始值到弧度的单一换算
  API并同时供CAN反馈与Gateway零点装载使用；Gateway初始化控制器时装载固定零点，
  不新增Task、状态机、动态内存、通信字段、ACK、重发或位置模式。KEY启停、10 ms周期、
  CURRENT控制和现有保护出口保持不变。
- 涉及文件：`config/core_config.h`、`Device/QD4310/`、`APP/Gateway/`、QD4310主机测试、
  README和本记录。
- 验证方式和结果：桌面三份CSV共3074个有效样本完成统计；I18与I20按协议换算相符，
  仅受3位小数和异步TEL/ENC采样影响。arm-none-eabi-gcc 10.3 Debug/Release全量链接
  零警告，FLASH/RAM分别为20772/12344 B、17584/12336 B；QD4310源码与原始值测试以
  `-Wall -Wextra -Wpedantic -Werror` 严格ARM编译通过。已烧录Debug BIN的SHA-256为
  `436506B36EF1057379A90FC99C7965876DC7E59FC5644506CF15E3775CE71F85`。
- 烧录与运行：D盘xPack OpenOCD识别Cortex-M4、DEV_ID尾码0x413和512 KiB Flash，
  编程、校验和复位成功。运行快照中主程序/Task故障为0、Task Ready=1、心跳4515；
  控制器 `zero_captured=1`、`zero_angle_rad=5.2906847`、active=0、故障计数0，电机命令门
  保持失能。
- 已知限制：本次快照中QD4310反馈sequence=0、online=0、angle_raw=0，尚未形成CAN实物
  零点闭环；烧录和运行初始化成功不能替代机械水平与电流极性确认。
- 待确认：恢复QD4310供电/CAN反馈；水管接近水平、机构低风险时首次按KEY，验证
  CH20=55183附近的相对误差为0且电流方向把机构推回零点。

## 2026-07-29：水平闭环第一轮刚度调参并烧录

- 修改原因：固定零点闭环实测不够硬。调参前运行证据为零点55183、当前56009，偏差约
  4.54°，闭环active=1、QD4310在线且使能，但实际电流仅约-0.043 A、转速0、积分0；
  KEY和CAN链路正常，根因是P增益偏小且该偏差位于1°积分区之外。
- 最终决策：用户批准第一轮调整并要求烧录。只将KP由0.010提高到0.030 A/deg、转速
  阻尼KD由0.0005提高到0.0010 A/rpm；KI、死区、积分区、电流上限0.30 A和所有保护
  保持不变。相同4.54°偏差的P输出由约0.043 A提高到约0.130 A。
- 架构边界：仅修改 `config/core_config.h` 的现有参数及说明文档；不修改Task、算法结构、
  Device、通信、ISR/DMA、状态机或保护逻辑，不新增资源。
- 涉及文件：`config/core_config.h`、`README.md`、LevelCurrentControl README、协作文档和
  本记录。
- 验证方式和结果：烧录前确认KEY计数6、active=0、命令门/反馈使能均为0，电机约0 A。
  Debug/Release全量链接零警告，FLASH/RAM分别为20772/12344 B、17584/12336 B。D盘
  xPack OpenOCD识别F407 512 KiB Flash，编程、校验和复位成功；Debug BIN SHA-256为
  `662756910E64D105DE94D67D732FC9D0146FBBFD29176277B7B23EAA8247F999`。
- 运行检查：主程序/Task故障0、Ready=1、心跳2204；控制器实际装载KP=0.030、
  KI=0.0002、KD=0.0010、电流上限0.30 A和固定零点5.2906847 rad，active=0、控制故障0；
  QD4310在线、反馈失能、电流0 A。
- 已知限制：尚未按KEY执行新参数的运动闭环，不能把编译、烧录和参数内存值视为刚度、
  阻尼或稳定性实物通过。
- 待确认：支撑水管并置于水平附近，按F407 KEY启动；观察是否回零、是否振荡及CH20稳态
  偏差。若方向错误立即再按一次停机；若仍软再依据新快照分阶段调整P或积分，不直接扩大
  电流上限。

## 2026-07-29：振动反馈后将KP回退到0.020并烧录

- 修改原因：用户实测KP=0.030 A/deg时振动明显，明确判断P过大。
- 最终决策：否决KP=0.030，只把KP回退到0.020 A/deg；KD=0.0010 A/rpm、KI、固定零点
  55183、电流上限0.30 A及全部保护保持不变，以单变量方式继续测试。
- 执行边界：仅修改既有配置参数和文档，不改变算法、Task、通信、Device、状态机、保护
  或硬件资源。
- 安全状态：烧录前实测KEY计数4、错误0、闭环active=0、命令门和反馈使能均为0、
  QD4310在线且电流/转速均为0。
- 验证方式和结果：Debug/Release全量链接零警告，FLASH/RAM仍为20772/12344 B、
  17584/12336 B。D盘xPack OpenOCD编程、校验、复位成功；Debug BIN SHA-256为
  `75C4EC73EAE55CFF61FCAF883D3797FBC14560CAB96DAB4AF2ACA4FA02DADF03`。
- 运行检查：主程序/Task故障0、Ready=1、心跳2308；控制器实际装载KP=0.020、
  KD=0.0010、电流上限0.30 A、固定零点5.2906847 rad，active=0、故障0；QD4310在线、
  反馈失能、电流0 A。
- 已知限制：新参数尚未执行运动闭环，振动是否消除仍待实物确认。
- 待确认：按F407 KEY测试KP=0.020；只反馈“仍振/不振但软/合适”以及VOFA波形，再决定
  是否调整KD或积分，不恢复已否决的KP=0.030。

## 2026-07-29：持续振动后将KP回退到0.015并烧录

- 修改原因：用户实测KP=0.020 A/deg仍振动，要求继续减小P或增加D。
- 最终决策：本轮只将KP回退到0.015 A/deg，KD保持0.0010 A/rpm。未同时提高KD，避免
  QD4310转速反馈量化噪声可能带来的高频抖动，并保持单变量试验。KP=0.030和0.020均已
  因明显振动否决。
- 执行边界：仅修改既有配置参数和文档；固定零点55183、KI、电流上限0.30 A、保护、
  Task、通信和硬件配置不变。
- 安全状态：烧录前实测KEY计数4、错误0、active=0、命令门/反馈使能均为0、QD4310
  在线，电流约0 A、转速0。
- 验证方式和结果：Debug/Release全量链接零警告，FLASH/RAM为20772/12344 B、
  17584/12336 B；Debug BIN SHA-256为
  `EB36BCBF75BCA5D2D6266E047140142BB4B83984382EE445F1447E4750252D3A`。
- 烧录：两枚探针同时在线时，首次未限定探针的OpenOCD命令选择了G3519 DAPLink并识别为
  Cortex-M0+，在Flash自动探测阶段即失败，未执行写入。随后以
  `cmsis-dap vid_pid 0x0416 0x5051`明确锁定F407探针，识别Cortex-M4、DEV_ID尾码0x413、
  512 KiB Flash，编程、校验和复位成功。后续双探针环境禁止依赖自动选择。
- 运行检查：主程序/Task故障0、Ready=1、心跳2141；实际装载KP=0.015、KD=0.0010、
  电流上限0.30 A、固定零点5.2906847 rad，active=0、控制故障0；QD4310在线、失能、
  电流0 A。
- 已知限制：新参数尚未执行运动闭环，振动是否消除及刚度是否足够待实物确认。
- 待确认：按F407 KEY测试KP=0.015并记录VOFA；若仍振，先依据I20/速度/电流波形判断是否
  增加KD或降低控制带宽，不继续盲目同时改P、D。

## 2026-07-29：增加零点附近积分并烧录

- 修改原因：用户要求在KP=0.015、KD=0.0010的当前基线上增加少量I，以改善零点附近的
  稳态误差。
- 最终决策：只将KI由0.0002提高到0.0010 A/(deg*s)；积分作用区仍为±1°，积分分量仍
  限幅到±0.010 A，总电流仍限幅到±0.30 A。离开积分区立即清积分的原行为不变。
- 执行边界：仅修改现有配置参数和文档，不改变算法、Task、Device、通信、状态机、保护
  或硬件资源。
- 安全状态：烧录前运行固件为KP=0.015、KI=0.0002，主程序/Task故障0、Ready=1；
  QD4310在线、闭环和电机均失能、电流/转速0。
- 验证方式和结果：Debug/Release全量链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B；Debug BIN SHA-256为
  `E6F716AF902D854ED7275D8B5F8C0CA8A0DF0BC02AA86823CD21C743CDEC6BA2`。以
  `cmsis-dap vid_pid 0x0416 0x5051`锁定F407探针，D盘xPack OpenOCD编程、校验、复位
  成功。
- 运行检查：主程序/Task故障0、Ready=1、心跳2414；实际装载KP=0.015、KI=0.0010、
  KD=0.0010、积分区1°、积分限幅0.010 A、电流上限0.30 A、固定零点5.2906847 rad；
  active=0、积分0、控制故障0，QD4310在线、失能、电流0 A。
- 已知限制：新增积分尚未执行运动闭环，稳态误差改善和低频摆动风险待实物确认。
- 待确认：按F407 KEY启动后观察接近零点时的收敛速度和低频摆动；若出现缓慢往复，立即
  停机并回退KI，不同时修改P/D。

## 2026-07-29：依据三组采样将水平目标微调为+0.13°并烧录

- 修改原因：用户要求依据此前VOFA采样把目标角稍微增加。三份记录的CH20中位数为
  55357、55183、55081；固定机械零点仍取三者中位数55183，三者均值55207比零点高
  24计数，对应 `24*360/65535=0.13184°`。
- 最终决策：不改固定零点，只将 `GATEWAY_LEVEL_TARGET_DEG` 从0改为+0.13°。KP=0.015
  A/deg、KI=0.0010 A/(deg*s)、KD=0.0010 A/rpm、积分区/限幅、电流上限及全部保护保持
  不变。
- 执行边界：仅修改现有配置参数和说明文档，不改变算法、Task、Device、通信、状态机、
  引脚或保护逻辑。
- 安全状态：烧录前实测闭环active=0、命令门和反馈使能均为0、QD4310在线、电流0 A、
  主程序/Task故障0；确认失能后才执行烧录。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。以 `cmsis-dap vid_pid 0x0416 0x5051` 锁定F407探针，D盘xPack OpenOCD
  编程、校验、复位成功；Debug BIN SHA-256为
  `5FD04F086912EEB0A5F59E7B5551978BB168D9F89BD430432A3D7DCA50683CC5`。
- 运行检查：主程序/Task故障0、心跳2025；控制器实际装载目标0.129999995°、固定零点
  5.2906847 rad、KP/KI/KD分别为0.015/0.0010/0.0010。active=0、命令门=0；QD4310
  在线、反馈失能、转速0，反馈电流约0.000305 A。
- 已知限制：尚未按KEY执行+0.13°目标的运动闭环，不能把参数回读视为角度精度、稳定性
  或稳态误差通过。
- 待确认：支撑机构并按F407 KEY启动，观察VOFA CH20是否稳定在约55207附近以及是否出现
  振动或低频摆动；异常时再按一次KEY执行 `0 A + Disable`。

## 2026-07-29：Git上传前隔离ESP8266本机凭据

- 修改原因：创建远端代码仓库前发现 `core_config.h` 含本机ESP8266热点凭据，不应进入
  Git历史。
- 最终决策：本机真实SSID/密码移入被Git忽略的 `config/local_secrets.h`；仓库提交
  `local_secrets.example.h`，未创建本机文件时使用可编译的示例值。现有本机文件优先
  覆盖示例值，因此当前烧录配置不变。
- 执行边界：仅改变配置值的存放与Git边界，不修改ESP8266状态机、Task、协议、引脚、
  QD4310控制参数或保护行为。
- 涉及文件：`config/core_config.h`、`config/local_secrets.h`、
  `config/local_secrets.example.h`、`.gitignore`、`README.md`及本记录。
- 验证方式和结果：本机凭据文件存在且被忽略；Debug/Release完整链接零警告，FLASH/RAM
  仍分别为20772/12344 B、17584/12336 B，证明本机实际配置和固件尺寸未改变。
- 已知限制：新克隆默认使用示例热点参数，使用者需复制模板并填写自己的本机配置。
- 待确认：无；后续提交前继续确认 `config/local_secrets.h` 不在Git索引中。
