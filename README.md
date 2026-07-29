# F407-Gateway

基于 TI-Template 分层架构的 STM32F407VET6 无线网关工程：
手机/电脑连 ESP8266 热点 → TCP 发命令 → CAN 总线控制 QD4310 伺服电机，
电机反馈实时回传 TCP，也经板间 DMA 串口送往 G3519 蓝牙统一输出。

## 硬件映射

| 外设 | 引脚 | 参数 | 用途 |
|---|---|---|---|
| KEY1 | PA0 | 下拉输入，按下高电平 | 启动/停止水管水平电流闭环 |
| USART1 | PA9 TX / PA10 RX | 115200 8N1 | 对端 MCU 心跳与 QD4310 遥测（F407 节点 1） |
| USART3 | PB10 TX / PB11 RX | 115200 8N1 | ESP8266（板上跳线帽接通） |
| CAN1 | PB9 TX / PB8 RX | 1 Mbps | QD4310 总线（经板载 CAN 收发器） |
| 时钟 | HSE 25 MHz | SYSCLK 168 MHz | APB1 42M / APB2 84M |

QD4310 CAN 协议固定 1 Mbps：控制帧 `0x400+ID` DLC3，反馈帧 `0x500+ID` DLC8。
电机需先经 USB 上位机完成基础校准并配置唯一 CAN ID（手册 3.2/3.3 节）。

## 目录（TI-Template 风格）

```text
F407-Gateway/
├─ APP/
│  ├─ Algorithm/Packet/     LF/CRC16 组帧（TI-Template 原样移植）
│  ├─ Algorithm/SPSC_Ring/  无锁字节环（原样移植，暂未使用）
│  ├─ Algorithm/LevelCurrentControl/  绝对角外环、转速阻尼与电流限幅
│  └─ Gateway/              TCP<->CAN 网关业务
├─ BSP/
│  ├─ bsp_uart.h bsp_can.h  与 MCU 无关的 BSP 接口（TI-Template 原样）
│  ├─ bsp_init.h
│  └─ bsp_f407.c            F407 寄存器级实现：时钟/UART/CAN
├─ Device/
│  ├─ ESP8266/              AT 状态机：AP 热点 + TCP 服务器，非阻塞
│  ├─ PeerLink/             USART1 MCU 心跳、QD4310 遥测与在线状态
│  └─ QD4310/               QD4310 CAN 驱动（TI-Template 原样移植）
├─ FreeRTOS/
│  ├─ Config/               FreeRTOSConfig.h（CM4F，PRIO_BITS=4，纯静态分配）
│  └─ Kernel/               FreeRTOS V10.3.1 + GCC/ARM_CM4F port
├─ Task/                    静态网关 Task，10ms 周期 vTaskDelayUntil
├─ config/core_config.h     可调参数唯一入口（端口/周期/电机 ID）
├─ config/local_secrets.example.h  ESP8266 本机凭据模板
├─ startup/ linker/         GNU 启动文件与链接脚本（F407VE 512K/128K+64K CCM）
└─ main.c                   BSP_Init -> Task_Init -> vTaskStartScheduler
```

## 构建

需要 arm-none-eabi-gcc、CMake ≥3.21、Ninja（均已在 PATH）。

```powershell
cmake --preset debug
cmake --build --preset debug     # 产物 build/Debug/f407_gateway.{elf,hex,bin}
```

## 使用流程

1. 给电机和 F407 同步上电；CANH/CANL 接开发板 CAN 收发器输出。当前实测电机 CAN
   ID 为1，固定机械水平零点由 `GATEWAY_LEVEL_ZERO_RAW` 提供。
2. 首次使用可复制 `config/local_secrets.example.h` 为 `config/local_secrets.h` 并填写
   ESP8266 热点参数；该本机文件已被 Git 忽略。未创建时使用可编译的示例热点
   `F407-Gateway`（密码 `change-me`）。
3. TCP 连接 `192.168.4.1:3333`（ESP8266 AP 默认网关 IP）。
4. 发送 Packet 帧（ASCII，LF 结尾，CRC-16/CCITT-FALSE 四位大写十六进制）：

板载 KEY1（PA0）稳定按下 30 ms 后触发一次命令：固件使用标定后的固定绝对编码器
水平零点；在“反馈在线、电机失能、|转速|≤1 rpm”时，第一次按键发送 `Enable + 0 A`，
随后每收到一帧新反馈执行一次“绝对角外环 → CURRENT”控制；第二次按键发送
`0 A + Disable`。长按不会重复触发。反馈 100 ms 超时、相对角越过 ±90°、计算或
CAN 发送失败时也会停止并清积分，且不会自动重启。

```text
@1|CMD|<seq>|<len>|<payload>|<CRC16>\n
```

payload 命令表（motor_id 为十进制 CAN ID）：

| payload | 含义 |
|---|---|
| `1,ENABLE` | 使能电机 1 |
| `1,DISABLE` | 失能 |
| `1,SPEED,100` | 速度模式 100 rpm（±500 默认限幅） |
| `1,CURRENT,0.5` | 电流模式 0.5 A（±1 A 默认限幅） |
| `1,LOWSPEED,3` | 低速模式（<5 rpm 推荐） |
| `1,NOP` | 仅请求一帧反馈 |

水平闭环运行期间，主电机只接受 `NOP` 或 `DISABLE`，其他远程模式切换返回 `ERR`。
网关不开放 `ANGLE`/`STEP`，水平控制不会调用 QD4310 内部位置模式。

网关回复：

- `ACK` 帧：`<seq>,OK` 或 `<seq>,ERR`（seq 为命令帧序号）。
- `FB` 帧（每 100ms 自动 NOP 请求 + 有新反馈才发）：
  `<motor_id>,<en>,<current_A>,<speed_rpm>,<angle_rad>,<online>`。

CRC 计算范围：`@` 起到 CRC 前那个 `|` 止（含）。例：`@1|CMD|0|8|1,ENABLE|` 的
CRC16-CCITT-FALSE 附在后面再加 `\n`。

## 水平电流闭环调参

所有参数集中在 `config/core_config.h` 的 `GATEWAY_LEVEL_*`。当前值为10 ms、
P=`0.017 A/deg`、I=`0.0010 A/(deg*s)`、转速阻尼D=`0.0010 A/rpm`、误差限幅
`8°`、死区 `0.20°`、积分区 `1°`、积分限幅
`0.010 A`、电流限幅 `0.30 A`、极性 `+1`。P=`0.030/0.020 A/deg`均振动明显，
当前P由0.015小幅增加到0.017；I由 `0.0002` 谨慎提高到 `0.0010 A/(deg*s)`，积分区和
积分限幅保持 `1°/0.010 A`。最新VOFA最后连续使能段末100帧raw均为54379、标准差0、
转速0；按用户确认将54379设为固定机械零点，运行目标再减小约50 counts至 `-0.275°`
（约raw=54329）。此前增加raw以提高机械位置的方向已否决；目标/硬限位分别为
`85°/90°`。首次实机必须先确认电流极性：若轻推离开零点后电机继续加速远离，
立即停止并把 `GATEWAY_LEVEL_CURRENT_POLARITY` 改为 `-1.0F`，不得先加大增益。

### 临时五点机构角度标定

当前 `GATEWAY_LEVEL_CALIBRATION_KEY_MODE=1`。F407 KEY每次稳定按下只推进一个标定点；
第1次使能并进入点1，第2～5次依次换点，第6次发送 `0 A + Disable`。五点相对固定零点
54379的命令依次为 `-4.395/-2.197/0/+2.197/+4.395°`，约对应原始目标
`53579/53979/54379/54779/55179`。每点等待CH20和转速稳定后，记录实际CH20与杆面实际
倾角；若测左端上球头相对水平高度变化，则用 `theta=asin(delta_h/250 mm)` 换算。
完成后必须把标定模式改回0，恢复正常KEY一次启动、一次停止，并用实测点生成单调查表。

## MCU 板间串口心跳与蓝牙遥测

F407 端已经使用 USART1（PA9 TX / PA10 RX，DMA2 Stream7/Stream2 Channel4）与
MSPM0G3519 的 UART1（PB4 TX / PB5 RX，DMA_CH1/DMA_CH0）直接通信。本机节点 1、
G3519 节点 2，每 500 ms 双向发送一次 `HB`，1500 ms 无合法对端心跳判定离线；
F407每20 ms发送原有 `TEL`（主QD4310的ID、使能、电流、速度、单圈角度和在线），
并发送独立 `ENC`（CAN反馈data[6..7]原始绝对值和在线），用于机械水平零点标定。
通信使用静态缓冲区并由两端现有 10 ms Task 非阻塞推进；链路故障不会停止 KEY、
QD4310或ESP8266。已验证的小车固件仍将本地14路与F407六路合成固定20路；小车Agent
接入 `ENC` 后保持CH0～CH19不变，并把原始绝对值追加为CH20，形成固定21路VOFA+
FireWater。完整接线、帧格式、错误处理及联调验收见
[MCU 串口心跳通信协议](MCU_UART_HEARTBEAT_PROTOCOL.md)。

## 验证状态

- 当前五点标定固件Debug/Release完整链接零警告，FLASH/RAM分别为
  `21048/12344 B`、`17828/12336 B`；水平控制算法严格ARM编译通过。本机无host GCC，
  本轮未执行host测试程序。
- F407 Debug已使用D盘OpenOCD并按VID:PID `0416:5051`锁定探针完成烧录、逐段校验和
  复位；BIN SHA-256为
  `E515AD14ECE4CDEA78D0EEC47DC2317B7D43677873F467578B911011FC447F29`。上电检查中
  控制未激活、命令门关闭、QD4310失能且故障计数为0。
- 最新VOFA稳定段将固定机械零点标定为54379；当前正常运行目标为相对零点-0.275°，
  P/I/D为0.017/0.0010/0.0010。五点KEY控制已完成并实测左端高度
  10.0/9.6/9.3/9.2/9.1 cm，对应杆面角约
  +1.6045/+0.6876/0/-0.2292/-0.4584°；本轮实际稳定CH20尚未保存，因此最终
  `raw -> 杆面角`查表和反向间隙仍待补测。
- 已验证：加入 F407 USART1 DMA 心跳与 `TEL` 后的 arm-none-eabi-gcc 10.3
  Debug/Release 编译链接零警告；Debug 使用 FLASH 3.52% / RAM 9.34%，Release 使用
  FLASH 2.98% / RAM 9.34%。D 盘 OpenOCD 已将最新 Debug BIN 烧录到两端，并分别
  完整校验 F407 `18460` 字节和 G3519 `55456` 字节。
- 双端实机验证：同步启动后连续运行约 62.5 s，双方心跳发送/接收均为 `126/126`；
  F407 成功发送 `3142` 帧 `TEL`，G3519 完整接收 `3142` 帧。双方在线，两端
  Task/PeerLink 故障、TX 丢弃、Packet/CRC/节点/重复序号、UART 溢出/帧错误和 DMA
  错误全部为 0。双方现已部署 `ENC`/21列VOFA固件；本轮尚未重新独占COM5采集21列
  动态值和200 ms超时置零证据。
- 此前版本已实机验证：STM32F407VE 512 KiB 识别、Flash 编程/校验/复位、168 MHz 时钟与
  FreeRTOS 10 ms Task、ESP8266 READY、CAN1 1 Mbps、QD4310 ID 1 周期反馈、
  KEY1 两次触发 Enable/Disable 及电机状态反馈；物理 KEY1 连续按压事件已由目标端
  计数确认，命令错误为 0，最终保持 Disable。
- TFT 测试已停用：当前目标不使能 SPI1、不配置 TFT GPIO、不执行 ST7735S 初始化；
  DMA2 现在仅服务 USART1 的 Stream2/Stream7，TFT 原 Stream3 仍未配置。先前 TFT
  Device/APP/接口源码保留在工作区，但未编入当前固件。
- 早期CAN ACK错误和QD4310离线问题已经恢复；最新烧录前检查中QD4310在线、转速0、
  电流接近0并保持失能。当前遗留项是五点实际稳定CH20、最终查表及扰动回位验证。
