# F407-Gateway

基于 TI-Template 分层架构的 STM32F407VET6 滚球控制工程：F407通过CAN控制QD4310，
通过本地BMI088完成高速加速度补偿，并接收MaixCAM视觉位置。ESP8266代码保留，当前默认
关闭且不参与1 ms控制周期；电机反馈只在F407本地用于控制，不再跨板发送给G3519。

## 硬件映射

| 外设 | 引脚 | 参数 | 用途 |
|---|---|---|---|
| KEY1 | PA0 | 下拉输入，按下高电平 | 启动/停止水管水平电流闭环 |
| USART1 | PA9 TX / PA10 RX | 115200 8N1 | 对端 MCU 心跳与 G3519 `CAR` 接收（F407 节点 1） |
| USART2 | PA2 TX / PA3 RX | 115200 8N1，中断收发 | MaixCAM Pro `VIS`候选链路；接线尚待实物确认 |
| USART3 | PB10 TX / PB11 RX | 115200 8N1 | 预留ESP8266；当前运行配置关闭 |
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
│  ├─ Algorithm/RodAngleMap/          实测杆角分段插值与逆查
│  ├─ Algorithm/BallPositionControl/  球位置/速度PD与纯滚动逆模型
│  ├─ Algorithm/BallStateEstimator/   三状态固定滞后Kalman（已接入、默认关闭）
│  └─ Gateway/              TCP/CAN、滚球反馈与底盘前馈编排
├─ BSP/
│  ├─ bsp_uart.h bsp_can.h  与 MCU 无关的 BSP 接口（TI-Template 原样）
│  ├─ bsp_init.h
│  └─ bsp_f407.c            F407 寄存器级实现：时钟/UART/CAN
├─ Device/
│  ├─ ESP8266/              可选AT状态机；当前默认不初始化、不轮询
│  ├─ PeerLink/             USART1心跳及七字段CAR Latest Value
│  ├─ QD4310/               QD4310 CAN 驱动（TI-Template 原样移植）
│  └─ VisionLink/           USART2 VIS位置、flags、序号与年龄门控
├─ FreeRTOS/
│  ├─ Config/               FreeRTOSConfig.h（CM4F，PRIO_BITS=4，纯静态分配）
│  └─ Kernel/               FreeRTOS V10.3.1 + GCC/ARM_CM4F port
├─ Task/                    静态网关 Task，1ms周期（1 kHz）vTaskDelayUntil
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
2. 当前`GATEWAY_ESP8266_ENABLE=0`，不会建立热点或处理TCP。确需恢复旧无线调试时才改为1，
   并复制`config/local_secrets.example.h`为`config/local_secrets.h`填写热点参数。
3. 无线调试启用后，TCP连接`192.168.4.1:3333`并发送Packet帧（ASCII，LF结尾，
   CRC-16/CCITT-FALSE四位大写十六进制）：

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

所有参数集中在 `config/core_config.h`。电机外环名义周期为1 ms（1 kHz）、P=`0.017 A/deg`、
I=`0.0010 A/(deg*s)`、转速阻尼D=`0.0010 A/rpm`、静态承重前馈`-0.090 A`、
误差限幅`10°`、死区`0.20°`、积分区`1°`、积分限幅`0.010 A`、电流限幅`0.30 A`、
极性`+1`。固定机械参考零点为
`raw=54379`；工作微调`raw=54329`相对它约为电机角`-0.275°`，经当前机构实测表换算
则是杆角约`+0.061°`，两种角域不能混用。首次实机必须先确认电流极性：若轻推离开
目标后电机继续加速远离，立即停止并修正极性，不得先加大增益。

### 杆角表与滚球闭环

`GATEWAY_LEVEL_CALIBRATION_KEY_MODE=0`，KEY已恢复一次启动、一次停止。当前使用11点
向上扫描表；在此前确认的机械零点`raw=54379`处分段插值得到CH12示值`24.6164°`并相减，
所以控制角是相对旧零点的杆角，不是另立IMU零点。可用范围约`-3.9764°..+0.8336°`，
区间外命令只饱和到端点，不外推。该表没有下行分支，换向回差仍未标定。

当前以右侧固定转轴指向左侧执行端为杆轴`x>0`，以左侧执行端抬高为杆角`theta>0`。
相机画面右正，因此`x=-x_camera`；实测负电流使raw减小并抬高执行端。BMI088的
Sensor X朝车头，无刷失能时两次向车头手推的主动作均实测为处理后加速度先正后负。
执行端位于车头，向前加速的平衡补偿必须降低该端，即`theta<0`。因此源码配置冻结为
视觉`-1`、BMI088 X映射`+1`、加速度前馈`-1`、电机电流`+1`。

当前数据流为：

```text
VIS -> 球位置/真实采样间隔差分速度 -> 位置/速度PD -> 实心球逆模型杆角
BMI088 -> Pitch互补估计/去重力 -> 两级2.25 ms精确EMA -> 加速度前馈杆角
trim + 球反馈杆角 -> 实测分段逆表 -> raw -> 电机编码器CURRENT外环
```

`GATEWAY_BALL_VISUAL_CONTROL_ENABLE=1`已显式开启VIS位置/速度PD；VIS失效时清球反馈并
回到trim，不自动失能。板间CAR仍可接收和诊断，但完全不进入控制；控制只消费F407
本地BMI088。SPI-DMA在每个1 ms周期内分两次非阻塞推进，ACC/GYRO流水目标为完整六轴
  1 kHz帧；Pitch先由Gyro Y预测，仅在重力模长有效且候选纵向加速度不超过150 mm/s²时
  才用加速度计慢校正，扣除X轴重力投影后做两级2.25 ms精确EMA。1 kHz下总截止约46 Hz、
峰值滞后约2 ms，两个滤波级均单调且无过冲。按KEY后控制器先回到固定机械
零点`raw=54379`，位置误差不超过0.50电机度且转速不超过1 rpm连续300 ms后，才以
`b=0.50`启用BMI088补偿；公式为`theta_ff=-0.50*atan2(a_bmi_x/g)`，补偿杆角和视觉合成后的
总杆角均限于`±0.60°`。底盘三状态融合仍关闭；球Kalman已用于视觉帧间的1 ms模型预测。
当前角环P保持`0.017 A/deg`；
历史`0.030 A/deg`虽能提高远端电流但出现过振动，不再作为当前值。USART2接线、相机实物
朝向、下行回差和带球闭环仍须实物确认。

球目标上电默认值为`0 mm`，可在停机或回零phase 1期间通过PeerLink
`BTG=2,<target_0p1mm>`或Maix视觉链路`VBT` Packet预设
`[-120,+120] mm`内目标，停机与再次启动不会清除该值。活动参考按`80 mm/s`线性推进；
只有phase 2且视觉测量有效时，杆球模型杆角才进入最终命令，并继续与BMI088加速度前馈
杆角相加。`BST=2`仍优先执行原有`0 A -> Disable`；BTG/VBT都不会隐式启动，启动仍由
F407本地KEY负责。

当前球外环为`Kp/Ki/Kd=1.00/0.030/1.00`。破静摩擦脉冲在误差大于`3 mm`、
球速不超过`12 mm/s`并持续`150 ms`时触发：从`0.45°`开始，每次增加`0.10°`，
最高`0.70°`，脉宽`150 ms`、冷却`200 ms`；检测到球移动或进入目标区即撤销。

电机绝对编码器硬边界来自零电流手动实测：`49325..59233`。目标raw、11点杆角表和
实时反馈同时受此边界约束；反馈越界立即发送0 A并Disable，位于边界时只允许指向区间
内部的电流。当前11点表`53577..58499`更窄，因此正常目标仍优先受实测杆角表约束。

三状态估计器由`GATEWAY_BALL_KALMAN_ENABLE=1`启用。状态为`[x,v,b]`：

```text
kappa = 1/(1+I/(mR^2)) = 5/7
a_model = -kappa*(g*sin(theta_rod) + a_fwd*cos(theta_rod))
x_dot = v
v_dot = a_model + b + w_a
b_dot = w_b
z_VIS = x + n
```

其中杆角来自QD4310 raw经11点实测表的反馈值；BMI088处理量已经是模型所需的车辆
向前正加速度。`b`是由VIS残差估计的慢变等效加速度。128个静态历史快照在1 ms下
覆盖约127 ms；
延迟VIS在历史时刻用5σ创新门控校正后重放到当前。至少接受3帧VIS且`Pxx/Pvv`
达标才输出可信状态。两帧YOLO实测之间按实际杆角和BMI088底盘加速度每1 ms传播；
短时无实测帧时继续使用可信预测，距最后接受的实测达到100 ms后停止球外环输出。
目标轮速差分、自适应权重和底盘三状态融合本轮均未启用。

## MCU 板间串口心跳与蓝牙遥测

F407 端已经使用 USART1（PA9 TX / PA10 RX，DMA2 Stream7/Stream2 Channel4）与
MSPM0G3519 的 UART1（PB4 TX / PB5 RX，DMA_CH1/DMA_CH0）直接通信。本机节点 1、
G3519 节点 2，每 500 ms 双向发送一次 `HB`，1500 ms 无合法对端心跳判定离线；
F407不再发送主QD4310的 `TEL/ENC` CAN明细，只接收G3519以100 Hz提交的七字段
`CAR`：平均车速、ICM45686 X/Z加速度计比力、零偏校正后的Pitch/Y角速度和有效位。
PeerLink检查来源、范围、独立序号、`sample_ms`顺序、重启会话和50/51 ms新鲜度；
通信使用静态缓冲区并由现有1 ms Task非阻塞推进。CAR仅保留通信兼容与诊断，不进入
加速度补偿、融合或电机目标；底盘三状态融合仍关闭。完整接线、帧格式、错误处理及联调验收见
[MCU 串口心跳通信协议](MCU_UART_HEARTBEAT_PROTOCOL.md)。

F407本地目标合成、BMI088完整帧和电机角外环调度目标均为1 kHz；实物更新率仍须在
烧录后按序号/时间测量确认。现有115200、8N1下，七字段ASCII CAR保持100 Hz且不参与控制。

为方便实时调试，F407在每次Gateway计算之后以50 Hz提交只读`DBG`，G3519将它追加到
COM5 FireWater的CH22～CH31；原CH0～CH21顺序不变。VOFA使用COM5、115200、8N1、
FireWater。CH31 bit9由G3519置位表示DBG年龄不超过100 ms；该链路只做抽取显示，不改变
1 kHz控制调度，也不作为任何控制输入。

MaixCAM Pro候选链路独占USART2，`VIS`格式、flags及年龄门控见
[VisionLink说明](Device/VisionLink/README.md)。相机发送端和实物接线均不属于本轮已验证范围。

## 验证状态

- LevelCurrentControl、RodAngleMap、BallPositionControl、BallStateEstimator、VisionLink、
  PeerLink和QD4310七组主机测试的历史基线均以`-Wall -Wextra -Werror`通过。本轮冻结日志
  回放确认两级滤波静止RMS约`14.24 mm/s²`、强推正/负削峰约`5.0%/4.3%`、峰滞后约2 ms；
  `chassis_imu_acceleration`和`control_polarity_chain`测试均以ARM GCC严格警告完成编译。
  当前环境没有本机C运行器，因此本轮没有把“编译通过”写成测试运行PASS。当前源码
  Debug/Release完整链接且无警告，FLASH/RAM分别为`38396/20072 B`和`33472/20048 B`。
  额外使用`GATEWAY_BALL_KALMAN_ENABLE=1`强制构建已链接通过，最终默认配置仍为0。
- 无刷失能下两次向车头短推均确认BMI088处理加速度的主动作先正后负，当前源码
  `GATEWAY_BMI088_ACCEL_X_SIGN=+1`、`GATEWAY_CHASSIS_FF_POLARITY=-1`与车头执行端应降低
  的物理动作一致。采样结束后已关闭OpenOCD；后续修改均为离线，未再控制F407。
- 本轮未进行USART2或带球闭环实测。G3519的CAR发送端和MaixCAM的VIS发送端不在当前
  F407工作树；F407当前`b=0.50`，但桌面MaixCAM工程仍以
  `F4_UART_CALIBRATION_VALID=False`发送，所以在视觉标定位放行前F407会按协议拒绝其位置。
- 11点向上表和旧机械零点已经进入代码；下行表、回差、CH12坐标关系的绝对角验证、
  工作区双向小阶跃、负载扰动恢复以及相机极性仍待实测。
- 此前版本曾完成F407/G3519 `HB/TEL/ENC`烧录和约62.5 s通信验证，并验证KEY启停、
  CAN1/QD4310及最终Disable状态；这些历史结果不能替代本轮新增代码的硬件验证。
- TFT测试仍停用；SPI1现专用于BMI088，DMA2 Stream0/Stream3服务SPI1 RX/TX，
  Stream2/Stream7继续服务USART1 RX/TX。
