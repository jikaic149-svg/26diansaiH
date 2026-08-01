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
│  ├─ Algorithm/BallModelInterpolator/ 杆球模型1 ms插值与alpha-beta视觉校正
│  ├─ Algorithm/BallPositionControl/  球位置/速度PD与纯滚动逆模型
│  ├─ Algorithm/BallStateEstimator/   三状态固定滞后Kalman（已接入、默认关闭）
│  └─ Gateway/              TCP/CAN、滚球反馈与底盘前馈编排
├─ BSP/
│  ├─ bsp_uart.h bsp_can.h  与 MCU 无关的 BSP 接口（TI-Template 原样）
│  ├─ bsp_init.h
│  └─ bsp_f407.c            F407 寄存器级实现：时钟/UART/CAN
├─ Device/
│  ├─ ESP8266/              可选AT状态机；当前默认不初始化、不轮询
│  ├─ PeerLink/             USART1心跳及兼容八/十字段CAR Latest Value
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

所有参数集中在 `config/core_config.h`。电机外环名义周期为1 ms（1 kHz）。只有新鲜
`competition_mode=3`使用2026-07-31 08:51参数：`P/I/D=0.014/0.0010/0.0010`、静态承重
前馈`-0.072 A`、死区`0.20°`、积分限幅`0.010 A`。Task2/4/5/6使用原参数：
`P/I/D=0.014/0.0020/0.0010`、前馈`-0.090 A`、死区`0.05°`、积分限幅`0.020 A`。
两组参数的误差限幅均为`10°`、积分区均为`2°`、电流限幅均为`0.30 A`、极性均为`+1`；
PID公式不变，不增加转速滤波或最终电流限斜率。固定机械参考零点为
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
M3: VIS -> 120 ms多帧差分速度 -> 位置/速度环 -> 实心球逆模型杆角
M2/M4/M5/M6: VIS位置校正 -> 1 ms轻量杆球模型[x,v] -> 位置/速度环 -> 实心球逆模型杆角
M3 -> 底盘加速度前馈关闭；M4/M5/M6统一规划 -> 小系数负加速度停车基准 + 三状态预测；CAR轮速 + F407/BMI088 -> 融合校正；M2/规划失鲜 -> 纯BMI088
选定纵向加速度 -> 50 ms新鲜度门 -> 加速度前馈杆角
trim + 球反馈杆角 -> 实测分段逆表 -> raw -> 电机编码器CURRENT外环
```

`GATEWAY_BALL_VISUAL_CONTROL_ENABLE=1`已显式开启VIS位置/速度PD；单个不可用VIS不再
覆盖最后有效测量，旧测量按自身年龄在100 ms处严格失效，不自动失能电机。新版十字段CAR追加车头正向的规划纵向加速度及独立有效位；只有
心跳在线、M4/M5/M6、CAR不超过50 ms且规划有效时才启用规划、轮速与BMI088融合。G3519实测
加速度不进入控制；M4/M5/M6负规划另以固定`0.25`系数提供小幅停车基准，正加速不叠加。
M2、旧八字段或规划过期时继续使用本地BMI088；M3仍采样BMI088，但不将其用于杆角前馈。SPI-DMA在每个1 ms周期内分两次非阻塞推进，ACC/GYRO流水目标为完整六轴
  1 kHz帧；Pitch先由Gyro Y预测，仅在重力模长有效且候选纵向加速度不超过150 mm/s²时
  才用加速度计慢校正，扣除X轴重力投影后做两级2.25 ms精确EMA。1 kHz下总截止约46 Hz、
峰值滞后约2 ms，两个滤波级均单调且无过冲。按KEY后控制器先回到固定机械
零点`raw=54379`，相对零点误差不超过`1.50°`且转速绝对值不超过`15 rpm`连续300 ms后，才以
`b=0.25`启用纵向加速度补偿；公式为`theta_ff=-0.25*atan2(a_selected/g)`，补偿杆角限于
`±0.60°`，最终总杆角仍受不对称机械范围保护。底盘三状态融合已启用；球Kalman保持关闭。
Task3角环继续使用2026-07-31 08:51:36提交的原始PID参数，不增加Task3转速滤波或电流限斜率；
滚球远距离运动与其他赛题共用连续速度PI，但M3直接使用真实视觉多帧差分球速；
历史`0.030 A/deg`虽能提高远端电流但出现过振动，不再作为当前值。USART2接线、相机实物
朝向、下行回差和带球闭环仍须实物确认。

球目标上电默认值为`0 mm`，可在停机或回零phase 1期间通过PeerLink
`BTG=2,<target_0p1mm>`或Maix视觉链路`VBT` Packet预设
`[-120,+120] mm`内目标，停机与再次启动不会清除该值。活动参考按`120 mm/s`线性推进；
只有phase 2且视觉测量有效时，当前模式的球反馈杆角才进入最终命令，并继续与底盘加速度前馈
杆角相加。`BST=2`仍优先执行原有`0 A -> Disable`；BTG/VBT都不会隐式启动，启动仍由
F407本地KEY负责。

当前近目标位置环使用`Kp/Ki/Kd=0.60/0/0.55`。Task2/4/5/6的远距离速度PI仍限制为
`±30 mm/s`并按`180 mm/s²`真实时间限斜率，切环和安静保持误差均为`3 mm`；Task3单独使用
`±45 mm/s`、`300 mm/s²`和`1.5 mm`切环/保持误差，电机角环继续使用07-31原PID。
同一目标若再次低速停在入口阈值之外达250 ms，速度PI重新脱离静摩擦。离散角度脉冲和
近目标Kd调度保持禁用，真实扰动达到退出滞环并连续3帧后恢复控制。

电机绝对编码器硬边界来自零电流手动实测：`49325..59233`。目标raw、11点杆角表和
实时反馈同时受此边界约束；反馈越界立即发送0 A并Disable，位于边界时只允许指向区间
内部的电流。当前11点表`53577..58499`更窄，因此正常目标仍优先受实测杆角表约束。

当前由`GATEWAY_BALL_MODEL_INTERPOLATION_ENABLE=1`为Task2/4/5/6启用轻量二状态插值；
Task3绕过该模型并直接使用真实VIS多帧差分球速，Kalman仍为离线对照且默认关闭。模型状态为
`[x,v]`：

```text
kappa = 1/(1+I/(mR^2)) = 5/7
a_model = -kappa*(g*sin(theta_rod) + a_fwd*cos(theta_rod))
x_dot = v
v_dot = a_model
z_VIS = x
```

其中杆角来自QD4310 raw经实测表的反馈并扣除水平trim，底盘加速度使用Gateway最终选择值。
F407每1 ms传播状态，真实YOLO位置按测量年龄做固定滞后alpha-beta校正；绝对创新超过
12 mm时拒绝该帧。至少接受2帧实测才输出可信状态，距最后实测达到100 ms后停止采用。
该模块不估计慢变偏置，也不把模型预测伪装成新的视觉测量。

## MCU 板间串口心跳与蓝牙遥测

F407 端已经使用 USART1（PA9 TX / PA10 RX，DMA2 Stream7/Stream2 Channel4）与
MSPM0G3519 的 UART1（PB4 TX / PB5 RX，DMA_CH1/DMA_CH0）直接通信。本机节点 1、
G3519 节点 2，每 500 ms 双向发送一次 `HB`，1500 ms 无合法对端心跳判定离线；
F407不再发送主QD4310的 `TEL/ENC` CAN明细，只接收G3519以100 Hz提交的兼容八/十字段
`CAR`：基础八字段为平均车速、ICM45686 X/Z加速度计比力、零偏校正后的Pitch/Y角速度、
有效位和赛题mode `2..6`；十字段在尾部追加规划纵向加速度和独立有效位。mode继续转发到
Maix显示且不参与滚球启停；其中mode 3仅用于禁止底盘加速度前馈。
PeerLink检查来源、范围、独立序号、`sample_ms`顺序、重启会话和50/51 ms新鲜度；
通信使用静态缓冲区并由现有1 ms Task非阻塞推进。M4/M5/M6新鲜有效规划量进入前馈，轮速和
本地BMI088已接入并启用底盘三状态融合框架；M3关闭该前馈，M2或规划失鲜时自动回退纯BMI088。完整接线、字段、M4/M5/M6
规划约束和联调验收见
[MCU 串口心跳通信协议](MCU_UART_HEARTBEAT_PROTOCOL.md)。

F407本地目标合成、BMI088完整帧和电机角外环调度目标均为1 kHz；实物更新率仍须在
烧录后按序号/时间测量确认。规划量复用CAR尾字段，不增加新的100 Hz Packet类型。

为方便实时调试，F407在每次Gateway计算之后以50 Hz提交只读`DBG`，G3519将它追加到
COM5 FireWater的CH22～CH31；原CH0～CH21顺序不变。VOFA使用COM5、115200、8N1、
FireWater。CH22～CH24分别为最终采用纵向加速度、补偿杆角和总杆角命令；CH31 bit9由
G3519置位表示DBG年龄不超过100 ms。该链路只做抽取显示，不改变
1 kHz控制调度，也不作为任何控制输入。

MaixCAM Pro候选链路独占USART2，`VIS`格式、flags及年龄门控见
[VisionLink说明](Device/VisionLink/README.md)。相机发送端和实物接线均不属于本轮已验证范围。

## 验证状态

- LevelCurrentControl、RodAngleMap、BallPositionControl、BallStateEstimator、VisionLink、
  PeerLink和QD4310七组主机测试的历史基线均以`-Wall -Wextra -Werror`通过。本轮冻结日志
  回放确认两级滤波静止RMS约`14.24 mm/s²`、强推正/负削峰约`5.0%/4.3%`、峰滞后约2 ms；
  本轮`level_current_control`和`control_polarity_chain`均以MSYS2 GCC 16.1严格警告运行PASS。
  当前源码Debug/Release clean-first完整链接且无警告，FLASH/RAM分别为
  `50456/20800 B`和`43692/20784 B`。
  额外使用`GATEWAY_BALL_KALMAN_ENABLE=1`强制构建已链接通过，最终默认配置仍为0。
- 无刷失能下两次向车头短推均确认BMI088处理加速度的主动作先正后负，当前源码
  `GATEWAY_BMI088_ACCEL_X_SIGN=+1`、`GATEWAY_CHASSIS_FF_POLARITY=-1`与车头执行端应降低
  的物理动作一致。当前Debug已完整校验烧录并复位；复位后电机保持在线但失能、0 mA、
  0 rpm、Task fault=0，尚未按KEY进行Task3动态复测。
- 本轮未进行USART2或带球闭环实测。G3519的CAR发送端和MaixCAM的VIS发送端不在当前
  F407工作树；F407当前`b=0.50`，但桌面MaixCAM工程仍以
  `F4_UART_CALIBRATION_VALID=False`发送，所以在视觉标定位放行前F407会按协议拒绝其位置。
- 11点向上表和旧机械零点已经进入代码；下行表、回差、CH12坐标关系的绝对角验证、
  工作区双向小阶跃、负载扰动恢复以及相机极性仍待实测。
- 此前版本曾完成F407/G3519 `HB/TEL/ENC`烧录和约62.5 s通信验证，并验证KEY启停、
  CAN1/QD4310及最终Disable状态；这些历史结果不能替代本轮新增代码的硬件验证。
- TFT测试仍停用；SPI1现专用于BMI088，DMA2 Stream0/Stream3服务SPI1 RX/TX，
  Stream2/Stream7继续服务USART1 RX/TX。
