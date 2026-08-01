# Process

## 2026-08-01：最新整圈弧线速度阻尼小调

- 修改原因：桌面11:04版`1.csv`共1920帧、38.4秒，为一整圈而非仅第一弧线。第一弧线CH24为`-0.724..+0.332 deg`，进入第二直线后继续达到`-0.864 deg`；同期弧线CH23底盘补偿约`0.011 deg RMS`、无杆角限幅，表明主要是滚球趋势制动不足，不是BMI088/规划补偿压过位置环。
- 最终决策：保持位置`Kp=0.50`，仅将视觉球速阻尼`Kd`由`0.75`提高到`0.80 1/s`；`Ki=0`、60 ms测速窗口、20 ms低通、助启动`250 ms / 8 mm/s`、题号门控、模型插值、底盘补偿及全部保护不变。
- 涉及文件：`config/core_config.h`和本记录；未修改Task、协议、状态机、视觉程序或电机启停逻辑。
- 验证与部署：BallPositionControl与control polarity chain严格主机测试均PASS；`git diff --check`通过；ARM Debug clean-first完整链接通过，FLASH/RAM为`57156/20992 B`，BIN SHA-256为`99A0E93D4A66DAA645A07E51EE80F17CF8CC7C5DF618805F2F3EA96F0C51F052`。用户确认电机失能后，OpenOCD锁定F407 CMSIS-DAP `0416:5051`写入Debug ELF，报告`Programming Finished`并复位；未做verify/readback，未按KEY。
- 已知限制与待确认：VOFA仍不含同步真实球位/球速；软件与烧录成功不能证明弧线偏移已改善。下一次只短测第一直线和第一弧线，观察弧线出口是否不再把CH24拖到大负角，异常立即按KEY停止。

## 2026-08-01：撤销弧线位置Kp小调

- 最终决策：用户要求还原上一版，仅将滚球位置环`Kp`由`0.55`恢复为`0.50 1/s²`；上一轮提高Kp的方向已否决。其余位置环、视觉测速、模型插值、底盘补偿、Task、协议和保护均保持不变。
- 涉及文件：`config/core_config.h`和本记录。
- 验证与部署：ARM Debug clean-first完整链接通过，FLASH/RAM为`57156/20992 B`；BIN SHA-256恢复为上一版`B7990B15CCFBD1993C0B550D4AB3D1741E3D6FD90B79D772EFDF4EDCCD720E81`。用户授权烧录后，OpenOCD锁定F407 CMSIS-DAP `0416:5051`写入Debug ELF，报告`Programming Finished`并复位；未做verify/readback，未按KEY。
- 待确认事项：由用户按KEY复测；当前仅证明上一版镜像已恢复，未进行动态实车验证。

## 2026-08-01：最新整圈弧线位置恢复力度小调

- 修改原因：桌面最新 `1.csv` 共5201帧、104.02秒，包含两次约5830 mm完整运行。弧线区间内CH23底盘补偿仅约`8～14 mdeg RMS`且没有持续限幅；`±600 mdeg`瞬态均出现在两次路线结束之后，不能据此继续削弱正常赛道前馈。用户目视仍有少量弧线跑飞，现象更符合滚球位置恢复偏软。
- 最终决策：仅将滚球位置环`Kp`由`0.50`小幅提高到`0.55 1/s²`；保持`Ki=0`、`Kd=0.75 1/s`、60 ms视觉测速窗口、20 ms低通、模型位置`alpha=0.80`、速度助启动、底盘补偿及全部保护边界不变。
- 涉及文件：`config/core_config.h`和本记录；未修改Task、协议、状态机、视觉程序、底盘规划或电机启停行为。
- 验证方式和结果：`git diff --check`通过；ARM Debug clean-first完整链接通过，FLASH/RAM为`57156/20992 B`，BIN SHA-256为`64A30634D00C34748A8D2C73AC027BC91A66C62ADCEA32EF9AF2661686D837D3`。用户确认停机失能后，OpenOCD锁定F407 CMSIS-DAP `0416:5051`写入Debug ELF，报告`Programming Finished`并复位；按要求未做verify/readback，未按KEY。
- 已知限制：VOFA 34列仍不含同步球位置和球目标，数据只能排除弧线补偿持续限幅，不能离线计算钢球最大偏移；本次参数尚未进行动态实车验证。
- 待确认事项：按KEY后只复测弧线，观察偏移是否减小且直线/停车振荡没有增加。

## 2026-08-01：Task3连续位置PD对照实验

- 修改原因：用户提供的参考视频表现为全程小角度连续修正，并要求在保存当前Git基线后测试该思路；当前Task3仍会进入一次性速度PI助启动，实机出现接近目标后难以停车。
- 最终决策：仅在新鲜`competition_mode=3`下关闭速度PI交接，使位置误差与真实VIS多帧差分速度阻尼从起步到停车连续拥有球环输出；Task2/4/5/6的一次性速度PI助启动保持不变。Task3的`Kp/Ki/Kd=0.60/0/0.55`、`1.5 mm`安静保持带、电机角环、目标、启停、视觉门控及全部机械/电流/raw保护不变。
- 涉及文件：`APP/Gateway/app_gateway.c`、滚球控制README和根README；未修改Task周期、通信协议、BSP、ISR/DMA、底盘前馈或电机启停所有权。
- 验证方式和结果：BallPositionControl与control polarity chain均以`-Wall -Wextra -Werror`主机测试PASS；ARM Debug/Release clean-first完整链接通过，FLASH/RAM分别为`55904/20952 B`和`48244/20928 B`；`git diff --check`通过。
- 已知限制与待确认：按用户要求本轮不连接探针、不烧录、不启动电机，以上结果不构成实物验证。后续Task3需确认纯位置PD能否克服静摩擦、接近目标是否提前收敛以及最终停车误差；若无法起滚，可直接回到基线提交`66c37ab`。

## 2026-08-01：Task3专属提速并收紧到位误差

- 修改原因：Task3首次完整运行后实测可到目标，但用户反馈50 mm旅程太慢且最终误差偏大；原软件将球速限制为30 mm/s，并在3 mm误差内允许安静保持。
- 最终决策：只在新鲜`competition_mode=3`下把连续速度PI上限从`30`提高到`45 mm/s`、加速度限斜率从`180`提高到`300 mm/s²`，并把速度环交接与安静保持位置误差同时收紧到`1.5 mm`。Task3继续使用2026-07-31原电机角环PID；Task2/4/5/6继续使用`30 mm/s`、`180 mm/s²`和`3 mm`，其他赛题参数不变。运行时参数重复应用为无扰no-op，只有参数组实际变化才清当前旅程并重提。
- 涉及文件：`APP/Algorithm/BallPositionControl/`、`APP/Gateway/app_gateway.c`、`config/core_config.h`、对应测试和README；未修改Task周期、协议、底盘前馈、电机启停、BSP、ISR/DMA或保护边界。
- 验证方式和结果：BallPositionControl与control polarity chain严格主机测试均PASS，`git diff --check`通过；ARM Debug/Release clean-first完整链接通过，FLASH/RAM分别为`55552/20952 B`和`47876/20928 B`。用户确认电机已失能后，锁定F407 CMSIS-DAP写入Debug BIN并从`0x08000000`完整回读`55552 B`，源镜像与回读SHA-256均为`6980B12728F80C63981188410092C9B6D053A8DCC9DF26D00498FD0B1B0371F4`，随后`reset run`。
- 运行检查：复位后主程序/Task/滚球控制故障均为0，Task Ready=1、心跳持续运行；滚球phase为0、命令电流0 A，QD4310在线但命令门和反馈均为失能。
- 已知限制与待确认：构建、回读与停机状态不能替代新参数的动态实物验证；下一次Task3只需复测`0 -> +50 -> -50 mm`时间、最大误差和是否出现过冲/振动。

## 2026-08-01：非Task3连续速度环到位置环锁存交接

- 修改原因：旧逻辑等待100 ms后才启用速度辅助，并在3..11 mm内混合两环；误差噪声可反复重启状态机，表现为脉冲、早刹和停不到。
- 最终决策：每个新目标按最终`target_command`建立一次旅程；误差超过11 mm立即由连续速度PI独占，按剩余制动距离生成目标球速；进入3 mm且球速不超过8 mm/s或越过目标后锁存位置环，直到新目标或测量复位。离散kick保持为0，Task3运行时旁路该交接。
- 涉及文件：`APP/Algorithm/BallPositionControl/`、`APP/Gateway/app_gateway.c`、`config/core_config.h`及滚球控制测试；未修改协议、Task周期、电机启停或保护。
- 验证方式和结果：BallPositionControl与control polarity chain严格主机测试PASS；ARM Debug clean-first链接通过，FLASH/RAM为`51236/20792 B`；`git diff --check`通过。未烧录、未启动电机。
- 待确认：实机需验证远距离速度连续性、近目标单次切环以及任意非零目标停车；Task3应保持专属行为且不进入该速度旅程。

## 2026-08-01：Task3由G3519 KEY直接启停

- 修改原因：用户明确纠正Task3必须由G3519的KEY启动和结束，不再等待F407本地KEY；旧Maix `WAIT_REARM`门控因此被否决。
- 最终决策：新鲜`mode=3`下，G3519第一次短按发送既有`BTG=0 mm`启动事件，F407经新增幂等`APP_Gateway_StartPrimaryMotor()`走原有反馈、转速、raw及使能保护启动；第二次短按发送既有`BST`并走零电流后失能。F407用VST `status_flags.bit14`明确转发远端Task3活动状态；M3继续禁止全部底盘加速度前馈。
- 涉及文件：`APP/Gateway/app_gateway.[ch]`、`Task/task_app.c`、`Device/VisionLink/device_vision_link.[ch]`及定向测试。
- 验证与部署：Debug clean-first链接成功，FLASH/RAM为`51208/20800 B`；烧录前VST为phase 0、命令`0 mA`、速度0且驱动失能。CMSIS-DAP `0416:5051`完成编程和复位，按用户要求未做verify/readback。
- 待确认：实机按一次G3519 KEY应启动Task3，再按一次应安全失能；普通Task和F407本地KEY逻辑未改。

## 2026-08-01：M4/M5/M6统一规划前馈与双端部署

- 修改原因：用户要求M4/M5/M6统一缓起步、匀速中段和缓停车，并让结尾规划前馈连续覆盖到
  G3519的jerk收尾完成，避免只给M4规划量或停车时提前回退。
- 最终决策：F407只在PeerLink在线、CAR年龄不超过50 ms、`competition_mode=4/5/6`且
  `plan_valid=1`时采用G3519规划加速度；正/零规划沿用`0.20`增益，负规划沿用`0.25`停车
  增益，总补偿仍受`±0.60°`限制。M3继续完全禁用底盘加速度前馈；M2、旧八字段、过期或
  STOPPED后的无效规划继续使用本地BMI088，融合开关及现有保护不变。
- 涉及文件：`Task/task_app.c`、`config/core_config.h`、PeerLink/协议/根README和本记录。
  未增加协议字段、Task、状态机、动态内存，也未修改1 ms周期、BSP、ISR/DMA或杆球PID。
- 验证方式和结果：PeerLink十字段与ChassisAccelerationFusion主机测试以
  `-Wall -Wextra -Wpedantic -Werror`运行PASS；Debug/Release clean-first完整链接零警告，
  FLASH/RAM分别为`51032/20800 B`和`44156/20784 B`；`git diff --check`通过。
- 部署结果：锁定`VID:PID 0416:5051`并识别Cortex-M4、DEV_ID尾码`0x413`及512 KiB Flash，
  写入Debug `51032 B`，OpenOCD verify及从`0x08000000`全长回读均通过；源BIN与回读
  SHA-256均为`6ADF10AA41D0DC2147F56E010B64E6BA9FD9FBF92C27EC2FB27EDD0BE599C7E8`，
  随后`reset run`。复位后DBG持续为phase 0、命令0 mA、速度0 rpm且电机失能；未按KEY。
- 已知限制与待确认：尚未做M4/M5/M6动态实车验收。下一轮VOFA应确认中段规划加速度为0、
  结尾负规划和补偿角连续回零、参考速度到0后轮速闭环没有提前释放，并核对球位与杆角限幅。

## 2026-08-01：Task3独立恢复07-31 08:51原始PID并放宽回零完成死区

- 修改原因：此前误把Task3参数写入共享`GATEWAY_LEVEL_*`，连带改变了其他赛题；用户明确纠正为只调Task3，并要求回机械零点在一定范围内即可满足启动要求。
- 最终决策：只有新鲜`competition_mode=3`选择提交`7498b0d`（2026-07-31 08:51:36）的`Kp/Ki/Kd=0.014/0.0010/0.0010`、承重前馈`-0.072 A`、死区`0.20°`和积分限幅`0.010 A`。Task2/4/5/6恢复原参数`0.014/0.0020/0.0010`、`-0.090 A`、`0.05°`和`0.020 A`。参数组真正切换时只清积分，重复应用同组参数不扰动控制器；PID公式不变，不增加滤波或电流限斜率。
- 回零门：KEY启动后仍先回固定机械零点；相对零点`±1.5°`且`|speed|<=15 rpm`连续满足300 ms即进入下一phase，任何一项越界都会重新计时。
- 范围：保留Task4/5/6加速度规划、停车位置、板间CAR、底盘融合和VOFA改动；不改变Task周期、协议、BSP、保护或滚球外环参数。此前全局替换参数以及Task3角度/转速滤波、`1.0 A/s`最终电流限斜率均为已否决方向。
- 涉及文件：`APP/Algorithm/LevelCurrentControl/`、`APP/Gateway/app_gateway.c`、`config/core_config.h`、LevelCurrentControl测试、README和协作记录。
- 验证方式和结果：MSYS2 GCC 16.1以`-Wall -Wextra -Wpedantic -Werror`运行`level_current_control`和`control_polarity_chain`均PASS；ARM Debug/Release clean-first完整链接零警告，FLASH/RAM分别为`51032/20800 B`和`44156/20784 B`；`git diff --check`通过。烧录前DBG确认phase 0、命令0 mA、速度0 rpm、电机在线但失能且Task fault=0；锁定CMSIS-DAP `VID:PID 0416:5051`写入Debug BIN（SHA-256 `6ADF10AA41D0DC2147F56E010B64E6BA9FD9FBF92C27EC2FB27EDD0BE599C7E8`），OpenOCD完整校验`Verified OK`并`reset run`。复位后DBG序号重新起步且持续保持相同安全停止状态，未按KEY。
- 已知限制与待确认：软件测试和停止态检查不能证明Task3动态振动已经消失；需按KEY短测并记录CH25～CH29，确认收敛、转速峰值和A→B→A时间。

## 2026-08-01：Task3专属最终电流限斜率

- 结论：Task3保留原始角度P/I反馈、`Kp/Ki/Kd=0.014/0.0020/0.0010`、`-0.090 A`承重前馈及`20 ms`转速D输入低通，新增`1.0 A/s`最终电流限斜率；其他赛题完全旁路。
- 修改原因：当前`vofa+.csv`的2613个有效phase 2样本中，目标raw基本不动，但命令电流在约`0`与`-95 mA`之间跳变，相邻变化不小于70 mA的样本有1584个，转速P95绝对值约11.29 rpm。主要激励来自承重前馈随误差换向快速释放，不应继续滤波角度或提高P/I。
- 最终决策：限斜率作用于极性和总限幅后的正常控制电流；1 ms名义控制周期下每个新反馈最多变化1 mA。Start/Stop将历史归零，Task3发车从0 A平滑进入承重电流；raw越界恢复、机械边界和停机零电流保持原快速路径。
- 涉及文件：LevelCurrentControl算法/API/README、Gateway接入、`config/core_config.h`、LevelCurrentControl测试、根README和协作记录。
- 验证方式和结果：MSYS2 GCC 16.1严格主机测试`level_current_control`与`control_polarity_chain`均PASS；ARM Debug/Release clean-first完整链接零警告，FLASH/RAM分别为`50908/20824 B`和`44128/20808 B`；`git diff --check`通过。按20 ms旧日志离线重放后电流标准差由44.74 mA降至约19.40 mA，70 mA以上相邻跳变归零，该结果仅为软件回放。烧录前DBG为phase 0、命令0 mA、速度0且电机失能；使用`VID:PID 0416:5051`写入50908 B Debug BIN（SHA-256 `F952A51D7EA21DE141C9B405FDBAC5CDC5A9881C8DC584CF88A96336E1AF51D8`），OpenOCD报告`Verified OK`并`reset run`。复位后连续93帧DBG最终仍为phase 0、命令0 mA、速度0、电机在线但失能且Task fault未置位；未按KEY。
- 已知限制：旧CSV采集于本次限斜率之前，烧录后的停止态不能证明实机振动已经消失。
- 待确认事项：烧录后只在新鲜`mode=3`下短测，重新记录CH25～CH29；重点确认相邻电流变化、转速P95、A→B→A时间和球最大误差，再决定是否把`1.0 A/s`调整为`0.8`或`1.5 A/s`。

## 2026-08-01：Task3滤波修正为仅转速D输入

- 结论：用户实车否决“角度和转速同时低通”；角度P/I反馈与`Kp/Ki/Kd=0.014/0.0020/0.0010`保持原样，Task3只对QD4310编码器转速D输入使用`20 ms`一阶低通。
- 修改原因：上一版虽然没有改变PID增益，但角度反馈的20 ms延迟改变了内环相位裕量，用户立即观察到PID振荡并要求角度PID不变。
- 最终决策：绝对角度、raw恢复、机械边界及VST遥测全部使用原始反馈；仅新鲜`mode=3`时过滤转速，其他赛题完全旁路，首帧转速无扰初始化。旧的角度反馈滤波方向已被否决，不得再次采用。
- 涉及文件：LevelCurrentControl算法、Gateway接入、`config/core_config.h`、测试、README和本记录。
- 验证与部署：速度滤波/原始角度路径/旁路主机测试PASS；ARM Debug clean-first完整链接通过，FLASH/RAM为`50684/20816 B`。烧录前VST确认`command=0`、`speed=0`、`raw=58837`且电机失能；CMSIS-DAP `0416:5051`完成program与reset，未做verify/readback、未启动电机。
- 已知限制：用户报告后的5 s样本中电机已经失能，未记录到振荡运动段；同段视觉仍出现`-122..+124 mm`错误跳点，后续动态判断必须把视觉误检与内环振荡分开。

## 2026-08-01：Task3专属QD4310编码器一阶低通并烧录

- 结论：Task3单独启用QD4310角度与转速一阶低通；Debug固件已烧录并复位，未启动电机。
- 修改原因：用户指出Task3运行时杆电机抖动，要求读取画面和编码器，并明确要求Task3特殊处理。现场8 s VST采样确认当时电机失能、`command=0 mA`、`speed=0 rpm`、编码器稳定在约`58793 raw`；画面为`M3:FAILED:OFFLINE`，因此该静态样本不能量化运行时抖动。
- 最终决策：滤波时间常数为`20 ms`，约`8 Hz`截止，同时平滑绝对角度与编码器转速。仅新鲜`competition_mode=3`时使能，Task2/4/5/6和mode离线时保持旧的直通行为；切换时以首帧真实反馈无扰初始化，并正确处理0/360度回绕。绝对raw恢复、机械边界和VST遥测始终使用未滤波反馈。M3继续禁止全部底盘加速度前馈。
- 涉及文件：`APP/Algorithm/LevelCurrentControl/algorithm_level_current_control.[ch]`及README、`APP/Gateway/app_gateway.[ch]`、`Task/task_app.c`、`config/core_config.h`、LevelCurrentControl测试和本记录。
- 验证方式和结果：LevelCurrentControl滤波/回绕/旁路主机测试PASS，control polarity chain测试PASS，`git diff --check`无空白错误；ARM Debug clean-first完整链接通过，FLASH/RAM为`50860/20816 B`。CMSIS-DAP `0416:5051`识别Cortex-M4和512 KiB Flash，完成program与reset；按用户要求未做verify/readback。
- 已知限制：烧录前采样处于电机失能和M3离线状态，尚未获得Task3运动段编码器噪声谱，`20 ms`是保守首轮值。
- 待确认事项：3519需持续发送新鲜`mode=3`并让Maix退出`FAILED:OFFLINE`后再按KEY；首轮动态采样重点检查raw、rpm、电流换向和A→B→A运动时间，再决定是否把时间常数调整为10或30 ms。

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

## 2026-07-29：水平目标由+0.13°再次增加到+0.26°并烧录

- 修改原因：用户完成首轮偏置后要求再增加一点角度。本轮沿用上次约0.13°的步长。
- 最终决策：只将 `GATEWAY_LEVEL_TARGET_DEG` 从+0.13°调整为+0.26°，约对应固定零点
  55183之上的raw=55230；固定零点、KP=0.015、KI=0.0010、KD=0.0010、限幅和保护均
  保持不变。
- 执行边界：仅修改现有目标参数和说明文档，不改变算法、Task、Device、通信、状态机、
  引脚或保护逻辑。
- 安全状态：烧录前实测闭环active=0、命令门和反馈使能均为0、QD4310在线、电流/转速
  为0、主程序/Task故障0；确认失能后才执行烧录。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。锁定F407探针0416:5051后，D盘xPack OpenOCD编程、校验和复位成功；
  Debug BIN SHA-256为
  `72FAF0D773E54AB4E1EE6626A5CA12D5839E6D19717F72FFDDC15D3D003B631A`。
- 运行检查：主程序/Task故障0、心跳3145；实际装载目标0.25999999°、固定零点
  5.2906847 rad，KP/KI/KD保持0.015/0.0010/0.0010；active=0、命令门=0，QD4310
  在线、反馈失能、电流/转速0。
- 已知限制：尚未按KEY执行+0.26°目标的运动闭环，角度精度和稳定性仍待实物确认。
- 待确认：按F407 KEY启动，观察VOFA CH20是否稳定在约55230附近；振动或方向异常时再按
  一次KEY执行 `0 A + Disable`。

## 2026-07-29：取半差count向正方向追加并烧录

- 修改原因：用户要求利用刚采集的位置数据与当前目标的差异，取一半count向正方向追加，
  并明确未经同意禁止任何Git操作。
- 最终决策：实测尾段约55045与上一目标55230相差185 counts，取一半92.5 counts加到
  55230，得到目标raw约55323；相对固定零点55183为
  `GATEWAY_LEVEL_TARGET_DEG=+0.77°`。固定零点、P/I/D、限幅和保护不变。曾误解为设置
  绝对均值-0.25°，已在电机始终失能时立即覆盖，禁止再次采用该错误方向。
- 执行边界：仅修改目标参数和本地说明文档；不修改算法、Task、Device、通信、状态机、
  引脚或保护逻辑；本轮未执行任何Git命令。
- 安全状态：烧录前实测闭环active=0、命令门和反馈使能均为0、QD4310在线、电流/转速
  为0、主程序/Task故障0。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。锁定F407探针0416:5051后，D盘xPack OpenOCD编程、校验和复位成功；
  最终Debug BIN SHA-256为
  `1C9FDC25CCDFA4170A1D11D8304A0EA6D9C1E2B54882EF1C7F8C3DD30EDE99A5`。
- 运行检查：主程序/Task故障0、心跳2137；目标实际装载为+0.769999981°，固定零点
  5.2906847 rad，KP/KI/KD保持0.015/0.0010/0.0010；active=0、命令门=0，QD4310
  在线、反馈失能，电流/转速0。
- 已知限制：新目标尚未按KEY执行运动闭环，最终稳定CH20和机械角度仍待实物确认。
- 待确认：按F407 KEY启动，观察CH20是否稳定在约55323；异常时再按一次KEY失能。

## 2026-07-29：读取工程内VOFA并再次追加半差count

- 修改原因：用户要求读取工程文件夹内最新VOFA并再增加一点角度，且继续禁止未经同意的
  Git操作。
- 数据证据：`F407-Gateway/vofa+.csv` 共1166行；有效使能在线帧891行，末50帧CH20
  全部为56191、转速均值0、电流均值约-0.06812 A。上一目标约55323，二者相差868
  counts。
- 最终决策：沿用半差法，取434 counts向正方向追加，目标raw约55757；相对固定零点
  55183设置 `GATEWAY_LEVEL_TARGET_DEG=+3.153°`。固定零点、P/I/D、限幅和保护不变。
- 执行边界：仅修改目标参数和本地说明文档；不修改算法、Task、Device、通信、状态机、
  引脚或保护逻辑；本轮未执行任何Git命令。
- 安全状态：首次检查时闭环仍active、电机使能，未烧录；用户要求烧录后再次检查确认
  active=0、命令门/反馈使能均为0、转速0，才执行烧录。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。锁定F407探针0416:5051后，D盘xPack OpenOCD编程、校验和复位成功；
  Debug BIN SHA-256为
  `36E32360CEC0CAD63696836F0B082793462D61CC2293B902E728162183CEB3FA`。
- 运行检查：主程序/Task故障0、心跳2701；目标实际装载3.15300012°、固定零点
  5.2906847 rad，KP/KI/KD保持0.015/0.0010/0.0010；active=0、命令门=0，QD4310
  在线、反馈失能，转速0、反馈电流约-0.000305 A。
- 已知限制：新目标尚未按KEY执行运动闭环，最终稳定CH20和机械角度仍待实物确认。
- 待确认：按F407 KEY启动，观察CH20是否稳定在约55757；异常时再按一次KEY失能。

## 2026-07-29：目标明显提高到最新VOFA稳定位置

- 修改原因：用户要求在+3.153°基础上再高且提高幅度更明显，并继续禁止未经同意的Git
  操作。
- 最终决策：在上一目标raw约55757上再次增加434 counts，目标取工程内最新VOFA末50帧
  稳定值56191；相对固定零点55183设置 `GATEWAY_LEVEL_TARGET_DEG=+5.537°`。固定零点、
  P/I/D、限幅和保护不变。
- 执行边界：仅修改目标参数和本地说明文档；不修改算法、Task、Device、通信、状态机、
  引脚或保护逻辑；本轮未执行任何Git命令。
- 安全状态：烧录前实测active=0、命令门/反馈使能均为0、QD4310在线、转速0、主程序/
  Task故障0。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。锁定F407探针0416:5051后，D盘xPack OpenOCD编程、校验和复位成功；
  Debug BIN SHA-256为
  `05319E81DB3937FB71F074081A9E05EDDE51A0DAE70265D51765F41BCE24BED0`。
- 运行检查：主程序/Task故障0、心跳2072；目标实际装载5.53700018°、固定零点
  5.2906847 rad，KP/KI/KD保持0.015/0.0010/0.0010；active=0、命令门=0，QD4310
  在线、反馈失能，转速0、反馈电流约-0.000610 A。
- 已知限制：新目标尚未按KEY执行运动闭环，最终稳定CH20和机械角度仍待实物确认。
- 待确认：按F407 KEY启动，观察CH20是否稳定在约56191；异常时再按一次KEY失能。

## 2026-07-29：按实机方向纠正目标count

- 修改原因：用户实机确认机械高度与编码器raw方向相反；此前为提高机械高度而增加raw的
  方向错误。
- 最终决策：以上一正确比较基准raw=55757为中心，将原本错误增加的434 counts改为减小
  434 counts，目标raw约55323；相对固定零点55183设置
  `GATEWAY_LEVEL_TARGET_DEG=+0.77°`。`+5.537°/raw=56191`方向已否决，禁止再次用于
  “提高机械高度”。固定零点、P/I/D、限幅和保护不变。
- 执行边界：仅修改目标参数和本地说明文档；不修改算法、Task、Device、通信、状态机、
  引脚或保护逻辑；本轮未执行任何Git命令。
- 安全状态：烧录前实测active=0、命令门/反馈使能均为0、QD4310在线、转速0、主程序/
  Task故障0。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。锁定F407探针0416:5051后，D盘xPack OpenOCD编程、校验和复位成功；
  Debug BIN SHA-256为
  `1C9FDC25CCDFA4170A1D11D8304A0EA6D9C1E2B54882EF1C7F8C3DD30EDE99A5`。
- 运行检查：主程序/Task故障0、心跳2287；目标实际装载0.769999981°、固定零点
  5.2906847 rad，KP/KI/KD保持0.015/0.0010/0.0010；active=0、命令门=0，QD4310
  在线、反馈失能，转速0、反馈电流约-0.000305 A。
- 已知限制：新目标尚未按KEY执行运动闭环，机械高度是否达到期望仍待实物确认。
- 待确认：按F407 KEY启动；若仍需提高机械高度，后续只向更小raw微调，不再增加raw。

## 2026-07-29：沿正确方向继续减小434 counts

- 修改原因：用户确认方向后要求继续减小目标raw。
- 最终决策：沿用上一轮步长，在raw=55323基础上再减小434 counts，目标raw约54889；
  相对固定零点55183设置 `GATEWAY_LEVEL_TARGET_DEG=-1.615°`。固定零点、P/I/D、限幅和
  保护不变。
- 执行边界：仅修改目标参数和本地说明文档；不修改算法、Task、Device、通信、状态机、
  引脚或保护逻辑；本轮未执行任何Git命令。
- 安全状态：烧录前实测active=0、命令门/反馈使能均为0、QD4310在线、转速0、主程序/
  Task故障0。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。锁定F407探针0416:5051后，D盘xPack OpenOCD编程、校验和复位成功；
  Debug BIN SHA-256为
  `568FD50AD0C2A780F32BEF773D1D96768A2750583D805659670B3716A65EA710`。
- 运行检查：主程序/Task故障0、心跳2336；目标实际装载-1.61500001°、固定零点
  5.2906847 rad，KP/KI/KD保持0.015/0.0010/0.0010；active=0、命令门=0，QD4310
  在线、反馈失能，转速0、反馈电流约-0.000610 A。
- 已知限制：新目标尚未按KEY执行运动闭环，机械高度是否达到期望仍待实物确认。
- 待确认：按F407 KEY启动，观察CH20是否稳定在约54889；若仍需提高则继续减小raw。

## 2026-07-29：以两倍步长继续减小目标raw

- 修改原因：用户要求继续减小，并要求本轮减小幅度更大。
- 最终决策：采用上一轮434 counts的两倍步长，在raw=54889基础上减小868 counts，目标
  raw约54021；相对固定零点55183设置 `GATEWAY_LEVEL_TARGET_DEG=-6.383°`。固定零点、
  P/I/D、限幅和保护不变。
- 执行边界：仅修改目标参数和本地说明文档；不修改算法、Task、Device、通信、状态机、
  引脚或保护逻辑；本轮未执行任何Git命令。
- 安全状态：烧录前实测active=0、命令门/反馈使能均为0、电流/转速0、主程序/Task故障0；
  QD4310反馈当时已离线。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。锁定F407探针0416:5051后，D盘xPack OpenOCD编程、校验和复位成功；
  Debug BIN SHA-256为
  `F2B285D67F6561363401CD4F7B884768529437166971C1F5B851A36E7DA758D3`。
- 运行检查：主程序/Task故障0、心跳2875；目标实际装载-6.3829999°、固定零点
  5.2906847 rad，KP/KI/KD保持0.015/0.0010/0.0010；active=0、命令门=0、电机反馈
  失能，电流/转速0。QD4310反馈仍离线。
- 已知限制：反馈离线，尚不能执行新目标的运动闭环或验证机械高度。
- 待确认：恢复QD4310供电/CAN在线后按F407 KEY启动，观察CH20是否稳定在约54021。

## 2026-07-29：以最新稳定角重新标定机械零点

- 修改原因：最新VOFA在错误偏置目标测试中停于55297，用户确认该机械位置恰好在水平
  零点附近；因此该数据应作为零点标定证据，而不是把54021当作水平目标。
- 数据证据：最后连续使能段10.7 s；末100帧CH20均为55297、标准差0、转速0。该值比
  旧零点55183高114 counts，约0.626°。此前目标54021与该位置相差1276 counts，不能用
  该偏置目标的稳态误差直接判断零点附近PID合格。
- 最终决策：固定零点更新为 `GATEWAY_LEVEL_ZERO_RAW=55297`，水平目标恢复为
  `GATEWAY_LEVEL_TARGET_DEG=0°`。KP/KI/KD、死区、积分区、限幅和保护不变。
- 执行边界：仅修改零点、目标和本地说明文档；不修改算法、Task、Device、通信、状态机、
  引脚或保护逻辑；本轮尚未执行Git，需扰动回零测试合格后再按用户授权上传并备注
  “零点标定”。
- 安全状态：烧录前实测active=0、命令门/反馈使能均为0、QD4310在线、转速0、主程序/
  Task故障0；KEY历史命令错误计数1，不影响本次安全状态。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。锁定F407探针0416:5051后，D盘xPack OpenOCD编程、校验和复位成功；
  Debug BIN SHA-256为
  `A99CF3C1BF4AB4FF37D91E229C27292E40F5FB8C62DA6D72E0AEF0D5B1B5A919`。
- 运行检查：主程序/Task故障0、心跳2767；目标实际装载0°、零点5.30161428 rad，
  KP/KI/KD保持0.015/0.0010/0.0010；active=0、命令门=0，QD4310在线、反馈失能，
  转速0、反馈电流约-0.000610 A。
- 已知限制：尚未用新零点执行扰动回零测试，不能据此宣告角度PID合格，也不能触发Git
  上传条件。
- 待确认：按F407 KEY启动，在零点两侧分别轻推后释放；采集最新VOFA，确认能回到55297
  附近、无持续振荡且稳态误差不超过约36 counts（0.2°）。

## 2026-07-29：读取新VOFA后小步减小100 counts

- 修改原因：用户要求读取新数据，并在重新标定零点基础上再减小一点目标。
- 数据证据：最新连续使能段71帧、约1.42 s；末30帧CH20均为56167、转速0、电流约
  -0.0684 A。相对固定零点55297仍高870 counts，但采样时间较短，因此本轮不采用半差
  大步调整。
- 最终决策：固定零点保持55297，只将目标小步减小100 counts到约55197，对应
  `GATEWAY_LEVEL_TARGET_DEG=-0.55°`。P/I/D、死区、积分区、限幅和保护不变。
- 执行边界：仅修改目标参数和本地说明文档；不修改算法、Task、Device、通信、状态机、
  引脚或保护逻辑；本轮未执行任何Git命令。
- 安全状态：烧录前实测active=0、命令门/反馈使能均为0、QD4310在线、转速0、主程序/
  Task故障0。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。锁定F407探针0416:5051后，D盘xPack OpenOCD编程、校验和复位成功；
  Debug BIN SHA-256为
  `9C1879D87D3B9DEA7653CA760DEAD7AB18DE4329FB2C1AC3481C8E34C41F18C5`。
- 运行检查：主程序/Task故障0、心跳2568；目标实际装载-0.550000012°、零点
  5.30161428 rad，KP/KI/KD保持0.015/0.0010/0.0010；active=0、命令门=0，QD4310
  在线、反馈失能，转速0、反馈电流约-0.000916 A。
- 已知限制：尚未用-0.55°目标执行运动闭环；新数据段较短，不能据此判断PID最终合格。
- 待确认：按F407 KEY启动并采集至少5 s稳定数据，观察CH20是否向55197靠近且无持续
  振荡。

## 2026-07-29：沿用小步长再减小100 counts并烧录

- 修改原因：用户在上一轮目标基础上要求继续减小。
- 最终决策：沿用上一轮约100 counts的小步长；固定零点保持55297，目标累计减小约
  200 counts到55097，对应 `GATEWAY_LEVEL_TARGET_DEG=-1.10°`。P/I/D、死区、积分区、
  限幅和保护不变。
- 执行边界：仅修改目标参数和本地说明文档；不修改算法、Task、Device、通信、状态机、
  引脚或保护逻辑；本轮未执行任何Git命令。
- 安全状态：烧录前实测active=0、命令门/反馈使能均为0、QD4310在线、转速0、反馈电流
  约-0.000305 A，主程序/Task故障0。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。通过VID:PID `0416:5051`锁定F407探针，D盘xPack OpenOCD完成编程、
  逐段校验和复位；Debug BIN SHA-256为
  `BF2A951BD9AAA4564137D0A907387C7104512EC1628A02F08BAA783AD8395375`。
- 运行检查：主程序/Task故障0、心跳1186；目标实际装载-1.10000002°、零点
  5.30161428 rad，KP/KI/KD保持0.015/0.0010/0.0010；active=0、命令门=0，QD4310
  在线、反馈失能，转速0、反馈电流约-0.000610 A。
- 已知限制：尚未按KEY执行-1.10°目标的运动闭环，不能据此判断最终位置或PID合格。
- 待确认：支撑机构后按一次F407 KEY启动，至少采集5 s VOFA；观察CH20是否向约55097
  靠近且无持续振荡。异常时再按一次KEY执行 `0 A + Disable`。

## 2026-07-29：读取新VOFA后再减小50 counts并烧录

- 修改原因：用户要求读取最新数据后再小幅减小目标。
- 数据证据：工程内 `vofa+.csv` 共2442帧，包含3段连续使能数据；最后一段398帧、约
  7.96 s。其末100帧raw均值56002.78、标准差0.63、范围56001..56003；转速均值
  0.0003 rpm、范围0..0.03 rpm，电流均值-0.07136 A。该段已静止且未见持续振荡。
- 最终决策：不按约906 counts的稳态差值做大步追赶，只把目标再减小50 counts；固定
  零点保持55297，目标累计减小约250 counts到55047，对应
  `GATEWAY_LEVEL_TARGET_DEG=-1.375°`。P/I/D、死区、积分区、限幅和保护不变。
- 涉及文件：`config/core_config.h`、`README.md`、
  `APP/Algorithm/LevelCurrentControl/README.md`及本记录；未执行任何Git命令。
- 安全状态：烧录前实测active=0、命令门/反馈使能均为0、QD4310在线、转速0、反馈
  电流约-0.000305 A，主程序/Task故障0。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。通过VID:PID `0416:5051`锁定F407探针，D盘xPack OpenOCD完成编程、
  逐段校验和复位；Debug BIN SHA-256为
  `1B8044480E089EDBEBC0B6E3D96D32168E8B56FB0AA47A2A49039FF88FA4092C`。
- 运行检查：主程序/Task故障0、心跳1550；目标实际装载-1.375°、零点5.30161428 rad，
  KP/KI/KD保持0.015/0.0010/0.0010；active=0、命令门=0，QD4310在线、反馈失能，
  转速0、反馈电流约-0.000916 A。
- 已知限制：新目标尚未执行运动闭环；旧目标下的静止稳定不能替代新目标的位置验证。
- 待确认：按一次F407 KEY启动并采集至少5 s新VOFA，观察稳定raw是否低于56003且无持续
  振荡；异常时再按一次KEY执行 `0 A + Disable`。

## 2026-07-29：采用最后使能段并额外减小500 counts后烧录

- 修改原因：用户明确要求采用最新连续使能段的角度，并随后要求再明显减小。
- 数据证据：最新 `vofa+.csv` 共3143帧；最后连续使能段701帧、约14.02 s，末100帧
  raw均值53493.9。用户明确覆盖“该段尚未完全停稳”的保守判断，要求直接使用该段。
- 最终决策：以末100帧均值约53494为基准再减小500 counts，最终目标取52994；固定
  零点保持55297，对应 `GATEWAY_LEVEL_TARGET_DEG=-12.651°`。P/I/D、死区、积分区、
  限幅和保护不变。中间目标-9.905°未烧录，已由本最终值覆盖。
- 涉及文件：`config/core_config.h`、`README.md`、
  `APP/Algorithm/LevelCurrentControl/README.md`及本记录；未执行任何Git命令。
- 安全状态：首次准备烧录时实测active=1、命令门/反馈使能均为1，因此未烧录；随后
  实测active=0、命令门/反馈使能均为0、QD4310在线、转速0、电流约-0.000610 A、
  主程序/Task故障0，确认失能后才烧录。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。通过VID:PID `0416:5051`锁定F407探针，D盘xPack OpenOCD完成编程、
  逐段校验和复位；Debug BIN SHA-256为
  `01BBFF7A8387FB91C3D20AF21DA4E5C4D4B7AF7A7FE0090948A06BF80E720D17`。
- 运行检查：主程序/Task故障0、心跳1183；目标实际装载-12.651°、零点5.30161428 rad，
  KP/KI/KD保持0.015/0.0010/0.0010；active=0、命令门=0，QD4310在线、反馈失能，
  转速0、反馈电流约-0.000305 A。
- 已知限制：-12.651°尚未执行运动闭环，烧录和参数回读不能替代机构运动验证。
- 待确认：支撑机构后按一次F407 KEY启动并观察运动；异常时再按一次KEY执行
  `0 A + Disable`。

## 2026-07-29：目标增加100 counts并将KP小幅提高到0.017后烧录

- 修改原因：用户要求角度增大一点，并在修改过程中追加要求小幅提高KP。
- 最终决策：固定零点保持55297，目标从52994增加100 counts到53094，对应
  `GATEWAY_LEVEL_TARGET_DEG=-12.102°`；KP从0.015提高到0.017 A/deg。KI=0.0010、
  KD=0.0010、死区、积分区、限幅和保护不变；此前KP=0.020/0.030振动明显，仍不采用。
- 涉及文件：`config/core_config.h`、`README.md`、
  `APP/Algorithm/LevelCurrentControl/README.md`及本记录；未执行任何Git命令。
- 安全状态：烧录前实测active=0、命令门/反馈使能均为0、QD4310在线、转速0、电流约
  -0.000305 A，主程序/Task故障0。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。通过VID:PID `0416:5051`锁定F407探针，D盘xPack OpenOCD完成编程、
  逐段校验和复位；Debug BIN SHA-256为
  `FAD15103D8086BF8DE4AA0E00864CE83986655DE37FD6EB7591A8BCAB50A5146`。
- 运行检查：主程序/Task故障0、心跳1219；目标实际装载-12.1020002°、零点
  5.30161428 rad，KP/KI/KD实际装载0.017/0.0010/0.0010；active=0、命令门=0，
  QD4310在线、反馈失能，转速0、电流0 A。
- 已知限制：新目标和KP=0.017尚未执行运动闭环，不能据此判断刚度或振动情况。
- 待确认：支撑机构后按一次F407 KEY启动，观察是否仍无明显振动；异常时再按一次KEY
  执行 `0 A + Disable`。

## 2026-07-29：目标继续增加100 counts并烧录

- 修改原因：用户要求在上一目标基础上再增大一点。
- 最终决策：固定零点保持55297，目标从53094继续增加100 counts到53194，对应
  `GATEWAY_LEVEL_TARGET_DEG=-11.552°`。KP/KI/KD保持0.017/0.0010/0.0010，死区、
  积分区、限幅和保护不变。
- 涉及文件：`config/core_config.h`、`README.md`、
  `APP/Algorithm/LevelCurrentControl/README.md`及本记录；未执行任何Git命令。
- 安全状态：烧录前实测active=0、命令门/反馈使能均为0、QD4310在线、转速0、电流0 A，
  主程序/Task故障0。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。通过VID:PID `0416:5051`锁定F407探针，D盘xPack OpenOCD完成编程、
  逐段校验和复位；Debug BIN SHA-256为
  `825310C2F1C5032B9BA9ED5CD779B043A44751A21DF553703E6EB1A12BD77FD7`。
- 运行检查：主程序/Task故障0、心跳1307；目标实际装载-11.552°、零点5.30161428 rad，
  KP/KI/KD实际装载0.017/0.0010/0.0010；active=0、命令门=0，QD4310在线、反馈失能，
  转速0、电流约-0.000305 A。
- 已知限制：新目标尚未执行运动闭环，不能据此判断最终角度。
- 待确认：按一次F407 KEY启动并观察；异常时再按一次KEY执行 `0 A + Disable`。

## 2026-07-29：依据稳定VOFA末段重新标定零点并烧录

- 修改原因：用户要求读取当前数据；若角度稳定，则将其设为0点并提交Git。
- 数据证据：最新 `vofa+.csv` 共1110帧；最后连续使能段169帧、约3.38 s，末100帧
  raw全部为54379，均值54379、标准差0、转速0，电流均值约-0.079 A；该稳定末段
  满足零点标定条件。
- 最终决策：将 `GATEWAY_LEVEL_ZERO_RAW` 从55297更新为54379；用户随后要求再稍微减小，
  因此运行目标取约54329，对应 `GATEWAY_LEVEL_TARGET_DEG=-0.275°`。KP/KI/KD保持
  0.017/0.0010/0.0010，死区、积分区、限幅和保护不变。
- 涉及文件：`config/core_config.h`、`README.md`、
  `APP/Algorithm/LevelCurrentControl/README.md`及本记录；按用户条件授权后执行Git。
- 安全状态：烧录前实测active=0、命令门/反馈使能均为0、QD4310在线、转速0、电流约
  -0.000610 A，主程序/Task故障0。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。通过VID:PID `0416:5051`锁定F407探针，D盘xPack OpenOCD完成编程、
  逐段校验和复位；Debug BIN SHA-256为
  `871D96EFE79ED6056E07517242ECD0049A27519E22241A76F2338CA927F9AF17`。
- 运行检查：零点0°版本主程序/Task故障0、心跳1200；目标实际装载0°、零点5.21360064 rad，
  KP/KI/KD实际装载0.017/0.0010/0.0010；active=0、命令门=0，QD4310在线、反馈失能，
  转速0、电流约-0.000610 A。
- 已知限制：零点稳定性已由末100帧验证；最终-0.275°运行目标尚待重新烧录和运动验证。
- 待确认：重新烧录后按一次F407 KEY观察是否能在约54329附近保持；异常时再按一次KEY执行
  `0 A + Disable`。

## 2026-07-29：目标累计增加300 counts到53494并烧录

- 修改原因：用户先要求比上一轮多增加一点，随后在尚未烧录时再次要求增加一点并明确
  要求烧录。
- 最终决策：固定零点保持55297；从已烧录目标53194累计增加300 counts，最终目标取
  53494，对应 `GATEWAY_LEVEL_TARGET_DEG=-9.904°`。中间目标53394未烧录，已由最终值
  覆盖。KP/KI/KD保持0.017/0.0010/0.0010，死区、积分区、限幅和保护不变。
- 涉及文件：`config/core_config.h`、`README.md`、
  `APP/Algorithm/LevelCurrentControl/README.md`及本记录；未执行任何Git命令。
- 安全状态：中间准备烧录时active=1，因此未烧录；最终烧录前实测active=0、命令门/
  反馈使能均为0、QD4310在线、转速0、电流约-0.000916 A、主程序/Task故障0。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。通过VID:PID `0416:5051`锁定F407探针，D盘xPack OpenOCD完成编程、
  逐段校验和复位；Debug BIN SHA-256为
  `BE592AA900BC661E69A3B6509D0C31AA83FB8120DBF25D9644DE6192F682F45C`。
- 运行检查：主程序/Task故障0、心跳1284；目标实际装载-9.90400028°、零点
  5.30161428 rad，KP/KI/KD实际装载0.017/0.0010/0.0010；active=0、命令门=0，
  QD4310在线、反馈失能，转速0、电流0 A。
- 已知限制：新目标尚未执行运动闭环，不能据此判断最终角度。
- 待确认：按一次F407 KEY启动并观察；异常时再按一次KEY执行 `0 A + Disable`。

## 2026-07-29：零点54379与-0.275°最终目标标定收口

- 修改原因：用户要求依据协作文档完成滚球侧绝对编码器零点标定，并在新零点基础上再
  稍微减小运行目标。
- 最终决策：固定机械零点为54379；运行目标为相对零点-0.275°，约对应raw=54329。
  KP/KI/KD保持0.017/0.0010/0.0010；未修改HB/TEL/ENC/DIS、VOFA通道、小车侧逻辑、
  Task、状态机或保护。
- 数据证据：最新VOFA最后连续使能段末100帧CH20均为54379、标准差0、转速0，电流
  均值约-0.079 A；用户确认采用该稳定角作为机械零点。
- 验证方式和结果：Debug/Release完整链接零警告，FLASH/RAM分别为20772/12344 B、
  17584/12336 B。通过VID:PID `0416:5051`锁定F407探针，D盘xPack OpenOCD完成编程、
  逐段校验和复位；Debug BIN SHA-256为
  `B9FE1CC0420ED8B90141381CE600CADFC53A8312AE56BBCAFB8CACB8EBA3FAD3`。
- 运行检查：主程序/Task故障0、心跳1312；目标实际装载-0.275000006°、零点
  5.21360064 rad，KP/KI/KD实际装载0.017/0.0010/0.0010；active=0、命令门=0，
  QD4310在线、反馈失能，转速0、电流约-0.000305 A。
- 已知限制：机械零点的静态采样已通过；最终-0.275°目标尚未执行扰动回位测试，完整
  `raw -> 杆面实际倾角` 五点映射仍需按协作文档另行实测。
- 待确认：按F407 KEY执行最终目标测试，观察是否在约54329附近稳定且无持续振荡。

## 2026-07-29：F407 KEY五点机构角度标定控制

- 修改原因：用户要求按协作文档完成滚球机构标定控制；每按一次KEY进入下一个固定角度，
  逐点实测后生成电机绝对编码器raw到杆面实际倾角的查表。
- 最终决策：临时启用 `GATEWAY_LEVEL_CALIBRATION_KEY_MODE=1`。第1次KEY发送使能并进入
  -4.395°，第2～5次依次进入-2.197°、0°、+2.197°、+4.395°，第6次执行
  `0 A + Disable`；再次按下可从点1重新开始。五点约对应命令raw
  53579/53979/54379/54779/55179，每次换点清积分，仍只使用QD4310 `CURRENT`模式。
- 涉及文件：`config/core_config.h`、`APP/Algorithm/LevelCurrentControl/`、
  `APP/Gateway/`、`Task/task_app.c`、两份README、算法测试、本记录和桌面
  `COOPERATION.md`；未修改小车侧逻辑或通信协议，未执行Git提交或推送。
- 验证方式和结果：算法严格ARM编译通过；Debug/Release完整链接零警告，FLASH/RAM分别
  为21048/12344 B、17828/12336 B。通过VID:PID `0416:5051`锁定F407探针，D盘xPack
  OpenOCD完成编程、逐段校验和复位；Debug BIN SHA-256为
  `E515AD14ECE4CDEA78D0EEC47DC2317B7D43677873F467578B911011FC447F29`。回读五点数组
  正确，上电后控制未激活、命令门关闭、电机失能且故障计数为0。
- 实测结果：用户按顺序测得左端绝对高度10.0/9.6/9.3/9.2/9.1 cm；以第3点9.3 cm为
  水平高度基准，`delta_h`为+7/+3/0/-1/-2 mm，按杆长250 mm换算得到实际杆面角
  +1.6045/+0.6876/0/-0.2292/-0.4584°。角度随命令raw增加而单调减小。
- 已知限制：工程内 `vofa+.csv` 修改时间仍为2026-07-29 20:58:49，不含本轮五点采样，
  因此实际稳定CH20尚缺；命令raw不能替代闭环后的实际稳定CH20。当前只能确认命令点与
  高度的单调关系，不能完成最终编码器查表或评估反向间隙。
- 待确认：保存包含本轮五点的最新VOFA，或逐点提供实际稳定CH20。收齐后按实际CH20排序
  生成单调查表，再将临时KEY模式改回0并恢复正常一次启动、一次停止。

## 2026-07-29：第一晚最终交付记录

- 修改原因：用户要求以“第一晚最后一舞”为备注上传滚球侧代码，并记录第一晚完成内容。
- 最终完成：F407与G3519的HB/TEL/ENC/DIS链路及21列VOFA数据路径已经建立；F407保留
  QD4310原始绝对编码器值，使用外部角度P/I/转速阻尼D计算电流，始终只命令QD4310
  `CURRENT`模式。固定机械零点为54379，正常目标-0.275°，P/I/D为
  0.017/0.0010/0.0010。
- 标定功能：F407 KEY临时五点模式依次进入-4.395/-2.197/0/+2.197/+4.395°，第6次
  `0 A + Disable`，切点清积分。用户已测得高度10.0/9.6/9.3/9.2/9.1 cm；以第3点
  为水平，杆面角为+1.6045/+0.6876/0/-0.2292/-0.4584°，角度随命令raw增加单调减小。
- 其他边界：TFT初始化保持关闭；没有下载或使用pyOCD；烧录始终使用D盘OpenOCD并锁定
  F407探针VID:PID `0416:5051`；未修改小车侧控制逻辑。`ACC`底盘前馈仅完成协议规划，
  尚未加入双端固件或当前线上协议。
- 验证：五点版Debug/Release完整链接零警告，FLASH/RAM为21048/12344 B和
  17828/12336 B；严格ARM编译通过。Debug固件已烧录、逐段校验并复位，BIN SHA-256为
  `E515AD14ECE4CDEA78D0EEC47DC2317B7D43677873F467578B911011FC447F29`；上电保持失能且
  故障计数为0。双端历史连续62.5 s实测中HB为126/126、TEL为3142帧，通信错误为0。
- 已知限制：工程内VOFA文件不含本轮五点实际稳定CH20，因此最终编码器查表、反向间隙、
  最终目标扰动回位和21列最新COM5采集仍待第二阶段完成。
- 提交边界：本次仅提交明确列出的源码、测试和工程文档；排除 `vofa+.csv`、
  `config/local_secrets.h`、构建目录和桌面外部 `COOPERATION.md`。

## 2026-07-30：滚球机构模型与五点标定精度核验

- 修改原因：用户确认五点高度已经采样，并补充机构尺寸草图，要求用模型核验当前标定是否
  精准。草图标注杆面右侧固定转轴至左端连接点250 mm、左侧构件75 mm、横向构件74 mm、
  电机圆半径12.5 mm和右侧固定转轴离底板86 mm。
- 临时模型：按当前最可能的标准四杆解释，将250 mm作为输出杆、75 mm作为连杆、74 mm
  作为绕电机轴转动的输入摇臂；水平参考位取75 mm连杆近似竖直、74 mm摇臂近似水平，使用
  闭环约束 `|P(theta)-Q(q)|=75 mm` 数值求解。该拓扑只是计算假设，尚未写入产品配置或源码。
- 模型结果：命令raw为53579/53979/54379/54779/55179时，理论杆面角依次约为
  +1.2996/+0.6503/0/-0.6503/-1.2997°；对应五点实测为
  +1.6045/+0.6876/0/-0.2292/-0.4584°。模型与实测角度RMS偏差约0.443°、最大偏差约
  0.841°；换算到左端高度的RMS偏差约1.93 mm、最大偏差约3.67 mm。负raw侧前两点较接近，
  正raw侧偏差明显，不能把电机角直接当杆面角或用单一固定传动比。
- 精度边界：本轮高度只记录到0.1 cm，单个读数的舍入边界约±0.5 mm，即杆面角约
  ±0.115°；与同样舍入的9.3 cm零点相减时最坏约±1 mm，即约±0.229°。正raw侧最大
  3.67 mm偏差已超过该读数边界，更可能包含闭环跟踪偏差、机械间隙/回差、静摩擦或模型
  拓扑和初始夹角误差。
- 软件尺度复核：QD4310协议换算为`0..65535 -> 0..360°`，单count约0.005493°电机角；
  五个目标换算到整数raw的舍入角误差小于0.0005°，不是当前主要误差源。现有角度外环死区
  0.20°约合36.4个协议count，且五点实际稳定CH20未保存，因此不能由命令raw反推出各点
  实际电机位置。
- 最终决策：现有五点已证明命令到杆面角的单调关系，可作为首版同方向分段线性查表；否决
  单一`K_raw_per_deg`和用二次多项式向标定区间外推。以最小已测倾角0.458°、实心球纯滚动
  理想模型估算，从+5 cm到-5 cm的10 cm单程换位最短bang-bang时间约2.7 s，仅说明执行
  角度范围具备满足5 s指标的理论余量，不代表实物闭环已经通过。
- 涉及文件：仅更新本记录；未修改控制源码、配置、协议、Task、状态机或保护，未构建、烧录
  或执行新的机构运动。
- 已知限制：草图仍未明确74 mm横杆右端连接电机轴圆心还是半径12.5 mm的偏心销，也未确认
  75/74 mm构件之间是转动副还是同一刚性折弯件；电机轴相对杆面转轴的精确水平/竖直偏置、
  `q0`、`theta0`、装配分支和反向间隙仍缺。不同拓扑会得到不同四杆方程和理论角度。
- 待确认：先用实物照片或连接说明确认上述拓扑，再保存五点实际稳定CH20并反向扫一遍；按
  实际CH20建立单调查表、量化回差后，才能冻结最终机构模型和标定精度。

## 2026-07-30：IMU与QD4310向上双遍标定及模型交叉确认

- 修改原因：用户安装IMU后授权使用现有无刷电流闭环自动打表，并纠正机构从地面起步时必须
  向上运动；要求同时依据候选四杆模型和实测数据确认精度，合格后记录并允许进入Git。
- 方向与范围：两遍有效运动均为机构向上、QD4310 raw递减、IMU `CH12`示值递增。落地硬
  止点raw约在58999～59125间漂移，未作为精密节点；本轮有效标定区间为raw=58499～53577。
- 有效上行表如下。表值为稳定窗口`CH12`中位数；第二遍raw=55000窗口在失能回落过程中
  取得，标准差1.265°，判为无效，因此该节点保留第一遍稳定值。

| 目标raw | 实际raw | CH12中位数/° | 标准差/° |
|---:|---:|---:|---:|
| 58500 | 58499 | 20.640 | 0.0129 |
| 58000 | 57999 | 21.160 | 0.0047 |
| 57500 | 57499 | 21.650 | 0.0085 |
| 57000 | 56999 | 22.110 | 0.0073 |
| 56500 | 56499 | 22.570 | 0.0106 |
| 56000 | 55999 | 23.030 | 0.0046 |
| 55500 | 55499 | 23.460 | 0.0103 |
| 55000 | 54999 | 24.000 | 0.0124（第一遍） |
| 54500 | 54499 | 24.470 | 0.0599 |
| 54000 | 53999 | 25.080 | 0.0117 |
| 53579 | 53577 | 25.450 | 0.0072 |

- 重复性与线性诊断：两遍共同稳定节点的最大重复差约0.14°，说明同方向重复性较好；该值
  属于IMU/杆面示值角域，不能与QD4310电机角域的0.20°控制死区直接比较。仅用于诊断的
  全区间直线为`CH12=77.2608875-0.00096783266*raw`，节点MAE=0.038°、RMSE=0.046°、
  最大残差0.087°。按250 mm杆长换算，0.14°和0.087°分别约为执行端0.61 mm和0.38 mm
  高度差，只用于说明本轮相对打表误差量级。等效局部尺度约1033.24 count/杆面示值度，
  杆面示值角/电机角约0.1762；最终控制换算仍采用上述分段线性表，不用该直线外推。
- 模型交叉确认：沿用上一节候选几何`L=250 mm、l=75 mm、r=74 mm`并以raw=54379为
  相对0°，模型在本轮两端预测约-6.566°和+1.303°。把IMU示值在raw=54379处分段插值为
  24.6164°后，本轮相对示值为-3.976°到+0.834°；不缩放模型时RMSE约1.450°、最大偏差
  2.589°，绝对传动比例未通过。最小二乘仅作诊断可得
  `theta_IMU_rel≈0.6042*theta_model+0.0190°`，缩放后RMSE约0.042°、最大残差约0.077°。
  因此模型的方向、单调性和局部曲线形状与实测一致，但单一缩放与直接线性拟合精度相当，
  不能唯一证明74/75/12.5 mm构件的拓扑解释正确，也不能用候选模型替代实测表。
- 旧五点复核：旧高度法使用命令raw而非稳定反馈raw；在同五个目标raw处，与本轮IMU相对
  示值的RMSE约0.399°、最大差约0.773°，其中raw约53579处的差值已超过高度差读数最坏
  约0.229°的舍入边界。旧表只保留为早期方向证据，不再作为最终角度映射输入。
- 最终决策：本轮同方向重复差最大约0.14°、线性诊断最大残差0.087°，数据足以作为当前
  raw到IMU相对示值表并按用户授权提交本记录；否决把该结论升级为绝对杆角标定通过，也
  不据此修改`GATEWAY_LEVEL_ZERO_RAW=54379`。未经反向扫和绝对零偏标定，不向区间外外推。
- 源码收口：临时Enable pending/NOP握手因实机未收到`enabled=1`已否决并完整回退，
  `APP/Gateway/app_gateway.c`恢复到Git HEAD；未修改控制参数、Task、协议或保护。
- 构建与烧录：回退后Debug/Release均执行clean-first完整构建且零警告，FLASH/RAM分别为
  21048/12344 B、
  17828/12336 B；Debug BIN SHA-256为
  `E515AD14ECE4CDEA78D0EEC47DC2317B7D43677873F467578B911011FC447F29`。烧录前连续新鲜
  反馈为online=1、enabled=0、速度0、电流约-1 mA、raw=59127；随后按VID:PID
  `0416:5051`锁定F407探针，OpenOCD完成Debug BIN编程、整镜像校验和复位。
- 最终安全状态：复位后COM3连续新鲜反馈仍为online=1、enabled=0、速度0、电流约
  -1～-2 mA、raw=59119；机构位于落地侧，本轮收尾未重新启动电机。另一次复位后SWD
  快照确认主程序、Task、FreeRTOS、角度控制和PeerLink故障均为0，Task心跳21329，闭环
  active=0、命令门=0、反馈enabled=0且online=1，速度0、电流约-0.61 mA、raw=59109；
  复位恢复运行后COM3新鲜反馈仍为enabled=0、速度0、电流约-1 mA、raw=59099。
- 涉及文件：仅更新`process.md`；保留用户/其他Agent的`COOPERATION.md`改动，不跟踪或提交
  `vofa+.csv`。
- 用户纠正：`CH12=ICM45686 pitch+JY61P pitch`。此前直接写成“beta0未确定”过强，已否决。
  若ICM是车体对世界pitch、JY是杆对车体的相对pitch且两者轴/符号一致，相加可以得到杆对
  世界角；若JY本身也是对世界pitch，相加会重复计入车体倾角。当前源码事实仍是ICM轴映射
  未冻结且JY只tare yaw、未tare pitch，所以尚不能仅凭字段名证明CH12是真水平绝对角。
- 当前相对控制不等待上述判别：在旧机械参考`raw=54379`处把CH12插值为`24.6164°`并相减，
  等效把该点定义为相对杆角0°，常量安装偏置不会进入相对命令。绝对角最小验证仍为“固定
  车体只抬杆”与“锁杆随车体整体倾斜”两步；反向扫同一节点后才能量化回差。

## 2026-07-30：F407滚球闭环、ACC/VIS接入及烧录

- 修改原因：用户要求补齐ACC消费、实测分段查表和球位置/速度闭环，并要求把AgentB与
  AgentA/AgentC的协作边界写入`COOPERATION.md`；随后明确授权烧录。
- 最终决策：KEY标定模式恢复为0。正常10 ms Task依次推进PeerLink和VisionLink，再执行
  `VIS -> x/v -> 位置/速度PD -> 实心球逆模型杆角`、
  `ACC -> 当前/下一规划加速度选择 -> 前馈杆角`、
  `trim+反馈+前馈 -> 上行实测逆表 -> raw -> 电机相对角 -> CURRENT外环`。连续滚球目标
  使用`SetTrackingTargetDeg()`，不再每10 ms清积分；旧`SetTargetDeg()`仅供标定换点。
- 杆角映射：采用2026-07-30的11点向上表，在此前确认的`ZERO_RAW=54379`处减去插值示值
  `24.6164°`，不是另立零点；杆角范围约`-3.9764°..+0.8336°`，区间外目标饱和到端点，
  禁止外推。工作微调`TRIM_RAW=54329`约为电机角`-0.275°`、杆角`+0.061°`。
- 球环初值：`a_cmd=1.2*(x_target-x)-1.8*v`，速度按真实相机`sample_ms`差分并以
  `tau=0.080 s`一阶滤波；实心球使用`theta=-asin(1.4*a_cmd/9807)`。首版对称限幅从
  `0.70°`保守收至`0.60°`，trim后距上行表正端约`0.173°`、约197 raw count；理想10 cm
  bang-bang时间约2.34 s只用于余量检查，不是实物成绩。
- ACC协议：F407接收六字段
  `node_id,sample_ms,a_now,a_next,next_after_ms,valid`，使用独立序号、远端样本前半区规则、
  单槽Latest Value和本地年龄；达到50 ms即失效。HB离线恢复或uptime非回绕倒退时清旧
  ACC会话，允许G3519独立重启后从0恢复。语法范围`±40000 mm/s²`不是执行范围；当前
  `GATEWAY_CHASSIS_FF_GAIN=0`、preview/transport=0，故链路和公式已接通但前馈尚未施加。
- VIS链路：候选USART2为PA2 TX/PA3 RX、115200 8N1、字节中断环；Payload为
  `sample_ms,proc_age_ms,position_mm,flags`，拒绝旧/重复序号、预测/无标定/clamp数据，
  保守年龄达到100 ms即失效。VIS失效只清球反馈回trim，ACC失效只清前馈，均不自动失能。
  相机坐标极性暂取`-1`，实物方向未冻结。
- 涉及文件：新增`RodAngleMap`、`BallPositionControl`、`VisionLink`及测试/README；修改
  `PeerLink`、`LevelCurrentControl`、`Gateway`、`Task`、`BSP`、`CMakeLists.txt`、
  `core_config.h`、根README、板间协议、本记录和`COOPERATION.md`。保留既有协作内容和
  未跟踪`vofa+.csv`，未执行Git提交或推送。
- 验证方式和结果：五组主机测试均以`-Wall -Wextra -Werror`通过；Debug/Release均
  clean-first完整链接零警告，FLASH/RAM分别为`27916/14000 B`、`23804/13984 B`；
  `git diff --check`通过。Debug BIN 27916 B，SHA-256为
  `D89F452A900CBDD2C3B241209E9F56BA8927DDB370383E4999D656B5115DC8EF`。
- 烧录与安全状态：按`VID:PID 0416:5051`锁定CMSIS-DAP并识别Cortex-M4、DEV_ID
  `0x413`、512 KiB Flash。烧录前完整回读27916 B已与本轮Debug BIN哈希一致；匹配新ELF
  后的SWD快照确认主程序/Task/控制/Peer/Vision故障均0、Task Ready=1、闭环active=0、
  命令门/反馈使能=0、速度/电流=0，QD4310反馈当前离线。D盘xPack OpenOCD随后完成编程、
  `Verified OK`和复位；复位后再次SWD回读上述故障与使能状态仍为0，Peer/Vision初始化成功。
  未按KEY、未启动机构。较早一次用旧ELF地址的快照读到栈填充值，已判无效且未作为结论。
- 已知限制：只有上行表，没有下行回差；双IMU组合角能否代表真水平仍取决于两个pitch的
  坐标/参考定义。G3519 ACC发送端和Maix VIS发送端不在本仓库，USART2接线、相机极性、
  真实视觉年龄、工作区双向小阶跃、带球负载和扰动恢复均未验证。旧大阶跃只支持约
  `0.10 s`的10%～90%上升、约6%超调和约`0.30..0.34 s`稳定量级，不能替代小信号验收。
  现有`-220 mm/s²`制动理论需约`+1.285°`前馈，超过表正端，不能声称所有加速度可补偿。
- 待确认：AgentA实现并验证G3519六字段ACC发送；AgentC实现VIS发送并完成九点/端点标定；
  冻结USART2接线和两条极性；从高位向下扫同一11点并做工作点双向小阶跃。完成这些证据后
  才能小步启用`b>0`并整定球环。

## 2026-07-30：500 Hz调度与默认关闭的三状态球Kalman

- 修改原因：用户将F407本地控制频率冻结为500 Hz，并要求先完成基于当前杆球模型、
  VIS和G3519加速度的三状态Kalman代码，但审核逻辑与公式前不启用。
- 最终决策：Gateway Task和电机角外环名义周期由10 ms改为2 ms；FreeRTOS仍为1 kHz
  Tick、单一静态Task、`idle+2`优先级和512 `StackType_t`栈。CAN邮箱等待改为`0 ms`单次
  非阻塞检查，QD4310 Device允许该值；无空邮箱仍沿用现有发送失败停机语义。
- 跨板频率边界：现有115200 8N1的六字段ASCII `ACC`在100 Hz约占单向理论线速49%；
  500 Hz需约280 kbit/s，不可直接提频。本轮保持G3519 100 Hz产生带`sample_ms`的
  `a_now/a_next/next_after_ms`，F407在500 Hz间持Latest Value并按倒计时切换。若后续要求
  500 Hz全新帧，须新建协议版本并升至少460800、建议921600或重设定长二进制帧。
- Kalman模型：状态为`X=[x_mm,v_mm_s,b_mm_s2]`；实心球比例`kappa=5/7`，
  `a_model=-kappa*(g*sin(theta_rod)+a_chassis*cos(theta_rod))`，
  `x+=v*dt+0.5*(a_model+b)*dt^2`、`v+=(a_model+b)*dt`、`b`按随机游走。原始观测只是
  VIS毫米位置；杆角来自QD4310 raw经11点表；底盘输入首版为G3519规划ACC，不将
  其写成IMU实测。`b`无独立传感器，由VIS创新估计慢变模型残差。
- 延迟和可信度：使用64个静态快照，2 ms下覆盖约126 ms。VIS按
  `proc_age_ms+接收/串行年龄`在历史时刻校正，然后按已保存的输入重放；观测落在快照
  之间时拆步。连续白加速度Q离散为`dt^3/3、dt^2/2、dt`；创新用
  `nu^2/S<=25`的5σ门控。至少接受3帧VIS、最新观测低于100 ms且`Pxx/Pvv`达标才
  输出trusted状态。过期、杆角表外或不可信时球反馈归零，不自动Disable。
- 启用边界：`GATEWAY_BALL_KALMAN_ENABLE=0`已冻结；当前固件仍使用VIS真实`sample_ms`
  差分+一阶滤波的球速基线，Kalman不会改变杆角或电机命令。原始VIS/杆角/ACC日志、
  三条极性、固定延迟和Q/R未整定前不得改为1。加速度计噪声问题采用“规划ACC主输入，
  未建模误差由Q/b吸收”；若后续增加IMU/轮速融合实测，新建独立`ACC_MEAS`而不混用规划`ACC`。
- 内存与文件：新增`BallStateEstimator`与主机测试，增加`BallPositionControl_SetEstimatedState()`，
  修改Gateway、QD4310非阻塞配置/测试、`core_config.h`、CMake、README、`COOPERATION.md`和本记录。
  未新增Task、ISR、DMA、动态内存、ACK或重发。估计器历史为静态RAM；Gateway栈深保持不变。
- 验证方式和结果：LevelCurrentControl、RodAngleMap、BallPositionControl、BallStateEstimator、
  VisionLink、PeerLink/ACC和QD4310七组主机测试以`-Wall -Wextra -Werror`通过；覆盖刚体预测、
  整快照/半步延迟重放、首帧年龄传播、极性、创新拒绝、trusted和超时重置。Debug/Release
  clean-first链接零警告，FLASH/RAM为`28996/16968 B`和`24724/16960 B`。独立强制
  `GATEWAY_BALL_KALMAN_ENABLE=1`的Debug链接也通过，为`36324/16968 B`；最终配置已恢复0。
  `git diff --check`通过。最终Debug BIN 28996 B，SHA-256为
  `8FBB7B6F7F07ED9705580EA420386F6D9940BFA215B69C195815F94FFB463D10`。
- 烧录和实物验证：按既定`VID:PID 0416:5051`以200 kHz尝试只读识别时，OpenOCD报
  `unable to find a matching CMSIS-DAP device`；Windows也未枚举到该设备。因此本轮未擦除、未编程、
  未复位目标，也未测`g_TASK_Heartbeat`或QD4310新反馈频率。只能宣称名义500 Hz，不能宣称实测
  Task/电机闭环500 Hz。
- 已知限制：当前ACC仍是规划值且G3519发送端未在本仓库；VIS发送、相机延迟/极性、
  Q/R、实际杆角极性、带球模型残差、Task WCET/抖动、栈高水位、CAN错误计数和QD4310实际响应频率
  均未实物验证。只有上行杆角表且正端余量有限，这些限制不因Kalman插值而消失。
- 待确认：用户先审核三状态公式、坐标和输入语义；Agent A交付100 Hz规划ACC与计数/
  时延证据，Agent C交付原始VIS及延迟/标定日志。重新连接`0416:5051`后才能烧录并分别测
  Task心跳与QD反馈频率；完成离线回放和Q/R整定、用户再次确认后才能将Kalman开关改为1。

## 2026-07-30：F407 / G3519 与 MaixCAM2 通信复测

- 测试边界：用户要求重新测试通信并顺带检查MaixCAM2回路。本轮未烧录、未复位、未按KEY、
  未启动电机；仅按指定DAPLink读取G3519运行计数。F407的HID CMSIS-DAP
  `VID:PID 0416:5051`仍未被Windows枚举，因此没有读取F407侧接收计数。
- 固件身份：G3519 Bring-up Debug BIN与`MSPM0G3519.acc.readback.bin`均为79216 B，
  SHA-256同为`48B3F2DEDA506255FF07147CA516846E8DFF40DA22C583D83A0967424088452B`，
  与当前包含`ACC/ACC_MEAS`发送端的ELF匹配。
- 运行结果：两次快照的G3519本地时间为31300 ms和64510 ms，33.210 s有效运行窗口内，
  HB发送/接收均增加66，ACC发送增加3321（100.00 Hz），ACC_MEAS发送增加1661
  （约50.02 Hz）；HB、ACC、ACC_MEAS发送丢弃和发送错误、PeerLink Step错误、Task/Main/
  Bring-up故障均为0，链路保持online。F407远端uptime持续增长，证明F407正在运行且
  F407 TX -> G3519 RX心跳通路成立；G3519 TX -> F407 RX的HB/ACC/ACC_MEAS仍须F407侧计数确认。
- 调试侵入说明：第二次GDB读取期间G3519相对F407少运行约11.29 s，并新增1次硬件overrun和
  1次parser error；这是在线halt窗口内UART仍收字节造成的测试侵入，不能算正常运行错误。
  启动阶段已有2164字节软件环溢出且在两次快照间未增长；后续不再用长时间halt做频率采样。
- MaixCAM2结果：USB复合设备`VID:PID 2D95:6012`已连接，序列号
  `10AE6D1MQF004P4`；其MTP接口处于Problem Code 19，ADB未枚举设备。当前源码启用
  `/dev/ttyS2`、B0 TX/B1 RX、115200 8N1和最多50 Hz VIS，毫米标定有效位仍关闭；
  `VOFA_UDP_ENABLED=False`，电脑1347端口5 s无包符合配置，不能作为视觉Task未运行的证据。
  因F407探针缺失且没有UART逻辑分析仪抓帧，本轮不能宣称MaixCAM2 -> F407 VIS实收或
  F407 -> MaixCAM2 VST回传通过。
- 下一步：插回`0416:5051`后，在不长停CPU的短快照中读取F407的HB、ACC、ACC_MEAS、VIS、
  VST及UART/DMA错误计数；同时修复MaixCAM2 MTP/ADB访问或接逻辑分析仪，连续抓至少10 s
  VIS/VST，核对50 Hz上限、序号、CRC、年龄和无效标定flags。

## 2026-07-30：插回F407探针后的通信闭环复测

- 探针与构建：Windows已枚举F407 CMSIS-DAP `0416:5051`及COM3，OpenOCD识别
  Cortex-M4、DEV_ID尾码`0x413`和512 KiB Flash。重建时发现三状态估计器`.c`与旧二状态
  `.h`不同步；仅补齐状态bias、协方差`p02/p12/p22`和对应配置字段后，Debug完整链接通过，
  FLASH/RAM为28732/17600 B。BIN SHA-256为
  `FBF3F17C10080BE5D8CEA0776690D353222509BF6D2EF85FEC57127ECB6CF8B1`。
- 烧录：用户再次明确允许烧录；当前Debug ELF已编程、校验成功并复位。测试期间未按KEY、
  未发送电机启动命令，主程序/Task故障均为0。
- F407运行快照：`g_TASK_Heartbeat=48932`、`g_TASK_NowMs=97862`，实测Task约500.0 Hz；
  PeerLink Ready/online，HB TX=196、RX=116，远端会话复位1次，100 Hz业务帧接收3216；
  HB发送drop/error、业务Payload/来源/重复/旧序号/旧样本、Step、UART overflow/overrun/
  framing/parity/DMA错误均为0。单个parser error与调试/远端重启窗口同时出现，不影响online。
- 协议真值修正：15:11后G3519工作树和Debug ELF被后续版本切回七字段`CAR`，当前F407也
  只解析`CAR`，所以3216是CAR接收计数，不是ACC。CAR携带轮速、ICM X/Z比力和Pitch角速度；
  不携带规划`a_now/a_next/next_after_ms`。F407当前`Task_MakeControlInput()`只取VIS，未把CAR
  送入Gateway，因此当前滚球控制没有使用任何底盘加速度；此前ACC/ACC_MEAS发送和融合链
  已被后续并发版本覆盖，不能写成仍在线。
- MaixCAM2回路：VisionLink Ready且故障0，但`vis_rx_count=0`、RX字节缓冲为0，所有UART/
  DMA和解析错误也为0；说明F407 USART2已正常初始化，但相机没有向PA3送达任何字节。
  当前优先检查Maix应用是否实际启动、B0/UART2_TX到PA3/RX是否交叉并共地；在收到首字节前
  无需调整CRC、标定表或Kalman。
- G3519复读限制：切换探针后DAPLink出现USB I/O Error并从Windows有效枚举中消失，未能在
  15:11版本上再读一次G侧计数；F407实收CAR和远端uptime已证明G3519发送端正在运行，但
  F407 HB是否被15:11版本接收仍待DAPLink恢复后的单次短快照确认。

## 2026-07-30：ICM45686纵向加速度读取与预处理

- 修改原因：`CAR`已经稳定回传ICM45686的X/Z比力和零偏校正后的Pitch/Y角速度，但静止时
  X轴仍含车体倾角造成的重力投影，不能直接作为小车纵向线加速度使用。
- 最终决策：新增无MCU/RTOS依赖的`ChassisImuAcceleration`纯算法。只处理年龄不超过50 ms
  的新CAR样本；以`atan2(-fx,fz)`得到加速度倾角，Pitch角速度积分后用0.50 s时间常数互补
  校正，按`a_linear=fx+g*sin(pitch_est)`去除X轴重力投影，再以0.05 s时间常数一阶低通。
  X/Z重力模长偏离`g`超过2500 mm/s^2时停用加速度倾角校正但保留陀螺预测；采样步长超过
  50 ms时仅在重力门通过后重置姿态；输出绝对值限制为40000 mm/s^2。
- 架构和行为边界：算法位于`APP/Algorithm/ChassisImuAcceleration`，Task从PeerLink Latest
  Value取样并填入Gateway的底盘运动测量字段；未新增Task、Topic、ISR、DMA、动态内存、ACK
  或重发。`GATEWAY_CHASSIS_ACCEL_FUSION_ENABLE=0`、`GATEWAY_CHASSIS_FF_GAIN=0`保持不变，
  所以该测量当前不会进入滚球Kalman、杆角目标或电机控制，也不会启动电机。
- 涉及文件：新增`APP/Algorithm/ChassisImuAcceleration/algorithm_chassis_imu_acceleration.[ch]`
  和主机测试；修改`Task/task_app.c`、`config/core_config.h`、`CMakeLists.txt`、`README.md`及
  `COOPERATION.md`。工作树内其余滚球、通信和电机修改属于此前任务，本条不重复认领。
- 实机证据：静止原始CAR快照为`speed=0 mm/s`、`fx=-734 mm/s^2`、`fz=9843 mm/s^2`、
  `pitch_rate=-0.01 deg/s`。处理得到`pitch_acc=4.2647 deg`、`pitch_est=4.2715 deg`、
  `gravity_norm=9870.33 mm/s^2`、瞬时线加速度`-3.54 mm/s^2`、低通输出`-8.40 mm/s^2`；
  更新3471次、重力门控拒绝0、非法样本0。Main/Task fault为0，Task约500 Hz，HB TX/RX
  为97/97、CAR RX为4834，UART/DMA错误为0。该证据说明静止时约`-0.734 m/s^2`主要是
  约4.27 deg倾角重力投影，去除后接近0。
- 验证方式和结果：ChassisImuAcceleration主机测试通过；Debug/Release clean-first完整链接
  零警告，FLASH/RAM分别为`34708/17672 B`和`30180/17656 B`。此前已完成测量版F407编程、
  校验和复位；本次重新构建未再次烧录，测试期间没有发送电机启动命令。
- 已知限制：只有静止单点证据。加速度计无法单独区分持续纵向加速与倾角，0.50 s互补校正会
  使持续加速度呈高通特性；仅用X/Z重力模长，不覆盖侧向动态。ICM安装轴正负、运动噪声、
  50 ms输出滤波的相位延迟、轮速比例和与轮速差分加速度的一致性均未动态验证。
- 待确认：保持电机失能，采集水平静止、正反向手推、起步和制动原始CAR及处理输出，统计
  静止均值/标准差、方向、延迟和饱和/拒绝计数；再依据日志调整时间常数和融合Q/R。未经该
  动态确认，不启用融合或前馈，不把ICM测量冒充具有前瞻性的规划加速度。

## 2026-07-30：KEY回零后启用ICM补偿与QD4310绝对raw限幅

- 修改原因：用户要求实时观察ICM补偿，并要求电机只能在其手动指定的范围内运动；随后明确
  KEY启动必须先进入机械零点，再开启加速度补偿。
- 手动标定证据：滚球闭环未运行，先提交`ENABLE`和`CURRENT 0 A`；提交前驱动反馈已为
  `0.000 A/0 rpm`，两条命令均返回成功。调试链路异常导致F407随后复位，端点正式读取时
  驱动反馈为Disable且电流约0 A。用户指定第一端`raw=59233`、第二端`raw=49325`，范围跨度
  9908 count，按单圈65535 count换算约54.43电机度；范围不跨单圈回绕。
- 最终决策：新增控制phase `STOPPED/ZEROING/ACCELERATION_COMPENSATION`。KEY启动时先把目标
  设为固定机械零点`raw=54379`并发送Enable、0 A；反馈相对零点误差不超过0.50电机度且
  `|speed|<=1 rpm`连续300 ms后才进入ICM补偿。KEY再次按下沿用0 A后Disable退出。
- 补偿行为：补偿phase直接消费Task提交的CAR实测支路，不开启底盘三状态融合；使用已完成的
  Pitch互补、去重力和50 ms低通结果，`theta_ff=-0.20*atan2(a_linear,g)`，杆角另限
  `±0.50 deg`。球Kalman仍为0；VIS有效时原球位置反馈仍可与trim和补偿相加。
- 限幅行为：新增绝对raw边界`49325..59233`，目标raw超界拒绝；反馈严格越界立即0 A并
  Disable；反馈恰在边界时将控制电流换算回未施加极性的电机方向，只阻断继续向外的电流。
  现有11点杆角表`53577..58499`仍是正常目标的更窄边界，现有电流限0.30 A也保持不变。
- 架构和资源：只修改Gateway现有控制编排、Telemetry字段和`core_config.h`，未改2 ms Task、
  优先级、栈、CAN/UART/DMA/ISR、通信协议、动态内存、ACK或重发。Telemetry新增phase、实际
  补偿杆角和raw限幅阻断标志，便于调试观察；无线FB帧格式未改变。
- 验证方式和结果：Debug/Release clean-first完整链接零警告，FLASH/RAM分别为
  `35284/17696 B`和`30756/17672 B`；`git diff --check`通过。Debug BIN为35284 B，SHA-256
  `1C4BBDE6308F882CF7756226F26C1709569FA7E621C11EB54E035A9D111AB918`。OpenOCD以50 kHz和
  CMSIS-DAP quirk完成编程、校验和复位。复位后只读状态：phase=STOPPED、驱动反馈Disable、
  电流0 A、raw=58939、raw limit blocked=0、Task fault=0；本轮没有替用户按KEY。
- 已知限制：新的KEY phase转换、回零方向、300 ms稳定判定、补偿正负和raw边界方向阻断尚未
  在电机运动中验证。ICM新样本仍为100 Hz而非500 Hz；50 ms低通和0.20增益是首次安全值，
  无法保证球完全静止。下行回差、相机位置和动态加速度残差仍未标定。
- 待确认：首次按KEY必须有人旁站；确认先回到raw约54379且没有振动，phase再从1变2。随后
  轻推小车确认加速度正负；若平台朝错误方向、raw接近49325/59233或出现异常电流，立即再次
  按KEY。保存raw最值、phase、处理后加速度、补偿角、blocked和fault后再提高增益。

## 2026-07-30：`HB/CAR`最终协议与双板Debug部署

- 修改原因：用户最终要求板间不发送QD4310/CAN明细，只传F407需要的车速、ICM45686
  X/Z比力和Pitch角速度，并明确授权烧录当前双端Debug镜像。
- 最终决策：活动协议只保留双向500 ms `HB`及G3519到F407的100 Hz七字段`CAR`；
  `TEL/ENC/ACC/ACC_MEAS/DIS`均不发送或解析。F407本地QD4310/CAN逻辑不变，CAR纵向
  预处理已接入，但`GATEWAY_CHASSIS_ACCEL_FUSION_ENABLE=0`和
  `GATEWAY_CHASSIS_FF_GAIN=0`保持不变，不影响滚球目标或电机控制。
- 涉及文件：F407 PeerLink文档、`MCU_UART_HEARTBEAT_PROTOCOL.md`、既有CAR消费和
  ChassisImuAcceleration链、双端`COOPERATION.md`及process记录；未改Task周期、引脚、
  QD4310控制参数、融合开关或前馈增益。
- 验证方式和结果：PeerLink CAR和ChassisImuAcceleration主机测试通过；Debug/Release
  clean-first完整链接零警告，FLASH/RAM为`34708/17672 B`和`30180/17656 B`。G3519
  Communication主机测试、Cortex-M0+严格编译和Bring-up Debug/Release完整链接也通过。
- 烧录与回读：F407使用`VID:PID 0416:5051`的CMSIS-DAP，OpenOCD识别Cortex-M4、
  DEV_ID尾码`0x413`和512 KiB Flash；写入Debug 34708 B并从`0x08000000`完整回读，
  源BIN与回读SHA-256均为
  `56362B8704CF4822E8008C2781EF096270E2D152747D8F73F34A44A86B9E6A36`。G3519使用指定
  DAPLink首次写入后检测到工作区又完成按键/Task更新，故以当前源码覆盖写入Bring-up
  Debug 79368 B，从`0x00000000`完整回读，SHA-256为
  `E5E0E1E4D75A56A84940B3E9EDB1323AB68E16F5F6CDBCD830E45F18BDE5385C`。两端均已复位运行。
- 已知限制：本次只证明目标识别、写入、整镜像一致性和复位运行；没有在最终双镜像下
  连续抓取100 Hz CAR或验证50 ms断流失效、动态ICM轴向、轮速比例和滚球前馈。
- 待确认：保持滚球电机失能，连续采集至少10 s双端HB/CAR和UART/DMA计数，再做正反向
  手推、起步、制动对照；只有轴向、延迟和噪声证据通过后才讨论启用融合或前馈。

## 2026-07-30 16:08：当前F407控制行为覆盖说明

- 本条晚于上方`HB/CAR最终协议与双板Debug部署`。板间协议仍保持双向HB和100 Hz CAR，
  但F407控制行为已经按用户后续明确要求更新：KEY回零稳定后启用ICM补偿，前馈增益现为
  `0.20`而不是0；绝对raw硬边界为`49325..59233`。详细状态机、限幅、构建、烧录和静态
  验证证据见紧邻上方的“KEY回零后启用ICM补偿与QD4310绝对raw限幅”条目。
- 上方协议部署条目中的“前馈保持0、不会影响电机控制”只描述SHA-256以`56362B...`开头的
  前一版F407镜像，已被当前SHA-256
  `1C4BBDE6308F882CF7756226F26C1709569FA7E621C11EB54E035A9D111AB918`覆盖，不再代表现状。

## 2026-07-30：ICM补偿响应与力度第二档

- 修改原因：用户完成旧档位实机测试，确认KEY启动顺序、补偿方向和保护没有问题，但反馈
  响应偏慢、纠正力度不足。
- 最终决策：ICM输出一阶低通时间常数由50 ms降至20 ms，补偿增益由`0.20`提高至`0.50`，
  ICM补偿角和视觉合成后的总杆角限幅均由`±0.50°`提高至`±0.60°`。角环继续使用
  `P=0.017 A/deg`、电流限`0.30 A`，未恢复历史实测会振动的`P=0.020 A/deg`。2 ms Task、
  100 Hz CAR、KEY状态机、绝对raw边界`49325..59233`和11点杆角表均未改变。
- 涉及文件：控制参数位于`config/core_config.h`；同步更新`README.md`、`COOPERATION.md`和
  本记录。工作树内其他未提交修改不属于本次调参。
- 验证方式和结果：Debug clean-first完整链接通过，FLASH/RAM为`35348/17696 B`；Debug BIN
  35348 B，SHA-256为`3D8C9AF77B7BAF691760BD0F5581A7123A62B99F9BC305F9501578956A1917B6`。
  CMSIS-DAP重插后OpenOCD编程和整镜像verify成功并复位。运行快照为`phase=STOPPED`、驱动
  Disable、命令未使能、速度0 rpm、raw=58953、raw blocked=0、Task fault=0；电流仅剩协议
  量化级零附近值。
- 已知限制：CAR仍只有100 Hz新测量，500 Hz Task只能在样本间保持/计算；20 ms低通和0.50
  增益尚未完成第二档带球动态验证。补偿输出、总杆角限幅和角环电流中究竟谁先成为瓶颈，
  需要日志才能判断，不能只凭体感继续加大KP。
- 待确认：用户旁站按KEY测试第二档，记录处理后加速度、补偿角、合成杆角、raw、电流、
  blocked和fault。若仍显弱且补偿角未触及0.60°，再提高前馈增益；若角度已触顶但电流很小，
  再评估角环P/死区；若出现振动或方向异常立即再次按KEY停机。

## 2026-07-30 16:29：水管中心、零点与限幅语义复核

- 修改原因：用户再次强调“补偿点是水管中心”并要求不要混淆滚球目标、机构零点和限幅。
- 当前唯一有效坐标定义：水管几何中心是球位置`x_target=0 mm`；机构回零点是QD4310单圈
  `raw=54379`；`raw=54329`只对应历史水平工作trim，不能写成新的球位置零点或机构零点。
- 当前命令链：KEY首次按下进入`ZEROING`并只命令相对杆角0°；满足电机角误差`±0.50°`、
  `|speed|<=1 rpm`连续300 ms后进入`ACCELERATION_COMPENSATION`。补偿阶段使用
  `theta_imu=-0.50*atan2(a_icm/g)`并限`±0.60°`；视觉有效时以`x_target=0 mm`执行球位置/速度
  闭环。代码实际限制的是`theta_ball+theta_imu`相对水平trim的动态偏移`±0.60°`，随后才加
  trim并查11点向上表，所以不得表述为“相对机械零点的最终绝对杆角恒为±0.60°”。当前
  trim约`+0.061°`，补偿phase相对机械零点的实际命令约`-0.539°..+0.661°`，对应raw约
  `54917..53774`，完整位于11点表内。
- 数据边界：本轮补偿只消费100 Hz `CAR`中的ICM45686 X/Z比力和Pitch角速度，经去重力及
  20 ms一阶低通后使用；轮速字段虽保留在Task输入结构中，但底盘融合开关为0，补偿phase
  直接选择处理后的ICM加速度，轮速不参与该角度计算。球Kalman也保持关闭。
- 物理边界：`0.50`增益在小角度未饱和区只施加约一半的理想静态抵消角；`0.60°`杆角对应
  完整理想静态抵消能力约`g*tan(0.60°)=103 mm/s²`，而当前0.50增益约在输入加速度
  `g*tan(1.20°)=205 mm/s²`时触及角限。它不能保证任意加速度下小球静止；没有有效VIS时
  也没有球位置误差闭环，水管中心只是冻结目标，不是仅靠IMU即可保证的结果。视觉与ICM
  共用同一个`±0.60°`动态预算：例如`185 mm/s²`时ICM约占`0.540°`，同方向视觉只剩约
  `0.060°`，并非两路各自都能输出`±0.60°`。
- raw保护证据边界：低raw侧配置`49325`由用户明确标记的第二端实测`raw=49319`向内收
  6 count；高raw侧`59233`保留此前落地侧约`59293`读数的内缩余量，但缺少同条件的新一轮
  明确端点复测。两者都是保护边界，不是水管中心或杆角表端点；正常11点查表范围仍更窄。
- 并发协作收口：此前20 ms只读`DBG`提案曾撤回；用户随后明确要求用串口持续送入VOFA，
  因而当前源码已重新完成双端DBG并追加为G3519 FireWater CH22～CH31。该新方向覆盖旧撤回
  结论，但截至本条更新仍未烧录或完成32列实物抓取。
- 验证：当前源码Debug/Release clean-first完整链接零警告，FLASH/RAM分别为
  `35348/17696 B`和`30836/17672 B`。9组主机测试以
  `-Wall -Wextra -Wpedantic -Werror`重新编译运行并全部PASS，覆盖BallPositionControl、
  BallStateEstimator、ChassisAccelerationFusion、ChassisImuAcceleration、LevelCurrentControl、
  RodAngleMap、PeerLink CAR、VisionLink和QD4310 raw。当前Debug BIN为35348 B，SHA-256=
  `3D8C9AF77B7BAF691760BD0F5581A7123A62B99F9BC305F9501578956A1917B6`；Release BIN为
  30836 B，SHA-256=`6C1E42BC5FD0EEF5E8C0D059755CC47D6F2747770BDB23085681FAF51F95BF05`；
  `git diff --check`无空白错误。Debug哈希与上一条已编程/整镜像verify并取得STOPPED/Disable
  快照的镜像一致。
- 本轮硬件复核限制：先清理残留OpenOCD PID 41628；随后三次50 kHz短连接均识别到Cortex-M4，
  但目标在`halt`时报告`unknown state`，因此未读到任何新RAM快照，也未据此声称Peer/CAR计数
  或电机状态再次通过。本轮没有擦除、编程、复位、按KEY或发送CAN控制命令，OpenOCD均已退出，
  3333/4444端口无监听。

## 2026-07-30：F407关键调试量经G3519送入VOFA

- 修改原因：用户选择持续串口显示，要求将关键调试数据送入VOFA，避免SWD逐次读取。
- 最终决策：F407在500 Hz Gateway计算之后每20 ms非阻塞提交一次只读`DBG`，字段依次为
  处理后底盘加速度`mm/s²`、ICM补偿角`mdeg`、最终杆目标`mdeg`、目标/反馈raw、命令/反馈
  电流`mA`、速度`0.01 rpm`、phase和bit0～bit8状态。DBG使用独立序号；BUSY整帧丢弃且
  不推进序号，无ACK和重发，不作为控制输入。
- 接收/显示：G3519检查节点、规范十进制、范围、phase、flags和重复/旧序号，以100 ms
  本地年龄保存Latest Value，追加到原CH0～CH21后的CH22～CH31。G3519在CH31额外置bit9
  表示DBG新鲜；101 ms起十列全0。VOFA使用COM5、115200、8N1、FireWater，名义50 Hz。
- 涉及文件：`APP/Gateway`、`Device/PeerLink`、`Task/task_app.c`、`config/core_config.h`、
  PeerLink测试/README、根README、板间协议、`COOPERATION.md`和本记录；未改500 Hz Task、
  KEY状态机、QD4310控制参数、引脚、DMA或保护行为。
- 验证方式和结果：F407 PeerLink host测试以`-Wall -Wextra -Wpedantic -Werror`编译运行PASS，
  覆盖payload、边界、BUSY不推进序号和非法值；Debug/Release clean-first完整链接通过，
  FLASH为`36936/32056 B`，RAM为`17744/17736 B`。G3519 Communication测试和双配置链接
  结果记录在其`docs/PROCESS.md`。
- 已知限制：双端探针当前均未枚举；Windows只看到COM5的`0D28:4001`串口复合设备，未见
  F407 `0416:5051`或G3519 `0D28:0204` CMSIS-DAP，因此本轮没有擦写、复位或启动电机。
  也尚无32列FireWater格式器host测试、双端字节互喂和实际串口带宽证据。
- 待确认事项：重新连接两枚探针后，先确认phase=STOPPED、命令未使能、反馈Disable和电流
  约0，再烧录双端Debug；随后连续抓COM5，确认严格32列、约50 Hz、CH31 bit9新鲜、
  CH22～CH30动态正确，并观察DBG发送/接收/BUSY/解析计数无持续增长。

## 2026-07-30：DBG首次实抓与角环KP第三档

- 修改原因：用户提供292帧、5.82 s的F407 `DBG`实抓；小球持续滚动，要求确认90 mA不再
  增力的原因，并明确决定先增大KP、保留先回零稳定再开启补偿，不采用按KEY立即补偿。
- 实抓结论：DBG序号9919～10210连续，周期17～23 ms、均值20 ms；全程phase=1、flags=3，
  目标raw固定54379，反馈raw为55353～55383，最终误差982 count/约5.394电机度。命令/反馈
  电流均约-88 mA，速度基本为0，位置未向目标收敛；加速度、补偿角和杆目标字段均为0，
  证明当时仍在ZEROING，尚未进入加速度补偿。
- 控制器解释：原始误差约-5.394 deg，扣除0.20 deg死区后，P项为
  `0.017*(-5.194)≈-0.088 A`。误差超过1 deg积分区时积分每周期清零，不是积分饱和；即使
  进入积分区，积分分量也只允许`±0.010 A`。`±0.30 A`总电流限幅没有触发。
- 最终决策：仅把`GATEWAY_LEVEL_KP_A_PER_DEG`从0.017提高到0.030 A/deg；相同误差下预计
  P电流约-0.156 A。KEY状态机继续`STOPPED -> ZEROING(raw=54379) -> 稳定300 ms ->
  ACCELERATION_COMPENSATION`，不实现直接启动补偿；KI、KD、死区、积分区、积分/总电流限幅、
  raw保护、标定表和DBG协议均不变。
- 涉及文件：`config/core_config.h`及本记录；工作树其他滚球、通信和协作文档改动不在本条
  重复认领。
- 验证方式和结果：LevelCurrentControl主机测试以`-Wall -Wextra -Wpedantic -Werror`重新
  编译运行PASS；Debug/Release clean-first完整链接通过，FLASH/RAM分别为`36936/17744 B`
  和`32056/17736 B`。新KP Debug/Release BIN SHA-256分别为
  `F63C30F391A1073CF522477F13F7ECFCF732148E019FA25626B2C8496FDFB9F1`和
  `3D1CA192161C0CF544FE051D990CA116C7DE779AE799739D1B5B5BBF923830EB`。
- 已知限制：0.030在历史实机中出现过振动，本轮只完成软件验证；新KP镜像尚未烧录、复位或
  启动电机。DBG在ZEROING中主动报告acceleration/ball无效，不能用flags=3单独证明相机断线。
- 待确认事项：保持人员旁站，先确认旧固件已按KEY停止，再烧录新KP镜像。首次只做短时回零，
  观察电流是否约升至156 mA、raw是否向54379收敛；若振动、方向异常或接近raw边界，立即
  再按KEY执行0 A后Disable。进入phase=2后再分别检查acceleration_valid和ball_valid。

## 2026-07-30：KP第三档与DBG版F407烧录

- 修改原因：用户明确要求立即烧录当前最新F407 Debug镜像。
- 烧录结果：Windows精确枚举`VID:PID 0416:5051`；xPack OpenOCD以200 kHz识别Cortex-M4、
  DEV_ID尾码`0x413`和512 KiB Flash。写入当前Debug ELF并完成`Verified OK`、复位；对应BIN
  SHA-256为`F63C30F391A1073CF522477F13F7ECFCF732148E019FA25626B2C8496FDFB9F1`。
- 复位安全发现：MCU本地已为`phase=STOPPED`、命令电流0、Task故障0，但QD4310驱动器保留
  烧录前使能状态，反馈电流约`-0.1004 A`。这证明“MCU复位后本地状态清零”不等于驱动器
  已Disable。
- 处置与复读：通过当前ELF和现有`Device_QD4310_Disable()`接口向主电机发送Disable，返回
  true；0.8 s后复读为反馈`enabled=0`、电流约`-0.000305 A`、速度0、online=1、raw=59091，
  `phase=0`、命令电流0、`g_TASK_Fault=0`、Task Ready=1。随后恢复运行并关闭OpenOCD。
- 行为边界：全程未按KEY、未启动滚球控制。G3519 `0D28:0204`本轮未枚举，故未重复擦写；
  其协作记录中的当前80768 B镜像已完成写入和回读。
- 已知限制与待确认：当前启动路径只初始化本地STOPPED状态，没有在每次MCU复位时主动发送
  QD4310 Disable，驱动器可能跨复位保留旧命令。若要永久修复，应经用户确认后在安全启动
  顺序中增加一次显式Disable并重新测试断电、复位和CAN离线场景。

## 2026-07-30：KP=0.030 Debug烧录与复位安全快照

- 修改原因：用户在软件验证后明确授权烧录新KP镜像，但要求不立即开启补偿。
- 最终决策：部署上一条记录的`GATEWAY_LEVEL_KP_A_PER_DEG=0.030 A/deg`；KEY状态机、
  KI/KD、0.30 A电流限幅、raw保护、回零稳定门和补偿参数均不变。本轮不按KEY。
- 烧录前证据：以50 kHz SWD非停核读取，phase=STOPPED、命令电流0、反馈Disable、速度0、
  `command_enabled=0`、Task fault=0，raw=59023且位于`49325..59233`范围内。
- 烧录与验证：Debug BIN 36936 B，SHA-256=
  `F63C30F391A1073CF522477F13F7ECFCF732148E019FA25626B2C8496FDFB9F1`。OpenOCD锁定
  `VID:PID 0416:5051`并识别Cortex-M4、DEV_ID尾码0x413和512 KiB Flash；编程完成、
  整镜像`Verified OK`并复位。
- 复位后证据：非停核读取phase=STOPPED、命令/反馈电流0、反馈Disable、速度0、
  `command_enabled=0`、raw=59027、Task fault=0；约32.24 s内Task heartbeat=16119，约500 Hz。
  OpenOCD已退出，3333/4444无监听。
- 涉及文件：`config/core_config.h`、控制说明、`COOPERATION.md`和本记录；未改变通信、Task、
  状态机或保护逻辑。
- 已知限制与待确认：烧录/静态快照不等于KP=0.030动态通过；该值历史上出现过振动。首次
  只做人员旁站的短时回零，确认电流约156 mA且raw向54379收敛；振动、方向错误或逼近边界
  时立即按KEY执行0 A后Disable，再依据DBG决定降低P或提高阻尼。

## 2026-07-30：ICM加速度补偿极性反向对照固件

- 修改原因：用户提供DBG序号24506～24782并要求立即反转补偿极性后烧录，用于与原方向做
  实机对照；本次“极性”明确限定为ICM底盘加速度到杆角的前馈符号，不是电机电流或D阻尼。
- 数据证据：277帧全部为phase=2、flags=7，说明电机在线/使能且ICM加速度有效，球视觉位
  始终无效。加速度范围-657～824 mm/s²，补偿角42/277帧触及±0.60°。末100帧目标raw均值
  54302.4、反馈53796.6，平均误差505.8 count/2.778°；98帧速度为0，命令均值-46.3 mA。
  末帧按当前控制律可还原为-90 mA静态前馈+42.5 mA P+0 mA D=-47.5 mA，与实测-48 mA
  一致；该日志不支持反转电机位置环或D阻尼。
- 最终决策：仅将`GATEWAY_CHASSIS_FF_POLARITY`从-1改为+1；增益0.50、补偿限幅±0.60°、
  电机`CURRENT_POLARITY=+1`、P/I/D、静态承重前馈、状态机和全部保护保持不变。JY61P仍只
  作为G3519 VOFA CH12/CH13指示，不进入该补偿或任何控制。
- 验证方式和结果：Debug/Release clean-first完整链接通过，FLASH/RAM分别为36988/17752 B
  和32124/17736 B。烧录前SWD读到phase=STOPPED、驱动Disable，并再次发送Disable成功；
  Debug ELF写入后所有段`compare-sections`匹配。Debug BIN SHA-256为
  `D68C3CD453845431C2686387C962AD35BE9A3F4118CBC8E60CB3A577A8FD51D1`。复位后快照为
  phase=STOPPED、active=0、反馈Disable、电流约-0.00061 A、速度0、Task fault=0。
- 已知限制：当前只完成反向镜像部署和静态安全检查，尚未启动电机，不能宣称反向更好。
- 待确认：人员旁站做同方向、同力度的短时推车对照；观察CH22与CH23是否由原反号变为同号，
  同时比较球位移和raw跟随。若球位移明显变大或机构趋向端点，立即按KEY执行0 A+Disable。

## 2026-07-30：重力前馈实测、极性确认与大误差提速部署

- 修改原因：KP=0.030短测能抬升机构，但目标附近仍需约-0.10 A承重；用户随后反馈响应偏慢，
  并要求再次确认正负极性。
- 当前有效控制参数：否决会振动的KP=0.030，恢复`KP=0.017 A/deg`；新增独立静态重力前馈
  `-0.090 A`。KI/KD仍为`0.0010/0.0010`，总电流限幅仍为0.30 A，raw边界和KEY状态机不变。
- 实机证据：旧KP=0.030短测从raw约59091运行0.8 s到55023，电流约-0.100 A，Disable后回落；
  当前KP+前馈短测在0.3 s到raw=54343，距目标54379仅36 count，命令/反馈电流分别约
  `-0.08997/-0.08972 A`，积分仅0.000033 A，速度0且Task fault=0。负电流使raw减小并把机构
  向上送至目标，因此电机角环和静态承重前馈极性已确认；该证据不确认ICM加速度补偿极性。
- 已部署镜像：上述KP+前馈的Debug镜像SHA-256为
  `C90782B46B9571D11BE766456118CF2A6BF7691186E9766039136C057987B426`，已编程并`Verified OK`。
- 提速修改：为提高远距离回零电流而不改变近目标刚度，仅把角误差限幅从8°提高到10°；
  最大远端命令约由-0.223 A提高到-0.257 A，KP、死区、前馈和0.30 A总限流不变。Debug/Release
  clean-first零警告，FLASH/RAM分别为`36988/17752 B`和`32124/17736 B`；BIN SHA-256分别为
  `C1107165A4CA60C7DC070E6AB2DAB7AACF4556DED0A2275A011E2221E8B37967`和
  `C7788ABCAC4CFF8B210BDAFBF6FD47EBBD1C33FA4521A69F42AE79820032B343`。Debug已烧录并
  `Verified OK`；复位回读实际装载error limit=10°、KP=0.017、前馈=-0.090 A。
- 烧录安全证据：CMSIS-DAP以50 kHz保守quirk模式连接。烧录前回读QD4310为online=1、
  enabled=0、电流约-0.00061 A、速度0、raw=59033，MCU为STOPPED/inactive/command gate 0；
  烧录后仍为STOPPED，主程序/Task fault=0，驱动反馈Disable。
- 0.3 s短测：从落地侧启动后约0.3 s进入phase=ACCELERATION_COMPENSATION；当时目标/反馈raw为
  `54231/54207`，命令/反馈电流约`-86.01/-86.37 mA`，速度约-4.00 rpm，积分近0，控制fault=0，
  raw未触边。说明大误差回零方向正确且已到目标区；该帧含动态目标，但未同时读取带符号ICM
  加速度和补偿角，因此仍不能单凭本次冻结ICM物理极性。
- 停机复读：首次stop调用返回true后状态又一次进入phase=2，来源未冻结；再次调用现有
  `stop_primary_level_control()`返回true，0.5 s后复读为phase=STOPPED、inactive、
  `command_enabled=0`、enabled=0、电流约-0.00061 A、速度0、raw=59049、Task fault=0。
  OpenOCD已退出；后续DAP短测期间禁止同时操作KEY，以免混淆启停边沿。
- 协作复核：Agent A确认根目录`vofa+.csv`是旧`KP=0.017、FF=0`数据，全程ZEROING、ACC/补偿
  均为0；只能确认负电流对应raw减小，不能用于评价当前重力前馈或ICM补偿，也不能据此加KP。
- 待确认：使用VOFA同时记录phase、目标/反馈raw、电流、ICM加速度和补偿角，完成同方向起步/
  制动对照；ICM动态极性和带球效果未验收前不得上赛道。近目标仍弱时先减少死区或做小步KP，
  不直接恢复历史会振动的0.020/0.030。

## 2026-07-30：视觉、JY61P、ICM45686与无刷全链路极性统一（未烧录）

- 修改原因：用户要求暂不烧录，依据视觉、JY61P、车体IMU和QD4310无刷电机统一所有符号，
  并避免把ICM补偿、电机角环和转速阻尼混为同一个“极性”。用户口述的ICM42688经源码和
  `WHO_AM_I=0xE9`复核，当前实际器件/驱动仍是ICM45686。
- 最终决策：统一物理坐标为右侧固定转轴指向左侧执行端`x>0`、执行端抬高`theta>0`。
  MaixCAM2画面右正且画面右侧是固定转轴，所以`x=-x_camera`，VIS极性保持`-1`；11点上行
  实测为raw减小、CH12增大、执行端上升，且负电流实测使raw减小，所以电机电流极性保持
  `+1`，D项继续使用负速度反馈。ICM预处理保持车体X动态符号；在车体`+X`与杆轴`+x`
  同向的安装约定下，底盘`a_parallel>0`要求`theta_ff<0`，因此源码将此前只用于对照的
  `GATEWAY_CHASSIS_FF_POLARITY=+1`覆盖为`-1`。JY61P仍只作为CH12/CH13角度指示，不进入控制。
- 涉及文件：`config/core_config.h`、`APP/Gateway/app_gateway.c`、根`README.md`、
  `tests/control_polarity_chain/test_control_polarity_chain.c`、`COOPERATION.md`及本记录。
  未修改CAR/VIS/DBG协议、Task周期、P/I/D、静态承重前馈、限幅、状态机或硬件配置。
- 验证方式和结果：新增端到端主机测试同时覆盖相机左右位置、BallPositionControl逆模型、
  ICM X/Z去重力符号、RodAngleMap正反变换和LevelCurrentControl电流符号，以
  `-Wall -Wextra -Wpedantic -Werror`编译运行PASS。Debug/Release clean-first完整链接零警告，
  FLASH/RAM分别为`36988/17752 B`和`32124/17736 B`。
- 实时只读证据：COM5严格32列链路在线。首次4 s抓取中F407为`phase=1,flags=515`，目标/
  反馈约`54379/54125`、命令/反馈约`-70/-70 mA`，未进入补偿且ACC/VIS有效位均为0；按要求
  提醒用户停机后再次抓到`phase=0,flags=513`、命令0 mA、反馈约-1 mA、驱动失能。
  JY61P CH12持续有效且CH13年龄约0 ms。该数据确认通信、JY指示和最终停机，不确认ICM/VIS
  物理极性。Windows只枚举G3519串口`0D28:4001/COM5`；F407探针和MaixCAM2 ADB均未枚举。
- 已知限制：本轮严格未烧录、未启动新极性固件；硬件仍运行此前ICM `+1`反向对照镜像。
  MaixCAM工程仍为`TROUGH_POSITIVE_AT_RIGHT=True`、`F4_UART_CALIBRATION_VALID=False`，所以
  F407不会放行视觉位置。G3519当前Device_IMU使用sensor到body默认同轴映射，但实物车体
  `+X`究竟朝杆自由端还是固定轴没有实时原始量/照片证据，源码`-1`只能视为统一坐标下的
  正确物理符号，不能写成实物轴向已通过。
- 待确认事项：恢复F407探针和MaixCAM2连接后，先保持无刷失能完成相机九点/端点标定；再
  以一次朝自由端和一次朝固定轴的手推对齐轮速、CAR X和杆轴。只有确认`+X`朝自由端后才
  可烧录当前`-1`镜像；随后在视觉有效、人员旁站条件下短测，要求CH22与CH23反号，手把球
  移到画面左侧时目标杆角为正、目标raw下降、命令电流为负。任一关系不符立即停机。

## 2026-07-30：视觉/JY61P/ICM45686/无刷电机极性统一（未烧录）

- 修改原因：当前源码把已经在上一轮决定为`+1`的ICM前馈重新写成`-1`，且README、极性测试
  和未启用的球状态模型仍混用了“杆轴正方向”与“车辆前进正方向”。用户要求本轮不烧录，
  先统一全部符号。用户口述的ICM42688与工程及实车数据源不符，当前器件继续按ICM45686记录。
- 最终决策：杆轴`x>0`为右侧固定转轴指向左侧执行端；相机画面右正，故
  `x=-x_camera`、`GATEWAY_BALL_MEASUREMENT_POLARITY=-1`。执行端抬升为`theta>0`；11点
  JY61P+车体IMU表证明`theta>0 -> raw减小`，电机实测证明负电流使raw减小，故
  `GATEWAY_LEVEL_CURRENT_POLARITY=+1`保持不变。处理后的ICM45686纵向加速度以车辆向前为
  `a_fwd>0`；按用户确认的补偿动作冻结为`a_fwd>0 -> theta_ff>0 -> raw减小 -> 负电流`，
  `GATEWAY_CHASSIS_FF_POLARITY=+1`。旧`-1`方向已否决。
- 模型同步：三状态球估计器的底盘输入改为车辆前进正坐标：
  `a_model=-kappa*(g*sin(theta)-a_fwd*cos(theta))`。因此正向车体加速度使球模型沿杆轴正方向
  加速，正杆角产生反向重力分量；前馈正杆角可抵消该扰动。估计器开关仍为0。
- 涉及文件：`config/core_config.h`、`README.md`、`APP/Algorithm/BallStateEstimator/`、
  `tests/control_polarity_chain/`、`tests/ball_state_estimator/`、`COOPERATION.md`和本记录。
  视觉PID保持开启；未改变Task、通信协议、状态机、增益、限幅或保护。
- 验证方式和结果：严格主机测试`control_polarity_chain`、`ball_state_estimator`、
  `ball_position_control`、`chassis_imu_acceleration`全部PASS。Debug/Release ARM clean-first完整
  链接零警告，FLASH/RAM分别为`36988/17752 B`和`32124/17736 B`。
- 已知限制：本轮没有连接探针、烧录、发送Enable/电流命令或驱动电机。视觉符号有Maix源码
  “画面右正”和安装“画面左为执行端”的证据，但Maix仍未提供`CALIBRATION_VALID`实时帧，
  因此不能把离线测试写成带球实物闭环通过。JY61P继续只用于杆角标定/指示，不直接进入控制。
- 待确认：电机失能时把球分别放在画面左/右端，确认F407接收的物理位置分别为正/负；随后
  人员旁站短时测试`a_fwd>0`时CH23为正、目标raw下降。任一方向不符立即停止，不调整增益掩盖。

## 2026-07-30：ICM45686纵向动态极性双次实车确认

- 结论：极性确认完成。执行端位于车头；车辆向前加速时应降低车头执行端，即`theta<0`。
  两次向车头短推均确认当前安装的ICM mapped X先变负，F407预处理保留该动态符号，因此
  `GATEWAY_CHASSIS_FF_POLARITY=+1`正确：`a_icm_x<0 -> theta_ff<0 -> raw增大 -> 车头端降低`。
  旧的“车辆向前为正且应抬高车头执行端”方向已否决。
- 修改原因：用户要求以两次同方向手推结果一致作为冻结条件，并允许根据实测修改极性；
  当前数值已经是计算所得的`+1`，所以不做无意义翻转，只修正相反的注释、极性链测试和说明。
- 实机证据：标定全程无刷失能，未按KEY、未发送Enable/电流命令。第一次有效采样静止基线
  X约`-0.730400 m/s^2`，向车头推动段出现负向动态、回弹出现相反正向动态；第二次长窗口
  基线X为`-0.697868 m/s^2`，运动范围`-4.582502..+1.667562 m/s^2`，负峰时左右轮速为
  `+33/+123 mm/s`，与第一次符号一致。
- 涉及文件：`config/core_config.h`、`tests/control_polarity_chain/test_control_polarity_chain.c`、
  `README.md`、`COOPERATION.md`和本记录；未改变运行极性数值、增益、限幅、状态机、通信或保护。
- 软件验证：更新后的`control_polarity_chain`以`-Wall -Wextra -Wpedantic -Werror`重新编译，
  运行输出`PASS`；验证向车头加速使用负mapped X，并得到负杆角、raw增大和正电流方向。
- 安全收尾：采样脚本已释放COM5；G3519 OpenOCD收到`shutdown`，3333/4444无监听；F407无
  调试会话。两块板均未烧录、未驱动电机。
- 已知限制：本次确认的是ICM纵向测量到杆角前馈的物理符号，不等于视觉毫米标定或带球闭环
  已通过。当前关闭的Ball Kalman使用车辆向前正的`a_fwd`模型语义；启用前须先显式转换当前
  向前为负的mapped X，不能直接混用。

## 2026-07-30：F407本地BMI088替换ICM45686加速度控制源

- 修改原因：用户决定保留F407与G3519通信，但彻底停止消费G3519 `CAR`中的ICM45686数据，
  改由F407直连BMI088完成高速读取、去重力、滤波和无刷加速度补偿。
- 最终决策：SPI1使用PA5/PA6/PA7、Mode3、5.25 MHz；ACC_CS=PA4、GYRO_CS=PC4，PC13保持高
  隔离共享总线的板载W25Q128。SPI1_RX/TX分别使用DMA2 Stream0/Stream3 Channel3。
  Device采用静态缓冲非阻塞Start/Poll状态机；Accel为1600 Hz+OSR4+±6 g，Gyro为
  1000 Hz/116 Hz+±500 dps；1 ms Task形成约500 Hz完整六轴帧。控制Task只把BMI088最新有效
  帧提交给现有ChassisImuAcceleration和Gateway，PeerLink仍运行但`CAR`不再进入控制。
- 坐标与控制：实机静止原始读数约`X=+0.29、Z=-9.72 m/s²`，确认当前安装传感器Z朝下；映射
  冻结为`body X=Sensor X`、`body Z=-Sensor Z`、Pitch积分输入`=+Sensor GyroY`。按+X朝车头
  的安装约定，向前正加速度通过`FF_POLARITY=-1`映射为车头执行端降低。8 ms输出低通、
  `±0.60°`动态限幅、杆角查表、绝对raw边界和电流保护全部保留。
- 涉及文件：`BSP/bsp_spi.h`、`BSP/bsp_f407.c`、`Device/BMI088/`、`Task/task_app.[ch]`、
  `config/core_config.h`、`CMakeLists.txt`及本记录。放弃的UART4/JY61P草稿已移除。
- 验证方式和结果：Debug完整链接通过，FLASH/RAM为`40748/17920 B`。OpenOCD按VID:PID
  `0416:5051`锁定F407，编程并`Verified OK`。运行快照中BMI状态`ONLINE(2)`，Heartbeat
  `16816`时六轴序号`8331`，实测速率约495 Hz；坐标修正后处理加速度约`10 mm/s²`、Pitch
  约`-1.731°`。Task fault=0，电机phase=STOPPED、命令电流0 A；OpenOCD随后已关闭。
- 已知限制：尚未执行向车头手推，BMI088 X动态极性和陀螺Pitch动态符号仍需一次失能状态下
  的短动作确认；8 ms滤波未按电机振动频谱整定。PA4同时连接开发板KEY3，运行时禁止按KEY3。
- 待确认事项：保持无刷失能，向车头短推一次；若处理加速度先为正，则当前`FF=-1`正确，若
  为负则只调整BMI轴映射，不修改电机、视觉、杆角表或保护极性。

## 2026-07-30：BMI088极性冻结、1 kHz DMA链与加速度处理收口（离线）

- 结论：两次无刷失能下向前短推的主动作分别为`+264 -> -219 mm/s²`和
  `+1519 -> -2362 mm/s²`，符号一致，冻结完整链为`Sensor X正 -> 车辆向前加速度正 ->
  FF_POLARITY=-1 -> theta<0 -> 目标raw增大 -> 车头执行端降低`。摆杆实测链仍为
  `theta>0 -> raw减小 -> 车头执行端升高`，因此`BMI088 X_SIGN=+1`、前馈`-1`和
  电机电流极性`+1`均不再翻转；`±0.60°`动态角限幅、杆角表、raw边界和电流保护全部保留。
- 修改原因：旧实现每1 ms只能轮询推进一笔ACC或GYRO事务，完整六轴约495 Hz且固定多一拍；
  8 ms低通响应偏慢，Pitch互补校正还会把持续纵向加速度逐渐误认为车体倾角。
- 最终决策：BMI088保持Accel 1600 Hz/OSR4/±6 g和Gyro 1000 Hz/116 Hz/±500 dps；
  SPI1 RX DMA终结ISR只按事务序号通知Gateway Task，Task以有限等待在同一1 ms周期依次推进
  ACC/GYRO，健康稳态两笔，初始化或遗留相位最多三笔。迟到/重复IRQ由`active/event_sent`
  去重，Abort清DMA标志和两个NVIC pending；若通知后SPI仍`BSY`则留到下周期Poll，不等待
  不存在的第二个通知。完整帧和Gateway调度目标均为1 kHz，FreeRTOS Tick仍为1 kHz。
- 数据处理：输出一阶低通由8 ms改为4 ms。离线回放中事件滞后约由6 ms降至2 ms，静止噪声
  约由11.6增至14.3 mm/s² RMS，强动作削峰约由18%降至8%。Pitch先以Gyro Y预测，计算
  `a_candidate=fx+g*sin(pitch_pred)`；仅当重力模长有效且`|a_candidate|<=150 mm/s²`时允许
  加速度计校正，否则保持陀螺预测。没有运行中自适应消除约`+10.5 mm/s²`静态偏置。
- 同步修正：默认关闭的Ball Kalman模型统一为
  `a_model=-kappa*(g*sin(theta)+a_fwd*cos(theta))`，历史容量由64增至128，在1 ms周期下保留
  约127 ms固定滞后窗口。按用户此前决定，ESP8266默认关闭，不初始化、不轮询、不收发；
  G3519 PeerLink、MaixCAM视觉、CAN和本地BMI088链路不受影响。
- 涉及文件：`BSP/bsp_f407.c`、`BSP/bsp_spi.h`、`Device/BMI088/`、`Task/task_app.c`、
  `APP/Algorithm/ChassisImuAcceleration/`、`APP/Algorithm/BallStateEstimator/`、
  `APP/Gateway/app_gateway.c`、`config/core_config.h`、对应测试、README及本记录。标定用8192点
  双通道日志已删除，释放约64 KiB临时RAM。
- 验证方式和结果：严格主机测试`chassis_imu_acceleration`、`control_polarity_chain`和
  `ball_state_estimator`均PASS；持续`1000 mm/s²`达500 ms时Pitch保持0°且加速度不被吃掉。
  Debug/Release均clean-first完整链接通过，FLASH/RAM分别为`37636/20064 B`和
  `32696/20048 B`，无编译错误。
- 已知限制与待确认：1 kHz是当前离线实现和读取目标，尚未上板按Heartbeat、BMI sequence、
  QD反馈sequence和CAN成功率实测；未使用DRDY，Gyro 1 kHz读取可能偶发重复样本。实际电流
  闭环率受`min(1 kHz, QD4310应答率, CAN成功率)`限制。采样结束后OpenOCD已关闭；本轮没有
  连接、复位、烧录、发送Enable/电流命令或驱动电机，也未宣称视觉带球闭环已实物通过。

## 2026-07-30～31：BMI088纵向加速度两级快速滤波与烧录

- 结论：极性保持`BMI088 X_SIGN=+1`、`FF_POLARITY=-1`、电机电流极性`+1`不变。输出滤波
  由单级Euler 4 ms改为两级相同的精确EMA，每级`tau=2.25 ms`、
  `alpha=1-exp(-dt/tau)`；1 kHz下总截止约46 Hz、无过冲，冻结强推波形峰滞后约2 ms。
- 修改原因：用户要求根据此前静止/手推冻结波形重新滤波，同时注重响应速度。旧单级滤波
  高频滚降只有一阶，且`dt/(tau+dt)`没有精确表达配置的连续时间常数；超长采样间隔还可能
  把间隔前的旧方向值作为当前样本发布。
- 最终决策：两级使用本拍更新后的第一级值，增加一个`float`静态状态和每新样本一次额外
  乘加；不新增Task、动态内存、通信字段或状态机。超过最大采样间隔后，两级直接以当前有效
  去重力加速度重种，禁止继续发布旧方向。采样率、Pitch门控、前馈增益、`±0.60°`角限幅、
  raw边界、电流限幅和全部安全行为不变。
- 回放证据：旧单级Euler 4 ms在冻结日志上的静止RMS约`12.68 mm/s²`、强推正/负削峰约
  `9.35%/7.45%`；两级2.25 ms为`14.24 mm/s²`、约`5.0%/4.3%`，峰滞后约2 ms。1 kHz理论
  幅值在10/20/50/100/200/500 Hz依次约为`0.981/0.927/0.672/0.345/0.127/0.048`。
- 涉及文件：`APP/Algorithm/ChassisImuAcceleration/`、`Task/task_app.c`、`config/core_config.h`、
  两组相关测试、README、COOPERATION和本记录。
- 验证方式和结果：新增冻结幅值正转负短脉冲、500 Hz交替噪声和断帧反向重种断言；两组
  测试以ARM GCC `-Wall -Wextra -Wpedantic -Werror`完成编译。Debug/Release完整链接无警告，
  FLASH/RAM分别为`38396/20072 B`和`33472/20048 B`。当前环境没有本机C运行器，因此没有
  宣称测试运行PASS。2026-07-31用户允许烧录后，以VID:PID `0416:5051`锁定F407探针；
  OpenOCD识别Cortex-M4、DEV_ID尾码`0x413`和512 KiB Flash，Debug ELF编程完成并复位。
  按用户要求未做额外回读；未发送Enable/电流命令或启动无刷电机。
- 已知限制与待确认：冻结静止段没有无刷启用后的振动谱，所以不增加固定陷波。VOFA仍只以
  50 Hz抽取，极短峰可能在显示端漏采；这不影响1 kHz内部滤波。后续上板只需核对BMI序号率、
  静止RMS、短推双峰和电机开启后的振动主频，再决定是否需要针对性陷波。
## 2026-07-31：BMI088 Sensor Y纵向加速度链与F407烧录

- 结论：控制加速度已改为`a_forward=(Sensor Y-Ay_bias)*+1`。启动前300个连续有效样本求Ay运行均值并冻结；标定完成前不提交控制加速度。旧Sensor X/Z、Pitch、Gyro Y和重力投影逻辑已从控制链删除。
- 修改原因：实车VOFA确认CH23/Sensor Y平行车身，向前推动时先出现正主峰、停车时出现负反峰；旧Pitch链会把重力漂移误判为持续纵向加速度。
- 最终决策：保持两级精确EMA，每级`tau=2.25 ms`；超过`5 ms`采样间隙时以当前`Ay-bias`重种两级滤波。前馈仍为`0.50*(-1)*atan2(a_forward,g)`，前馈角和视觉+B​​MI088总动态杆角限幅均保持`±0.60°`，杆角表、绝对raw边界和电流保护不变。
- 涉及文件：`APP/Algorithm/ChassisImuAcceleration/`、`Task/task_app.[ch]`、`config/core_config.h`以及对应两组主机测试。VOFA CH22/CH23/CH24继续保持原始BMI088 X/Y/Z，便于实车观察；G3519固件未改、未烧录。
- 验证方式和结果：F407 Debug完整链接通过，FLASH/RAM为`33308/20056 B`。OpenOCD按`VID:PID 0416:5051`锁定F407，写入当前Debug ELF并复位运行；按用户要求未执行额外verify或运行回读。
- 已知限制：本轮只完成编译和部署，没有宣称带球闭环效果已通过。启动零偏假定复位后车辆至少静止约`0.3 s`；若启动阶段移动车辆，本次冻结偏置会错误，需重新复位并静止标定。
- 待确认事项：静止约0.3 s后按KEY1，观察向前加速时车头执行端是否降低，以及带球稳态、7～10 Hz机构追振和`±0.60°`饱和占比。

## 2026-07-31：无刷使能振动陷波与加速度前馈降增益

- 结论：新`vofa+1.csv`证明旧版目标长时间撞`±0.60°`不是BMI088本底噪声，而是无刷/机构约9～14 Hz振动回灌。现在`Ay-bias`后先加1 kHz RBJ陷波，再进入原两级EMA；前馈增益由`0.50`降为`0.20`。
- 数据证据：新phase2共3279帧/65.58 s，旧版目标端点占比`70.57%`，目标到反馈仍约滞后`40 ms`，主振在约`9.4～13.7 Hz`内漂移。电机失能时Ay标准差约`30～43 mm/s²`，使能后约`500～600 mm/s²`。视觉有效位未出现，这组数据不用于评价视觉闭环。
- 最终决策：陷波参数为`fs=1000 Hz`、`f0=11.7 Hz`、`Q=1.2`；两级EMA仍为每级`tau=2.25 ms`。离线回放估计静车振动RMS约由`561`降至`187 mm/s²`，增益0.20时静车角目标限幅率约由`49.4%`降至`1.56%`。不改`±0.60°`、raw边界、电流保护、PID和任何极性。
- 涉及文件：`APP/Algorithm/ChassisImuAcceleration/algorithm_chassis_imu_acceleration.[ch]`、`Task/task_app.c`、`config/core_config.h`及两组相关主机测试。
- 验证方式和结果：Debug固件完整链接通过，FLASH/RAM为`38520/20096 B`。以F407探针`VID:PID 0416:5051`写入Debug ELF并复位，OpenOCD报告`Programming Finished`；按用户要求未执行额外回读。
- 烧录后实机证据：最新`vofa+.csv`的phase2共2532帧/50.64 s，目标端点占比由上一组`70.57%`降至`34.32%`，跟踪MAE由`517.4`降至`227.8 raw`，转速RMS由`9.63`降至`4.65 rpm`，最大误差由`3896`降至`985 raw`；目标到反馈滞后仍约`40 ms`。旧的11.6～13.5 Hz不再主导目标，但仍有8.2～9.1 Hz残余，目标主能量现在0.75～1.34 Hz。VOFA I23依旧是陷波前的原始Sensor Y，其RMS不会直接显示陷波效果。
- 已知限制与待确认：50 Hz VOFA抽取可能混叠高频分量；新版虽明显改善，但目标仍有`34.32%`时间在端点。最新数据全程无视觉有效位，不能用来评价球位置闭环；还需区分0.75～1.34 Hz成分是人为推车的真实加速度，还是电机使能后的低频自激。

## 2026-07-31：静车滚球运行时目标与有限斜率（离线）

- 修改原因：赛题第3项要求球先稳定在`0 mm`，再执行`0→+50→-50 mm`；原球位置控制器
  只有编译期固定目标`0 mm`，运行中无法安全换点。
- 最终决策：在现有`BallPositionControl`内增加有界目标指令、活动参考和线性推进接口；
  Gateway仅在phase 2接受目标。当前产品范围只开放`[-50,+50] mm`，参考斜率固定为
  `50 mm/s`，目标切换不清位置/速度状态；停机和再次启动均立即恢复`0 mm`。未新增自动
  序列状态机、Task或通信字段，KEY第二次按下仍为`0 A + Disable`。
- 无刷边界：保持滚球角环`0.30 A`限流、raw`49325..59233`、raw恢复电流`0.10 A`、
  动态杆角总限幅`±0.60°`、P/I/D、极性和全部原有停机行为不变。
- 涉及文件：`APP/Algorithm/BallPositionControl/`、`APP/Gateway/app_gateway.[ch]`、
  `config/core_config.h`、两组相关测试、根`README.md`及本记录。
- 验证方式和结果：GCC 16.1以`-Wall -Wextra -Wpedantic -Werror`运行
  `ball_position_control`与`control_polarity_chain`均PASS，覆盖非有限/越界拒绝、失败后旧
  目标不变、`0/+50/-50`斜率、换点保留状态及控制极性。Debug/Release ARM clean-first完整
  链接通过，FLASH/RAM分别为`39104/20120 B`和`33828/20096 B`。
- 已知限制：当前API没有运行中的协议调用者；电机使能时不得用停核SWD/GDB调用，因为停核
  期间QD4310可能保持上一电流。现有DBG也没有球目标字段，不能只用32列VOFA证明误差。
  本轮未连接探针、未烧录、未启动MaixPy、未发送Enable/电流命令，也未做带球实测。
- 待确认事项：若用户批准跨双端的小范围协议修改，再由G3519 M3/调试入口向F407 Task发送
  专用运行命令；不复用CAR/DBG，不改变KEY急停语义。实测前还必须确认机构raw已回到安全
  工作区，先做中心及`±10/±20 mm`短测，再扩大到`±50 mm`。

## 2026-07-31：F407球目标/停止PeerLink入口（离线）

- 决策：新增`BTG=2,<target_0p1mm>` Latest Value和`BST=2`停止事件，各自独立16位序号、
  静态单槽、重复/旧帧抑制，无ACK/自动重发。协议域±1250，产品安全域±1200。BST优先并
  复用原`0 A -> Disable`；BTG仅phase 2且上一周期视觉有效时SetTarget，不隐式启动。
- 保持：未新增Task/状态机；KEY、0.30 A限流、raw`49325..59233`和杆角`±0.60°`未改。
- 验证：PeerLink测试源Cortex-M4严格编译零警告；Debug/Release clean-first链接通过，
  FLASH/RAM为`39864/20192 B`和`34412/20168 B`。Host测试可执行文件本轮未运行。
- 未验证：未连接/烧录/启动MaixPy；双端抓帧、实际失能及动态目标均待逐级实机验证。

## 2026-07-31：VTP/VPA/VST与PeerLink会话收口（离线）

- 修改原因：现有F407改动已接入VIS/VTP接收、VPA/VST发送和滚球PID运行时调参；只读审查
  发现PeerLink远端会话重启时清除了BST序号基线，却没有清除旧会话尚未消费的STOP槽；
  同时VST把所有VisionLink运行期错误都误映射为`CONFIG_INVALID`。
- 最终决策：`reset_remote_session()`同时清`ball_stop_pending`和`pending_ball_stop`，避免旧
  会话STOP在新会话继续执行；补充“旧STOP被清、BST序号baseline重开”的Host断言。
  VST的b14只在`Device_VisionLink_Init()`拒绝静态UART/缓冲/超时配置时置位；运行期Step
  错误归入UART错误，VST/VPA发送问题归入TX错误。`MEASUREMENT_USED_SEEN`和最后使用序号
  改为Task私有粘滞快照，链路超时100 ms宽限从VisionLink初始化时刻计算。
- 行为边界：没有修改滚球PID公式/增益、目标、杆角/电流/raw限幅、视觉失效动作、KEY启停、
  Task周期、UART引脚或任何硬件配置；没有烧录、复位、发送Enable/电流命令或启动电机。
- 涉及文件：`Device/PeerLink/device_peer_link.c`、
  `tests/peer_link_encoder/test_device_peer_link_encoder.c`、`Task/task_app.c`、
  `Device/VisionLink/README.md`和本记录；其余工作树中的VTP/VPA/VST与PID改动均原样保留。
- 验证方式和结果：MSYS2 GCC 16.1以`-Wall -Wextra -Wpedantic -Werror`实际运行
  `peer_link_encoder`、`vision_link`、`ball_position_control`和`control_polarity_chain`，四组
  均PASS；`git diff --check`通过。Debug/Release ARM均clean-first完整链接通过，FLASH/RAM
  分别为`43052/20520 B`和`37152/20496 B`。
- 已知限制与待确认：Task内VST快照组合随目标固件完成编译，但尚无独立Gateway/Task Host
  集成测试；VST/VPA的真实20 ms发送率、BUSY计数、VIS接收/使用序号及故障位仍须双端连续
  抓帧确认。实机闭环前仍需确认MaixCAM2到F407的3.3 V、共地和交叉接线、视觉毫米标定、
  相机到杆轴极性以及当前机构raw位于安全工作区。

## 2026-07-31：实机闭环积分与杆角上限

- 修改原因：用户确认位置停在非零点，批准`Ki=0.30 s^-3`并允许扩大杆角上限。
- 最终决策：上电默认滚球PID为`Kp/Ki/Kd=1.20/0.30/1.80`；滚球杆角与总杆角限幅由
  `±0.60°`扩大到`±0.80°`。`0.30 A`电流限幅、raw`49325..59233`、KEY启停和
  100 ms视觉超时行为不变。
- 涉及文件：`config/core_config.h`和本记录。
- 验证方式和结果：Debug链接通过，FLASH/RAM为`43052/20520 B`；使用D盘xPack OpenOCD、
  CMSIS-DAP `0x0416:0x5051`完成最终编程与复位。按用户要求，最终烧录未执行verify/readback，
  烧录后未再次启动电机。
- 已知限制：`±0.80°`的收敛效果尚未实测；Maix当前使用未标定的整幅画面线性坐标。

## 2026-07-31：水平电流环下行前馈释放与下行回差定位

- 修改原因：目标`raw≈55031`、反馈`raw≈54871`时位置误差约`+0.8789°`，原公式中的
  `-0.072 A`静态承重前馈大于P项和受限积分之和，静止理论电流仍约
  `-0.0525..-0.0625 A`；实测下行需要正电流，因此目标无法继续下行。
- 最终决策：保留目标点完整静态前馈；当位置误差与前馈方向相反时，在现有`0.20°`
  死区内将前馈从原值线性释放到0。误差越过死区、P项开始作用后不再叠加反向前馈。
  P/I/D增益、`0.30 A`电流限幅、raw`49325..59025`边界与恢复、KEY启停和视觉超时均不变。
- 涉及文件：`APP/Algorithm/LevelCurrentControl/algorithm_level_current_control.[ch]`、模块
  `README.md`、`tests/level_current_control/test_algorithm_level_current_control.c`、
  `config/core_config.h`注释和本记录。
- 验证方式和结果：MSYS2 GCC 16.1以`-Wall -Wextra -Wpedantic -Werror`实际运行
  `level_current_control`和`control_polarity_chain`，两组均PASS；`git diff --check`通过。
  Debug/Release ARM均clean-first完整链接通过，FLASH/RAM分别为`43180/20520 B`和
  `37248/20496 B`。160 count示例在静止且积分为0时理论电流为`+0.00950 A`。用户确认停机
  后，使用D盘OpenOCD锁定`0416:5051`写入Debug ELF并复位，报告`Programming Finished`；
  按要求未做verify/readback，电机仍由用户KEY启动。
- 实机定位：随后稳定段视觉/F407位置约`+6.3 cm/-63 mm`、速度0，外环杆命令约
  `-0.55°`且未触及`-3.80°`限幅；但电机目标raw约`54750..54950`，反馈raw约
  `54990..55010`，内环因此持续给出约`-0.08..-0.09 A`抬杆电流。用户同时观察到连杆
  未实际下行。源码杆角表只有向上扫描分支，F407闭环反馈只有电机编码器raw而没有杆输出
  实际角度，因此该矛盾指向下行回差/间隙造成的电机raw与真实杆角不一致，不是球位置积分
  不足或当前raw/负向杆角限幅。
- 已知限制：前馈释放解决了反向前馈直接压过大误差下行电流，但不能观测或补偿连杆下行
  回差；在现有`0.20°`过渡内还可能形成约31..36 count的目标前电流换向区，不能据此宣称
  球位置无稳态误差。
- 待确认事项：优先取得下行分支的实际杆角反馈并建立方向相关映射；快速台架方案可先在
  人员旁站下逐级增加有界下行raw补偿，找到刚好越过回差的count，再决定是否接入杆角传感器
  形成真实输出角闭环。未经用户批准不继续修改参数或运行行为。

## 2026-07-31：249点双向连杆标定扫描（待烧录）

- 修改原因：现有11点表只有raw减小的向上分支，实机已出现电机raw越过目标但连杆未按
  命令下行；用户授权从物理下限到上限再返回下限采集更密的双向JY61P/raw数据集。
- 最终决策：临时`GATEWAY_LEVEL_CALIBRATION_KEY_MODE=1`。KEY第一下自动扫描
  `58825 -> 49525 -> 58825`，75 raw一步，单程125点、往返249个稳定点；第二下随时复用
  现有`0 A -> Disable`停机，最后一点保持完成后也自动停机。两端分别比绝对raw边界
  `59025/49325`内缩200 count，`0.30 A`限流、raw恢复和边界电流阻断不变，视觉外环不运行。
- 稳定与同步：每点连续满足目标/反馈差不超过50 raw且绝对转速不超过1 rpm达到300 ms后，
  现有DBG `phase`由1变2并保持500 ms，名义产生约25帧稳定DBG；任一点10 s未稳定即安全
  停机。G3519无需改协议，可在同一32列VOFA行使用CH12杆角、CH13 JY61P年龄、CH25目标raw、
  CH26反馈raw、CH27/28电流、CH29转速和CH30 phase对齐；建表必须使用反馈raw而非目标raw。
- 涉及文件：`config/core_config.h`、`APP/Gateway/app_gateway.[ch]`和本记录；Task周期、
  PeerLink/DBG字段、BSP、引脚、CAN与UART配置均未修改。
- 验证方式和结果：扫描公式静态复算确认首点58825、转折49525、回程首点49600、末点58825，
  总点数249；每点稳定确认加报告平台的理论最短总时长199.2 s，另加实际移动时间。
  `level_current_control`和`control_polarity_chain`以MSYS2 GCC 16.1严格编译运行均PASS；
  Debug/Release ARM clean-first完整链接通过，FLASH/RAM分别为`43788/20536 B`和
  `37840/20520 B`。
- 已知限制：当前仅完成离线软件验证，尚未证明75 raw小步能够克服下行静摩擦，也未证明
  raw电气边界等于连杆机械无干涉边界；首次运行必须全程人员旁站并保持KEY可立即停机。
- 待确认事项：烧录前确认G3519 VOFA已持续记录且JY61P CH13年龄正常；运行后按phase=2分组，
  对每点的CH12与CH26取稳定中位数，分别生成raw减小/增大两张严格单调表并检查回差宽度。

## 2026-07-31：标定首点抬升门控修复与扫描顺序纠正

- 修改原因：实机按KEY后没有抬升。源码复核确认标定首点`index=0`被排除在电流输出条件外，
  因而只能Enable后持续发送`0 A`；同时停机后的约`-8.33 rpm`运动会被普通`1 rpm`启动门槛拒绝。
- 最终决策：废止上一条从下端开始的扫描顺序。第一次KEY改为先只允许`-0.20 A`向上预定位到
  内缩上端`53577`，再按`53577 -> 58499 -> 53577`采集下行/上行分支，仍为单程125点、
  往返249点。标定启动转速单独限为`12 rpm`，普通滚球启动仍保持`1 rpm`；第二次KEY仍立即
  `0 A -> Disable`，绝对raw保护、标定电流和10 s单点超时不变。
- 涉及文件：`config/core_config.h`、`APP/Gateway/app_gateway.[ch]`和本记录；G3519记录固件、
  MaixCAM、Task周期、通信协议、BSP和引脚均未修改。
- 验证方式和结果：f4独立复核现有VOFA与交接记录，确认负电流使raw减小并对应抬升；
  Debug/Release ARM均clean-first完整链接通过，FLASH/RAM分别为`43980/20536 B`和
  `38016/20520 B`。D盘xPack OpenOCD锁定`0416:5051`写入Debug ELF并复位，报告
  `Programming Finished`；按用户要求未执行verify/readback，未替用户按KEY或启动电机。
- 已知限制与待确认：方向证据来自既有实测，当前新顺序尚未做本轮动态实机确认。首次KEY后应看到
  目标`53577`、命令约`-200 mA`且连杆抬升；若目视方向不符，立即再次按KEY停机。

## 2026-07-31：标定高频震荡抑制

- 修改原因：首点抬升恢复后，用户立即观察到电机高频震荡并按KEY停机。固定`0.20 A`与
  到点后`0 A`在位置容差边缘反复切换，且没有速度阻尼，构成极限环。
- 最终决策：移动远端仍保留`0.20 A`上限，进入800 raw减速区后渐减至`0.12 A`，加入
  `0.008 A/rpm`速度阻尼；首次进入15 raw目标带后锁定到点状态，改用`-0.072 A`重力前馈、
  `0.0015 A/raw`位置P和同一速度D连续保持，不再切换为0 A。稳定/报告窗口缩为`100/120 ms`，
  以满足G3519每点9帧记录并缩短249点扫描时间；第二次KEY急停、方向门控和raw边界不变。
- 验证方式和结果：Debug/Release ARM均clean-first完整链接通过，FLASH/RAM分别为
  `44300/20544 B`和`38272/20520 B`，`git diff --check`无空白错误。用户确认停机后，D盘
  xPack OpenOCD锁定`0416:5051`写入Debug ELF并复位，报告`Programming Finished`；按要求未做
  verify/readback，未替用户按KEY。
- 已知限制与待确认：新阻尼参数为基于当前0.20 A扫描和既有约`-0.072 A`承重电流的首轮值；
  仍须用户旁站确认首次抬升无高频震荡，并让自动扫描完成或随时用第二次KEY停机。

## 2026-07-31：滚球过零振荡首轮阻尼修正

- 修改原因：实机滚球明显反复过零；默认外环`Kp/Ki/Kd=0.6/0.15/0.2`满足
  `Kd*Kp=0.12 < Ki=0.15`，线性化三阶闭环不满足Routh稳定条件。
- 最终决策：仅将滚球速度阻尼`Kd`提高到`1.2 1/s`，保持滚球`Kp/Ki`、杆角内环、
  前馈、电流与动态杆角限幅不变。反馈/启动硬边界恢复为`59025 raw`，杆角目标查表仍止于
  `57500 raw`，因此视觉不会命令进入高raw死区。
- 涉及文件：`config/core_config.h`；Maix端运行时VTP默认值同步为`0.6/0.15/1.2`。
- 验证方式和结果：Debug链接通过，FLASH/RAM为`43988/20544 B`；D盘xPack OpenOCD锁定
  `0416:5051`完成编程与复位，报告`Programming Finished`，按要求未做verify/readback。
- 已知限制与待确认：未替用户按KEY，尚待实机确认过零次数；若仍有慢摆，下一轮经用户同意后
  再将`Ki`由`0.15`降至`0.05`并收缩积分限幅。

## 2026-07-31：视觉闭环启动恢复与最新实机参数

- 修改原因：实机采样发现原`Kd=1.2`放大视觉速度噪声，5秒内杆角误差过零101次、转速达到
  `-43.7..+53.1 rpm`；同时启动位置位于`raw≈59015`时，QD4310首帧尚未确认Enable，旧逻辑
  立即停机，导致零点环和视觉环均未进入。
- 最终决策：滚球外环固定为`Kp/Ki/Kd=0.60/0.03/0.55`，积分加速度限`10 mm/s²`、速度滤波
  `tau=0.18 s`；反馈硬边界保持`48000..59025`，但`raw>57500`只以`-0.10 A`向内回收，回到
  单调映射区后才运行角度环。Enable后新增最长`200 ms`确认窗口，期间保持`0 A`并请求反馈。
- 验证方式和结果：Debug链接通过，FLASH/RAM为`44068/20544 B`；D盘OpenOCD完成编程与复位，
  未做verify/readback。实机KEY启动已观察到`phase 1 -> phase 2`、motor enabled、视觉球测量有效，
  未再因首帧`enabled=0`立即停机。
- 已知限制：视觉闭环仍会令杆目标在`53283..56761 raw`间快速反转，滚球尚未证明收敛到中心；
  当前提交保留该实机状态，后续目标斜率限制或继续调参必须另经用户同意。

## 2026-07-31：加速度前馈10 Hz二阶Butterworth低通

- 修改原因：保留加速度前馈后，14 Hz机构回灌虽由宽陷波抑制，但原两级8 ms EMA仍表现为
  噪声偏大；继续增大EMA时间常数又会使速度突变前馈响应过慢。用户同意以真正的二阶低通
  替换两级EMA，且不改本轮PID、前馈增益、极性、电流和杆角限幅。
- 最终决策：保持固定采样率`1000 Hz`、陷波`14 Hz/Q=0.7`，其后每个1 ms子步立即级联
  `Q=1/sqrt(2)`的10 Hz二阶Butterworth DF2T低通。2~5 ms合法采样缺口以前一拍输入推进
  前N-1个子步，最后一步才输入当前样本，避免非因果提前响应；超过5 ms时同时将陷波和低通
  重种到当前去偏加速度。单次更新子步配置上限为32，防止错误配置形成长循环。
- 涉及文件：`APP/Algorithm/ChassisImuAcceleration/algorithm_chassis_imu_acceleration.[ch]`、
  `Task/task_app.c`、`config/core_config.h`、两组相关测试和本记录。Task周期、BMI088采集、
  通信协议、控制状态机、PID与保护边界均未修改。
- 验证方式和结果：MSYS2 GCC 16.1以`-Wall -Wextra -Werror`编译并运行加速度处理和全链路
  极性测试，均PASS；测试覆盖正负阶跃、长间隔重种、1/5 ms稠密/稀疏状态等价、14 Hz陷波、
  10 Hz约-3 dB及20 Hz衰减。ARM Debug链接通过，FLASH/RAM为`43888/20568 B`。D盘
  xPack OpenOCD锁定`0416:5051`完成写入和复位，报告`Programming Finished`；按用户要求
  未做verify/readback，未按KEY或启动电机。
- 已知限制与待确认：Butterworth低频群延迟约22.5 ms，较旧8 ms两级EMA增加延迟但明显增强
  高频抑制；尚待用户旁站按KEY后采集实机响应。任何后续参数调整和电机启动仍需用户同意。

## 2026-07-31：前馈响应与稳态修正首轮提速

- 修改原因：10 Hz Butterworth实机闭环无高频失控，但用户观察到瞬态响应和稳态误差修正偏慢；
  用户否决本轮增加分路相位超前/条件积分，决定先做简单参数调整，后续再考虑Kalman滤波。
- 最终决策：Butterworth截止频率由`10 Hz`提高到`12 Hz`，滚球位置积分增益由
  `0.03`提高到`0.045 1/s^3`。保持14 Hz/Q=0.7陷波、`Kp/Kd=0.60/0.55`、积分限幅
  `10 mm/s^2`、前馈增益、内环、电流和杆角保护不变。
- 涉及文件：`config/core_config.h`和本记录；算法结构、Task、通信协议与状态机均未修改。
- 验证方式和结果：全链路极性测试以MSYS2 GCC严格编译运行PASS；ARM Debug链接通过，
  FLASH/RAM为`43888/20568 B`。烧录前通过现有BST序号50030安全停机，DBG确认`phase=0`、
  命令电流0 A、速度0；随后D盘OpenOCD完成写入并复位，未做verify/readback。
- 已知限制与待确认：12 Hz低频群延迟约18.8 ms，位置积分修正理论快约50%，但尚未进行
  本轮实机闭环采样；电机保持停机，须用户再次按KEY后确认过零次数和稳态误差。

## 2026-07-31：钢球破静摩擦小阶跃

- 修改原因：实机UDP位置在约`+35.6 mm`保持不动，而10秒DBG显示电机仍在运动，说明残余
  稳态误差更符合钢球未越过起滚静摩擦，不宜继续无条件提高全局Ki。用户同意仅在远离零点且
  球近似静止时施加刚好起滚的小杆角脉冲，并要求接近零点后完全禁用。
- 最终决策：视觉速度有效、`|位置误差|>8 mm`且`|滤波球速|<=8 mm/s`持续250 ms后，沿PID
  回零方向叠加`0.10°`；最长120 ms，检测到球开始移动、误差进入`±8 mm`或方向反转时立即
  撤销，随后冷却400 ms。最终杆角仍经过原`2.0°`算法限幅和总杆角/raw/电流保护。
- 涉及文件：`APP/Algorithm/BallPositionControl/algorithm_ball_position_control.[ch]`、
  `APP/Gateway/app_gateway.c`、`config/core_config.h`、滚球算法测试和本记录；Task周期、通信协议、
  PID、前馈和保护边界未修改。
- 验证方式和结果：滚球算法测试覆盖双向脉冲、250 ms门控、120 ms超时、运动即撤销、
  `±8 mm`禁用和非法配置，MSYS2 GCC严格编译运行PASS；全链路极性测试PASS；ARM Debug链接
  通过，FLASH/RAM为`44488/20608 B`。D盘OpenOCD完成写入并复位，未做verify/readback。
- 版本与待确认：小阶跃之前的12 Hz/`Ki=0.045`基线已先提交并推送GitHub `ball`分支，提交
  `b0fb2c4`；小阶跃改动尚未提交。电机保持停机，首次按KEY后须观察脉冲是否刚好起滚，若
  仍不动或越过零点，只能经用户同意后小步调整脉冲角度，不能直接增大Ki。

## 2026-07-31：视觉闭环进入速度门槛修正

- 修改原因：小阶跃首次运行未出现。串口确认系统持续停在`phase=1`；3秒内电机速度反馈为
  `-9.25..+13.0 rpm`、平均绝对值`4.18 rpm`，但raw位置误差最大仅48 count。旧逻辑要求
  连续300 ms同时满足位置条件和`|speed|<=1 rpm`，因此始终无法进入`phase=2`，视觉环与
  小阶跃均未执行。
- 最终决策：仅将回零完成速度门槛`GATEWAY_LEVEL_ZERO_ENTRY_MAX_RPM`由`1`提高到`15 rpm`；
  保持启动前KEY速度门槛、位置容差、300 ms稳定时间、PID、小阶跃和全部保护不变。
- 涉及文件：`config/core_config.h`和本记录。
- 验证方式和结果：烧录前BST序号50032停机，DBG确认`phase=0、0 A、0 rpm`；滚球算法和
  全链路极性测试均PASS，ARM Debug链接通过，FLASH/RAM为`44488/20608 B`。D盘OpenOCD
  完成写入与复位，未做verify/readback。
- 已知限制与待确认：15 rpm只放宽`phase=1 -> phase=2`的进入判据，不抑制电机现有高频速度
  抖动。电机保持停机，须用户再次按KEY后确认能进入phase2并观察0.10°脉冲是否使球起滚。

## 2026-07-31：钢球破静摩擦阶梯脉冲

- 修改原因：实机确认固定`0.15°`脉冲仍不足以可靠克服钢球静摩擦；随后又确认阶梯从
  `0.15°`起步增长过慢，并批准把近零禁用区从`±8 mm`缩小。用户批准逐次增大、提高
  初始值并缩小禁用区，同时要求保留明确上限。
- 最终决策：破静摩擦脉冲从`0.30°`开始；连续一次未推动后增加`0.05°`，最高`0.40°`。
  `|位置误差|>5 mm`、`|滤波球速|<=8 mm/s`持续`250 ms`的门控、`120 ms`脉宽与
  `400 ms`冷却不变。检测到移动、进入`±8 mm`或误差换向时立即撤销并把下一次重置为
  `0.30°`；最终杆角仍受原`±2.0°`及下游raw/电流保护限制。原`0.15°`初始值因增长过慢
  已被否决，原`±8 mm`禁用区也已由用户缩小为`±5 mm`。
- 涉及文件：`APP/Algorithm/BallPositionControl/algorithm_ball_position_control.[ch]`、
  `APP/Gateway/app_gateway.c`、`config/core_config.h`、滚球算法测试和本记录。
- 验证方式和结果：MSYS2 GCC 16.1严格构建并运行滚球算法测试与全链路极性测试，均PASS；
  ARM Debug clean-first完整链接通过，FLASH/RAM为`44704/20616 B`。用户确认电机失能后，
  使用CMSIS-DAP `0416:5051`完成写入与复位，OpenOCD报告`Programming Finished`；按要求
  未做verify/readback，未代用户按KEY。
- 已知限制与待确认：阶梯脉冲尚未做本轮烧录后的运动验证；首次运行须观察实际触发幅度、
  起滚速度和是否越过视觉零点，任何进一步调参仍需用户同意。

## 2026-07-31：MaixCAM触屏任意毫米球目标（离线）

- 修改原因：用户要求直接在MaixCAM2触屏指定小球在水管中的任意位置，同时明确必须继续
  使用F407本地杆球模型和底盘加速度前馈，不能转成QD4310内部位置指令。
- 最终决策：VisionLink新增独立`VBT` Latest Value，Payload为有符号`target_0p1mm`，协议域
  `±1250`、产品域`±1200`；独立16位序号、100 ms会话重开、重复/旧帧抑制。Task在现有
  1 ms Gateway周期消费最新目标。目标可在停机或phase 1预设并跨停机/再次启动保留，命令
  本身不启动电机；进入phase 2且视觉有效后仍执行
  `a_cmd=Kp*(target-position)+I-Kd*velocity`和实心球逆模型，并保持
  `requested_offset_deg=ball_angle_deg+chassis_angle_deg`。Maix滑条范围`-120..+120 mm`、
  1 mm步进，松手发送，按VST目标确认，未确认时250 ms重发且最多20次；首次VST先同步
  现有目标。触屏沿用画面左负右正，并以`-1`转换成F407物理坐标左正右负；VBT/VST协议
  本身保持F407物理坐标，不在接收端重复应用测量极性。
- 涉及文件：`Device/VisionLink/device_vision_link.[ch]`、`Task/task_app.c`、
  `APP/Gateway/app_gateway.c`、两组Host测试、根/算法/VisionLink README，以及Maix视觉仓库的
  `ball_target_touch.py`、`f4_vision_link.py`、`main.py`、配置、应用清单和PC测试。
- 验证方式和结果：Maix Python共12项测试PASS；MSYS2 GCC 16.1以
  `-Wall -Wextra -Wpedantic -Werror`构建并实际运行VisionLink、BallPositionControl和
  control polarity chain三组Host测试，均PASS。ARM Debug clean-first完整链接通过，
  FLASH/RAM为`44932/20648 B`；`git diff --check`通过。
- 已知限制：VST只以整数毫米回报目标，因此当前触屏也限定整数毫米；双端UART实收、触摸坐标、
  显示刷新、重发确认和非零目标带球收敛仍需真机验证。目标设置期间的活动参考继续按现有
  `50 mm/s`推进，但只有phase 2才执行杆角输出。
- 待确认事项：本轮未烧录、未启动MaixPy、未按KEY、未发送Enable/电流命令。部署F407前
  必须再次确认电机已失能；按用户要求烧录后不做verify/readback，视觉应用必须由设备完整
  工程的`maixpy_run.py`入口启动，且不得代用户按KEY。
## 2026-07-31：任意位置闭环首轮提速、增阻尼与破静摩擦增强

- 结论：已根据MaixCAM实测轨迹完成首轮参数更新、烧录和视觉重连；电机保持失能，动态效果待用户按KEY验证。
- 修改原因：目标`-59 mm`时，MaixCAM记录到小球由`+34 mm`长时间静止后冲到`-72 mm`，随后多次往返，表现为起步脉冲偏弱、闭环响应慢且阻尼不足。
- 最终决策：球外环改为`Kp/Ki/Kd=1.00/0.030/1.00`，活动目标斜率由`50`提高到`80 mm/s`。破静摩擦脉冲改为`0.45°`起步、每次`+0.10°`、上限`0.70°`，误差门槛`3 mm`、静止速度门槛`12 mm/s`、静止门控/脉宽/冷却分别为`150/150/200 ms`。raw限制保护本身不变，仅把`motor_raw_limit_blocked`诊断从历史粘滞改为当前控制周期状态。
- 涉及文件：`config/core_config.h`、`APP/Gateway/app_gateway.c`、根`README.md`、`APP/Algorithm/BallPositionControl/README.md`和本记录；Maix端同步`vision_config.py`运行时VTP。
- 验证方式和结果：BallPositionControl和全链路极性主机测试均PASS；ARM Debug clean-first链接成功，FLASH/RAM为`44948/20648 B`。确认VST为停机失能后，使用CMSIS-DAP `0416:5051`写入并复位，OpenOCD报告`Programming Finished`；按要求未做verify/readback。Maix重启后VPA接受`1.000/0.030/1.000`，持续VST显示`fault=0x0000`。
- 已知限制：本轮只完成软件、烧录和失能状态链路验证；更强脉冲是否一次起滚、超调是否下降、任意目标停车精度仍需实机动态确认。
- 待确认事项：用户按KEY后先做小范围目标变化；观察起滚延迟、首次峰值、过零次数和最终误差，异常时立即再按KEY停机。

## 2026-07-31：F407杆球模型视觉帧间插值启用

- 结论：已启用现有三状态固定滞后Kalman；YOLO实测帧负责位置校正，F407以实际杆角和BMI088底盘加速度在1 ms Gateway周期内传播位置、速度和模型偏差，最后一次实测达到100 ms后停止球外环输出。
- 修改原因：约50 Hz视觉帧之间以及短时漏检会令原视觉差分控制离散或立即回水平；用户要求先用简单方案提高最终控制计算频率，不引入自适应权重或目标轮速差分。
- 最终决策：设置`GATEWAY_BALL_KALMAN_ENABLE=1`；无当前视觉实测时不再立即重置估计器，只要杆角反馈有效且估计仍可信就继续模型预测。只接受YOLO实测帧作Kalman校正，Maix预测帧不作为二次测量。PID保持`1.00/0.030/1.00`，静摩擦脉冲和全部电流/raw保护不变。
- 涉及文件：`config/core_config.h`、`APP/Gateway/app_gateway.c`、根README、`APP/Algorithm/BallStateEstimator/README.md`和本记录。
- 验证方式和结果：BallStateEstimator严格主机测试PASS；Maix 12项Python测试PASS；ARM Debug clean-first完整链接成功，FLASH/RAM为`47732/20648 B`。烧录前VST为`status=0x323F`、`theta=0`且电机未使能；CMSIS-DAP `0416:5051`写入并复位，OpenOCD报告`Programming Finished`，按要求未做verify/readback。
- 已知限制：1 kHz结果是模型状态预测，不是1 kHz新视觉测量；首次可信输出需要可靠杆角反馈并接受至少3帧真实视觉。底盘三状态融合、自适应高低通权重和目标轮速差分均未启用。
- 待确认事项：用户按KEY后观察VST的估计有效/速度有效位、短时`NO_BALL`期间杆角连续性、超调和稳定时间；异常立即再按KEY停机。

## 2026-07-31：G3519赛题mode只读转发至视觉

- 修改原因：用户要求F407接收G3519当前赛题mode，并同步给Maix视觉程序。
- 最终决策：PeerLink把`CAR`扩展为八字段并只接受尾字段`competition_mode=2..6`；F407按
  本地接收时间提供1500 ms新鲜度读取。VisionLink在既有14个VST状态字段之后追加第15字段
  `competition_mode`，`0`表示没有新鲜mode，`2..6`表示有效赛题号。Task只把该只读状态
  组装进VST，不据此切换F407控制、启停电机或修改滚球目标；未新增Task、ACK、重发或
  状态机。
- 涉及文件：`Device/PeerLink/device_peer_link.[ch]`、`Device/VisionLink/device_vision_link.[ch]`、
  `Task/task_app.c`、对应README与PeerLink/VisionLink主机测试；并行存在的电机遥测扩展不属于
  本次mode同步成果。
- 验证方式和结果：PeerLink与VisionLink主机测试均PASS；F407 Debug/Release完整链接通过，
  BIN分别为`48196/41112 B`；`git diff --check`通过。
- 已知限制与待确认：本轮未烧录F407；G3519→F407的八字段`CAR`实收、1500 ms过期后回退
  `0`以及F407→Maix的15字段`VST`仍需双端UART联机验证。

## 2026-07-31：mode同步实物分段验证与F407部署

- 修改原因：用户要求立即验证G3519→F407→Maix的mode实物链路。
- 部署与安全状态：当前Debug clean-first完整链接通过，FLASH/RAM为`48724/20648 B`，BIN
  SHA-256为`556679897711A47BD625D88ABBA880DD154656AC21265EE48052B58F69F57D0B`。
  烧录前COM3为`phase=0`、命令`0 mA`、速度`0`、驱动未使能；OpenOCD锁定
  `VID:PID 0416:5051`写入ELF并`Verified OK`、复位。复位后再次抓到`phase=0`、命令
  `0 mA`、反馈约`-1 mA`、速度`0`、flags仅bit0 motor_online。
- 分段验证：Maix持续收到当前15字段VST。为隔离链路，使用SWD仅向F407的只读PeerLink
  接收缓存临时注入新鲜`mode=2`；Maix日志连续记录3条`mode=2`，超过1500 ms后自动恢复
  `mode=0`，证明F407→Maix转发和新鲜度回退生效。临时mode、时间戳和seen位随后均已清零，
  未改Flash、未按KEY、未启用电机。
- 未通过项：F407非停核运行快照显示`heartbeat_rx_count=0`、`car_state_rx_count=0`、
  `packet_parser_error_count=1`、USART1 framing error为1，未收到任何合法G3519帧。指定G3519
  DAPLink当前未枚举，故本次不能证明G3519→F407实物链路。
- 待确认事项：重新连接G3519指定DAPLink/目标电源并核对G3519 TX→F407 PA10及共地后，
  要求F407的heartbeat/CAR计数持续增长且Maix无需SWD注入即可稳定显示`M2..M6`。

## 2026-07-31：近目标速度阻尼增强与安静保持

- 结论：近目标阻尼调度已被两轮实测否决；当前统一使用基础`Kd=2.5`，静止保持改为
  位置滞回锁定。最终固件已烧录，电机保持失能，等待用户按KEY做动态验证。
- 修改原因：实测基础`Kp/Ki/Kd=2.0/0.30/2.5`能明显降低振荡但仍可能在目标附近抖动；
  用户明确纠正“小距离更强调加速度”为“小距离更强调阻尼”。此前准备的近目标Kp增益
  方案已被否决，未烧录。
- 最终决策：首轮近目标`Kd=4.0`和第二轮`Kd=3.0`均被实测否决，近目标调度现已禁用，
  全域统一使用基础`Kp/Ki/Kd=2.0/0.30/2.5`，球速一阶低通保持`30 ms`。静止保持在
  `|位置误差|<=2 mm`且`|滤波球速|<=8 mm/s`时进入；进入后忽略速度估计噪声并持续清零
  目标球速、积分、加速度和滚球杆角，只有位置误差超过`3 mm`才释放。破静摩擦脉冲继续关闭。
- 涉及文件：`APP/Algorithm/BallPositionControl/algorithm_ball_position_control.[ch]`、
  `APP/Gateway/app_gateway.c`、`config/core_config.h`及滚球算法/全链路极性测试。
- 验证方式和结果：首轮`Kd=4.0/60 ms/5 mm/s`动态采样约20秒，位置持续约
  `-17..+38 mm`往复、球速峰值约`-53..+41 mm/s`、杆角约`-1.37..+0.79 deg`，未收敛，
  用户随后按KEY停机。第二轮`Kd=3.0/30 ms/8 mm/s`可从约101 mm在2.3秒内回到零点并曾
  在`0..2 mm`保持约8秒，但随后重新形成约`-18..+9 mm`低频往复，故再次停机并否决。
  最终滞回实现的BallPositionControl、BallStateEstimator和全链路极性Host测试均PASS；
  ARM Debug clean-first完整链接通过，FLASH/RAM为`49436/20672 B`。CMSIS-DAP `0416:5051`
  完成编程与复位，OpenOCD报告`Programming Finished`；按用户要求未做verify/readback。
  烧录后VST恢复为`fault=0x0002`，命令0、速度0、滚球杆角0和脉冲0。
- 已知限制：位置滞回可防止速度噪声把保持状态反复踢出，但尚未证明真实钢珠逸出超过3 mm后
  的重新介入不会形成较小极限环；该行为必须由下一轮动态采样确认。
- 待确认事项：用户按KEY后先观察回零进入phase2，再验证是否能长时间保持在3 mm滞回带内；
  若位置再次持续超过10 mm往复，立即再次按KEY停机。

## 2026-07-31：持续极限环后的Kp单变量回退

- 结论：位置驱动力`Kp=1.2`实测收敛更慢且振幅扩大，已被否决；当前恢复`Kp=2.0`，
  同时大幅降低积分增益和积分限幅，最终固件已烧录并等待动态验证。
- 修改原因：滞回版本连续采样约30秒后仍出现约`-14..+13 mm`往复、球速约
  `-36..+32 mm/s`且杆角多次达到约`+/-0.8 deg`。钢珠高速穿过进入带时无法满足低速
  锁定条件，说明应先降低进入目标区前的位置驱动力，而不是继续扩大保持区或提高阻尼。
- 最终决策：`Kp=1.2`方案已否决；恢复`Kp=2.0`，将`Ki`由`0.30`降至`0.05`，积分
  加速度限幅由`30`降至`10 mm/s^2`，减少远端回零时积分蓄积和后续反向推球。`Kd=2.5`、
  球速低通`30 ms`、`2 mm/8 mm/s`进入与`3 mm`释放滞回、Kalman和保护保持不变。
- 涉及文件：`config/core_config.h`及本记录。
- 验证方式和结果：`Kp=1.2/Ki=0.30`动态采样10秒仍出现约`-37..+78 mm`振荡，最新位置
  仍约`-32 mm`、球速约`-67..+38 mm/s`，用户确认收敛慢并按KEY停机。最终低积分版本的
  BallPositionControl、BallStateEstimator和全链路极性Host测试均PASS；ARM Debug clean-first
  完整链接通过，FLASH/RAM为`49436/20672 B`。CMSIS-DAP `0416:5051`完成编程与复位，
  OpenOCD报告`Programming Finished`；未做verify/readback。烧录后VST恢复`fault=0x0002`，
  命令0、速度0、滚球杆角0和脉冲0，视觉接收序号持续增长。
- 已知限制：`Kp/Ki/Kd=2.0/0.05/2.5`及积分限幅10尚未做动态验证；降低积分可减少远端
  windup，但可能增大克服静摩擦所需时间。
- 待确认事项：按KEY后比较首次过零速度、峰值位置和衰减包络；若仍持续超过10 mm往复，
  立即按KEY停机。

## 2026-07-31：取消保持自动释放并统一全域球环参数

- 结论：已取消`3 mm`位置滞回自动释放；进入`2 mm/8 mm/s`安静区后持续保持不动，只有
  触屏/PeerLink毫米目标真正改变或测量状态复位才重新介入。固件已烧录，电机保持失能。
- 修改原因：`Kp/Ki/Kd=2.0/0.05/2.5`曾连续约7秒保持在`0..2 mm`，但原保持状态在位置
  漂出`3 mm`后自动释放，随后再次形成约`-14..+10 mm`慢摆。用户认为分段过多导致又慢又抖，
  明确要求进入小死区后不要再动。
- 最终决策：全域继续使用唯一一套`Kp/Ki/Kd=2.0/0.05/2.5`，近目标Kd调度保持禁用，
  球速低通`30 ms`、积分加速度限幅`10 mm/s^2`不变；离散静摩擦脉冲继续禁用。保持一旦
  进入便清零目标球速、积分、期望加速度和滚球杆角，不再检查自动释放位置阈值。重复发送
  相同目标不会解锁，不同目标及停机/重启引起的测量复位会解锁。
- 涉及文件：`APP/Algorithm/BallPositionControl/algorithm_ball_position_control.[ch]`、
  `APP/Gateway/app_gateway.c`、`config/core_config.h`、根/算法README及滚球算法/极性链测试。
- 验证方式和结果：BallPositionControl、BallStateEstimator、control polarity chain三组
  MSYS2 GCC 16.1严格主机测试均PASS；ARM Debug clean-first完整链接通过，FLASH/RAM为
  `49352/20672 B`。烧录前VST连续为命令0、速度0；CMSIS-DAP `0416:5051`完成写入和复位，
  OpenOCD报告`Programming Finished`，按用户要求未做verify/readback。烧录后VST仍为命令0、
  速度0、滚球杆角0，Maix视觉进程和状态接收序号持续增长。
- 已知限制：该锁存以“进入时球速足够低”为依据，进入后不主动纠正外力推动、管道残余倾斜
  或视觉零漂造成的位置逸出；这是用户为消除近目标反复动作明确选择的行为。
- 待确认事项：用户按KEY后连续采样至少30秒，确认首次进入保持后滚球杆角持续为0，并记录
  最终停车误差、首次过零速度和是否仍有机械抖动；异常时按KEY停机。

## 2026-07-31：滚球速度估计视觉跳点硬门控与烧录

- 结论：已确认速度环反复换向来自视觉位置跳点被Kalman接受；当前固件增加`12 mm`绝对
  创新门，并收紧估计器噪声与球速低通，已完成烧录，电机保持失能。
- 修改原因：实测约20至25 ms内视觉位置出现`-45 -> -26 -> -36 -> -63 mm`跳变，旧
  `3.5 sigma`门在位置协方差约`100 mm^2`时可放行约`37.7 mm`创新，导致估计球速在
  `-10 -> +27 -> -42 -> -28 mm/s`间翻向并直接扰动速度PI。
- 最终决策：已有状态的视觉校正必须同时满足`|innovation|<=12 mm`和sigma门，首次有效测量
  仍直接种状态。Kalman参数改为`R=36 mm^2`、模型加速度`Q=2000 mm^2/s^3`、偏置随机游走
  `Q=100 mm^2/s^4/s`、`2.5 sigma`；控制器球速一阶低通由`30 ms`改为`100 ms`。
- 涉及文件：`APP/Algorithm/BallStateEstimator/algorithm_ball_state_estimator.[ch]`及README、
  `APP/Gateway/app_gateway.c`、`config/core_config.h`和BallStateEstimator主机测试。
- 验证方式和结果：新增首次种状态绕过硬门、`13 mm`拒绝和`11 mm`接受断言；MSYS2
  GCC 16.1严格主机测试PASS。ARM Debug clean-first完整链接通过，FLASH/RAM为
  `49716/20688 B`。烧录前后VST均为命令`0 mA`、速度`0 rpm`、phase停止；CMSIS-DAP
  `0416:5051`写入并复位，OpenOCD报告`Programming Finished`，按要求未做verify/readback。
- 已知限制：硬门只抑制相对当前历史估计超过12 mm的单帧错误校正，不能修复连续、同方向且
  每帧均小于12 mm的系统性视觉漂移；100 ms低通会增加少量真实刹车速度反馈延迟。
- 待确认事项：用户按KEY后对`-40/0/+40/0 mm`做短采样，核对创新拒绝计数、估计速度方向、
  杆角换向次数、首次过零速度与停车误差，再决定是否仅微调硬门或低通时间常数。

## 2026-07-31：取消球状态Kalman融合并保留视觉速度环

- 结论：用户否决Kalman/杆球模型融合球速；当前固件改为真实视觉多帧测速的串级速度环，
  BMI088加速度仅独立补偿最终杆角，已完成烧录。
- 修改原因：Kalman版本实测零目标仍持续约`-57..+34 mm`摆动，估计速度约
  `-82..+87 mm/s`并多次令杆角换向，继续调Q/R不能形成可靠收敛。
- 最终决策：`GATEWAY_BALL_KALMAN_ENABLE=0`；保留位置P生成目标球速、目标球速限加减速、
  速度PI生成目标加速度和滚动逆模型。球速使用约`120 ms`真实视觉位置差分，再经`60 ms`
  一阶低通；连续视觉帧间隔异常时清空测速历史。底盘加速度不参与球位置或球速估计。
- 涉及文件：`APP/Algorithm/BallPositionControl/algorithm_ball_position_control.[ch]`及README、
  `APP/Gateway/app_gateway.c`、`config/core_config.h`和滚球/极性链测试。
- 验证方式和结果：新增多帧测速主机断言；BallPositionControl严格主机测试PASS。
  ARM Debug clean-first完整链接通过，FLASH/RAM为`46632/20768 B`。用户确认停机后，
  CMSIS-DAP `0416:5051`写入并复位，OpenOCD报告`Programming Finished`，未做verify/readback。
- 已知限制：速度环只在新真实视觉帧到达时更新，视觉帧间保持最近杆角；多帧差分会引入约
  120 ms测速窗口延迟，但不会再被杆角模型或底盘加速度预测拉动。
- 待确认事项：按KEY后先在零目标短采样，确认视觉球速方向、振幅衰减和停车误差；稳定后再
  测试`-40/0/+40/0 mm`，不稳定则立即停机。

## 2026-07-31：前馈输入切换为G3519 ICM45686 CAR加速度

- 结论：按用户最新要求，滚球加速度前馈只消费TI Template现有CAR中的ICM45686车体X轴
  比力；本轮仅完成构建，未烧录、未复位、未启动电机。
- 修改原因：此前F407把本地BMI088处理量同时填入计划输入和测量输入，并在phase 2再次用
  测量量覆盖前馈选择，无法使用G3519随车提交的加速度。
- 最终决策：1 ms Task在PeerLink Step后读取50 ms内有效的CAR Latest Value，把X轴比力
  作为当前前馈输入；下一值等于当前值且无预告切换。Gateway移除phase 2的BMI088强制覆盖，
  继续复用`theta=-0.20*atan2(f_x,g)`、`±0.60 deg`角限幅、回零phase门和现有杆角/raw保护。
  CAR过期只清前馈，不停止视觉闭环或电机；本地BMI088继续采集、滤波和诊断。
- 涉及文件：`Task/task_app.c`、`APP/Gateway/app_gateway.c`、`config/core_config.h`、根README、
  `Device/PeerLink/README.md`和本记录；未修改协议、Task周期、BSP、引脚、视觉控制参数或保护。
- 验证方式和结果：`git diff --check`通过；ARM Debug/Release完整链接通过且无编译警告，
  FLASH/RAM分别为`47204/20776 B`和`40424/20752 B`。
- 已知限制与待确认：当前直接使用ICM45686 X轴比力，没有新增去重力或低通；+X动态符号、
  100 Hz阶梯输入下的前馈方向、峰值球偏差和饱和率仍须在确认静止且可立即停机的实物条件下
  验证。本轮没有硬件运行证据。

## 2026-07-31：M4/M5/M6规划加速度前馈与轮速/IMU融合框架

- 结论：F407侧框架已完成，融合继续默认关闭；本轮未烧录、未复位、未按KEY或启动电机。
- 修改原因：用户要求等待G3519完成轮速加速度环和jerk规划后，以规划加速度做主前馈并与
  IMU/轮速观测融合；随后冻结M4为`100 mm/s²、400 mm/s³`，M5/M6为
  `80 mm/s²、320 mm/s³、234 mm/s`，并提供H题PDF核对路线与8/30秒题面约束。
- 最终决策：现有100 Hz `CAR`兼容接受严格八字段或十字段；十字段在mode后追加车头正向
  `planned_acceleration_x_mm_s2,plan_valid`，规划范围`±5000 mm/s²`且有效性独立于IMU
  `valid`。十字段规划量优先进入前馈；八字段暂时回退ICM X以保留当前行为。CAR平均轮速
  与本地BMI088处理加速度使用独立序号/样本身份校正已有`[v,a,b_imu]`融合器，避免100 Hz
  轮速Latest Value在1 kHz Task内重复使用。`GATEWAY_CHASSIS_ACCEL_FUSION_ENABLE=0`不变。
- VOFA：恢复既有DBG协议语义，CH22～CH24分别发送最终采用纵向加速度、底盘补偿角和总
  杆角命令，不再临时占用为BMI088原始XYZ。协议文档要求G3519本地同时记录mode/路段/
  用时、规划/实测速度、速度误差、规划/IMU加速度、加速度环输出、左右轮目标/实测和循迹
  误差。M5/M6理想S曲线单圈校核约`29.42 s`，仅余约`0.58 s`，必须实车复核。
- 涉及文件：`Device/PeerLink/device_peer_link.[ch]`及README、`Task/task_app.c`、
  `APP/Gateway/app_gateway.[ch]`、`ChassisAccelerationFusion`说明/测试、PeerLink测试、
  `core_config.h`、根README、板间协议和本记录。未修改Task周期/优先级/栈、BSP、UART波特率、
  ISR/DMA、控制phase、前馈增益、杆球参数或电机保护。
- 验证方式和结果：MSYS2 GCC 16.1以`-Wall -Wextra -Wpedantic -Werror`编译并运行PeerLink
  与ChassisAccelerationFusion测试，均PASS；覆盖八/十字段、规划独立有效、非法字段/范围、
  50/51 ms新鲜度及轮速/IMU分次校正。ARM Debug/Release clean-first完整链接无警告，
  FLASH/RAM分别为`47412/20792 B`和`40664/20760 B`；
  `git diff --check`通过。
- 已知限制与待确认：G3519十字段发送端和加速度环尚未在本工作树交付；当前只有软件接口，
  没有100 Hz实收、jerk连续性、轮速比例、坐标符号、延迟、VOFA列或球误差实证。融合Q/R、
  30 ms延迟和0.12 s响应模型仍是预测值，不能直接启用。待G3519完成后先以融合开关0采集
  M4起步/过B及M5/M6起步、切点、制动全量日志，再离线整定并另行确认是否启用融合。

## 2026-07-31：M4/M5/M6加速度前馈框架烧录到F407

- 结论：当前Debug固件已烧录、完整校验并复位运行；滚球电机保持停止，未按KEY。
- 修改原因：用户在框架完成并通过离线验证后明确要求烧录F407。本轮只执行部署和只读状态
  核对，没有继续修改控制逻辑、参数、协议或构建选项。
- 最终决策：使用`VID:PID 0416:5051`锁定F407 CMSIS-DAP，未操作同时存在的G3519
  `0D28:0204`探针；部署`build/Debug/f407_gateway.elf`，对应BIN长度`47412 B`、SHA-256
  `387ACD6ED700E912E261588A3A2ED4144BEEFF4D43CD2C5961B70367A3795E88`。
- 涉及文件：仅更新本记录；烧录镜像来自本轮已完成验证的Debug构建。
- 验证方式和结果：OpenOCD识别Cortex-M4、DEV_ID尾码`0x413`和512 KiB Flash，报告
  `Programming Finished`、`Verified OK`并复位。复位后COM3只读抓取到连续DBG序号
  `2806..2824`及增长的HB；DBG显示最终采用加速度`0 mm/s²`、phase=`STOPPED`、命令/
  反馈电流`0 mA`、速度`0 rpm`、状态位`0`，即电机未在线/未使能且Task fault位未置位。
  串口已释放，OpenOCD已退出。
- 已知限制与待确认：本次只证明镜像部署完整和静态停止状态；没有启动电机或进行动态前馈
  验证。`GATEWAY_CHASSIS_ACCEL_FUSION_ENABLE=0`仍保持关闭；G3519十字段CAR发送端、轮速
  加速度环和M4/M5/M6 jerk规划仍须由G3519侧完成后再做双端实车验收。
## 2026-07-31：滚球速度阻尼降低并烧录

- 修改原因：实机日志显示视觉球速在闭环内频繁换向，`Kd=0.55`使杆角和电流命令高频反向；用户要求立即减小并烧录。
- 最终决策：仅将`GATEWAY_BALL_VELOCITY_KD_PER_S`由`0.55`降至`0.30`，Kp、Ki、静摩擦速度辅助、视觉协议和状态机均不变。
- 验证与部署：ARM Debug链接通过，FLASH/RAM为`47412/20792 B`。烧录前最近VST为`fault=0x0002、command=0、speed=0`；CMSIS-DAP `0416:5051`写入Debug ELF并复位，OpenOCD报告`Programming Finished`。按用户要求未做额外测试、verify或readback，也未启动闭环。
## 2026-07-31：滚球视觉速度低通加强并烧录

- 修改原因：`Kd=0.30`实机明显改善后仍有较大速度噪声；用户批准加强球速低通并立即烧录。
- 最终决策：保持`Kp=0.60、Ki=0、Kd=0.30`，仅将视觉球速一阶低通时间常数由`60 ms`提高到`100 ms`。
- 验证与部署：烧录前VST为`command=0、speed=0、fault=0x0002`；ARM Debug链接通过，FLASH/RAM为`47412/20792 B`。CMSIS-DAP `0416:5051`写入Debug ELF并复位，OpenOCD报告`Programming Finished`；按用户要求未做额外测试、verify或readback。
## 2026-07-31：禁止G3519加速度前馈并烧录

- 修改原因：用户明确要求任何G3519测量或规划加速度均不得进入滚球杆角前馈。
- 最终决策：恢复F407本地BMI088作为唯一加速度前馈源；G3519 CAR只保留模式、任务和轮速通信，其测量/规划加速度不再写入控制输入。
- 验证与部署：烧录前VST为`command=0、speed=0、fault=0x0002`；ARM Debug链接通过，FLASH/RAM为`47380/20792 B`。CMSIS-DAP `0416:5051`写入Debug ELF并复位，OpenOCD报告`Programming Finished`；未做额外测试、verify或readback。
## 2026-07-31：还原滚球Kd与球速滤波并烧录

- 修改原因：用户要求还原本轮F407滚球速度阻尼和滤波参数。
- 最终决策：`Kd`由`0.30`恢复为`0.55`，视觉球速一阶低通时间常数由`100 ms`恢复为`60 ms`；禁止G3519加速度前馈的边界保持不变。
- 验证与部署：烧录前VST为`command=0、speed=0、fault=0x0002`；ARM Debug链接通过，FLASH/RAM为`47380/20792 B`。CMSIS-DAP `0416:5051`写入Debug ELF并复位，OpenOCD报告`Programming Finished`；未做额外测试、verify或readback。

## 2026-07-31：M4 A→B固定起步加速度前馈并双端烧录

- 修改原因：用户要求依据最新VOFA为A→B起步加入固定补偿，并纠正部署范围为F407与G3519两端。
- 最终决策：覆盖此前“完全禁止G3519规划加速度”的决定，但只开放严格门控的M4规划量：
  PeerLink心跳在线、CAR年龄不超过50 ms、`competition_mode=4`且`plan_valid=1`同时成立时，
  F407用十字段规划量替代本地BMI088。G3519实测加速度始终不进入杆角前馈，M2/M3/M5/M6
  及旧八字段仍用F407本地BMI088；融合开关保持0，前馈增益/极性保持`0.20/-1`。
- 验证与部署：Debug/Release clean-first完整链接通过，FLASH/RAM为`47428/20792 B`和
  `40680/20760 B`，`git diff --check`通过。锁定CMSIS-DAP `0416:5051`写入Debug 47428 B，
  完整回读SHA-256为`359AB4549FD867BE36B8BB311B8F0B5091C557A36BA2B497C10BC17E0B46A42F`，
  随后`reset run`。未按KEY或发送启动命令。
- 已知限制：本轮只验证编译、协议和镜像完整性，尚未动态确认CAR十字段实收、600 ms时基
  对齐、补偿角方向和滚球误差改善；首次运行须保留人员旁站并采集新版32列VOFA。

## 2026-07-31：启用M4规划、BMI088与轮速三状态融合并烧录

- 修改原因：最新VOFA确认3450 ms后BMI088持续工作，但停车段规划约`-99 mm/s²`时，本地BMI088
  中值仅约`-33 mm/s²`，且杆角长时间触及正向上限；需要用M4的平滑减速规划提供基准，并保留
  BMI088冲击响应和轮速闭环校正。
- 最终决策：启用现有`[v,a,b_imu]`融合器。仅在心跳在线、CAR年龄不超过50 ms、M4且规划有效时
  以G3519规划驱动预测，并分别按独立序号融合F407本地BMI088和CAR轮速；无新鲜规划时立即复位
  融合器并回退纯BMI088，避免把同一BMI测量同时作为模型输入和观测。保持BMI前馈增益`0.20`、
  前馈角限幅`±0.60°`及总杆角正向上限`+0.75°`不变；G3519实测IMU加速度仍不进入控制。
- 涉及文件：`APP/Gateway/app_gateway.[ch]`、`Task/task_app.c`、`config/core_config.h`及本记录；
  未修改线上协议、Task周期、ISR/DMA、引脚、按键语义或电机保护。
- 验证与部署：融合算法主机严格编译测试PASS；Debug/Release clean-first完整链接通过，FLASH/RAM
  分别为`50068/20792 B`和`43240/20776 B`。使用`VID:PID 0416:5051`锁定F407 CMSIS-DAP，写入
  Debug BIN 50068 B，并从`0x08000000`完整回读；源BIN与回读SHA-256均为
  `6E622160C84C63F7053D44B30CC86B74DBACCC9A8767E2ACE78F436C2E263D6A`，随后`reset run`。
  未按KEY或发送启动命令。
- 已知限制与待确认：烧录和回读不等于动态控制验收；需用新版VOFA确认减速段CH22接近规划
  `-99 mm/s²`且保留BMI瞬态、CH23连续、bit3饱和率下降，并确认CH24不再持续卡在`793 mdeg`。

## 2026-07-31：M4停车负规划增加小系数确定性前馈

- 修改原因：最新VOFA中M4约`7.42 s`通过B，满足8 s约束但仅余约0.58 s；3519规划减速度
  `-99 mm/s²`平滑，实际停车冲击却达到约`-312 mm/s²`并快速反向。用户要求不改变AB时间和
  距离规划，只让停车规划承担一部分补偿，剩余交给加速度与视觉闭环。
- 最终决策：保持3519的`320 mm/s`、`100 mm/s²`、`400 mm/s³`及B后50 mm目标不变。F407
  仅在心跳在线、M4、CAR新鲜且负规划有效时，按固定增益`0.25`额外叠加确定性停车前馈；
  BMI/融合主增益保持`0.20`，正加速、规划失鲜和其他赛题不叠加。规划项与融合项相加后仍受
  `±0.60°`底盘补偿限幅，总偏移继续受`-2.00°..+0.75°`边界和raw保护约束。
- 涉及文件：`APP/Gateway/app_gateway.c`、`config/core_config.h`、`README.md`及本记录；未修改
  G3519固件、线上协议、Task周期、状态机、时间/距离规划、ISR/DMA、引脚或电机保护。
- 验证方式和结果：按最新VOFA离线回放，新增项最大约`+0.145°`，负规划区间预计总杆命令峰值
  约`+0.511°`，低于`+0.75°`上限。F407 Debug/Release clean-first完整链接通过，FLASH/RAM
  分别为`50148/20792 B`和`43336/20776 B`；`git diff --check`通过。使用`VID:PID 0416:5051`
  锁定F407 CMSIS-DAP，写入Debug BIN 50148 B，并从`0x08000000`完整回读；源BIN与回读
  SHA-256均为`8177C05D519677E8D56CD6C8F26676F0FC75FF9502B7E64EBC6317649698FB86`，随后
  `reset run`。未按KEY或发送启动命令；G3519固件未变，未重复烧录。
- 已知限制与待确认：离线回放和镜像回读不能替代实车球位`±1 cm`验收；新一轮须同时检查B点
  时间、球位误差、CH23/CH24连续性及CH31 bit3/bit4。

## 2026-07-31：滚球静摩擦辅助门槛降至8 mm

- 修改原因：最新AB视觉数据中球位稳定在目标左侧约`11.10 mm`，标准差仅约`0.25 mm`，但原
  `12 mm`严格进入门槛始终不满足，静摩擦速度辅助量保持为0。
- 最终决策：仅将`GATEWAY_BALL_STICTION_MIN_ERROR_MM`由`12.0 mm`降至`8.0 mm`；保持
  `Kp=0.60、Ki=0、Kd=0.55`、速度辅助增益、渐变距离、杆角限幅及全部状态逻辑不变。误差约
  `11.1 mm`且低速持续100 ms时，辅助目标速度约`8.6 mm/s`，随后随误差接近8 mm连续减小。
- 涉及文件：`config/core_config.h`及本记录；未修改Task、协议、ISR/DMA、硬件映射或保护逻辑。
- 验证方式和结果：MSYS2 GCC 16.1以`-Wall -Wextra -Wpedantic -Werror`重新编译并运行
  `control_polarity_chain`测试，结果PASS；ARM Debug/Release完整链接无警告，FLASH/RAM分别为
  `50148/20792 B`和`43336/20776 B`；`git diff --check`通过。使用`VID:PID 0416:5051`
  锁定F407 CMSIS-DAP，识别Cortex-M4、DEV_ID尾码`0x413`及512 KiB Flash；写入Debug BIN
  50148 B并从`0x08000000`完整回读，源BIN与回读SHA-256均为
  `721757AEE6DAE606860B2E3519F9C6F4F57DBB9ECBA04D6C6A4FC8151A29491B`，随后`reset run`。
  未按KEY或发送启动命令；G3519未操作。
- 已知限制与待确认：尚未进行实车闭环验证；下一轮需确认辅助状态实际进入、球能越过约11 mm
  静差且不在8 mm边界形成往复动作。若仍停在3～8 mm，再单独评估极小Ki。

## 2026-07-31：M3禁止底盘加速度前馈

- 修改原因：用户明确要求第三题不引入加速度前馈；原逻辑在无M4规划时默认使用F407本地
  BMI088，因此M3仍会产生底盘补偿杆角。
- 最终决策：现有1 ms Task读取最近1.5 s内有效的CAR赛题号；mode 3时不向Gateway提交
  BMI088加速度、融合预测或规划前馈有效位。BMI088仍持续采样、滤波和维护零偏，视觉球位
  闭环、杆角内环、启停及保护逻辑不变；M4/M5/M6现有路径不变。
- 涉及文件：`Task/task_app.c`、`config/core_config.h`、根README、PeerLink README、板间协议
  文档及本记录；未增加协议字段、状态机、Task、动态内存或周期变化。
- 验证方式和结果：PeerLink CAR/mode严格主机测试PASS；ARM Debug/Release完整链接无警告，
  FLASH/RAM分别为`50212/20792 B`和`43400/20776 B`；`git diff --check`通过。使用
  `VID:PID 0416:5051`锁定F407 CMSIS-DAP，识别Cortex-M4、DEV_ID尾码`0x413`及512 KiB
  Flash；写入Debug BIN 50212 B并从`0x08000000`完整回读，源BIN与回读SHA-256均为
  `8D803BD88759E5365A43E64C3ABBDD22630ACF27CFAC94189DB9A2991DEB1874`，随后`reset run`。
  未按KEY或发送启动命令；G3519未操作。
- 已知限制与待确认：尚未实车运行。M3实测应确认CH22加速度、CH23补偿角及加速度有效状态位
  为0，同时CH24仍保留视觉球位闭环输出。若CAR mode超过1.5 s未更新，模式信息按现有超时
  语义失效，系统恢复默认本地BMI088路径。

## 2026-07-31：破静摩擦速度环与位置环平滑交接

- 结论：用户否决`Kp/Ki/Kd=0.80/0/1.20`及速度辅助与位置环直接叠加的方向；当前位置环已
  恢复Git基线`0.60/0/0.55`，破静摩擦环已改为速度PI，并在靠近目标时平滑切回位置环。
  固件已烧录，电机未启动。
- 修改原因：30 s动态采样中静摩擦辅助长期顶在`245 mdeg`，且与位置PD相加；用户明确要求
  速度环仅用于脱离静摩擦，靠近目标点后平滑、迅速转换成位置环逼近。
- 最终决策：原速度P输出固定卡在`30 mm/s²`、`245 mdeg`，不能继续增大克服静摩擦，已被
  否决。保留`|误差|>3 mm`、`|球速|<=8 mm/s`、100 ms进入门控和`30 mm/s`目标球速；速度
  PI使用`Kp=1.0`、`Ki=10.0`，静止时积分增大，最大加速度`180 mm/s²`约对应`1.47 deg`。
  检测到球速超过`8 mm/s`立即清速度积分；在`3..11 mm`交接带按smoothstep切回位置环，
  两环不直接相加。离散脉冲、Kalman球状态融合继续关闭，其余保护不变。
- 涉及文件：`APP/Algorithm/BallPositionControl/algorithm_ball_position_control.[ch]`、其README、
  `config/core_config.h`、滚球控制测试及本记录；未修改Task、通信协议、G3519链路或底盘
  加速度前馈。
- 验证方式和结果：新增“速度积分在静止时增长、起滚后立即清零”及远端独占/交接断言；
  BallPositionControl与control polarity chain严格主机测试均PASS；ARM Debug clean-first链接
  通过，FLASH/RAM为`50440/20800 B`。烧录前VST为`command_ma=0`、
  `speed_rpm=0.00`且`CONTROL_ARMED/MOTOR_ENABLED`均未置位；CMSIS-DAP `0416:5051`写入
  Debug ELF并复位，OpenOCD报告`Programming Finished`。按用户要求未做verify/readback，
  未按KEY或发送启动命令。
- 已知限制与待确认：尚未动态验证；最近20 s视觉数据有60次单帧跳变超过12 mm，动态结果
  需同时区分视觉误跳和控制响应。上机重点检查起滚积分斜坡、首次运动后积分清零、交接连续性
  和`±3 mm`保持。

## 2026-08-01：双端滤波轮速加速度融合与 M6 任意球位权限保护

- 结论：G3519 已生成并通过 CAR 回传滤波轮速加速度；F407 已将其作为独立无 IMU 零偏观测加入三状态融合器。M6 不按球位置放大平移前馈，球位环优先占用杆角权限，底盘融合补偿只用剩余余量。
- 修改原因：VOFA 表明原单步轮速差分存在持续正偏，BMI088 单独提高增益会放大噪声；最终题还要求钢球处于任意目标位置时保持控制权限。
- 最终决策：CAR 新增兼容 13 字段尾部 `filtered_wheel_acceleration,wheel_accel_valid`，保留 8/9/10/11 字段；融合器新增 `H=[0,1,0]`、方差 `6400 (mm/s²)²` 的轮速加速度校正。quiet-hold 使用 5 mm/12 mm/s 退出滞环并连续 3 帧确认，误差始终相对当前任意目标计算。
- 涉及文件：`Device/PeerLink`、`Task/task_app.c`、`APP/Algorithm/ChassisAccelerationFusion`、`APP/Algorithm/BallPositionControl`、`APP/Gateway`、`config/core_config.h`、协议/测试/本记录；未修改 Task 周期、优先级、BSP、ISR/DMA、引脚或动态内存策略。
- 验证：PeerLink、融合器、球位控制主机测试通过；F407 Debug/Release 完整链接通过，BIN 分别为 52068/45048 B。
- 已知限制与待确认：本轮未烧录。测量方差 6400、100 ms 滤波和 3 帧退出为保守初值，仍需双端烧录后用新 VOFA 对起步、巡航、弯道和停车实测；旧协议下融合器继续工作但没有轮速加速度增强观测。

## 2026-08-01：滤波轮速加速度融合双端部署

- 结论：匹配的 F407 与 G3519 Bring-up Debug 镜像已完成烧录、全长回读校验并复位运行；未按 KEY，未启动底盘或滚球闭环。
- 修改原因：13 字段 CAR 扩展要求双端同步部署，用户明确要求烧录。
- 最终决策：F407 使用 `0416:5051` CMSIS-DAP 写入 `52068 B` Debug BIN；G3519 使用 `0D28:0204` DAPLink 写入 `87456 B` Bring-up Debug BIN。G3519 通用 Flash 入口曾短暂写入 `18792 B` AT8236 基础镜像，识别后立即由 Bring-up 产品镜像完整覆盖，最终回读只对应 Bring-up 镜像。
- 涉及文件：仅更新本记录；没有修改控制参数、Task、协议、BSP、ISR/DMA 或硬件配置。
- 验证方式和结果：F407 源 BIN 与 `0x08000000` 起始的 `52068 B` 回读 SHA-256 均为 `55C8ED9F1AA3B3DB00A9E0714E7099C2FFC3C63661F4B98722EE3B2C35C8A953`；G3519 源 BIN 与 `0x00000000` 起始的 `87456 B` 回读 SHA-256 均为 `91EB1A2F0A2CFCCE10496F84892FF07E4F0BBEDF2B7FB41A5B51EC1B549CA644`。复位后 COM5 连续收到 34 列 VOFA，停止态 `CH30=0`、`CH31=552`、`CH32=0`、`CH33=0`，证明 DBG 新鲜且新增通道存在，规划退出时轮速加速度及有效位为零。
- 已知限制与待确认：静态帧显示滚球电机 online/enabled 位均未置位且杆角饱和位已置位；本轮未做动态闭环或赛道验证。恢复电机在线后需检查 M4/M5/M6 运行期 CH32/CH33、CH22/CH23 及 M6 端部杆角余量。

## 2026-08-01：轻量杆球模型插值与速度环连续化

- 结论：F407 已用轻量两状态杆球模型在视觉帧间传播球位和球速；真实视觉继续作为绝对位置校正，底盘加速度仍作为独立杆角前馈。速度 PI 目标改为连续限斜率，并允许位置环锁存后在确认静摩擦稳态误差时重新脱困。Debug ELF 已烧录并复位，尚未按 KEY 或进行动态上机验证。
- 修改原因：约 30 Hz 的视觉位置存在采样延迟、单帧无效和跳点，直接差分速度使高频控制间歇且阶跃明显；已有位置环切入后若球因静摩擦停在目标外，速度 PI 无法再次介入。用户要求先采用简单模型插值和独立加速度补偿，不恢复三状态 Kalman 融合。
- 最终决策：1 ms 控制周期按实际杆角减去水平 trim、并结合已选底盘加速度传播 `[x,v]`；真实 YOLO 按测量年龄做固定滞后 alpha-beta 校正，`alpha=0.65`、`beta=0.08`、绝对创新门 `12 mm`，连续接受 2 帧后可信，超过 `100 ms` 无实测即停用。无效 VIS 只更新链路统计，不覆盖或刷新最后有效测量。速度 PI 目标按真实 `delta_ms` 和 `180 mm/s2` 限斜率；位置环锁存后，同一目标下 `|误差|>8 mm`、`|球速|<=8 mm/s` 持续 `250 ms` 才重入速度 PI。Task3 继续旁路，离散角度脉冲继续关闭。位置环保持 `Kp/Ki/Kd=0.60/0/0.55`；底盘融合主前馈增益由 `0.20` 小步提高到 `0.25`，原限幅和 M3 禁用规则不变。
- 涉及文件：新增 `APP/Algorithm/BallModelInterpolator/` 和 `tests/ball_model_interpolator/`；修改 `APP/Gateway/app_gateway.c`、`APP/Algorithm/BallPositionControl/`、`Device/VisionLink/`、`config/core_config.h`、`CMakeLists.txt`、根 README 及对应测试。未修改 MaixCAM、协议字段、Task 周期/优先级、BSP、ISR/DMA、引脚或电机启停逻辑。
- 验证方式和结果：BallPositionControl、control polarity chain、VisionLink、BallModelInterpolator 主机测试均 PASS；ARM Debug/Release clean-first 完整链接通过，FLASH/RAM 分别为 `54432/20944 B` 和 `47028/20928 B`；`git diff --check`通过。用户确认电机失能后，使用 `VID:PID 0416:5051` 锁定 F407 CMSIS-DAP，OpenOCD 写入 Debug ELF 并报告 `Programming Finished`，随后复位；按用户要求未做 verify/readback，未按 KEY 或发送启动命令。
- 已知限制与待确认：模型参数、前馈增益及重入条件尚未实车验证。上机应检查视觉短失效时杆角连续性、模型球速方向与幅值、静摩擦重入、交接连续性、超调及任意目标停车误差。采样文件 `vofa+.csv`、`vofa+1.csv` 不提交。

## 2026-08-01：Task3复用连续速度PI并烧录

- 结论：Task3已取消滚球速度旅程旁路，远距离目标与其他赛题共用现有连续速度PI；Debug固件已完成校验、全长回读和复位，Maix视觉已重启，电机保持失能。
- 修改原因：Task3实测在`+50 mm`目标下8 s超时，视觉位置约`-5.3 mm`且电机速度接近0；原逻辑明确旁路速度PI，只靠小位置杆角无法克服静摩擦。用户要求只修Task3并复用其他赛题的速度环。
- 最终决策：Gateway始终启用BallPositionControl已有速度旅程。沿用`30 mm/s`速度上限、`180 mm/s²`加速度限斜率和速度`Kp/Ki=1.0/10.0`；进入3 mm低速区后仍交给原位置环，离散kick、角环参数、底盘前馈和Task4/5/6参数均不变。
- 涉及文件：`APP/Gateway/app_gateway.c`、`APP/Algorithm/BallPositionControl/README.md`、`config/core_config.h`、根`README.md`及本记录；未修改Task、协议、BSP、引脚、Maix业务代码或其他赛题参数。
- 验证方式和结果：BallPositionControl、control polarity chain和BallModelInterpolator严格主机测试均PASS；ARM Debug/Release clean-first完整链接通过，FLASH/RAM分别为`54416/20944 B`和`47012/20928 B`；`git diff --check`通过。F407 `0416:5051` CMSIS-DAP写入Debug BIN并报告`Verified OK`，从`0x08000000`回读54416 B，源镜像与回读SHA-256均为`D0FCBF0BD2E62AFEABFB94D25E09B2AAD59BC3247D3199422C86C341B61F2E16`。Maix应用和三份模型重启成功。
- 已知限制与待确认：尚未进行本镜像的Task3动态往返；当前G3519 mode为0，复位后VST确认命令0、速度0且电机失能。需切到Task3并启动，检查`0 -> +50 -> -50 mm`耗时、起滚积分、交接和停车误差。

## 2026-08-01：Task3专属提速、收紧停车带并改用真实视觉球速

- 结论：Task3速度/加速度上限提高到`45 mm/s`和`300 mm/s²`，速度切环与安静保持误差收紧到`1.5 mm`，并绕过轻量杆球模型，直接使用真实VIS的`120 ms`多帧差分和`60 ms`低通球速；Task2/4/5/6继续使用原模型及`30/180/3 mm`参数。Debug固件已烧录、完整回读并复位，Maix视觉已恢复，未启动电机。
- 修改原因：上一版Task3往返能够完成，但速度偏慢；球实际停止后模型仍输出约`30..33 mm/s`虚假速度，导致位置环无法正确停车并最终回弹到约`-37.1 mm`，相对`-50 mm`目标误差约`12.9 mm`。
- 最终决策：新增有校验的运行时运动上限和最终位置带API；只有参数真正变化时清速度PI积分、目标球速、旅程和安静保持状态。Gateway只在新鲜`competition_mode=3`时选用Task3参数与真实视觉路径；进入/退出Task3时清旧反馈源历史。其他赛题的数据源、参数、角环PID、底盘前馈、Task周期和启停保护均不变；Task3模型球速方向已被本次实测否决。
- 涉及文件：`APP/Algorithm/BallPositionControl/algorithm_ball_position_control.[ch]`及README、`APP/Gateway/app_gateway.c`、`config/core_config.h`、滚球控制测试、根README和本记录；未修改MaixCAM、协议、BSP、引脚、Task逻辑或其他赛题参数。
- 验证方式和结果：BallPositionControl、control polarity chain和BallModelInterpolator均以严格主机编译实际运行PASS；ARM Debug/Release clean-first完整链接通过，FLASH/RAM分别为`55552/20952 B`和`47876/20928 B`；`git diff --check`通过。烧录前发现旧固件仍在phase 2且电机使能，先复位使其执行启动期Disable并由DBG确认phase 0、命令/反馈0 mA、转速0、enabled=0；锁定`VID:PID 0416:5051`写入Debug BIN并`Verified OK`，从`0x08000000`完整回读55552 B，源镜像与回读SHA-256均为`6980B12728F80C63981188410092C9B6D053A8DCC9DF26D00498FD0B1B0371F4`，随后`reset run`。Maix应用和三份模型恢复后，VST确认`mode=3`、`fault=0`、task_active=0、motor_enabled=0、命令0 mA、转速0 rpm。
- 已知限制：真实视觉测速约需`120 ms`建立窗口，停车速度仍存在视觉采样和低通延迟；软件测试与完整链接不能证明实机最终误差。
- 待确认事项：只测试Task3的`0 -> +50 -> -50 mm`，确认往返耗时、穿越峰值、最终稳定误差以及静止后回传球速能否收敛到0，再决定是否继续调整`45/300/1.5`。

## 2026-08-01：M6连续位置—速度串级与可执行杆角限幅

- 结论：M6速度PI输出上限已按较弱的`+0.75°`杆角权限收紧到`80 mm/s²`；速度串级在越过目标后清旧积分并连续反向，只在误差进入最终位置带且球速足够低时交给位置/安静保持路径。Debug固件已烧录并复位，未启动电机。
- 修改原因：实机观察到位置控制长期撞限幅。原M6速度PI允许`180 mm/s²`，对应约`1.47°`杆角，明显超过正向可执行权限并造成下游限幅外积分积累；原方向变化还会把仍在运动的球过早交给弱位置PD。
- 最终决策：只为`competition_mode=6`选择`80 mm/s²`最大目标加速度，目标速度仍为`30 mm/s`；取消`8 mm + 250 ms`速度环重入门槛，误差超过最终位置带或球速过高即保持/恢复速度串级；误差方向变化时清速度PI状态并按新方向继续。离散脉冲保持关闭，`Kp/Ki/Kd=0.60/0/0.55`、底盘加速度前馈、视觉模型插值、Task周期、协议和启停保护不变。
- 涉及文件：`APP/Algorithm/BallPositionControl/algorithm_ball_position_control.[ch]`、`APP/Gateway/app_gateway.c`、`config/core_config.h`、滚球控制测试和本记录。
- 验证方式和结果：MSYS2 GCC 16.1以`-Wall -Wextra -Wpedantic -Werror`运行BallPositionControl测试PASS，新增覆盖最终带外立即重入及越过目标后清积分反向；ARM Debug clean-first完整链接通过，FLASH/RAM为`55520/20952 B`，BIN SHA-256为`0F86642FB58124EF56706ED3424C15131CFDC321517278F883F14D8722840A2C`；`git diff --check`通过。用户确认电机失能后，OpenOCD锁定CMSIS-DAP `0416:5051`写入Debug ELF并报告`Programming Finished`，随后复位；按用户要求未做verify/readback。
- 已知限制与待确认：尚未进行动态实机验证。M6应先在非零目标做短距离测试，观察VST杆角是否退出持续限幅、球速估计是否在停车后回到低速以及`±3 mm`最终带能否稳定进入；若仍限幅，下一步只检查模型速度/D项，不继续提高位置Kp。

## 2026-08-01：M6限时助启动后由位置PD追踪

- 结论：连续速度串级因实机左右往复已被用户否决。当前速度PI只提供至少`250 ms`的助启动；达到最短时间且球速超过`8 mm/s`后清速度积分，后续由位置PD追踪。Debug固件已烧录并复位，未启动电机。
- 修改原因：上一版速度环在目标两侧连续反向造成左右晃动；用户明确要求加速度只用于启动，追踪仍依靠位置环，同时要求助启动不能过短。
- 最终决策：新目标误差超过`8 mm`时立即进入助启动并至少保持`250 ms`；若提前进入最终误差带或越过目标则立即退出，避免继续推过头。位置PD接管后，只有低速且误差持续超过`8 mm`达`250 ms`才再次助启动。M6助启动加速度上限继续保持`80 mm/s²`，离散脉冲保持关闭，位置`Kp/Ki/Kd=0.60/0/0.55`和底盘加速度前馈不变。
- 涉及文件：`APP/Algorithm/BallPositionControl/algorithm_ball_position_control.[ch]`、`config/core_config.h`、滚球控制测试和本记录；未修改协议、Task、BSP、引脚或启停保护。
- 验证方式和结果：BallPositionControl以MSYS2 GCC 16.1严格编译运行PASS，覆盖最短助启动时间、运动后退出和持续卡滞重入；ARM Debug clean-first完整链接通过，FLASH/RAM为`55584/20952 B`，BIN SHA-256为`7005B4CB0D9BC1840403F6576194FE0E64A9C6D999837B0D27B8350C7AD52357`；`git diff --check`通过。烧录前先复位旧固件并留出`750 ms`执行启动期Disable，随后锁定CMSIS-DAP `0416:5051`写入Debug ELF，OpenOCD报告`Programming Finished`并复位；未做verify/readback。
- 已知限制与待确认：尚未实机确认250 ms是否足以稳定起滚以及位置PD接管后的阻尼。下一次测试只观察助启动退出时刻、左右换向次数和最终误差，不同时改变Kp/Kd。

## 2026-08-01：M5/M6底盘加速度前馈连续置信度门控

- 结论：M5/M6继续完整采用G3519的jerk限幅规划加速度；BMI088/轮速融合只作为受轮速证据约束的残差修正，CD匀速和弯道切换噪声不再直接形成大杆角。Debug固件已烧录并复位，未启动电机。
- 修改原因：桌面`1.csv`中CD规划加速度全程为0、滤波轮速加速度均值为`0.01 mm/s2`，但F407融合值达到`-146..+379 mm/s2`并产生`-0.553..+0.213 deg`补偿，返程重新推动钢球。直接按路段清零会丢失真实扰动并产生模式切换，已被用户否决。
- 最终决策：输出采用`a_plan + LPF(w*(a_fusion-a_plan))`。轮速绝对加速度在`25..80 mm/s2`间用smoothstep建立证据权重，融合与轮速差在`80..200 mm/s2`间平滑撤销一致性权重；残差限于`+-100 mm/s2`并用`120 ms`一阶低通。只作用于competition mode 5/6；规划路径不经过该低通，M4及其他模式保持原行为。
- 涉及文件：`APP/Gateway/app_gateway.c`、`config/core_config.h`和本记录；未修改协议、Task周期/优先级、状态机、BSP、引脚、保护边界或电机启停逻辑。
- 验证方式和结果：桌面`1.csv`离线回放中CD门控输出由原`-146..+379`降至约`-2.3..+0.3 mm/s2`，启动和刹车规划仍保留；BallPositionControl、ChassisAccelerationFusion、BallModelInterpolator和control polarity chain严格主机测试均PASS。ARM Debug/Release clean-first完整链接通过，FLASH/RAM分别为`56048/20960 B`和`48420/20944 B`，Debug BIN SHA-256为`3A6E3B221DDA02BEFA447B5750642312A9FB792CF1DD59C063CD6BA35D0A4E32`；`git diff --check`通过。用户确认电机失能后，OpenOCD锁定`0416:5051`写入Debug ELF、报告`Programming Finished`并复位；按要求未做verify/readback，未按KEY。
- 已知限制与待确认：CSV回放不能替代实车；下一次M5/M6整圈应重点观察CD的CH22/CH23、球位是否保持，以及AB启动和最终刹车是否仍能抵消底盘惯性。

## 2026-08-01：M6仅在非零规划阶段启用底盘加速度补偿

- 结论：上一版M5/M6连续置信度门控因半圆实测异常被用户否决并撤销。M5恢复原融合行为；M6仅在G3519新鲜规划`I6!=0`的启动和停车阶段使用原融合补偿，`I6==0`巡航时最终选用加速度与补偿角均强制为0，只保留视觉球位环。
- 修改原因：用户确认直线抑制有效但半圆控制劣化，并明确选择最初的“仅启动/停车补偿”策略，不接受额外赛道分段协议或连续置信度门控。
- 最终决策：不改CAR协议、不按ABCD建立新状态；在F407原融合计算完成后，仅对`competition_mode=6 && plan_valid && planned_acceleration==0`覆盖最终选用加速度为0。规划非零、M2/M4/M5及其他原逻辑保持不变。
- 涉及文件：`APP/Gateway/app_gateway.c`、`config/core_config.h`和本记录；未修改G3519、MaixCAM、通信协议、Task、状态机、BSP、引脚或启停保护。
- 验证方式和结果：ARM Debug clean-first完整链接通过，FLASH/RAM为`55616/20952 B`，BIN SHA-256为`4F12E876E6BEB27EE6BB63DD157FE9E6030E2641A2D6723B749B75545B893E64`；`git diff --check`通过。用户确认电机失能后，OpenOCD锁定`0416:5051`写入Debug ELF、报告`Programming Finished`并复位；按要求未做verify/readback，未按KEY。
- 已知限制与待确认：该策略会在M6所有零规划巡航阶段（包括直线与半圆）关闭底盘加速度补偿；需实车确认半圆仅靠视觉球位环是否满足要求，并检查`I6`过零切换时的杆角变化。

## 2026-08-01：M5/M6四段独立加速度融合

- 结论：上一版“M6零规划强制补偿为0”已按用户决定撤销。AB/BC/CD/DA四段均恢复原加速度融合，每次跨段先清空F407融合历史，再用当前新段样本重新建立状态。
- 修改原因：用户确认直线段仍需要补偿，但半圆动态会污染下一段融合状态，因此要求四段互相隔离而不是关闭某类路段补偿。
- 最终决策：CAR在现有13字段完整格式尾部追加`route_segment=0..3`形成14字段；F407继续兼容旧8/9/10/11/13字段。M5/M6首次收到段号或段号变化时，在Predict/Correct前复位融合器并清三个观测身份；规划失效或离开M5/M6时清段号历史。未改视觉位置环、融合参数、Task周期、保护和KEY语义。
- 涉及文件：双端`Device_PeerLink`、G3519 `task_bringup_chassis.c`、F407 `task_app.c`/`app_gateway.[ch]`、协议测试、配置和文档。
- 验证方式和结果：双端PeerLink严格主机测试PASS；G3519 Bring-up Debug完整链接通过，FLASH/SRAM_BANK0为`87864/19472 B`；F407 Debug完整链接通过，FLASH/RAM为`55888/20952 B`。用户确认电机失能后，F407 `0416:5051`与G3519 `0D28:0204`均完成编程和复位；按要求未verify/readback，未按KEY。F407/G3519源BIN SHA-256分别为`0AB6C5F5133DF41C08C42A035CB76EE690D6C68BF7D3EC18127C00B2BD9BE9C3`和`B85866654884F494236C5D95202883080A7927C960698E9E501C06D16808E21E`。
- 已知限制与待确认：尚未实车运行验证段号边界和换段瞬态。下一次整圈重点观察B/C/D/A前后补偿是否短暂重新建立、直线球位保持及半圆是否不再把残差带入下一段。

## 2026-08-01：M5/M6最终静态规划—轮速加速度融合

- 结论：四段复位实测更差，已被用户否决并从控制中移除。M5/M6最终采用无状态静态融合：大加减速相信规划，匀速只保留小幅低通轮速加速度补偿，视觉位置环负责球位收敛。
- 修改原因：桌面`1.csv`三圈中，BC/CD规划加速度为0、滤波轮速加速度标准差仅约`12..13 mm/s²`，但原融合输出标准差仍有`56..66 mm/s²`且尖峰达到`-233..+429 mm/s²`；D点附近融合尖峰把补偿角打到`-0.60°`。分段只能清历史，不能阻止每段重新受BMI088/轮速状态耦合污染。
- 最终决策：M5/M6使用`w=clamp(|a_plan|/50,0,1)`和`a_selected=a_plan+0.50*(1-w)*a_wheel_filtered`；M4保留原Kalman融合。离线回放预测BC/CD选用加速度标准差降至约`6.1/6.5 mm/s²`、范围约`-20..+15 mm/s²`，而起步/刹车`±100 mm/s²`规划不变。F407兼容接收当前3519的14字段，但只校验并丢弃旧段号；新3519源码恢复13字段。
- 涉及文件：F407 `APP/Gateway/app_gateway.c`、`config/core_config.h`、`Device/PeerLink`、协议测试/文档和本记录；G3519撤销段号发送源码。未改视觉位置环参数、底盘速度规划、Task周期、保护或KEY语义。
- 验证方式和结果：双端PeerLink严格主机测试PASS；F407 Debug完整链接通过，FLASH/RAM为`55904/20952 B`，BIN SHA-256为`4D563E1FCF61B34FC7110FDAA4F6ECEB500AD23D5338CA4DA372D787AEEB956B`。用户确认电机失能后，锁定`0416:5051`写入F407并复位，未verify/readback、未按KEY。3519 `0D28:0204`烧录时未枚举，因此未重写；其当前14字段固件已由F407兼容解析，不阻断本轮控制测试。
- 已知限制与待确认：离线回放只证明补偿输入明显降噪，不能证明球位实机最优。下一圈只观察BC/CD是否停止乱摆、D点是否不再出现补偿限幅、起步与停车是否仍有效；不再同时修改位置环参数。

## 2026-08-01：整圈欠阻尼数据复核与位置PD调参

- 结论：桌面最新`1.csv`表明问题并非只在半圆，BC和CD均存在球位置环持续往返修正；位置`Kp`由`0.60`降为`0.50`，速度阻尼`Kd`由`0.55`提高为`1.10`，Debug固件已烧录并复位，未按KEY。
- 修改原因：BC/CD的加速度补偿角仅约`0.011/0.009 deg RMS`，视觉与F407数据基本连续且没有杆角限幅；同期球环自身杆角约`0.214/0.271 deg RMS`并持续换向，说明当前主要矛盾是位置PD欠阻尼。用户补充要求不能把直线视为已通过，因此整圈各段均使用同一组新PD参数。
- 最终决策：只修改`GATEWAY_BALL_POSITION_KP_PER_S2=0.50`和`GATEWAY_BALL_VELOCITY_KD_PER_S=1.10`；`Ki=0`、轻量模型插值、静摩擦速度辅助、加速度前馈、分段、任务状态机、协议、Task周期和保护均不变。
- 涉及文件：`config/core_config.h`及本记录。
- 验证方式和结果：BallPositionControl、control polarity chain、BallModelInterpolator严格主机测试均PASS；ARM Debug clean-first完整链接通过，FLASH/RAM为`55904/20952 B`，`git diff --check`通过，BIN SHA-256为`92BC029C4C14DCA867527A2E0FE815E2C78C02A344D9519FC1486F1FBDC1718F`。烧录前串口仍为`phase2/enabled/-83 mA`，先复位旧固件并确认`phase0/disabled/0 mA/0 rpm`，再锁定F407 CMSIS-DAP `0416:5051`写入Debug ELF并复位；OpenOCD报告`Programming Finished`，按用户要求未verify/readback。复位后串口仍为`phase0/disabled/0 mA/0 rpm`，视觉与DBG帧在线。
- 已知限制与待确认：本轮尚未动态验证。下一次整圈应同时观察AB、BC、CD、DA的球位峰值、换向次数和进入`±10 mm`所需时间；若仍振荡，先核对模型球速与真实视觉位移方向和幅值，不再直接提高Kp。

## 2026-08-01：Task3参考视频匀速抬杆与越点回程

- 结论：Task3已改为一次性“原点附近确认、匀速抬杆、画面越过`+5 cm`、立即位置PD回程”的策略；代码和双配置完整链接已完成，未烧录、未启动实物。
- 修改原因：连续位置PD从原点直接追踪首段目标仍可能受静摩擦影响。用户要求复现参考视频思路，允许原点误差，先让杆匀速上升，观测球越过`+5`点后立即向下，并让回程位置环参数与其他赛题统一。
- 最终决策：Maix画面目标`0 -> +50 -> -50 mm`按现有坐标契约对应F407物理目标`0 -> -50 -> +50 mm`。只有收到首段`-50 mm`目标、新鲜视觉球位在画面原点`±10 mm`内时才启动；杆角偏置按真实时间以`+0.40 deg/s`上升并限于`+0.60 deg`。首个新VIS确认画面位置`>=+50 mm`，或回程目标已经到达时，同周期清安静保持/速度辅助状态但保留球位与真实视觉球速，立即使用公共`Kp/Ki/Kd=0.50/0/0.75`位置PD追踪物理`+50 mm`。视觉失效会撤销开环偏置并回到原点等待，不自动重启；停止、回零和退出Task3均复位该一次性序列。
- 涉及文件：新增`APP/Gateway/task3_rod_sequence.[ch]`及严格主机测试；修改Gateway接入、BallPositionControl位置环状态清理API、配置、CMake和说明文档。未修改MaixCAM、G3519、通信字段、Task周期/优先级、底盘前馈、电机角环、BSP、引脚或既有机械/raw/电流保护。
- 验证方式和结果：Task3RodSequence与BallPositionControl均以`-Wall -Wextra -Wpedantic -Werror`严格主机编译运行PASS；Debug/Release clean-first完整链接通过，FLASH/RAM分别为`57092/20992 B`和`49064/20976 B`；`git diff --check`通过。Debug/Release BIN SHA-256分别为`487002D15DC86EF575696ACE052C04BD0392BED3A6A2D0CC69393A0DAE1651ED`和`071AD46A747D6EDAF3BABC89BF1835561D86120AD2A7F42EDCEEFBF1A03B1195`。
- 已知限制与待确认：尚未进行动态硬件验证，`±10 mm`原点容差、`0.40 deg/s`抬升速度和`+0.60 deg`上限需要实物确认。首次测试只跑Task3，确认未收到首段目标时不抬杆、视觉失效立即回平、越过画面`+50 mm`当帧杆角反向，以及最终`-5 cm`停车误差；本轮不烧录。

## 2026-08-01：模型位置插值与真实视觉速度阻尼解耦

- 结论：模型插值模式不再把杆球模型预测速度送入位置环D项；模型只以1 ms周期补位置，阻尼速度仅由真实视觉位置的`120 ms`多帧差分和`60 ms`低通产生。`Kp/Ki/Kd`调整为`0.50/0/0.75`，Debug固件已烧录，未启动电机。
- 修改原因：桌面最新`1.csv`（08:57）中AB/BC/CD/DA球环杆角RMS约为`0.218/0.141/0.143/0.183 deg`，换向约`9/10/11/22`次，AB另有4次限幅；提高模型速度D项虽然降低BC/CD振幅，但起步和换段时的预测速度误差被`Kd=1.10`放大，不能继续用加大Kd解决。
- 最终决策：每个新VIS序列只更新一次独立视觉测速历史；视觉窗口尚未建立时D项无效，建立后模型高频位置更新保持该视觉速度及有效位，不再滤入模型速度。Kalman保持关闭，底盘加速度前馈、速度助启动、任务状态机、协议、Task周期和保护不变；`Kd`回落到`0.75`。
- 涉及文件：`APP/Algorithm/BallPositionControl/algorithm_ball_position_control.[ch]`、`APP/Gateway/app_gateway.c`、`config/core_config.h`、滚球控制测试和本记录；未修改MaixCAM或G3519。
- 验证方式和结果：新增“模型位置更新保持真实视觉速度”测试；BallPositionControl、control polarity chain、BallModelInterpolator均以严格主机编译实际运行PASS；ARM Debug链接通过，FLASH/RAM为`57092/20992 B`，`git diff --check`通过。烧录前COM5确认`phase0`、命令`0 mA`、反馈约`-1 mA`、速度`0 rpm`且使能位未置位；F407 CMSIS-DAP `0416:5051`写入Debug ELF并报告`Programming Finished`，随后复位，按要求未verify/readback。烧录镜像SHA-256为`487002D15DC86EF575696ACE052C04BD0392BED3A6A2D0CC69393A0DAE1651ED`。
- 已知限制与待确认：烧录后COM5未再收到34列DBG，随后F407探针也不再枚举，因此无法完成复位后的状态复核或动态整圈验证。重新连接F407/3519链路后先确认`phase0/disabled/0 mA/0 rpm`，再按KEY测试；新数据仍需同时评价AB、BC、CD、DA，不能只看弯道。

## 2026-08-01：Task3匀速抬杆固件烧录与完整回读

- 结论：当前Task3“原点门控、匀速抬杆、画面越过`+50 mm`立即位置PD回程”Debug固件已写入F407并完成全长回读校验；复位后保持停机失能，未启动Task3。
- 修改原因：用户在代码与构建完成后明确允许烧录。
- 最终决策：只烧录F407当前Debug镜像；精确锁定CMSIS-DAP `VID:PID 0416:5051`，未操作G3519、MaixCAM、相机进程或控制参数。写入后执行完整镜像校验和`57092 B`回读，再`reset run`使固件回到默认停止态；未按KEY或发送启动命令。
- 涉及文件：`build/Debug/f407_gateway.bin`、独立回读文件`build/hardware-test/f407_task3_487002d1_readback_20260801.bin`及本记录；未修改控制源码。
- 验证方式和结果：Debug clean-first完整链接通过，FLASH/RAM为`57092/20992 B`。烧录前COM3连续DBG为`phase=0`、命令`0 mA`、速度`0 rpm`且状态位未包含电机使能；OpenOCD识别Cortex-M4、DEV_ID尾码`0x413`和`512 KiB Flash`，写入并校验全部`57092 B`。源BIN与全长回读SHA-256均为`487002D15DC86EF575696ACE052C04BD0392BED3A6A2D0CC69393A0DAE1651ED`；复位后COM3再次确认`phase=0`、命令`0 mA`、速度`0 rpm`、电机未使能。
- 已知限制与待确认：本轮只验证镜像一致性和复位后安全停止状态，未进行Task3动态实物验证。后续需由用户按KEY启动，观察原点容差、实际匀速抬杆、越点反向和最终`-5 cm`停车误差。

## 2026-08-01：Task3实测接受为Git基准

- 结论：用户认为本次第三问表现“还可以”，明确要求以当前实现作为后续Task3基准并上传Git；当前控制参数和序列不再调整。
- 修改原因：烧录后的首次Task3数据已返回，用户选择先冻结可用版本，避免继续调参失去当前效果。
- 最终决策：基准保持原点`±10 mm`门控、`0.40 deg/s`匀速抬杆、画面`+50 mm`越点切换、公共`Kp/Ki/Kd=0.50/0/0.75`位置PD回程和M3底盘前馈关闭；不修改相机、G3519或协议。
- 涉及文件：当前F407控制源码、测试、配置、README和本记录；实测附件只用于离线判断，不纳入Git。
- 验证方式和结果：附件共134帧、2.663 s、严格34列且约50 Hz；全段`CH30=2`，`CH31=547`，表示F407 DBG新鲜、电机在线且使能、视觉球位有效，并且没有杆角饱和、raw边界、Peer/Vision/Task故障。M3的`CH22/CH23`全为0，确认底盘加速度前馈未参与。杆角命令`CH24`范围`-203..+337 mdeg`，目标raw范围`53979..54621`，出现明确换向；电机速度峰值约`+7.39/-4.33 rpm`，记录末端速度回到0。
- 已知限制与待确认：当前VOFA没有球位置和球目标通道，不能仅凭本附件定量证明画面越过`+50 mm`的准确时刻或最终`-50 mm`停车误差；记录末端目标/反馈raw仍相差约138 counts、命令电流约`-82 mA`。该限制不阻止用户将当前主观可用效果冻结为基准。

## 2026-08-01：真实视觉测速延迟缩短并烧录

- 结论：滚球真实视觉测速窗口由`120 ms`缩短到`60 ms`，速度低通时间常数由`60 ms`缩短到`20 ms`；`Kp/Ki/Kd=0.50/0/0.75`保持不变。Debug固件已写入F407并复位，未启动电机。
- 修改原因：MaixCAM完整算法实测约`24..27 Hz`，原测速差分与低通叠加造成明显延迟；用户明确要求缩短延迟并优先加快响应。`60 ms`阈值在当前帧率下一般仍选取约两帧、实际约`75..85 ms`的差分基线，避免直接退化为噪声较大的单帧差分。
- 最终决策：本轮只修改测速窗口和低通，不同时改变位置/速度增益、模型位置插值、Kalman开关、底盘前馈、任务状态机、协议、Task周期或保护。
- 涉及文件：`config/core_config.h`及本记录。
- 验证方式和结果：BallPositionControl、control polarity chain、BallModelInterpolator严格主机测试均PASS；ARM Debug clean-first完整链接通过，FLASH/RAM为`57092/20992 B`。烧录请求到达时旧固件仍为`phase2/enabled`且命令约`-79 mA`，先复位并由COM5确认`phase0`、命令`0 mA`、反馈约`-1 mA`、速度`0 rpm`、使能位未置位；随后锁定F407 CMSIS-DAP `0416:5051`写入Debug ELF，OpenOCD报告`Programming Finished`并复位，按要求未verify/readback。BIN SHA-256为`EF4004B7D6C179A4E85281560D5B31D0CD726D64DD0B0DA51A7756ADDD36DCC9`；复位后COM5仍为相同安全停止状态且DBG/视觉有效。
- 已知限制与待确认：尚未做动态响应验证；下一次测试应采集Maix真实球位以及COM5杆角/电机输出，重点确认更快速度估计是否减少慢刹车，同时没有放大单帧位置量化噪声。

## 2026-08-01：安静保持速度滞环放宽并烧录

- 结论：安静保持的位置入口/退出范围继续保持`3/5 mm`；入口球速由`8`放宽到`12 mm/s`、确认时间由`60`缩短到`40 ms`，退出球速由`12`放宽到`20 mm/s`。Debug固件已烧录并复位，未启动电机。
- 修改原因：桌面`1.csv`共5379帧、107.58秒；最终停车后约10.18秒仍持续调节，杆角RMS约`0.176 deg`、主振荡约`0.39 Hz`、电机速度RMS约`6.47 rpm`且峰值约`36.19 rpm`。上一轮`60/20 ms`视觉测速缩短延迟后放大了速度量化噪声，原`8 mm/s`入口门槛会反复清除保持候选，表现为始终不能进入死区。
- 最终决策：本轮只放宽安静保持的速度滞环和进入时间，不修改`Kp/Ki/Kd=0.50/0/0.75`、视觉测速`60 ms`窗口/`20 ms`低通、模型位置插值、速度助启动、底盘前馈、任务状态机、通信协议或保护。
- 涉及文件：`config/core_config.h`及本记录。
- 验证方式和结果：BallPositionControl、control polarity chain、BallModelInterpolator严格主机测试均PASS；ARM Debug clean-first完整链接通过，FLASH/RAM为`57092/20992 B`，BIN SHA-256为`A986C3AE15E298FF7DA411FED9EEAC78AD2A347F76F0734F7F5BDAF7B48C5E9D`。烧录前COM3连续DBG为`phase0`、命令`0 mA`、速度`0 rpm`、`status=1`（仅online，enabled未置位）；锁定CMSIS-DAP `0416:5051`写入Debug ELF并复位，OpenOCD报告`Programming Finished`，按用户要求未verify/readback。复位后仍为相同安全停止状态。
- 已知限制：34列VOFA数据不含真实球位、真实球速或`hold_active`，因此离线日志只能证明停车后执行机构仍振荡，不能直接确认是位置还是速度条件阻止进入；本轮参数判断结合了当前控制逻辑和真实视觉约`24..27 Hz`的已测事实。
- 待确认事项：由用户按KEY运行并重新采样；重点观察最终进入`±3 mm`后杆角和电机速度能否迅速归零。若仍不能锁定，下一轮必须增加只读`hold_candidate/hold_active`诊断或同步抓取Maix球位，不能继续盲调PID。

## 2026-08-01：停车底盘残余补偿门控与视觉位置校正提速

- 结论：M4/M5/M6在规划加速度为零且实测底盘轮速不超过`10 mm/s`时，强制采用底盘加速度归零；杆球模型真实视觉位置校正权重由`0.65`提高到`0.80`。Debug固件已烧录并复位，未启动电机。
- 修改原因：桌面09:47版`1.csv`共4128帧、82.56秒；末2秒小车命令通道全零，但F407仍采用约`-108.5 mm/s²`底盘加速度并生成约`+0.155 deg`补偿，杆角命令最高`+0.793 deg`，电机速度范围约`-15.78..+27.71 rpm`。静止不稳的直接扰动来自规划为零时仍叠加的轮速差分残余，而非需要继续增大位置PID。
- 最终决策：只在明确停车证据下屏蔽轮速加速度残余；非零规划或轮速超过门槛时原有前馈立即恢复。视觉提速采用增大模型位置alpha，不再缩短`60 ms`真实视觉测速窗口或`20 ms`速度低通，避免放大D项噪声和再次阻止安静保持进入。
- 涉及文件：`APP/Gateway/app_gateway.c`、`config/core_config.h`及本记录；未修改Task周期、通信协议、MaixCAM、G3519、PID、速度助启动、死区位置范围或保护。
- 验证方式和结果：BallPositionControl、BallModelInterpolator、ChassisAccelerationFusion及control polarity chain严格主机测试均PASS；ARM Debug clean-first完整链接通过，FLASH/RAM为`57140/20992 B`，BIN SHA-256为`05BDF1D4B809B19308F16D6C2CD0B934F384635D2263617A1E43AE0339EF910E`。烧录前后COM3均确认`phase0`、命令`0 mA`、速度`0 rpm`、`status=1`（仅online、enabled未置位）；锁定CMSIS-DAP `0416:5051`写入并复位，OpenOCD报告`Programming Finished`，按用户要求未verify/readback。
- 已知限制：新门控和alpha尚未进行运动实测；旧VOFA不含真实球位、目标、hold状态或F407接收轮速，不能由离线数据证明最终停车误差。
- 待确认事项：用户按KEY重新采样。停车后应重点检查CH22/CH23是否迅速归零、CH24和CH29是否停止换向；若底盘补偿已归零而球环仍抖，下一轮先增加只读hold诊断并同步抓Maix真实球位，再决定是否改位置环。

## 2026-08-01：速度助启动交接延后并烧录

- 结论：滚球速度PI最短助启动时间由`250`延长到`400 ms`，交回位置PD的实测球速门槛由`8`提高到`12 mm/s`。Debug固件已烧录并复位，未启动电机。
- 修改原因：桌面10:10版`1.csv`共5833帧、116.66秒。停车尾段8.74秒中，上一轮门控已将末5秒底盘补偿RMS由约`0.154 deg`降至`0.009 deg`，但球环杆角RMS仍约`0.222 deg`、电机速度RMS约`8.32 rpm`，并在约4.36秒和8.34秒出现短时大角度重入。说明剩余问题是速度助启动在刚产生小幅运动后过早交给不足以持续克服静摩擦的位置PD，而不是底盘补偿或需要继续增大位置Kp。
- 最终决策：保留离散脉冲禁用和速度PI仅助启动的架构；只延长最短助启动并提高有效运动交接门槛。`Kp/Ki/Kd=0.50/0/0.75`、安静保持、视觉`alpha=0.80`、`60/20 ms`真实视觉速度通道、停车底盘门控、任务逻辑和保护均不变。
- 涉及文件：`config/core_config.h`及本记录；未修改MaixCAM、G3519、通信协议、Task周期或公共API。
- 验证方式和结果：BallPositionControl与control polarity chain严格主机测试均PASS；ARM Debug clean-first完整链接通过，FLASH/RAM为`57140/20992 B`，BIN SHA-256为`8182A18257907F477CA2D5AC2D5D469E8A0DC1BC8363CC5DEB6355713F5EB4CA`。烧录前后COM3均为`phase0`、命令`0 mA`、速度`0 rpm`、`status=1`（仅online、enabled未置位）；锁定CMSIS-DAP `0416:5051`写入并复位，OpenOCD报告`Programming Finished`，按用户要求未verify/readback。
- 已知限制：新助启动参数尚未进行运动实测；Maix停止态被动抓取确认真实视觉约32 Hz、位置噪声很小，但不能替代本轮运动过程中同步球位/目标/速度证据。
- 待确认事项：用户按KEY重新采样，检查远目标能否持续起滚、约4秒的重复重入是否消失、到点后CH24/CH29是否归零；若仍未到点，不再盲目增加时间，而应同步记录Maix 22字段UDP和COM5。

## 2026-08-01：第一弧线跑飞后回退助启动并增加题号门控

- 结论：`400 ms / 12 mm/s`速度助启动版本被实机否决，已回退到`250 ms / 8 mm/s`。BMI088加速度补偿在题号未知、PeerLink离线或题号失鲜时关闭；收到新鲜合法题号后同周期立即启用Task2/4/5/6，Task3继续始终关闭。Debug固件已烧录并复位，未启动电机。
- 修改原因：桌面10:24版`1.csv`共7434帧、148.68秒，用户确认第一弧线钢球直接脱离控制。该版CH22底盘加速度范围`-1051..+892 mm/s²`，CH23补偿多次达到`±0.600 deg`，总杆角范围`-1.453..+0.793 deg`，电机速度约`-50.66..+48.10 rpm`；延长助启动与弧线加速度补偿叠加后控制权限过大，不能保留。
- 最终决策：恢复上一版短助启动交接，不新增分段或额外状态机。Task仅复用现有PeerLink题号Latest Value及其新鲜度判断：mode有效且为2/4/5/6时允许加速度输入，mode=0/离线/失鲜/3时阻断；不增加延迟、渐变、协议字段、ACK或重发。
- 涉及文件：`Task/task_app.c`、`config/core_config.h`及本记录；未修改MaixCAM、G3519、PeerLink协议、Task周期/优先级、PID、死区、视觉滤波或机械保护。
- 验证方式和结果：BallPositionControl与control polarity chain严格主机测试均PASS；ARM Debug clean-first完整链接通过，FLASH/RAM为`57156/20992 B`，BIN SHA-256为`B7990B15CCFBD1993C0B550D4AB3D1741E3D6FD90B79D772EFDF4EDCCD720E81`。烧录前后COM3均为`phase0`、命令`0 mA`、速度`0 rpm`、`status=1`（仅online、enabled未置位）；锁定CMSIS-DAP `0416:5051`写入并复位，OpenOCD报告`Programming Finished`，按用户要求未verify/readback。
- 已知限制：本轮只完成软件验证、部署和停止态确认，尚未重新运行第一弧线；VOFA仍不含同步真实球位/目标/速度助启动状态，因此不能仅凭34列数据分离弧线底盘补偿和球环助启动的瞬时贡献。
- 待确认事项：首次复测只观察第一直线和第一弧线；确认选题前CH22/CH23为零、选题后除Task3外立即恢复，并确保总杆角不再出现上一版的持续极限叠加。异常立即按KEY停止。

## 2026-08-01：第二半圆刹车前馈去重与圈间静止门控

- 结论：M4/M5/M6的负规划加速度不再额外叠加第二份`0.25`停车前馈，正负规划统一只使用公共`0.25`增益；规划失效且实测底盘速度不超过`10 mm/s`时，本地BMI088补偿归零，避免圈间持续扰杆。Debug固件已烧录并复位，未按KEY。
- 修改原因：桌面11:24版`1.csv`共2151帧、43.02秒。第一半圆BC的补偿角仅`0.010 deg RMS`，第二半圆DA进入刹车后补偿角最高`+0.292 deg`；同期球位置环平均给出约`-0.066 deg`反向杆角，证明负规划的公共前馈与额外停车前馈正在重复叠加并互相对抗。记录结束后规划失效，F407仍采用约`-47.9 mm/s² RMS`的本地BMI088残余，产生约`0.070 deg RMS`补偿，电机转速仍约`7.30 rpm RMS`。
- 最终决策：仅将`GATEWAY_CHASSIS_STOP_PLAN_FF_GAIN`从`0.25`改为`0`，保留公共双向前馈、轮速巡航补偿和`±0.60 deg`限幅；Task4/5/6只在“规划无效且CAR轮速在±10 mm/s内”时阻断本地BMI088回退。Task2继续使用本地BMI088，Task3继续始终关闭补偿；位置`Kp/Ki/Kd=0.50/0/0.80`、视觉测速、插值、速度助启动、Task周期、协议、状态机和保护不变。
- 涉及文件：`config/core_config.h`、`Task/task_app.c`、`tests/control_polarity_chain/test_control_polarity_chain.c`及本记录。
- 验证方式和结果：34列数据离线回放预计第二半圆总杆角RMS由`0.164 deg`降至约`0.119 deg`、正向峰值由`+0.443 deg`降至约`+0.298 deg`；新增正负等幅规划前馈对称性断言，control polarity chain严格主机测试PASS；ARM Debug clean-first完整链接通过，FLASH/RAM为`57204/20992 B`，BIN SHA-256为`4F442AEA5CC6432422AE79F833AE432C005D87AE53FE46A864235F2B80F7F314`，`git diff --check`通过。烧录前COM3连续确认`phase0/disabled/0 mA/0 rpm`；OpenOCD锁定F407 CMSIS-DAP `0416:5051`写入Debug ELF，报告`Programming Finished`并复位，按要求未做verify/readback。复位后COM3仍连续为`phase0/disabled/0 mA/0 rpm`。
- 已知限制：VOFA仍没有同步真实球位、目标和速度辅助状态，离线回放和烧录成功不能宣称第二半圆实物已经通过；本轮尚未动态验证。
- 待确认事项：由用户按KEY复测。重点看第二半圆刹车段CH23是否不超过约`+0.15 deg`、CH24是否不再与位置环对抗，以及停车后CH22/CH23能否归零。
