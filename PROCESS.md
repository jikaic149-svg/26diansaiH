# PROCESS

## 2026-08-01 修复Task3活动位被视觉解码器拒收

- 修改原因：F407已正式用VST `status_flags.bit14/0x4000`转发G3519的Task3活动状态，但Maix解码器仍把该位当保留位，导致活动帧被整帧拒收，Task3无法启动。
- 最终决策：允许VST状态位`0x4000`进入任务管理器；仅保留`fault_bits.bit15`的保留位检查。Task3仍要求本地滚球环已使能、视觉有效且无故障才开始目标序列，G3519 KEY不控制F407电机使能。
- 涉及文件：`f4_vision_link.py`、`tests/test_f4_vision_link.py`及本记录。
- 验证与部署：VST与任务管理器8项单测通过，覆盖`0x4000`接受和fault保留位拒绝。完整17个Python文件已同步至`/root/steelball_horizontal_trough_pro`并由`maixpy_run.py`重启；单进程持续运行，60 FPS相机、VIS和VST在线。部署时F407回报mode 2、命令电流0、速度0，未启动电机。
- 待确认：切换到M3并在F407本地滚球环已就绪后，实测G3519第一次短按启动、第二次短按停止。

## 2026-08-01 Task3改由G3519 KEY直接启停

- 修改原因：用户否决“选择M3后还要F407失能并重新按本地KEY”的`WAIT_REARM`设计，明确第一次G3519短按启动、第二次短按结束。
- 最终决策：Maix读取F407 VST `status_flags.bit14`作为明确Task3活动状态；活动位为1且真实滚球环在线时启动现有Q3序列，活动位清零立即取消并回到`WAIT_START`。不再根据F407预先使能状态猜测按键事件。
- 涉及文件：`competition_task_manager.py`、对应Python测试及本记录。
- 验证与部署：5项任务管理器单测通过；完整Python工程已同步至`/root/steelball_horizontal_trough_pro`并从`maixpy_run.py`重启，进程持续运行，VIS/VST在线。部署后VST为phase 0、`0 mA`、速度0、驱动失能。
- 待确认：实机执行一次完整的G3519 KEY启动、A→B→A、再次KEY停止。

## 2026-08-01 第三题视觉目标冻结为绝对±5 cm

- 修改原因：实机第二段未到`-50 mm`，8 s硬超时后旧逻辑把目标覆盖为最后视觉位置
  `-25 mm`，上位机因此显示第二目标只有`-2.5 cm`；用户明确要求绝对`±5 cm`。
- 最终决策：第三题只发送视觉目标`+50 mm`和`-50 mm`，取消此前用于破静摩擦的
  `+70/-70 mm`超调命令。硬超时仍标记`FAILED`，但目标保持`-50 mm`，不再改为任意实测位置。
- 涉及文件：`ball_target_touch.py`、`vision_config.py`、`main.py`、对应测试和运行辅助脚本。
- 验证方式和结果：PC端26项Python单元测试全部通过，关键模块`py_compile`和
  `git diff --check`通过。完整项目已同步到`/root/steelball_horizontal_trough_pro`并重启；
  板端回读配置为`RIGHT/LEFT/FINAL=50/-50/-50 mm`且存在`Q3_TIMEOUT_FINAL`，三份关键文件
  SHA-256与本地一致。新进程PID 85090持续运行，启动、UART、VIS和VST均在线，stderr为空；
  VST确认F407命令0 mA、速度0 rpm、fault=0，部署过程未启动电机。
- 已知限制：目标严格等于`±50 mm`不代表当前机械闭环一定能在5 s内到达；反向段收敛能力仍需实测。

## 2026-07-31 G3519赛题驱动的视觉任务调度

- 修改原因：用户明确task指题目2～6的业务任务，MaixCAM应消费F407转发的G3519 mode并
  完成对应视觉任务，而不是只显示mode。
- 最终决策：新增独立纯Python任务管理器。M2只监测；M3预置中心并等待F407滚球外环、
  电机使能和无故障状态后执行现有本地序列；M4/M5锁中心；M6保留并开放触屏任意目标。
  mode=0或VST过期时保留当前任务状态、停止推进并锁定触屏，但不启动或停止电机。
- 涉及文件：`competition_task_manager.py`、`ball_target_touch.py`、`main.py`、
  `vision_config.py`、`app.yaml`、README与对应PC测试。
- 验证结果：20项Python单元测试通过，关键模块`compileall`和`git diff --check`通过。
- 待实物确认：G3519实际切换M2～M6时屏幕状态、VBT目标及M3使能边沿行为。

## 2026-07-31 MaixCAM2 UI尺寸标定与冻结

- 实测尺寸：面板驱动日志`480×640`，MaixPy display/touch逻辑坐标`640×480`，
  YOLO/视觉画布`416×320`；FIT_CONTAIN内容区为`(8,0,624,480)`，底部滑条触摸区
  映射为`(26,393,588,87)`。
- 最终决策：新增纯Python尺寸计算与严格校验；将四个实际尺寸写入`vision_config.py`。
  尺寸不一致时启动失败，避免屏幕旋转、模型输入变化或换屏导致触摸坐标错位。
- 验证状态：23项Python测试、关键模块py_compile和`git diff --check`通过；冻结版已部署，
  板端输出`UI_SIZE_FROZEN|frame=416x320|display=640x480|content=8,0,624,480|`
  `slider_touch=26,393,588,87`，视觉进程持续运行且stderr为空。

## 2026-07-31 临时无标定直接控制

- 修改原因：用户要求启动后直接回传球坐标并允许F407控制，暂时跳过交互标定。
- 最终决策：关闭启动阈值菜单和快速位置标定；使用动态凹槽端点及25 cm线性换算；临时置位VIS标定有效标志。
- 涉及文件：`vision_config.py`、`main.py`、`README.md`。
- 验证结果：10个Python文件AST通过；启动格式32个字段与32个参数匹配；Host固定向量得到中心`0 mm`和VIS flags `0x35`。
- 已知限制：坐标没有现场点标定精度保证，端点识别偏差会直接形成位置误差。
- 待确认事项：上板确认`F4_UART_OPEN`、`F4_VIS_TX_STARTED`、F407 `vis_rx_count/vis_usable_count`递增及实际控制方向。

## 2026-07-31 快速整幅水平轴闭环

- 修改原因：现场白槽轴误检导致`BALL_OFF_TROUGH`，用户要求跳过标定快速闭环并恢复彩色显示。
- 最终决策：关闭二值显示；快速模式始终以YOLO球心所在水平线和整幅416 px宽度做25 cm
  线性换算，轴源按协议标为`HELD`。PID固定为用户批准的`1.20/0.30/1.80`。
- 涉及文件：`main.py`、`vision_config.py`及项目内`.maixpy`部署/监视辅助脚本。
- 实机证据：Maix运行于`192.168.215.66`；VPA接受增益，VST曾达到
  `last_rx_vis=last_used_vis`、`status=0x3FFF`、`fault=0`并输出非零杆角，证明位置测量已进入
  F407滚球外环。随后按用户要求停机并重新烧录F407，未做最终烧录后的回读或运动验证。
- 已知限制：整幅画面端点不是现场毫米标定；YOLO多目标切换会造成位置跳变，正式版本应恢复
  槽端点/点标定并限定唯一钢球。

## 2026-07-31 过零振荡阻尼同步

- 修改原因：实机滚球反复过零，F407默认速度阻尼不足。
- 最终决策：运行时VTP默认值同步为`Kp/Ki/Kd=0.60/0.15/1.20`；该命令仍不启动电机。
- 涉及文件：`vision_config.py`。
- 验证状态：本地配置已修改；当前板端视觉进程未重启，本轮F407烧录后以固件默认值生效。

## 2026-07-31 最新实机VTP同步

- 最终决策：根据COM3动态采样，将运行时VTP更新为`Kp/Ki/Kd=0.60/0.03/0.55`；F407端
  速度滤波为`0.18 s`、积分加速度限为`10 mm/s²`。该配置不启动电机。
- 实机状态：视觉约34 FPS且VIS持续送达；F407已成功经历`phase 1 -> phase 2`。滚球仍未证明
  收敛到中心，后续杆目标斜率限制或继续调参需另行确认。
## 2026-07-31 任意位置闭环首轮实机调参同步

- MaixCAM实测目标`-59 mm`时，小球由`+34 mm`长时间静止后冲至`-72 mm`并多次往返；据此将运行时VTP更新为`Kp/Ki/Kd=1.00/0.030/1.00`。
- 本地12项Python测试PASS；完整工程经项目部署脚本同步到`/root/steelball_horizontal_trough_pro`并从`maixpy_run.py`重启。
- F407回传`F4_VPA accepted=1`，连续VST的视觉位置有效且`fault=0x0000`；电机保持失能，动态效果待用户按KEY验证。

## 2026-07-31 F407赛题mode只读显示与遥测同步

- 修改原因：用户要求视觉程序同步显示G3519经F407转发的当前赛题mode。
- 最终决策：解析VST第15字段`competition_mode`；只接受`0`或`2..6`，其中`0`表示F407
  当前没有新鲜mode。日志追加`mode`，画面显示`M2..M6`，未知时显示`M-`；VOFA UDP在现有
  FireWater字段尾部追加mode。视觉侧只解析、显示和上报，不允许反向修改mode，也不改变
  检测、滚球控制或启动行为。
- 涉及文件：`f4_vision_link.py`、`main.py`、`vofa_udp_telemetry.py`、
  `tests/test_f4_vision_link.py`、README和本记录。
- 验证方式和结果：本地12项Python测试PASS；`git diff --check`通过。
- 已知限制与待确认：本轮未向Maix部署或启动视觉程序；15字段VST的UART实收、画面mode
  刷新和UDP接收端新增尾字段兼容性仍需联机实物验证。

## 2026-07-31 F407赛题mode实物分段验证

- 修改原因：用户要求立即验证G3519经F407同步到Maix的mode实物链路。
- 部署状态：停止占用摄像头的旧`/root/steelball_horizontal_trough_pro/maixpy_run.py`后，
  当前13个Python文件已上传至`/tmp/steelball_closed_loop`，新`main.py`持续运行；相机、显示、
  VIS发送、VPA接收和15字段VST解析均在线。启动命令的SSH等待虽超时，但远端PID和持续日志
  证明应用没有退出。
- 分段验证结果：F407通过SWD向只读PeerLink缓存临时注入新鲜`mode=2`后，Maix实际UART日志
  连续出现3条`mode=2`；1500 ms后自动恢复`mode=0`。最终VST为命令`0 mA`、速度`0`、
  `mode=0`，临时注入已在F407侧清除。该结果证明Maix解析/日志链路和F407超时回退，不证明
  G3519发送链路。
- 已知限制与待确认：指定G3519 DAPLink未枚举且F407的heartbeat/CAR接收计数均为0，因此
  完整G3519→F407→Maix链路未通过。重新连接G3519目标板和UART后，须在无SWD注入条件下
  观察稳定`mode=2..6`并核对屏幕`M2..M6`与UDP尾字段。
