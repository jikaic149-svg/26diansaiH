# BallPositionControl

这是无MCU/RTOS依赖的球位置控制器。它支持按真实`sample_ms`对VIS位置做多帧差分球速，
也支持外部估计器输入带时间戳的`x/v`状态；后一条路径仍按实际间隔推进有界积分、
破静摩擦状态和球速低通。测量真正过期时由Gateway调用`ResetMeasurement()`清除测量状态。

正常位置环、远距离速度环和实心球纯滚动逆模型为：

```text
a_position = Kp * (x_target - x) + I - Kd * v
a_brake_distance = max(abs(x_command - x) - e_near, 0)
v_distance = direction * min(v_max, sqrt(2 * a_max * a_brake_distance))
v_target = slew(v_target_previous, v_distance, a_max * real_delta_t)
a_speed = limit(Kvp * (v_target - v) + integral(Kvi * (v_target - v)), +/-a_max)
a_cmd = speed_trip_active ? a_speed : a_position
theta_ball = -asin((1 + 0.4) * a_cmd / g)
```

每次目标命令形成一次锁存旅程：最终目标误差超过入口阈值时速度PI立即、连续且独占控制，
距离规划球速再按真实测量间隔和最大加速度连续限斜率。运行时策略可选择真实球速起来后
立即锁存位置环，或持续推进到最终误差带/越过目标后才切入位置环。同一目标下若球再次低速停在入口阈值之外，持续
`stiction_kick_duration_ms`后允许速度PI重新脱离静摩擦；单帧位置噪声、高速运动或误差回到
入口内都会清除本次重入计时。离散kick、首次100 ms等待门控和速度/位置smoothstep混环均不再
参与控制。Gateway可在赛题切换时无扰选择运行时速度/加速度上限与最终位置带；当前
Task2/5保留速度PI助启动以及`30 mm/s`、`180 mm/s²`和`3 mm`参数。Task4关闭速度
助启动但保留既有视觉模型位置路径；Task6直接使用真实视觉位置/速度，在约`11 mm`
外以`20 mm/s`、`120 mm/s²`连续速度PI推进且不因球速超过`8 mm/s`退出，直到`6 mm`
内或越过目标才切位置PD并允许安静保持；接近目标的距离速度规划独立使用`25 mm/s²`
减速度，满速约从`14 mm`误差开始制动且不降低PI输出上限。每次新速度旅程先保证
误差方向`0.50°`球环倾角下限，真实球速首次超过`8 mm/s`后本次永久撤销；该下限不与
PI相加、不会在途中反复出现，`8 mm/s`仍用于受扰停滞后的重入判定，
Gateway继续独立叠加底盘加速度前馈，不使用模型位置插值。Task3首段由Gateway在视觉
原点门控后匀速抬升杆角，视觉越过`+50 mm`时调用
`ResetTargetPositionMm()`立即跳到返程目标，再用`RearmPositionLoop()`清除保持/积分/速度
辅助状态并进入共享位置PD。若返程PD随后低速停在目标外，现有速度PI按同一静摩擦重入门槛
短时助启动，真实球速起来后重新锁存位置PD；该交接保留最新实测位置、滤波速度和速度历史。

统一物理坐标以右侧固定转轴指向左侧执行端为`x>0`，左侧执行端抬高为`theta>0`。
MaixCAM当前输出画面左负右正，所以配置`measurement_polarity=-1`后，物理左侧为正。
球位于左侧时控制器命令正杆角抬高左端，使球向右侧固定转轴滚动，形成负反馈。

`GATEWAY_BALL_VISUAL_CONTROL_ENABLE=1`显式开启这条支路。Task2/4/5由Gateway的轻量
BallModelInterpolator每1 ms提交插值位置，但位置PD的速度阻尼统一使用YOLO实测位置的
`120 ms`多帧差分与`60 ms`一阶低通；模型速度只参与模型自身传播，不再直接进入Kd，避免
停车后虚假模型球速持续驱动杆角。Task3/6直接使用同一真实VIS位置/速度。进入/退出直接视觉模式时
Gateway清除旧来源状态。输出仍经过配置杆角限幅；可信视觉/模型状态过期时Gateway清测量状态
并令该反馈项归零，不自动失能电机。

运行时目标由`SetTargetPositionMm()`只更新指令值，再由`AdvanceTarget()`按配置斜率推进
活动参考；切换目标不会清位置、差分速度或速度有效位。Gateway允许在停机和回零phase 1
预设目标，并跨停止/再次启动保留；只有进入phase 2且视觉测量有效时，当前模式的球反馈才实际
进入杆角命令。产品目标范围为`[-120,+120] mm`，当前产品斜率为`120 mm/s`。

`UpdateVisualVelocity()`每个真实相机序列只调用一次，`SetEstimatedPosition()`接收模型的高频
物理位置且保留该真实视觉速度；`SetEstimatedState()`只保留给同时提交`x/v`的可选Kalman路径。
当前`GATEWAY_BALL_KALMAN_ENABLE=0`。BMI088底盘加速度仅在Gateway最终杆角求和处独立前馈补偿。

共享位置环使用`Kp/Ki/Kd=0.50/0/0.75`，近目标增益调度和离散破静摩擦脉冲均禁用。
所有赛题保持误差统一为`3 mm`；同时满足位置误差、滤波球速不超过`8 mm/s`并持续`60 ms`
时锁存安静保持，球环输出持续为零。保持退出仍使用更宽的滞环和连续样本确认。
