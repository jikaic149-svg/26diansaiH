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
距离规划球速再按真实测量间隔和最大加速度连续限斜率；进入近目标且球速足够低（或已经越过
目标）后切入位置环。同一目标下若球再次低速停在入口阈值之外，持续
`stiction_kick_duration_ms`后允许速度PI重新脱离静摩擦；单帧位置噪声、高速运动或误差回到
入口内都会清除本次重入计时。离散kick、首次100 ms等待门控和速度/位置smoothstep混环均不再
参与控制。Gateway可在赛题切换时无扰选择运行时速度/加速度上限与最终位置带；当前
Task2/4/5/6保持`30 mm/s`、`180 mm/s²`和`3 mm`，Task3单独使用`45 mm/s`、
`300 mm/s²`和`1.5 mm`，但不改变Task3已验证的电机角环PID。

统一物理坐标以右侧固定转轴指向左侧执行端为`x>0`，左侧执行端抬高为`theta>0`。
MaixCAM当前输出画面左负右正，所以配置`measurement_polarity=-1`后，物理左侧为正。
球位于左侧时控制器命令正杆角抬高左端，使球向右侧固定转轴滚动，形成负反馈。

`GATEWAY_BALL_VISUAL_CONTROL_ENABLE=1`显式开启这条支路。Task2/4/5/6由Gateway的轻量
BallModelInterpolator每1 ms提交位置/速度状态；Task3直接使用YOLO实测位置的`120 ms`多帧
差分与`60 ms`一阶低通，避免模型虚假球速妨碍停车和误差重入。进入/退出Task3时Gateway清除
旧来源状态。输出仍经过配置杆角限幅；可信视觉/模型状态过期时Gateway清测量状态并令该反馈项
归零，不自动失能电机。

运行时目标由`SetTargetPositionMm()`只更新指令值，再由`AdvanceTarget()`按配置斜率推进
活动参考；切换目标不会清位置、差分速度或速度有效位。Gateway允许在停机和回零phase 1
预设目标，并跨停止/再次启动保留；只有进入phase 2且视觉测量有效时，当前模式的球反馈才实际
进入杆角命令。产品目标范围为`[-120,+120] mm`，当前产品斜率为`120 mm/s`。

`SetEstimatedState()`是模型和可选Kalman估计器的接入边界；当前
`GATEWAY_BALL_KALMAN_ENABLE=0`。BMI088底盘加速度仅在Gateway最终杆角求和处独立前馈补偿。

正常位置环使用Git基线`Kp/Ki/Kd=0.60/0/0.55`，近目标增益调度和离散破静摩擦脉冲均
禁用。通用保持误差为`3 mm`，Task3为`1.5 mm`；同时满足对应位置误差、滤波球速不超过
`8 mm/s`并持续`60 ms`时锁存安静保持，球环输出持续为零。保持退出仍使用更宽的滞环和
连续样本确认。
