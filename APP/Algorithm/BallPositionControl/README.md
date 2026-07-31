# BallPositionControl

这是无MCU/RTOS依赖的球位置控制器。它支持按真实`sample_ms`对VIS位置做多帧差分球速，
也支持由BallStateEstimator输入带时间戳的`x/v`估计；后一条路径仍按实际间隔推进有界积分、
破静摩擦状态和球速低通。测量真正过期时由Gateway调用`ResetMeasurement()`清除测量状态。

正常位置环、远距离速度环和实心球纯滚动逆模型为：

```text
a_position = Kp * (x_target - x) + I - Kd * v
a_brake_distance = max(abs(x_command - x) - e_near, 0)
v_target = direction * min(v_max, sqrt(2 * a_max * a_brake_distance))
a_speed = limit(Kvp * (v_target - v) + integral(Kvi * (v_target - v)), +/-a_max)
a_cmd = speed_trip_active ? a_speed : a_position
theta_ball = -asin((1 + 0.4) * a_cmd / g)
```

每次目标命令形成一次锁存旅程：最终目标误差超过入口阈值时速度PI立即、连续且独占控制，
目标球速按剩余制动距离下降；进入近目标且球速足够低（或已经越过目标）后一次性切入位置环。
测量噪声不会让同一旅程重新切回速度环；只有新目标或`ResetMeasurement()`才会重新判定。
离散kick、100 ms等待门控和速度/位置smoothstep混环均不再参与控制；Task3运行时旁路速度旅程。

统一物理坐标以右侧固定转轴指向左侧执行端为`x>0`，左侧执行端抬高为`theta>0`。
MaixCAM当前输出画面左负右正，所以配置`measurement_polarity=-1`后，物理左侧为正。
球位于左侧时控制器命令正杆角抬高左端，使球向右侧固定转轴滚动，形成负反馈。

`GATEWAY_BALL_VISUAL_CONTROL_ENABLE=1`显式开启这条支路；当前YOLO实测位置直接进入约
`120 ms`多帧差分与`60 ms`一阶低通，不使用BallStateEstimator或杆球模型预测球速。输出仍
经过配置杆角限幅；视觉过期时Gateway清测量状态并令该反馈项归零，不自动失能电机。

运行时目标由`SetTargetPositionMm()`只更新指令值，再由`AdvanceTarget()`按配置斜率推进
活动参考；切换目标不会清位置、差分速度或速度有效位。Gateway允许在停机和回零phase 1
预设目标，并跨停止/再次启动保留；只有进入phase 2且视觉测量有效时，杆球模型输出才实际
进入杆角命令。产品目标范围为`[-120,+120] mm`，当前产品斜率为`120 mm/s`。

`SetEstimatedState()`保留为可选估计器接入边界；当前`GATEWAY_BALL_KALMAN_ENABLE=0`，
正常固件不使用该路径。BMI088底盘加速度仅在Gateway最终杆角求和处独立前馈补偿。

正常位置环使用Git基线`Kp/Ki/Kd=0.60/0/0.55`，近目标增益调度和离散破静摩擦脉冲均
禁用。误差不超过`3 mm`且滤波球速不超过`8 mm/s`并持续`60 ms`时锁存安静保持，球环输出持续为
零；保持不会因后续测量漂移自动退出，仅在目标命令改变或`ResetMeasurement()`时解除。
