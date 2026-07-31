# BallPositionControl

这是无MCU/RTOS依赖的球位置控制器。它支持按真实`sample_ms`对VIS位置做多帧差分球速，
也支持由BallStateEstimator输入带时间戳的`x/v`估计；后一条路径仍按实际间隔推进有界积分、
破静摩擦状态和球速低通。测量真正过期时由Gateway调用`ResetMeasurement()`清除测量状态。

正常位置环、破静摩擦速度环和实心球纯滚动逆模型为：

```text
a_position = Kp * (x_target - x) + I - Kd * v
a_speed = limit(Kvp * (v_target - v) + integral(Kvi * (v_target - v)), +/-a_max)
a_cmd = (1 - blend) * a_position + blend * a_speed
theta_ball = -asin((1 + 0.4) * a_cmd / g)
```

`blend`只在钢球满足低速静止门控后启用；大误差时为1，由速度环克服静摩擦并驱动起滚，
进入配置的近目标交接带后按smoothstep连续降到0，由位置环完成逼近和停车。两个环不直接
相加，避免破静摩擦期间把杆角重复放大。速度积分只在球速不超过静止门槛时累积；一旦检测
到起滚立即清零，防止静摩擦积分变成运动阶段的冲击能量。

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
