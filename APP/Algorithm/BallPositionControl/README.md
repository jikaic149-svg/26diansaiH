# BallPositionControl

这是无MCU/RTOS依赖的球位置-速度串级外环。它支持按真实`sample_ms`对VIS位置做多帧差分球速，
也支持由BallStateEstimator输入带时间戳的`x/v`估计；后一条路径仍按实际间隔推进有界积分、
破静摩擦状态和球速低通。测量真正过期时由Gateway调用`ResetMeasurement()`清除测量状态。

控制律和实心球纯滚动逆模型为：

```text
v_target = limit((Kp / Kd) * (x_target - x), +/-v_max)
a_cmd = Kd * (v_target - v) + I
theta_ball = -asin((1 + 0.4) * a_cmd / g)
```

统一物理坐标以右侧固定转轴指向左侧执行端为`x>0`，左侧执行端抬高为`theta>0`。
MaixCAM当前输出画面左负右正，所以配置`measurement_polarity=-1`后，物理左侧为正。
球位于左侧时控制器命令正杆角抬高左端，使球向右侧固定转轴滚动，形成负反馈。

`GATEWAY_BALL_VISUAL_CONTROL_ENABLE=1`显式开启这条支路；当前YOLO实测位置直接进入约
`80 ms`多帧差分与`40 ms`一阶低通，不使用BallStateEstimator或杆球模型预测球速。输出仍
经过配置杆角限幅；视觉过期时Gateway清测量状态并令该反馈项归零，不自动失能电机。

运行时目标由`SetTargetPositionMm()`只更新指令值，再由`AdvanceTarget()`按配置斜率推进
活动参考；切换目标不会清位置、差分速度或速度有效位。Gateway允许在停机和回零phase 1
预设目标，并跨停止/再次启动保留；只有进入phase 2且视觉测量有效时，杆球模型输出才实际
进入杆角命令。产品目标范围为`[-120,+120] mm`，当前产品斜率为`80 mm/s`。

`SetEstimatedState()`保留为可选估计器接入边界；当前`GATEWAY_BALL_KALMAN_ENABLE=0`，
正常固件不使用该路径。BMI088底盘加速度仅在Gateway最终杆角求和处独立前馈补偿。

产品配置全域统一使用一套`Kp/Ki/Kd=3.20/0.050/2.50`，近目标增益调度和离散破静摩擦
脉冲均禁用。误差不超过`2 mm`且滤波球速不超过`8 mm/s`时锁存安静保持，球环输出持续为
零；保持不会因后续测量漂移自动退出，仅在目标命令改变或`ResetMeasurement()`时解除。
