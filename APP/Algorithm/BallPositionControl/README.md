# BallPositionControl

这是无MCU/RTOS依赖的球位置外环。每个新VIS样本按真实`sample_ms`差分速度，再用一阶滤波；
采样间隔不在`1..maximum_velocity_dt_ms`时保留位置和有界积分状态、清零速度并等待
下一对有效样本；VIS真正过期时仍由Gateway调用`ResetMeasurement()`清除全部测量状态。

控制律和实心球纯滚动逆模型为：

```text
a_cmd = Kp * (x_target - x) + I - Kd * v
theta_ball = -asin((1 + 0.4) * a_cmd / g)
```

统一物理坐标以右侧固定转轴指向左侧执行端为`x>0`，左侧执行端抬高为`theta>0`。
MaixCAM当前输出画面左负右正，所以配置`measurement_polarity=-1`后，物理左侧为正。
球位于左侧时控制器命令正杆角抬高左端，使球向右侧固定转轴滚动，形成负反馈。

`GATEWAY_BALL_VISUAL_CONTROL_ENABLE=1`显式开启这条VIS位置/速度PD支路；它不绕过
YOLO来源、毫米标定、clamp和100 ms年龄门，也不等于启用Kalman。输出按配置杆角限幅。
当前参数只是低动态首版初值，视觉延迟、速度噪声、带球负载和双向机构响应尚未实物整定。
VIS过期时Gateway清测量状态并令该反馈项归零，不自动失能电机。

运行时目标由`SetTargetPositionMm()`只更新指令值，再由`AdvanceTarget()`按配置斜率推进
活动参考；切换目标不会清位置、差分速度或速度有效位。Gateway允许在停机和回零phase 1
预设目标，并跨停止/再次启动保留；只有进入phase 2且视觉测量有效时，杆球模型输出才实际
进入杆角命令。产品目标范围为`[-120,+120] mm`，当前产品斜率为`80 mm/s`。

`SetEstimatedState()`是默认关闭的BallStateEstimator接入边界：输入已是物理坐标的
`x/v`，不再应用相机极性，也不再做差分。当前`GATEWAY_BALL_KALMAN_ENABLE=0`，
因此正常固件仍使用本模块原有的VIS差分速度路径。
