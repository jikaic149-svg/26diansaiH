# BallModelInterpolator

这是一个无 MCU、RTOS、动态内存和 Kalman 协方差依赖的两状态滚球模型插值器，供后续接入
F407 高频控制循环。状态仅为球位置 `x_mm` 与球速度 `v_mm_s`。

预测采用实心球纯滚动模型：

```text
kappa = 1 / (1 + I/(mR^2)) = 5/7
a_ball = -kappa * (g*sin(theta_rod) + a_chassis*cos(theta_rod))
x += v*dt + 0.5*a_ball*dt^2
v += a_ball*dt
```

`Predict()`面向 1 ms 调用。状态有效时，调度间隔超过
`maximum_prediction_step_ms`会使状态失效，避免跨未知间隔积分。状态位置与速度始终受配置上限
约束，任何非有限输入或计算结果均被拒绝。

`CorrectPosition()`不保存历史数组，也不逐帧硬重置。它用测量年龄和当前模型加速度把当前状态
近似回推至相机采样时刻，形成位置创新；随后在该时刻应用固定 alpha-beta 修正，并把修正近似
传播回当前时刻：

```text
x_lag = x_now - v_now*age + 0.5*a_ball*age^2
innovation = z_camera - x_lag
dv = beta*innovation / delta_measurement
x_now += alpha*innovation + dv*age
v_now += dv
```

该固定滞后只是常加速度近似，不会估计加速度偏置，也不能替代真实视觉校正。已有状态的创新
超过 `maximum_absolute_innovation_mm` 时只计数并保持预测状态；首次有效测量仍可种下状态。
过期、重复及乱序测量同样不污染状态。上述软拒绝返回 `true`，调用者通过
`GetDiagnostics()`的 `last_correction_accepted` 和各拒绝计数区分；参数、调用时序或数值错误返回
`false`。

`IsTrusted()`要求已接受配置数量的视觉测量，且从最后一帧实际采样时刻起的纯模型滑行时间不
超过 `maximum_coast_ms`。模块只提供估计，不包含 PID、任务切换、电机使能或通信逻辑。
