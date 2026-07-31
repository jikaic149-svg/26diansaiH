# ChassisAccelerationFusion

该模块是无MCU/RTOS依赖并已接入产品控制的三状态线性Kalman滤波器。状态为底盘实际纵向速度`v`、实际纵向加速度`a`和IMU纵向加速度零偏`b_imu`：

```text
X = [v, a, b_imu]
a_dot = (k_a*a_plan_delayed-a)/tau_a + w_a
v_dot = a + w_v
b_imu(k+1) = b_imu(k) + w_b
z_wheel = v + n_v
z_imu = a + b_imu + n_a
a_used = a
```

规划加速度经过纯延迟和一阶执行响应产生1 kHz预测；新版`CAR`中的实际平均轮速和F407
本地BMI088经过安装轴变换、零偏初校及抗振滤波的纵向加速度，只在各自新样本到达时独立
校正。轮速作为速度观测直接使用，不先差分成高噪声加速度，也不把100 Hz Latest Value
在1 kHz Task中重复当成新观测。

可信门要求轮速与IMU各自达到最少接受次数、两者均新鲜且`P(a,a)`低于阈值。任一条件不满足时上层必须退回原规划ACC。该退回只决定模型输入，不自动失能电机。

当前配置基于最新实车日志启用；无新鲜M4规划时上层复位融合器并直接回退本地BMI088。噪声方差、过程噪声、固定延迟和可信阈值仍应继续用后续原始日志复核。
