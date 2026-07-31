#ifndef CORE_CONFIG_H
#define CORE_CONFIG_H

/* F407-Gateway 核心可调参数唯一入口。
 * 只收录源码实际消费、需按产品整定的默认值。
 * 引脚/时钟/中断映射属于 BSP，FreeRTOS 内核参数属于 FreeRTOSConfig.h，
 * QD4310 协议常量属于 Device/QD4310。
 */

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Task 与调度                                                          */
/* ------------------------------------------------------------------ */

/* 网关控制 Task：周期 ms、tskIDLE_PRIORITY 之上的偏移、栈深(StackType_t)。 */
#define GATEWAY_TASK_PERIOD_MS            UINT32_C(1)
#define GATEWAY_TASK_PRIORITY_OFFSET      2U
#define GATEWAY_TASK_STACK_DEPTH          512U
#define GATEWAY_TASK_NAME                 "gateway"

/* ------------------------------------------------------------------ */
/* ESP8266 无线（AP + TCP 服务器）                                       */
/* ------------------------------------------------------------------ */

/* 当前控制器不使用ESP8266。0时不初始化、不轮询、不收发，避免无线字节流
 * 占用1 ms控制周期；仅改为1才恢复原AP/TCP链路。 */
#define GATEWAY_ESP8266_ENABLE              0U

/* ESP8266 建热点的 SSID / 密码（密码至少 8 字符，WPA2）。 */
/* 本机可在被 Git 忽略的 local_secrets.h 中覆盖以下默认值。 */
#if defined(__has_include)
#if __has_include("local_secrets.h")
#include "local_secrets.h"
#endif
#endif
#ifndef ESP8266_AP_SSID
#define ESP8266_AP_SSID                   "F407-Gateway"
#endif
#ifndef ESP8266_AP_PASSWORD
#define ESP8266_AP_PASSWORD               "change-me"
#endif
/* AP 信道 1..13，热点最大同时接入 4 个客户端（固件上限）。 */
#define ESP8266_AP_CHANNEL                5

/* TCP 服务器监听端口，客户端（手机/电脑）连接此端口收发帧。 */
#define ESP8266_TCP_SERVER_PORT           3333

/* AT 交互超时与重连节奏，单位 ms。 */
#define ESP8266_BOOT_WAIT_MS              UINT32_C(1500)
#define ESP8266_AT_TIMEOUT_MS             UINT32_C(2000)
#define ESP8266_RECONNECT_INTERVAL_MS     UINT32_C(3000)

/* ------------------------------------------------------------------ */
/* TCP<->CAN 网关                                                       */
/* ------------------------------------------------------------------ */

/* 总线上 QD4310 电机数量与各自 CAN ID（0x00..0x0F，总线内唯一）。 */
#define GATEWAY_MOTOR_COUNT               1U
#define GATEWAY_MOTOR_IDS                 { 0x01U }

/* 板载 KEY1(PA0，高有效)的消抖时间，单位 ms；稳定按下只触发一次。 */
#define GATEWAY_KEY_DEBOUNCE_MS           UINT32_C(30)

/* 手动连杆标定已经完成，KEY恢复普通视觉闭环启停。保留下面的标定参数，
 * 仅供后续显式重新进入标定固件时使用。 */
#define GATEWAY_LEVEL_CALIBRATION_KEY_MODE                 0U
#define GATEWAY_LEVEL_CALIBRATION_MANUAL_MODE              1U
#define GATEWAY_LEVEL_CALIBRATION_MANUAL_ARM_MS UINT32_C(100)
#define GATEWAY_LEVEL_CALIBRATION_START_RAW    UINT16_C(53577)
#define GATEWAY_LEVEL_CALIBRATION_TURN_RAW     UINT16_C(58499)
#define GATEWAY_LEVEL_CALIBRATION_SPAN_RAW      UINT32_C(4922)
#define GATEWAY_LEVEL_CALIBRATION_ONE_WAY_POINTS           125U
#define GATEWAY_LEVEL_CALIBRATION_STABLE_TOL_RAW   UINT16_C(15)
#define GATEWAY_LEVEL_CALIBRATION_MIN_CURRENT_A              0.20F
#define GATEWAY_LEVEL_CALIBRATION_APPROACH_MIN_CURRENT_A      0.12F
#define GATEWAY_LEVEL_CALIBRATION_SLOWDOWN_RAW       UINT16_C(800)
#define GATEWAY_LEVEL_CALIBRATION_HOLD_KP_A_PER_RAW          0.0015F
#define GATEWAY_LEVEL_CALIBRATION_SPEED_KD_A_PER_RPM         0.0080F
#define GATEWAY_LEVEL_CALIBRATION_START_MAX_SPEED_RPM       12.0F
#define GATEWAY_LEVEL_CALIBRATION_STABLE_MAX_RPM           1.0F
#define GATEWAY_LEVEL_CALIBRATION_STABLE_TIME_MS UINT32_C(100)
#define GATEWAY_LEVEL_CALIBRATION_HOLD_TIME_MS   UINT32_C(120)
#define GATEWAY_LEVEL_CALIBRATION_POINT_TIMEOUT_MS \
    UINT32_C(10000)

/* 周期性向电机发 NOP 请求反馈并把反馈帧回传 TCP 的周期，单位 ms。
 * QD4310 是应答式：只有收到控制/NOP 帧才回反馈帧。 */
#define GATEWAY_FEEDBACK_PERIOD_MS        UINT32_C(100)

/* 500 Hz Task禁止等待CAN邮箱；0表示检查后立即成功或返回TIMEOUT。 */
#define GATEWAY_CAN_SEND_TIMEOUT_MS       UINT32_C(0)

/* QD4310 产品侧命令限幅：电流 A、速度 rpm。不得超过 Device 登记的 2 A / 800 rpm
 * 物理峰值；首轮保持额定值 1 A / 500 rpm。 */
#define GATEWAY_MOTOR_CURRENT_LIMIT_A     1.0F
#define GATEWAY_MOTOR_SPEED_LIMIT_RPM     500.0F

/* 反馈在线超时，单位 ms。必须大于反馈请求周期并留出 Task/CAN 抖动余量。 */
#define GATEWAY_MOTOR_FEEDBACK_TIMEOUT_MS UINT32_C(300)

/* ------------------------------------------------------------------ */
/* 水管绝对角外环 -> QD4310 CURRENT 电流环                              */
/* ------------------------------------------------------------------ */

/* 控制器名义周期和新反馈超时，单位 ms。控制只在收到一帧新反馈时更新一次；
 * 名义周期参与积分计算，必须与当前 Gateway Task 周期一致。 */
#define GATEWAY_LEVEL_CONTROL_PERIOD_MS    UINT32_C(1)
#define GATEWAY_LEVEL_FEEDBACK_TIMEOUT_MS UINT32_C(100)
#define GATEWAY_LEVEL_ENABLE_CONFIRM_TIMEOUT_MS UINT32_C(200)

/* 旧机械水平零点继续固定为raw=54379；本轮IMU向上扫描不会覆盖它。
 * TRIM_RAW=54329保留此前约-0.275电机度的工作微调。改变安装关系后必须重新标定。 */
#define GATEWAY_LEVEL_ZERO_RAW             UINT16_C(54379)
#define GATEWAY_LEVEL_TRIM_RAW             UINT16_C(54329)

/* 2026-07-31手动往返标定覆盖raw=47707..58831。JY61 Pitch与raw相关系数
 * -0.962；raw约57500后Pitch不再继续下降，进入机构死区并产生此前的端部震荡。
 * 杆角目标查表只使用单调区48000..57500，绝不命令进入高raw死区；59025仅是
 * 反馈/启动硬边界，用于容纳任意安全起始位置。反馈越过该硬边界时暂停PID，
 * 仅以RECOVERY_CURRENT向范围内部回收，位于硬边界时也只允许向内电流。 */
#define GATEWAY_LEVEL_RAW_MIN_LIMIT         UINT16_C(48000)
#define GATEWAY_LEVEL_RAW_CONTROL_MAX       UINT16_C(57500)
#define GATEWAY_LEVEL_RAW_MAX_LIMIT         UINT16_C(59025)
#define GATEWAY_LEVEL_RAW_RECOVERY_CURRENT_A          0.10F

/* KEY启动后先回到固定机械零点。位置/转速连续满足条件达到settle时间后，
 * 才允许使用F407本地BMI088纵向加速度补偿。单位依次为电机deg、rpm、ms。 */
#define GATEWAY_LEVEL_ZERO_ENTRY_TOL_DEG       1.50F
#define GATEWAY_LEVEL_ZERO_ENTRY_MAX_RPM      15.00F
#define GATEWAY_LEVEL_ZERO_ENTRY_SETTLE_MS UINT32_C(300)

/* 2026-07-31手动往返的JY61 Pitch中值表：raw减小、杆执行端抬升、Pitch增大。
 * 初始化时仍在机械零点54379处分段插值并减去安装偏置。表只覆盖严格单调的
 * 48000..57500；已剔除高raw机构死区，双向往返的剩余回差由内环吸收。 */
#define GATEWAY_ROD_ANGLE_CALIBRATION_POINTS \
    { \
        { UINT16_C(57500), 13.510F }, \
        { UINT16_C(57000), 13.800F }, \
        { UINT16_C(56500), 13.960F }, \
        { UINT16_C(56000), 14.380F }, \
        { UINT16_C(55500), 14.620F }, \
        { UINT16_C(55000), 15.200F }, \
        { UINT16_C(54500), 15.610F }, \
        { UINT16_C(54000), 16.040F }, \
        { UINT16_C(53500), 16.290F }, \
        { UINT16_C(53000), 16.790F }, \
        { UINT16_C(52500), 17.310F }, \
        { UINT16_C(52000), 17.520F }, \
        { UINT16_C(51500), 18.050F }, \
        { UINT16_C(51000), 18.440F }, \
        { UINT16_C(50500), 18.970F }, \
        { UINT16_C(50000), 19.730F }, \
        { UINT16_C(49500), 20.070F }, \
        { UINT16_C(49000), 20.275F }, \
        { UINT16_C(48500), 20.580F }, \
        { UINT16_C(48000), 21.110F }, \
    }

/* KEY启动闭环时允许的最大绝对转速，单位rpm；必须>=0。 */
#define GATEWAY_LEVEL_START_MAX_SPEED_RPM  1.0F

/* LevelCurrentControl内部仍使用相对固定机械零点的电机角，单位deg；滚球侧先
 * 通过实测杆角表逆查raw，再换算成此角域。 */
#define GATEWAY_LEVEL_TARGET_LIMIT_DEG     85.0F
#define GATEWAY_LEVEL_HARD_LIMIT_DEG       90.0F

/* ------------------------------------------------------------------ */
/* 球位置/速度外环                                                      */
/* ------------------------------------------------------------------ */

/* 全链路统一物理坐标：
 * - 杆轴x正方向从右侧固定转轴指向左侧执行端；水管几何中心为x=0 mm；
 * - 杆角theta正方向表示左侧执行端抬高；实测为theta增大时QD4310 raw减小；
 * - 相机画面右正，而画面右侧是固定转轴，因此x=-x_camera；
 * - BMI088安装为Sensor X朝车头；两次向车头手推的主动作实测均为处理后a_x先正后负；
 * - 执行端位于车头，车辆向前加速时应降低执行端，即theta<0；因此BMI088正向
 *   a_x通过前馈极性-1映射成负theta、raw增大；
 * - 实测负电流使raw减小并抬高执行端，故电机电流极性保持+1。
 * 这些定义统一视觉、双IMU杆角表、底盘IMU和无刷电机的符号；改装任何器件后
 * 必须重新核对，不能在上层临时叠加负号。 */
#define GATEWAY_BALL_VISUAL_CONTROL_ENABLE  1U
#define GATEWAY_BALL_MEASUREMENT_POLARITY  -1.0F
#define GATEWAY_BALL_TARGET_MM               0.0F

/* 25 cm水管和直径1 cm球对应球心几何域[-120,+120] mm。该值仅是软件
 * 安全边界，扩大实机目标仍须从小范围逐级验证。目标参考按80 mm/s线性
 * 推进，避免阶跃换向冲击。 */
#define GATEWAY_BALL_TARGET_LIMIT_MM        120.0F
#define GATEWAY_BALL_TARGET_SLEW_MM_S       120.0F
#define GATEWAY_BALL_MAX_TARGET_VELOCITY_MM_S 50.0F
#define GATEWAY_BALL_MAX_TARGET_ACCELERATION_MM_S2 200.0F

/* 位置PID直接生成目标球加速度：Kp*位置误差+Ki积分-Kd*视觉球速。
 * VTP字段名保持兼容；速度环不再参与正常运动和停车。 */
#define GATEWAY_BALL_POSITION_KP_PER_S2      0.60F
#define GATEWAY_BALL_POSITION_KI_PER_S3      0.000F
#define GATEWAY_BALL_VELOCITY_KD_PER_S       0.55F
#define GATEWAY_BALL_INTEGRAL_ACCEL_LIMIT_MM_S2 10.0F
/* 球速只由真实视觉位置计算：120 ms多帧差分适配当前约33 Hz有效视觉帧，
 * 再做60 ms一阶低通，避免两帧差分尖峰把杆角反复推向限幅。 */
#define GATEWAY_BALL_VELOCITY_ESTIMATION_WINDOW_MS UINT32_C(120)
#define GATEWAY_BALL_VELOCITY_FILTER_TAU_S   0.060F
#define GATEWAY_BALL_MAX_VELOCITY_DT_MS      UINT32_C(100)
/* 目标附近同时满足位置误差不超过3 mm、球速不超过8 mm/s并保持60 ms时，
 * 进入安静死区并锁定清零球环输出；
 * 锁定后不再因测量变化自动退出，仅在目标改变或测量状态复位时解锁。 */
#define GATEWAY_BALL_HOLD_POSITION_ERROR_MM     3.0F
#define GATEWAY_BALL_HOLD_MAX_VELOCITY_MM_S     8.0F
#define GATEWAY_BALL_HOLD_STABLE_DURATION_MS UINT32_C(60)
/* 实测近目标Kd跳变形成低频极限环，当前禁用该调度并统一使用基础Kd=2.5。 */
#define GATEWAY_BALL_NEAR_TARGET_ERROR_MM        0.0F
#define GATEWAY_BALL_NEAR_TARGET_KD_MULTIPLIER   0.0F

/* 离散角度脉冲保持禁用。大误差且低速持续120 ms后，短暂启用限幅速度辅助；
 * 一旦视觉检测到球移动立即撤销，正常运动和停车仍完全由位置PID负责。 */
#define GATEWAY_BALL_STICTION_KICK_ANGLE_DEG          0.00F
#define GATEWAY_BALL_STICTION_KICK_STEP_ANGLE_DEG     0.00F
#define GATEWAY_BALL_STICTION_KICK_MAX_ANGLE_DEG      0.00F
#define GATEWAY_BALL_STICTION_ASSIST_VELOCITY_KP_PER_S 3.50F
#define GATEWAY_BALL_STICTION_ASSIST_EXIT_DISTANCE_MM  18.0F
#define GATEWAY_BALL_STICTION_MIN_ERROR_MM            12.0F
#define GATEWAY_BALL_STICTION_MAX_VELOCITY_MM_S       20.0F
#define GATEWAY_BALL_STICTION_STATIONARY_MS        UINT32_C(100)
#define GATEWAY_BALL_STICTION_DURATION_MS          UINT32_C(250)
#define GATEWAY_BALL_STICTION_COOLDOWN_MS          UINT32_C(100)

/* 视觉串口实时调参的产品边界。协议值只有通过这些边界后才能进入控制器；
 * 角度、电流和raw保护仍在更下游独立生效。 */
#define GATEWAY_BALL_TUNING_KP_MAX_PER_S2     5.0F
#define GATEWAY_BALL_TUNING_KI_MAX_PER_S3     1.0F
#define GATEWAY_BALL_TUNING_KD_MAX_PER_S      5.0F

/* 实心钢球逆模型参数与球反馈杆角限幅。新标定证明负向机械单调余量约2.2deg，
 * 因此动态偏移限制为-2.0deg；正向仍按已整定的+0.75deg。水平trim在限幅后
 * 相加，两端均留在48000..57500实测单调表内。 */
#define GATEWAY_BALL_GRAVITY_MM_S2            9807.0F
#define GATEWAY_BALL_INERTIA_RATIO             0.4F
#define GATEWAY_BALL_ROD_ANGLE_LIMIT_DEG       2.00F
#define GATEWAY_TOTAL_ROD_COMMAND_MIN_DEG      (-2.00F)
#define GATEWAY_TOTAL_ROD_COMMAND_MAX_DEG        0.75F

/* 三状态[x,v,模型残差加速度]固定滞后Kalman保留用于离线对照，当前关闭。
 * 正常控制只使用真实视觉多帧差分球速，底盘加速度独立做杆角前馈。 */
#ifndef GATEWAY_BALL_KALMAN_ENABLE
#define GATEWAY_BALL_KALMAN_ENABLE                     0U
#endif
#define GATEWAY_BALL_KALMAN_POSITION_R_MM2            36.0F
#define GATEWAY_BALL_KALMAN_MODEL_ACCEL_Q_MM2_S3    2000.0F
#define GATEWAY_BALL_KALMAN_BIAS_WALK_Q_MM2_S4_S     100.0F
#define GATEWAY_BALL_KALMAN_INITIAL_POSITION_P_MM2     25.0F
#define GATEWAY_BALL_KALMAN_INITIAL_VELOCITY_P_MM2_S2 90000.0F
#define GATEWAY_BALL_KALMAN_INITIAL_BIAS_P_MM2_S4  250000.0F
#define GATEWAY_BALL_KALMAN_INNOVATION_GATE_SIGMA       2.5F
/* Independent hard gate prevents a large covariance from accepting a
 * physically impossible inter-frame vision jump into the velocity state. */
#define GATEWAY_BALL_KALMAN_MAX_INNOVATION_MM           12.0F
#define GATEWAY_BALL_KALMAN_MAX_POSITION_MM            150.0F
#define GATEWAY_BALL_KALMAN_MAX_VELOCITY_MM_S         1500.0F
#define GATEWAY_BALL_KALMAN_MAX_BIAS_MM_S2            1000.0F
#define GATEWAY_BALL_KALMAN_TRUST_POSITION_P_MM2       100.0F
#define GATEWAY_BALL_KALMAN_TRUST_VELOCITY_P_MM2_S2   3600.0F
#define GATEWAY_BALL_KALMAN_MAX_PREDICT_STEP_MS UINT32_C(10)
/* 无新视觉校正时继续使用杆球模型预测的最长时间。它只控制估计输出保持，
 * 不放宽VisionLink对单帧测量年龄的100 ms入帧校验。 */
#define GATEWAY_BALL_KALMAN_MAX_COAST_MS        UINT32_C(300)
#define GATEWAY_BALL_KALMAN_MIN_ACCEPTED_VIS          UINT32_C(3)

/* 底盘三状态[v,a,IMU零偏]融合器。首轮参数仅依据100 Hz轮速/IMU数据源和旧电机响应
 * 预测：规划输入纯延迟30 ms、一阶时间常数0.12 s。代码接入但必须保持0，直到ACC_MEAS
 * 的轴向、重力补偿、轮速尺度、延迟和Q/R由实车日志确认。 */
#ifndef GATEWAY_CHASSIS_ACCEL_FUSION_ENABLE
#define GATEWAY_CHASSIS_ACCEL_FUSION_ENABLE                 0U
#endif
#define GATEWAY_CHASSIS_FUSION_PLAN_GAIN                  1.0F
#define GATEWAY_CHASSIS_FUSION_RESPONSE_TAU_S            0.12F
#define GATEWAY_CHASSIS_FUSION_PLAN_DELAY_MS       UINT32_C(30)
#define GATEWAY_CHASSIS_FUSION_WHEEL_R_MM2_S2           225.0F
#define GATEWAY_CHASSIS_FUSION_IMU_R_MM2_S4           14400.0F
#define GATEWAY_CHASSIS_FUSION_VELOCITY_Q_MM2_S3        400.0F
#define GATEWAY_CHASSIS_FUSION_ACCEL_Q_MM2_S5        160000.0F
#define GATEWAY_CHASSIS_FUSION_BIAS_Q_MM2_S5              4.0F
#define GATEWAY_CHASSIS_FUSION_INITIAL_VELOCITY_P_MM2_S2 900.0F
#define GATEWAY_CHASSIS_FUSION_INITIAL_ACCEL_P_MM2_S4  10000.0F
#define GATEWAY_CHASSIS_FUSION_INITIAL_BIAS_P_MM2_S4    6400.0F
#define GATEWAY_CHASSIS_FUSION_GATE_SIGMA                  5.0F
#define GATEWAY_CHASSIS_FUSION_MAX_VELOCITY_MM_S         2000.0F
#define GATEWAY_CHASSIS_FUSION_MAX_ACCEL_MM_S2           3000.0F
#define GATEWAY_CHASSIS_FUSION_MAX_BIAS_MM_S2            1000.0F
#define GATEWAY_CHASSIS_FUSION_TRUST_ACCEL_P_MM2_S4     40000.0F
#define GATEWAY_CHASSIS_FUSION_MAX_PREDICT_STEP_MS UINT32_C(10)
#define GATEWAY_CHASSIS_FUSION_MAX_MEAS_AGE_MS     UINT32_C(30)
#define GATEWAY_CHASSIS_FUSION_MIN_WHEEL_SAMPLES   UINT32_C(3)
#define GATEWAY_CHASSIS_FUSION_MIN_IMU_SAMPLES     UINT32_C(3)

/* BMI088本地纵向加速度预处理。实车动态标定确认Sensor Y平行车身，向前
 * 加速时Ay先增大，因此车身向前正方向直接取+Sensor Y。上电后前300个
 * 连续样本求静止Ay零偏并冻结；完成前不向电机控制提交加速度。1kHz下约
 * 需300ms。控制链不使用Pitch、Gyro Y或Z轴重力补偿。 */
#define GATEWAY_BMI088_ACCEL_Y_BIAS_CALIBRATION_SAMPLES UINT32_C(300)
/* 纵向输出先用1kHz RBJ陷波抑制实测约14Hz机构回灌，再使用10Hz二阶
 * Butterworth低通。实测噪声主要来自电机使能后的机构耦合；该组合保留
 * 1~3Hz加速度前馈响应，同时压低机械高频振动。截止频率单位Hz。 */
#define GATEWAY_BMI088_ACCEL_NOTCH_SAMPLE_HZ       1000.0F
#define GATEWAY_BMI088_ACCEL_NOTCH_CENTER_HZ         14.0F
#define GATEWAY_BMI088_ACCEL_NOTCH_QUALITY_FACTOR     0.7F
#define GATEWAY_BMI088_ACCEL_LOWPASS_CUTOFF_HZ        10.0F
#define GATEWAY_BMI088_ACCEL_MAX_ABS_OUTPUT_MM_S2    40000.0F
#define GATEWAY_BMI088_ACCEL_MAX_SAMPLE_STEP_MS   UINT32_C(5)
#define GATEWAY_BMI088_ACCEL_MAX_INPUT_AGE_MS     UINT32_C(10)

/* 动态实测：Sensor Y平行车身，向前加速的主峰为正；Sensor Z为重力轴。 */
#define GATEWAY_BMI088_ACCEL_Y_SIGN                    1.0F

/* 底盘加速度补偿：theta=b*polarity*atan2(a_forward,g)。KEY启动并完成零点稳定后，
 * 当前版本只使用F407本地BMI088经启动零偏和两级2.25ms低通处理的+Y测量，完全剔除
 * G3519/ICM45686的CAR加速度。+Y朝车头时向前加速度a_forward>0，极性-1将其映射为
 * theta<0、raw增大并降低车头执行端。
 * 新VOFA中0.50增益仍有70.57%时间撞限幅，因此首轮降至0.20；离线回放预计
 * 静车饱和率约1.6%。补偿与视觉+BMI088动态和均限0.60deg，加上水平trim后的
 * 最终命令仍受杆角表和绝对raw边界约束。 */
#define GATEWAY_CHASSIS_FF_GAIN                0.20F
#define GATEWAY_CHASSIS_FF_POLARITY           (-1.0F)
#define GATEWAY_CHASSIS_FF_ANGLE_LIMIT_DEG      0.60F
#define GATEWAY_ACC_PREVIEW_LEAD_MS            UINT32_C(0)
#define GATEWAY_ACC_TRANSPORT_BOUND_MS         UINT32_C(0)

/* 实机水平闭环参数：角度 P、电流积分 I、转速反馈阻尼 D。
 * 单位依次为 A/deg、A/(deg*s)、A/rpm；增益均需>=0。
 * 目标点使用负静态前馈承重；当位置误差请求相反方向时，前馈在DEADBAND范围内
 * 连续释放，并在P项开始作用前降为0，避免负前馈压过raw增大的下行请求。 */
#define GATEWAY_LEVEL_KP_A_PER_DEG          0.014F
#define GATEWAY_LEVEL_KI_A_PER_DEG_S        0.0020F
#define GATEWAY_LEVEL_KD_A_PER_RPM          0.0010F
#define GATEWAY_LEVEL_FEEDFORWARD_CURRENT_A (-0.090F)

/* 角误差限幅、死区和积分作用区，单位 deg。先限幅再扣除死区；仅在
 * |原始误差|<=INTEGRAL_ZONE 时积分，离开该区立即清积分。 */
#define GATEWAY_LEVEL_ERROR_LIMIT_DEG       10.0F
#define GATEWAY_LEVEL_DEADBAND_DEG           0.05F
#define GATEWAY_LEVEL_INTEGRAL_ZONE_DEG      2.0F

/* 积分分量和最终电流的对称限幅，单位 A。控制电流限幅不得超过 Device
 * 产品侧电流限幅；POLARITY 只能是 +1 或 -1，用于实机确认电机方向。 */
#define GATEWAY_LEVEL_INTEGRAL_LIMIT_A       0.020F
#define GATEWAY_LEVEL_CURRENT_LIMIT_A        0.30F
#define GATEWAY_LEVEL_CURRENT_POLARITY       1.0F

#if GATEWAY_LEVEL_ZERO_RAW > UINT16_MAX
#error "Level zero raw value must fit uint16_t"
#endif

#if GATEWAY_LEVEL_RAW_MIN_LIMIT >= GATEWAY_LEVEL_RAW_MAX_LIMIT
#error "Level raw limits must be strictly increasing"
#endif

#if (GATEWAY_LEVEL_RAW_CONTROL_MAX <= GATEWAY_LEVEL_RAW_MIN_LIMIT) || \
    (GATEWAY_LEVEL_RAW_CONTROL_MAX >= GATEWAY_LEVEL_RAW_MAX_LIMIT)
#error "Level raw control maximum must be inside the hard raw limits"
#endif

#if (GATEWAY_LEVEL_ZERO_RAW < GATEWAY_LEVEL_RAW_MIN_LIMIT) || \
    (GATEWAY_LEVEL_ZERO_RAW > GATEWAY_LEVEL_RAW_MAX_LIMIT)
#error "Level zero raw value must be inside manual raw limits"
#endif

#if (GATEWAY_LEVEL_TRIM_RAW < GATEWAY_LEVEL_RAW_MIN_LIMIT) || \
    (GATEWAY_LEVEL_TRIM_RAW > GATEWAY_LEVEL_RAW_MAX_LIMIT)
#error "Level trim raw value must be inside manual raw limits"
#endif

#if GATEWAY_LEVEL_CALIBRATION_KEY_MODE > 1U
#error "Level calibration key mode must be 0 or 1"
#endif

#if (GATEWAY_LEVEL_CALIBRATION_START_RAW > GATEWAY_LEVEL_RAW_MAX_LIMIT) || \
    (GATEWAY_LEVEL_CALIBRATION_START_RAW <= GATEWAY_LEVEL_RAW_MIN_LIMIT)
#error "Calibration start raw must be at or below the maximum raw limit"
#endif

#if (GATEWAY_LEVEL_CALIBRATION_SPAN_RAW == 0U) || \
    (GATEWAY_LEVEL_CALIBRATION_SPAN_RAW >= UINT32_C(32768))
#error "Calibration raw span must be non-zero and below half a turn"
#endif

#if GATEWAY_LEVEL_CALIBRATION_ONE_WAY_POINTS < 2U
#error "Calibration scan needs at least two points"
#endif

#if (GATEWAY_LEVEL_CALIBRATION_START_RAW + \
     GATEWAY_LEVEL_CALIBRATION_SPAN_RAW) != \
        GATEWAY_LEVEL_CALIBRATION_TURN_RAW
#error "Calibration turn raw must match the contracted downward span"
#endif

#if GATEWAY_LEVEL_CALIBRATION_STABLE_TOL_RAW == 0U
#error "Calibration stable raw tolerance must be non-zero"
#endif

#if (GATEWAY_LEVEL_CALIBRATION_STABLE_TIME_MS == 0U) || \
    (GATEWAY_LEVEL_CALIBRATION_HOLD_TIME_MS == 0U) || \
    (GATEWAY_LEVEL_CALIBRATION_POINT_TIMEOUT_MS <= \
        (GATEWAY_LEVEL_CALIBRATION_STABLE_TIME_MS + \
         GATEWAY_LEVEL_CALIBRATION_HOLD_TIME_MS))
#error "Calibration timing must allow stable confirmation and reporting hold"
#endif

#if GATEWAY_BALL_VISUAL_CONTROL_ENABLE > 1U
#error "Ball visual control enable must be 0 or 1"
#endif

#if GATEWAY_BALL_KALMAN_ENABLE > 1U
#error "Ball Kalman enable must be 0 or 1"
#endif

#if GATEWAY_CHASSIS_ACCEL_FUSION_ENABLE > 1U
#error "Chassis acceleration fusion enable must be 0 or 1"
#endif

#if GATEWAY_LEVEL_CONTROL_PERIOD_MS != GATEWAY_TASK_PERIOD_MS
#error "Level controller period must match the Gateway Task period"
#endif

#if GATEWAY_CAN_SEND_TIMEOUT_MS >= GATEWAY_TASK_PERIOD_MS
#error "CAN send timeout must be shorter than the Gateway Task period"
#endif

/* ------------------------------------------------------------------ */
/* MCU 板间 USART1 心跳                                                  */
/* ------------------------------------------------------------------ */

/* F407 为节点 1，对端 MCU 为节点 2；节点 ID 必须非零且不同。 */
#define PEER_LINK_LOCAL_NODE_ID           UINT8_C(1)
#define PEER_LINK_REMOTE_NODE_ID          UINT8_C(2)

/* 心跳发送周期与离线超时，单位 ms。超时应不小于一个心跳周期。 */
#define PEER_LINK_HEARTBEAT_PERIOD_MS     UINT32_C(500)
#define PEER_LINK_DEBUG_PERIOD_MS         UINT32_C(20)
#define PEER_LINK_OFFLINE_TIMEOUT_MS      UINT32_C(1500)

/* 现有1 ms Task每周期最多解析的USART1字节数，范围1..128。 */
#define PEER_LINK_RX_BYTES_PER_STEP       128U

/* ------------------------------------------------------------------ */
/* MaixCAM Pro视觉链路（USART2 PA2/PA3候选接线）                         */
/* ------------------------------------------------------------------ */

#define VISION_LINK_UART_INDEX            2U
#define VISION_LINK_RX_BYTES_PER_STEP     128U
#define VISION_LINK_SESSION_TIMEOUT_MS    UINT32_C(100)
#define VISION_LINK_MEASUREMENT_MAX_AGE_MS UINT32_C(100)
/* 47字节VIS在115200 8N1的纯串行时间向上取5ms。相机曝光/流水线固定延迟
 * 尚未实测，所以CAL先置0，只能用于功能调试。 */
#define VISION_LINK_CAMERA_PIPELINE_CAL_MS UINT32_C(0)
#define VISION_LINK_TRANSPORT_BOUND_MS     UINT32_C(5)
#define VISION_LINK_STATUS_TX_PERIOD_MS    UINT32_C(20)

#endif
