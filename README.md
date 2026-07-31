# MaixCAM2横向水管凹槽钢球位置实验

本目录是从已提交的竖向版本 `steelball_test_pro/` 复制出的独立上板工程，原版本
不再修改。新车结构中的半截水管凹槽横跨画面左右，因此算法改为逐列扫描上下边。

## 数据流

```text
当前帧 -> YOLO钢球框 -----------------------------┐
       -> LAB白色二值 -> 步长8粗扫 + 端点精扫 -> 折线中轴 ├-> 球心投影
                                                     └-> position_cm/px/norm/t
```

当前按凹槽已知长度25cm做线性厘米换算，但不进行旧机位透视修正：

- `position_cm`：以凹槽中点为0，默认左负右正，范围约 `-12.5~+12.5cm`；
- `position_px`：以凹槽中点为0，默认左负右正的轴向像素误差；
- `position_norm`：同一位置归一化到约 `-1~+1`；
- `t`：从有向端点P0到P1的 `0~1` 比例。

## 文件职责

- `main.py`：相机、YOLO、横向凹槽调度、显示和结构化日志；
- `vision_config.py`：模型、白色阈值、横向ROI、逐列扫描与调试开关；
- `ball_target_touch.py`：底部毫米目标滑条、松手提交、VST确认与有限周期重发；
- `horizontal_trough_tracker.py`：粗扫白段、端点逐列精扫、直线离群筛选和轻量折线中轴；
- `ball_axis_position.py`：YOLO候选筛选、球心到折线投影及弧长位置；
- `f4_vision_link.py`：MaixCAM2 UART2、Packet V1、VIS发送、VST接收与链路诊断；
- `kalman_tracker.py`：YOLO有框时直通，只在短时漏检时预测；
- `rtsp_debug_stream.py`：可选原始画面RTSP，首轮算法测试默认关闭；
- `app.yaml/app.png`：MaixVision离线应用清单与图标。

### F407通信

MaixCAM2使用B0 `UART2_TX`、B1 `UART2_RX`与F407 USART2通信，设备为
`/dev/ttyS2`，参数为115200 8N1。程序最多50 Hz发送合作协议第11节定义的`VIS`帧，
并保留`VST`解析；该链路与G3519↔F407的USART1 `HB/CAR`完全独立。

屏幕底部滑条可选择`-120..+120 mm`整数目标。拖动时只更新屏幕选择值，松手后才以
独立序号的`VBT`发送；首次收到`VST`且用户尚未操作时，滑条先同步F407当前目标，避免
应用重启时误发`0 mm`。`VST.target_mm`匹配时显示`OK`，等待确认时显示`PENDING`并每
250 ms重发，最多20次；500 ms没有VST时显示`OFFLINE`。该目标只是小球在水管中的位置：F407仍按
`Kp*(target-position)+I-Kd*velocity`和实心球逆模型计算杆角，再叠加底盘加速度前馈；
MaixCAM不直接发送杆角，也不控制QD4310内部位置环。触屏设置不会启动电机。
滑条沿用画面坐标“左负右正”；发送和显示VST确认时通过
`F4_TOUCH_TARGET_TO_F407_POLARITY=-1`转换为F407杆球物理坐标“左正右负”，与VIS测量
进入控制器时的`measurement_polarity=-1`保持同一误差方向。

出现`F4_UART_OPEN`表示串口打开，出现`F4_VIS_TX_STARTED`表示至少完整写出一帧；
F407端仍须以`vis_rx_count`递增确认实收。正式标定流程中
`F4_UART_CALIBRATION_VALID`应保持关闭，完成九点与端点毫米标定后再打开。

当前按用户要求启用临时直接控制模式：启动阈值菜单和快速位置标定均关闭，程序直接使用
配置中的颜色阈值、动态凹槽端点和25 cm线性换算发送坐标，同时置位VIS标定有效标志，
使F407可以消费该坐标。该模式没有现场点标定精度保证；正式运行前应恢复两个启动开关并
将`F4_UART_CALIBRATION_VALID`改回`False`完成验证。

## 首轮上板配置

```python
TROUGH_COLOR_MODE = "GREEN"  # 也可切换为"WHITE"
TROUGH_WHITE_LAB_THRESHOLDS = ((77, 100, -18, 18, -18, 22),)
TROUGH_ROI_RATIO = (0.00, 0.45, 1.00, 0.55)
TROUGH_ROI_MASK_ENABLED = True
TROUGH_COLUMN_STEP = 8
TROUGH_ENDPOINT_REFINE_ENABLED = True
TROUGH_ENDPOINT_REFINE_RADIUS_PX = 12
TROUGH_CURVED_CENTERLINE_ENABLED = True
TROUGH_CENTERLINE_SEGMENTS = 6
TROUGH_CENTERLINE_RESIDUAL_FILTER_ENABLED = True
TROUGH_CENTERLINE_RESIDUAL_FILTER_ALPHA = 0.20
TROUGH_CENTERLINE_RESIDUAL_MAX_STEP_PX = 1.5
TROUGH_UPDATE_INTERVAL_FRAMES = 1
TROUGH_POSITIVE_AT_RIGHT = True
POSITION_MAPPING_ENABLED = True
TROUGH_PHYSICAL_LENGTH_CM = 25.0
STARTUP_THRESHOLD_TUNING_ENABLED = False
QUICK_POSITION_CALIBRATION_ENABLED = False
F4_UART_CALIBRATION_VALID = True
RTSP_DEBUG_ENABLED = False
```

噪声较多时的扫描保护参数：

```python
TROUGH_GUIDED_SCAN_HALF_HEIGHT_PX = 28
TROUGH_MAX_INTERNAL_GAP_PX = 6
TROUGH_HEIGHT_CHANGE_WEIGHT = 0.15
TROUGH_GAP_PENALTY_WEIGHT = 0.12
```

当前装车测试使用宽阈值：

```python
TROUGH_GREEN_THRESHOLD_PRESET = "WIDE"
TROUGH_GREEN_LAB_THRESHOLDS_WIDE = ((35, 95, -60, -5, 5, 70),)
```

若背景噪声重新增多，可切到 `"TIGHT"` 对比；宽阈值是当前实测调参结果，不是跨光照条件的最终标定值。

已有中轴时只在预测线附近扫描；较小的内部间隙合并阈值避免把背景反光连接成厚白段；轨迹选择按连续覆盖率、横向跨度、中心偏差和厚度变化综合排序，不再单纯按候选点数最长优先。若真实水管因遮挡出现断裂，可逐步把 `TROUGH_MAX_INTERNAL_GAP_PX` 调回 `8~10`，不要直接恢复到很大的间隙。

端点保持动态，不写死为屏幕坐标，但增加了三层保护：

- `TROUGH_ENDPOINT_MIN_ASPECT_RATIO`：要求扫描轨迹满足横向细长形状；
- `TROUGH_ENDPOINT_MIN_SUPPORT_POINTS`：统计左右端点附近的连续采样点；
- 中轴滤波器在跨度突然缩小时保持上一帧端点，并限制单帧端点最大移动量。

因此仍然会随真实视场变化调整端点，但不会因为一小段反光突然向外伸长，或因为一帧漏检突然缩回。主体长度、细长度和RMS合格时，单侧端点支撑不足不会提前否决整条轴，而是继续交给YOLO/历史端点遮挡逻辑；只有左右两侧都弱时才拒绝。

主体列使用步长8降低扫描量。粗轨迹通过质量门后，只从左右粗端点各向外最多12像素做步长1精扫；遇到第一列不匹配即停止，并且不会穿过YOLO排除区。直线 `y=a*x+b` 仍负责鲁棒剔除离群点，但最终中轴由6段7节点折线表示；每个节点使用所在区段相对直线残差的中位数，因此能贴合轻微径向畸变，同时避免单点噪声直接拉弯中轴。

钢球不再只投影到P0-P1直线，而是投影到最近折线段；`t`、`position_px`和25cm线性厘米值均按折线累计弧长计算。该方法修正的是中轴弯曲和端点采样量化，不能替代最终装车后的0/5/10/15/20/25cm一维打表。

折线局部弯曲残差单独使用EMA和单帧限幅，整体直线的平移、倾斜与动态端点仍沿用原中轴滤波，因此车辆运动时不会为了压住局部扭曲而拖慢整条中轴。YOLO框贴近画面端部时直接按YOLO保持端点；钢球位于左右半段但尚未贴边时，只有颜色缺口边界与YOLO框相邻才补全该侧，并标记为 `YOLO_MASK_LEFT/RIGHT`。YOLO框位于中部且与远端缺口无关时不补全；YOLO无框时仍保留 `MASK_LEFT/RIGHT` 兜底。

正常运行时，`TROUGH_ROI_MASK_ENABLED=True` 会先用官方 `Image.crop()` 裁出ROI，再做LAB二值化，减少全帧复制和颜色处理。若裁剪API异常，程序自动回退全帧并输出 `mask_scope=FULL_FALLBACK`；打开 `SHOW_TROUGH_BINARY=True` 时入口会临时停用裁剪，保证屏幕显示完整尺寸二值图，因此调阈值时性能会低于正式模式。

## VOFA+无线波形调试

VOFA+截图中的“本地端口”是电脑接收端口。若保持默认配置，板端向
`255.255.255.255:1347`广播，VOFA+选择UDP/FireWater并监听本地端口`1347`即可。
仅做单向遥测时，截图中的远程IP `127.0.0.1` 和远程端口 `1346` 可以保持不变；
它们只在VOFA+主动向外发送数据时使用。
电脑防火墙需要允许UDP 1347入站；如果广播被网络屏蔽，将
`VOFA_UDP_HOST`改为电脑的实际Wi-Fi地址。

## MaixPy 真机 AI 调试

官方当前配置要求与本工程的对应关系：

- `nn.YOLO11` 需要 MaixPy 4.7.0 或更新版本；
- MaixCAM2 模型包的 `.mud` 和其引用的 `.axmodel` 必须放在设备同一目录，
  本工程入口为
  `/root/mymodel/steelball_yolo11n_v32_best_320x416_maixcam2.mud`；
- 相机宽、高和图像格式由 `detector.input_width()`、`input_height()`、
  `input_format()`读取，避免手写尺寸与模型不一致；
- 本工程保持 `dual_buff=False`。官方说明双缓冲虽然提高吞吐量，但
  `detect()`返回上一帧结果；位置闭环和标注图调试要求框与当前帧对齐；
- 屏幕只使用 `display.Display()`，不直接访问 framebuffer。

先运行无工程内依赖的单文件探针，分别验证模型、相机、NPU和显示：

```powershell
py -3 <maixpy-skill-dir>\scripts\maixpy_skill\cli.py run `
  .\steelball_horizontal_trough_pro\maixpy_ai_probe.py `
  --debug-images --timeout 30
```

运行记录保存在当前目录的 `.maixpy/runs/<run_id>/`。重点检查
`summary.json`、`stdout.log`、`stderr.log`和`artifacts/`；探针正常结束时应看到
`MAIXPY_AI_CONFIG`、`MAIXPY_AI_DONE`和`MAIXPY_AI_RESOURCES_CLOSED`。

完整应用也支持同一套图像回传环境变量，但默认关闭。只有通过运行器显式传入
`--debug-images`时，才会周期性覆盖保存当前屏幕调试图；正常离线运行不会写图，
不会引入闪存写入和JPEG编码开销。

官方参考：

- [YOLO目标检测](https://wiki.sipeed.com/maixpy/doc/zh/vision/yolov5.html)
- [dual_buff说明](https://wiki.sipeed.com/maixpy/doc/zh/vision/dual_buff.html)
- [AI模型下载、调试和部署](https://wiki.sipeed.com/maixpy/doc/zh/ai_model_converter/ai_model_deploy.html)
- [MaixCAM2快速开始](https://wiki.sipeed.com/maixpy/doc/zh/README_MaixCAM2.html)

```python
VOFA_UDP_ENABLED = True
VOFA_UDP_HOST = "255.255.255.255"
VOFA_UDP_PORT = 1347
VOFA_UDP_INTERVAL_MS = 25
```

FireWater每行7个数值，依次为：

```text
position_mm,valid,predicted,axis_source,score_pct,dt_ms,sequence
```

VOFA调试数据只用于观察波形，不参与控制闭环；正式控制仍应使用独立的UART二进制
帧。该模块不启动或停止Wi-Fi，运行前需先让板端和电脑处于同一Wi-Fi/AP网络。

第一次上板建议临时设置：

```python
SHOW_TROUGH_BINARY = True
```

## 2026-07-30 中轴滤波与测量时序字段

中轴检测在当前帧有效且置信度达到
`TROUGH_AXIS_FILTER_MIN_CONFIDENCE` 时，对拟合直线的斜率、截距、横向覆盖范围和宽度做EMA。
小幅抖动进入 `EMA`，超过 `TROUGH_AXIS_FILTER_SNAP_SHIFT_PX` 或
`TROUGH_AXIS_FILTER_SNAP_SLOPE_DELTA` 的变化进入 `SNAP`，避免滤波滞后。
低置信度直线短时间使用上一次可靠中轴，并在日志中标记为 `CACHED`。

`BALL_AXIS` 日志新增：

```text
frame_ts_ms=帧读出完成时间|measure_age_ms=结果准备时距离帧读出的毫秒数|send_ts_ms=发送准备时间
axis_filter=INIT/EMA/SNAP|axis_conf=中轴置信度|axis_filter_shift_px=滤波前后中心线偏移
```

VOFA+ FireWater新增字段顺序为：

```text
position_mm,valid,predicted,axis_source,score_pct,tx_dt_ms,udp_sequence,measurement_age_ms,frame_timestamp_ms,frame_index
```

`frame_ts_ms`是`cam.read()`完成时刻的近似帧时间，不能等同于传感器曝光时间；F407可用
`measurement_age_ms`估算测量年龄。当前工程已通过独立UART模块发送相同时间语义的`VIS`，
VOFA和结构化日志继续用于对照验证。

调试显示开关：

```python
DISPLAY_EVERY_FRAME = True
```

打开后每一帧都绘制并刷新原图、YOLO框、凹槽中轴和钢球投影点；关闭后恢复
`DISPLAY_INTERVAL_FRAMES`的隔帧显示。该开关只影响显示，不改变视觉算法和数据输出频率。

二值图中横向凹槽主体应连续为白，背景尽量为黑。黄色框是横向ROI；若开启采样
点，绿色/蓝色点分别表示每个粗扫白段的上边和下边；青色折线是轻量弯曲中轴，黄色端点是P0，
紫色端点是P1，紫色十字是钢球中心在中轴上的投影。

当前ROI依据装车实测设为全宽且下边贴底，上边界位于画面高度的45%；在416x320
模型画面中对应`x=0, y=144, width=416, height=176`。凹槽上方仍保留余量，
用于覆盖车辆振动、管体倾斜和安装轻微位移。

钢球到达左右端部并截断管体颜色时，端点采用单侧遮挡保持：YOLO框贴近哪一侧，
就只保持该侧的历史端点，另一侧和中轴仍正常动态更新。若程序启动时钢球已经位于
端部，有YOLO框时直接以YOLO确认；无框但二值图只有一侧出现球宽量级缺口、另一侧
仍贴住ROI边缘时，以`BOOTSTRAP_MASK_LEFT/RIGHT`从ROI边缘低置信度初始化。日志中的
`endpoint_filter`和`endpoint_mode`可区分YOLO确认、掩膜补全和释放保持。
若启动时左右两端都不贴边或同时出现缺口，单帧颜色掩膜无法唯一判断被遮挡侧，
程序会保持`CLEAR`而不强行猜测；等待YOLO恢复或钢球稍微移动后再建立可靠端点。
`BOOTSTRAP_MASK_*`只是无YOLO时的启发式近似，不能当作已经识别到真实物理管口。

若凹槽在画面上下范围较固定，优先收紧 `TROUGH_ROI_RATIO` 的y和height；不要通过
放宽白色阈值把白纸、灯光和浅灰车体一并选入。凹槽阴影断裂时再逐步降低L下限。

## 日志

启动日志应包含：

```text
STEELBALL_HORIZONTAL_START|...|trough_color=GREEN|mapping_enabled=1|trough_length_cm=25.00|...
```

运行日志：

```text
BALL_AXIS|frame=...|valid=1|position_cm=...|position_px=...|position_norm=...|t=...|
trough_valid=1|trough_scan=GUIDED|...
```

`MEASURED`表示本帧重新测量凹槽，`HELD`表示按配置隔帧复用，`CACHED`表示本帧
提取失败后的短时缓存。闭环不能长期使用无效或过期结果。

端部与性能排错重点看：

```text
trough_fallback=本次失败原因（CACHED时trough_reason仍可能是上一条轴的OK）
endpoint_left_support/endpoint_right_support=左右端点带内支撑率
endpoint_quality=BOTH_STRONG/LEFT_WEAK/RIGHT_WEAK/BOTH_WEAK
endpoint_refine=NONE/LEFT/RIGHT/BOTH
endpoint_refine_left_px/endpoint_refine_right_px=精扫相对粗端点向外延伸的像素
centerline=POLYLINE|centerline_nodes=7|centerline_curve_px=相对鲁棒直线的最大弯曲像素
centerline_residual_filter=INIT/EMA/DISABLED|centerline_residual_shift_px=局部残差滤波相对本帧原始值的最大修正像素
axis_p0_x/axis_p1_x=经过端点精扫和时间滤波后的有向端点横坐标
mask_scope=ROI/FULL/FULL_FALLBACK/HELD
```

`measure_age_ms`按MaixCDK的 `ticks_diff(last, now)` 参数顺序计算。若
`send_ts_ms-frame_ts_ms`约为几十毫秒而该字段长期为0，说明板端仍在运行旧入口文件。

## 当前边界

- 横向逐列访问图像内存不如逐行连续访问友好，MaixCAM2实际FPS必须重新测量；
- 端点仍来自颜色白带的有效范围，不等于已识别物理管口；当前通过细长形状、端点支撑和时间门控抑制漂移，`t/norm`仍需上板验证；
- 当前默认假设凹槽大致水平且完整横跨画面，轻微倾斜和弯曲可以拟合，但接近竖直不适用；
- 当前厘米值仅按 `position_cm=(t-0.5)*25` 线性换算，没有恢复旧竖向透视表；
  若最终五点实测误差超出要求，只对横向`t`重新做一维标定；
- 模型仍从 `/root/mymodel/` 加载，不包含在离线应用ZIP或Git中。
