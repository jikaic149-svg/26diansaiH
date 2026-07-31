"""钢球YOLO的轻量单目标卡尔曼跟踪器。

本模块不依赖MaixPy或NumPy，便于在PC上做纯逻辑测试，也避免给
MaixCAM Pro增加大型运行依赖。只跟踪一个主钢球：YOLO正常时直接输出
本帧原始框，同时在内部更新卡尔曼状态；短暂漏检时才输出预测框，超过
限制立即判为丢失。
"""


class _AxisKalman:
    """一维恒速度卡尔曼：状态为位置和速度，测量只有位置。"""

    def __init__(self, process_accel_var, measurement_var):
        self.process_accel_var = float(process_accel_var)
        self.measurement_var = float(measurement_var)
        self.position = 0.0
        self.velocity = 0.0
        self.p00 = 400.0
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 2500.0

    def reset(self, position):
        self.position = float(position)
        self.velocity = 0.0
        self.p00 = 400.0
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 2500.0

    def predict(self, dt):
        dt = float(dt)
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        q = self.process_accel_var

        self.position += self.velocity * dt

        old_p00 = self.p00
        old_p01 = self.p01
        old_p10 = self.p10
        old_p11 = self.p11
        self.p00 = (
            old_p00 + dt * (old_p01 + old_p10) + dt2 * old_p11
            + 0.25 * dt4 * q
        )
        self.p01 = old_p01 + dt * old_p11 + 0.5 * dt3 * q
        self.p10 = old_p10 + dt * old_p11 + 0.5 * dt3 * q
        self.p11 = old_p11 + dt2 * q

    def correct(self, measurement):
        residual = float(measurement) - self.position
        innovation = self.p00 + self.measurement_var
        if innovation <= 0.0:
            return

        gain_position = self.p00 / innovation
        gain_velocity = self.p10 / innovation
        self.position += gain_position * residual
        self.velocity += gain_velocity * residual

        old_p00 = self.p00
        old_p01 = self.p01
        old_p10 = self.p10
        old_p11 = self.p11
        self.p00 = (1.0 - gain_position) * old_p00
        self.p01 = (1.0 - gain_position) * old_p01
        self.p10 = old_p10 - gain_velocity * old_p00
        self.p11 = old_p11 - gain_velocity * old_p01


class PrimaryBallTracker:
    """跟踪一个主钢球，并只在很短的YOLO漏检期间提供预测框。"""

    def __init__(
        self,
        max_predict_frames=2,
        max_predict_ms=350,
        min_gate_px=45.0,
        gate_diagonal_scale=1.25,
        process_accel_var=2500.0,
        measurement_var=36.0,
        size_measurement_weight=0.65,
    ):
        self.max_predict_frames = int(max_predict_frames)
        self.max_predict_ms = int(max_predict_ms)
        self.min_gate_px = float(min_gate_px)
        self.gate_diagonal_scale = float(gate_diagonal_scale)
        self.size_measurement_weight = float(size_measurement_weight)

        self.x_filter = _AxisKalman(process_accel_var, measurement_var)
        self.y_filter = _AxisKalman(process_accel_var, measurement_var)
        self.reset()

    def reset(self):
        self.initialized = False
        self.width = 0.0
        self.height = 0.0
        self.score = 0.0
        self.missed_frames = 0
        self.last_step_ms = None
        self.last_measurement_ms = None

    @staticmethod
    def _center(detection):
        return (
            float(detection["x"]) + 0.5 * float(detection["w"]),
            float(detection["y"]) + 0.5 * float(detection["h"]),
        )

    @staticmethod
    def _valid_detections(detections):
        return [
            detection for detection in detections
            if float(detection.get("w", 0.0)) > 1.0
            and float(detection.get("h", 0.0)) > 1.0
        ]

    def _start(self, detection, timestamp_ms):
        center_x, center_y = self._center(detection)
        self.x_filter.reset(center_x)
        self.y_filter.reset(center_y)
        self.width = float(detection["w"])
        self.height = float(detection["h"])
        self.score = float(detection.get("score", 0.0))
        self.missed_frames = 0
        self.last_step_ms = int(timestamp_ms)
        self.last_measurement_ms = int(timestamp_ms)
        self.initialized = True

    def _best_initial_detection(self, detections):
        return max(detections, key=lambda item: float(item.get("score", 0.0)))

    def _match_detection(self, detections):
        # 以当前预测框对角线生成关联门。第一版采用最近邻，不做多目标ID管理。
        diagonal = (self.width * self.width + self.height * self.height) ** 0.5
        gate = max(self.min_gate_px, self.gate_diagonal_scale * diagonal)
        best = None
        best_cost = None

        for detection in detections:
            center_x, center_y = self._center(detection)
            dx = center_x - self.x_filter.position
            dy = center_y - self.y_filter.position
            distance = (dx * dx + dy * dy) ** 0.5
            if distance > gate:
                continue

            # 距离是主条件；相近时略微偏向置信度较高的框。
            cost = distance / gate - 0.10 * float(detection.get("score", 0.0))
            if best_cost is None or cost < best_cost:
                best = detection
                best_cost = cost

        return best

    def _result(self, source):
        return {
            "x": self.x_filter.position - 0.5 * self.width,
            "y": self.y_filter.position - 0.5 * self.height,
            "w": self.width,
            "h": self.height,
            "cx": self.x_filter.position,
            "cy": self.y_filter.position,
            "vx": self.x_filter.velocity,
            "vy": self.y_filter.velocity,
            "score": self.score,
            "source": source,
            "missed": self.missed_frames,
        }

    def _raw_measurement_result(self, detection):
        """原样返回本帧YOLO框；速度字段只供诊断，不改变测量坐标。"""
        output = dict(detection)
        center_x, center_y = self._center(detection)
        output.update({
            "cx": center_x,
            "cy": center_y,
            "vx": self.x_filter.velocity,
            "vy": self.y_filter.velocity,
            "source": "YOLO_RAW",
            "missed": 0,
        })
        return output

    def update(self, detections, timestamp_ms):
        """YOLO有框时直通原始框；无框时短时返回卡尔曼预测框。"""
        timestamp_ms = int(timestamp_ms)
        detections = self._valid_detections(detections)

        if not self.initialized:
            if not detections:
                return None
            measured = self._best_initial_detection(detections)
            self._start(measured, timestamp_ms)
            return self._raw_measurement_result(measured)

        elapsed_ms = timestamp_ms - self.last_step_ms
        # 防止异常时间戳把预测一步推到很远；正常板端约0.1~0.2秒一帧。
        dt = max(0.02, min(0.35, elapsed_ms / 1000.0))
        self.x_filter.predict(dt)
        self.y_filter.predict(dt)
        self.last_step_ms = timestamp_ms

        matched = self._match_detection(detections)
        if matched is not None:
            center_x, center_y = self._center(matched)
            self.x_filter.correct(center_x)
            self.y_filter.correct(center_y)

            weight = self.size_measurement_weight
            self.width = weight * float(matched["w"]) + (1.0 - weight) * self.width
            self.height = weight * float(matched["h"]) + (1.0 - weight) * self.height
            self.score = float(matched.get("score", self.score))
            self.missed_frames = 0
            self.last_measurement_ms = timestamp_ms
            # 卡尔曼只更新内部速度/协方差；控制与显示使用本帧原始YOLO框。
            return self._raw_measurement_result(matched)

        if detections:
            # 本帧并未漏检，只是新框落在旧轨迹关联门外。单球任务中应立即
            # 用当前最高置信框重建轨迹，不能让旧卡尔曼预测覆盖真实YOLO框。
            measured = self._best_initial_detection(detections)
            self._start(measured, timestamp_ms)
            return self._raw_measurement_result(measured)

        self.missed_frames += 1
        lost_ms = timestamp_ms - self.last_measurement_ms
        if (
            self.missed_frames > self.max_predict_frames
            or lost_ms > self.max_predict_ms
        ):
            # 连续漏检超过限制后直接失效，不继续输出陈旧预测。
            self.reset()
            return None

        return self._result("PREDICTED")
