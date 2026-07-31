"""轻量一维 alpha-beta 状态滤波器。

该模块只维护一个标量位置和速度，不依赖NumPy、OpenCV或MaixPy，便于在
MaixCAM板端运行，也便于PC纯逻辑回归。调用方负责先完成几何与质量门控；
本滤波器不能把错误端点变成正确端点。
"""


def _clamp(value, lower, upper):
    return max(float(lower), min(float(upper), float(value)))


class AlphaBetaFilter1D:
    """以毫秒时间戳更新的一维匀速 alpha-beta 滤波器。"""

    def __init__(self, alpha=0.70, beta=0.15,
                 max_velocity=240.0, max_predict=12.0,
                 max_innovation=48.0, reset_gap_ms=600,
                 clock_diff_ms=None):
        self.alpha = _clamp(alpha, 0.0, 1.0)
        self.beta = max(0.0, float(beta))
        self.max_velocity = max(0.0, float(max_velocity))
        self.max_predict = max(0.0, float(max_predict))
        self.max_innovation = max(0.0, float(max_innovation))
        self.reset_gap_ms = max(0, int(reset_gap_ms))
        self.clock_diff_ms = clock_diff_ms
        self.reset()

    def reset(self, position=None, timestamp_ms=None):
        self.initialized = position is not None
        self.position = float(position) if position is not None else 0.0
        self.velocity = 0.0
        self.last_timestamp_ms = (
            int(timestamp_ms) if timestamp_ms is not None else None)
        return self._result("INIT" if self.initialized else "RESET", 0.0, 0.0)

    def _elapsed_ms(self, timestamp_ms):
        if self.last_timestamp_ms is None:
            return 0
        now_ms = int(timestamp_ms)
        if self.clock_diff_ms is not None:
            try:
                return max(0, int(self.clock_diff_ms(
                    now_ms, self.last_timestamp_ms)))
            except Exception:
                pass
        return max(
            0,
            (now_ms - int(self.last_timestamp_ms)) & 0xFFFFFFFF,
        )

    def _prediction(self, dt_seconds):
        shift = self.velocity * max(0.0, float(dt_seconds))
        if self.max_predict > 0.0:
            shift = _clamp(shift, -self.max_predict, self.max_predict)
        return shift

    def _result(self, mode, prediction_px, innovation_px):
        return {
            "mode": str(mode),
            "position": float(self.position),
            "velocity": float(self.velocity),
            "prediction_px": float(prediction_px),
            "innovation_px": float(innovation_px),
        }

    def set_position(self, position, timestamp_ms=None, zero_velocity=False):
        """由调用方执行ROI/左右不交叉约束后同步受限位置。"""
        self.position = float(position)
        self.initialized = True
        if timestamp_ms is not None:
            self.last_timestamp_ms = int(timestamp_ms)
        if zero_velocity:
            self.velocity = 0.0

    def hold(self, timestamp_ms, zero_velocity=True, mode="HOLD"):
        """冻结位置；遮挡或测量失败时默认同时清除旧速度。"""
        if not self.initialized:
            return self._result("UNINITIALIZED", 0.0, 0.0)
        self.last_timestamp_ms = int(timestamp_ms)
        if zero_velocity:
            self.velocity = 0.0
        return self._result(mode, 0.0, 0.0)

    def predict(self, timestamp_ms):
        """按当前速度外推到指定时刻，并限制单次最大预测位移。"""
        if not self.initialized:
            return self._result("UNINITIALIZED", 0.0, 0.0)
        elapsed_ms = self._elapsed_ms(timestamp_ms)
        if self.reset_gap_ms > 0 and elapsed_ms > self.reset_gap_ms:
            return self.hold(timestamp_ms, zero_velocity=True, mode="HOLD_GAP")
        prediction_px = self._prediction(0.001 * elapsed_ms)
        self.position += prediction_px
        self.last_timestamp_ms = int(timestamp_ms)
        return self._result("PREDICT", prediction_px, 0.0)

    def correct(self, measurement, timestamp_ms):
        """先匀速预测，再用当前位置残差同时校正位置和速度。"""
        measured = float(measurement)
        if not self.initialized:
            return self.reset(measured, timestamp_ms)

        elapsed_ms = self._elapsed_ms(timestamp_ms)
        if self.reset_gap_ms > 0 and elapsed_ms > self.reset_gap_ms:
            return self.reset(measured, timestamp_ms)

        dt_seconds = 0.001 * elapsed_ms
        prediction_px = self._prediction(dt_seconds)
        predicted = self.position + prediction_px
        innovation = measured - predicted

        # 大残差通常表示相机/机构发生真实突变，或旧速度已经失效；此时直接
        # 重置比沿错误速度继续外推更安全。正常端点还会先经过调用方的单帧限幅。
        if self.max_innovation > 0.0 and \
                abs(innovation) > self.max_innovation:
            result = self.reset(measured, timestamp_ms)
            result["mode"] = "RESET_INNOVATION"
            result["prediction_px"] = float(prediction_px)
            result["innovation_px"] = float(innovation)
            return result

        self.position = predicted + self.alpha * innovation
        if dt_seconds > 1e-6:
            self.velocity += self.beta * innovation / dt_seconds
            if self.max_velocity > 0.0:
                self.velocity = _clamp(
                    self.velocity,
                    -self.max_velocity,
                    self.max_velocity,
                )
        self.last_timestamp_ms = int(timestamp_ms)
        return self._result("CORRECT", prediction_px, innovation)
