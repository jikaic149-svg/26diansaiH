"""VOFA+ FireWater UDP遥测模块。

本模块只负责把已经计算好的钢球位置发送到电脑，不负责启动或停止Wi-Fi，
也不参与UART闭环控制。这样关闭 ``VOFA_UDP_ENABLED`` 后不会改变视觉主线。

每个UDP数据报是一行FireWater数字：

    position_mm,valid,predicted,axis_source,score_pct,dt_ms,sequence\n
其中 ``position_mm`` 是横向凹槽中点为0、左负右正的物理位置；无效帧的位置
字段发送0，必须结合 ``valid`` 通道判断，不能把0误认为球在中点。
"""

import socket


AXIS_SOURCE_NONE = 0
AXIS_SOURCE_MEASURED = 1
AXIS_SOURCE_HELD = 2
AXIS_SOURCE_CACHED = 3


def axis_source_code(source):
    """把凹槽轴来源转换为VOFA+可显示的整数通道。"""
    normalized = str(source or "").strip().upper()
    return {
        "MEASURED": AXIS_SOURCE_MEASURED,
        "HELD": AXIS_SOURCE_HELD,
        "CACHED": AXIS_SOURCE_CACHED,
    }.get(normalized, AXIS_SOURCE_NONE)


def format_firewater_sample(position_mm=0.0, valid=False,
                            predicted=False, axis_source=0,
                            score_pct=-1.0, dt_ms=0.0, sequence=0,
                            measurement_age_ms=0.0,
                            frame_timestamp_ms=0, frame_index=0,
                            f4_status=None):
    """生成一行不带中文标签的FireWater数据。"""
    if not bool(valid):
        position_mm = 0.0
    position_mm = float(position_mm)
    score_pct = float(score_pct)
    if score_pct < 0.0:
        score_pct = -1.0
    else:
        score_pct = max(0.0, min(100.0, score_pct))
    f4 = f4_status or {}
    # 前10个字段保持原顺序；后12个字段追加F407球状态、杆角、电机、故障和mode。
    return (
        "%+.3f,%d,%d,%d,%.1f,%.3f,%d,%.3f,%d,%d,"
        "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n" % (
            position_mm,
            1 if valid else 0,
            1 if predicted else 0,
            int(axis_source),
            score_pct,
            max(0.0, float(dt_ms)),
            int(sequence) & 0xFFFF,
            max(0.0, float(measurement_age_ms)),
            int(frame_timestamp_ms) & 0xFFFFFFFF,
            int(frame_index) & 0xFFFFFFFF,
            int(f4.get("estimate_mm", 0)),
            int(f4.get("velocity_mm_s", 0)),
            int(f4.get("target_mm", 0)),
            int(f4.get("theta_cmd_mdeg", 0)),
            int(f4.get("motor_target_raw", 0)),
            int(f4.get("motor_feedback_raw", 0)),
            int(f4.get("motor_command_ma", 0)),
            int(f4.get("motor_speed_centi_rpm", 0)),
            int(f4.get("stiction_kick_mdeg", 0)),
            int(f4.get("status_flags", 0)),
            int(f4.get("fault_bits", 0)),
            int(f4.get("competition_mode", 0)),
        )
    ).encode("ascii")


def _elapsed_ms(now_ms, previous_ms):
    """处理Maix ticks_ms的32位回绕。"""
    return (int(now_ms) - int(previous_ms)) & 0xFFFFFFFF


class VofaUdpTelemetry:
    """可选、非阻塞的FireWater UDP发送器。"""

    def __init__(self, enabled=False, host="255.255.255.255",
                 port=1347, interval_ms=25, broadcast=True,
                 socket_factory=None, clock_ms=None):
        self.enabled = bool(enabled)
        self.host = str(host)
        self.port = int(port)
        self.interval_ms = max(0, int(interval_ms))
        self.broadcast = bool(broadcast)
        self.socket_factory = socket_factory or socket.socket
        self.clock_ms = clock_ms
        self.sock = None
        self.available = False
        self.sequence = 0
        self.last_send_ms = None
        self.tx_count = 0
        self.error_count = 0
        self.last_error = ""

    def _now_ms(self):
        if self.clock_ms is not None:
            return int(self.clock_ms())
        # MaixPy和CPython均有time模块，但不保证同名ticks_ms接口；
        # 这里仅作为UDP节流计时，不把它用于视觉时间戳。
        import time
        ticks = getattr(time, "ticks_ms", None)
        if ticks is not None:
            return int(ticks())
        return int(time.time() * 1000.0)

    def open(self):
        """打开UDP socket；失败时返回False，不抛出到视觉主循环。"""
        if not self.enabled:
            return False
        if self.available and self.sock is not None:
            return True
        try:
            self.sock = self.socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
            if self.broadcast:
                self.sock.setsockopt(
                    socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            try:
                self.sock.setblocking(False)
            except Exception:
                # 某些Maix固件的socket没有setblocking；UDP短包仍可发送。
                pass
            self.available = True
            self.last_error = ""
            print("VOFA_UDP_OPEN|host=%s|port=%d|interval_ms=%d" % (
                self.host, self.port, self.interval_ms))
            return True
        except Exception as error:
            self.sock = None
            self.available = False
            self.error_count += 1
            self.last_error = str(error)
            print("VOFA_UDP_OPEN_FAILED|error=%s" % self.last_error)
            return False

    def send_sample(self, position_mm=0.0, valid=False,
                    predicted=False, axis_source=0, score_pct=-1.0,
                    now_ms=None, measurement_age_ms=0.0,
                    frame_timestamp_ms=0, frame_index=0,
                    f4_status=None):
        """按发送间隔发送一帧FireWater数字，成功发送返回True。"""
        if not self.enabled or not self.available or self.sock is None:
            return False
        now_ms = self._now_ms() if now_ms is None else int(now_ms)
        if self.last_send_ms is not None:
            elapsed = _elapsed_ms(now_ms, self.last_send_ms)
            if elapsed < self.interval_ms:
                return False
        else:
            elapsed = 0
        payload = format_firewater_sample(
            position_mm=position_mm,
            valid=valid,
            predicted=predicted,
            axis_source=axis_source,
            score_pct=score_pct,
            dt_ms=elapsed,
            sequence=self.sequence,
            measurement_age_ms=measurement_age_ms,
            frame_timestamp_ms=frame_timestamp_ms,
            frame_index=frame_index,
            f4_status=f4_status,
        )
        try:
            written = self.sock.sendto(payload, (self.host, self.port))
            if written is not None and int(written) != len(payload):
                raise OSError("UDP short write %d/%d" % (
                    int(written), len(payload)))
        except Exception as error:
            self.error_count += 1
            self.last_error = str(error)
            # 保留socket，允许Wi-Fi短暂断线后下一帧自动恢复发送。
            return False
        self.last_send_ms = now_ms
        self.sequence = (self.sequence + 1) & 0xFFFF
        self.tx_count += 1
        self.last_error = ""
        return True

    def send_position(self, position, tracked_ball=None,
                      trough_axis=None, now_ms=None,
                      measurement_age_ms=0.0,
                      frame_timestamp_ms=0, frame_index=0,
                      f4_status=None):
        """把横向位置结果转换为VOFA通道并发送。"""
        position = position or {}
        tracked_ball = tracked_ball or {}
        trough_axis = trough_axis or {}
        valid = bool(
            position.get("valid") and
            position.get("position_cm") is not None)
        position_mm = (
            float(position.get("position_cm")) * 10.0 if valid else 0.0)
        ball_source = str(tracked_ball.get("source", "")).upper()
        predicted = ball_source == "PREDICTED"
        score = tracked_ball.get("score", -1.0)
        try:
            score = float(score)
        except (TypeError, ValueError):
            score = -1.0
        return self.send_sample(
            position_mm=position_mm,
            valid=valid,
            predicted=predicted,
            axis_source=axis_source_code(trough_axis.get("source")),
            score_pct=score * 100.0 if 0.0 <= score <= 1.0 else score,
            now_ms=now_ms,
            measurement_age_ms=measurement_age_ms,
            frame_timestamp_ms=frame_timestamp_ms,
            frame_index=frame_index,
            f4_status=f4_status,
        )

    def close(self):
        """幂等关闭socket。"""
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None
        self.available = False
