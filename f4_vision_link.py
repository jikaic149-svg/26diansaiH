"""MaixCAM2 与 F407 的独立 VIS/VST UART 链路。

板级资源固定为 B0/UART2_TX、B1/UART2_RX、/dev/ttyS2、115200 8N1。
本模块独占引脚、UART 生命周期、Packet V1 编解码和收发诊断；视觉算法只提交
已经计算好的位置结果，不依赖 ``maix`` UART API。
"""

import time as _python_time
import math


PACKET_VERSION = 1
PACKET_MAX_FRAME_SIZE = 128
PACKET_MAX_PAYLOAD_LENGTH = 111

UART_DEVICE = "/dev/ttyS2"
UART_BAUD = 115200
UART_TX_PIN = "B0"
UART_RX_PIN = "B1"
UART_TX_FUNCTION = "UART2_TX"
UART_RX_FUNCTION = "UART2_RX"

VIS_POSITION_MIN_MM = -125
VIS_POSITION_MAX_MM = 125

BALL_SOURCE_NONE = 0
BALL_SOURCE_YOLO_MEASURED = 1
BALL_SOURCE_PREDICTED = 2

AXIS_SOURCE_NONE = 0
AXIS_SOURCE_MEASURED = 1
AXIS_SOURCE_HELD = 2
AXIS_SOURCE_CACHED = 3

VIS_FLAG_POSITION_VALID = 0x10
VIS_FLAG_CALIBRATION_VALID = 0x20
VIS_FLAG_POSITION_CLAMPED = 0x40

PID_GAIN_SCALE = 1000.0
PID_KP_MAX_PER_S2 = 5.0
PID_KI_MAX_PER_S3 = 1.0
PID_KD_MAX_PER_S = 5.0
BALL_TARGET_MIN_MM = -120.0
BALL_TARGET_MAX_MM = 120.0
BALL_TARGET_SCALE_0P1MM = 10.0


def _elapsed_ms(now_ms, previous_ms):
    return (int(now_ms) - int(previous_ms)) & 0xFFFFFFFF


def _now_ms():
    monotonic = getattr(_python_time, "monotonic", None)
    if monotonic is not None:
        return int(monotonic() * 1000.0)
    return int(_python_time.time() * 1000.0)


def crc16_ccitt_false(data):
    """CRC-16/CCITT-FALSE：poly=0x1021、init=0xFFFF。"""
    crc = 0xFFFF
    for value in bytes(data):
        crc ^= int(value) << 8
        for _bit in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _type_is_valid(packet_type):
    if not isinstance(packet_type, str) or not 1 <= len(packet_type) <= 16:
        return False
    if not "A" <= packet_type[0] <= "Z":
        return False
    for char in packet_type[1:]:
        if not ("A" <= char <= "Z" or "0" <= char <= "9" or char == "_"):
            return False
    return True


def encode_packet(packet_type, sequence, payload):
    """编码 ``@1|TYPE|SEQ|LEN|PAYLOAD|CRC16\n``。"""
    if not _type_is_valid(packet_type):
        raise ValueError("invalid Packet type")
    sequence = int(sequence)
    if not 0 <= sequence <= 0xFFFF:
        raise ValueError("invalid Packet sequence")
    if isinstance(payload, str):
        payload_bytes = payload.encode("ascii")
    else:
        payload_bytes = bytes(payload)
    if len(payload_bytes) > PACKET_MAX_PAYLOAD_LENGTH or any(
            value < 0x20 or value > 0x7E for value in payload_bytes):
        raise ValueError("invalid Packet payload")

    prefix = "@1|%s|%d|%d|" % (
        packet_type, sequence, len(payload_bytes))
    covered = prefix.encode("ascii") + payload_bytes + b"|"
    frame = covered + ("%04X\n" % crc16_ccitt_false(covered)).encode("ascii")
    if len(frame) > PACKET_MAX_FRAME_SIZE:
        raise ValueError("Packet frame too long")
    return frame


def _parse_uint(text, maximum):
    if not text or (len(text) > 1 and text[0] == "0") or not text.isdigit():
        raise ValueError("non-canonical unsigned integer")
    value = int(text)
    if value > int(maximum):
        raise ValueError("unsigned integer out of range")
    return value


def _parse_int(text, minimum, maximum):
    if not text:
        raise ValueError("empty signed integer")
    if text[0] == "-":
        if len(text) == 1 or text[1] == "0" or not text[1:].isdigit():
            raise ValueError("non-canonical signed integer")
    elif (len(text) > 1 and text[0] == "0") or not text.isdigit():
        raise ValueError("non-canonical signed integer")
    value = int(text)
    if value < int(minimum) or value > int(maximum):
        raise ValueError("signed integer out of range")
    return value


def parse_packet(line):
    """严格解析一条不含 LF 的 Packet V1 物理行。"""
    data = bytes(line)
    if data.endswith(b"\r"):
        data = data[:-1]
    if not data or len(data) + 1 > PACKET_MAX_FRAME_SIZE:
        raise ValueError("invalid Packet line length")
    try:
        text = data.decode("ascii")
    except Exception as error:
        raise ValueError("Packet is not ASCII") from error
    parts = text.split("|")
    if len(parts) != 6 or parts[0] != "@1" or not _type_is_valid(parts[1]):
        raise ValueError("invalid Packet format")
    sequence = _parse_uint(parts[2], 0xFFFF)
    declared_length = _parse_uint(parts[3], PACKET_MAX_PAYLOAD_LENGTH)
    payload_bytes = parts[4].encode("ascii")
    if len(payload_bytes) != declared_length or any(
            value < 0x20 or value > 0x7E for value in payload_bytes):
        raise ValueError("invalid Packet payload length")
    if len(parts[5]) != 4 or any(
            char not in "0123456789ABCDEF" for char in parts[5]):
        raise ValueError("invalid Packet CRC field")
    covered = data[:-(len(parts[5]))]
    if crc16_ccitt_false(covered) != int(parts[5], 16):
        raise ValueError("Packet CRC mismatch")
    return parts[1], sequence, parts[4]


def encode_vis_frame(sequence, sample_ms, processing_age_ms,
                     position_mm, flags):
    sample_ms = int(sample_ms) & 0xFFFFFFFF
    processing_age_ms = max(0, min(0xFFFF, int(processing_age_ms)))
    position_mm = max(
        VIS_POSITION_MIN_MM, min(VIS_POSITION_MAX_MM, int(position_mm)))
    flags = int(flags)
    if not 0 <= flags <= 0x7F:
        raise ValueError("invalid VIS flags")
    payload = "%d,%d,%d,%d" % (
        sample_ms, processing_age_ms, position_mm, flags)
    return encode_packet("VIS", sequence, payload)


def decode_vst_packet(sequence, payload):
    """解析 cooperation 第11.3节定义的 VST Payload。"""
    fields = str(payload).split(",")
    if len(fields) != 9:
        raise ValueError("VST field count must be 9")
    status = {
        "sequence": int(sequence) & 0xFFFF,
        "last_rx_vis_seq": _parse_uint(fields[0], 0xFFFF),
        "last_used_vis_seq": _parse_uint(fields[1], 0xFFFF),
        "measurement_age_ms": _parse_uint(fields[2], 0xFFFF),
        "estimate_mm": _parse_int(fields[3], -32768, 32767),
        "velocity_mm_s": _parse_int(fields[4], -32768, 32767),
        "target_mm": _parse_int(fields[5], -125, 125),
        "theta_cmd_mdeg": _parse_int(fields[6], -85000, 85000),
        "status_flags": _parse_uint(fields[7], 0xFFFF),
        "fault_bits": _parse_uint(fields[8], 0xFFFF),
    }
    if status["status_flags"] & 0x4000 or status["fault_bits"] & 0x8000:
        raise ValueError("VST reserved bit is nonzero")
    return status


def decode_vpa_packet(sequence, payload):
    """解析F407对VTP实时调参请求的应用结果。"""
    fields = str(payload).split(",")
    if len(fields) != 5:
        raise ValueError("VPA field count must be 5")
    accepted = _parse_uint(fields[1], 1)
    return {
        "sequence": int(sequence) & 0xFFFF,
        "request_sequence": _parse_uint(fields[0], 0xFFFF),
        "accepted": bool(accepted),
        "kp_milli_per_s2": _parse_uint(fields[2], 0xFFFF),
        "ki_milli_per_s3": _parse_uint(fields[3], 0xFFFF),
        "kd_milli_per_s": _parse_uint(fields[4], 0xFFFF),
    }


def _round_half_away_from_zero(value):
    value = float(value)
    return int(value + 0.5) if value >= 0.0 else int(value - 0.5)


def build_vis_measurement(position, tracked_ball, trough_axis,
                          sample_ms, now_ms, calibration_valid):
    """把业务结果转换成协议字段；本函数可在 PC 上独立测试。"""
    position = position or {}
    tracked_ball = tracked_ball or {}
    trough_axis = trough_axis or {}

    ball_source = {
        "YOLO_RAW": BALL_SOURCE_YOLO_MEASURED,
        "MEASURED": BALL_SOURCE_YOLO_MEASURED,
        "PREDICTED": BALL_SOURCE_PREDICTED,
    }.get(str(tracked_ball.get("source", "")).upper(), BALL_SOURCE_NONE)
    axis_source = {
        "MEASURED": AXIS_SOURCE_MEASURED,
        "HELD": AXIS_SOURCE_HELD,
        "CACHED": AXIS_SOURCE_CACHED,
    }.get(str(trough_axis.get("source", "")).upper(), AXIS_SOURCE_NONE)
    flags = ball_source | (axis_source << 2)
    if bool(calibration_valid):
        flags |= VIS_FLAG_CALIBRATION_VALID

    valid = bool(position.get("valid") and
                 position.get("position_cm") is not None)
    raw_position_mm = 0
    if valid:
        try:
            raw_position_mm = _round_half_away_from_zero(
                float(position.get("position_cm")) * 10.0)
        except (TypeError, ValueError, OverflowError):
            valid = False
            raw_position_mm = 0
    if raw_position_mm < VIS_POSITION_MIN_MM or \
            raw_position_mm > VIS_POSITION_MAX_MM:
        flags |= VIS_FLAG_POSITION_CLAMPED
    position_mm = max(
        VIS_POSITION_MIN_MM, min(VIS_POSITION_MAX_MM, raw_position_mm))

    processing_age_ms = _elapsed_ms(now_ms, sample_ms)
    if processing_age_ms >= 0xFFFF:
        processing_age_ms = 0xFFFF
        valid = False
    if valid:
        flags |= VIS_FLAG_POSITION_VALID
    else:
        position_mm = 0
    return {
        "sample_ms": int(sample_ms) & 0xFFFFFFFF,
        "processing_age_ms": processing_age_ms,
        "position_mm": position_mm,
        "flags": flags & 0x7F,
    }


def _sequence_is_new(value, previous):
    difference = (int(value) - int(previous)) & 0xFFFF
    return 0 < difference < 0x8000


class F4VisionLink:
    """单所有者、无 ACK/重发的 MaixCAM2 UART 链路。"""

    def __init__(self, enabled=True, required=False, send_interval_ms=20,
                 tx_timeout_ms=100, rx_limit=256,
                 port_factory=None, clock_ms=None):
        self.enabled = bool(enabled)
        self.required = bool(required)
        self.send_interval_ms = max(20, int(send_interval_ms))
        self.tx_timeout_ms = max(1, int(tx_timeout_ms))
        self.rx_limit = max(1, int(rx_limit))
        self.port_factory = port_factory
        self.clock_ms = clock_ms
        self.port = None
        self.available = False
        self.sequence = 0
        self.pid_sequence = 0
        self.target_sequence = 0
        self.last_send_ms = None
        self.last_error = ""

        self._tx_frame = None
        self._tx_offset = 0
        self._tx_started_ms = 0
        self._tx_kind = None
        self._needs_resync = False
        self._rx_buffer = bytearray()
        self._rx_discarding = False
        self._last_vst_sequence = 0
        self._vst_sequence_seen = False
        self._last_vst_rx_ms = 0

        self.last_vst = None
        self.last_vpa = None
        self.tx_count = 0
        self.pid_tuning_tx_count = 0
        self.ball_target_tx_count = 0
        self.busy_drop_count = 0
        self.short_write_count = 0
        self.tx_timeout_count = 0
        self.tx_error_count = 0
        self.resync_count = 0
        self.rx_byte_count = 0
        self.vst_rx_count = 0
        self.vst_duplicate_count = 0
        self.vst_old_count = 0
        self.rx_rejected_count = 0
        self.rx_error_count = 0

    def _clock(self):
        return int(self.clock_ms()) if self.clock_ms is not None else _now_ms()

    def start(self):
        if not self.enabled:
            return False
        try:
            if self.port_factory is None:
                from maix import pinmap, uart
                for pin, function in (
                        (UART_TX_PIN, UART_TX_FUNCTION),
                        (UART_RX_PIN, UART_RX_FUNCTION)):
                    result = pinmap.set_pin_function(pin, function)
                    if result not in (None, 0):
                        raise RuntimeError(
                            "pin mapping failed %s -> %s: %s" %
                            (pin, function, str(result)))
                self.port = uart.UART(UART_DEVICE, UART_BAUD)
            else:
                self.port = self.port_factory(UART_DEVICE, UART_BAUD)
            self.available = self.port is not None
            if not self.available:
                raise RuntimeError("UART factory returned no port")
            self.last_error = ""
            print("F4_UART_OPEN|device=%s|baud=%d|tx=%s|rx=%s" % (
                UART_DEVICE, UART_BAUD, UART_TX_PIN, UART_RX_PIN))
            return True
        except Exception as error:
            self.port = None
            self.available = False
            self.last_error = str(error)
            print("F4_UART_OPEN_FAILED|error=%s" % self.last_error)
            if self.required:
                raise RuntimeError(
                    "required F4 UART open failed: %s" % self.last_error)
            return False

    def _finish_tx(self, now_ms):
        tx_kind = self._tx_kind
        if tx_kind == "VIS":
            sent_sequence = self.sequence
        elif tx_kind == "VTP":
            sent_sequence = self.pid_sequence
        else:
            sent_sequence = self.target_sequence
        sent_length = len(self._tx_frame)
        self._tx_frame = None
        self._tx_offset = 0
        self._tx_kind = None
        if tx_kind == "VIS":
            self.sequence = (self.sequence + 1) & 0xFFFF
            self.last_send_ms = int(now_ms)
            self.tx_count += 1
        elif tx_kind == "VTP":
            self.pid_sequence = (self.pid_sequence + 1) & 0xFFFF
            self.pid_tuning_tx_count += 1
        elif tx_kind == "VBT":
            self.target_sequence = (self.target_sequence + 1) & 0xFFFF
            self.ball_target_tx_count += 1
        self.last_error = ""
        if tx_kind == "VIS" and self.tx_count == 1:
            print("F4_VIS_TX_STARTED|sequence=%d|bytes=%d" % (
                sent_sequence, sent_length))
        elif tx_kind == "VTP":
            print("F4_VTP_TX|sequence=%d|bytes=%d" % (
                sent_sequence, sent_length))
        elif tx_kind == "VBT":
            print("F4_VBT_TX|sequence=%d|bytes=%d" % (
                sent_sequence, sent_length))

    def _drop_partial(self, needs_resync):
        self._tx_frame = None
        self._tx_offset = 0
        self._tx_kind = None
        if needs_resync:
            self._needs_resync = True

    def _write_pending(self, now_ms):
        if self._tx_frame is None:
            return False
        if self._tx_offset > 0 and _elapsed_ms(
                now_ms, self._tx_started_ms) >= self.tx_timeout_ms:
            self.tx_timeout_count += 1
            self._drop_partial(True)
            return False
        remaining = self._tx_frame[self._tx_offset:]
        try:
            written = self.port.write(remaining)
            if written is None:
                raise OSError("UART write returned no length")
            written = int(written)
            if written < 0 or written > len(remaining):
                raise OSError("invalid UART write length %d/%d" % (
                    written, len(remaining)))
        except Exception as error:
            self.tx_error_count += 1
            self.last_error = str(error)
            self._drop_partial(self._tx_offset > 0)
            return False

        if written == 0:
            if self._tx_offset == 0:
                self.busy_drop_count += 1
                self._drop_partial(False)
            return False
        self._tx_offset += written
        if self._tx_offset < len(self._tx_frame):
            self.short_write_count += 1
            return False
        self._finish_tx(now_ms)
        return True

    def _service_resync(self):
        if not self._needs_resync:
            return True
        try:
            written = self.port.write(b"\n")
            if written is not None and int(written) == 1:
                self._needs_resync = False
                self.resync_count += 1
                self.last_error = ""
                return True
            return False
        except Exception as error:
            self.tx_error_count += 1
            self.last_error = str(error)
            return False

    def _consume_rx(self, chunk, now_ms):
        if not chunk:
            return
        self.rx_byte_count += len(chunk)
        self._rx_buffer.extend(chunk)
        while True:
            newline = self._rx_buffer.find(b"\n")
            if newline < 0:
                break
            line = bytes(self._rx_buffer[:newline])
            del self._rx_buffer[:newline + 1]
            if self._rx_discarding:
                self._rx_discarding = False
                continue
            if not line:
                continue
            try:
                packet_type, sequence, payload = parse_packet(line)
                if packet_type == "VST":
                    status = decode_vst_packet(sequence, payload)
                elif packet_type == "VPA":
                    tuning = decode_vpa_packet(sequence, payload)
                    tuning["received_at_ms"] = int(now_ms) & 0xFFFFFFFF
                    self.last_vpa = tuning
                    print(
                        "F4_VPA|request_seq=%d|accepted=%d|"
                        "kp=%.3f|ki=%.3f|kd=%.3f" % (
                            tuning["request_sequence"],
                            1 if tuning["accepted"] else 0,
                            tuning["kp_milli_per_s2"] / PID_GAIN_SCALE,
                            tuning["ki_milli_per_s3"] / PID_GAIN_SCALE,
                            tuning["kd_milli_per_s"] / PID_GAIN_SCALE,
                        )
                    )
                    continue
                else:
                    continue
            except Exception:
                self.rx_rejected_count += 1
                continue

            if self._vst_sequence_seen and _elapsed_ms(
                    now_ms, self._last_vst_rx_ms) >= 100:
                self._vst_sequence_seen = False
            if self._vst_sequence_seen:
                difference = (sequence - self._last_vst_sequence) & 0xFFFF
                if difference == 0:
                    self.vst_duplicate_count += 1
                    continue
                if not _sequence_is_new(sequence, self._last_vst_sequence):
                    self.vst_old_count += 1
                    continue
            self._last_vst_sequence = sequence
            self._vst_sequence_seen = True
            self._last_vst_rx_ms = int(now_ms)
            status["received_at_ms"] = int(now_ms) & 0xFFFFFFFF
            self.last_vst = status
            self.vst_rx_count += 1
            if self.vst_rx_count == 1 or self.vst_rx_count % 25 == 0:
                print(
                    "F4_VST|rx=%d|last_rx_vis=%d|last_used_vis=%d|"
                    "age_ms=%d|estimate_mm=%d|velocity_mm_s=%d|"
                    "target_mm=%d|theta_mdeg=%d|status=0x%04X|"
                    "fault=0x%04X" % (
                        self.vst_rx_count,
                        status["last_rx_vis_seq"],
                        status["last_used_vis_seq"],
                        status["measurement_age_ms"],
                        status["estimate_mm"],
                        status["velocity_mm_s"],
                        status["target_mm"],
                        status["theta_cmd_mdeg"],
                        status["status_flags"],
                        status["fault_bits"],
                    )
                )

        if len(self._rx_buffer) >= PACKET_MAX_FRAME_SIZE:
            self._rx_buffer.clear()
            self._rx_discarding = True
            self.rx_rejected_count += 1

    def _poll_rx(self, now_ms):
        try:
            try:
                chunk = self.port.read(self.rx_limit)
            except TypeError:
                chunk = self.port.read()
            self._consume_rx(chunk, now_ms)
        except Exception as error:
            self.rx_error_count += 1
            self.last_error = str(error)

    def service(self, now_ms=None):
        """每次视觉主循环调用一次：推进短写事务并非阻塞接收 VST。"""
        if not self.enabled or not self.available or self.port is None:
            return False
        now_ms = self._clock() if now_ms is None else int(now_ms)
        if self._tx_frame is not None:
            self._write_pending(now_ms)
        if self._tx_frame is None and self._needs_resync:
            self._service_resync()
        self._poll_rx(now_ms)
        return True

    def send_position(self, position, tracked_ball=None, trough_axis=None,
                      sample_ms=0, now_ms=None, calibration_valid=False):
        """最多 50 Hz 提交 VIS；忙时丢当前业务帧，不排队、不重发。"""
        if not self.enabled or not self.available or self.port is None:
            return False
        now_ms = self._clock() if now_ms is None else int(now_ms)
        if self.last_send_ms is not None and _elapsed_ms(
                now_ms, self.last_send_ms) < self.send_interval_ms:
            return False
        if self._tx_frame is not None or self._needs_resync:
            self.busy_drop_count += 1
            return False

        measurement = build_vis_measurement(
            position, tracked_ball, trough_axis,
            sample_ms=sample_ms,
            now_ms=now_ms,
            calibration_valid=calibration_valid,
        )
        self._tx_frame = encode_vis_frame(
            self.sequence,
            measurement["sample_ms"],
            measurement["processing_age_ms"],
            measurement["position_mm"],
            measurement["flags"],
        )
        self._tx_kind = "VIS"
        self._tx_offset = 0
        self._tx_started_ms = int(now_ms)
        return self._write_pending(now_ms)

    def send_pid_gains(self, kp_per_s2, ki_per_s3, kd_per_s, now_ms=None):
        """发送一次VTP运行时参数；F407仍会执行产品边界二次校验。"""
        if not self.enabled or not self.available or self.port is None:
            return False
        gains = (
            float(kp_per_s2), float(ki_per_s3), float(kd_per_s))
        limits = (
            PID_KP_MAX_PER_S2, PID_KI_MAX_PER_S3, PID_KD_MAX_PER_S)
        if any(not math.isfinite(value) for value in gains) or any(
                value < 0.0 or value > limit
                for value, limit in zip(gains, limits)):
            raise ValueError("PID tuning gain is outside the safe range")
        if self._tx_frame is not None or self._needs_resync:
            self.busy_drop_count += 1
            return False

        now_ms = self._clock() if now_ms is None else int(now_ms)
        fixed = tuple(_round_half_away_from_zero(
            value * PID_GAIN_SCALE) for value in gains)
        payload = "%d,%d,%d" % fixed
        completed_before = self.pid_tuning_tx_count
        self._tx_frame = encode_packet("VTP", self.pid_sequence, payload)
        self._tx_kind = "VTP"
        self._tx_offset = 0
        self._tx_started_ms = int(now_ms)
        self._write_pending(now_ms)
        return (self._tx_frame is not None or
                self.pid_tuning_tx_count > completed_before)

    def send_ball_target(self, target_mm, now_ms=None):
        """Send one bounded VBT target in signed 0.1 mm fixed point."""
        if not self.enabled or not self.available or self.port is None:
            return False
        target_mm = float(target_mm)
        if not math.isfinite(target_mm) or \
                target_mm < BALL_TARGET_MIN_MM or \
                target_mm > BALL_TARGET_MAX_MM:
            raise ValueError("ball target is outside the safe range")
        if self._tx_frame is not None or self._needs_resync:
            self.busy_drop_count += 1
            return False

        now_ms = self._clock() if now_ms is None else int(now_ms)
        fixed = _round_half_away_from_zero(
            target_mm * BALL_TARGET_SCALE_0P1MM)
        completed_before = self.ball_target_tx_count
        self._tx_frame = encode_packet(
            "VBT", self.target_sequence, str(fixed))
        self._tx_kind = "VBT"
        self._tx_offset = 0
        self._tx_started_ms = int(now_ms)
        self._write_pending(now_ms)
        return (self._tx_frame is not None or
                self.ball_target_tx_count > completed_before)

    def close(self):
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
        self.port = None
        self.available = False
        self._tx_frame = None
        self._tx_kind = None
        self._rx_buffer.clear()
