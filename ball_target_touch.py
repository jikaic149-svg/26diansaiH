"""MaixCAM2 touch slider for the F407 ball-position target.

The slider only selects a position in millimetres.  The F407 remains the
owner of the ball-and-beam model, motor command, and chassis acceleration
feed-forward calculation.
"""


def _clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def quantize_target_mm(value, minimum_mm, maximum_mm, step_mm):
    """Clamp and quantize one target without depending on MaixPy."""
    minimum_mm = float(minimum_mm)
    maximum_mm = float(maximum_mm)
    step_mm = float(step_mm)
    if maximum_mm <= minimum_mm or step_mm <= 0.0:
        raise ValueError("invalid ball target slider range")
    value = _clamp(float(value), minimum_mm, maximum_mm)
    steps = int((value - minimum_mm) / step_mm + 0.5)
    quantized = minimum_mm + steps * step_mm
    return _clamp(quantized, minimum_mm, maximum_mm)


def slider_geometry(width, height):
    """Return the compact bottom-of-frame slider geometry."""
    width = max(1, int(width))
    height = max(1, int(height))
    left = min(12, max(0, width - 1))
    right = max(left + 1, width - 12)
    right = min(max(left + 1, right), width)
    track_y = max(0, height - 18)
    touch_top = max(0, height - 58)
    return {
        "left": left,
        "right": right,
        "track_y": track_y,
        "touch_top": touch_top,
        "touch_bottom": height,
    }


def slider_target_from_x(
        x, left, right, minimum_mm, maximum_mm, step_mm):
    """Map a frame x coordinate to a bounded, quantized millimetre target."""
    left = float(left)
    right = float(right)
    if right <= left:
        raise ValueError("slider right edge must exceed left edge")
    ratio = (_clamp(float(x), left, right) - left) / (right - left)
    raw = float(minimum_mm) + ratio * (
        float(maximum_mm) - float(minimum_mm))
    return quantize_target_mm(raw, minimum_mm, maximum_mm, step_mm)


def slider_x_from_target(target_mm, left, right, minimum_mm, maximum_mm):
    """Map a bounded millimetre target back to a frame x coordinate."""
    left = float(left)
    right = float(right)
    minimum_mm = float(minimum_mm)
    maximum_mm = float(maximum_mm)
    if right <= left or maximum_mm <= minimum_mm:
        raise ValueError("invalid slider geometry")
    value = _clamp(float(target_mm), minimum_mm, maximum_mm)
    ratio = (value - minimum_mm) / (maximum_mm - minimum_mm)
    return int(round(left + ratio * (right - left)))


def _elapsed_ms(now_ms, previous_ms):
    return (int(now_ms) - int(previous_ms)) & 0xFFFFFFFF


class BallTargetTouchController:
    """Touch selection, VST confirmation, and bounded VBT retry owner."""

    def __init__(self, enabled=True, minimum_mm=-120.0,
                 maximum_mm=120.0, step_mm=1.0, retry_ms=250,
                 max_attempts=20, offline_ms=500,
                 target_to_f4_polarity=1.0, touch_factory=None):
        self.enabled = bool(enabled)
        self.minimum_mm = float(minimum_mm)
        self.maximum_mm = float(maximum_mm)
        self.step_mm = float(step_mm)
        self.retry_ms = max(1, int(retry_ms))
        self.max_attempts = max(1, int(max_attempts))
        self.offline_ms = max(1, int(offline_ms))
        self.target_to_f4_polarity = float(target_to_f4_polarity)
        if self.target_to_f4_polarity not in (-1.0, 1.0):
            raise ValueError("target_to_f4_polarity must be -1 or +1")
        self.touch_factory = touch_factory
        self.touch = None
        self.available = False
        self.selected_target_mm = quantize_target_mm(
            0.0, self.minimum_mm, self.maximum_mm, self.step_mm)
        self.reported_target_mm = None
        self.reported_at_ms = None
        self.last_feedback_key = None
        self.user_selected = False
        self.initial_sync_done = False
        self.pending = False
        self.dragging = False
        self.was_pressed = False
        self.last_attempt_ms = None
        self.pending_attempt_count = 0
        self.retry_exhausted_logged = False
        self.tx_count = 0
        self.last_error = ""

    def start(self):
        if not self.enabled:
            return False
        try:
            if self.touch_factory is None:
                from maix import touchscreen
                self.touch = touchscreen.TouchScreen()
            else:
                self.touch = self.touch_factory()
            self.available = self.touch is not None
            if not self.available:
                raise RuntimeError("touchscreen factory returned no device")
            print(
                "BALL_TARGET_TOUCH_READY|min_mm=%.0f|max_mm=%.0f|step_mm=%.0f"
                % (self.minimum_mm, self.maximum_mm, self.step_mm))
            return True
        except Exception as error:
            self.touch = None
            self.available = False
            self.last_error = str(error)
            print("BALL_TARGET_TOUCH_UNAVAILABLE|error=%s" % self.last_error)
            return False

    @staticmethod
    def _map_touch_to_frame(raw_x, raw_y, frame, disp):
        try:
            from maix import image
            mapped = image.resize_map_pos_reverse(
                frame.width(), frame.height(),
                disp.width(), disp.height(), image.Fit.FIT_CONTAIN,
                int(raw_x), int(raw_y))
            return int(mapped[0]), int(mapped[1])
        except Exception:
            return (
                int(float(raw_x) * frame.width() /
                    max(1, int(disp.width()))),
                int(float(raw_y) * frame.height() /
                    max(1, int(disp.height()))),
            )

    def update_feedback(self, vst):
        if vst is None:
            return
        try:
            reported = (float(vst["target_mm"]) *
                        self.target_to_f4_polarity)
            received_at_ms = int(vst["received_at_ms"])
        except (KeyError, TypeError, ValueError, OverflowError):
            return
        feedback_key = (int(vst.get("sequence", 0)), received_at_ms)
        if feedback_key == self.last_feedback_key:
            return
        self.last_feedback_key = feedback_key
        self.reported_target_mm = reported
        self.reported_at_ms = received_at_ms
        if not self.initial_sync_done and not self.user_selected:
            self.selected_target_mm = reported
            self.initial_sync_done = True
        if self.pending and reported == self.selected_target_mm:
            self.pending = False
            print("BALL_TARGET_CONFIRMED|target_mm=%.0f" % reported)

    def poll_touch(self, frame, disp):
        """Update while dragging and queue one command on release."""
        if not self.enabled or not self.available or self.touch is None or \
                frame is None or disp is None:
            return False
        pressed = False
        touch_x = -1
        touch_y = -1
        try:
            raw_x, raw_y, raw_pressed = self.touch.read()
            pressed = bool(raw_pressed)
            if pressed:
                touch_x, touch_y = self._map_touch_to_frame(
                    raw_x, raw_y, frame, disp)
        except Exception as error:
            self.last_error = str(error)
            print("BALL_TARGET_TOUCH_ERROR|%s" % self.last_error)
            return False

        geometry = slider_geometry(frame.width(), frame.height())
        if pressed and not self.was_pressed:
            self.dragging = (
                geometry["touch_top"] <= touch_y <
                geometry["touch_bottom"])
        if pressed and self.dragging:
            self.selected_target_mm = slider_target_from_x(
                touch_x, geometry["left"], geometry["right"],
                self.minimum_mm, self.maximum_mm, self.step_mm)

        committed = False
        if not pressed and self.was_pressed and self.dragging:
            self.dragging = False
            self.user_selected = True
            self.initial_sync_done = True
            self.pending = True
            self.last_attempt_ms = None
            self.pending_attempt_count = 0
            self.retry_exhausted_logged = False
            committed = True
            print("BALL_TARGET_SELECTED|target_mm=%.0f" %
                  self.selected_target_mm)
        self.was_pressed = pressed
        return committed

    def try_send(self, link, now_ms):
        """Attempt VBT before VIS; retry only while awaiting VST match."""
        if not self.pending or link is None:
            return False
        if self.pending_attempt_count >= self.max_attempts:
            if not self.retry_exhausted_logged:
                print(
                    "BALL_TARGET_RETRY_EXHAUSTED|target_mm=%.0f|attempts=%d"
                    % (self.selected_target_mm,
                       self.pending_attempt_count))
                self.retry_exhausted_logged = True
            return False
        if self.last_attempt_ms is not None and _elapsed_ms(
                now_ms, self.last_attempt_ms) < self.retry_ms:
            return False
        try:
            f4_target_mm = (self.selected_target_mm *
                            self.target_to_f4_polarity)
            queued = bool(link.send_ball_target(
                f4_target_mm, now_ms=now_ms))
        except Exception as error:
            self.last_error = str(error)
            print("BALL_TARGET_TX_ERROR|%s" % self.last_error)
            return False
        if queued:
            self.last_attempt_ms = int(now_ms)
            self.pending_attempt_count += 1
            self.tx_count += 1
        return queued

    def link_status(self, now_ms):
        if self.reported_at_ms is None or _elapsed_ms(
                now_ms, self.reported_at_ms) > self.offline_ms:
            return "OFFLINE"
        return "PENDING" if self.pending else "OK"

    def draw(self, frame, now_ms):
        if not self.enabled or frame is None:
            return
        try:
            from maix import image
            white = image.Color.from_rgb(255, 255, 255)
            green = image.Color.from_rgb(0, 255, 0)
            yellow = image.Color.from_rgb(255, 220, 0)
            red = image.Color.from_rgb(255, 60, 60)
            cyan = image.Color.from_rgb(0, 230, 230)
        except Exception:
            return
        geometry = slider_geometry(frame.width(), frame.height())
        marker_x = slider_x_from_target(
            self.selected_target_mm,
            geometry["left"], geometry["right"],
            self.minimum_mm, self.maximum_mm)
        status = self.link_status(now_ms)
        status_color = green if status == "OK" else (
            yellow if status == "PENDING" else red)
        reported = "---" if self.reported_target_mm is None else "%+.0f" % (
            self.reported_target_mm)
        label_y = max(0, geometry["touch_top"] - 1)
        frame.draw_string(
            4, label_y,
            "SET:%+.0fmm F4:%smm %s" % (
                self.selected_target_mm, reported, status),
            color=status_color, scale=0.58)
        frame.draw_line(
            geometry["left"], geometry["track_y"],
            geometry["right"], geometry["track_y"],
            color=white, thickness=2)
        center_x = slider_x_from_target(
            0.0, geometry["left"], geometry["right"],
            self.minimum_mm, self.maximum_mm)
        frame.draw_line(
            center_x, geometry["track_y"] - 5,
            center_x, geometry["track_y"] + 5,
            color=white, thickness=1)
        frame.draw_line(
            marker_x, geometry["track_y"] - 9,
            marker_x, geometry["track_y"] + 9,
            color=cyan, thickness=3)

    def close(self):
        self.touch = None
        self.available = False
