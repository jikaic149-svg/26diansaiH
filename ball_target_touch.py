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


def question3_button_geometry(width, height):
    """Return a clearly separated top-right Q3 touch button."""
    width = max(1, int(width))
    height = max(1, int(height))
    button_width = min(108, max(1, width - 8))
    button_height = min(34, max(1, height - 8))
    return {
        "x": max(0, width - button_width - 4),
        "y": 4,
        "width": button_width,
        "height": button_height,
    }


def _point_in_rect(x, y, rectangle):
    return (
        int(x) >= rectangle["x"] and
        int(x) < rectangle["x"] + rectangle["width"] and
        int(y) >= rectangle["y"] and
        int(y) < rectangle["y"] + rectangle["height"])


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


def target_axis_t_from_mm(target_mm, minimum_mm, maximum_mm,
                          calibration_pairs=None):
    """Map a physical target to the same axis coordinate used by vision."""
    pairs = calibration_pairs or ()
    if len(pairs) >= 2:
        ordered = sorted(
            (float(item[1]) * 10.0, float(item[0])) for item in pairs)
        target = float(target_mm)
        if target <= ordered[0][0]:
            left, right = ordered[0], ordered[1]
        elif target >= ordered[-1][0]:
            left, right = ordered[-2], ordered[-1]
        else:
            left, right = ordered[0], ordered[1]
            for candidate_left, candidate_right in zip(
                    ordered, ordered[1:]):
                if candidate_left[0] <= target <= candidate_right[0]:
                    left, right = candidate_left, candidate_right
                    break
        span = right[0] - left[0]
        if abs(span) > 1e-9:
            ratio = (target - left[0]) / span
            return _clamp(
                left[1] + ratio * (right[1] - left[1]), 0.0, 1.0)

    minimum_mm = float(minimum_mm)
    maximum_mm = float(maximum_mm)
    if maximum_mm <= minimum_mm:
        return None
    return (
        _clamp(float(target_mm), minimum_mm, maximum_mm) - minimum_mm
    ) / (maximum_mm - minimum_mm)


def axis_point_from_t(trough_axis, target_t):
    """Return the arc-length point on the displayed trough centerline."""
    if not trough_axis or not trough_axis.get("valid") or target_t is None:
        return None
    points = trough_axis.get("centerline") or []
    if len(points) < 2:
        p0 = trough_axis.get("p0")
        p1 = trough_axis.get("p1")
        points = [p0, p1] if p0 is not None and p1 is not None else []
    segments = []
    total_length = 0.0
    for start, end in zip(points, points[1:]):
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        length = (dx * dx + dy * dy) ** 0.5
        if length <= 1e-9:
            continue
        segments.append((start, dx, dy, length, total_length))
        total_length += length
    if not segments or total_length <= 1e-9:
        return None
    distance = _clamp(float(target_t), 0.0, 1.0) * total_length
    for index, (start, dx, dy, length, cumulative) in enumerate(segments):
        if distance <= cumulative + length or index == len(segments) - 1:
            ratio = _clamp((distance - cumulative) / length, 0.0, 1.0)
            return (
                float(start[0]) + ratio * dx,
                float(start[1]) + ratio * dy,
            )
    return None


def _elapsed_ms(now_ms, previous_ms):
    return (int(now_ms) - int(previous_ms)) & 0xFFFFFFFF


class BallTargetTouchController:
    """Touch selection, VST confirmation, and bounded VBT retry owner."""

    def __init__(self, enabled=True, minimum_mm=-120.0,
                 maximum_mm=120.0, step_mm=1.0, retry_ms=250,
                 max_attempts=20, offline_ms=500,
                 target_to_f4_polarity=1.0, touch_factory=None,
                 question3_enabled=True, question3_button_enabled=True,
                 question3_center_mm=0.0,
                 question3_center_tolerance_mm=10.0,
                 question3_right_command_mm=50.0,
                 question3_right_reached_mm=50.0,
                 question3_left_command_mm=-50.0,
                 question3_left_reached_mm=-50.0,
                 question3_final_target_mm=-50.0,
                 question3_limit_ms=5000,
                 question3_hard_timeout_ms=8000):
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
        self.question3_enabled = bool(question3_enabled)
        self.question3_button_enabled = bool(question3_button_enabled)
        self.target_edit_enabled = True
        self.question3_center_mm = float(question3_center_mm)
        self.question3_center_tolerance_mm = max(
            0.0, float(question3_center_tolerance_mm))
        self.question3_right_command_mm = float(
            question3_right_command_mm)
        self.question3_right_reached_mm = float(
            question3_right_reached_mm)
        self.question3_left_command_mm = float(
            question3_left_command_mm)
        self.question3_left_reached_mm = float(
            question3_left_reached_mm)
        self.question3_final_target_mm = float(
            question3_final_target_mm)
        self.question3_limit_ms = max(1, int(question3_limit_ms))
        self.question3_hard_timeout_ms = max(
            self.question3_limit_ms, int(question3_hard_timeout_ms))
        self.question3_state = "IDLE"
        self.question3_button_armed = False
        self.question3_start_ms = None
        self.question3_elapsed_ms = None
        self.question3_right_position_mm = None
        self.question3_left_position_mm = None
        self.question3_last_position_mm = None
        self.question3_passed = None

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
                "BALL_TARGET_TOUCH_READY|min_mm=%.0f|max_mm=%.0f|"
                "step_mm=%.0f|q3=%d"
                % (self.minimum_mm, self.maximum_mm, self.step_mm,
                   1 if self.question3_enabled else 0))
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

    def _select_target_mm(self, target_mm, source):
        self.selected_target_mm = quantize_target_mm(
            target_mm, self.minimum_mm, self.maximum_mm, self.step_mm)
        self.user_selected = True
        self.initial_sync_done = True
        self.pending = True
        self.last_attempt_ms = None
        self.pending_attempt_count = 0
        self.retry_exhausted_logged = False
        print("BALL_TARGET_SELECTED|target_mm=%.0f|source=%s" % (
            self.selected_target_mm, str(source)))

    def select_target_mm(self, target_mm, source="TASK"):
        """Public task-owned target selection using the normal VBT path."""
        self._select_target_mm(target_mm, source)
        return True

    def set_target_edit_enabled(self, enabled):
        self.target_edit_enabled = bool(enabled)
        if not self.target_edit_enabled:
            self.dragging = False

    def cancel_question3(self, reason="CANCELLED"):
        """Stop local Q3 progression without starting or stopping F407."""
        was_active = self._question3_active()
        self.question3_state = "IDLE"
        self.question3_button_armed = False
        self.question3_start_ms = None
        self.question3_elapsed_ms = None
        self.question3_passed = None
        if was_active:
            print("Q3_CANCELLED|reason=%s" % str(reason))
        return was_active

    def _question3_active(self):
        return self.question3_state in ("CENTERING", "RIGHT", "LEFT")

    def _question3_log(self, event, now_ms, position_mm, vst):
        elapsed_ms = -1
        if self.question3_start_ms is not None:
            elapsed_ms = _elapsed_ms(now_ms, self.question3_start_ms)
        vst = vst or {}
        print(
            "%s|state=%s|elapsed_ms=%d|visual_position_mm=%.1f|"
            "target_raw=%d|feedback_raw=%d|speed_rpm=%.2f|fault=0x%04X"
            % (
                str(event), self.question3_state, elapsed_ms,
                float(position_mm),
                int(vst.get("motor_target_raw", 0)),
                int(vst.get("motor_feedback_raw", 0)),
                float(vst.get("motor_speed_centi_rpm", 0)) / 100.0,
                int(vst.get("fault_bits", 0xFFFF)) & 0xFFFF,
            ))

    def start_question3(self):
        """Arm the on-device Q3 sequence; timing begins after centering."""
        if not self.question3_enabled or self._question3_active():
            return False
        self.question3_state = "CENTERING"
        self.question3_start_ms = None
        self.question3_elapsed_ms = None
        self.question3_right_position_mm = None
        self.question3_left_position_mm = None
        self.question3_last_position_mm = None
        self.question3_passed = None
        self._select_target_mm(self.question3_center_mm, "Q3_CENTER")
        print("Q3_UI_START|state=CENTERING")
        return True

    def update_question3(self, position, vst, now_ms):
        """Advance Q3 from current real visual data and fresh fault-free VST."""
        if not self._question3_active():
            return False
        position = position or {}
        vst = vst or {}
        try:
            visual_valid = bool(
                position.get("valid") and
                position.get("position_cm") is not None)
            position_mm = float(position.get("position_cm")) * 10.0
        except (TypeError, ValueError, OverflowError):
            visual_valid = False
            position_mm = 0.0
        if visual_valid:
            self.question3_last_position_mm = position_mm

        try:
            vst_fresh = (
                _elapsed_ms(now_ms, int(vst["received_at_ms"])) <=
                self.offline_ms)
            fault_free = int(vst["fault_bits"]) == 0
        except (KeyError, TypeError, ValueError, OverflowError):
            vst_fresh = False
            fault_free = False

        if self.question3_start_ms is not None and _elapsed_ms(
                now_ms, self.question3_start_ms) > \
                self.question3_hard_timeout_ms:
            timeout_position_mm = self.question3_last_position_mm
            if timeout_position_mm is None:
                timeout_position_mm = self.question3_center_mm
            self.question3_elapsed_ms = _elapsed_ms(
                now_ms, self.question3_start_ms)
            self.question3_state = "FAILED"
            self.question3_passed = False
            # Question 3 owns exact visual targets. A timeout reports failure
            # but must not replace -50 mm with an arbitrary measured position.
            self._select_target_mm(
                self.question3_final_target_mm, "Q3_TIMEOUT_FINAL")
            self._question3_log(
                "Q3_UI_RESULT", now_ms, timeout_position_mm, vst)
            return True

        if not visual_valid or not vst_fresh or not fault_free:
            return False

        if self.question3_state == "CENTERING" and abs(
                position_mm - self.question3_center_mm) <= \
                self.question3_center_tolerance_mm:
            self.question3_start_ms = int(now_ms)
            self.question3_state = "RIGHT"
            self._question3_log(
                "Q3_CENTER_READY", now_ms, position_mm, vst)
            self._select_target_mm(
                self.question3_right_command_mm, "Q3_RIGHT")
            return True

        if self.question3_state == "RIGHT" and position_mm >= \
                self.question3_right_reached_mm:
            self.question3_right_position_mm = position_mm
            self.question3_state = "LEFT"
            self._question3_log(
                "Q3_RIGHT_REACHED", now_ms, position_mm, vst)
            self._select_target_mm(
                self.question3_left_command_mm, "Q3_LEFT")
            return True

        if self.question3_state == "LEFT" and position_mm <= \
                self.question3_left_reached_mm:
            self.question3_left_position_mm = position_mm
            self.question3_elapsed_ms = _elapsed_ms(
                now_ms, self.question3_start_ms)
            self.question3_passed = (
                self.question3_elapsed_ms <= self.question3_limit_ms)
            self.question3_state = "DONE"
            self._select_target_mm(
                self.question3_final_target_mm, "Q3_FINAL")
            self._question3_log(
                "Q3_UI_RESULT", now_ms, position_mm, vst)
            return True
        return False

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
        button = question3_button_geometry(
            frame.width(), frame.height())
        if pressed and not self.was_pressed:
            self.question3_button_armed = (
                self.question3_enabled and
                self.question3_button_enabled and
                _point_in_rect(touch_x, touch_y, button))
            self.dragging = (
                not self.question3_button_armed and
                not self._question3_active() and
                self.target_edit_enabled and
                geometry["touch_top"] <= touch_y <
                geometry["touch_bottom"])
        if pressed and self.dragging:
            self.selected_target_mm = slider_target_from_x(
                touch_x, geometry["left"], geometry["right"],
                self.minimum_mm, self.maximum_mm, self.step_mm)

        committed = False
        if not pressed and self.was_pressed and \
                self.question3_button_armed:
            self.question3_button_armed = False
            committed = self.start_question3()
        if not pressed and self.was_pressed and self.dragging:
            self.dragging = False
            self.question3_state = "IDLE"
            self.question3_passed = None
            self._select_target_mm(
                self.selected_target_mm, "TOUCH_SLIDER")
            committed = True
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

    def draw(self, frame, now_ms, trough_axis=None,
             calibration_pairs=None):
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
        # The slider knob is a point, not a vertical line: a vertical marker
        # looked like a second coordinate axis and hid the real mapping.
        frame.draw_circle(
            marker_x, geometry["track_y"], 5,
            color=cyan, thickness=3)

        # Draw the commanded millimetre target on the calibrated trough axis.
        # This uses the inverse of the same piecewise t->cm table used by the
        # visual measurement, so the overlay and the sent target correspond.
        target_t = target_axis_t_from_mm(
            self.selected_target_mm,
            self.minimum_mm, self.maximum_mm,
            calibration_pairs=calibration_pairs)
        target_point = axis_point_from_t(trough_axis, target_t)
        if target_point is not None:
            frame.draw_circle(
                int(round(target_point[0])), int(round(target_point[1])), 7,
                color=cyan, thickness=3)

        if self.question3_enabled and self.question3_button_enabled:
            button = question3_button_geometry(
                frame.width(), frame.height())
            active = self._question3_active()
            button_color = yellow if active else (
                green if self.question3_passed is not False else red)
            frame.draw_rect(
                button["x"], button["y"],
                button["width"], button["height"],
                color=button_color, thickness=2)
            button_label = "Q3 RUN" if active else "Q3 START"
            frame.draw_string(
                button["x"] + 7, button["y"] + 8,
                button_label, color=button_color, scale=0.66)
            if self.question3_state != "IDLE":
                elapsed_ms = self.question3_elapsed_ms
                if elapsed_ms is None and self.question3_start_ms is not None:
                    elapsed_ms = _elapsed_ms(
                        now_ms, self.question3_start_ms)
                elapsed_text = "WAIT" if elapsed_ms is None else (
                    "%.3fs" % (float(elapsed_ms) / 1000.0))
                result_text = self.question3_state
                if self.question3_state == "DONE":
                    result_text = (
                        "PASS" if self.question3_passed else "OVER 5S")
                frame.draw_string(
                    button["x"], button["y"] + button["height"] + 2,
                    "Q3:%s %s" % (result_text, elapsed_text),
                    color=button_color, scale=0.56)

    def close(self):
        self.touch = None
        self.available = False
