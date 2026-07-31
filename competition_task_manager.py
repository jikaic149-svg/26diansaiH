"""Competition task selection driven by F407-forwarded G3519 mode."""


VALID_MODES = (2, 3, 4, 5, 6)
STATUS_CONTROL_ARMED = 0x0040
STATUS_BALL_OUTER_ACTIVE = 0x0080
STATUS_MOTOR_ENABLED = 0x0400
STATUS_COMPETITION_TASK_ACTIVE = 0x4000


def _elapsed_ms(now_ms, previous_ms):
    return (int(now_ms) - int(previous_ms)) & 0xFFFFFFFF


class CompetitionTaskManager:
    """Own M2..M6 entry, exit and per-frame visual task updates."""

    def __init__(self, target_controller, offline_ms=500,
                 center_target_mm=0.0):
        if target_controller is None:
            raise ValueError("target_controller is required")
        self.target_controller = target_controller
        self.offline_ms = max(1, int(offline_ms))
        self.center_target_mm = float(center_target_mm)
        self.active_mode = 0
        self.state = "NO_MODE"
        self.change_count = 0
        self.vst_fresh = False
        self.target_controller.set_target_edit_enabled(False)

    def _fresh_mode(self, vst, now_ms):
        vst = vst or {}
        try:
            if _elapsed_ms(now_ms, int(vst["received_at_ms"])) > \
                    self.offline_ms:
                return 0
            mode = int(vst.get("competition_mode", 0))
        except (KeyError, TypeError, ValueError, OverflowError):
            return 0
        return mode if mode in VALID_MODES else 0

    @staticmethod
    def _ball_loop_active(vst):
        vst = vst or {}
        try:
            flags = int(vst.get("status_flags", 0)) & 0xFFFF
            faults = int(vst.get("fault_bits", 0xFFFF)) & 0xFFFF
        except (TypeError, ValueError, OverflowError):
            return False
        required = (STATUS_CONTROL_ARMED | STATUS_BALL_OUTER_ACTIVE |
                    STATUS_MOTOR_ENABLED)
        return faults == 0 and (flags & required) == required

    @staticmethod
    def _control_armed(vst):
        try:
            return bool(int((vst or {}).get("status_flags", 0)) &
                        STATUS_CONTROL_ARMED)
        except (TypeError, ValueError, OverflowError):
            return False

    @staticmethod
    def _competition_task_active(vst):
        try:
            return bool(int((vst or {}).get("status_flags", 0)) &
                        STATUS_COMPETITION_TASK_ACTIVE)
        except (TypeError, ValueError, OverflowError):
            return False

    def _enter_mode(self, mode):
        previous_mode = self.active_mode
        if previous_mode == 3:
            self.target_controller.cancel_question3("MODE_EXIT")
        self.active_mode = int(mode)
        self.change_count += 1

        if self.active_mode == 0:
            self.target_controller.set_target_edit_enabled(False)
            self.state = "NO_MODE"
        elif self.active_mode == 3:
            self.target_controller.set_target_edit_enabled(False)
            self.state = "WAIT_START"
        elif self.active_mode == 6:
            self.target_controller.set_target_edit_enabled(True)
            self.state = "HOLD_SELECTED"
        elif self.active_mode == 2:
            self.target_controller.set_target_edit_enabled(False)
            self.state = "MONITOR"
        else:
            self.target_controller.set_target_edit_enabled(False)
            self.target_controller.select_target_mm(
                self.center_target_mm, "M%d_CENTER" % self.active_mode)
            self.state = "HOLD_CENTER"
        print("COMPETITION_TASK_SWITCH|from=%d|to=%d|state=%s" % (
            previous_mode, self.active_mode, self.state))

    def update(self, position, vst, now_ms):
        """Apply one fresh mode snapshot and advance its visual task."""
        mode = self._fresh_mode(vst, now_ms)
        self.vst_fresh = mode in VALID_MODES
        if not self.vst_fresh:
            self.target_controller.set_target_edit_enabled(False)
            return False

        if mode != self.active_mode:
            self._enter_mode(mode)
        else:
            self.target_controller.set_target_edit_enabled(mode == 6)

        if self.active_mode != 3:
            return False
        task_active = self._competition_task_active(vst)
        if not task_active:
            changed = self.state != "WAIT_START"
            if changed:
                self.target_controller.cancel_question3("G3519_KEY_STOP")
            self.state = "WAIT_START"
            return changed
        if self.state in ("WAIT_START", "WAIT_BALL_LOOP"):
            if not self._ball_loop_active(vst):
                self.state = "WAIT_BALL_LOOP"
                return False
            if self.target_controller.start_question3():
                self.state = "RUNNING"
                print("COMPETITION_TASK_START|mode=3|state=RUNNING")
                return True
            return False
        if self.state != "RUNNING":
            return False

        if not self._control_armed(vst):
            self.target_controller.cancel_question3("CONTROL_DISARMED")
            self.target_controller.select_target_mm(
                self.center_target_mm, "M3_RESTART_CENTER")
            self.state = "WAIT_BALL_LOOP"
            return True

        changed = self.target_controller.update_question3(
            position, vst, now_ms)
        q3_state = self.target_controller.question3_state
        if q3_state == "DONE":
            self.state = "DONE"
        elif q3_state == "FAILED":
            self.state = "FAILED"
        return changed

    def status_text(self):
        suffix = "" if self.vst_fresh else ":OFFLINE"
        return ("M%d:%s%s" % (self.active_mode, self.state, suffix)) \
            if self.active_mode else "M-:NO_MODE"
