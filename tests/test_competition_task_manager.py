"""PC-side tests for G3519-mode-driven visual competition tasks."""

import unittest

from ball_target_touch import BallTargetTouchController
from competition_task_manager import CompetitionTaskManager


ACTIVE_FLAGS = 0x0040 | 0x0080 | 0x0400
TASK_ACTIVE = 0x4000


class CompetitionTaskManagerTest(unittest.TestCase):

    @staticmethod
    def _vst(mode, now_ms, flags=0, faults=0):
        return {
            "competition_mode": mode,
            "received_at_ms": now_ms,
            "status_flags": flags,
            "fault_bits": faults,
            "motor_target_raw": 1000,
            "motor_feedback_raw": 1000,
            "motor_speed_centi_rpm": 0,
        }

    @staticmethod
    def _position(mm, valid=True):
        return {"valid": valid, "position_cm": float(mm) / 10.0}

    def test_center_tasks_lock_target_and_mode6_keeps_touch_target(self):
        target = BallTargetTouchController()
        manager = CompetitionTaskManager(target)

        manager.update(self._position(0), self._vst(4, 10), 10)
        self.assertEqual(manager.state, "HOLD_CENTER")
        self.assertEqual(target.selected_target_mm, 0)
        self.assertFalse(target.target_edit_enabled)
        changes = manager.change_count
        manager.update(self._position(1), self._vst(4, 20), 20)
        self.assertEqual(manager.change_count, changes)

        target.select_target_mm(37, "TEST_PRESET")
        manager.update(self._position(1), self._vst(6, 30), 30)
        self.assertEqual(manager.state, "HOLD_SELECTED")
        self.assertEqual(target.selected_target_mm, 37)
        self.assertTrue(target.target_edit_enabled)

        manager.update(self._position(1), self._vst(2, 40), 40)
        self.assertEqual(manager.state, "MONITOR")
        self.assertEqual(target.selected_target_mm, 37)
        self.assertFalse(target.target_edit_enabled)

    def test_question3_waits_for_real_ball_loop_then_runs(self):
        target = BallTargetTouchController()
        manager = CompetitionTaskManager(target)

        manager.update(self._position(0), self._vst(3, 10), 10)
        self.assertEqual(manager.state, "WAIT_START")
        self.assertEqual(target.question3_state, "IDLE")

        manager.update(
            self._position(0),
            self._vst(3, 20, ACTIVE_FLAGS | TASK_ACTIVE), 20)
        self.assertEqual(manager.state, "RUNNING")
        self.assertEqual(target.question3_state, "CENTERING")

        manager.update(
            self._position(0),
            self._vst(3, 30, ACTIVE_FLAGS | TASK_ACTIVE), 30)
        self.assertEqual(target.question3_state, "RIGHT")
        self.assertEqual(target.selected_target_mm, 50)

        manager.update(
            self._position(56),
            self._vst(3, 1000, ACTIVE_FLAGS | TASK_ACTIVE), 1000)
        self.assertEqual(target.question3_state, "LEFT")
        manager.update(
            self._position(-52),
            self._vst(3, 3000, ACTIVE_FLAGS | TASK_ACTIVE), 3000)
        self.assertEqual(manager.state, "DONE")
        self.assertEqual(target.selected_target_mm, -50)

    def test_g3519_key_state_starts_stops_and_restarts_mode3(self):
        target = BallTargetTouchController()
        manager = CompetitionTaskManager(target)
        target.select_target_mm(25, "PRESET")

        manager.update(
            self._position(0), self._vst(3, 10, ACTIVE_FLAGS), 10)
        self.assertEqual(manager.state, "WAIT_START")
        self.assertEqual(target.question3_state, "IDLE")
        self.assertEqual(target.selected_target_mm, 25)
        manager.update(
            self._position(0),
            self._vst(3, 20, ACTIVE_FLAGS | TASK_ACTIVE), 20)
        self.assertEqual(manager.state, "RUNNING")
        manager.update(
            self._position(0), self._vst(3, 30, ACTIVE_FLAGS), 30)
        self.assertEqual(manager.state, "WAIT_START")
        self.assertEqual(target.question3_state, "IDLE")
        self.assertEqual(target.selected_target_mm, 0)
        manager.update(
            self._position(0),
            self._vst(3, 40, ACTIVE_FLAGS | TASK_ACTIVE), 40)
        self.assertEqual(manager.state, "RUNNING")

    def test_faulted_loop_does_not_start_and_stale_mode_is_ignored(self):
        target = BallTargetTouchController()
        manager = CompetitionTaskManager(target, offline_ms=500)
        manager.update(
            self._position(0), self._vst(3, 5, TASK_ACTIVE), 5)
        manager.update(
            self._position(0),
            self._vst(3, 10, ACTIVE_FLAGS | TASK_ACTIVE, faults=2), 10)
        self.assertEqual(manager.state, "WAIT_BALL_LOOP")
        manager.update(
            self._position(0),
            self._vst(3, 20, ACTIVE_FLAGS | TASK_ACTIVE), 20)
        self.assertEqual(target.question3_state, "CENTERING")

        stale = self._vst(3, 20, ACTIVE_FLAGS | TASK_ACTIVE)
        manager.update(self._position(0), stale, 521)
        self.assertEqual(manager.active_mode, 3)
        self.assertEqual(manager.state, "RUNNING")
        self.assertEqual(target.question3_state, "CENTERING")
        self.assertFalse(target.target_edit_enabled)
        self.assertFalse(manager.vst_fresh)

    def test_stale_mode6_locks_touch_until_fresh_again(self):
        target = BallTargetTouchController()
        manager = CompetitionTaskManager(target, offline_ms=500)
        manager.update(self._position(0), self._vst(6, 10), 10)
        self.assertTrue(target.target_edit_enabled)
        manager.update(self._position(0), self._vst(6, 10), 511)
        self.assertFalse(target.target_edit_enabled)
        manager.update(self._position(0), self._vst(6, 520), 520)
        self.assertTrue(target.target_edit_enabled)


if __name__ == "__main__":
    unittest.main()
