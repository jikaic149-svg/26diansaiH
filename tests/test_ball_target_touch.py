"""PC-side tests for the MaixCAM ball-target touch slider."""

import unittest

from ball_target_touch import (
    BallTargetTouchController,
    axis_point_from_t,
    quantize_target_mm,
    question3_button_geometry,
    slider_geometry,
    slider_target_from_x,
    slider_x_from_target,
    target_axis_t_from_mm,
)


class BallTargetTouchTest(unittest.TestCase):
    @staticmethod
    def _position(position_mm, valid=True):
        return {
            "valid": valid,
            "position_cm": float(position_mm) / 10.0,
        }

    @staticmethod
    def _vst(now_ms, fault_bits=0):
        return {
            "received_at_ms": now_ms,
            "fault_bits": fault_bits,
            "motor_target_raw": 1200,
            "motor_feedback_raw": 1190,
            "motor_speed_centi_rpm": 250,
        }

    def test_slider_boundaries_and_center(self):
        geometry = slider_geometry(320, 240)
        left = geometry["left"]
        right = geometry["right"]
        self.assertEqual(
            slider_target_from_x(
                left - 100, left, right, -120, 120, 1), -120)
        self.assertEqual(
            slider_target_from_x(
                right + 100, left, right, -120, 120, 1), 120)
        center = slider_x_from_target(0, left, right, -120, 120)
        self.assertEqual(
            slider_target_from_x(center, left, right, -120, 120, 1), 0)

    def test_quantization_and_reverse_mapping(self):
        self.assertEqual(quantize_target_mm(37.4, -120, 120, 1), 37)
        self.assertEqual(quantize_target_mm(37.6, -120, 120, 1), 38)
        self.assertEqual(quantize_target_mm(-200, -120, 120, 1), -120)
        self.assertEqual(quantize_target_mm(200, -120, 120, 1), 120)
        self.assertEqual(quantize_target_mm(12.4, -120, 120, 5), 10)
        self.assertEqual(quantize_target_mm(12.6, -120, 120, 5), 15)

    def test_target_uses_inverse_visual_calibration(self):
        pairs = (
            (0.08, -10.0),
            (0.28, -5.0),
            (0.49, 0.0),
            (0.72, 5.0),
            (0.90, 10.0),
        )
        self.assertAlmostEqual(
            target_axis_t_from_mm(0.0, -120, 120, pairs), 0.49)
        self.assertAlmostEqual(
            target_axis_t_from_mm(50.0, -120, 120, pairs), 0.72)

    def test_target_point_follows_trough_polyline(self):
        axis = {
            "valid": True,
            "centerline": [(10.0, 20.0), (60.0, 20.0), (60.0, 70.0)],
        }
        self.assertEqual(axis_point_from_t(axis, 0.25), (35.0, 20.0))
        self.assertEqual(axis_point_from_t(axis, 0.75), (60.0, 45.0))

    def test_initial_vst_sync_retry_and_confirmation(self):
        class FakeLink:
            def __init__(self):
                self.sent = []

            def send_ball_target(self, target_mm, now_ms=None):
                self.sent.append((target_mm, now_ms))
                return True

        controller = BallTargetTouchController(retry_ms=250)
        link = FakeLink()
        initial_vst = {
            "sequence": 1, "target_mm": 37, "received_at_ms": 10}
        controller.update_feedback(initial_vst)
        self.assertEqual(controller.selected_target_mm, 37)
        self.assertEqual(controller.link_status(10), "OK")

        controller.selected_target_mm = -42
        controller.user_selected = True
        controller.pending = True
        controller.update_feedback(initial_vst)
        self.assertTrue(controller.pending)
        self.assertTrue(controller.try_send(link, 20))
        self.assertFalse(controller.try_send(link, 269))
        self.assertTrue(controller.try_send(link, 270))
        self.assertEqual(link.sent, [(-42, 20), (-42, 270)])
        controller.update_feedback({
            "sequence": 2, "target_mm": -42, "received_at_ms": 280})
        self.assertFalse(controller.pending)
        self.assertEqual(controller.link_status(280), "OK")
        self.assertEqual(controller.link_status(781), "OFFLINE")

    def test_retry_has_finite_attempt_limit(self):
        class FakeLink:
            def send_ball_target(self, _target_mm, now_ms=None):
                return now_ms is not None

        controller = BallTargetTouchController(
            retry_ms=10, max_attempts=2)
        controller.pending = True
        self.assertTrue(controller.try_send(FakeLink(), 0))
        self.assertTrue(controller.try_send(FakeLink(), 10))
        self.assertFalse(controller.try_send(FakeLink(), 20))
        self.assertEqual(controller.pending_attempt_count, 2)

    def test_outside_ui_vst_does_not_false_confirm(self):
        controller = BallTargetTouchController()
        controller.selected_target_mm = 120
        controller.user_selected = True
        controller.initial_sync_done = True
        controller.pending = True
        controller.update_feedback({"target_mm": 125, "received_at_ms": 1})
        self.assertTrue(controller.pending)
        self.assertEqual(controller.reported_target_mm, 125)

    def test_visual_to_f407_target_polarity(self):
        class FakeLink:
            def __init__(self):
                self.target_mm = None

            def send_ball_target(self, target_mm, now_ms=None):
                self.target_mm = target_mm
                return now_ms is not None

        controller = BallTargetTouchController(
            target_to_f4_polarity=-1.0)
        controller.update_feedback({
            "sequence": 1, "target_mm": -37, "received_at_ms": 1})
        self.assertEqual(controller.selected_target_mm, 37)
        self.assertEqual(controller.reported_target_mm, 37)
        controller.selected_target_mm = -42
        controller.pending = True
        link = FakeLink()
        self.assertTrue(controller.try_send(link, 2))
        self.assertEqual(link.target_mm, 42)

    def test_question3_button_is_separate_from_slider(self):
        button = question3_button_geometry(320, 240)
        slider = slider_geometry(320, 240)
        self.assertLess(
            button["y"] + button["height"], slider["touch_top"])

    def test_question3_uses_real_frames_and_runs_local_sequence(self):
        controller = BallTargetTouchController(
            target_to_f4_polarity=-1.0)
        self.assertTrue(controller.start_question3())
        self.assertEqual(controller.question3_state, "CENTERING")
        self.assertEqual(controller.selected_target_mm, 0)

        # Invalid vision and faulted VST must not start the timed sequence.
        self.assertFalse(controller.update_question3(
            self._position(0, valid=False), self._vst(100), 100))
        self.assertFalse(controller.update_question3(
            self._position(0), self._vst(110, fault_bits=2), 110))
        self.assertEqual(controller.question3_state, "CENTERING")

        self.assertTrue(controller.update_question3(
            self._position(5), self._vst(120), 120))
        self.assertEqual(controller.question3_state, "RIGHT")
        self.assertEqual(controller.selected_target_mm, 50)
        self.assertEqual(controller.question3_start_ms, 120)

        self.assertTrue(controller.update_question3(
            self._position(50), self._vst(1500), 1500))
        self.assertEqual(controller.question3_state, "LEFT")
        self.assertEqual(controller.selected_target_mm, -50)

        self.assertTrue(controller.update_question3(
            self._position(-50), self._vst(4100), 4100))
        self.assertEqual(controller.question3_state, "DONE")
        self.assertEqual(controller.selected_target_mm, -50)
        self.assertEqual(controller.question3_elapsed_ms, 3980)
        self.assertTrue(controller.question3_passed)

    def test_question3_timeout_keeps_exact_negative_five_cm_target(self):
        controller = BallTargetTouchController(
            question3_hard_timeout_ms=8000)
        controller.start_question3()
        controller.update_question3(
            self._position(0), self._vst(10), 10)
        controller.update_question3(
            self._position(32), self._vst(100), 100)
        self.assertTrue(controller.update_question3(
            self._position(33, valid=False), self._vst(8011), 8011))
        self.assertEqual(controller.question3_state, "FAILED")
        self.assertEqual(controller.selected_target_mm, -50)


if __name__ == "__main__":
    unittest.main()
