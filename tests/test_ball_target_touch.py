"""PC-side tests for the MaixCAM ball-target touch slider."""

import unittest

from ball_target_touch import (
    BallTargetTouchController,
    quantize_target_mm,
    slider_geometry,
    slider_target_from_x,
    slider_x_from_target,
)


class BallTargetTouchTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
