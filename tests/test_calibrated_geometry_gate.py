import unittest

from ball_axis_position import (
    filter_detections_on_trough,
    project_ball_axis_position,
)


AXIS = {
    "valid": True,
    "p0": (0.0, 0.0),
    "p1": (100.0, 0.0),
    "centerline": [(0.0, 0.0), (100.0, 0.0)],
    "median_width_px": 10.0,
}


class CalibratedGeometryGateTest(unittest.TestCase):
    def test_calibrated_mode_can_keep_detection_beyond_dynamic_endpoint(self):
        detection = {"x": 108.0, "y": -5.0, "w": 10.0, "h": 10.0,
                     "score": 0.9}
        self.assertEqual([], filter_detections_on_trough(
            [detection], AXIS, endpoint_margin_ratio=0.08))
        self.assertEqual([detection], filter_detections_on_trough(
            [detection], AXIS, endpoint_margin_ratio=0.08,
            enforce_endpoint_bounds=False))

    def test_axis_distance_gate_is_only_relaxed_explicitly(self):
        ball = {"x": 45.0, "y": 45.0, "w": 10.0, "h": 10.0,
                "score": 0.9}
        strict = project_ball_axis_position(ball, AXIS)
        relaxed = project_ball_axis_position(
            ball, AXIS, enforce_axis_distance=False)
        self.assertFalse(strict["valid"])
        self.assertEqual("BALL_OFF_TROUGH", strict["reason"])
        self.assertTrue(relaxed["valid"])
        self.assertEqual(0.0, relaxed["position_norm"])


if __name__ == "__main__":
    unittest.main()
