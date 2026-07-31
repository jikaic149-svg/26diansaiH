"""Regression test for the frozen right trough endpoint."""

import unittest

from horizontal_trough_tracker import HorizontalTroughTracker


class TroughFixedRightTest(unittest.TestCase):
    @staticmethod
    def _axis(right_x):
        return {
            "valid": True,
            "p0": (20.0, 240.0),
            "p1": (float(right_x), 260.0),
            "centerline": [(20.0, 240.0), (float(right_x), 260.0)],
            "slope_y_per_x": 20.0 / (float(right_x) - 20.0),
            "intercept_y": 240.0 - 20.0 * 20.0 /
                (float(right_x) - 20.0),
            "confidence": 1.0,
            "median_width_px": 32.0,
            "roi": (0, 198, 416, 122),
        }

    def test_right_endpoint_never_corrects_or_predicts(self):
        tracker = HorizontalTroughTracker(settings={
            "endpoint_occlusion_enabled": False,
            "endpoint_ab_fixed_right_enabled": True,
            "endpoint_ab_fixed_right_x": 415.0,
        })
        first, accepted = tracker._filter_valid_axis(
            self._axis(405.0), timestamp_ms=0)
        self.assertTrue(accepted)
        self.assertAlmostEqual(first["p1"][0], 415.0)
        self.assertEqual(first["endpoint_v_right"], 0.0)

        second, accepted = tracker._filter_valid_axis(
            self._axis(382.0), timestamp_ms=100)
        self.assertTrue(accepted)
        self.assertAlmostEqual(second["p1"][0], 415.0)
        self.assertEqual(second["endpoint_v_right"], 0.0)
        self.assertEqual(second["endpoint_predict_px_right"], 0.0)
        self.assertIn("RIGHT_FIXED", second["endpoint_ab"])

        held = tracker._predict_held_axis(second, timestamp_ms=150)
        self.assertAlmostEqual(held["p1"][0], 415.0)
        self.assertEqual(held["endpoint_v_right"], 0.0)
        self.assertEqual(held["endpoint_predict_px_right"], 0.0)
        self.assertIn("RIGHT_FIXED", held["endpoint_ab"])


if __name__ == "__main__":
    unittest.main()
