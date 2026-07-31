"""PC-side tests for the frozen MaixCAM2 UI size contract."""

import unittest

from ui_size_calibration import (
    calibrate_frozen_ui,
    fit_contain_rect,
)


class UiSizeCalibrationTest(unittest.TestCase):

    def test_actual_maixcam2_fit_and_touch_region(self):
        layout = calibrate_frozen_ui(
            416, 320, 640, 480, 416, 320, 640, 480)
        self.assertEqual(
            layout["content"],
            {"x": 8, "y": 0, "width": 624, "height": 480})
        self.assertEqual(
            layout["slider_touch"],
            {"x": 26, "y": 393, "width": 588, "height": 87})

    def test_size_change_is_rejected_after_freeze(self):
        with self.assertRaises(RuntimeError):
            calibrate_frozen_ui(
                320, 320, 640, 480, 416, 320, 640, 480)

    def test_invalid_dimensions_are_rejected(self):
        with self.assertRaises(ValueError):
            fit_contain_rect(0, 320, 480, 640)


if __name__ == "__main__":
    unittest.main()
