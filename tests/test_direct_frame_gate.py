"""Host tests for the fast full-frame axis detection gate."""

import unittest

from ball_axis_position import (
    filter_detections_in_normalized_roi,
    highest_score_detection,
)


class DirectFrameGateTest(unittest.TestCase):
    def test_rejects_higher_score_detection_above_trough_roi(self):
        detections = [
            {"x": 260.0, "y": 0.0, "w": 20.0, "h": 20.0,
             "score": 0.90},
            {"x": 4.0, "y": 270.0, "w": 24.0, "h": 24.0,
             "score": 0.20},
        ]

        gated = filter_detections_in_normalized_roi(
            detections, 416, 320, (0.0, 0.45, 1.0, 0.55))

        self.assertEqual(gated, [detections[1]])
        self.assertIs(highest_score_detection(gated), detections[1])

    def test_includes_centers_on_roi_boundary(self):
        detections = [
            {"x": -1.0, "y": 143.0, "w": 2.0, "h": 2.0,
             "score": 0.10},
            {"x": 415.0, "y": 319.0, "w": 2.0, "h": 2.0,
             "score": 0.20},
        ]

        gated = filter_detections_in_normalized_roi(
            detections, 416, 320, (0.0, 0.45, 1.0, 0.55))

        self.assertEqual(gated, detections)

    def test_invalid_roi_or_frame_rejects_all(self):
        detection = {
            "x": 10.0, "y": 10.0, "w": 4.0, "h": 4.0,
            "score": 0.5,
        }
        self.assertEqual(
            filter_detections_in_normalized_roi(
                [detection], 0, 320, (0.0, 0.0, 1.0, 1.0)),
            [],
        )
        self.assertEqual(
            filter_detections_in_normalized_roi(
                [detection], 416, 320, (0.0, 0.0, 0.0, 1.0)),
            [],
        )


if __name__ == "__main__":
    unittest.main()
