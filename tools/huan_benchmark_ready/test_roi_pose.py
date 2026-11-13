"""Regression checks for ROI bounds and conversion back to full-image pixels."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

import numpy as np


class FakePoseDetector:
    def __init__(self):
        self.calls = []
        self.candidate = {"bbox": np.array([1., 2., 5., 6.]),
                          "xy": np.tile([1., 2.], (17, 1)),
                          "conf": np.full(17, 0.9)}

    def detect(self, frame):
        self.calls.append(frame.shape)
        return [self.candidate]


class RoiPoseTests(unittest.TestCase):
    def setUp(self):
        parent = types.ModuleType("src")
        parent.__path__ = []
        base = types.ModuleType("src.torso_depth")
        base.PoseTorsoDetector = FakePoseDetector
        path = Path(__file__).with_name("carrierbot_roi_pose.py")
        spec = importlib.util.spec_from_file_location("roi_under_test", path)
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {"src": parent, "src.torso_depth": base}):
            spec.loader.exec_module(self.module)
        self.detector = self.module.PoseTorsoDetector()
        self.frame = np.zeros((100, 100, 3), dtype=np.uint8)

    def test_padding_and_bounds(self):
        self.assertEqual(self.module.padded_roi([10, 20, 30, 60], 100, 100), (6, 12, 35, 69))
        self.assertEqual(self.module.padded_roi([-5, -5, 10, 10], 100, 100), (0, 0, 14, 14))

    def test_invalid_boxes_do_not_run_inference(self):
        boxes = [None, [1, 2, 3], [1, 2, 3, 4, 5], [10, 10, 5, 5],
                 [float("nan"), 1, 5, 5], [110, 110, 120, 120], [-20, -20, -10, -10]]
        for bbox in boxes:
            with self.subTest(bbox=bbox):
                self.assertIsNone(self.module.padded_roi(bbox, 100, 100))
                self.assertEqual(self.detector.detect(self.frame, [{"bbox": bbox}]), [])
        self.assertEqual(self.detector.calls, [])

    def test_restores_bbox_and_keypoints_without_mutating_pose(self):
        result = self.detector.detect(self.frame, [{"bbox": [10, 20, 30, 60]}])
        self.assertEqual(self.detector.calls, [(57, 29, 3)])
        np.testing.assert_array_equal(result[0]["bbox"], [7., 14., 11., 18.])
        np.testing.assert_array_equal(result[0]["xy"], np.tile([7., 14.], (17, 1)))
        np.testing.assert_array_equal(result[0]["conf"], np.full(17, 0.9))
        np.testing.assert_array_equal(self.detector.candidate["bbox"], [1., 2., 5., 6.])
        np.testing.assert_array_equal(self.detector.candidate["xy"], np.tile([1., 2.], (17, 1)))

    def test_all_tracks_are_processed(self):
        result = self.detector.detect(self.frame, [
            {"bbox": [10, 20, 30, 60]}, {"bbox": [60, 40, 80, 80]}])
        self.assertEqual(len(result), 2)
        self.assertEqual(len(self.detector.calls), 2)
        np.testing.assert_array_equal(result[1]["bbox"], [57., 34., 61., 38.])

    def test_no_tracks_has_no_inference(self):
        self.assertEqual(self.detector.detect(self.frame, []), [])
        self.assertEqual(self.detector.calls, [])


if __name__ == "__main__":
    unittest.main()
