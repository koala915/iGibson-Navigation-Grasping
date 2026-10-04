#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "detection" / "calibration"))

import calibrate_rear_ground_homography as calibration


class RearGroundHomographyTests(unittest.TestCase):
    def test_synthetic_mapping_is_recovered(self):
        pixels = np.asarray([
            [180, 260], [320, 260], [460, 260],
            [180, 360], [320, 360], [460, 360],
            [220, 440], [420, 440],
        ], dtype=np.float64)
        expected_h = np.asarray([
            [0.0, -0.002, 1.32],
            [-0.001, 0.0, 0.32],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        ground = cv2.perspectiveTransform(
            pixels.reshape(-1, 1, 2), expected_h).reshape(-1, 2)

        result = calibration.solve(
            pixels.tolist(), ground.tolist(), rear_cam_to_front_m=0.2,
            camera_url="test://rear")

        actual_h = np.asarray(result["H_pixel_to_ground"], dtype=np.float64)
        actual_h /= actual_h[2, 2]
        np.testing.assert_allclose(actual_h, expected_h, atol=1e-6)
        self.assertEqual(result["sample_count"], len(pixels))
        self.assertEqual(result["inlier_count"], len(pixels))
        self.assertLess(result["max_reprojection_error_m"], 1e-6)

    def test_fewer_than_six_samples_are_refused(self):
        with self.assertRaisesRegex(ValueError, "at least six"):
            calibration.solve(
                [[0.0, 0.0]] * 5, [[0.0, 0.0]] * 5,
                rear_cam_to_front_m=0.2, camera_url="test://rear")


if __name__ == "__main__":
    unittest.main(verbosity=2)
