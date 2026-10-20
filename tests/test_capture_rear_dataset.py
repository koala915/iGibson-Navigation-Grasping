"""Pure parts of the rear-camera dataset capture tool."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'capture', ROOT / 'detection/debug_tools/capture_rear_dataset.py')
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)

from integration.sugarbox_ground_calibration import load_ground_calibration, pixel_to_ground


class CaptureTests(unittest.TestCase):
    def test_arm_camera_never_counts_as_a_change(self):
        a = np.zeros((240, 320), np.uint8)
        b = a.copy()
        b[220:235, 160:190] = 200                  # inside the arm camera region
        self.assertEqual(capture.change_score(a, b), 0)
        b[50:60, 40:50] = 200                      # a far box moved: 100 pixels
        self.assertEqual(capture.change_score(a, b), 100)
        self.assertEqual(capture.change_score(None, b), float('inf'))

    def test_guide_lines_land_where_the_homography_says(self):
        homography = load_ground_calibration(str(ROOT / 'detection/rear_ground_homography.json'))[0]
        points = capture.distance_line(homography, 1.0)
        self.assertGreater(len(points), 5)
        for u, v in points[::8]:
            x, _ = pixel_to_ground(u, v, homography)
            self.assertAlmostEqual(x - capture.WHEEL_TO_FILE_X_M, 1.0, delta=0.02)
        near = np.mean([v for _, v in capture.distance_line(homography, 0.5)])
        far = np.mean([v for _, v in capture.distance_line(homography, 1.4)])
        self.assertGreater(near, far)              # nearer floor is lower in the image


if __name__ == '__main__':
    unittest.main()
