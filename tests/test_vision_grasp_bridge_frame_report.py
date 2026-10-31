"""The E1 bridge's per-frame report: what the camera saw, rejected boxes included."""
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from integration import vision_grasp_bridge as bridge

# 1 px = 1 mm, so the bottom-centre pixel reads straight off as base XY.
ARGS = SimpleNamespace(homography_matrix=[[0.001, 0, 0], [0, 0.001, 0], [0, 0, 1]])


def identity(u, v):
    return u, v


class FrameReportTests(unittest.TestCase):
    def test_estimate_is_the_bottom_centre_even_outside_the_hull(self):
        with patch.object(bridge.acg, "undistort_pixel", side_effect=identity):
            self.assertEqual(bridge.rough_floor_estimate(ARGS, (100, 0, 300, 186)),
                             [0.2, 0.186])

    def test_no_estimate_when_the_floor_contact_is_cut_off_or_uncalibrated(self):
        with patch.object(bridge.acg, "undistort_pixel", side_effect=identity):
            self.assertIsNone(bridge.rough_floor_estimate(
                ARGS, (100, 236, 300, bridge.acg.IMG_H - 1)))
            self.assertIsNone(bridge.rough_floor_estimate(
                SimpleNamespace(homography_matrix=None), (100, 0, 300, 186)))

    def test_record_says_what_was_seen_and_whether_it_was_used(self):
        with patch.object(bridge.acg, "undistort_pixel", side_effect=identity):
            record = bridge.frame_report_record(ARGS, (100, 0, 300, 186), None,
                                                "outside calibration hull", clock=lambda: 7.0)
        self.assertEqual(record, {"t": 7.0, "seen": True, "bbox": [100.0, 0.0, 300.0, 186.0],
                                  "border": ["top"], "accepted": False,
                                  "note": "outside calibration hull", "estimate": [0.2, 0.186]})
        self.assertEqual(bridge.frame_report_record(ARGS, None, None, None, clock=lambda: 8.0),
                         {"t": 8.0, "seen": False})

    def test_report_is_replaced_whole(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "frame.json"
            bridge.write_frame_report(path, {"t": 1.0, "seen": False})
            bridge.write_frame_report(path, {"t": 2.0, "seen": True})
            self.assertEqual(json.loads(path.read_text()), {"t": 2.0, "seen": True})
            self.assertEqual([p.name for p in Path(folder).iterdir()], ["frame.json"])

    def test_unwritable_report_never_stops_the_bridge(self):
        with tempfile.TemporaryDirectory() as folder:
            bridge.write_frame_report(Path(folder) / "missing" / "frame.json", {"t": 1.0})


if __name__ == "__main__":
    unittest.main()
