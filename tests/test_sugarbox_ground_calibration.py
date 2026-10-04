"""Read-only calibration regressions without importing the hardware/vision loop."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from integration.sugarbox_ground_calibration import (
    find_calibration_pixels as find_pixels,
    inside_calibration_hull as inside_hull,
    load_ground_calibration as load_calibration,
    resolve_asset_paths,
)


class GroundCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.data = {
            "H_pixel_to_ground": [[0.01, 0, 0], [0, 0.01, 0], [0, 0, 1]],
            "rear_cam_to_front_m": 0.2,
            "samples": [{"u": u, "v": v, "x": u / 100, "y": v / 100,
                         "inlier": True}
                        for u, v in [(20, 20), (120, 20), (120, 120), (20, 120)]],
        }

    def load_data(self, data):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "calibration.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            return load_calibration(path)

    def test_committed_json_reads_seven_inliers_and_enforces_range(self):
        path = ROOT / "detection" / "rear_ground_homography.json"
        data = json.loads(path.read_text(encoding="utf-8"))
        pixels = find_pixels(data)
        expected = [[row["u"], row["v"]] for row in data["samples"]
                    if row.get("inlier", True) != False]
        self.assertEqual(len(pixels), 7)
        np.testing.assert_array_equal(pixels, np.asarray(expected, dtype=np.float32))
        h, offset, hull = load_calibration(path)
        np.testing.assert_allclose(h, data["H_pixel_to_ground"])
        self.assertEqual(offset, 0.2)
        self.assertGreater(cv2.contourArea(hull), 0)
        self.assertTrue(inside_hull(*pixels.mean(axis=0), hull))
        self.assertFalse(inside_hull(50, 50, hull))
        # Sample 9 was rejected by calibration and lies beyond the allowed margin.
        self.assertFalse(inside_hull(253, 321, hull))

    def test_wrong_declared_resolution_is_refused(self):
        for key, value in [("frame_width", 1280), ("frame_height", 720),
                           ("frame_width", "640"), ("frame_height", None)]:
            with self.subTest(key=key, value=value):
                data = copy.deepcopy(self.data)
                data[key] = value
                with self.assertRaisesRegex(RuntimeError, key):
                    self.load_data(data)

    def test_e1_or_other_declared_calibration_type_is_refused(self):
        for kind in ["arm_camera_ground_homography", "v23_e1_grasp_home", None]:
            with self.subTest(kind=kind):
                data = copy.deepcopy(self.data)
                data["type"] = kind
                with self.assertRaisesRegex(RuntimeError, "calibration type"):
                    self.load_data(data)

    def test_wrong_declared_coordinate_axes_origin_or_input_is_refused(self):
        for key, value in [("ground_output", "x_forward_y_right"),
                           ("origin", "base_link"),
                           ("pixel_input", "bbox_center_u_v"),
                           ("x_axis", "backward_positive_meters"),
                           ("y_axis", "right_positive_left_negative_meters")]:
            with self.subTest(key=key, value=value):
                data = copy.deepcopy(self.data)
                data["coordinate_convention"] = {key: value}
                with self.assertRaisesRegex(RuntimeError, "coordinate_convention." + key):
                    self.load_data(data)

    def test_malformed_coordinate_metadata_is_refused(self):
        for convention in [None, "x_forward_y_left", []]:
            with self.subTest(convention=convention):
                data = copy.deepcopy(self.data)
                data["coordinate_convention"] = convention
                with self.assertRaisesRegex(RuntimeError, "coordinate_convention"):
                    self.load_data(data)

    def test_matching_partial_metadata_remains_supported(self):
        self.data.update({"frame_width": 640, "type": "rear_camera_ground_homography",
                          "coordinate_convention": {"ground_output": "x_forward_y_left"}})
        _, _, hull = self.load_data(self.data)
        self.assertTrue(inside_hull(60, 60, hull))

    def test_rejected_samples_do_not_expand_hull(self):
        _, _, original_hull = self.load_data(self.data)
        self.data["samples"].append({"u": 1000, "v": 1000, "inlier": False})
        self.data["samples"].append({"u": float("nan"), "v": float("nan"),
                                     "inlier": False})
        _, _, hull = self.load_data(self.data)
        np.testing.assert_array_equal(hull, original_hull)
        self.assertFalse(inside_hull(1000, 1000, hull))

    def test_legacy_pixel_arrays_remain_supported(self):
        pixels = [[20, 20], [120, 20], [120, 120], [20, 120]]
        for key in ["image_points", "pixel_points", "src_points", "pixels", "pixel_xy"]:
            with self.subTest(key=key):
                data = {"calibration": {"matrix": self.data["H_pixel_to_ground"],
                                       key: pixels}}
                _, offset, hull = self.load_data(data)
                self.assertEqual(offset, 0.2)
                self.assertTrue(inside_hull(60, 60, hull))
                self.assertFalse(inside_hull(300, 300, hull))

    def test_missing_pixels_cannot_disable_range_gate(self):
        del self.data["samples"]
        with self.assertRaisesRegex(RuntimeError, "finite accepted"):
            self.load_data(self.data)
        self.assertFalse(inside_hull(60, 60, None))

    def test_invalid_samples_do_not_fall_back_to_legacy_points(self):
        self.data["image_points"] = [[20, 20], [120, 20], [20, 120]]
        invalid_sample = copy.deepcopy(self.data["samples"])
        invalid_sample[0]["u"] = float("nan")
        for samples in [None, "invalid", {}, invalid_sample]:
            with self.subTest(samples=samples):
                self.data["samples"] = samples
                with self.assertRaisesRegex(RuntimeError, "finite accepted"):
                    self.load_data(self.data)

    def test_invalid_or_nonfinite_accepted_samples_are_refused(self):
        for changed in [None, {"u": 20}, {"u": float("nan"), "v": 20},
                        {"u": 20, "v": float("inf")}, {"u": "20", "v": 20},
                        {"u": True, "v": 20},
                        {"u": 20, "v": 20, "x": float("nan"), "y": 0}]:
            with self.subTest(sample=changed):
                data = copy.deepcopy(self.data)
                data["samples"][0] = changed
                with self.assertRaisesRegex(RuntimeError, "finite accepted"):
                    self.load_data(data)

    def test_too_few_duplicate_collinear_or_rejected_points_are_refused(self):
        for points in [[], [(20, 20), (120, 20)], [(20, 20)] * 4,
                       [(20, 20), (60, 60), (120, 120)]]:
            with self.subTest(points=points):
                self.data["samples"] = [{"u": u, "v": v} for u, v in points]
                with self.assertRaises(RuntimeError):
                    self.load_data(self.data)
        self.setUp()
        for row in self.data["samples"]:
            row["inlier"] = False
        with self.assertRaises(RuntimeError):
            self.load_data(self.data)

    def test_nonfinite_legacy_pixels_are_refused(self):
        del self.data["samples"]
        for invalid in [float("nan"), float("inf")]:
            self.data["image_points"] = [[20, 20], [120, 20], [20, invalid]]
            with self.assertRaises(RuntimeError):
                self.load_data(self.data)

    def test_missing_singular_or_nonfinite_matrix_is_refused(self):
        for h in [None, [[1, 0, 0], [0, 0, 0], [0, 0, 1]],
                  [[float("nan"), 0, 0], [0, 1, 0], [0, 0, 1]],
                  [[1, 0, 0], [0, float("inf"), 0], [0, 0, 1]]]:
            with self.subTest(h=h):
                data = copy.deepcopy(self.data)
                data["H_pixel_to_ground"] = h
                with self.assertRaisesRegex(RuntimeError, "Homography matrix"):
                    self.load_data(data)

    def test_nonfinite_offset_is_refused(self):
        for offset in [float("nan"), float("inf"), "invalid"]:
            with self.subTest(offset=offset):
                self.data["rear_cam_to_front_m"] = offset
                with self.assertRaisesRegex(RuntimeError, "must be finite"):
                    self.load_data(self.data)

    def test_invalid_query_or_hull_never_authorizes_target(self):
        _, _, hull = self.load_data(self.data)
        for u, v in [(float("nan"), 50), (50, float("inf")), ("bad", 50)]:
            with self.subTest(query=(u, v)):
                self.assertFalse(inside_hull(u, v, hull))
        for bad_hull in [None, [], [[0, 0], [1, 1], [2, 2]],
                         [[0, 0], [1, 1], [0, float("nan")]]]:
            with self.subTest(hull=bad_hull):
                self.assertFalse(inside_hull(50, 50, bad_hull))

    def test_existing_pixel_margin_is_preserved(self):
        _, _, hull = self.load_data(self.data)
        self.assertTrue(inside_hull(15, 60, hull))
        self.assertFalse(inside_hull(9, 60, hull))

    def test_missing_file_is_refused(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(FileNotFoundError):
                load_calibration(Path(folder) / "missing.json")


class AssetPathTests(unittest.TestCase):
    def asset_paths(self, environment):
        return resolve_asset_paths(str(ROOT), environment)

    def test_defaults_use_committed_repo_assets(self):
        paths = self.asset_paths({})
        self.assertEqual(Path(paths["yolo"]), ROOT / "detection/models/best.pt")
        self.assertEqual(Path(paths["homography"]), ROOT / "detection/rear_ground_homography.json")
        self.assertTrue(Path(paths["yolo"]).is_file())
        self.assertTrue(Path(paths["homography"]).is_file())

    def test_sam_runner_name_and_pr7_alias_are_supported(self):
        self.assertEqual(self.asset_paths({"SUGARBOX_SAM2_MODEL": "alias.pt"})["sam"], "alias.pt")
        self.assertEqual(self.asset_paths({"SUGARBOX_SAM_MODEL": "runner.pt",
                                          "SUGARBOX_SAM2_MODEL": "alias.pt"})["sam"], "runner.pt")

    def test_individual_and_asset_directory_overrides_are_preserved(self):
        paths = self.asset_paths({"SUGARBOX_ASSET_DIR": "assets", "SUGARBOX_YOLO_MODEL": "custom.pt",
                                 "SUGARBOX_HOMOGRAPHY": "custom.json"})
        self.assertEqual(paths["yolo"], "custom.pt")
        self.assertEqual(paths["homography"], "custom.json")
        self.assertEqual(Path(paths["sam"]), Path("assets/models/sam2.1_b.pt"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
