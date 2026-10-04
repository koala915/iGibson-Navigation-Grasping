import tempfile
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from integration import vision_grasp_bridge as bridge
from integration.grasp_home_homography import make_calibration, save_calibration


class FakeCapture:
    def __init__(self, opened, reads=()):
        self.opened = opened
        self.reads = list(reads)
        self.released = False

    def isOpened(self):
        return self.opened and not self.released

    def read(self):
        if self.reads:
            return self.reads.pop(0)
        return False, None

    def release(self):
        self.released = True


def geometry_args(**overrides):
    values = {
        "camera_pose": "grasp-home",
        "pose": None,
        "calibration_only": False,
        "homography": None,
        "camera_height": None,
        "camera_theta": None,
        "cam_x": None,
        "cam_y": None,
        "sign_x": None,
        "sign_y": None,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class VisionGraspBridgePoseTests(unittest.TestCase):
    def test_camera_open_is_confirmed_only_after_a_real_frame(self):
        frame = SimpleNamespace(size=3, shape=(1, 1, 3))
        cannot_open = FakeCapture(False)
        opens_and_reads = FakeCapture(True, [(False, None), (True, frame)])
        with patch.object(bridge, "open_capture",
                          side_effect=[cannot_open, opens_and_reads]):
            cap, first_frame, error = bridge.open_capture_checked(
                "mock", attempts=2, probe_reads=2,
                retry_delay=0.0, probe_delay=0.0,
            )
        self.assertIs(cap, opens_and_reads)
        self.assertIs(first_frame, frame)
        self.assertEqual(error, "")
        self.assertTrue(cannot_open.released)
        self.assertFalse(opens_and_reads.released)

    def test_camera_open_with_no_frames_fails_after_finite_retries(self):
        first = FakeCapture(True)
        second = FakeCapture(True)
        with patch.object(bridge, "open_capture", side_effect=[first, second]):
            cap, frame, error = bridge.open_capture_checked(
                "mock", attempts=2, probe_reads=2,
                retry_delay=0.0, probe_delay=0.0,
            )
        self.assertIsNone(cap)
        self.assertIsNone(frame)
        self.assertIn("produced no frame", error)
        self.assertTrue(first.released)
        self.assertTrue(second.released)

    def test_grasp_home_runtime_requires_homography(self):
        with self.assertRaisesRegex(SystemExit, "--homography"):
            bridge.resolve_camera_geometry(geometry_args())

    def test_calibration_only_never_needs_a_homography(self):
        args = geometry_args(calibration_only=True)
        bridge.resolve_camera_geometry(args)
        self.assertIsNone(args.homography_matrix)

    def test_calibration_only_rejects_nav_home(self):
        args = geometry_args(camera_pose="nav-home", calibration_only=True)
        with self.assertRaisesRegex(SystemExit, "only supported"):
            bridge.resolve_camera_geometry(args)

    def test_nav_home_resolves_legacy_measured_defaults(self):
        args = geometry_args(camera_pose="nav-home")
        bridge.resolve_camera_geometry(args)
        self.assertEqual(args.sign_x, 1.0)
        self.assertEqual(args.sign_y, -1.0)
        self.assertAlmostEqual(args.camera_height, bridge.H)

    def test_camera_pose_and_explicit_pose_cannot_disagree(self):
        args = geometry_args(camera_pose="grasp-home",
                             pose=bridge.acg.V17_NAV_HOME.name)
        with self.assertRaisesRegex(SystemExit, "conflicts"):
            bridge.resolve_pose(args)

    def test_calibration_rejects_a_clipped_bbox(self):
        record, reason = bridge.calibration_record_from_bbox(
            (0.0, 100.0, 50.0, 200.0), "sugarbox")
        self.assertIsNone(record)
        self.assertTrue(reason)

    def test_verified_homography_maps_inside_and_rejects_outside_hull(self):
        pixels = [(u, v) for v in (100.0, 300.0, 450.0)
                  for u in (100.0, 300.0, 500.0)]
        bases = [(0.30 - 0.0002 * v, 0.00025 * (u - 300.0))
                 for u, v in pixels]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "grasp_home.json"
            save_calibration(path, make_calibration(pixels, bases))
            args = geometry_args(homography=str(path))
            bridge.resolve_camera_geometry(args)
            x, y = bridge.apply_grasp_home_mapping(args, 300.0, 300.0)
            self.assertAlmostEqual(x, 0.24, places=8)
            self.assertAlmostEqual(y, 0.0, places=8)
            edge_x, edge_y = bridge.apply_grasp_home_mapping(args, 100.0, 100.0)
            self.assertAlmostEqual(edge_x, 0.28, places=8)
            self.assertAlmostEqual(edge_y, -0.05, places=8)
            with self.assertRaisesRegex(ValueError, "outside calibration hull"):
                bridge.apply_grasp_home_mapping(args, 50.0, 50.0)

    def _homography_args(self, directory, **over):
        pixels = [(u, v) for v in (100.0, 300.0, 450.0)
                  for u in (100.0, 300.0, 500.0)]
        bases = [(0.30 - 0.0002 * v, 0.00025 * (u - 300.0))
                 for u, v in pixels]
        path = Path(directory) / "grasp_home.json"
        save_calibration(path, make_calibration(pixels, bases))
        args = geometry_args(homography=str(path), **over)
        bridge.resolve_camera_geometry(args)
        return args

    def _payload(self, args, transform):
        return bridge.build_homography_payload(
            args, (280.0, 240.0, 320.0, 300.0), "sugarbox",
            bridge.acg.V23_E1_GRASP_HOME,
            {"_fallback": 0.015, "sugarbox": 0.015},
            {"sugarbox": 0.03}, base_xy_transform=transform)

    def test_a_rigid_base_transform_is_accepted(self):
        import math as _m

        def rotate(xy, angle=_m.radians(20.0), pivot=(0.118146, -0.003359)):
            dx, dy = xy[0] - pivot[0], xy[1] - pivot[1]
            return (pivot[0] + _m.cos(angle) * dx - _m.sin(angle) * dy,
                    pivot[1] + _m.sin(angle) * dx + _m.cos(angle) * dy)

        with tempfile.TemporaryDirectory() as directory:
            args = self._homography_args(
                directory, policy_x_range=[0.10, 0.40],
                policy_y_range=[-0.20, 0.20])
            payload, note = self._payload(args, rotate)
            self.assertIsNotNone(payload, note)

    def test_a_scaling_base_transform_is_refused(self):
        # The transform is documented rigid and the width is measured after it.
        # A scale would rescale the width and the bracketing test together, and
        # both would still look entirely reasonable.
        with tempfile.TemporaryDirectory() as directory:
            args = self._homography_args(
                directory, policy_x_range=[0.05, 0.60],
                policy_y_range=[-0.30, 0.30])
            payload, note = self._payload(args, lambda xy: (xy[0] * 1.5,
                                                            xy[1] * 1.5))
            self.assertIsNone(payload)
            self.assertIn("not rigid", note)

    def test_calibration_batch_uses_medians(self):
        rows = []
        for u in (100.0, 102.0, 500.0):
            rows.append({
                "class": "bottle-cap", "u": u, "v": 200.0,
                "left_u": u - 10.0, "left_v": 200.0,
                "right_u": u + 10.0, "right_v": 200.0,
                "raw_u": u + 1.0, "raw_v": 201.0,
            })
        result = bridge.median_calibration_record(rows)
        self.assertEqual(result["u"], 102.0)
        self.assertEqual(result["samples"], 3)

    def test_grasp_home_payload_uses_homography_and_keeps_v21_height(self):
        pixels = [(u, v) for v in (100.0, 300.0, 450.0)
                  for u in (100.0, 300.0, 500.0)]
        bases = [(0.30 - 0.0002 * v, 0.00025 * (u - 300.0))
                 for u, v in pixels]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "grasp_home.json"
            save_calibration(path, make_calibration(pixels, bases))
            args = geometry_args(homography=str(path))
            bridge.resolve_camera_geometry(args)
            with patch.object(bridge.acg, "undistort_pixel",
                              side_effect=lambda u, v: (u, v)):
                payload, note = bridge.build_homography_payload(
                    args, (280.0, 250.0, 320.0, 300.0), "sugarbox",
                    bridge.acg.V21_C3_GRASP_HOME,
                    {"_fallback": 0.02, "sugarbox": 0.0325},
                    {"sugarbox": 0.065},
                )
        self.assertIsNotNone(payload, note)
        self.assertAlmostEqual(payload["x"], 0.24)
        self.assertAlmostEqual(payload["y"], 0.0)
        self.assertEqual(payload["height"], 0.065)
        self.assertEqual(payload["cam_pose_name"],
                         bridge.acg.V21_C3_GRASP_HOME.name)
        self.assertEqual(payload["cam_pose"],
                         list(bridge.acg.V21_C3_GRASP_HOME.arm_deg))

    def test_rigid_base_transform_moves_target_and_preserves_width(self):
        pixels = [(u, v) for v in (100.0, 300.0, 450.0)
                  for u in (100.0, 300.0, 500.0)]
        bases = [(0.30 - 0.0002 * v, 0.00025 * (u - 300.0))
                 for u, v in pixels]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "grasp_home.json"
            save_calibration(path, make_calibration(pixels, bases))
            args = geometry_args(homography=str(path))
            bridge.resolve_camera_geometry(args)
            with patch.object(bridge.acg, "undistort_pixel",
                              side_effect=lambda u, v: (u, v)):
                plain, _ = bridge.build_homography_payload(
                    args, (280.0, 250.0, 320.0, 300.0), "sugarbox",
                    bridge.acg.V21_C3_GRASP_HOME,
                    {"_fallback": 0.02}, {},
                )
                moved, note = bridge.build_homography_payload(
                    args, (280.0, 250.0, 320.0, 300.0), "sugarbox",
                    bridge.acg.V21_C3_GRASP_HOME,
                    {"_fallback": 0.02}, {},
                    base_xy_transform=lambda xy: (xy[0], xy[1] + 0.02),
                )
                rejected, rejected_note = bridge.build_homography_payload(
                    args, (280.0, 250.0, 320.0, 300.0), "sugarbox",
                    bridge.acg.V21_C3_GRASP_HOME,
                    {"_fallback": 0.02}, {},
                    base_xy_transform=lambda xy: (xy[0], xy[1] + 0.20),
                )
        self.assertAlmostEqual(moved["x"], plain["x"])
        self.assertAlmostEqual(moved["y"], plain["y"] + 0.02)
        self.assertAlmostEqual(moved["w"], plain["w"])
        self.assertIn("rigid base-XY transform", note)
        self.assertIsNone(rejected)
        self.assertIn("outside the policy", rejected_note)

    def test_forward_offset_moves_only_base_x_and_is_bounded(self):
        self.assertEqual(
            bridge.apply_grasp_forward_offset((0.2687, -0.0035), 5.0),
            (0.2737, -0.0035),
        )
        for invalid in (-0.1, 15.1, float("inf"), float("nan")):
            with self.assertRaisesRegex(ValueError, "forward offset"):
                bridge.apply_grasp_forward_offset((0.25, 0.0), invalid)

    def test_forward_offset_is_checked_by_policy_after_translation(self):
        pixels = [(u, v) for v in (100.0, 300.0, 450.0)
                  for u in (100.0, 300.0, 500.0)]
        bases = [(0.30 - 0.0002 * v, 0.00025 * (u - 300.0))
                 for u, v in pixels]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "grasp_home.json"
            save_calibration(path, make_calibration(pixels, bases))
            args = geometry_args(
                homography=str(path), policy_x_range=[0.205, 0.243],
                policy_y_range=[-0.070, 0.065])
            bridge.resolve_camera_geometry(args)
            with patch.object(bridge.acg, "undistort_pixel",
                              side_effect=lambda u, v: (u, v)):
                payload, note = bridge.build_homography_payload(
                    args, (280.0, 250.0, 320.0, 300.0), "sugarbox",
                    bridge.acg.V21_C3_GRASP_HOME,
                    {"_fallback": 0.02}, {},
                    base_xy_transform=lambda xy:
                        bridge.apply_grasp_forward_offset(xy, 5.0),
                )
        self.assertIsNone(payload)
        self.assertIn("outside the policy", note)

    def test_bbox_sides_may_leave_center_hull_for_bounded_width_only(self):
        pixels = [(u, v) for v in (100.0, 300.0, 450.0)
                  for u in (100.0, 300.0, 500.0)]
        bases = [(0.30 - 0.0002 * v, 0.00025 * (u - 300.0))
                 for u, v in pixels]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "grasp_home.json"
            save_calibration(path, make_calibration(pixels, bases))
            args = geometry_args(homography=str(path))
            bridge.resolve_camera_geometry(args)
            with patch.object(bridge.acg, "undistort_pixel",
                              side_effect=lambda u, v: (u, v)):
                # Centre u=480 is calibrated; only the right silhouette point
                # u=520 lies outside the centre-point hull.  It is used for a
                # bounded 2 cm width estimate, never as the grasp target.
                payload, note = bridge.build_homography_payload(
                    args, (440.0, 250.0, 520.0, 300.0), "sugarbox",
                    bridge.acg.V21_C3_GRASP_HOME,
                    {"_fallback": 0.02, "sugarbox": 0.0325},
                    {"sugarbox": 0.065},
                )
                self.assertIsNotNone(payload, note)
                self.assertIn("bounded bbox-side width extrapolation", note)
                self.assertAlmostEqual(payload["w"], 0.02)

                # The exception is width-only: an uncalibrated target centre
                # remains fail-closed.
                payload, note = bridge.build_homography_payload(
                    args, (500.0, 250.0, 540.0, 300.0), "sugarbox",
                    bridge.acg.V21_C3_GRASP_HOME,
                    {"_fallback": 0.02, "sugarbox": 0.0325},
                    {"sugarbox": 0.065},
                )
                self.assertIsNone(payload)
                self.assertIn("outside calibration hull", note)

                # Width extrapolation cannot be used to wave through an object
                # wider than the physical jaw opening.
                payload, note = bridge.build_homography_payload(
                    args, (280.0, 250.0, 550.0, 300.0), "sugarbox",
                    bridge.acg.V21_C3_GRASP_HOME,
                    {"_fallback": 0.02, "sugarbox": 0.0325},
                    {"sugarbox": 0.065},
                )
                self.assertIsNone(payload)
                self.assertIn("past the 6.0 cm", note)

    def test_top_only_clip_requires_opt_in_and_keeps_other_edges_strict(self):
        pixels = [(u, v) for v in (100.0, 300.0, 450.0)
                  for u in (100.0, 300.0, 500.0)]
        bases = [(0.30 - 0.0002 * v, 0.00025 * (u - 300.0))
                 for u, v in pixels]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "grasp_home.json"
            save_calibration(path, make_calibration(pixels, bases))
            args = geometry_args(homography=str(path))
            bridge.resolve_camera_geometry(args)
            with patch.object(bridge.acg, "undistort_pixel",
                              side_effect=lambda u, v: (u, v)):
                payload, note = bridge.build_homography_payload(
                    args, (280.0, 0.0, 320.0, 300.0), "sugarbox",
                    bridge.acg.V21_C3_GRASP_HOME,
                    {"_fallback": 0.02, "sugarbox": 0.0325},
                    {"sugarbox": 0.065},
                )
                self.assertIsNone(payload)
                self.assertIn("top", note)

                args.allow_top_clipped_grasp_home = True
                payload, note = bridge.build_homography_payload(
                    args, (280.0, 0.0, 320.0, 300.0), "sugarbox",
                    bridge.acg.V21_C3_GRASP_HOME,
                    {"_fallback": 0.02, "sugarbox": 0.0325},
                    {"sugarbox": 0.065},
                )
                self.assertIsNotNone(payload, note)
                self.assertIn("accepted top-only clip", note)

                for bbox, edge in (((0.0, 0.0, 320.0, 300.0), "left"),
                                   ((280.0, 0.0, 639.0, 300.0), "right"),
                                   ((280.0, 0.0, 320.0, 479.0), "bottom")):
                    payload, note = bridge.build_homography_payload(
                        args, bbox, "sugarbox",
                        bridge.acg.V21_C3_GRASP_HOME,
                        {"_fallback": 0.02, "sugarbox": 0.0325},
                        {"sugarbox": 0.065},
                    )
                    self.assertIsNone(payload)
                    self.assertIn(edge, note)

    def test_calibration_only_leaves_every_homography_field_defined(self):
        """--calibration-only returns before a homography exists.

        The hulls used to be assigned only on the load-succeeded path, so this
        mode left them undefined and the safety was a property of the frame
        loop's if/elif rather than of resolve_camera_geometry. Reading one was
        an AttributeError, not a diagnosable failure.
        """
        args = geometry_args(calibration_only=True)
        bridge.resolve_camera_geometry(args)
        for field in ("homography_matrix", "homography_document",
                      "homography_pixel_hull", "homography_base_hull"):
            self.assertIsNone(getattr(args, field), field)

    def test_mapping_without_a_calibration_says_so(self):
        args = geometry_args(calibration_only=True)
        bridge.resolve_camera_geometry(args)
        with self.assertRaises(ValueError) as ctx:
            bridge.apply_grasp_home_mapping(args, 300.0, 260.0)
        self.assertIn("requires a loaded homography", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
