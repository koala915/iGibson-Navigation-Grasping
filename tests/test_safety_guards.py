"""Cross-cutting safety guards: grasp + navigation + cameras in one place.

Scope note: this suite deliberately reaches ACROSS subsystems. The per-subsystem
suites (grasp/v21/test_deploy_controller.py, test_servo_read.py,
test_deploy_floor_guard.py) go deep on the grasp stack; this one checks the seams
where a change in one component silently breaks another.

The grasp module is obtained through vision_grasp_pipeline._load_grasp_module(),
i.e. exactly the way modes A and C obtain it. Importing grasp.x3plus_real_grasp
directly (as this file did before 2026-08-01) tested the v17 stack that the
pipelines no longer load — the tests passed while the shipping path went
unexercised.

Runnable two ways:
    python x3plus/tests/test_safety_guards.py     # standalone, no pytest needed
    pytest x3plus/tests/test_safety_guards.py
"""
from __future__ import annotations

import contextlib
import importlib
import io
import math
import os
import json
import sys
import threading
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np

# Standalone bootstrap: put x3plus/ on sys.path so `integration`, `startup_device_check`
# and `stream_cam` import the same way they do under pytest's rootdir handling.
_X3PLUS = Path(__file__).resolve().parent.parent
if str(_X3PLUS) not in sys.path:
    sys.path.insert(0, str(_X3PLUS))

from integration import arm_cam_geometry as acg
from integration.nav_rl import (
    ActionDelay,
    GoalTracker,
    NavRLConfig,
    RosLaserScanSource,
    describe_scan,
    laser_scan_to_points,
    make_lidar,
    scan_to_rays,
    validate_orientation_evidence,
)
from integration import vision_grasp_pipeline as vgp
from integration import nav_rl_grasp_pipeline as nrgp
from integration import ros_io
from startup_device_check import camera_identity
from stream_cam import CameraState

# The grasp stack under test is whatever the pipelines actually load.
_g = vgp._load_grasp_module()
DeployConfig = _g.DeployConfig
DetectionReceiver = _g.DetectionReceiver
GraspController = _g.GraspController
JointMapper = _g.JointMapper
ServoController = _g.ServoController


class _FailingServoDevice:
    def set_uart_servo_angle_array(self, **_kwargs):
        raise OSError("simulated UART failure")


class _ClosedCapture:
    def isOpened(self):
        return False

    def release(self):
        pass


class _StaleDetection:
    """Never produces a fresh detection — the latch must not accept its default."""

    def snapshot(self):
        return np.array([0.25, 0.0, 0.02], dtype=np.float32), None, False, None


class GraspStackIdentityTests(unittest.TestCase):
    def test_pipelines_load_the_v21_grasp_stack(self):
        # If this fails, every other test in this file is testing the wrong stack.
        self.assertIn("v21", Path(_g.__file__).parts)

    def test_v21_contract_is_incremental(self):
        # v17 and v21 have identical 28D/6D shapes, so no shape check anywhere can
        # tell them apart. The contract name is the only discriminator.
        self.assertIn("incremental", DeployConfig().contract_name)


class SafetyGuardTests(unittest.TestCase):
    def test_jetson_host_environment_override(self):
        with mock.patch.dict(os.environ, {"X3PLUS_JETSON_HOST": "robot-test.local"}):
            importlib.reload(vgp)
            self.assertEqual(vgp.JETSON_IP, "robot-test.local")
            self.assertIn("robot-test.local:8080", vgp.URL_ARM)
            self.assertIn("robot-test.local:8080", vgp.URL_REAR)
        importlib.reload(vgp)

    def test_detection_payload_rejects_non_finite_and_bad_height(self):
        # v21's payload carries "height" (full z extent), not v17's "w" (width).
        pos, height, stamp = DetectionReceiver._parse_payload(
            b'{"x": 0.25, "y": 0.01, "z": 0.02, "height": 0.03}'
        )
        np.testing.assert_allclose(pos, [0.25, 0.01, 0.02], atol=1e-6)
        self.assertAlmostEqual(height, 0.03)
        self.assertIsNone(stamp, "no cam_pose in the payload means no stamp")
        with self.assertRaises(ValueError):
            DetectionReceiver._parse_payload(b'{"x": NaN, "y": 0, "z": 0.02}')
        with self.assertRaises(ValueError):
            DetectionReceiver._parse_payload(b'{"x": 0.25, "y": 0, "height": -0.01}')

    def test_servo_state_is_not_advanced_when_uart_write_raises(self):
        # v21 reports write failure by returning False rather than raising (v17 raised).
        # The invariant that matters is the same either way: a command that never left
        # must not advance _last_deg, because the rate limiter walks from _last_deg and
        # would otherwise start stepping from a pose the arm never took.
        controller = ServoController(DeployConfig(), dry_run=True)
        original = controller._last_deg.copy()
        controller.dry_run = False
        controller._has_servo = True
        controller._device = _FailingServoDevice()
        self.assertIs(controller.send_degrees([93, 137, 3, 3, 93, 33], run_time_ms=100),
                      False)
        self.assertEqual(controller._last_deg, original)
        controller._device = None

    def test_servo_write_without_a_board_is_refused_not_faked(self):
        # --real with no driver used to print angles and read exactly like success.
        controller = ServoController(DeployConfig(), dry_run=True)
        original = controller._last_deg.copy()
        controller.dry_run = False
        controller._has_servo = False
        self.assertIs(controller.send_degrees([93, 137, 3, 3, 93, 33], run_time_ms=100),
                      False)
        self.assertEqual(controller._last_deg, original)

    def test_emergency_stop_holds_instead_of_moving_home(self):
        controller = ServoController(DeployConfig(), dry_run=True)
        current = [100.0, 120.0, 20.0, 30.0, 80.0, 60.0]
        controller._last_deg = current.copy()
        controller.emergency_stop()
        self.assertEqual(controller._last_deg, current)

    def test_joint_mapper_rejects_nan_policy_action(self):
        mapper = JointMapper(DeployConfig())
        action = np.zeros(6, dtype=np.float32)
        action[2] = math.nan
        # Under the incremental contract the current pose is required to decode the
        # delta, so supply one; the NaN must still be rejected rather than added to it.
        current = np.zeros(5, dtype=np.float32)
        with self.assertRaises(ValueError):
            mapper.norm_action_to_sim_angles(action, current)

    def test_incremental_contract_refuses_to_decode_without_a_current_pose(self):
        # Decoding an incremental action as if it were absolute would command the arm
        # across its full range on every step, so this must fail rather than assume.
        mapper = JointMapper(DeployConfig())
        with self.assertRaises(Exception):
            mapper.norm_action_to_sim_angles(np.zeros(6, dtype=np.float32))

    def test_negative_action_delay_is_rejected(self):
        with self.assertRaises(ValueError):
            ActionDelay(-1)

    def test_goal_tracker_fix_age_honours_explicit_zero_time(self):
        tracker = GoalTracker()
        tracker._last_fix = -2.0
        self.assertEqual(tracker.fix_age(now=0.0), 2.0)

    def test_ros_laserscan_conversion_filters_invalid_ranges(self):
        points = laser_scan_to_points({
            "angle_min": -math.pi / 2,
            "angle_increment": math.pi / 4,
            "range_min": 0.1,
            "range_max": 10.0,
            "ranges": [1.0, math.inf, math.nan, 0.05, 2.0],
        })
        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[0][0], -90.0)
        self.assertAlmostEqual(points[1][0], 90.0)
        self.assertEqual([p[1] for p in points], [1.0, 2.0])
        with self.assertRaises(ValueError):
            laser_scan_to_points({
                "angle_min": 0.0,
                "angle_increment": 0.0,
                "range_min": 0.1,
                "range_max": 10.0,
                "ranges": [1.0],
            })

    def test_all_nan_scan_is_rejected_but_all_positive_inf_is_valid(self):
        base = {
            "angle_min": -math.pi,
            "angle_increment": 0.01,
            "range_min": 0.05,
            "range_max": 12.0,
        }
        with self.assertRaisesRegex(ValueError, "no usable ranges"):
            laser_scan_to_points(dict(base, ranges=[math.nan] * 20))
        self.assertEqual(laser_scan_to_points(dict(base, ranges=[math.inf] * 20)), [])

    def test_amcl_rejects_wrong_frame_bad_quaternion_and_negative_covariance(self):
        q = ros_io.yaw_to_quaternion(0.2)
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[35] = 0.01
        msg = {
            "header": {"frame_id": "map", "stamp": {"secs": 1, "nsecs": 0}},
            "pose": {"pose": {
                "position": {"x": 1.0, "y": 2.0},
                "orientation": dict(zip(("x", "y", "z", "w"), q)),
            }, "covariance": cov},
        }
        ros_io.parse_amcl_pose(msg)
        wrong_frame = json.loads(json.dumps(msg))
        wrong_frame["header"]["frame_id"] = "odom"
        with self.assertRaisesRegex(ValueError, "frame"):
            ros_io.parse_amcl_pose(wrong_frame)
        zero_q = json.loads(json.dumps(msg))
        zero_q["pose"]["pose"]["orientation"] = {k: 0.0 for k in ("x", "y", "z", "w")}
        with self.assertRaisesRegex(ValueError, "quaternion"):
            ros_io.parse_amcl_pose(zero_q)
        negative_cov = json.loads(json.dumps(msg))
        negative_cov["pose"]["covariance"][35] = -0.01
        with self.assertRaisesRegex(ValueError, "covariance"):
            ros_io.parse_amcl_pose(negative_cov)

    def test_fine_alignment_speed_never_exceeds_declared_si_limits(self):
        near_speed = vgp.choose_arm_forward_speed("ARM_NEAR")
        vx, _vy, _wz = vgp.action_to_vxyz("forward", near_speed)
        self.assertLessEqual(vx, vgp.ARM_NEAR_VX_MPS)
        turn_speed = vgp.speed_from_wz(vgp.ARM_MAX_WZ,
                                       vgp.ARM_MIN_TURN_SPEED,
                                       vgp.ARM_MAX_SPEED)
        _vx, _vy, wz = vgp.action_to_vxyz("turn_left", turn_speed)
        self.assertLessEqual(wz, vgp.ARM_MAX_WZ)

    def test_grasp_verification_is_unknown_without_usable_frames(self):
        nav = vgp.Navigator.__new__(vgp.Navigator)
        nav.open_cameras = lambda: None
        nav.cam_to_base_x = nav.cam_to_base_y = 0.0
        nav.sign_y = 1.0
        nav._detect_arm = lambda: (False, -1.0, 0.0, 0.0, None)
        with mock.patch.object(vgp.time, "sleep", return_value=None):
            self.assertIsNone(nav.verify_grasp([0.24, 0.0, 0.02]))

    def test_controller_close_releases_serial_even_if_another_cleanup_fails(self):
        class Part:
            def __init__(self, fail=False):
                self.closed, self.fail = False, fail
            def close(self):
                self.closed = True
                if self.fail:
                    raise RuntimeError("cleanup failed")
            def _close_device(self):
                self.closed = True

        controller = GraspController.__new__(GraspController)
        controller.detection = Part(fail=True)
        controller.servo = Part()
        controller.fk = Part()
        with contextlib.redirect_stdout(io.StringIO()):
            controller.close()
        self.assertTrue(controller.servo.closed)
        self.assertTrue(controller.fk.closed)

    def test_ros_scan_left_right_mapping_is_not_mirrored(self):
        cfg = NavRLConfig(lidar_angle_dir=1.0)
        right = scan_to_rays([(-85.0, 1.0)], cfg)
        left = scan_to_rays([(85.0, 1.0)], cfg)
        self.assertLessEqual(int(np.argmin(right)), 3)
        self.assertGreaterEqual(int(np.argmin(left)), 44)

    def test_scan_coverage_wraps_for_a_full_scan_and_is_not_orientation_proof(self):
        cfg = NavRLConfig(lidar_yaw_offset_deg=180.0)
        msg = {
            "header": {"frame_id": "laser"},
            "angle_min": -math.pi,
            "angle_increment": math.pi / 360.0,
            "ranges": [1.0] * 721,
        }
        info = describe_scan(msg, cfg)
        self.assertAlmostEqual(info["forward_coverage_deg"], 180.0)
        self.assertFalse(info["warnings"], info["warnings"])
        self.assertIn("physical meaning", describe_scan.__doc__)

    def test_scan_coverage_is_not_inverted_by_a_mirrored_lidar(self):
        """A symmetric window is unchanged by mirroring; coverage must agree.

        describe_scan walked the transformed window as ``start + raw_span``,
        which assumes the angles increase with raw index.  With
        ``lidar_angle_dir = -1`` (the rplidar backend sets this) they decrease,
        so the result came out exactly inverted: a correctly aimed +-85 deg
        scan reported 3% covered and failed the --real self-check, while the
        same scan pointing backwards reported 92% and passed.
        """
        msg = {
            "header": {"frame_id": "laser_link"},
            "angle_min": math.radians(-85.0),
            "angle_increment": math.radians(0.5),
            "ranges": [1.0] * 341,
        }
        for direction in (1.0, -1.0):
            aimed = NavRLConfig(lidar_yaw_offset_deg=0.0)
            aimed.lidar_angle_dir = direction
            info = describe_scan(msg, aimed)
            self.assertAlmostEqual(info["forward_coverage_deg"], 170.0,
                                   msg="dir=%s" % direction)
            self.assertFalse(info["warnings"], info["warnings"])

            flipped = NavRLConfig(lidar_yaw_offset_deg=180.0)
            flipped.lidar_angle_dir = direction
            info = describe_scan(msg, flipped)
            self.assertEqual(info["forward_coverage_deg"], 0.0,
                             "dir=%s must read as looking backwards" % direction)
            self.assertTrue(info["warnings"])

        # a driver publishing a descending sweep describes the same window
        descending = dict(msg, angle_min=math.radians(85.0),
                          angle_increment=math.radians(-0.5))
        self.assertAlmostEqual(
            describe_scan(descending, NavRLConfig())["forward_coverage_deg"], 170.0)

    def test_orientation_evidence_requires_result_and_matches_live_frame_and_offset(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "verified.json"
            path.write_text(json.dumps({
                "result": "PASS",
                "verified_at": "2026-08-04 12:00",
                "scan_frame": "laser",
                "policy_source_angle_offset_deg": 180,
                "evidence_log": "/tmp/four_direction.log",
            }), encoding="utf-8")
            cfg = NavRLConfig(lidar_yaw_offset_deg=180.0)
            info = {"frame_id": "laser"}
            ok, why = validate_orientation_evidence(str(path), cfg, info)
            self.assertTrue(ok, why)
            bad, why = validate_orientation_evidence(
                str(path), NavRLConfig(lidar_yaw_offset_deg=0.0), info)
            self.assertFalse(bad)
            self.assertIn("does not match", why)
            path.write_text(path.read_text(encoding="utf-8").replace(
                '"result": "PASS"', '"result": "MANUAL_INTERPRETATION_REQUIRED"'),
                encoding="utf-8")
            bad, why = validate_orientation_evidence(str(path), cfg, info)
            self.assertFalse(bad)
            self.assertIn("not PASS", why)

    def test_amcl_covariance_ok_but_local_receipt_stale_is_rejected(self):
        from integration import ros_io
        from integration.map_goal_provider import MapPose
        rio = object.__new__(ros_io.RosBridgeIO)
        rio._lock = threading.Lock()
        rio._pose = MapPose(1.0, 2.0, 0.0, 0.0)
        rio._pos_var = 0.01
        rio._yaw_var = 0.01
        rio._recv_ts = time.monotonic() - 10.0
        ok, why = rio.pose_quality_ok(max_age_s=1.0)
        self.assertFalse(ok)
        self.assertIn("stale", why)

    def test_ros_scan_source_updates_freshness_even_when_all_ranges_are_inf(self):
        source = object.__new__(RosLaserScanSource)
        source._points = []
        source._ts = 0.0
        source._ros_stamp = None
        source._lock = threading.Lock()
        source._last_error = ""
        source._raw_sample = None
        source._on_scan({
            "header": {"stamp": {"secs": 12, "nsecs": 500000000}},
            "angle_min": -1.0,
            "angle_increment": 0.1,
            "range_min": 0.01,
            "range_max": 50.0,
            "ranges": [math.inf, math.inf],
        })
        self.assertEqual(source.get_points(), [])
        self.assertTrue(math.isfinite(source.age()))
        self.assertEqual(source.ros_stamp(), 12.5)

    def test_ros_scan_source_uses_latest_only_subscription_and_closes(self):
        state = SimpleNamespace(queue_length=None, callback=None, unsubscribed=False,
                                terminated=False)

        class FakeRos:
            def __init__(self, host, port):
                self.host, self.port, self.is_connected = host, port, False

            def run(self, timeout):
                self.is_connected = True

            def terminate(self):
                state.terminated = True

        class FakeTopic:
            def __init__(self, _client, _name, _type, queue_length):
                state.queue_length = queue_length

            def subscribe(self, callback):
                state.callback = callback

            def unsubscribe(self):
                state.unsubscribed = True

        fake_roslibpy = SimpleNamespace(Ros=FakeRos, Topic=FakeTopic)
        with mock.patch.dict(sys.modules, {"roslibpy": fake_roslibpy}):
            source = RosLaserScanSource("127.0.0.1", 9090, "/scan")
            self.assertEqual(state.queue_length, 1)
            self.assertIsNotNone(state.callback)
            source.close()
        self.assertTrue(state.unsubscribed)
        self.assertTrue(state.terminated)

    def test_legacy_rplidar_backend_requires_an_explicit_port(self):
        with self.assertRaises(ValueError):
            make_lidar(NavRLConfig(), "rplidar")

    def test_camera_capture_failure_is_reported_to_main_thread(self):
        state = CameraState("missing", 640, 480, 15.0, None)
        with mock.patch.object(state, "open_capture", return_value=_ClosedCapture()):
            state.capture_loop()
        self.assertTrue(state.startup.is_set())
        self.assertIn("cannot open camera", state.capture_error)

    def test_camera_identity_groups_video_indices(self):
        first = camera_identity({"ID_PATH": "usb-port-a-video-index0"})
        second = camera_identity({"ID_PATH": "usb-port-a-video-index1"})
        self.assertEqual(first, second)
        self.assertIsNone(camera_identity({}))

    def test_hardware_smoke_modules_are_import_safe(self):
        for name in (
            "detection.debug_tools.test",
            "grasp.servo_test",
            "grasp.fk_test",
            "grasp.workspace_scan",
        ):
            importlib.import_module(name)

    def test_camera_to_base_mapping_is_explicit_and_validated(self):
        nav = vgp.Navigator(
            None,
            None,
            dry_run=True,
            cam_to_base_x=0.12,
            cam_to_base_y=-0.01,
            sign_y=-1.0,
        )
        nav._last_arm = {
            "dist": 0.22,
            "offset": 0.03,
            "box_w_px": 90,
            "class_name": None,
        }
        pos, width, height = nav.latch_arm_object()
        self.assertEqual(pos, [0.34, -0.04, 0.02])
        self.assertGreater(width, 0.0)
        self.assertIsNone(height, "no --class-height declared ⇒ no height claimed")
        with self.assertRaises(ValueError):
            vgp.Navigator(None, None, sign_y=0.0)

    def test_declared_class_height_reaches_the_latch(self):
        # The whole point of --class-height: an object whose height cannot be derived
        # from the bbox must still be able to reach the grasp side as a measurement.
        nav = vgp.Navigator(
            None, None, dry_run=True,
            class_z_m={"sugarbox": 0.0325, "_fallback": 0.02},
            class_height_m={"sugarbox": 0.065},
            cam_to_base_x=0.0, cam_to_base_y=0.0, sign_y=1.0,
        )
        nav._last_arm = {"dist": 0.25, "offset": 0.0, "box_w_px": 80,
                         "class_name": "sugarbox"}
        pos, width, height = nav.latch_arm_object()
        self.assertAlmostEqual(pos[2], 0.0325, msg="centroid z comes from --class-z")
        self.assertAlmostEqual(height, 0.065, msg="height comes from --class-height")
        # A class with no declared height must stay None, never inherit another's.
        nav._last_arm["class_name"] = "something-else"
        _, _, height2 = nav.latch_arm_object()
        self.assertIsNone(height2)

    def test_arm_cam_constants_are_tied_to_the_pose_they_were_measured_at(self):
        # The arm camera rides on arm_link4. H_ARM/THETA_ARM are mounting geometry
        # valid at one pose; the distance model returns a plausible number at ANY
        # pose, so nothing downstream can catch a mismatch. v21's home_deg is the C3
        # grasp pose, 11.7cm lower and 46.7 deg steeper than the pose these were
        # measured at -- repointing the pipelines to v21 silently moved detection
        # there, which is what this guard exists to stop.
        cal = vgp.ARM_CAM_CALIBRATED_AT_HOME_DEG
        self.assertEqual(tuple(cal), tuple(DeployConfig().home_deg),
                         "the active extrinsic set must be the pose v21 actually "
                         "starts from, or detection happens somewhere the numbers "
                         "do not describe")

        # Right pose, but the C3 extrinsics are still a URDF prediction. Under
        # --real that is its own refusal: a prediction reaching the arm without
        # anyone saying so out loud is exactly how a confident wrong grasp happens.
        self.assertFalse(vgp.ARM_CAM_POSE.fully_measured,
                         "if C3 has been measured, update this test and the registry")
        with self.assertRaises(SystemExit):
            vgp.check_arm_cam_pose(cal, real=True)
        self.assertFalse(vgp.check_arm_cam_pose(cal, real=False),
                         "dry-run warns rather than raising")
        self.assertFalse(vgp.check_arm_cam_pose(cal, real=True, acknowledged=True),
                         "acknowledgement gets past it, and still reports False")

        # A measured set at the same pose passes cleanly.
        measured = vgp.ARM_CAM_POSE.replace(theta_deg=89.6, h_m=0.215,
                                            cam_x_m=0.257, cam_y_m=-0.003)
        self.assertTrue(measured.fully_measured)
        self.assertTrue(measured.require_measured(real=True))

        # A DIFFERENT pose is refused under --real regardless of acknowledgement
        # state, and the message points at the registered pose it recognises.
        nav = vgp.acg.V17_NAV_HOME.arm_deg
        self.assertFalse(vgp.check_arm_cam_pose(nav, real=False),
                         "the v17 nav home is not the active pose")
        with self.assertRaises(SystemExit) as ctx:
            vgp.check_arm_cam_pose(nav, real=True)
        self.assertIn("v17_nav_home", str(ctx.exception),
                      "the refusal should name the pose it recognised")

        # The gripper column must not participate: the jaw opens and closes without
        # moving the camera, so S6 is irrelevant to the mounting geometry.
        same_but_jaw_closed = tuple(cal[:5]) + (180.0,)
        self.assertTrue(vgp.ARM_CAM_POSE.matches(same_but_jaw_closed),
                        "a different jaw angle is the same camera pose")
        # A single arm joint off by more than the tolerance is a different pose.
        nudged = (cal[0], cal[1] + 5.0) + tuple(cal[2:])
        self.assertFalse(vgp.ARM_CAM_POSE.matches(nudged))

    def test_class_height_parsing_rejects_nonsense_and_has_no_fallback(self):
        self.assertEqual(vgp.parse_class_height(["sugarbox=0.065"]), {"sugarbox": 0.065})
        self.assertEqual(vgp.parse_class_height([]), {},
                         "no silent _fallback: an undeclared class must send no height")
        for bad in ("sugarbox=0", "sugarbox=-0.01", "sugarbox=nan", "nope", "=0.065"):
            with self.assertRaises(ValueError, msg=bad):
                vgp.parse_class_height([bad])

    def test_real_pipeline_refuses_unconfirmed_camera_frame(self):
        args = SimpleNamespace(real=True, i_confirm_camera_frame=False)
        with self.assertRaises(SystemExit):
            vgp.run_pipeline(args)

    def test_real_integrated_grasp_refuses_legacy_nav_home_latch(self):
        with self.assertRaisesRegex(SystemExit, "grasp-home homography"):
            vgp.run_pipeline(SimpleNamespace(
                real=True,
                i_confirm_camera_frame=True,
            ))

        with self.assertRaisesRegex(SystemExit, "grasp-home homography"):
            nrgp.run_pipeline(SimpleNamespace(
                no_lidar=False,
                lidar_backend="ros",
                real=True,
                nav_only=False,
                lidar_orientation_evidence="route-b-gate.marker",
                i_confirm_camera_frame=True,
            ))

    def test_legacy_motor_calibration_blocks_nonstop_without_real_flag(self):
        module = importlib.import_module("detection.calibration.calibrate_mecanum_setmotor")
        if not module.MOTION_ALLOWED:
            with self.assertRaises(RuntimeError):
                module.send_motor_action(object(), "forward", 30)

    def test_real_socket_latch_refuses_default_position(self):
        controller = object.__new__(GraspController)
        controller.cfg = DeployConfig(latch_obj=True, latch_wait_sec=0.0)
        controller.detection = _StaleDetection()
        controller.obj_provider = None
        controller.real_servo = True
        controller._fixed_obj = np.array([0.26, 0.0, 0.02], dtype=np.float32)
        controller._fixed_height = None
        controller._obj_latched = False
        controller._latched_obj = None
        controller._latched_height = None
        with self.assertRaises(RuntimeError):
            controller._latch_object()


class PipelineGraspApiTests(unittest.TestCase):
    """The exact attributes vision_grasp_pipeline / nav_rl_grasp_pipeline call.

    DeployConfig is a plain dataclass, so a pipeline assigning a field v21 removed
    (e.g. v17's grip_width_control) does not raise — it creates a dead attribute and
    the feature vanishes silently. These assertions are what turns that into a test
    failure instead of a surprise on the robot.
    """

    def test_controller_exposes_what_the_pipelines_call(self):
        for name in ("move_home", "run", "close"):
            self.assertTrue(hasattr(GraspController, name), name)
        self.assertIn("obj_provider", GraspController.__init__.__code__.co_varnames)

    def test_removed_v17_fields_are_really_gone(self):
        import dataclasses
        fields = {f.name for f in dataclasses.fields(DeployConfig)}
        for gone in ("grip_width_control", "grip_max_object_width_m", "grip_min_close_deg"):
            self.assertNotIn(gone, fields, f"{gone} reappeared; re-check the pipelines")

    def test_move_home_and_run_return_bools(self):
        # The grasp module uses `from __future__ import annotations`, so annotations
        # are strings, not the type objects.
        self.assertEqual(GraspController.move_home.__annotations__.get("return"), "bool")
        self.assertEqual(GraspController.run.__annotations__.get("return"), "bool")

    def test_parse_deg6_csv_round_trips_and_rejects_wrong_arity(self):
        vals = _g.parse_deg6_csv("90,67.08,9.79,9.79,90,30")
        self.assertEqual(len(vals), 6)
        self.assertAlmostEqual(vals[1], 67.08)
        with self.assertRaises(Exception):
            _g.parse_deg6_csv("90,67,9")


class ArmCamGeometryTests(unittest.TestCase):
    """The near-vertical C3 pose breaks assumptions the old code baked in.

    Every one of these encodes a bug that was live on 2026-08-01 and would have
    produced a confident, wrong grasp rather than an error.
    """

    def test_ground_distance_is_signed_not_filtered(self):
        # At C3 the camera looks almost straight down, so the ground point for a
        # pixel below the principal point sits BEHIND the camera's own foot and
        # its forward distance is negative. That is a valid target, and 64% of the
        # image lands there -- the old `if dist > 0` filters discarded it and
        # reported "nothing detected" while staring at the object.
        c3 = acg.V21_C3_GRASP_HOME
        below = acg.ground_hit(acg.CX, 400.0, c3)
        self.assertLess(below.ground_dist_m, 0.0)
        self.assertGreater(below.obj_x, 0.15,
                           "a negative ground distance is still a reachable target")
        self.assertLess(below.obj_x, c3.cam_x_m)

        centre = acg.ground_hit(acg.CX, acg.CY, c3)
        self.assertAlmostEqual(centre.ground_dist_m, 0.0, delta=0.01,
                               msg="the principal ray lands at the camera's own foot")
        self.assertGreater(centre.depth_m, 0.0,
                           "depth stays positive even where ground distance is zero")

    def test_lateral_and_width_use_optical_depth_not_ground_distance(self):
        # X = (u-cx) * Z / fx, where Z is depth along the OPTICAL AXIS. Using the
        # ground distance instead is low by cos(theta+alpha): ~20-39% at the nav
        # home, and ~100% at C3, where it would report a 2.3cm box as millimetres.
        for pose in (acg.V17_NAV_HOME, acg.V21_C3_GRASP_HOME):
            hit = acg.ground_hit(acg.CX + 80.0, 300.0, pose)
            wrong = hit.ground_dist_m * 80.0 / acg.FX
            self.assertGreater(abs(hit.lateral_m), abs(wrong),
                               f"{pose.name}: depth-based lateral must exceed the "
                               f"ground-distance one")
            self.assertGreater(hit.metric_size(80.0), 0.0)
        c3 = acg.ground_hit(acg.CX + 80.0, acg.CY, acg.V21_C3_GRASP_HOME)
        self.assertGreater(c3.metric_size(90.0), 0.015,
                           "a 90px box at C3 must not measure as ~zero wide")

    def test_c3_cam_x_is_not_zero(self):
        # At C3 the ground distance spans only about +-4cm, so cam_x IS the target
        # x. The old CAM_TO_BASE_X = 0.0 put every grasp on top of the arm base.
        c3 = acg.V21_C3_GRASP_HOME
        self.assertGreater(c3.cam_x_m, 0.20)
        band = [acg.ground_hit(acg.CX, float(v), c3).obj_x for v in (0, 240, 479)]
        self.assertTrue(all(0.15 < x < 0.35 for x in band),
                        f"visible ground band must sit in the graspable range: {band}")

    def test_theta_is_measured_towards_base_x(self):
        # URDF FK reports the C3 optical axis 86.7 deg below horizontal with its
        # horizontal component pointing backward. Measured towards +X -- the
        # convention the ground model needs -- the same ray is 93.3 deg, and the
        # mounting error makes the real value ~89.7. Using 86.7 would mirror the
        # workspace about the camera.
        self.assertGreater(acg.V21_C3_GRASP_HOME.theta_deg, 87.0)
        self.assertLess(acg.V21_C3_GRASP_HOME.theta_deg, 92.0)

    def test_horizon_rays_raise_instead_of_answering(self):
        with self.assertRaises(acg.GroundGeometryError):
            acg.ground_hit(acg.CX, acg.CY, acg.V17_NAV_HOME.replace(theta_deg=0.0))
        with self.assertRaises(acg.GroundGeometryError):
            # Just off the horizon: |d| explodes, so it is refused by the band check
            # rather than handed over as a confident metre-scale number.
            acg.ground_hit(acg.CX, acg.CY, acg.V17_NAV_HOME.replace(theta_deg=1.0))

    def test_registered_extrinsics_live_in_the_policy_fk_frame(self):
        """cam_x/cam_y must match the DEPLOYMENT FK, not the raw URDF.

        x3plus_real_grasp.FKComputer loads the URDF at
        deploy_contract.URDF_TO_TRAINING_FRAME, so the policy's object
        coordinates sit 19.9 mm forward of the raw base_link frame that
        verify_camera_grasp_frame.py prints. The first version of this registry
        took cam_x from that tool and was 19.9 mm short -- most of the 22 mm
        entry tolerance, spent before the policy even starts, and invisible
        because both numbers look equally plausible.
        """
        try:
            import pybullet as p
        except ImportError:
            self.skipTest("pybullet not installed")

        cfg = DeployConfig()
        fk = _g.FKComputer(cfg.urdf_path)
        mapper = JointMapper(cfg)
        mono = None
        for i in range(p.getNumJoints(fk.body_id, physicsClientId=fk.physics_client)):
            info = p.getJointInfo(fk.body_id, i, physicsClientId=fk.physics_client)
            if info[12].decode() == "mono_link":
                mono = i
        self.assertIsNotNone(mono, "mono_link not found in the URDF")

        for pose in (acg.V17_NAV_HOME, acg.V21_C3_GRASP_HOME):
            for name, val in zip(["arm_joint1", "arm_joint2", "arm_joint3",
                                  "arm_joint4", "arm_joint5"],
                                 mapper.hw_deg_to_sim_arm(list(pose.arm_deg[:5]))):
                jid = fk.name2id.get(name)
                if jid is not None:
                    p.resetJointState(fk.body_id, jid, float(val),
                                      physicsClientId=fk.physics_client)
            state = p.getLinkState(fk.body_id, mono, computeForwardKinematics=True,
                                   physicsClientId=fk.physics_client)
            fk_x, fk_y, _fk_z = state[4]
            self.assertAlmostEqual(
                pose.cam_x_m, fk_x, places=3,
                msg=f"{pose.name}: cam_x {pose.cam_x_m} is not the policy-frame FK "
                    f"value {fk_x:.4f}. Did it come from verify_camera_grasp_frame.py, "
                    f"which prints the raw base_link frame?")
            self.assertAlmostEqual(
                pose.cam_y_m, fk_y, places=3,
                msg=f"{pose.name}: cam_y {pose.cam_y_m} is not the policy-frame FK "
                    f"value {fk_y:.4f}")

            # theta is a direction, so the frame shift must not touch it. It should
            # be the FK direction minus the mounting error solved at the nav home.
            rot = p.getMatrixFromQuaternion(state[5])
            fk_theta = math.degrees(math.atan2(-rot[8], rot[2]))
            mount_err = acg.V17_NAV_HOME.theta_deg - 40.0
            self.assertAlmostEqual(
                pose.theta_deg, fk_theta + mount_err, places=1,
                msg=f"{pose.name}: theta {pose.theta_deg} is not the FK direction "
                    f"{fk_theta:.3f} plus the {mount_err:+.3f} deg mounting error")

    def test_predicted_extrinsics_cannot_reach_the_arm_unacknowledged(self):
        c3 = acg.V21_C3_GRASP_HOME
        self.assertFalse(c3.fully_measured)
        with self.assertRaises(SystemExit):
            c3.require_measured(real=True)
        self.assertFalse(c3.require_measured(real=False))
        self.assertFalse(c3.require_measured(real=True, acknowledged=True))
        self.assertTrue(acg.V17_NAV_HOME.replace(cam_x_m=0.16, cam_y_m=0.0)
                        .require_measured(real=True))


class ReachEnvelopeTests(unittest.TestCase):
    """The camera can see further than the policy was trained to reach.

    Everything else in this file guards against a WRONG coordinate. This guards
    against a RIGHT one that names a spot the policy has never been evaluated on
    — which a perfectly healthy detector produces the moment the object sits near
    the edge of the frame.
    """

    def _controller(self, *, real):
        g = object.__new__(GraspController)
        g.cfg = DeployConfig()
        g.real_servo = real
        return g

    def test_targets_outside_the_evaluated_region_are_refused(self):
        g = self._controller(real=True)
        for pos, why in [((0.18, 0.0, 0.02), "too near"),
                         ((0.35, 0.0, 0.02), "too far"),
                         ((0.26, 0.12, 0.02), "too far left"),
                         ((0.26, -0.12, 0.02), "too far right")]:
            with mock.patch("sys.stdout"):
                self.assertFalse(g._check_target_envelope(np.array(pos, dtype=np.float32)),
                                 why)

    def test_targets_inside_the_evaluated_region_are_accepted(self):
        g = self._controller(real=True)
        for pos in ((0.26, 0.0, 0.02), (0.20, 0.0, 0.02), (0.33, 0.0, 0.02),
                    (0.26, 0.10, 0.02), (0.26, -0.10, 0.02)):
            with mock.patch("sys.stdout"):
                self.assertTrue(g._check_target_envelope(np.array(pos, dtype=np.float32)),
                                str(pos))

    def test_dry_run_warns_where_real_refuses(self):
        # The desk must stay able to exercise out-of-range paths.
        g = self._controller(real=False)
        with mock.patch("sys.stdout"):
            self.assertTrue(g._check_target_envelope(np.array([0.18, 0.0, 0.02],
                                                              dtype=np.float32)))

    def test_the_documented_range_only_warns(self):
        # x=0.30 is inside the evaluated region but past what manifest.json calls
        # valid: worth saying, not worth refusing.
        g = self._controller(real=True)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            ok = g._check_target_envelope(np.array([0.30, 0.0, 0.02], dtype=np.float32))
        self.assertTrue(ok)
        self.assertIn("beyond the documented", buf.getvalue())

    def test_the_bridge_drops_out_of_reach_detections_at_source(self):
        from integration import vision_grasp_bridge as vgb
        self.assertEqual(tuple(vgb.TRAINED_X_RANGE), tuple(DeployConfig().trained_x_range),
                         "the bridge and the grasp side must agree on the envelope")
        self.assertEqual(tuple(vgb.TRAINED_Y_RANGE), tuple(DeployConfig().trained_y_range))

        # A pose whose cam_x is 3cm short pushes bottom-of-frame detections below
        # the trained region — exactly what a mis-calibration would do.
        short = acg.V21_C3_GRASP_HOME.replace(cam_x_m=acg.V21_C3_GRASP_HOME.cam_x_m - 0.03)
        payload, note = vgb.build_payload((190.0, 430.0, 235.0, 470.0), "sugarbox",
                                          short, {"_fallback": 0.02}, {"sugarbox": 0.065})
        self.assertIsNone(payload)
        self.assertIn("outside the policy's evaluated range", note)

    def test_the_visible_band_is_checked_against_the_envelope_it_overhangs(self):
        # The reason both gates exist. If the camera's footprint ever moves well
        # inside the trained region this test says so, and the gates become
        # belt-and-braces rather than load-bearing.
        c3 = acg.V21_C3_GRASP_HOME
        xs = [acg.ground_hit_from_raw(float(u), float(v), c3).obj_x
              for v in range(0, 480, 20) for u in (0.0, 211.0, 423.0)]
        lo, hi = DeployConfig().trained_x_range
        self.assertLess(min(xs), lo + 0.005,
                        "the camera no longer reaches the near edge of the trained "
                        "region; re-check whether the envelope gate is still needed")
        self.assertLessEqual(max(xs), hi,
                             "the camera now sees past the far edge of the trained "
                             "region too — the gate must stay")


class CalibrationCaptureTests(unittest.TestCase):
    """capture_arm_cam_obs's pure logic. It imports without cv2/ultralytics."""

    def setUp(self):
        from integration import capture_arm_cam_obs as cap
        self.cap = cap

    def test_coordinate_parsing_rejects_nonsense(self):
        self.assertEqual(self.cap.parse_xy("0.247 0.018"), (0.247, 0.018))
        self.assertEqual(self.cap.parse_xy("0.247,0.018"), (0.247, 0.018))
        for bad in ("0.247", "a b", "", "1 2 3", "nan 0.0"):
            self.assertIsNone(self.cap.parse_xy(bad), bad)

    def test_coverage_report_catches_a_fit_that_cannot_constrain_the_solve(self):
        # All on one spot: theta/cam_x are degenerate and sign_y is undetermined.
        clustered = [{"base_x": 0.25, "base_y": 0.0, "u": acg.CX, "v": 240}
                     for _ in range(6)]
        problems = " ".join(self.cap.coverage_report(clustered))
        self.assertIn("forward span", problems)
        self.assertIn("off-centre", problems)

        # Spread out properly: no complaints.
        spread = [
            {"base_x": 0.205, "base_y": 0.000, "u": 233.2, "v": 412.6},
            {"base_x": 0.230, "base_y": -0.040, "u": 64.0, "v": 306.5},
            {"base_x": 0.255, "base_y": 0.000, "u": 233.7, "v": 200.4},
            {"base_x": 0.280, "base_y": 0.040, "u": 403.0, "v": 95.3},
            {"base_x": 0.300, "base_y": 0.000, "u": 232.9, "v": 11.0},
            {"base_x": 0.240, "base_y": 0.035, "u": 381.4, "v": 264.4},
        ]
        self.assertEqual(self.cap.coverage_report(spread), [])

    def test_placements_past_the_lateral_field_of_view_are_flagged(self):
        # The usable lateral range at C3 is about -5.9 to +10.7 cm and is
        # ASYMMETRIC, so -6 cm is off the edge while +6 cm is comfortably inside.
        # Worth saying before the operator drives out to the robot and wonders
        # why one side records and the other does not.
        window = acg.usable_placement_window(acg.V21_C3_GRASP_HOME, 0.065)
        outside = [
            {"base_x": 0.21, "base_y": -0.065, "u": 5.0, "v": 400},
            {"base_x": 0.26, "base_y": 0.000, "u": 233.0, "v": 200},
            {"base_x": 0.29, "base_y": 0.040, "u": 500.0, "v": 20},
            {"base_x": 0.25, "base_y": 0.030, "u": 360.0, "v": 260},
            {"base_x": 0.27, "base_y": -0.010, "u": 200.0, "v": 120},
        ]
        self.assertTrue(any("outside the window this object fits in" in p
                            for p in self.cap.coverage_report(outside, window)),
                        self.cap.coverage_report(outside, window))

        inside = [
            {"base_x": 0.242, "base_y": -0.020, "u": 120.0, "v": 400},
            {"base_x": 0.255, "base_y": 0.000, "u": 233.0, "v": 300},
            {"base_x": 0.270, "base_y": 0.045, "u": 480.0, "v": 150},
            {"base_x": 0.288, "base_y": 0.020, "u": 360.0, "v": 40},
            {"base_x": 0.262, "base_y": -0.025, "u": 100.0, "v": 200},
        ]
        self.assertFalse(any("outside the window this object fits in" in p
                             for p in self.cap.coverage_report(inside, window)),
                         self.cap.coverage_report(inside, window))

    def test_the_c3_ground_footprint_is_asymmetric_about_the_optical_axis(self):
        # The principal point is at x=212 in a 640-wide frame, not 320, so u - cx
        # spans -212..+427 px and the camera sees roughly twice as far to one side
        # as the other. Reading "cx is 212, so the frame must be ~424 wide" gets
        # the lateral reach wrong by more than a factor of two -- which this file
        # did until 2026-08-02, and which produced calibration advice telling the
        # operator to keep placements inside +-5 cm when one side reaches +10.7.
        self.assertEqual((acg.IMG_W, acg.IMG_H), (640, 480))
        c3 = acg.V21_C3_GRASP_HOME
        # Include the true frame corners: the extremes live there, and a coarse
        # stride walks straight past them.
        us = list(range(0, acg.IMG_W, 40)) + [acg.IMG_W - 1]
        vs = list(range(0, acg.IMG_H, 40)) + [acg.IMG_H - 1]
        ys = [acg.ground_hit_from_raw(float(u), float(v), c3).obj_y
              for v in vs for u in us]
        self.assertLess(min(ys), -0.055, "near-side lateral reach moved")
        self.assertGreater(max(ys), 0.100, "far-side lateral reach moved")
        self.assertGreater(abs(max(ys)), abs(min(ys)) * 1.5,
                           "the footprint should be markedly asymmetric")

    def test_a_frame_that_is_not_the_calibrated_size_is_refused(self):
        # Intrinsics are meaningless away from the size they were solved at, and
        # this robot's cameras swap /dev/video indices between boots, so reading
        # the rear camera through the arm pipeline is a live hazard.
        self.assertIsNone(acg.frame_size_mismatch(640, 480))
        for w, h in ((424, 336), (1280, 960), (640, 360), (320, 240)):
            msg = acg.frame_size_mismatch(w, h)
            self.assertIsNotNone(msg, f"{w}x{h} should be refused")
            self.assertIn("calibrated at 640x480", msg)

    def test_too_few_placements_warn_even_though_the_residuals_look_fine(self):
        # The trap this warning exists for: at three placements the fit's median
        # error is 1.4 mm but its p95 is 33 mm, and the residuals are just as
        # small in the bad case as the good one. A calibration that "fits
        # perfectly" is not evidence of anything at this count.
        from integration import solve_arm_cam_extrinsics as S
        self.assertGreaterEqual(S.MIN_SAFE_OBS, 5)

        c3 = acg.V21_C3_GRASP_HOME
        obs = [(h.obj_x, h.obj_y, u, v) for u, v, h in
               ((u, v, acg.ground_hit_from_raw(float(u), float(v), c3))
                for u, v in ((212, 410), (212, 240), (212, 60)))]
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            pose, _spread = S.solve(obs, c3.h_m, pose_name="thin")
        out = buf.getvalue()
        self.assertIn("WARNING", out)
        self.assertIn("p95", out, "the warning should quantify the risk, not just assert it")
        worst = max(max(abs(dx), abs(dy))
                    for _x, _y, dx, dy, _e in S.residuals(obs, pose))
        self.assertLess(worst, 0.001,
                        "noise-free data fits perfectly at three points too — which "
                        "is exactly why the residual cannot be the safety check")

    def test_holdout_split_keeps_the_last_placements_out_of_the_fit(self):
        obs = [{"base_x": 0.20 + 0.02 * i, "base_y": 0.0, "u": 212.0, "v": 400 - 50 * i}
               for i in range(6)]
        cmd, fit, held = self.cap.emit_solver_command(obs, 0.2151, "v21_c3_grasp_home", 2)
        self.assertEqual(len(fit), 4)
        self.assertEqual(len(held), 2)
        self.assertEqual(cmd.count("--obs"), 4,
                         "held-out placements must not reach the solver")
        self.assertIn("0.2151", " ".join(cmd))


class BridgePayloadTests(unittest.TestCase):
    """vision_grasp_bridge's pure functions. It imports without cv2/ultralytics."""

    def setUp(self):
        from integration import vision_grasp_bridge as vgb
        self.vgb = vgb

    def test_declared_height_sets_the_centroid_z(self):
        # Before this, --class-height sugarbox=0.065 with no --class-z shipped
        # z=0.02 (the --obj-z default) alongside height=0.065. The grasp side uses
        # BOTH, so wrist_z_offset came out 1.25cm off with nothing to flag it.
        out = self.vgb.reconcile_z_and_height({"_fallback": 0.02},
                                              {"sugarbox": 0.065})
        self.assertAlmostEqual(out["sugarbox"], 0.0325)
        self.assertAlmostEqual(out["_fallback"], 0.02,
                               msg="undeclared classes keep the fallback")

    def test_contradictory_z_and_height_are_refused(self):
        with self.assertRaises(ValueError):
            self.vgb.reconcile_z_and_height({"sugarbox": 0.02, "_fallback": 0.02},
                                            {"sugarbox": 0.065})
        # An explicit z that agrees is kept as given (asymmetric objects).
        out = self.vgb.reconcile_z_and_height({"sugarbox": 0.033, "_fallback": 0.02},
                                              {"sugarbox": 0.065})
        self.assertAlmostEqual(out["sugarbox"], 0.033)

    def test_payload_is_stamped_with_the_pose_the_grasp_side_verifies(self):
        pose = acg.V21_C3_GRASP_HOME
        payload, note = self.vgb.build_payload(
            (180.0, 200.0, 260.0, 300.0), "sugarbox", pose,
            {"sugarbox": 0.0325, "_fallback": 0.02}, {"sugarbox": 0.065})
        self.assertIsNotNone(payload, note)
        self.assertIn(acg.PAYLOAD_POSE_KEY, payload)
        self.assertEqual(payload[acg.PAYLOAD_POSE_NAME_KEY], pose.name)
        self.assertAlmostEqual(payload["height"], 0.065)
        self.assertAlmostEqual(payload["z"], 0.0325)

        # Round-trips through the grasp side's own parser, which is a separate
        # implementation (the grasp script must stay standalone on the Jetson).
        parsed_pos, parsed_h, parsed_stamp = DetectionReceiver._parse_payload(
            json.dumps(payload).encode("utf-8"))
        self.assertIsNotNone(parsed_stamp)
        self.assertEqual(parsed_stamp[1], pose.name)
        self.assertAlmostEqual(float(parsed_pos[0]), payload["x"], places=4)
        self.assertAlmostEqual(parsed_h, 0.065)

    def test_oversized_objects_are_dropped_with_a_reason(self):
        # Wide, but clear of every frame edge, so the width gate is what fires
        # rather than the clipped-bbox check.
        pose = acg.V21_C3_GRASP_HOME
        payload, note = self.vgb.build_payload(
            (20.0, 200.0, 620.0, 300.0), "sugarbox", pose,
            {"_fallback": 0.02}, {"sugarbox": 0.065}, max_width_m=0.06)
        self.assertIsNone(payload)
        self.assertIn("ungraspable", note)

    def test_a_declared_height_switches_on_the_centroid_estimator(self):
        # The bbox bottom-centre against the floor is off by -11 to -45 mm at the
        # C3 pose because the lowest pixel is often the object's TOP face. With a
        # declared height the bridge locates the silhouette centre on the object's
        # half-height plane instead, and the note says which was used.
        pose = acg.V21_C3_GRASP_HOME
        bbox = (225.0, 200.0, 315.0, 290.0)
        with_h, note_h = self.vgb.build_payload(
            bbox, "sugarbox", pose, {"sugarbox": 0.0325, "_fallback": 0.02},
            {"sugarbox": 0.065})
        without_h, note_no = self.vgb.build_payload(
            bbox, "sugarbox", pose, {"_fallback": 0.02}, {})
        self.assertIsNotNone(with_h, note_h)
        self.assertIsNotNone(without_h, note_no)
        self.assertIn("centroid", note_h)
        self.assertIn("bottom-edge", note_no)
        self.assertNotAlmostEqual(with_h["x"], without_h["x"], places=3,
                                  msg="the two estimators must not agree by accident")

    def test_reported_width_is_corrected_for_the_object_height(self):
        # The widest part of a box's silhouette is its TOP face, which is closer
        # to the camera than mid-height and so magnified more. Measuring the span
        # at mid-height depth over-reports by (H-h/2)/(H-h) -- 2.80 cm for a 2.3 cm
        # sugarbox, a constant +22%. That over-rejects at the width gate and
        # mis-sizes v17's close angle.
        import itertools
        pose = acg.V21_C3_GRASP_HOME
        th = math.radians(pose.theta_deg)

        def project(x, y, z):
            dx, dy, dz = x - pose.cam_x_m, y - pose.cam_y_m, z - pose.h_m
            xc, yc = dy * pose.sign_y, -dx*math.sin(th) - dz*math.cos(th)
            zc = dx*math.cos(th) - dz*math.sin(th)
            return acg.CX + acg.FX*xc/zc, acg.CY + acg.FY*yc/zc

        for w_true, h_true in ((0.023, 0.065), (0.030, 0.030)):
            pts = [project(0.262 + sx, sy, sz) for sx, sy, sz in
                   itertools.product((-w_true/2, w_true/2), (-w_true/2, w_true/2),
                                     (0.0, h_true))]
            us = [q[0] for q in pts]
            vs = [q[1] for q in pts]
            payload, note = self.vgb.build_payload(
                (min(us), min(vs), max(us), max(vs)), "o", pose,
                {"o": h_true/2, "_fallback": 0.02}, {"o": h_true})
            self.assertIsNotNone(payload, note)
            self.assertAlmostEqual(payload["w"], w_true, places=3,
                                   msg=f"width for a {w_true}x{h_true} box")

    def test_mode_a_and_mode_b_locate_an_object_the_same_way(self):
        """The two paths must not disagree about where an object is.

        Mode A (vision_grasp_pipeline) and mode B (vision_grasp_bridge) each turn
        a bbox into a grasp target. When the bridge moved to the silhouette-centre
        estimator and the pipeline did not, the same detection meant two different
        places -- 45 mm apart at worst. This is the same shape of divergence that
        let the 2026-08-01 arm-pose guard land in one file and not the other.
        """
        import itertools
        pose = acg.V21_C3_GRASP_HOME
        th = math.radians(pose.theta_deg)

        def project(x, y, z):
            dx, dy, dz = x - pose.cam_x_m, y - pose.cam_y_m, z - pose.h_m
            xc, yc = dy * pose.sign_y, -dx*math.sin(th) - dz*math.cos(th)
            zc = dx*math.cos(th) - dz*math.sin(th)
            return acg.CX + acg.FX*xc/zc, acg.CY + acg.FY*yc/zc

        w_true, h_true, x_true = 0.023, 0.065, 0.262
        pts = [project(x_true + sx, sy, sz) for sx, sy, sz in
               itertools.product((-w_true/2, w_true/2), (-w_true/2, w_true/2),
                                 (0.0, h_true))]
        us, vs = [q[0] for q in pts], [q[1] for q in pts]
        bbox = (min(us), min(vs), max(us), max(vs))

        from integration import vision_grasp_bridge as vgb
        payload, note = vgb.build_payload(bbox, "sugarbox", pose,
                                          {"sugarbox": h_true/2, "_fallback": 0.02},
                                          {"sugarbox": h_true})
        self.assertIsNotNone(payload, note)

        # Mode A's latch, fed the same detection the same way _detect_arm would.
        hit = acg.silhouette_centre_target(*bbox, pose, h_true)
        nav = vgp.Navigator(None, None, dry_run=True,
                            class_z_m={"sugarbox": h_true/2, "_fallback": 0.02},
                            class_height_m={"sugarbox": h_true})
        nav._last_arm = {"dist": hit.ground_dist_m, "offset": hit.lateral_m,
                         "box_w_px": bbox[2] - bbox[0], "class_name": "sugarbox",
                         "depth": hit.depth_m, "hit": hit, "obj_h": h_true}
        with mock.patch("sys.stdout"):
            pos, width, height = nav.latch_arm_object()

        self.assertAlmostEqual(pos[0], payload["x"], places=3,
                               msg="mode A and mode B disagree about x")
        self.assertAlmostEqual(pos[1], payload["y"], places=3,
                               msg="mode A and mode B disagree about y")
        self.assertAlmostEqual(width, payload["w"], places=3,
                               msg="mode A and mode B disagree about width")
        self.assertAlmostEqual(pos[0], x_true, delta=0.005,
                               msg="both should land on the object, not near it")
        self.assertAlmostEqual(width, w_true, delta=0.002)

    def test_the_usable_placement_window_shrinks_with_object_height(self):
        # A tall object seen from near vertical has a silhouette larger than its
        # footprint, so it runs off the frame edge sooner. Calibration placements
        # have to live inside THIS, not inside the camera's ground footprint.
        pose = acg.V21_C3_GRASP_HOME
        tall = acg.usable_placement_window(pose, 0.065)
        flat = acg.usable_placement_window(pose, 0.001)
        self.assertGreater(tall[0], flat[0], "near edge should pull in")
        self.assertLess(tall[1], flat[1], "far edge should pull in")
        self.assertLess(tall[1] - tall[0], flat[1] - flat[0])
        # The sugarbox window, which the calibration docs quote.
        self.assertAlmostEqual(tall[0], 0.239, places=2)
        self.assertAlmostEqual(tall[1], 0.292, places=2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
