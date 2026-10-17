import ast
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "diagnostics"))
from run_approach_camera_demo import MotorConnection, demo_environment, handoff_preview, near_pose


class CameraDemoTests(unittest.TestCase):
    def test_flags_preserve_improved_ppo_and_request_camera_only_handoff(self):
        env = demo_environment("10.29.101.252")
        self.assertEqual(env["SUGARBOX_TRAINING_CONTRACT"], "1")
        self.assertEqual(env["SUGARBOX_TARGET_INITIAL_ONLY"], "1")
        self.assertEqual(env["SUGARBOX_PPO_ONLY"], "0")
        self.assertEqual(env["SUGARBOX_ARM_HANDOFF"], "1")
        self.assertEqual(env["SUGARBOX_REAR_HANDOFF_V"], "470")

    def test_invalid_host_refused(self):
        with self.assertRaises(ValueError):
            demo_environment("not-an-ip")

    def test_cruise_connection_rejects_all_pose_switches_before_sending(self):
        motor = MotorConnection.__new__(MotorConnection)
        motor.sock = Mock()
        for action in ("arm_view", "arm_observe", "arm_oem_step", "grasp", "velocity"):
            with self.assertRaises(ValueError):
                motor.request(action)
        motor.sock.sendall.assert_not_called()

    def test_only_finite_cruise_feedback_is_accepted(self):
        pose = (90, 140, 0, 0, 90, 30)
        self.assertTrue(near_pose(list(pose), pose))
        for angles in ([90, 180, 0, 0, 90, 30], [90, 120, 0, 0, 90, 30],
                       [90, float("nan"), 0, 0, 90, 30], [90, True, 0, 0, 90, 30]):
            self.assertFalse(near_pose(angles, pose))

    def test_explicit_handoff_previews_once(self):
        preview = Mock()
        self.assertTrue(handoff_preview(42, preview))
        preview.assert_called_once_with()

    def test_exit_error_and_timeout_never_move_arm(self):
        for code, expired in ((0, False), (1, False), (None, False), (42, True)):
            with self.subTest(code=code, expired=expired):
                preview = Mock()
                self.assertFalse(handoff_preview(code, preview, expired))
                preview.assert_not_called()

    def test_demo_has_no_grasp_or_nonzero_velocity_api(self):
        tree = ast.parse((ROOT / "diagnostics/run_approach_camera_demo.py").read_text())
        forbidden = {"GraspTask", "graspctl", "run_pixel_stage", "velocity"}
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                name = (node.func.attr if isinstance(node.func, ast.Attribute)
                        else node.func.id if isinstance(node.func, ast.Name) else "")
                self.assertNotIn(name, forbidden)


if __name__ == "__main__":
    unittest.main()
