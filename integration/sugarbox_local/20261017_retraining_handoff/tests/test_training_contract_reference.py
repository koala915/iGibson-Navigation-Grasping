import math
import ast
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "integration"))
from sugarbox_training_contract import (
    TrainingActionReference, center_relative_goal, center_relative_hit, center_relative_point,
    limited_reference_twist, planar_speed, scan_to_center_rays, training_observation,
)
from types import SimpleNamespace


class TrainingContractReferenceTests(unittest.TestCase):
    def test_speed_includes_lateral_and_reverse_magnitude(self):
        self.assertAlmostEqual(planar_speed(-0.03, 0.04), 0.05)

    def test_center_translation_is_not_a_universal_range_addition(self):
        distance, angle = center_relative_hit(0.2, math.pi / 2, 0.16, 0)
        self.assertAlmostEqual(distance, math.hypot(0.2, 0.16))
        self.assertAlmostEqual(angle, math.atan2(0.2, 0.16))
        self.assertEqual(center_relative_point(1, 0.1, 0.16, 0), (1.16, 0.1))

    def test_reported_lidar_mount_uses_nine_centimeters_not_half_chassis_length(self):
        distance, angle = center_relative_hit(0.2, 0)
        self.assertAlmostEqual(distance, 0.29)
        self.assertAlmostEqual(angle, 0)
        distance, angle = center_relative_hit(0.2, math.pi / 2)
        self.assertAlmostEqual(distance, math.hypot(0.2, 0.09))
        self.assertAlmostEqual(angle, math.atan2(0.2, 0.09))

    def test_goal_and_lidar_origins_are_not_interchanged(self):
        x, y = center_relative_goal(0.8, 0.2)
        self.assertAlmostEqual(x, 0.935)
        self.assertAlmostEqual(y, 0.2)
        self.assertAlmostEqual(center_relative_hit(0.8, 0)[0], 0.89)

    def test_center_scan_preserves_raw_ranges_and_excludes_rear_hits(self):
        angles = np.linspace(-math.pi, math.pi, 2031)
        body = np.arctan2(np.sin(math.pi - angles), np.cos(math.pi - angles))
        raw = np.full(angles.shape, 16.0)
        raw[np.abs(body) < math.radians(25)] = 0.5
        raw[np.abs(body) > math.radians(150)] = 0.05
        scan = SimpleNamespace(ranges=raw, angle_min=float(angles[0]),
                               angle_increment=float(angles[1] - angles[0]), range_min=0.01, range_max=16)
        before = raw.copy()
        rays = scan_to_center_rays(scan, True)
        self.assertEqual(rays.shape, (48,))
        self.assertGreater(float(rays.min()), 0.55)
        self.assertAlmostEqual(float(rays[23]), 0.59, delta=0.002)
        self.assertEqual(float(rays[0]), 4.0)
        np.testing.assert_array_equal(raw, before)

    def test_observation_is_nonnegative_speed_and_does_not_mutate_rays(self):
        rays = np.full(48, 0.12)
        obs = training_observation(1, 0, -0.03, 0.04, -0.2, [0.8, -0.6], rays)
        self.assertEqual(obs.shape, (55,))
        self.assertAlmostEqual(float(obs[3]), 0.05)
        np.testing.assert_allclose(obs[5:7], [0.8, -0.6])
        np.testing.assert_allclose(obs[7:], 0.33)
        np.testing.assert_allclose(rays, 0.12)

    def test_policy_distance_scale_does_not_modify_shaping_input_or_goal(self):
        rays = np.full(48, 0.5)
        rays[-1] = 4.0
        obs = training_observation(1, 0, 0, 0, 0, [0, 0], rays, distance_scale=1.05)
        self.assertAlmostEqual(float(obs[7]), 0.525)
        self.assertEqual(float(obs[-1]), 4.0)
        self.assertEqual(float(obs[0]), 1.0)
        self.assertEqual(float(rays.min()), 0.5)
        ref = TrainingActionReference()
        for _ in range(3):
            result = self.step(ref, [1, 1], lidar=float(rays.min()))
        self.assertAlmostEqual(result["linear_scale"], 0.24 + 0.76 * 0.5 / 0.66)

    def test_unit_policy_distance_scale_preserves_center_ranges(self):
        rays = np.linspace(0.33, 4.0, 48)
        obs = training_observation(1, 0, 0, 0, 0, [0, 0], rays, distance_scale=1.0)
        np.testing.assert_allclose(obs[7:], rays)

    def test_invalid_policy_distance_scale_is_rejected(self):
        for scale in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                training_observation(1, 0, 0, 0, 0, [0, 0], np.full(48, 4), distance_scale=scale)

    def step(self, ref, action, distance=0.8, lidar=4, speed=0.1):
        return ref.step(action, lidar, distance, 0, speed, 0)

    def test_action_is_delayed_two_ticks_and_reset_clears_queue(self):
        ref = TrainingActionReference()
        for _ in range(2):
            result = self.step(ref, [0.8, 0.5])
            self.assertEqual((result["vx"], result["wz"]), (0, 0))
        result = self.step(ref, [-0.2, -0.4])
        self.assertAlmostEqual(result["vx"], 0.4)
        self.assertAlmostEqual(result["wz"], 0.6)
        ref.reset()
        self.assertEqual(self.step(ref, [1, 1])["vx"], 0)

    def test_stall_assist_can_act_during_initial_delayed_zero_commands(self):
        ref = TrainingActionReference()
        result = self.step(ref, [0, 0], distance=1.0, speed=0)
        self.assertTrue(result["stall_assist"])
        self.assertAlmostEqual(result["vx"], 0.14)

    def test_zero_delay_uses_current_action_without_sign_lag(self):
        ref = TrainingActionReference(delay_steps=0)
        for action in ([0.8, 0.5], [-0.6, -0.4], [0.2, -0.7]):
            result = self.step(ref, action)
            np.testing.assert_allclose(result["delayed_action"], action)
            self.assertAlmostEqual(result["vx"], action[0] * (0.5 if action[0] >= 0 else 0.15))
            self.assertAlmostEqual(result["wz"], action[1] * 1.2)

    def test_zero_delay_reset_does_not_insert_initial_zero_ticks(self):
        ref = TrainingActionReference(delay_steps=0)
        self.step(ref, [-1, -1])
        ref.reset()
        result = self.step(ref, [1, 1])
        self.assertAlmostEqual(result["vx"], 0.5)
        self.assertAlmostEqual(result["wz"], 1.2)

    def test_zero_delay_retains_near_obstacle_shaping_and_assist(self):
        ref = TrainingActionReference(delay_steps=0)
        result = self.step(ref, [1, 1], lidar=0.33)
        self.assertAlmostEqual(result["vx"], 0.31)
        self.assertAlmostEqual(result["wz"], 1.14)
        result = self.step(ref, [0, 0], distance=1.0, speed=0)
        self.assertTrue(result["stall_assist"])
        self.assertAlmostEqual(result["vx"], 0.14)

    def test_invalid_delay_is_rejected(self):
        for delay in (-1, 0.5, True, None):
            with self.assertRaises(ValueError):
                TrainingActionReference(delay_steps=delay)

    def test_live_contract_uses_zero_delay_but_reference_default_is_two(self):
        source = Path(__file__).resolve().parents[1] / "integration" / "sugarbox_rl_approach_final2.py"
        tree = ast.parse(source.read_text(encoding="utf-8-sig"))
        assignment = next(n for n in tree.body if isinstance(n, ast.Assign) and
                          any(isinstance(t, ast.Name) and t.id == "CONTRACT_ACTION_DELAY_STEPS"
                              for t in n.targets))
        self.assertEqual(ast.literal_eval(assignment.value), 0)
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
        constructor = next(n for n in ast.walk(main) if isinstance(n, ast.Call) and
                           isinstance(n.func, ast.Attribute) and n.func.attr == "TrainingActionReference")
        flag = next(k.value for k in constructor.keywords if k.arg == "delay_steps")
        self.assertIsInstance(flag, ast.Name)
        self.assertEqual(flag.id, "CONTRACT_ACTION_DELAY_STEPS")
        self.assertEqual(TrainingActionReference().delay_steps, 2)

    def test_near_obstacle_training_scaling_is_not_an_arbitrary_stop(self):
        ref = TrainingActionReference()
        for _ in range(3):
            result = self.step(ref, [1, 1], lidar=0.33)
        self.assertAlmostEqual(result["linear_scale"], 0.62)
        self.assertAlmostEqual(result["angular_scale"], 0.95)
        self.assertAlmostEqual(result["vx"], 0.31)
        self.assertAlmostEqual(result["wz"], 1.14)

    def test_reverse_uses_reverse_scale_not_forward_scale(self):
        ref = TrainingActionReference()
        for _ in range(3):
            result = self.step(ref, [-1, -0.5])
        self.assertAlmostEqual(result["vx"], -0.15)
        self.assertAlmostEqual(result["wz"], -0.6)

    def test_proportional_limit_preserves_commanded_curvature(self):
        vx, wz = limited_reference_twist(0.5, 1.2)
        self.assertAlmostEqual(vx, 0.15)
        self.assertAlmostEqual(wz, 0.36)
        self.assertAlmostEqual(wz / vx, 1.2 / 0.5)
        vx, wz = limited_reference_twist(-0.15, -1.2)
        self.assertAlmostEqual(vx, -0.08)
        self.assertAlmostEqual(wz / vx, 8)

    def test_invalid_observation_and_action_are_rejected_without_advancing_delay(self):
        ref = TrainingActionReference()
        with self.assertRaises(ValueError):
            self.step(ref, [float("nan"), 0])
        self.assertEqual(self.step(ref, [1, 1])["vx"], 0)
        with self.assertRaises(ValueError):
            training_observation(1, 0, 0, 0, 0, [0, 0], np.full(48, float("nan")))
        with self.assertRaises(ValueError):
            limited_reference_twist(1, 1, yaw_limit=0)

    def test_contract_drive_is_rejected_before_any_hardware_initialization(self):
        source = Path(__file__).resolve().parents[1] / "integration" / "sugarbox_rl_approach_final2.py"
        tree = ast.parse(source.read_text(encoding="utf-8-sig"))
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
        namespace = {"TRAINING_CONTRACT_DIAGNOSTIC": True, "RUN_MODE": "DRIVE",
                     "CONTRACT_DRIVE_AUTHORIZED": False}
        exec(compile(ast.Module(body=[main], type_ignores=[]), str(source), "exec"), namespace)
        with self.assertRaisesRegex(ValueError, "requires explicit"):
            namespace["main"]()

    def test_contract_drive_gate_accepts_explicit_authorization(self):
        source = Path(__file__).resolve().parents[1] / "integration" / "sugarbox_rl_approach_final2.py"
        tree = ast.parse(source.read_text(encoding="utf-8-sig"))
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
        # Test only the early authorization gate, never hardware initialization.
        main.body = main.body[:1]
        namespace = {"TRAINING_CONTRACT_DIAGNOSTIC": True, "RUN_MODE": "DRIVE",
                     "CONTRACT_DRIVE_AUTHORIZED": True}
        exec(compile(ast.Module(body=[main], type_ignores=[]), str(source), "exec"), namespace)
        self.assertIsNone(namespace["main"]())

    def test_existing_modes_are_default_and_contract_mode_is_opt_in(self):
        source = Path(__file__).resolve().parents[1] / "integration" / "sugarbox_rl_approach_final2.py"
        tree = ast.parse(source.read_text(encoding="utf-8-sig"))
        names = {"TRAINING_CONTRACT_DIAGNOSTIC", "PPO_ONLY_DIAGNOSTIC", "PPO_LIDAR_DISTANCE_SCALE"}
        nodes = [n for n in tree.body if isinstance(n, ast.Assign) and
                 any(isinstance(t, ast.Name) and t.id in names for t in n.targets)]
        module = compile(ast.Module(body=nodes, type_ignores=[]), str(source), "exec")
        with patch.dict(os.environ, {"SUGARBOX_TRAINING_CONTRACT": "0", "SUGARBOX_PPO_ONLY": "0"}):
            namespace = {"os": os}
            exec(module, namespace)
            self.assertFalse(namespace["PPO_ONLY_DIAGNOSTIC"])
            self.assertEqual(namespace["PPO_LIDAR_DISTANCE_SCALE"], 1.15)
        with patch.dict(os.environ, {"SUGARBOX_TRAINING_CONTRACT": "1", "SUGARBOX_PPO_ONLY": "0"}):
            namespace = {"os": os}
            exec(module, namespace)
            self.assertTrue(namespace["PPO_ONLY_DIAGNOSTIC"])
            self.assertEqual(namespace["PPO_LIDAR_DISTANCE_SCALE"], 1.0)


if __name__ == "__main__":
    unittest.main()
