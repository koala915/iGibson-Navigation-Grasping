#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from integration import nav_rl
from integration import nav_safety as ns


class NavigationSafetyTests(unittest.TestCase):
    def setUp(self):
        # These fixtures describe robot-frame sectors. Select their mounting
        # explicitly instead of inheriting the deployment/CLI mounting.
        self.nav = nav_rl.NavRLConfig(
            lidar_yaw_offset_deg=0.0, lidar_forward_offset_m=0.0)
        self.clear = [(float(angle), 4.0) for angle in range(-125, 126, 5)]

    def test_disabled_mode_preserves_the_existing_emergency_front_brake(self):
        safety = ns.NavigationSafety(ns.SafetyConfig(enabled=False))
        decision = safety.apply(0.2, 0.4, [(0.0, 0.20)] + self.clear, self.nav)
        self.assertEqual(decision.vx, 0.0)
        self.assertEqual(decision.wz, 0.4)
        self.assertTrue(decision.braked)

    @staticmethod
    def _sensor_points(robot_points, cfg):
        """Express robot polar fixtures in a selected physical sensor frame."""
        points = []
        for angle, distance in robot_points:
            angle_rad = math.radians(angle)
            x = distance * math.cos(angle_rad) - cfg.lidar_forward_offset_m
            y = distance * math.sin(angle_rad)
            sensor_angle = (math.degrees(math.atan2(y, x))
                            - cfg.lidar_yaw_offset_deg) / cfg.lidar_angle_dir
            points.append((sensor_angle, math.hypot(x, y)))
        return points

    def test_disabled_brake_preserves_clusters_speckles_and_emergencies(self):
        cases = (
            ("one normal-zone speckle", [(0.0, 0.30)], False),
            ("two normal-zone speckles", [(-0.2, 0.30), (0.2, 0.30)], False),
            ("three adjacent obstacle returns",
             [(-0.2, 0.30), (0.0, 0.30), (0.2, 0.30)], True),
            ("three angularly separated speckles",
             [(-5.0, 0.30), (0.0, 0.30), (5.0, 0.30)], False),
            ("three spatially separated speckles",
             [(-0.2, 0.27), (0.0, 0.34), (0.2, 0.27)], False),
            ("single emergency return", [(0.0, 0.20)], True),
            ("robot self return", [(0.0, 0.10)], False),
            ("side wall outside swept corridor",
             [(math.degrees(math.atan2(0.23, 0.30)), math.hypot(0.30, 0.23))],
             False),
            ("rear return", [(180.0, 0.30)], False),
            ("clear scan", [], False),
        )
        for yaw, offset in ((0.0, 0.0), (180.0, 0.10)):
            cfg = nav_rl.NavRLConfig(
                lidar_yaw_offset_deg=yaw, lidar_forward_offset_m=offset)
            safety = ns.NavigationSafety()  # Enhanced mode remains opt-in.
            self.assertFalse(safety.cfg.enabled)
            for label, robot_points, expected_brake in cases:
                with self.subTest(case=label, yaw=yaw, offset=offset):
                    points = self._sensor_points(robot_points, cfg)
                    existing_front = nav_rl.front_min_brake(points, cfg)
                    self.assertEqual(existing_front < cfg.safety_brake_dist,
                                     expected_brake)
                    decision = safety.apply(0.2, 0.4, points, cfg)
                    self.assertEqual(decision.braked, expected_brake)
                    self.assertEqual(decision.vx, 0.0 if expected_brake else 0.2)
                    self.assertEqual(decision.wz, 0.4)
                    self.assertEqual(decision.front.minimum, existing_front)

    def test_disabled_brake_keeps_reverse_and_turn_available(self):
        safety = ns.NavigationSafety()
        for vx in (-0.10, 0.0):
            with self.subTest(vx=vx):
                decision = safety.apply(vx, -0.4, [(0.0, 0.20)], self.nav)
                self.assertEqual((decision.vx, decision.wz), (vx, -0.4))
                self.assertFalse(decision.braked)

    def test_disabled_emergency_brake_clears_when_return_disappears(self):
        safety = ns.NavigationSafety()
        self.assertTrue(safety.apply(0.2, 0.4, [(0.0, 0.20)], self.nav).braked)
        clear = safety.apply(0.2, 0.4, self.clear, self.nav)
        self.assertEqual((clear.vx, clear.wz), (0.2, 0.4))
        self.assertFalse(clear.braked)

    def test_robust_front_distance_ignores_one_isolated_low_return(self):
        points = [(0.0, 0.175)] + [(float(a), 0.50) for a in range(-20, 21, 2)]
        cfg = ns.SafetyConfig(enabled=True)
        front, _right, _left = ns.scan_metrics(points, self.nav, cfg)
        self.assertAlmostEqual(front.minimum, 0.175)
        self.assertGreater(front.robust, 0.30)

    def test_hard_stop_uses_minimum_and_latches_until_exit(self):
        safety = ns.NavigationSafety(ns.SafetyConfig(enabled=True))
        stopped = safety.apply(0.2, 0.0, [(0.0, 0.175)] + self.clear,
                               self.nav, now=1.0)
        self.assertEqual((stopped.vx, stopped.wz), (0.0, 0.0))
        self.assertEqual(stopped.reason, "HARD_STOP_HOLD")
        held = safety.apply(0.2, 0.0, [(0.0, 0.20)] + self.clear,
                            self.nav, now=1.1)
        self.assertEqual(held.vx, 0.0)
        released = safety.apply(0.2, 0.0, [(0.0, 0.23)] + self.clear,
                                self.nav, now=1.2)
        self.assertGreaterEqual(released.vx, 0.0)
        self.assertNotEqual(released.reason, "HARD_STOP_HOLD")

    def test_side_guard_blocks_turning_toward_close_side(self):
        safety = ns.NavigationSafety(ns.SafetyConfig(enabled=True))
        points = self.clear + [(float(a), 0.15) for a in range(-90, -50, 5)]
        decision = safety.apply(0.1, -0.5, points, self.nav, now=1.0)
        self.assertEqual(decision.wz, 0.0)
        self.assertIn("SIDE_GUARD_R", decision.reason)

    def test_enhanced_front_excludes_self_returns_in_both_mountings(self):
        for yaw, offset in ((0.0, 0.0), (180.0, 0.10)):
            with self.subTest(yaw=yaw, offset=offset):
                cfg = nav_rl.NavRLConfig(
                    lidar_yaw_offset_deg=yaw, lidar_forward_offset_m=offset)
                safety = ns.NavigationSafety(ns.SafetyConfig(enabled=True))
                self_points = self._sensor_points([(0.0, 0.15)], cfg)
                decision = safety.apply(0.2, 0.4, self_points, cfg)
                self.assertEqual((decision.vx, decision.wz), (0.2, 0.4))
                self.assertEqual(decision.front.count, 0)
                self.assertFalse(decision.braked)

                # An obstacle immediately beyond the measured front plane is
                # retained, even as a single return.
                obstacle_points = self._sensor_points([(0.0, 0.175)], cfg)
                stopped = safety.apply(0.2, 0.4, obstacle_points, cfg)
                self.assertEqual((stopped.vx, stopped.wz), (0.0, 0.0))
                self.assertEqual(stopped.reason, "HARD_STOP_HOLD")
                self.assertAlmostEqual(stopped.front.minimum, 0.175)

    def test_enhanced_front_uses_forward_x_instead_of_radial_range(self):
        # At this angle the radial range exceeds the hard-stop threshold,
        # while forward x is inside it and beyond the measured front plane.
        safety = ns.NavigationSafety(ns.SafetyConfig(enabled=True))
        distance, angle = 0.19, 20.0
        forward_x = distance * math.cos(math.radians(angle))
        self.assertGreater(distance, safety.cfg.hard_stop_enter_m)
        self.assertGreater(forward_x, self.nav.robot_front_extent_m)
        self.assertLess(forward_x, safety.cfg.hard_stop_enter_m)
        stopped = safety.apply(0.2, 0.4, [(angle, distance)], self.nav)
        self.assertEqual((stopped.vx, stopped.wz), (0.0, 0.0))
        self.assertAlmostEqual(stopped.front.minimum, forward_x)

    def test_enhanced_side_guards_remain_active_in_both_mountings(self):
        for yaw, offset in ((0.0, 0.0), (180.0, 0.10)):
            for angle, turn, reason in ((-70.0, -0.4, "SIDE_GUARD_R"),
                                        (70.0, 0.4, "SIDE_GUARD_L")):
                with self.subTest(yaw=yaw, offset=offset, angle=angle):
                    cfg = nav_rl.NavRLConfig(
                        lidar_yaw_offset_deg=yaw, lidar_forward_offset_m=offset)
                    safety = ns.NavigationSafety(ns.SafetyConfig(enabled=True))
                    points = self._sensor_points([(angle, 0.15)], cfg)
                    decision = safety.apply(0.2, turn, points, cfg)
                    self.assertEqual(decision.wz, 0.0)
                    self.assertIn(reason, decision.reason)
                    self.assertEqual(decision.vx, safety.cfg.side_vx_cap_mps)
                    side = decision.right if angle < 0.0 else decision.left
                    self.assertAlmostEqual(side.minimum, 0.15)
                    self.assertEqual(side.count, 1)

    def test_recovery_is_opt_in_and_odometry_measured(self):
        cfg = ns.SafetyConfig(enabled=True, recovery_enabled=True)
        safety = ns.NavigationSafety(cfg)
        blocked = [(0.0, 0.175)] + self.clear
        safety.apply(0.2, 0.0, blocked, self.nav, now=1.0)
        start = safety.apply(0.2, 0.0, blocked, self.nav, now=1.31)
        self.assertEqual(start.reason, "HARD_RECOVERY_START")
        self.assertLess(start.vx, 0.0)
        moving = safety.apply(0.2, 0.0, blocked, self.nav,
                              odom_vx=-0.08, now=1.56)
        self.assertEqual(moving.reason, "HARD_RECOVERY_BACKUP")

    def test_angular_limiter_does_not_change_linear_action(self):
        limiter = ns.AngularActionRateLimiter(0.15)
        out = limiter.apply(np.array([0.7, 0.8], dtype=np.float32))
        self.assertAlmostEqual(float(out[0]), 0.7, places=6)
        self.assertAlmostEqual(float(out[1]), 0.15, places=6)
        out = limiter.apply(np.array([-0.4, -0.8], dtype=np.float32))
        self.assertAlmostEqual(float(out[0]), -0.4, places=6)
        self.assertAlmostEqual(float(out[1]), 0.0, places=6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
