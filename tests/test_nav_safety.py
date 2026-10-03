#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from integration import nav_rl
from integration import nav_safety as ns


class NavigationSafetyTests(unittest.TestCase):
    def setUp(self):
        self.nav = nav_rl.NavRLConfig()
        self.clear = [(float(angle), 4.0) for angle in range(-125, 126, 5)]

    def test_disabled_mode_preserves_the_existing_raw_front_brake(self):
        safety = ns.NavigationSafety(ns.SafetyConfig(enabled=False))
        decision = safety.apply(0.2, 0.4, [(0.0, 0.20)] + self.clear, self.nav)
        self.assertEqual(decision.vx, 0.0)
        self.assertEqual(decision.wz, 0.4)
        self.assertTrue(decision.braked)

    def test_robust_front_distance_ignores_one_isolated_low_return(self):
        points = [(0.0, 0.10)] + [(float(a), 0.50) for a in range(-20, 21, 2)]
        cfg = ns.SafetyConfig(enabled=True)
        front, _right, _left = ns.scan_metrics(points, self.nav, cfg)
        self.assertAlmostEqual(front.minimum, 0.10)
        self.assertGreater(front.robust, 0.30)

    def test_hard_stop_uses_minimum_and_latches_until_exit(self):
        safety = ns.NavigationSafety(ns.SafetyConfig(enabled=True))
        stopped = safety.apply(0.2, 0.0, [(0.0, 0.17)] + self.clear,
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

    def test_recovery_is_opt_in_and_odometry_measured(self):
        cfg = ns.SafetyConfig(enabled=True, recovery_enabled=True)
        safety = ns.NavigationSafety(cfg)
        blocked = [(0.0, 0.10)] + self.clear
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
