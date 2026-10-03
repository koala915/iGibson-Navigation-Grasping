#!/usr/bin/env python3
from __future__ import annotations

import dataclasses
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from integration import target_approach as ta
from integration.trash_target import TrashTarget


def target(stamp, *, center=320.0, bottom=420.0):
    return TrashTarget(
        True, 0.55, 0.0, 0.55, stamp, "accepted", "test",
        bbox_xyxy=(center - 20.0, bottom - 80.0, center + 20.0, bottom),
        on_floor=True,
    )


class TargetApproachTests(unittest.TestCase):
    def make(self, **over):
        cfg = ta.ApproachConfig(enabled=True, **over)
        return ta.TargetApproachController(cfg)

    def test_waits_for_bbox_to_reach_bottom_trigger(self):
        ctl = self.make()
        self.assertIsNone(ctl.step(target(1.0, bottom=300.0), 0.0, 1.0))
        cmd = ctl.step(target(2.0, bottom=400.0), 0.0, 2.0)
        self.assertTrue(cmd.active)
        self.assertEqual(ctl.phase, ctl.CENTER)

    def test_explicit_non_floor_detection_is_rejected(self):
        ctl = self.make()
        observation = target(1.0)
        observation = dataclasses.replace(observation, on_floor=False)
        self.assertIsNone(ctl.step(observation, 0.0, 1.0))

    def test_centers_on_two_fresh_frames_then_advances_by_odom(self):
        ctl = self.make(final_travel_m=0.02, final_timeout_s=2.0)
        one = ctl.step(target(1.0), 0.0, 1.0)
        self.assertEqual(one.reason, "CENTER_CONFIRM")
        two = ctl.step(target(2.0), 0.0, 1.1)
        self.assertEqual(two.reason, "CENTER_DONE")
        drive = ctl.step(target(3.0), 0.10, 1.2)
        self.assertEqual(drive.reason, "FINAL_FORWARD")
        done = ctl.step(target(4.0), 0.10, 1.4)
        self.assertTrue(done.done)
        self.assertTrue(ctl.ready)

    def test_same_detection_holds_command_without_double_counting(self):
        ctl = self.make()
        first = ctl.step(target(1.0, center=500.0), 0.0, 1.0)
        second = ctl.step(target(1.0, center=320.0), 0.0, 1.1)
        self.assertEqual(first.wz, second.wz)
        self.assertEqual(second.reason, "CENTER_HOLD")

    def test_large_bbox_jump_stops_and_rejects_that_frame(self):
        ctl = self.make()
        ctl.step(target(1.0, center=100.0), 0.0, 1.0)
        rejected = ctl.step(target(2.0, center=500.0), 0.0, 1.1)
        self.assertEqual(rejected.wz, 0.0)
        self.assertEqual(rejected.reason, "CENTER_REJECT_JUMP")

    def test_center_timeout_fails_closed(self):
        ctl = self.make(center_timeout_s=0.5)
        ctl.step(target(1.0, center=500.0), 0.0, 1.0)
        failed = ctl.step(None, 0.0, 1.6)
        self.assertTrue(failed.failed)
        self.assertTrue(ctl.failed)


if __name__ == "__main__":
    unittest.main(verbosity=2)
