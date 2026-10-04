#!/usr/bin/env python3
"""Tests for the offboard SAM2 target adapter.

No ROS, no camera, no torch. The point of this file is the sign convention:
the publisher reports y LEFT-positive and the mission wants offset
RIGHT-positive, and getting that backwards produces a perfectly plausible
float that steers the robot away from the object every time.
"""

from __future__ import annotations

import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from integration import trash_target as tt


NOW = 1_000_000.0


def payload(**over):
    base = {
        "valid": True,
        "source": "windows_yolo_sam2",
        "frame_id": "base_footprint",
        "coordinate_convention": "x_forward_y_left",
        "object_x_base": 0.80,
        "object_y_base": 0.00,
        "distance_m": 0.80,
        "reason": "accepted",
        "timestamp_unix": NOW,
    }
    base.update(over)
    return base


class TestSignConvention(unittest.TestCase):
    """The reason this module exists."""

    def test_object_on_the_left_reports_a_negative_offset(self):
        # Publisher: y LEFT-positive, so +0.25 means 25 cm to the robot's left.
        # Mission: estimate_offset_x is "> 0 object to the right", so the same
        # object must come back as -0.25.
        target = tt.parse_trash_target(payload(object_y_base=0.25))
        found, dist, offset = tt.to_rear_detection(target, NOW)
        self.assertTrue(found)
        self.assertAlmostEqual(dist, 0.80)
        self.assertAlmostEqual(offset, -0.25)

    def test_object_on_the_right_reports_a_positive_offset(self):
        target = tt.parse_trash_target(payload(object_y_base=-0.25))
        _found, _dist, offset = tt.to_rear_detection(target, NOW)
        self.assertAlmostEqual(offset, +0.25)

    def test_dead_ahead_is_zero_either_way(self):
        target = tt.parse_trash_target(payload(object_y_base=0.0))
        _found, _dist, offset = tt.to_rear_detection(target, NOW)
        self.assertAlmostEqual(offset, 0.0)

    def test_the_flip_is_not_accidentally_symmetric(self):
        # A bug that returned abs() or 0.0 would pass a same-magnitude check.
        left = tt.to_rear_detection(
            tt.parse_trash_target(payload(object_y_base=0.4)), NOW)[2]
        right = tt.to_rear_detection(
            tt.parse_trash_target(payload(object_y_base=-0.4)), NOW)[2]
        self.assertLess(left, 0.0)
        self.assertGreater(right, 0.0)
        self.assertNotAlmostEqual(left, right)


class TestFailsClosed(unittest.TestCase):
    def test_no_target_is_not_found(self):
        self.assertEqual(tt.to_rear_detection(None, NOW), tt.NOT_FOUND)

    def test_invalid_payload_is_not_found(self):
        target = tt.parse_trash_target(payload(valid=False))
        self.assertIsNotNone(target)
        self.assertFalse(target.valid)
        self.assertEqual(tt.to_rear_detection(target, NOW), tt.NOT_FOUND)

    def test_a_stale_fix_is_not_found(self):
        target = tt.parse_trash_target(payload())
        fresh = tt.to_rear_detection(target, NOW + 0.5, max_age_s=1.0)
        stale = tt.to_rear_detection(target, NOW + 1.5, max_age_s=1.0)
        self.assertTrue(fresh[0])
        self.assertEqual(stale, tt.NOT_FOUND,
                         "a dead offboard link must stop the robot acting on "
                         "the last thing it said")

    def test_a_wildly_future_stamp_is_not_found(self):
        # Clock skew between machines must not produce a fix that never ages.
        target = tt.parse_trash_target(payload())
        self.assertEqual(tt.to_rear_detection(target, NOW - 60.0, max_age_s=1.0),
                         tt.NOT_FOUND)

    def test_an_object_behind_the_origin_is_not_found(self):
        for x in (0.0, -0.3):
            target = tt.parse_trash_target(payload(object_x_base=x))
            self.assertEqual(tt.to_rear_detection(target, NOW), tt.NOT_FOUND,
                             f"x={x} is not ahead of the robot")

    def test_a_frame_mismatch_is_refused(self):
        for over in ({"frame_id": "map"},
                     {"coordinate_convention": "x_forward_y_right"}):
            target = tt.parse_trash_target(payload(**over))
            self.assertFalse(target.valid, over)
            self.assertEqual(tt.to_rear_detection(target, NOW), tt.NOT_FOUND)

    def test_non_finite_coordinates_are_refused(self):
        for bad in ("NaN", "Infinity"):
            raw = json.dumps(payload()).replace('0.8', bad, 1)
            target = tt.parse_trash_target(raw)
            if target is not None and target.valid:
                self.fail(f"accepted a non-finite coordinate: {bad}")


class TestParsing(unittest.TestCase):
    def test_accepts_json_text_mapping_and_rosbridge_envelope(self):
        body = payload()
        for form in (body, json.dumps(body), {"data": json.dumps(body)}):
            target = tt.parse_trash_target(form)
            self.assertIsNotNone(target, form)
            self.assertTrue(target.valid)
            self.assertAlmostEqual(target.x_forward_m, 0.80)

    def test_garbage_returns_none_rather_than_raising(self):
        for junk in ("", "not json", b"\xff\xfe", [], 42, None, "{}"):
            self.assertIsNone(tt.parse_trash_target(junk), junk)

    def test_distance_is_derived_when_absent(self):
        body = payload(object_x_base=0.3, object_y_base=0.4)
        body.pop("distance_m")
        target = tt.parse_trash_target(body)
        self.assertAlmostEqual(target.distance_m, 0.5)

    def test_the_publisher_shutdown_message_parses_as_invalid(self):
        # rear_cam_sam2_publisher sends this in its finally block.
        shutdown = {
            "valid": False, "source": "windows_yolo_sam2",
            "class_name": "sugarbox", "frame_id": "base_footprint",
            "coordinate_convention": "x_forward_y_left",
            "reason": "publisher_shutdown", "on_floor": None,
            "floor_check_enabled": False, "timestamp_unix": NOW,
        }
        target = tt.parse_trash_target(shutdown)
        self.assertIsNotNone(target)
        self.assertFalse(target.valid)
        self.assertEqual(target.reason, "publisher_shutdown")
        self.assertEqual(tt.to_rear_detection(target, NOW), tt.NOT_FOUND)

    def test_optional_approach_metadata_is_preserved(self):
        target = tt.parse_trash_target(payload(
            bbox_xyxy=[10, 20, 50, 80], mask_bottom_u=30,
            mask_bottom_v=79, on_floor=True, observation_age_s=0.2))
        self.assertEqual(target.bbox_xyxy, (10.0, 20.0, 50.0, 80.0))
        self.assertTrue(target.on_floor)
        self.assertAlmostEqual(target.observation_age_s, 0.2)

    def test_latency_compensation_propagates_target_to_now(self):
        target = tt.parse_trash_target(payload(
            object_x_base=1.0, object_y_base=0.0,
            observation_age_s=0.5))
        found, forward, offset = tt.to_rear_detection(
            target, NOW, compensate_vx=0.2, compensate_wz=0.0)
        self.assertTrue(found)
        self.assertAlmostEqual(forward, 0.9)
        self.assertAlmostEqual(offset, 0.0)

    def test_latency_compensation_includes_local_receipt_age(self):
        target = tt.parse_trash_target(payload(
            object_x_base=1.0, object_y_base=0.0,
            timestamp_unix=NOW - 0.25, observation_age_s=0.25))
        found, forward, _offset = tt.to_rear_detection(
            target, NOW, compensate_vx=0.2, compensate_wz=0.0)
        self.assertTrue(found)
        self.assertAlmostEqual(forward, 0.9)


class TestContractWithThePublisher(unittest.TestCase):
    """Pin the field names against the publisher, so a rename cannot drift."""

    def test_the_publisher_and_the_adapter_agree_on_every_field(self):
        src = (Path(__file__).resolve().parents[1]
               / "detection" / "rear_cam_sam2_publisher.py").read_text(
                   encoding="utf-8")
        for field in ("valid", "object_x_base", "object_y_base", "distance_m",
                      "timestamp_unix", "frame_id", "coordinate_convention",
                      "reason", "source", "bbox_xyxy", "observation_age_s"):
            self.assertIn(f'"{field}"', src,
                          f"the adapter reads {field!r} but the publisher no "
                          f"longer writes it")

    def test_the_declared_convention_matches_what_the_adapter_expects(self):
        src = (Path(__file__).resolve().parents[1]
               / "detection" / "rear_cam_sam2_publisher.py").read_text(
                   encoding="utf-8")
        self.assertIn(f'"{tt.EXPECTED_CONVENTION}"', src)
        self.assertIn(f'"{tt.EXPECTED_FRAME}"', src)

    def test_the_pipeline_still_defines_offset_as_right_positive(self):
        # If someone flips estimate_offset_x, the negation here becomes wrong.
        src = (Path(__file__).resolve().parents[1]
               / "integration" / "vision_grasp_pipeline.py").read_text(
                   encoding="utf-8")
        self.assertIn(">0 object to the right", src,
                      "estimate_offset_x's convention changed; the sign flip "
                      "in trash_target.to_rear_detection must be rechecked")


if __name__ == "__main__":
    unittest.main(verbosity=2)
