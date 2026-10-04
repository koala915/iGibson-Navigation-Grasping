"""Standalone LiDAR frame regressions; no ROS, vision models or hardware."""
import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from integration import nav_rl
from integration.sugarbox_lidar_geometry import (
    LidarGeometry, nearest_policy_ranges, robot_scan_samples, wrap_angle,
)


def runtime_geometry_functions():
    """Load the actual adapters without importing the ROS/Ultralytics loop."""
    names = {
        'NUM_LIDAR_RAYS', 'LIDAR_MAX_M', 'PPO_LIDAR_DISTANCE_SCALE',
        'POLICY_ANGLES_DEG', 'LIDAR_GEOMETRY', 'FRONT_SAFETY_HALF_ANGLE_DEG',
        'FRONT_ROBUST_MIN_POINTS', 'FRONT_ROBUST_PERCENTILE',
        'SIDE_SAFETY_MAX_ANGLE_DEG', 'SIDE_SAFETY_FRONT_EXCLUDE_DEG',
        'SIDE_ROBUST_MIN_POINTS', 'SIDE_ROBUST_PERCENTILE',
        'scan_to_policy_rays', 'raw_front_metrics', 'raw_side_metrics',
    }
    source = ROOT / 'integration/sugarbox_rl_approach_final2.py'
    selected = []
    for node in ast.parse(source.read_text(encoding='utf-8')).body:
        if isinstance(node, ast.FunctionDef) and node.name in names:
            selected.append(node)
        elif isinstance(node, ast.Assign) and any(
                isinstance(target, ast.Name) and target.id in names
                for target in node.targets):
            selected.append(node)
    namespace = {
        'np': np, 'math': math, 'os': SimpleNamespace(environ={}),
        'LidarGeometry': LidarGeometry,
        'nearest_policy_ranges': nearest_policy_ranges,
        'robot_scan_samples': robot_scan_samples,
    }
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(source), 'exec'),
         namespace)
    return namespace


class SugarboxLidarGeometryTests(unittest.TestCase):
    def setUp(self):
        self.geometry = LidarGeometry()
        self.runtime = runtime_geometry_functions()

    def test_measured_tf_rotates_without_reflecting_left_and_right(self):
        # The ROS TF observed on the Jetson: yaw pi, x +0.10 m.
        cases = ((180.0, 1.0, 1.1, 0.0), (90.0, 1.0, 0.1, -1.0),
                 (-90.0, 1.0, 0.1, 1.0), (0.0, 1.0, -0.9, 0.0),
                 (135.0, 1.0, 0.1 + math.sqrt(0.5), -math.sqrt(0.5)))
        for raw_angle, distance, x, y in cases:
            with self.subTest(raw_angle=raw_angle):
                sample = robot_scan_samples(
                    [distance], math.radians(raw_angle), 0.01, self.geometry)[0]
                self.assertAlmostEqual(sample.range_m * math.cos(sample.angle_rad), x)
                self.assertAlmostEqual(sample.range_m * math.sin(sample.angle_rad), y)

    def test_valid_samples_match_existing_canonical_ros_geometry(self):
        msg = {'angle_min': -math.pi, 'angle_increment': math.pi / 12,
               'range_min': 0.02, 'range_max': 100.0,
               'ranges': [0.0, 0.01, 0.02, float('inf'), float('nan')]
                         + [0.14, 0.5, 1.0, 2.0] * 5}
        cfg = nav_rl.NavRLConfig(lidar_yaw_offset_deg=180.0,
                                 lidar_forward_offset_m=0.10)
        expected = [nav_rl._transform_scan_sample(angle, distance, cfg)
                    for angle, distance in nav_rl.laser_scan_to_points(msg)]
        expected = [sample for sample in expected if sample is not None]
        actual = [sample for sample in robot_scan_samples(
            msg['ranges'], msg['angle_min'], msg['angle_increment'], self.geometry)
            if sample.range_m is not None]
        self.assertEqual(len(actual), len(expected))
        for sample, (angle, distance) in zip(actual, expected):
            self.assertAlmostEqual(wrap_angle(sample.angle_rad - angle), 0.0)
            self.assertAlmostEqual(sample.range_m, distance)

    def test_zero_to_two_pi_scan_has_the_same_geometry(self):
        a = robot_scan_samples([1.0] * 360, -math.pi, math.pi / 180,
                               self.geometry)
        b = robot_scan_samples([1.0] * 360, math.pi, math.pi / 180,
                               self.geometry)
        for first, second in zip(a, b):
            self.assertAlmostEqual(wrap_angle(first.angle_rad - second.angle_rad), 0.0)
            self.assertAlmostEqual(first.range_m, second.range_m)

    def _scan_with_robot_hit(self, beam_index, body_distance, *, missing=False):
        # Align one raw sensor sample exactly with the selected corrected body
        # beam; this exposes the old reflection and the missing translation.
        angle = math.radians(float(self.runtime['POLICY_ANGLES_DEG'][beam_index]))
        sensor_x = body_distance * math.cos(angle) - 0.10
        sensor_y = body_distance * math.sin(angle)
        raw_angle = wrap_angle(math.atan2(sensor_y, sensor_x) - math.pi)
        increment, hit_index = math.pi / 1024, 1024
        ranges = np.full(2048, float('inf') if missing else 3.0)
        ranges[hit_index] = math.hypot(sensor_x, sensor_y)
        return SimpleNamespace(ranges=ranges,
                               angle_min=raw_angle - hit_index * increment,
                               angle_increment=increment)

    def test_right_and_left_hits_reach_the_original_48_beam_indices(self):
        for beam_index, side in ((8, 'right'), (39, 'left')):
            with self.subTest(side=side):
                scan = self._scan_with_robot_hit(beam_index, 0.40)
                rays = self.runtime['scan_to_policy_rays'](scan)
                self.assertEqual(rays.shape, (48,))
                self.assertEqual(rays.dtype, np.float32)
                self.assertEqual(int(np.argmin(rays)), beam_index)
                self.assertAlmostEqual(float(rays[beam_index]), 0.40 * 1.15, places=6)
                right, right_min, _, left, left_min, _ = self.runtime['raw_side_metrics'](scan)
                self.assertAlmostEqual(right_min if side == 'right' else left_min,
                                       0.40, places=6)
                self.assertGreater(left_min if side == 'right' else right_min, 2.8)

    def test_front_policy_and_safety_share_translation_without_scaling_safety(self):
        scan = self._scan_with_robot_hit(23, 0.35)
        rays = self.runtime['scan_to_policy_rays'](scan)
        robust, minimum, count = self.runtime['raw_front_metrics'](scan)
        self.assertEqual(int(np.argmin(rays)), 23)
        self.assertAlmostEqual(float(rays[23]), 0.35 * 1.15, places=6)
        self.assertAlmostEqual(minimum, 0.35, places=6)
        self.assertGreater(robust, minimum)
        self.assertGreater(count, 8)

    def test_nearest_sampling_is_preserved_instead_of_min_pooling(self):
        angle = float(self.runtime['POLICY_ANGLES_DEG'][8])
        samples = robot_scan_samples([0.5, 0.1], math.radians(angle), 0.01,
                                     LidarGeometry(0.0, 0.0))
        rays = nearest_policy_ranges(samples, self.runtime['POLICY_ANGLES_DEG'],
                                     max_range_m=4.0, distance_scale=1.15)
        self.assertAlmostEqual(rays[8], 0.5 * 1.15)

    def test_missing_returns_are_not_filled_with_a_neighbouring_hit(self):
        scan = self._scan_with_robot_hit(8, 0.40, missing=True)
        rays = self.runtime['scan_to_policy_rays'](scan)
        self.assertAlmostEqual(float(rays[8]), 0.40 * 1.15, places=6)
        self.assertEqual(int(np.count_nonzero(rays < 4.0)), 1)
        scan.ranges[:] = float('inf')
        np.testing.assert_array_equal(self.runtime['scan_to_policy_rays'](scan),
                                      np.full(48, 4.0, dtype=np.float32))
        self.assertEqual(self.runtime['raw_front_metrics'](scan),
                         (float('inf'), float('inf'), 0))

    def test_partial_scan_does_not_populate_unobserved_policy_angles(self):
        samples = robot_scan_samples([1.0], math.radians(40), 0.01,
                                     LidarGeometry(0.0, 0.0))
        rays = nearest_policy_ranges(samples, self.runtime['POLICY_ANGLES_DEG'],
                                     max_range_m=4.0, distance_scale=1.15)
        self.assertEqual(sum(value < 4.0 for value in rays), 1)

    def test_empty_scan_keeps_the_adapter_failure_contract(self):
        for ranges in (None, []):
            scan = SimpleNamespace(ranges=ranges, angle_min=0.0, angle_increment=1.0)
            self.assertIsNone(self.runtime['scan_to_policy_rays'](scan))

    def test_mounting_override_and_invalid_metadata_fail_closed(self):
        for yaw, offset in ((float('nan'), 0.1), (180.0, float('inf'))):
            with self.assertRaises(ValueError):
                LidarGeometry(yaw, offset)
        for start, increment in ((float('nan'), 0.1), (0.0, 0.0)):
            with self.assertRaises(ValueError):
                robot_scan_samples([1.0], start, increment, self.geometry)
        sample = robot_scan_samples([0.5], 0.0, 0.1, LidarGeometry(0.0, 0.0))[0]
        self.assertAlmostEqual(sample.range_m, 0.5)
        self.assertAlmostEqual(sample.angle_rad, 0.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
