"""enoch's training-matched PPO inputs and shaping, on the team's measured LiDAR frame."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'integration'))
import sugarbox_training_contract as tc                                   # noqa: E402
from sugarbox_lidar_geometry import LidarGeometry, robot_scan_samples   # noqa: E402

GEOMETRY = LidarGeometry(yaw_offset_deg=180.0, forward_offset_m=0.10)    # the measured TF
BEAMS = 2020


def scan_with(robot_hits=(), rear=None):
    """A TG30-like scan in the SENSOR frame; robot-frame hits given as (deg, m).

    The sensor faces backwards (yaw 180): a robot angle a is raw angle a - 180.
    """
    angle_min, step = -math.pi, 2 * math.pi / BEAMS
    ranges = np.full(BEAMS, np.inf)
    raw = angle_min + np.arange(BEAMS) * step
    robot = np.degrees(np.arctan2(np.sin(raw + math.pi), np.cos(raw + math.pi)))
    for deg, distance in robot_hits:
        ranges[np.abs(robot - deg) <= 3.0] = distance
    if rear is not None:                        # chassis echoes behind the sensor
        ranges[np.abs(np.degrees(raw)) < 60] = rear
    return robot_scan_samples(ranges, angle_min, step, GEOMETRY)


def side_wall_scan(y_wall, x_from=-0.30, x_to=2.0):
    """A straight wall parallel to the robot, y_wall from the centre line (left +)."""
    angle_min, step = -math.pi, 2 * math.pi / BEAMS
    raw = angle_min + np.arange(BEAMS) * step
    robot = raw + math.pi                       # beam direction from the sensor
    ranges = np.full(BEAMS, np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        t = y_wall / np.sin(robot)
    x = GEOMETRY.forward_offset_m + t * np.cos(robot)
    hit = (t > 0) & (x >= x_from) & (x <= x_to)
    ranges[hit] = t[hit]
    return robot_scan_samples(ranges, angle_min, step, GEOMETRY)


def ray_index(deg):
    return int(round((deg + 90.0) / 180.0 * 47))


class GeometryTests(unittest.TestCase):
    def test_goal_moves_back_to_the_chassis_centre_only(self):
        self.assertEqual(tc.center_relative_goal(0.8, 0.2), (0.8 + 0.135, 0.2))

    def test_speed_is_unsigned_planar_speed(self):
        self.assertAlmostEqual(tc.planar_speed(-0.03, 0.04), 0.05)

    def test_obstacle_on_the_left_lands_on_a_left_ray(self):
        # The point of the change from enoch's snapshot: the measured TF rotates
        # the scan; his mirror would have put this obstacle on the right.
        rays = tc.center_rays(scan_with([(60.0, 0.5)]))
        # Seen from the centre, 0.10 m behind the sensor, it is ~51 deg left.
        centre_deg = math.degrees(math.atan2(0.5 * math.sin(math.radians(60)),
                                             0.5 * math.cos(math.radians(60)) + 0.10))
        self.assertLess(rays[ray_index(centre_deg)], 1.0)
        self.assertEqual(rays[ray_index(-centre_deg)], 4.0)

    def test_straight_ahead_range_is_measured_from_the_centre(self):
        rays = tc.center_rays(scan_with([(0.0, 0.5)]))
        # 0.5 m from the sensor, which sits 0.10 m ahead of the centre.
        self.assertAlmostEqual(float(rays[ray_index(0)]), 0.6, delta=0.01)

    def test_rear_echoes_never_become_policy_hits(self):
        rays = tc.center_rays(scan_with(rear=0.05))
        np.testing.assert_array_equal(rays, np.full(48, 4.0, dtype=np.float32))

    def test_wall_beside_the_robot_reaches_the_side_rays(self):
        # 2026-10-12: a wall 35 cm to the right read as "nothing" at -90..-78 deg,
        # because only beams ahead of the sensor counted.
        rays = tc.center_rays(side_wall_scan(-0.35))
        for deg in (-90.0, -86.0, -82.0, -78.0):
            self.assertLess(rays[ray_index(deg)], 0.40, deg)
        self.assertEqual(rays[ray_index(80.0)], 4.0)

    def test_robot_body_behind_the_sensor_is_not_an_obstacle(self):
        # Mast and arm-base echoes measured on 2026-10-12, inside the footprint.
        rays = tc.center_rays(scan_with([(135.0, 0.14), (-135.0, 0.14), (180.0, 0.10)]))
        np.testing.assert_array_equal(rays, np.full(48, 4.0, dtype=np.float32))

    def test_near_hits_are_floored_like_training_rays(self):
        rays = tc.center_rays(scan_with([(80.0, 0.05)]))
        self.assertAlmostEqual(float(rays.min()), 0.33, places=5)

    def test_empty_scan_reads_no_hit_everywhere(self):
        np.testing.assert_array_equal(tc.center_rays(scan_with()), np.full(48, 4.0, dtype=np.float32))


class ObservationAndActionTests(unittest.TestCase):
    def test_observation_layout(self):
        rays = np.full(48, 2.0, dtype=np.float32)
        obs = tc.training_observation(1.0, 0.0, -0.05, 0.0, 0.3, [0.2, -0.1], rays)
        self.assertEqual(obs.shape, (55,))
        np.testing.assert_allclose(obs[:7], [1.0, 0.0, 1.0, 0.05, 0.3, 0.2, -0.1], atol=1e-6)
        np.testing.assert_allclose(obs[7:], 2.0)

    def test_zero_delay_acts_now_and_training_delay_waits_two_steps(self):
        now = tc.TrainingActionReference(delay_steps=0)
        later = tc.TrainingActionReference(delay_steps=2)
        action = np.array([1.0, 0.5])
        args = (2.0, 2.0, 0.0, 0.2, 0.0)          # clear, far, straight, moving
        self.assertAlmostEqual(now.step(action, *args)['vx'], 0.5)
        self.assertEqual(later.step(action, *args)['vx'], 0.0)
        self.assertEqual(later.step(action, *args)['vx'], 0.0)
        self.assertAlmostEqual(later.step(action, *args)['vx'], 0.5)

    def test_near_obstacle_slows_and_reverse_is_slow(self):
        ref = tc.TrainingActionReference(delay_steps=0)
        near = ref.step(np.array([1.0, 1.0]), 0.33, 2.0, 0.0, 0.2, 0.0)
        self.assertAlmostEqual(near['vx'], 0.5 * (0.24 + 0.76 * 0.5), places=6)
        back = ref.step(np.array([-1.0, 0.0]), 2.0, 2.0, 0.0, 0.2, 0.0)
        self.assertAlmostEqual(back['vx'], -0.15)

    def test_limits_keep_the_commanded_curvature(self):
        vx, wz = tc.limited_reference_twist(0.5, 1.2, 0.15, 0.08, 0.78)
        self.assertAlmostEqual(vx / wz, 0.5 / 1.2)
        self.assertLessEqual(vx, 0.15 + 1e-12)
        vx, _ = tc.limited_reference_twist(-0.15, 0.0, 0.15, 0.08, 0.78)
        self.assertAlmostEqual(vx, -0.08)


if __name__ == '__main__':
    unittest.main()
