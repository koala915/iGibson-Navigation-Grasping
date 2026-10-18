"""Placement-box docking geometry and checks: no ROS, robot or model."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integration import placement_dock as dock

ANGLE_MIN = -math.pi
STEP = 2 * math.pi / 720


def scan(points_robot):
    """Raw TG30 ranges (yaw 180 deg, 0.10 m ahead of the centre) for robot-frame points."""
    ranges = [float('inf')] * 720
    for x, y in points_robot:
        sx, sy = x - 0.10, y                         # sensor position, robot axes
        angle = math.atan2(sy, sx) - math.pi         # raw beam angle
        index = int(round((dock.wrap(angle) - ANGLE_MIN) / STEP)) % 720
        ranges[index] = min(ranges[index], math.hypot(sx, sy))
    return ranges


def wall(x, half=0.30, step=0.01):
    return [(x, -half + i * step) for i in range(int(2 * half / step) + 1)]


class GeometryTests(unittest.TestCase):
    def test_map_points_come_out_in_the_chassis_frame(self):
        pose = (1.0, 1.0, math.pi / 2)                    # facing +y
        self.assertEqual([round(v, 6) for v in dock.relative_point(pose, (1.0, 2.0))], [1.0, 0.0])
        self.assertEqual([round(v, 6) for v in dock.relative_point(pose, (0.0, 1.0))], [0.0, 1.0])
        bearing, distance = dock.box_bearing(pose, {'x': 0.0, 'y': 2.0})
        self.assertAlmostEqual(math.degrees(bearing), 45.0)
        self.assertAlmostEqual(distance, math.sqrt(2))
        x, y = dock.point_ahead((1.0, 1.0, math.pi / 2), 0.5)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 1.5)

    def test_seed_is_the_near_edge_half_a_box_short_of_the_centre(self):
        x, y = dock.seed_point((0.0, 0.0, 0.0), {'x': 1.2, 'y': 0.0})
        self.assertAlmostEqual(x, 1.2 - 0.0675)
        self.assertAlmostEqual(y, 0.0)
        for far in (0.2, 3.5):
            with self.subTest(far=far), self.assertRaises(ValueError):
                dock.seed_point((0.0, 0.0, 0.0), {'x': far, 'y': 0.0})

    def test_release_point_sits_over_the_box_middle(self):
        # SAM measures the near edge: it stops half the 13.5 cm depth short of 0.28 m.
        self.assertAlmostEqual(dock.PLACEMENT_FINAL_X_M + dock.BOX_DEPTH_M / 2,
                               dock.RELEASE_CENTER_X_M, delta=0.001)
        self.assertGreater(dock.PLACEMENT_FINAL_X_M, 0.15 + 0.03)   # chassis front + 3 cm

    def test_committed_box_map_loads_and_is_not_yet_verified(self):
        box = dock.load_box_map()
        self.assertFalse(box['verified'])
        self.assertTrue(math.isfinite(box['x']) and math.isfinite(box['y']))


class LocalizationTests(unittest.TestCase):
    def record(self, **changes):
        value = {'ready': True, 'age': 0.2, 'state': {'pose': [1.0, 0.5, 0.3]}}
        value.update(changes)
        return value

    def test_fresh_ready_pose_is_used(self):
        self.assertEqual(dock.localization_pose(self.record()), (1.0, 0.5, 0.3))

    def test_anything_doubtful_refuses(self):
        for bad in (None, self.record(ready=False, reason='waiting_for_seed'),
                    self.record(age=3.0), self.record(age=-1), self.record(age=None),
                    self.record(state={}), self.record(state={'pose': [1, 2, float('nan')]})):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                dock.localization_pose(bad)


class ArrivalTests(unittest.TestCase):
    final, aim = 0.212, -0.028

    def report(self, **changes):
        value = {'arrived': True, 'reason': 'PURSUIT_DONE x=0.211 y=-0.030', 'seeded': False,
                 'x': 0.211, 'y': -0.030}
        value.update(changes)
        return value

    def test_a_checked_stop_may_release(self):
        self.assertIsNone(dock.arrival_problem(self.report(), self.final, self.aim))

    def test_anything_else_never_releases(self):
        for bad in (None, self.report(arrived=False, reason='NO_PROGRESS'),
                    self.report(reason='PURSUIT_TIMEOUT x=0.400 y=0.000'),
                    self.report(seeded=True), self.report(seeded=None),
                    self.report(x=None), self.report(x=0.25), self.report(x=0.17),
                    self.report(y=0.03), self.report(y=-0.08)):
            with self.subTest(bad=bad):
                self.assertIsNotNone(dock.arrival_problem(bad, self.final, self.aim))


class LidarTests(unittest.TestCase):
    def test_wall_ahead_reads_from_the_sensor_like_the_front_brake(self):
        ranges = scan(wall(0.60))
        self.assertAlmostEqual(dock.corridor_distance(ranges, ANGLE_MIN, STEP), 0.50, delta=0.005)

    def test_side_returns_outside_the_corridor_do_not_count(self):
        ranges = scan(wall(0.60) + [(0.30, 0.25), (0.30, -0.25)])
        self.assertAlmostEqual(dock.corridor_distance(ranges, ANGLE_MIN, STEP), 0.50, delta=0.005)

    def test_rear_clearance_skips_the_robot_itself(self):
        echoes = [(-0.12, y / 100) for y in range(-10, 11)]          # mast and wiring
        ranges = scan(wall(-0.55) + echoes)
        self.assertAlmostEqual(dock.rear_clearance(ranges, ANGLE_MIN, STEP), 0.40, delta=0.005)
        self.assertEqual(dock.rear_clearance(scan(echoes), ANGLE_MIN, STEP), float('inf'))

    def test_wall_correction_moves_only_small_known_errors(self):
        self.assertEqual(dock.wall_correction(0.480, 0.470), 0.0)
        self.assertAlmostEqual(dock.wall_correction(0.500, 0.470), 0.030)
        self.assertAlmostEqual(dock.wall_correction(0.440, 0.470), -0.030)
        for measured in (0.60, 0.38, float('inf')):
            with self.subTest(measured=measured), self.assertRaises(ValueError):
                dock.wall_correction(measured, 0.470)


if __name__ == '__main__':
    unittest.main()
