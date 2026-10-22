"""Placement-box docking geometry and checks: no ROS, robot or model."""
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

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

    def test_release_point_sits_over_the_box(self):
        # SAM measures the near edge: it stops half the 13.5 cm depth short of 0.28 m,
        # and 4 cm farther since 2026-10-21 (the robot ran into the box).
        self.assertAlmostEqual(dock.PLACEMENT_FINAL_X_M + dock.BOX_DEPTH_M / 2
                               - dock.PLACEMENT_STOP_BACK_M,
                               dock.RELEASE_CENTER_X_M, delta=0.001)
        # The jaw still opens over the box, a sugarbox's half width past its near edge.
        self.assertGreaterEqual(dock.RELEASE_CENTER_X_M - dock.PLACEMENT_FINAL_X_M, 0.0265)
        self.assertGreater(dock.PLACEMENT_FINAL_X_M, 0.15 + 0.03)   # chassis front + 3 cm


class SceneLayoutTests(unittest.TestCase):
    """The box position comes from ncu's scene_layout.json (2026-10-19)."""

    def layout(self, valid=False):
        # The shape of maps/demo_box_20261018_patrol/scene_layout.json (PR #5, acca195).
        return {'frame_id': 'map', 'bin': {
            'pose_map': {'x': 1.7584, 'y': 0.8763, 'yaw': 1.12},
            'pose_map_valid_for_current_placement': valid,
            'placement_update': {'midpoint_pose_map_candidate': {'x': 1.5599, 'y': 0.8333}}}}

    def test_the_moved_box_candidate_stands_in_unverified(self):
        box = dock.box_from_layout(self.layout())
        self.assertEqual((box['x'], box['y'], box['verified']), (1.5599, 0.8333, False))

    def test_a_pose_ncu_marks_valid_is_the_verified_position(self):
        box = dock.box_from_layout(self.layout(valid=True))
        self.assertEqual((box['x'], box['y'], box['verified']), (1.7584, 0.8763, True))

    def test_a_layout_without_a_usable_box_is_refused(self):
        broken = self.layout()
        broken['bin']['placement_update'] = {}
        for layout in ({}, {'frame_id': 'odom', 'bin': {}}, broken, None):
            with self.subTest(layout=layout), self.assertRaises(ValueError):
                dock.box_from_layout(layout)

    def test_the_room_walls_come_from_the_layout(self):
        layout = dict(self.layout(), room_polygon=[[0, 0], [2, 0], [2, 2], [0, 2]])
        self.assertEqual(dock.walls_from_layout(layout), [(0, 0), (2, 0), (2, 2), (0, 2)])
        for broken in ({}, dict(layout, room_polygon=[[0, 0], [1, 1]]),
                       dict(layout, room_polygon=[[0, 0], [2, 0], [2, 'x']])):
            with self.subTest(broken=broken):
                self.assertIsNone(dock.walls_from_layout(broken))

    def test_near_a_wall_or_beyond_it(self):
        room = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
        for point, near in (((1.0, 1.0), False), ((0.2, 1.0), False), ((0.1, 1.0), True),
                            ((1.95, 1.95), True), ((-0.3, 1.0), True), ((1.0, 2.5), True)):
            with self.subTest(point=point):
                self.assertEqual(dock.near_wall(point, room, 0.15), near)
        # ncu's room is turned 26 deg in the map
        tilted = [(-0.70, 0.03), (1.08, -0.83), (1.96, 0.99), (0.18, 1.85)]
        self.assertFalse(dock.near_wall((0.63, 0.51), tilted, 0.15))     # the middle
        self.assertTrue(dock.near_wall((0.19, -0.37), tilted, 0.15))     # on a wall

    def test_a_local_copy_reads_the_same(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'scene_layout.json'
            path.write_text(json.dumps(self.layout()), encoding='utf-8')
            box = dock.load_box_map(path)
        self.assertEqual((box['x'], box['verified'], box['source']), (1.5599, False, str(path)))

    def test_the_robot_copy_is_read_over_ssh(self):
        done = subprocess.CompletedProcess([], 0, stdout=json.dumps(self.layout()), stderr='')
        with patch.object(dock.subprocess, 'run', return_value=done) as remote:
            box = dock.fetch_box('10.0.0.1')
        self.assertIn('/home/jetson/Documents/deploy_jetson2/maps/demo_box_20261018_patrol/'
                      'scene_layout.json', remote.call_args.args[0][-1])
        self.assertFalse(box['verified'])
        missing = subprocess.CompletedProcess([], 1, stdout='', stderr='No such file')
        with patch.object(dock.subprocess, 'run', return_value=missing), \
             self.assertRaises(ValueError):
            dock.fetch_box('10.0.0.1')


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


class WaitLocalizationTests(unittest.TestCase):
    """A short outage is waited out with the robot stopped (2026-10-21 run 21)."""

    def good(self, stamp, pose=(1.0, 0.5, 0.3), odom=(0.0, 0.0, 0.0)):
        return {'ready': True, 'age': 0.1, 'stamp': stamp,
                'state': {'pose': list(pose), 'odom': list(odom)}}

    def wait(self, records, before=None, wait_s=20.0):
        records, clock, reads = iter(records), [0.0], []

        def read(host):
            reads.append(host)
            return next(records)

        def sleep(seconds):
            clock[0] += seconds
        with patch('builtins.print'):
            record = dock.wait_localization('h', read, before=before, wait_s=wait_s,
                                            sleep=sleep, now=lambda: clock[0])
        return record, len(reads), clock[0]

    def test_a_ready_record_is_one_read_with_no_wait(self):
        self.assertEqual(self.wait([self.good(1)])[1:], (1, 0.0))

    def test_an_outage_ends_on_three_new_agreeing_records(self):
        lost = {'ready': False, 'reason': 'waiting for fresh scan', 'stamp': 1}
        record, reads, waited = self.wait(
            [lost, lost, self.good(3), self.good(3), self.good(4),
             self.good(5, pose=(1.10, 0.5, 0.3)),      # moved 10 cm: not settled yet
             self.good(6, pose=(1.10, 0.5, 0.3)), self.good(7, pose=(1.11, 0.5, 0.3))])
        self.assertEqual((record['stamp'], reads), (7, 8))
        self.assertAlmostEqual(waited, 7 * dock.LOCALIZATION_POLL_S)

    def test_an_outage_that_lasts_gives_up(self):
        lost = {'ready': False, 'reason': 'stale AMCL'}
        with self.assertRaisesRegex(ValueError, 'after waiting 2 s: not localized: stale AMCL'):
            self.wait([lost] * 20, wait_s=2.0)

    def test_a_pose_odometry_cannot_explain_is_a_jump(self):
        lost = {'ready': False, 'reason': 'AMCL restarted'}
        before = self.good(0, pose=(1.0, 0.5, 0.0), odom=(0.0, 0.0, 0.0))
        # Odometry went 0.20 m ahead: the map pose should be near (1.20, 0.5).
        after = lambda stamp, x: self.good(stamp, pose=(x, 0.5, 0.0), odom=(0.20, 0.0, 0.0))
        record, _, _ = self.wait([lost] + [after(s, 1.22) for s in (1, 2, 3)], before=before)
        self.assertEqual(record['stamp'], 3)
        with self.assertRaisesRegex(ValueError, 'relocalize'):
            self.wait([lost] + [after(s, 1.50) for s in (1, 2, 3)], before=before)
        # No odometry to compare: the supervisor's own checks stand alone.
        self.assertIsNone(dock.localization_jump({'state': {'pose': [0, 0, 0]}},
                                                 after(1, 1.50)))
        # The jump test runs only across an outage, never on a first good read.
        self.assertEqual(self.wait([after(1, 1.50)], before=before)[1], 1)

    def test_odometry_turns_are_carried_into_the_map_frame(self):
        # map = odom rotated +90 deg: 0.20 m along odom x is 0.20 m along map y.
        before = self.good(0, pose=(1.0, 1.0, math.pi / 2), odom=(0.0, 0.0, 0.0))
        moved = self.good(1, pose=(1.0, 1.2, math.pi / 2 + 0.1), odom=(0.2, 0.0, 0.1))
        self.assertIsNone(dock.localization_jump(before, moved))
        self.assertIn('0.28 m', dock.localization_jump(
            before, self.good(1, pose=(1.2, 1.0, math.pi / 2), odom=(0.2, 0.0, 0.0))))

    def test_an_empty_ssh_reply_is_no_record(self):
        done = subprocess.CompletedProcess([], 255, stdout='', stderr='timeout')
        with patch.object(dock.subprocess, 'run', return_value=done):
            self.assertIsNone(dock.read_localization('h'))


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

    def test_backing_is_checked_against_the_walls_on_the_map(self):
        room = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
        facing_east = (1.0, 1.0, 0.0)                 # backing goes west
        self.assertIsNone(dock.back_path_problem(facing_east, 0.20, room))
        # rear edge 1.0 - 0.23 - 0.60 = 0.17 m off the west wall: still fine
        self.assertIsNone(dock.back_path_problem(facing_east, 0.60, room))
        # 1.0 - 0.23 - 0.70 = 0.07 m: inside the 0.08 m margin
        self.assertIn('within', dock.back_path_problem(facing_east, 0.70, room))
        # the right-side cables: facing north, the robot's right is east
        self.assertIn('within', dock.back_path_problem((1.75, 1.0, math.pi / 2), 0.10, room))
        # 2026-10-21: an angled stop with a wheel at the east wall; backing straight
        # takes it away from the wall, so it is the way out, not refused.
        at_wall = (1.68, 1.0, math.radians(47))
        self.assertIsNone(dock.back_path_problem(at_wall, 0.20, room))
        self.assertIsNone(dock.back_path_problem(at_wall, 0.05, room))
        # Backing toward a wall within the margin still is.
        self.assertIn('within', dock.back_path_problem((1.68, 1.0, math.radians(180 + 47)),
                                                       0.20, room))
        x, y = dock.to_map((1.0, 1.0, math.pi / 2), (1.0, 0.5))
        self.assertAlmostEqual(x, 0.5)
        self.assertAlmostEqual(y, 2.0)

    def test_the_patrol_mask_reads_like_ncus_policy(self):
        # 3 x 2 cells of 0.5 m from (0, 0); the top-left one allowed (rows run down).
        pgm = b'P5\n# ncu\n3 2\n255\n' + bytes([255, 0, 0, 0, 0, 0])
        mask = dock.NavMask(*dock.parse_pgm(pgm), resolution=0.5, origin=(0.0, 0.0))
        self.assertTrue(mask.allowed(0.2, 0.7))
        self.assertFalse(mask.allowed(0.2, 0.2))
        self.assertFalse(mask.allowed(-0.1, 0.7))     # off the map
        self.assertTrue(mask.inside((0.25, 0.75), margin=0.2))
        self.assertFalse(mask.inside((0.25, 0.75), margin=0.3))
        with self.assertRaises(ValueError):
            dock.parse_pgm(b'P5\n3 2\n255\n' + bytes(5))

    def test_backing_never_sweeps_into_the_placement_box(self):
        # The box is below the LiDAR's scan plane too (2026-10-20 review).
        room = [(0.0, 0.0), (3.0, 0.0), (3.0, 3.0), (0.0, 3.0)]
        facing_east = (1.5, 1.5, 0.0)                 # backing goes west
        behind = dock.bin_obstacle({'x': 1.05, 'y': 1.5, 'verified': True})
        self.assertIn('placement box', dock.back_path_problem(facing_east, 0.20, room,
                                                              obstacles=[behind]))
        # Released over it, the box is ahead: backing straight away is the way out.
        ahead = dock.bin_obstacle({'x': 1.85, 'y': 1.5, 'verified': True})
        self.assertIsNone(dock.back_path_problem(facing_east, 0.20, room, obstacles=[ahead]))
        # 2026-10-20, second delivery: the unverified map box 0.21 m ahead, 0.05 m left,
        # the front corners just past its centre. Backing away was refused.
        pose = (1.45, 0.652, 0.799)
        unverified = dock.bin_obstacle({'x': 1.5599, 'y': 0.8333, 'verified': False})
        self.assertIsNone(dock.back_path_problem(pose, 0.20, room, obstacles=[unverified]))
        # An unverified position keeps a wider berth.
        side = {'x': 1.30, 'y': 1.95, 'verified': True}
        self.assertIsNone(dock.back_path_problem(facing_east, 0.20, room,
                                                 obstacles=[dock.bin_obstacle(side)]))
        self.assertIsNotNone(dock.back_path_problem(
            facing_east, 0.20, room, obstacles=[dock.bin_obstacle(dict(side, verified=False))]))
        self.assertIsNone(dock.bin_obstacle(None))

    def test_wall_correction_moves_only_small_known_errors(self):
        self.assertEqual(dock.wall_correction(0.480, 0.470), 0.0)
        self.assertAlmostEqual(dock.wall_correction(0.500, 0.470), 0.030)
        self.assertAlmostEqual(dock.wall_correction(0.440, 0.470), -0.030)
        for measured in (0.60, 0.38, float('inf')):
            with self.subTest(measured=measured), self.assertRaises(ValueError):
                dock.wall_correction(measured, 0.470)


if __name__ == '__main__':
    unittest.main()
