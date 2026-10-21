"""Competition loop orchestration: no robot, camera, model or ssh."""
import ast
import importlib.util
import json
import math
from pathlib import Path
import shlex
import tempfile
import unittest
import unittest.mock
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('mission_loop', ROOT / 'integration/mission_loop.py')
mission = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mission)


class FakeReader:
    def __init__(self, frames):
        self.frames, self.stamp = frames, 0.0

    def latest(self):
        self.stamp += 0.1
        return (np.zeros((2, 2, 3), np.uint8) if self.frames else None), self.stamp


class FakeWatcher:
    """Each look: None, an (x, y) calibrated sighting, or {'far': (x, y)} beyond it."""

    def __init__(self, points):
        self.points = list(points)

    def look(self, frame):
        point = self.points.pop(0) if self.points else None
        if point is None:
            return None
        return (point['far'], True) if isinstance(point, dict) else (point, False)


class ClockReader:
    """Frames on a shared clock, so the stream never looks stale."""

    def __init__(self, clock):
        self.clock = clock

    def latest(self):
        self.clock[0] += 0.1
        return np.zeros((2, 2, 3), np.uint8), self.clock[0]

    def close(self):
        pass


class FakeLink:
    """The vision permit link: what was sent, a fixed pose (None: no pose), and the
    patrol process as the Jetson end reports it (`running(host, pid)`)."""

    def __init__(self, pose=(0.0, 0.0, 0.0), running=None):
        self.pose, self.sent, self.allowed, self.closed = pose, [], False, False
        self.running, self.watched, self.dead = running, [], False

    def send(self, allow, age):
        self.sent.append(allow)
        self.allowed = allow

    def pose_now(self):
        return self.pose

    def frame_pose(self, stamp, moving=True):
        # This fake never moves: the frame was taken where the robot stands.
        return None if self.pose is None else (self.pose, 0.0, 0.0)

    def watch(self, pid):
        self.watched.append(pid)

    def patrol_alive(self, pid):
        if self.running is None or pid not in self.watched:
            return None
        return self.running('host', pid)

    def alive(self):
        return not self.dead

    def close(self):
        self.closed = True


LAYOUT = {'frame_id': 'map', 'room_polygon': [[-1, -1], [3, -1], [3, 3], [-1, 3]], 'bin': {
    'pose_map': {'x': 1.76, 'y': 0.88}, 'pose_map_valid_for_current_placement': False,
    'placement_update': {'midpoint_pose_map_candidate': {'x': 1.56, 'y': 0.83}}}}


def resident_reply(host, command):
    """Required fields of ncu's G2 status/health protocol, not an arm-only fake."""
    chassis = {'arm_pose': 'travel', 'moving': False, 'maintenance': False}
    if command == 'health':
        return {'ok': True, 'stack_version': 'v23', 'service_pid': 123,
                'chassis_tcp_ready': True, 'holding': False, 'chassis': chassis}
    return {'chassis': chassis}


class PatrolTests(unittest.TestCase):
    def test_patrol_is_ncus_cycle_detached_with_its_pid(self):
        line = mission.patrol_command()
        self.assertTrue(line.startswith('cd /home/jetson/Documents/deploy_jetson2 && '))
        # $! must be nohup's own PID (it execs bash, which execs python), not a
        # subshell's: only `nohup ...` may be backgrounded.
        self.assertTrue(line.endswith('< /dev/null & echo $!; }'))
        self.assertIn(' && { nohup /bin/bash -c ', line)
        inner = shlex.split(line.split('nohup /bin/bash -c ')[1])[0]
        for part in ('exec /usr/bin/python deploy/ros/g2_map_patrol.py',
                     '--destination patrol-cycle', '--boundary-micro', '--real',
                     '--output /tmp/mission_patrol.json',
                     '--vision-permit /dev/shm/mission_vision_permit.json',
                     '--forward-pulse-max 0.6'):
            self.assertIn(part, inner)

    def test_three_consistent_sightings_confirm_like_the_approach_lock(self):
        confirm = mission.Confirm()
        self.assertFalse(confirm.update((1.0, 0.1), 0.0))
        self.assertFalse(confirm.update((1.02, 0.1), 0.1))
        self.assertTrue(confirm.update((1.01, 0.12), 0.2))
        confirm.reset()
        confirm.update((1.0, 0.1), 0.0)
        confirm.update((1.6, 0.1), 0.1)                       # a jump starts over
        self.assertFalse(confirm.update((1.6, 0.1), 0.2))
        confirm.update(None, 1.5)                             # a long gap forgets
        self.assertFalse(confirm.update((1.6, 0.1), 1.6))

    def test_detection_point_matches_the_approachs_centre_frame(self):
        # Ground = (v / 100, u / 100): a plain scale for the test.
        H = np.array([[0, 0.01, 0], [0.01, 0, 0], [0, 0, 1]], dtype=float)
        hull = np.array([[[0, 0]], [[640, 0]], [[640, 480]], [[0, 480]]], dtype=np.float32)
        x, y = mission.detection_point((300, 100, 340, 200), H, 0.20, hull)
        self.assertAlmostEqual(x, 2.00 - 0.20 + 0.135)
        self.assertAlmostEqual(y, 3.20)
        self.assertIsNone(mission.detection_point((700, 500, 720, 520), H, 0.20, hull))

    def test_arm_camera_region_matches_the_approach_engine(self):
        tree = ast.parse((ROOT / 'integration/sugarbox_rl_approach_final2.py').read_text(
            encoding='utf-8'))
        regions = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                       and ast.unparse(n.targets[0]) == 'REAR_IGNORE_REGIONS')
        self.assertEqual(regions, (mission.ARM_CAMERA_REGION,))

    def test_map_point(self):
        x, y = mission.map_point((1.0, 1.0, math.pi / 2), (1.0, 0.5))
        self.assertAlmostEqual(x, 0.5)
        self.assertAlmostEqual(y, 2.0)

    def test_far_sightings_confirm_with_the_far_seeds_tolerance(self):
        confirm = mission.Confirm()
        self.assertFalse(confirm.update((1.6, 0.4), 0.0, far=True))
        self.assertFalse(confirm.update((1.9, 0.4), 0.1, far=True))     # 0.30 apart: kept
        self.assertTrue(confirm.update((1.7, 0.45), 0.2, far=True))
        self.assertTrue(confirm.far)
        confirm.reset()
        confirm.update((1.0, 0.1), 0.0)
        confirm.update((1.3, 0.1), 0.1)                       # measured: 0.30 starts over
        self.assertFalse(confirm.update((1.3, 0.1), 0.2))

    def test_far_point_extrapolates_within_the_seed_range(self):
        H = np.array([[0, 0.01, 0], [0.01, 0, 0], [0, 0, 1]], dtype=float)
        x, y = mission.far_point((80, 100, 120, 150), H, 0.20)
        self.assertAlmostEqual(x, 1.50 - 0.20 + 0.135)
        self.assertAlmostEqual(y, 1.00)
        self.assertIsNone(mission.far_point((300, 100, 340, 200), H, 0.20))   # 3.7 m away

    def run_patrol(self, points=(), running=None, reports=None, box=None, pose=(0.0, 0.0, 0.0),
                   skip=(), make_room=None, watcher=None, walls=None, clear_frames=1, link=None,
                   guard=None):
        clock, started, stopped = [0.0], [], []
        running = running if running is not None else (lambda host, pid: True)
        reports = iter(reports or [{'ok': True}] * 10)
        link = link or FakeLink(pose, running)
        guard = list(skip) if guard is None else guard
        self.started, self.stopped = started, stopped
        with patch.object(mission, 'START_CLEAR_FRAMES', clear_frames), \
             patch.object(mission, 'PATROL_CHECK_S', 0.0), \
             patch.object(mission, 'start_patrol',
                          side_effect=lambda host, scene: started.append(scene) or 40 + len(started)), \
             patch.object(mission, 'patrol_running', side_effect=running), \
             patch.object(mission, 'stop_patrol',
                          side_effect=lambda host, pid, link=None: stopped.append(pid)), \
             patch.object(mission, 'patrol_report', side_effect=lambda host: next(reports)), \
             patch.object(mission.time, 'sleep'), patch('builtins.print'):
            seen = mission.patrol_until_sugarbox(
                'host', 'scene', ClockReader(clock), watcher or FakeWatcher(points), link, box,
                skip=skip, guard=guard, make_room=make_room, walls=walls, now=lambda: clock[0])
        return seen, started, stopped, link

    def test_a_candidate_parks_the_robot_and_counts_once_confirmed_there(self):
        seen, started, stopped, link = self.run_patrol(
            [None, (1.0, 0.1)] + [(1.0, 0.1)] * 3, pose=(1.0, 0.0, 0.0))
        self.assertEqual((started, stopped), (['scene'], [41]))
        # Granted before the patrol starts, withdrawn on the first candidate, and
        # kept withdrawn while the parked camera confirms.
        self.assertEqual(link.sent[:2], [True, False])
        self.assertNotIn(True, link.sent[2:])
        self.assertEqual([round(v, 6) for v in seen['point']], [1.0, 0.1])
        self.assertEqual([round(v, 6) for v in seen['map']], [2.0, 0.1])

    def test_a_sugarbox_beyond_the_calibrated_area_parks_it_too(self):
        seen, _, stopped, _ = self.run_patrol([None] + [{'far': (1.8, 0.5)}] * 4)
        self.assertEqual([round(v, 6) for v in seen['point']], [1.8, 0.5])
        self.assertEqual(stopped, [41])

    def test_the_patrol_never_starts_with_a_candidate_in_view(self):
        seen, started, stopped, link = self.run_patrol([(1.0, 0.1)] * 4)
        self.assertEqual((started, stopped), ([], []))
        self.assertNotIn(True, link.sent)
        self.assertEqual([round(v, 6) for v in seen['point']], [1.0, 0.1])

    def test_the_delivered_sugarbox_never_stops_the_patrol(self):
        points = [None] + [(1.0, 0.1)] * 3 + [(0.8, -0.3)] * 4
        for verified in (True, False):
            with self.subTest(verified=verified):
                box = {'x': 1.0, 'y': 0.1, 'verified': verified}
                seen, _, stopped, link = self.run_patrol(list(points), box=box)
                self.assertEqual([round(v, 6) for v in seen['point']], [0.8, -0.3])
                self.assertEqual(stopped, [41])
                self.assertEqual(link.sent[:5], [True] * 4 + [False])

    def test_a_sighting_against_a_wall_is_a_clip_not_a_sugarbox(self):
        # 2026-10-19: the black clips holding the wall panels read up to 0.73.
        room = [(-1.0, -1.0), (3.0, -1.0), (3.0, 3.0), (-1.0, 3.0)]
        points = [None] + [(2.9, 0.0)] * 3 + [{'far': (3.5, 0.5)}] * 2 + [(1.0, 0.1)] * 4
        seen, _, stopped, link = self.run_patrol(points, walls=room)
        self.assertEqual([round(v, 6) for v in seen['point']], [1.0, 0.1])
        self.assertEqual(stopped, [41])
        self.assertEqual(link.sent[:7], [True] * 6 + [False])

    def test_a_spot_where_grasping_failed_twice_is_left_alone(self):
        # A skipped box to the side may be passed over; one directly ahead is
        # now a collision hazard, not a clear frame (test_flow_contracts).
        points = [None] + [(1.0, 0.5)] * 3 + [(0.8, -0.3)] * 4
        seen, _, stopped, _ = self.run_patrol(points, skip=[(1.05, 0.5)])
        self.assertEqual([round(v, 6) for v in seen['point']], [0.8, -0.3])
        self.assertEqual(stopped, [41])

    def test_an_unconfirmed_candidate_is_passed_over_and_the_patrol_goes_on(self):
        seen, started, stopped, _ = self.run_patrol(
            [None, (1.0, 0.1)], running=lambda host, pid: pid == 41)
        self.assertIsNone(seen)                       # then two whole laps with nothing
        self.assertEqual((len(started), stopped), (3, [41]))
        # A clip passes over only sightings in its band by a wall (2026-10-20 review).
        room = [(-1.0, -1.0), (3.0, -1.0), (3.0, 1.2), (-1.0, 1.2)]
        self.assertTrue(mission.passed_over((1.0, 1.0), None, (), [(1.1, 1.0)], room))
        self.assertFalse(mission.passed_over((1.0, 0.8), None, (), [(1.1, 1.0)], room))
        self.assertFalse(mission.passed_over((1.0, 1.0), None, (), [(1.5, 1.0)], room))
        self.assertFalse(mission.passed_over(None, {'x': 0, 'y': 0}, [(0, 0)], [(0, 0)]))

    def test_a_camera_that_keeps_seeing_unconfirmed_candidates_stops_the_mission(self):
        class Flicker:
            """A candidate whenever the patrol drives, each somewhere new (so none is
            passed over as already rejected), gone once the robot parks."""
            def look(watcher, frame):
                if len(self.started) > len(self.stopped):
                    return (1.0 + 0.5 * len(self.stopped), 0.1), False
                return None
        with patch.object(mission, 'MAX_FALSE_STOPS_PER_LAP', 1), \
             patch.object(mission, 'CONFIRM_S', 0.6), self.assertRaises(mission.MissionError):
            self.run_patrol(watcher=Flicker())
        self.assertEqual(len(self.stopped), 2)

    def test_the_patrol_starts_only_after_several_clear_frames(self):
        # One frame where YOLO misses a box right ahead must not start the patrol.
        points = [None, (1.0, 0.1), None, None, None] + [(1.0, 0.1)] * 4
        seen, started, _, _ = self.run_patrol(points, clear_frames=3)
        self.assertEqual(started, [])
        self.assertEqual([round(v, 6) for v in seen['point']], [1.0, 0.1])
        seen, started, _, _ = self.run_patrol([None] * 3 + [(1.0, 0.1)] * 4, clear_frames=3)
        self.assertEqual(started, ['scene'])

    def test_no_permit_and_no_start_without_a_pose(self):
        class Pose(FakeLink):
            """No pose for the first looks, then one."""
            def __init__(link):
                super().__init__()
                link.asked = 0

            def pose_now(link):
                link.asked += 1
                return None if 2 <= link.asked <= 5 else (0.0, 0.0, 0.0)
        link = Pose()
        seen, started, _, _ = self.run_patrol([None] * 7 + [(1.0, 0.1)] * 4, link=link)
        # Four clear frames without a pose sent nothing; the three after it, a permit.
        self.assertEqual(link.sent[:4], [True] * 3 + [False])
        self.assertEqual(started, ['scene'])
        self.assertIsNotNone(seen)
        with patch.object(mission.time, 'sleep'), self.assertRaises(mission.MissionError):
            clock = iter(range(100))
            mission.patrol_until_sugarbox('host', 'scene', ClockReader([0.0]), FakeWatcher([]),
                                          FakeLink(None), now=lambda: next(clock))

    def test_a_box_the_camera_cannot_vouch_for_in_the_way_ends_the_mission(self):
        # 2026-10-19: the patrol pushed a sugarbox it could not see and kept going.
        # The corridor ahead is wider on the right (the cables reach y -0.29 m), and
        # the turn-in-place circle counts as well.
        for spot, ends in (((0.6, 0.1), True), ((0.3, -0.35), True), ((0.0, -0.38), True),
                           ((0.6, 0.5), False), ((1.2, 0.0), False), ((-0.5, 0.0), False),
                           ((0.3, 0.36), False)):
            with self.subTest(spot=spot):
                if ends:
                    with self.assertRaises(mission.MissionError) as raised:
                        self.run_patrol([None] * 3, guard=[spot])
                    self.assertIn('cannot vouch for', str(raised.exception))
                    self.assertEqual(self.started, [])
                else:
                    seen, _, _, _ = self.run_patrol([None] + [(1.0, -0.25)] * 4, guard=[spot])
                    self.assertIsNotNone(seen)
        spot, x, y = mission.box_in_the_way((1.0, 1.0, math.pi / 2), [(1.0, 1.7)])  # facing +y
        self.assertAlmostEqual(x, 0.7)
        self.assertIsNone(mission.box_in_the_way((1.0, 1.0, math.pi / 2), [(1.7, 1.0)]))

    def test_an_unconfirmed_candidate_seen_this_near_is_guarded_not_passed_over(self):
        # Parked this near, the gripper may hide it: not seeing it again says nothing.
        with patch.object(mission, 'CONFIRM_S', 0.6), \
                self.assertRaises(mission.MissionError) as raised:
            self.run_patrol([None, (0.55, 0.05)] + [None] * 20)
        self.assertIn('cannot vouch for', str(raised.exception))
        self.assertEqual(self.stopped, [41])            # stopped there, never restarted

    def test_every_exclusion_must_hold_with_the_spot_error_bound(self):
        # 2026-10-20 review: a frame taken mid-turn and placed with the later pose
        # moved a real box 0.4 m, into the wall-clip zone.
        room = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
        self.assertTrue(mission.passed_over((1.0, 0.10), None, (), (), room, error=0.0))
        self.assertTrue(mission.passed_over((1.0, 0.09), None, (), (), room, error=0.05))
        self.assertFalse(mission.passed_over((1.0, 0.12), None, (), (), room, error=0.05))
        self.assertFalse(mission.passed_over((1.0, 0.05), None, (), (), room, error=0.2))
        clips = [(1.0, 1.85)]                           # in the band by the y = 2 wall
        self.assertTrue(mission.passed_over((1.2, 1.85), None, (), clips, room, error=0.0))
        self.assertFalse(mission.passed_over((1.2, 1.85), None, (), clips, room, error=0.08))

    def test_a_frame_taken_mid_turn_cannot_place_a_sighting(self):
        class Turning(FakeLink):
            def frame_pose(link, stamp, moving=True):
                return link.pose, 0.0, 0.5
        spot, error = mission.sighting_spot(Turning(), 1.0, (1.0, 0.0))
        self.assertAlmostEqual(error, 0.5)
        self.assertGreater(error, mission.SPOT_ERROR_MAX_M)

    def test_a_far_sighting_mid_turn_waits_for_a_placeable_frame_before_stopping(self):
        # A wall clip seen while the patrol turns at a corner: the next frame, with
        # the robot settled, places it by the wall and the patrol never stops.
        class Corner(FakeLink):
            def __init__(link):
                super().__init__()
                link.asked = 0

            def frame_pose(link, stamp, moving=True):
                link.asked += 1
                return link.pose, 0.0, (0.5 if link.asked <= 2 else 0.0)
        room = [(-1.0, -1.0), (3.0, -1.0), (3.0, 1.05), (-1.0, 1.05)]
        corner = Corner()
        seen, started, stopped, link = self.run_patrol(
            [None, (1.5, 0.95), (1.5, 0.95), (1.5, 0.95), None, None],
            link=corner, walls=room, running=lambda host, pid: len(corner.sent) < 9)
        self.assertEqual(stopped, [])                   # never parked for the clip
        self.assertNotIn(False, link.sent)

    def test_a_far_sighting_seen_on_and_off_mid_turn_still_stops_in_time(self):
        # 2026-10-20 review: a miss between two unplaceable hits must not restart
        # the wait nor renew the permit.
        class Turning(FakeLink):
            def frame_pose(link, stamp, moving=True):
                return link.pose, 0.0, 0.5
        seen, started, stopped, link = self.run_patrol(
            [None] * 5 + [(1.5, 0.0), None] * 6 + [(1.5, 0.0)] * 4, link=Turning(),
            clear_frames=mission.START_CLEAR_FRAMES)
        first = 5                                       # the permits before it was seen
        self.assertNotIn(True, link.sent[first:])
        self.assertEqual(stopped, [41])

    def test_an_unconfirmed_candidate_by_a_wall_becomes_a_clip_passed_over(self):
        room = [(-1.0, -1.0), (3.0, -1.0), (3.0, 1.2), (-1.0, 1.2)]
        with patch.object(mission, 'CONFIRM_S', 0.6):
            seen, started, stopped, _ = self.run_patrol(
                [None, (1.0, 1.0)] + [None] * 6 + [(1.0, 1.0)] * 3 + [(1.0, -0.2)] * 4,
                walls=room)
        # Parked once for the clip; seen again it is passed over and the patrol
        # restarts (42), stopping next for the real sugarbox.
        self.assertEqual(stopped, [41, 42])
        self.assertEqual([round(v, 6) for v in seen['point']], [1.0, -0.2])

    def test_the_robot_end_reports_the_patrol_so_the_loop_never_waits_on_ssh(self):
        asked, clock = [], [0.0]
        link = FakeLink(running=lambda host, pid: clock[0] < 2.0)
        with patch.object(mission, 'START_CLEAR_FRAMES', 1), \
             patch.object(mission, 'start_patrol', return_value=41), \
             patch.object(mission, 'patrol_running',
                          side_effect=lambda host, pid: asked.append(clock[0]) or False), \
             patch.object(mission, 'patrol_report', return_value={'ok': True}), \
             patch.object(mission, 'stop_patrol'), \
             patch.object(mission.time, 'sleep'), patch('builtins.print'):
            seen = mission.patrol_until_sugarbox(
                'host', 'scene', ClockReader(clock), FakeWatcher([None] * 60), link,
                now=lambda: clock[0])
        self.assertIsNone(seen)                         # two laps ended, as the link said
        self.assertEqual(link.watched, [41, 41])
        self.assertEqual(asked, [])                     # no ssh round trip in the loop

    def test_without_a_report_from_the_robot_ssh_is_asked_once_a_second(self):
        asked, clock = [], [0.0]
        with patch.object(mission, 'START_CLEAR_FRAMES', 1), \
             patch.object(mission, 'start_patrol', return_value=41), \
             patch.object(mission, 'patrol_running',
                          side_effect=lambda host, pid: asked.append(clock[0]) or True), \
             patch.object(mission, 'stop_patrol'), \
             patch.object(mission.time, 'sleep'), patch('builtins.print'):
            mission.patrol_until_sugarbox(
                'host', 'scene', ClockReader(clock), FakeWatcher([None] * 30 + [(1.0, 0.1)] * 4),
                FakeLink(), now=lambda: clock[0])
        self.assertLessEqual(len(asked), 4)             # 3 s of frames at 10 Hz

    def test_a_lost_pose_or_a_dead_link_ends_the_wait(self):
        class Lost(FakeLink):
            def __init__(link):
                super().__init__()
                link.asked = 0

            def pose_now(link):
                link.asked += 1
                return (0.0, 0.0, 0.0) if link.asked == 1 else None
        with self.assertRaises(mission.MissionError) as raised:
            self.run_patrol([None] * 200, link=Lost())
        self.assertIn('localization lost', str(raised.exception))
        dead = FakeLink()
        dead.dead = True
        with self.assertRaises(mission.MissionError) as raised:
            self.run_patrol([None] * 5, link=dead)
        self.assertIn('link to the robot ended', str(raised.exception))

    def test_the_patrol_drives_ncus_own_pulses(self):
        self.assertEqual(mission.PATROL_FORWARD_PULSE_S, 0.6)

    def test_finished_laps_restart_and_two_empty_laps_end_the_search(self):
        seen, started, _, _ = self.run_patrol([None] * 9, running=lambda host, pid: False)
        self.assertIsNone(seen)
        self.assertEqual(len(started), mission.EMPTY_LAPS_TO_END)

    def test_a_failed_patrol_stops_the_mission(self):
        with self.assertRaises(mission.MissionError):
            self.run_patrol([None] * 5, running=lambda host, pid: False,
                            reports=[{'ok': False, 'error': 'AMCL uncertainty', 'pulses': 7}])

    def test_a_refusal_to_start_makes_room_and_asks_again(self):
        refused = {'ok': False, 'pulses': 0, 'error': 'no reachable entry to patrol route'}
        alive, room = iter([False] + [True] * 9), []
        seen, started, stopped, _ = self.run_patrol(
            [None, None, (1.0, 0.1)] + [(1.0, 0.1)] * 3, running=lambda host, pid: next(alive),
            reports=[refused], make_room=lambda: room.append('back') or True)
        self.assertEqual((started, stopped, room), (['scene', 'scene'], [42], ['back']))
        with self.assertRaises(mission.MissionError):
            self.run_patrol([None] * 5, running=lambda host, pid: False, reports=[refused] * 3,
                            make_room=lambda: False)

    def test_a_refusal_means_it_never_moved(self):
        self.assertTrue(mission.patrol_refused(
            {'ok': False, 'pulses': 0, 'error': 'start outside navigation mask'}))
        self.assertFalse(mission.patrol_refused(
            {'ok': False, 'pulses': 3, 'error': 'start outside navigation mask'}))
        self.assertFalse(mission.patrol_refused({'ok': False, 'pulses': 0, 'error': 'AMCL'}))
        self.assertFalse(mission.patrol_refused({'ok': True}))

    def test_no_frames_never_starts_the_patrol(self):
        with patch.object(mission, 'start_patrol') as starting, \
             patch.object(mission.time, 'sleep'), self.assertRaises(mission.MissionError):
            clock = iter(range(100))
            mission.patrol_until_sugarbox('host', 'scene', FakeReader(False), FakeWatcher([]),
                                          FakeLink(), now=lambda: next(clock))
        starting.assert_not_called()


class VisionLinkTests(unittest.TestCase):
    class Stdin:
        def __init__(self, broken=False):
            self.lines, self.broken, self.closed = [], broken, False

        def write(self, text):
            if self.broken:
                raise OSError('broken pipe')
            self.lines.append(text)

        def flush(self):
            pass

        def close(self):
            self.closed = True

    class Proc:
        def __init__(self, stdin, lines):
            self.stdin, self.stdout = stdin, iter(lines)

        def wait(self, timeout=None):
            return 0

    def link(self, lines=(), broken=False):
        stdin, calls = self.Stdin(broken), []

        def popen(command, **kwargs):
            calls.append(command)
            return self.Proc(stdin, list(lines))
        link = mission.VisionLink('10.0.0.1', popen=popen, clock=lambda: 0.0)
        return link, stdin, calls[0]

    def wait_for_pose(self, link):
        for _ in range(200):
            if link.pose is not None:
                return
            mission.time.sleep(0.005)

    def test_permits_go_out_one_line_each_to_the_jetson_end(self):
        link, stdin, command = self.link()
        self.assertEqual(command[:5], ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8'])
        self.assertIn('python3 -u -c ', command[-1])
        self.assertIn(mission.PERMIT_PATH, command[-1])
        link.send(True, 0.4)
        link.send(False, -1.0)                        # a negative age never goes out
        self.assertEqual([json.loads(line) for line in stdin.lines],
                         [{'allow': True, 'age': 0.4}, {'allow': False, 'age': 0.0}])
        self.assertFalse(link.allowed)
        link.close()
        self.assertTrue(stdin.closed)                 # the Jetson end then denies

    def test_only_a_fresh_ready_pose_is_kept(self):
        good = json.dumps({'t': 10.0, 'ready': True, 'stamp': 9.9, 'pose': [1, 2, 0.5]})
        stale = json.dumps({'t': 10.0, 'ready': True, 'stamp': 7.0, 'pose': [5, 5, 0]})
        lost = json.dumps({'t': 10.0, 'ready': False, 'error': 'no file'})
        link, _, _ = self.link([stale + '\n', lost + '\n', 'junk\n', good + '\n'])
        self.wait_for_pose(link)
        self.assertEqual(link.pose_now(), (1.0, 2.0, 0.5))
        link, _, _ = self.link([stale + '\n'])
        self.assertIsNone(link.pose_now())

    def test_a_lost_link_stops_the_mission(self):
        link, _, _ = self.link(broken=True)
        with self.assertRaises(mission.MissionError):
            link.send(True, 0.3)

    def test_the_patrol_report_counts_only_while_fresh(self):
        clock = [0.0]
        line = json.dumps({'t': 10.0, 'ready': True, 'stamp': 9.9, 'pose': [1, 2, 0.5],
                           'patrol': {'pid': 41, 'alive': False}}) + '\n'
        link = mission.VisionLink('10.0.0.1', popen=lambda command, **kw: self.Proc(
            self.Stdin(), [line]), clock=lambda: clock[0])
        self.wait_for_pose(link)
        self.assertIs(link.patrol_alive(41), False)
        self.assertIsNone(link.patrol_alive(42))          # another patrol: no word on it
        clock[0] += mission.PATROL_REPORT_MAX_AGE_S + 0.1
        self.assertIsNone(link.patrol_alive(41))          # stalled link: ask ssh instead


class StopPatrolTests(unittest.TestCase):
    def stop(self, link, ssh_alive=()):
        asked, answers = [], iter(ssh_alive)
        with patch.object(mission, 'ssh') as ssh, \
             patch.object(mission, 'patrol_running',
                          side_effect=lambda host, pid: asked.append(pid) or next(answers)), \
             patch.object(mission.time, 'sleep'):
            mission.stop_patrol('host', 41, link=link)
        ssh.assert_called_once_with('host', 'kill -TERM 41')
        return asked

    def test_the_jetson_ends_word_replaces_the_ssh_polls(self):
        # 2026-10-21: each poll was a new ssh connection, ~0.5-1 s on Windows.
        words = iter([True, True, False])
        link = Mock(patrol_alive=lambda pid: next(words))
        self.assertEqual(self.stop(link), [])

    def test_without_a_fresh_word_ssh_is_asked_as_before(self):
        self.assertEqual(self.stop(Mock(patrol_alive=lambda pid: None), [True, False]), [41, 41])
        self.assertEqual(self.stop(None, [False]), [41])


class RoomTests(unittest.TestCase):
    def test_backs_out_then_a_step_more_up_to_the_limit(self):
        backs = []
        room = mission.Room('host', back=lambda host, m: backs.append(round(m, 3)))
        self.assertFalse(room.make())                 # the start: back into the field by hand
        room.leave()
        self.assertTrue(room.make())
        self.assertFalse(room.make())                 # 0.20 + 0.10 = 0.30 m in all
        self.assertEqual(backs, [0.2, 0.1])
        room.reset()
        self.assertFalse(room.make())


class StateTests(unittest.TestCase):
    def state(self, chassis=None, holding=True, error=None):
        status = {'chassis': dict({'arm_busy': False, 'moving': False, 'arm_pose': 'travel'},
                                  **(chassis or {}))}

        def ctl(host, command):
            if error:
                raise error
            return status if command == 'status' else {'holding': holding}
        with patch.object(mission.runner, 'ctl', side_effect=ctl), patch('builtins.print'):
            return mission.arm_state('host')

    def test_only_a_settled_known_arm_counts(self):
        self.assertEqual(self.state(), 'holding')
        self.assertEqual(self.state({'arm_pose': 'e1'}, holding=False), 'empty')
        for chassis, holding in (({'arm_busy': True}, True), ({'moving': True}, True),
                                 ({'arm_pose': 'unknown'}, False), ({}, None)):
            with self.subTest(chassis=chassis, holding=holding):
                self.assertEqual(self.state(chassis, holding), 'unknown')
        self.assertEqual(self.state(error=RuntimeError('graspctl failed')), 'unknown')

    def test_a_spot_counts_after_two_failures_close_together(self):
        self.assertEqual(mission.tried_out([(1.0, 1.0)]), [])
        self.assertEqual(len(mission.tried_out([(1.0, 1.0), (1.1, 1.0)])), 2)
        self.assertEqual(mission.tried_out([(1.0, 1.0), (2.0, 2.0)]), [])

    def test_the_seed_is_reprojected_from_the_pose_after_stopping(self):
        sighting = {'point': (1.0, 0.0), 'map': (2.0, 1.0)}
        with patch.object(mission.dock, 'read_localization', return_value={
                'ready': True, 'age': 0.1, 'state': {'pose': [1.5, 1.0, 0.0]}}):
            self.assertEqual([round(v, 6) for v in mission.approach_seed('h', sighting)],
                             [0.5, 0.0])
        with patch.object(mission.dock, 'read_localization', return_value=None), \
             patch('builtins.print'):
            self.assertEqual(mission.approach_seed('h', sighting), (1.0, 0.0))
        self.assertIsNone(mission.approach_seed('h', {'point': (0.1, 0.0), 'map': None}))


class SearchTests(unittest.TestCase):
    def search(self, seen_after_turns):
        clock, turned = [0.0], []

        class Watcher:
            def look(self, frame):
                return ((1.5, 0.2), False) if len(turned) >= seen_after_turns else None
        with patch('builtins.print'), patch.object(mission.dock, 'read_localization', return_value={
                'ready': True, 'age': 0.0, 'state': {'pose': [0, 0, 0], 'turn_clear': True}}):
            found = mission.search_placement('host', ClockReader(clock), Watcher(),
                                             turn=lambda host, r: turned.append(r),
                                             now=lambda: clock[0])
        return found, turned

    def test_turns_until_the_box_shows(self):
        found, turned = self.search(2)
        self.assertTrue(found)
        self.assertEqual(turned, [mission.SEARCH_STEP_RAD] * 2)

    def test_a_receiver_just_started_gets_its_first_frame_first(self):
        # 2026-10-20 review: the look-around failed at once, 'rear camera stream lost'.
        clock, turned = [100.0], []

        class Starting(ClockReader):
            def __init__(reader):
                super().__init__(clock)
                reader.calls = 0

            def latest(reader):
                reader.calls += 1
                if reader.calls <= 5:
                    clock[0] += 0.1
                    return None, 0.0
                return super().latest()

        class Watcher:
            def look(self, frame):
                return (1.5, 0.2), False
        with patch('builtins.print'), patch.object(mission.time, 'sleep'):
            self.assertTrue(mission.search_placement(
                'host', Starting(), Watcher(), turn=lambda host, r: turned.append(r),
                now=lambda: clock[0]))
        self.assertEqual(turned, [])

    def test_gives_up_after_about_a_full_circle(self):
        found, turned = self.search(99)
        self.assertFalse(found)
        self.assertEqual(len(turned), mission.SEARCH_STEPS)
        self.assertGreaterEqual(math.degrees(sum(turned)), 360)


class StageTests(unittest.TestCase):
    def setUp(self):
        self.args = mission.argparse.Namespace(host='10.0.0.1', wall_m=None,
                                               homography=Path('h.json'), gstreamer=None)
        self.calls = []

    def stage(self, codes):
        codes = iter(codes)
        return lambda host, argv, log_path=None: self.calls.append(
            (argv[1], '--turn-to-box' in argv, Path(log_path).name, list(argv))) or next(codes)

    def fetch(self, code, state, camera='ok'):
        with tempfile.TemporaryDirectory() as folder, \
             patch.object(mission, 'ensure_arm_camera', return_value=None), \
             patch.object(mission, 'unlock'), \
             patch.object(mission, 'approach_seed', return_value=(1.2, -0.3)), \
             patch.object(mission, 'run_stage', side_effect=self.stage([code])), \
             patch.object(mission, 'arm_state', return_value=state), \
             patch.object(mission, 'arm_camera_state', return_value=camera), \
             patch.object(mission, 'to_travel', side_effect=lambda h: self.calls.append('travel')):
            return mission.fetch_sugarbox(self.args, {'point': (1.2, -0.3), 'map': None},
                                          Path(folder), 3)

    def test_a_held_object_goes_on_to_the_placement(self):
        self.assertEqual(self.fetch(0, 'holding'), 'held')
        argv = self.calls[0][3]
        self.assertEqual(argv[argv.index('--seed') + 1:argv.index('--seed') + 3],
                         ['1.200', '-0.300'])
        self.assertEqual(self.calls[0][2], 'grasp_3.log')

    def test_a_miss_with_the_robot_at_the_box_says_so(self):
        self.assertEqual(self.fetch(mission.runner.AT_BOX_EXIT, 'empty'), 'missed_at_box')

    def test_any_other_clean_failure_is_a_miss_that_patrols_on(self):
        # The user, 2026-10-20 (option C): the 10-19 table, back out and patrol on.
        for code in (2, 3, 4, 5, 7, -1):
            with self.subTest(code=code):
                self.calls = []
                self.assertEqual(self.fetch(code, 'empty'), 'missed')
                self.assertIn('travel', self.calls)

    def test_anything_doubtful_stops(self):
        for code, state, camera in ((2, 'unknown', 'ok'), (0, 'empty', 'ok'),
                                    (2, 'empty', 'unplugged')):
            with self.subTest(code=code, state=state, camera=camera), \
                 self.assertRaises(mission.MissionError):
                self.fetch(code, state, camera)
        self.assertNotIn('travel', self.calls)

    def place(self, codes, states, found=True):
        states = iter(states)
        with tempfile.TemporaryDirectory() as folder, \
             patch.object(mission, 'run_stage', side_effect=self.stage(codes)), \
             patch.object(mission, 'arm_state', side_effect=lambda h: next(states)), \
             patch.object(mission, 'to_travel'), patch('builtins.print'), \
             patch.object(mission, 'search_placement',
                          side_effect=lambda h, r, w: self.calls.append('search') or found):
            mission.place_held(self.args, Path(folder) / 'scene_layout.json', Path(folder), 2,
                               lambda: ClockReader([0.0]), lambda: 'watcher')

    def test_a_placement_turns_toward_the_box_first(self):
        self.place([0], ['empty'])
        self.assertEqual([c[:3] for c in self.calls], [('placement', True, 'place_2.log')])

    def test_still_holding_after_a_failure_looks_around_once_and_retries(self):
        self.place([2, 0], ['holding', 'empty'])
        self.assertEqual([c if c == 'search' else c[:3] for c in self.calls],
                         [('placement', True, 'place_2.log'), 'search',
                          ('placement', False, 'place_2_retry.log')])
        for codes, states, found in (([2], ['holding'], False),
                                     ([2, 2], ['holding', 'holding'], True),
                                     ([2], ['empty'], True), ([2], ['unknown'], True)):
            with self.subTest(codes=codes, states=states, found=found), \
                 self.assertRaises(mission.MissionError):
                self.calls = []
                self.place(codes, states, found)


class MissionTests(unittest.TestCase):
    """The patrol loop's order, with every stage faked."""

    def run_mission(self, sightings, grasps, argv=(), place_error=None, at_pose=None):
        order, links, self.guards = [], [], []
        self.backs = backs = []
        sightings, grasps = iter(sightings), iter(grasps)
        args = mission.argparse.Namespace(host='10.0.0.1', scene='scene', homography=Path('h'),
                                          gstreamer=None, wall_m=None, max_deliveries=None)
        for key, value in argv:
            setattr(args, key, value)

        class FakeRoom:
            def leave(self, metres=mission.BACK_OUT_M):
                order.append('leave')
                backs.append(metres)

            def make(self):
                return False

            def reset(self):
                order.append('reset')

        def patrol(host, scene, reader, watcher, link, box, skip=(), guard=(), blind=(),
                   make_room=None, walls=None):
            self.assertEqual(walls, [(-1, -1), (3, -1), (3, 3), (-1, 3)])
            order.append(('patrol', len(skip)))
            self.guards.append([tuple(round(v, 6) for v in g) for g in blind])
            links.append(link)
            return next(sightings)

        def place(*a):
            order.append('place')
            if place_error:
                raise place_error
        with tempfile.TemporaryDirectory() as folder, \
             patch.object(mission.dock, 'fetch_layout', return_value=LAYOUT), \
             patch.object(mission, 'stream_running', return_value=True), \
             patch.object(mission, 'SugarboxWatcher'), \
             patch('integration.rear_stream.StreamReader'), \
             patch.object(mission, 'patrol_until_sugarbox', side_effect=patrol), \
             patch.object(mission, 'fetch_sugarbox',
                          side_effect=lambda *a: order.append('grasp') or next(grasps)), \
             patch.object(mission, 'place_held', side_effect=place), \
             patch.object(mission, 'box_left_at', side_effect=lambda host, ahead=None: at_pose), \
             patch.object(mission.runner, 'stop') as stopping, patch('builtins.print'):
            code = mission.patrol_mission(args, '10.0.0.2', Path(folder), room=FakeRoom(),
                                          new_link=lambda host: FakeLink())
            copied = json.loads((Path(folder) / 'scene_layout.json').read_text(encoding='utf-8'))
        stopping.assert_called_once()
        self.assertTrue(links and all(link.closed for link in links))
        self.assertEqual(copied, LAYOUT)               # what this run turned by, kept
        return code, order

    def test_it_runs_until_two_empty_laps(self):
        spot = {'point': (1.0, 0.0), 'map': (0.5, 0.5)}
        code, order = self.run_mission([spot, spot, None], ['held', 'held'])
        self.assertEqual(code, 0)
        self.assertEqual(order, [('patrol', 0), 'reset', 'grasp', 'place', 'leave',
                                 ('patrol', 0), 'reset', 'grasp', 'place', 'leave',
                                 ('patrol', 0)])

    def test_a_missed_grasp_backs_out_and_a_spot_missed_twice_is_skipped(self):
        spot = {'point': (1.0, 0.0), 'map': (0.5, 0.5)}
        code, order = self.run_mission([spot, spot, None], ['missed_at_box', 'missed_at_box'])
        self.assertEqual(code, 0)
        self.assertEqual(order, [('patrol', 0), 'reset', 'grasp', 'leave',
                                 ('patrol', 0), 'reset', 'grasp', 'leave',
                                 ('patrol', 2)])

    def test_a_miss_at_the_box_backs_out_far_enough_to_see_it_again(self):
        # The user, 2026-10-20 (option C): 0.28 + 0.45 = 0.73 m, past the blind zone;
        # guarded within the blind zone meanwhile, from where the approach left it.
        spot = {'point': (1.0, 0.0), 'map': (0.5, 0.5)}
        code, order = self.run_mission([spot, None], ['missed_at_box'], at_pose=(0.62, 0.48))
        self.assertEqual(code, 0)
        self.assertEqual(self.backs, [mission.AT_BOX_BACK_OUT_M])
        self.assertEqual(self.guards, [[], [(0.62, 0.48)]])
        code, order = self.run_mission([spot, None], ['missed'])
        self.assertEqual((code, self.backs), (0, [mission.BACK_OUT_M]))
        self.assertEqual(self.guards, [[], []])         # stopped short: the camera sees it

    def test_the_back_out_puts_the_box_where_the_camera_measures_it(self):
        self.assertEqual(mission.at_box_back_out(None), 0.45)          # E1 missed at 0.28
        self.assertEqual(mission.at_box_back_out((0.40, 0.0)), 0.33)   # held on the way in
        self.assertEqual(mission.at_box_back_out((0.70, 0.0)), mission.BACK_OUT_M)
        self.assertEqual(mission.at_box_back_out((0.05, 0.0)), 0.60)   # never more than 0.60
        self.assertGreater(mission.AT_BOX_SEEN_M, mission.GUARD_BLIND_M)

    def test_the_spot_comes_from_where_the_run_left_the_box(self):
        report = mission.runner.AT_BOX_REPORT
        report.write_text(json.dumps({'x': 0.41, 'y': -0.05}), encoding='utf-8')
        try:
            self.assertEqual(mission.at_box_ahead(), (0.41, -0.05))
        finally:
            report.unlink()
        self.assertIsNone(mission.at_box_ahead())

    def test_a_worker_ended_by_the_stage_stops_the_mission(self):
        args = mission.argparse.Namespace(host='10.0.0.1', wall_m=None,
                                          homography=Path('h.json'), gstreamer=None)
        dead = unittest.mock.Mock(usable=lambda: False)
        with tempfile.TemporaryDirectory() as folder, \
             patch.object(mission, '_runtime', dead), \
             patch.object(mission, 'ensure_arm_camera', return_value=None), \
             patch.object(mission, 'unlock'), \
             patch.object(mission, 'approach_seed', return_value=(1.2, -0.3)), \
             patch.object(mission, 'run_stage', return_value=2), \
             patch.object(mission, 'to_travel') as travel, \
             self.assertRaisesRegex(mission.MissionError, 'model worker was ended'):
            mission.fetch_sugarbox(args, {'point': (1.2, -0.3), 'map': None}, Path(folder), 1)
        travel.assert_not_called()

    def test_a_box_grasped_at_last_is_no_longer_guarded(self):
        spot = {'point': (1.0, 0.0), 'map': (0.6, 0.5)}
        code, order = self.run_mission([spot, spot, None], ['missed_at_box', 'held'],
                                       at_pose=(0.62, 0.48))
        self.assertEqual(code, 0)
        self.assertEqual(self.guards, [[], [(0.62, 0.48)], []])

    def test_max_deliveries_stops_after_the_last_one(self):
        spot = {'point': (1.0, 0.0), 'map': None}
        code, order = self.run_mission([spot], ['held'], argv=[('max_deliveries', 1)])
        self.assertEqual((code, order[-1], order.count('place')), (0, 'leave', 1))

    def test_no_room_walls_no_patrol(self):
        layout = {k: v for k, v in LAYOUT.items() if k != 'room_polygon'}
        args = mission.argparse.Namespace(host='10.0.0.1', scene='scene', homography=Path('h'),
                                          gstreamer=None, wall_m=None, max_deliveries=None)
        with tempfile.TemporaryDirectory() as folder,              patch.object(mission.dock, 'fetch_layout', return_value=layout),              patch.object(mission, 'patrol_until_sugarbox') as patrolling,              patch.object(mission.runner, 'stop'), patch('builtins.print'):
            self.assertEqual(mission.patrol_mission(args, '10.0.0.2', Path(folder)), 2)
        patrolling.assert_not_called()

    def test_a_doubtful_stage_stops_the_loop_where_it_is(self):
        spot = {'point': (1.0, 0.0), 'map': None}
        code, order = self.run_mission([spot], ['held'],
                                       place_error=mission.MissionError('arm state unknown'))
        self.assertEqual(code, 2)
        self.assertNotIn('leave', order)


class MainTests(unittest.TestCase):
    def test_default_is_a_preflight_without_motion(self):
        with patch.object(mission, 'preflight', return_value=[]), \
             patch.object(mission, 'pc_address', return_value='10.0.0.2'), \
             patch.object(mission, 'start_patrol') as starting, \
             patch.object(mission, 'run_stage') as stage:
            self.assertEqual(mission.main(['--host', '10.0.0.1']), 0)
        starting.assert_not_called()
        stage.assert_not_called()

    def test_blocked_preflight_and_missing_confirmation_never_move(self):
        with patch.object(mission, 'preflight', return_value=['localization: not localized']), \
             patch.object(mission, 'run_stage') as stage:
            self.assertEqual(mission.main(['--host', '10.0.0.1', '--real',
                                           '--i-am-beside-the-robot']), 2)
        with patch.object(mission, 'preflight', return_value=[]), \
             patch.object(mission, 'pc_address', return_value='10.0.0.2'), \
             patch.object(mission, 'run_stage') as stage:
            self.assertEqual(mission.main(['--host', '10.0.0.1', '--real']), 3)
        stage.assert_not_called()

    def test_real_patrol_mode_relocalizes_then_runs_the_patrol_mission(self):
        order = []
        with patch.object(mission, 'preflight', return_value=[]), \
             patch.object(mission, 'pc_address', return_value='10.0.0.2'), \
             patch.object(mission, 'relocalize', side_effect=lambda host: order.append('reloc')), \
             patch.object(mission, 'patrol_mission',
                          side_effect=lambda *a: order.append('patrol') or 0) as patrolling, \
             patch.object(mission, 'grasp_and_place') as one, patch('builtins.print'):
            self.assertEqual(mission.main(['--host', '10.0.0.1', '--real',
                                           '--i-am-beside-the-robot']), 0)
        self.assertEqual(order, ['reloc', 'patrol'])        # the user, 2026-10-21
        self.assertIsNone(patrolling.call_args.args[0].max_deliveries)   # until laps run dry
        one.assert_not_called()

    def test_a_failed_relocalization_never_starts_the_patrol(self):
        with patch.object(mission, 'preflight', return_value=[]), \
             patch.object(mission, 'pc_address', return_value='10.0.0.2'), \
             patch.object(mission, 'relocalize',
                          side_effect=mission.RelocalizeError('AMBIGUOUS: markers [0]')), \
             patch.object(mission, 'patrol_mission') as patrolling, \
             patch('builtins.print') as printed:
            self.assertEqual(mission.main(['--host', '10.0.0.1', '--real',
                                           '--i-am-beside-the-robot']), 2)
        patrolling.assert_not_called()
        self.assertIn('AMBIGUOUS', ' '.join(str(c) for c in printed.call_args_list))

    def test_no_patrol_and_the_dry_check_do_not_relocalize(self):
        with patch.object(mission, 'preflight', return_value=[]), \
             patch.object(mission, 'pc_address', return_value='10.0.0.2'), \
             patch.object(mission, 'relocalize') as reloc, \
             patch.object(mission, 'grasp_and_place', return_value=0), patch('builtins.print'):
            mission.main(['--host', '10.0.0.1'])
            mission.main(['--host', '10.0.0.1', '--no-patrol', '--real', '--i-am-beside-the-robot'])
        reloc.assert_not_called()

    def test_a_real_patrol_preflight_leaves_localization_to_the_relocalization(self):
        args = mission.argparse.Namespace(
            host='10.0.0.1', no_patrol=False, resume_at=None, scene='scene', real=True,
            homography=ROOT / 'detection/rear_ground_homography.json', gstreamer=__file__)
        with patch.object(mission.runner, 'ctl', side_effect=resident_reply), \
             patch.object(mission.dock, 'read_localization', return_value=None) as reading, \
             patch.object(mission.dock, 'fetch_box', return_value={'x': 1, 'y': 1, 'verified': True}), \
             patch.object(mission, 'ensure_arm_camera', return_value=None), \
             patch.object(mission, 'patrol_supports_permit', return_value=True):
            self.assertEqual(mission.preflight(args), [])
        reading.assert_not_called()

    def test_preflight_with_patrol_reads_the_box_from_the_robot(self):
        args = mission.argparse.Namespace(
            host='10.0.0.1', no_patrol=False, resume_at=None, scene='scene',
            homography=ROOT / 'detection/rear_ground_homography.json', gstreamer=__file__)
        with patch.object(mission.runner, 'ctl', side_effect=resident_reply), \
             patch.object(mission.dock, 'read_localization', return_value={
                 'ready': True, 'age': 0.1, 'state': {'pose': [0, 0, 0]}}), \
             patch.object(mission, 'ensure_arm_camera', return_value=None), \
             patch.object(mission, 'patrol_supports_permit', return_value=True), \
             patch('builtins.print'):
            with patch.object(mission.dock, 'fetch_box',
                              return_value={'x': 1, 'y': 1, 'verified': False}) as fetching:
                self.assertEqual(mission.preflight(args), [])
            fetching.assert_called_once_with('10.0.0.1', 'scene')
            with patch.object(mission.dock, 'fetch_box', side_effect=ValueError('no layout')):
                self.assertEqual(mission.preflight(args), ['placement box position: no layout'])
            with patch.object(mission.dock, 'fetch_box', return_value={'x': 1, 'y': 1,
                                                                       'verified': True}), \
                 patch.object(mission, 'patrol_supports_permit', return_value=False):
                self.assertIn('--vision-permit', mission.preflight(args)[0])


class NoPatrolTests(unittest.TestCase):
    """--no-patrol: the one-object flow that delivered twice on 2026-10-18."""

    def run_main(self, argv, holding=(True,), stage_codes=None, camera='ok', problem=None):
        calls = []
        holding = iter(holding)
        stage_codes = iter(stage_codes or [0, 0])

        def ctl(host, cmd):
            calls.append(cmd)
            if cmd == 'health':
                return {'holding': next(holding)}
            return {'chassis': {'maintenance': False, 'arm_pose': 'travel'}}

        def stage(host, argv, log_path=None):
            calls.append(('stage', argv[1], '--handoff-only' in argv, Path(log_path).name))
            return next(stage_codes)

        with tempfile.TemporaryDirectory() as folder, \
             patch.object(mission, 'preflight', return_value=[]), \
             patch.object(mission, 'pc_address', return_value='10.0.0.2'), \
             patch.object(mission, 'stream_running', return_value=True), \
             patch.object(mission, 'ensure_arm_camera',
                          side_effect=lambda host, fix: calls.append('camera') or problem), \
             patch.object(mission, 'arm_camera_state', return_value=camera), \
             patch.object(mission.runner, 'ctl', side_effect=ctl), \
             patch.object(mission.runner, 'service_command') as unlock, \
             patch.object(mission, 'run_stage', side_effect=stage), \
             patch.object(mission, 'patrol_until_sugarbox') as patrol, \
             patch.object(mission.runner, 'back_straight') as backing, \
             patch.object(mission.runner, 'stop') as stopping, \
             patch('builtins.print') as printed:
            code = mission.main(['--host', '10.0.0.1', '--no-patrol', '--isolated-stages', '--real',
                                 '--i-am-beside-the-robot', '--log-dir', folder] + argv)
            text = ' '.join(str(c.args[0]) for c in printed.call_args_list if c.args)
        patrol.assert_not_called()
        backing.assert_not_called()
        stopping.assert_called_once()
        unlock.assert_not_called()                       # the service was not locked
        return code, calls, text

    def test_grasp_then_place_with_their_own_logs(self):
        code, calls, text = self.run_main([])
        self.assertEqual(code, 0)
        stages = [c for c in calls if isinstance(c, tuple)]
        self.assertEqual(stages, [('stage', 'sugarbox', False, 'grasp.log'),
                                  ('stage', 'placement', False, 'place.log')])
        self.assertLess(calls.index('camera'), calls.index(stages[0]))
        self.assertIn('grasped and released', text)

    def test_resume_at_the_box_only_runs_e1_then_places(self):
        code, calls, _ = self.run_main(['--resume-at', 'grasp-e1'])
        self.assertEqual(code, 0)
        self.assertEqual([c for c in calls if isinstance(c, tuple)][0],
                         ('stage', 'sugarbox', True, 'grasp.log'))

    def test_resume_at_place_skips_the_grasp_and_the_arm_camera(self):
        code, calls, _ = self.run_main(['--resume-at', 'place'], holding=())
        self.assertEqual(code, 0)
        self.assertEqual([c for c in calls if isinstance(c, tuple)],
                         [('stage', 'placement', False, 'place.log')])
        self.assertNotIn('camera', calls)

    def test_an_arm_camera_dropout_says_how_to_resume(self):
        code, calls, text = self.run_main([], stage_codes=[2], camera='unplugged')
        self.assertEqual(code, 2)
        self.assertEqual(len([c for c in calls if isinstance(c, tuple)]), 1)   # no placement
        self.assertIn('--resume-at grasp-e1', text)

    def test_an_unfixable_arm_camera_stops_before_any_motion(self):
        code, calls, text = self.run_main([], problem='the arm camera is off the USB bus')
        self.assertEqual(code, 2)
        self.assertFalse([c for c in calls if isinstance(c, tuple)])
        self.assertIn('off the USB bus', text)

    def test_a_failed_placement_that_still_holds_says_how_to_resume(self):
        code, _, text = self.run_main([], holding=(True, True), stage_codes=[0, 2])
        self.assertEqual(code, 2)
        self.assertIn('--resume-at place', text)

    def test_one_object_per_run_and_resume_only_without_patrol(self):
        for argv in (['--no-patrol', '--max-deliveries', '2'], ['--resume-at', 'place'],
                     ['--max-deliveries', '0']):
            with self.subTest(argv=argv), patch('sys.stderr'), self.assertRaises(SystemExit):
                mission.main(['--host', '10.0.0.1'] + argv)

    def test_preflight_without_patrol_needs_no_localization(self):
        args = mission.argparse.Namespace(
            host='10.0.0.1', no_patrol=True, resume_at=None, scene=mission.SCENE,
            homography=ROOT / 'detection/rear_ground_homography.json', gstreamer=__file__)
        with patch.object(mission.runner, 'ctl', side_effect=resident_reply), \
             patch.object(mission.dock, 'read_localization', side_effect=OSError('down')), \
             patch.object(mission, 'ensure_arm_camera', return_value=None):
            self.assertEqual(mission.preflight(args), [])


class ArmCameraTests(unittest.TestCase):
    def test_states(self):
        fresh = lambda host: {'seen': False, 'age': 0.3}
        with patch.object(mission, 'arm_camera_state', return_value='unplugged'):
            self.assertIn('replug', mission.ensure_arm_camera('h', fix=True, look=fresh))
        with patch.object(mission, 'arm_camera_state', return_value='ok'):
            self.assertIsNone(mission.ensure_arm_camera('h', fix=False, look=fresh))

    def test_a_bridge_that_gave_up_is_restarted(self):
        states = iter(['service_down', 'ok'])
        with patch.object(mission, 'arm_camera_state', side_effect=lambda h: next(states)), \
             patch.object(mission, 'ssh') as remote, patch.object(mission.time, 'sleep'), \
             patch('builtins.print'):
            self.assertIsNone(mission.ensure_arm_camera(
                'h', fix=True, look=lambda host: {'seen': True, 'age': 0.2}))
        self.assertIn('restart grasp-vision', remote.call_args.args[1])
        with patch.object(mission, 'arm_camera_state', return_value='service_down'), \
             patch.object(mission, 'ssh') as remote:
            self.assertIn('restarts', mission.ensure_arm_camera('h', fix=False,
                                                                look=lambda host: None))
            remote.assert_not_called()


class StageLogTests(unittest.TestCase):
    def test_stage_output_reaches_the_terminal_and_the_log(self):
        with tempfile.TemporaryDirectory() as folder:
            script = Path(folder) / 'fake_runner.py'
            script.write_text("import sys\nprint('[ARRIVED] 糖果盒 x=0.21')\nsys.exit(3)\n",
                              encoding='utf-8')
            log = Path(folder) / 'grasp.log'
            with patch.object(mission, 'RUNNER', script), patch('sys.stdout') as out, \
                 patch('builtins.print'):
                code = mission.run_stage('10.0.0.1', [], log)
            self.assertEqual(code, 3)
            self.assertIn('[ARRIVED] 糖果盒 x=0.21', log.read_text(encoding='utf-8'))
            self.assertTrue(any('[ARRIVED]' in str(c.args[0]) for c in out.write.call_args_list))


if __name__ == '__main__':
    unittest.main()
