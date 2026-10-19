"""Competition loop orchestration: no robot, camera, model or ssh."""
import ast
import importlib.util
import json
import math
from pathlib import Path
import shlex
import tempfile
import unittest
from unittest.mock import patch

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


LAYOUT = {'frame_id': 'map', 'bin': {
    'pose_map': {'x': 1.76, 'y': 0.88}, 'pose_map_valid_for_current_placement': False,
    'placement_update': {'midpoint_pose_map_candidate': {'x': 1.56, 'y': 0.83}}}}


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
                     '--output /tmp/mission_patrol.json'):
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

    def run_patrol(self, points, running=None, reports=None, box=None, pose=(0.0, 0.0, 0.0),
                   skip=(), make_room=None):
        started, stopped = [], []
        running = running if running is not None else (lambda host, pid: True)
        reports = iter(reports or [{'ok': True}] * 10)
        with patch.object(mission, 'start_patrol',
                          side_effect=lambda host, scene: started.append(scene) or 40 + len(started)), \
             patch.object(mission, 'patrol_running', side_effect=running), \
             patch.object(mission, 'stop_patrol', side_effect=lambda host, pid: stopped.append(pid)), \
             patch.object(mission, 'patrol_report', side_effect=lambda host: next(reports)), \
             patch.object(mission.dock, 'read_localization',
                          return_value={'ready': True, 'age': 0.1, 'state': {'pose': list(pose)}}), \
             patch.object(mission.time, 'sleep'), patch('builtins.print'):
            seen = mission.patrol_until_sugarbox('host', 'scene', FakeReader(True),
                                                 FakeWatcher(points), box, skip=skip,
                                                 make_room=make_room, now=lambda: 0.0)
        return seen, started, stopped

    def test_a_confirmed_sugarbox_stops_the_patrol(self):
        seen, started, stopped = self.run_patrol([None, (1.0, 0.1), (1.0, 0.1), (1.0, 0.1)],
                                                 pose=(1.0, 0.0, 0.0))
        self.assertEqual(started, ['scene'])
        self.assertEqual(stopped, [41])
        self.assertEqual([round(v, 6) for v in seen['point']], [1.0, 0.1])
        self.assertEqual([round(v, 6) for v in seen['map']], [2.0, 0.1])

    def test_a_sugarbox_beyond_the_calibrated_area_stops_the_patrol_too(self):
        seen, _, stopped = self.run_patrol([{'far': (1.8, 0.5)}] * 3)
        self.assertEqual([round(v, 6) for v in seen['point']], [1.8, 0.5])
        self.assertEqual(stopped, [41])

    def test_the_delivered_sugarbox_is_not_fetched_again(self):
        points = [(1.0, 0.1)] * 3 + [(0.8, -0.3)] * 3
        for verified in (True, False):
            with self.subTest(verified=verified):
                box = {'x': 1.0, 'y': 0.1, 'verified': verified}
                seen, _, stopped = self.run_patrol(list(points), box=box)
                self.assertEqual([round(v, 6) for v in seen['point']], [0.8, -0.3])
                self.assertEqual(stopped, [41])

    def test_a_spot_where_grasping_failed_twice_is_left_alone(self):
        points = [(1.0, 0.1)] * 3 + [(0.8, -0.3)] * 3
        seen, _, _ = self.run_patrol(points, skip=[(1.05, 0.1)])
        self.assertEqual([round(v, 6) for v in seen['point']], [0.8, -0.3])

    def test_finished_laps_restart_and_two_empty_laps_end_the_search(self):
        alive = iter([False, True, True, True, True, True, True, True, True])
        _, started, _ = self.run_patrol([None, (1.0, 0.0), (1.0, 0.0), (1.0, 0.0)],
                                        running=lambda host, pid: next(alive))
        self.assertEqual(started, ['scene', 'scene'])
        seen, started, _ = self.run_patrol([None] * 9, running=lambda host, pid: False)
        self.assertIsNone(seen)
        self.assertEqual(len(started), mission.EMPTY_LAPS_TO_END)

    def test_a_failed_patrol_stops_the_mission(self):
        with self.assertRaises(mission.MissionError):
            self.run_patrol([None] * 5, running=lambda host, pid: False,
                            reports=[{'ok': False, 'error': 'AMCL uncertainty', 'pulses': 7}])

    def test_a_refusal_to_start_makes_room_and_asks_again(self):
        refused = {'ok': False, 'pulses': 0, 'error': 'no reachable entry to patrol route'}
        alive, room = iter([False] + [True] * 9), []
        seen, started, stopped = self.run_patrol(
            [None, (1.0, 0.1), (1.0, 0.1), (1.0, 0.1)], running=lambda host, pid: next(alive),
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
                                          now=lambda: next(clock))
        starting.assert_not_called()


class RoomTests(unittest.TestCase):
    def test_backs_out_then_a_step_more_up_to_the_limit(self):
        backs = []
        room = mission.Room('host', back=lambda host, m: backs.append(round(m, 3)))
        room.leave()
        self.assertTrue(room.make())
        self.assertFalse(room.make())                 # 0.20 + 0.10 = 0.30 m in all
        self.assertEqual(backs, [0.2, 0.1])
        room.reset()
        self.assertEqual([room.make() for _ in range(4)], [True, True, True, False])


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
        with patch('builtins.print'):
            found = mission.search_placement('host', ClockReader(clock), Watcher(),
                                             turn=lambda host, r: turned.append(r),
                                             now=lambda: clock[0])
        return found, turned

    def test_turns_until_the_box_shows(self):
        found, turned = self.search(2)
        self.assertTrue(found)
        self.assertEqual(turned, [mission.SEARCH_STEP_RAD] * 2)

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

    def test_a_clean_miss_stows_and_patrols_on(self):
        self.assertEqual(self.fetch(2, 'empty'), 'missed')
        self.assertEqual(self.calls[-1], 'travel')

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

    def run_mission(self, sightings, grasps, argv=(), place_error=None):
        order = []
        sightings, grasps = iter(sightings), iter(grasps)
        args = mission.argparse.Namespace(host='10.0.0.1', scene='scene', homography=Path('h'),
                                          gstreamer=None, wall_m=None, max_deliveries=None)
        for key, value in argv:
            setattr(args, key, value)

        class FakeRoom:
            def leave(self):
                order.append('leave')

            def make(self):
                return False

            def reset(self):
                order.append('reset')

        def patrol(host, scene, reader, watcher, box, skip=(), make_room=None):
            order.append(('patrol', len(skip)))
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
             patch.object(mission.runner, 'stop') as stopping, patch('builtins.print'):
            code = mission.patrol_mission(args, '10.0.0.2', Path(folder), room=FakeRoom())
            copied = json.loads((Path(folder) / 'scene_layout.json').read_text(encoding='utf-8'))
        stopping.assert_called_once()
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
        code, order = self.run_mission([spot, spot, None], ['missed', 'missed'])
        self.assertEqual(code, 0)
        self.assertEqual(order, [('patrol', 0), 'reset', 'grasp', 'leave',
                                 ('patrol', 0), 'reset', 'grasp', 'leave',
                                 ('patrol', 2)])

    def test_max_deliveries_stops_after_the_last_one(self):
        spot = {'point': (1.0, 0.0), 'map': None}
        code, order = self.run_mission([spot], ['held'], argv=[('max_deliveries', 1)])
        self.assertEqual((code, order[-1], order.count('place')), (0, 'leave', 1))

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

    def test_real_patrol_mode_runs_the_patrol_mission(self):
        with patch.object(mission, 'preflight', return_value=[]), \
             patch.object(mission, 'pc_address', return_value='10.0.0.2'), \
             patch.object(mission, 'patrol_mission', return_value=0) as patrolling, \
             patch.object(mission, 'grasp_and_place') as one, patch('builtins.print'):
            self.assertEqual(mission.main(['--host', '10.0.0.1', '--real',
                                           '--i-am-beside-the-robot']), 0)
        patrolling.assert_called_once()
        self.assertIsNone(patrolling.call_args.args[0].max_deliveries)   # until laps run dry
        one.assert_not_called()

    def test_preflight_with_patrol_reads_the_box_from_the_robot(self):
        args = mission.argparse.Namespace(
            host='10.0.0.1', no_patrol=False, resume_at=None, scene='scene',
            homography=ROOT / 'detection/rear_ground_homography.json', gstreamer=__file__)
        with patch.object(mission.runner, 'ctl', side_effect=lambda h, c: (
                {'holding': False} if c == 'health'
                else {'chassis': {'arm_pose': 'travel', 'moving': False}})), \
             patch.object(mission.dock, 'read_localization', return_value={
                 'ready': True, 'age': 0.1, 'state': {'pose': [0, 0, 0]}}), \
             patch.object(mission, 'ensure_arm_camera', return_value=None), \
             patch('builtins.print'):
            with patch.object(mission.dock, 'fetch_box',
                              return_value={'x': 1, 'y': 1, 'verified': False}) as fetching:
                self.assertEqual(mission.preflight(args), [])
            fetching.assert_called_once_with('10.0.0.1', 'scene')
            with patch.object(mission.dock, 'fetch_box', side_effect=ValueError('no layout')):
                self.assertEqual(mission.preflight(args), ['placement box position: no layout'])


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
            code = mission.main(['--host', '10.0.0.1', '--no-patrol', '--real',
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
        with patch.object(mission.runner, 'ctl', side_effect=lambda h, c: (
                {'holding': False} if c == 'health'
                else {'chassis': {'arm_pose': 'travel', 'moving': False}})), \
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
