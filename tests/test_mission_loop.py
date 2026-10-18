"""Competition loop orchestration: no robot, camera, model or ssh."""
import ast
import importlib.util
import math
from pathlib import Path
import shlex
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
    def __init__(self, points):
        self.points = list(points)

    def look(self, frame):
        return self.points.pop(0) if self.points else None


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

    def run_patrol(self, points, running=None, report=None, box=None, pose=(0.0, 0.0, 0.0)):
        started, stopped = [], []
        running = running if running is not None else (lambda host, pid: True)
        with patch.object(mission, 'start_patrol',
                          side_effect=lambda host, scene: started.append(scene) or 40 + len(started)), \
             patch.object(mission, 'patrol_running', side_effect=running), \
             patch.object(mission, 'stop_patrol', side_effect=lambda host, pid: stopped.append(pid)), \
             patch.object(mission, 'patrol_report', return_value=report or {'ok': True}), \
             patch.object(mission.dock, 'read_localization',
                          return_value={'ready': True, 'age': 0.1, 'state': {'pose': list(pose)}}), \
             patch.object(mission.time, 'sleep'):
            seen = mission.patrol_until_sugarbox('host', 'scene', FakeReader(True),
                                                 FakeWatcher(points), box,
                                                 now=lambda: 0.0)
        return seen, started, stopped

    def test_a_confirmed_sugarbox_stops_the_patrol(self):
        seen, started, stopped = self.run_patrol([None, (1.0, 0.1), (1.0, 0.1), (1.0, 0.1)])
        self.assertEqual(started, ['scene'])
        self.assertEqual(stopped, [41])
        self.assertEqual([round(v, 6) for v in seen], [1.0, 0.1])

    def test_the_delivered_sugarbox_is_not_fetched_again(self):
        box = {'x': 1.0, 'y': 0.1, 'verified': True}
        points = [(1.0, 0.1)] * 3 + [(0.8, -0.3)] * 3
        seen, _, stopped = self.run_patrol(points, box=box)
        self.assertEqual([round(v, 6) for v in seen], [0.8, -0.3])
        self.assertEqual(stopped, [41])

    def test_finished_cycles_restart_and_a_failed_patrol_stops_the_mission(self):
        alive = iter([False, True, True, True, True, True, True, True, True])
        _, started, _ = self.run_patrol([None, (1.0, 0.0), (1.0, 0.0), (1.0, 0.0)],
                                        running=lambda host, pid: next(alive))
        self.assertEqual(started, ['scene', 'scene'])
        with self.assertRaises(mission.MissionError):
            self.run_patrol([None] * 5, running=lambda host, pid: False,
                            report={'ok': False, 'error': 'AMCL uncertainty'})

    def test_no_frames_never_starts_the_patrol(self):
        with patch.object(mission, 'start_patrol') as starting, \
             patch.object(mission.time, 'sleep'), self.assertRaises(mission.MissionError):
            clock = iter(range(100))
            mission.patrol_until_sugarbox('host', 'scene', FakeReader(False), FakeWatcher([]),
                                          now=lambda: next(clock))
        starting.assert_not_called()


class BackOutTests(unittest.TestCase):
    def scans(self, clearance):
        return lambda host: [{'ranges': [], 'angle_min': 0.0, 'angle_increment': 0.01}] * 3

    def test_backs_out_only_with_room_behind(self):
        moves = []
        with patch.object(mission.dock, 'rear_clearance', return_value=0.5):
            mission.back_out('host', scans=self.scans(0.5),
                             drive=lambda host, m, limit: moves.append((m, limit)))
        self.assertEqual(moves, [(-0.20, 0.30)])
        with patch.object(mission.dock, 'rear_clearance', return_value=0.25), \
             self.assertRaises(mission.MissionError):
            mission.back_out('host', scans=self.scans(0.25),
                             drive=lambda host, m, limit: moves.append((m, limit)))
        self.assertEqual(len(moves), 1)


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

    def test_one_loop_runs_patrol_grasp_place_back_out_in_order(self):
        order = []
        holding = iter([True])
        with patch.object(mission, 'preflight', return_value=[]), \
             patch.object(mission, 'pc_address', return_value='10.0.0.2'), \
             patch.object(mission, 'stream_running', return_value=True), \
             patch.object(mission, 'SugarboxWatcher'), \
             patch('integration.rear_stream.StreamReader'), \
             patch.object(mission.dock, 'load_box_map', return_value={'verified': False}), \
             patch.object(mission, 'patrol_until_sugarbox',
                          side_effect=lambda *a: order.append('patrol')), \
             patch.object(mission.runner, 'service_command',
                          side_effect=lambda host, c: order.append(c)), \
             patch.object(mission, 'run_stage',
                          side_effect=lambda host, argv: order.append(argv[1]) or 0), \
             patch.object(mission.runner, 'ctl',
                          side_effect=lambda host, c: {'holding': next(holding)}), \
             patch.object(mission, 'back_out', side_effect=lambda host: order.append('back')), \
             patch.object(mission.runner, 'stop'):
            code = mission.main(['--host', '10.0.0.1', '--real', '--i-am-beside-the-robot'])
        self.assertEqual(code, 0)
        self.assertEqual(order, ['patrol', 'maintenance-unlock', 'sugarbox', 'placement', 'back'])

    def test_a_failed_grasp_stops_before_placing(self):
        order = []
        with patch.object(mission, 'preflight', return_value=[]), \
             patch.object(mission, 'pc_address', return_value='10.0.0.2'), \
             patch.object(mission, 'stream_running', return_value=True), \
             patch.object(mission, 'SugarboxWatcher'), \
             patch('integration.rear_stream.StreamReader'), \
             patch.object(mission.dock, 'load_box_map', return_value={'verified': False}), \
             patch.object(mission, 'patrol_until_sugarbox'), \
             patch.object(mission.runner, 'service_command'), \
             patch.object(mission, 'run_stage',
                          side_effect=lambda host, argv: order.append(argv[1]) or 2), \
             patch.object(mission, 'back_out') as backing, \
             patch.object(mission.runner, 'stop') as stopping:
            code = mission.main(['--host', '10.0.0.1', '--real', '--i-am-beside-the-robot'])
        self.assertEqual(code, 2)
        self.assertEqual(order, ['sugarbox'])
        backing.assert_not_called()
        stopping.assert_called_once()


if __name__ == '__main__':
    unittest.main()
