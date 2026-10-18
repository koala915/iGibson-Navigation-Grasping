"""Competition loop orchestration: no robot, camera, model or ssh."""
import ast
import importlib.util
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
             patch.object(mission, 'back_out') as backing, \
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
        for argv in (['--no-patrol', '--loops', '2'], ['--resume-at', 'place']):
            with self.subTest(argv=argv), patch('sys.stderr'), self.assertRaises(SystemExit):
                mission.main(['--host', '10.0.0.1'] + argv)

    def test_preflight_without_patrol_needs_no_localization(self):
        args = mission.argparse.Namespace(
            host='10.0.0.1', no_patrol=True, resume_at=None, box_map=mission.dock.BOX_MAP,
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
