"""No-hardware regression for approach arrival -> E1 -> fresh grasp."""
import copy
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('runner', Path(__file__).resolve().parents[1] /
                                        'integration/sugarbox_approach_grasp.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class HandoffTests(unittest.TestCase):
    def setUp(self):
        self.clock = 0
        self.calls = []
        self.snapshot = {'chassis': {'moving': False, 'arm_busy': False,
            'motors': [0, 0, 0, 0], 'board_rx_age_s': 0.01,
            'stopped_for_s': 1, 'arm_pose': 'e1'},
            'odom': {'valid': True, 'failures': 0, 'twist': [0, 0, 0]},
            'detection': {'fresh': True, 'pos': [0.26, 0, 0.0325]}}

    def sleep(self, seconds):
        self.clock += seconds

    def read(self, host, command):
        self.calls.append(command)
        return copy.deepcopy(self.snapshot) if command == 'status' else {'ok': True, 'confirmed': True}

    def run_flow(self, read=None):
        return runner.handoff('host', read=read or self.read,
            send_stop=lambda host: self.calls.append('stop'), sleep=self.sleep, now=lambda: self.clock)

    def test_success_requires_stop_home_then_two_fresh_observations(self):
        self.assertTrue(self.run_flow()['confirmed'])
        self.assertEqual(self.calls, ['stop', 'status', 'home', 'status', 'status', 'grasp'])
        self.assertGreaterEqual(self.clock, 3.6)

    def test_moving_or_stale_feedback_never_moves_arm(self):
        for field, value in [('moving', True), ('board_rx_age_s', 2), ('stopped_for_s', 0.1)]:
            with self.subTest(field=field):
                self.setUp()
                self.snapshot['chassis'][field] = value
                with self.assertRaises(RuntimeError): self.run_flow()
                self.assertNotIn('home', self.calls)
                self.assertNotIn('grasp', self.calls)

    def test_stale_or_outside_target_never_grasps(self):
        for detection in [{'fresh': False, 'pos': [0.26, 0, 0.0325]},
                          {'fresh': True, 'pos': [0.25, -0.083, 0.0325]},
                          {'fresh': True, 'pos': [float('nan'), 0, 0.0325]}]:
            with self.subTest(detection=detection):
                self.setUp()
                self.snapshot['detection'] = detection
                with self.assertRaises(RuntimeError): self.run_flow()
                self.assertNotIn('grasp', self.calls)

    def test_motion_after_home_aborts(self):
        def read(host, cmd):
            if cmd == 'home': self.snapshot['chassis']['moving'] = True
            return self.read(host, cmd)
        with self.assertRaises(RuntimeError): self.run_flow(read)
        self.assertNotIn('grasp', self.calls)

    def test_unconfirmed_grasp_is_failure(self):
        def read(host, cmd):
            result = self.read(host, cmd)
            if cmd == 'grasp': result['confirmed'] = False
            return result
        with self.assertRaises(RuntimeError): self.run_flow(read)

    def test_failed_approach_never_hands_off(self):
        snapshot = copy.deepcopy(self.snapshot)
        snapshot['servo_deg'] = [90, 74, 8, 9, 90, 30]
        snapshot['chassis']['arm_pose'] = 'travel'
        commands = []
        def read(host, cmd):
            commands.append(cmd)
            return snapshot if cmd == 'status' else {'ok': True}
        with patch.object(runner, 'preflight', return_value=({}, [])), \
             patch.object(runner, 'ctl', side_effect=read), \
             patch.object(runner, 'stop') as stopping, \
             patch.object(runner.time, 'sleep'), \
             patch.object(runner.subprocess, 'run') as process:
            process.return_value.returncode = 3
            self.assertEqual(runner.main(['--real', '--i-am-beside-the-robot']), 2)
            self.assertNotIn('home', commands)
            self.assertNotIn('grasp', commands)
            self.assertEqual(stopping.call_count, 2)

    def test_default_check_never_stows_or_starts_approach(self):
        snapshot = copy.deepcopy(self.snapshot)
        snapshot['servo_deg'] = [90, 74, 8, 9, 90, 30]
        with patch.object(runner, 'preflight', return_value=({}, [])), \
             patch.object(runner, 'ctl', return_value=snapshot) as read, \
             patch.object(runner, 'stop') as stopping, \
             patch.object(runner.subprocess, 'run') as process:
            self.assertEqual(runner.main([]), 0)
            self.assertEqual(read.call_args.args[1], 'status')
            stopping.assert_not_called()
            process.assert_not_called()

    def test_default_calibration_uses_committed_json(self):
        snapshot = copy.deepcopy(self.snapshot)
        snapshot['servo_deg'] = [90, 74, 8, 9, 90, 30]
        with patch.dict(runner.os.environ, {}, clear=True), \
             patch.object(runner, 'preflight', return_value=({}, [])) as check, \
             patch.object(runner, 'ctl', return_value=snapshot):
            self.assertEqual(runner.main([]), 0)
            self.assertEqual(check.call_args.args[0].homography,
                             runner.ROOT / 'detection/rear_ground_homography.json')

    def test_bad_calibration_blocks_real_run_before_stow(self):
        snapshot = copy.deepcopy(self.snapshot)
        snapshot['servo_deg'] = [90, 74, 8, 9, 90, 30]
        with tempfile.TemporaryDirectory() as folder:
            placeholder = Path(folder) / 'asset'
            placeholder.write_text('', encoding='utf-8')
            bad = Path(folder) / 'bad.json'
            bad.write_text('{}', encoding='utf-8')
            with patch.object(runner.importlib.util, 'find_spec', return_value=True), \
                 patch.object(runner, 'ctl', return_value=snapshot) as read, \
                 patch.object(runner, 'stop') as stopping, \
                 patch.object(runner.subprocess, 'run') as process:
                result = runner.main(['--real', '--i-am-beside-the-robot',
                                      '--sam', str(placeholder),
                                      '--gstreamer', str(placeholder),
                                      '--homography', str(bad)])
                self.assertEqual(result, 2)
                read.assert_called_once_with('10.16.224.252', 'status')
                stopping.assert_not_called()
                process.assert_not_called()


if __name__ == '__main__':
    unittest.main()
