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

    def frame(self, **overrides):
        record = {'age': 0.4, 'seen': True, 'accepted': False, 'border': [],
                  'estimate': [0.296, -0.005]}
        record.update(overrides)
        return record

    def test_adjustment_only_for_a_fresh_box_rejected_ahead_or_behind_the_band(self):
        # 2026-10-16: parked 4 cm short (bottom-centre far above the band), then
        # crept 1.3 cm too far (bottom edge cut off).
        self.assertAlmostEqual(runner.adjust_distance(self.frame()), 0.296 - runner.E1_AIM_X)
        self.assertEqual(runner.adjust_distance(self.frame(border=['bottom'], estimate=None)),
                         -runner.ADJUST_BACK_M)
        self.assertAlmostEqual(runner.adjust_distance(self.frame(estimate=[0.24, 0.0])),
                               0.24 - runner.E1_AIM_X)
        self.assertEqual(runner.adjust_distance(self.frame(estimate=[0.5, 0.0])), runner.ADJUST_MAX_M)
        for record in [None, self.frame(age=2.0), self.frame(age=-0.1), self.frame(seen=False),
                       self.frame(accepted=True), self.frame(accepted=None),
                       self.frame(border=['left', 'bottom']), self.frame(border=['right']),
                       self.frame(estimate=[0.262, 0.0]), self.frame(estimate=[0.3, 0.09]),
                       self.frame(estimate=None), self.frame(estimate=[float('nan'), 0.0])]:
            with self.subTest(record=record):
                self.assertIsNone(runner.adjust_distance(record))

    def adjust_flow(self, frames, accept_after_moves=1):
        self.moves = []

        def read(host, cmd):
            if cmd in ('stow', 'home'):
                self.snapshot['chassis']['arm_pose'] = 'travel' if cmd == 'stow' else 'e1'
            return self.read(host, cmd)

        def drive(host, metres):
            self.calls.append('drive')
            self.moves.append(metres)
            if len(self.moves) >= accept_after_moves:
                self.snapshot['detection'] = {'fresh': True, 'pos': [0.26, 0, 0.0325]}

        self.snapshot['detection'] = {'fresh': False, 'pos': [0.26, 0, 0.0325]}
        return runner.handoff('host', read=read, send_stop=lambda host: self.calls.append('stop'),
                              sleep=self.sleep, now=lambda: self.clock,
                              look=lambda host: copy.deepcopy(frames), drive=drive)

    def test_box_out_of_band_moves_in_travel_pose_then_grasps_a_fresh_reading(self):
        self.assertTrue(self.adjust_flow(self.frame(border=['bottom'], estimate=None))['confirmed'])
        self.assertEqual(self.moves, [-runner.ADJUST_BACK_M])
        drive_at = self.calls.index('drive')
        self.assertEqual(self.calls[drive_at - 2:drive_at], ['stow', 'status'])
        self.assertIn('home', self.calls[drive_at:])
        self.assertEqual(self.calls[-1], 'grasp')

    def test_adjustment_gives_up_after_its_tries(self):
        with self.assertRaises(RuntimeError):
            self.adjust_flow(self.frame(), accept_after_moves=99)
        self.assertEqual(len(self.moves), runner.ADJUST_TRIES)
        self.assertNotIn('grasp', self.calls)

    def test_flickering_detection_still_adjusts_a_box_at_the_far_edge(self):
        # 2026-10-16: raw 0.2755 flickered between payload x 0.280/0.281 and nothing.
        frames = iter([self.frame(estimate=[0.2755, -0.008]), self.frame(seen=False),
                       self.frame(estimate=[0.2757, -0.008])] + [self.frame(seen=False)] * 20)
        self.moves = []

        def read(host, cmd):
            if cmd in ('stow', 'home'):
                self.snapshot['chassis']['arm_pose'] = 'travel' if cmd == 'stow' else 'e1'
            return self.read(host, cmd)

        def drive(host, metres):
            self.moves.append(metres)
            self.snapshot['detection'] = {'fresh': True, 'pos': [0.27, 0, 0.0325]}

        self.snapshot['detection'] = {'fresh': False, 'pos': [0.26, 0, 0.0325]}
        with patch('builtins.print'):
            runner.handoff('host', read=read, send_stop=lambda host: None, sleep=self.sleep,
                           now=lambda: self.clock, look=lambda host: next(frames), drive=drive)
        self.assertEqual(len(self.moves), 1)
        self.assertAlmostEqual(self.moves[0], 0.2757 - runner.E1_AIM_X)

    def test_unhelpful_frames_never_drive(self):
        for frame in [self.frame(border=['left']), self.frame(estimate=[0.262, 0.0]), None]:
            with self.subTest(frame=frame):
                self.setUp()
                with self.assertRaises(RuntimeError):
                    self.adjust_flow(frame)
                self.assertEqual(self.moves, [])
                self.assertNotIn('stow', self.calls)

    def test_wheels_wait_for_a_confirmed_travel_pose(self):
        def read(host, cmd):
            return self.read(host, cmd)       # stow never reaches travel
        self.snapshot['detection'] = {'fresh': False, 'pos': [0.26, 0, 0.0325]}
        driven = []
        with self.assertRaises(RuntimeError):
            runner.handoff('host', read=read, send_stop=lambda host: None, sleep=self.sleep,
                           now=lambda: self.clock, look=lambda host: self.frame(),
                           drive=lambda host, m: driven.append(m))
        self.assertEqual(driven, [])

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

    def test_no_progress_stop_is_reported_and_never_hands_off(self):
        snapshot = copy.deepcopy(self.snapshot)
        snapshot['servo_deg'] = [90, 139, 0, 0, 90, 30]
        snapshot['chassis']['arm_pose'] = 'travel'
        commands = []
        def read(host, cmd):
            commands.append(cmd)
            return snapshot if cmd == 'status' else {'ok': True}
        with patch.object(runner, 'preflight', return_value=({}, [])), \
             patch.object(runner, 'ctl', side_effect=read), \
             patch.object(runner, 'stop'), \
             patch.object(runner.time, 'sleep'), \
             patch.object(runner.subprocess, 'run') as process, \
             patch('builtins.print') as printed:
            process.return_value.returncode = 4
            self.assertEqual(runner.main(['--real', '--i-am-beside-the-robot']), 2)
            self.assertNotIn('home', commands)
            self.assertNotIn('grasp', commands)
            self.assertIn('no progress', ' '.join(str(c.args[0]) for c in printed.call_args_list))

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

    def launched_env(self, argv, inherited=None):
        snapshot = copy.deepcopy(self.snapshot)
        snapshot['servo_deg'] = [90, 139, 0, 0, 90, 30]
        with patch.dict(runner.os.environ, inherited or {}, clear=True), \
             patch.object(runner, 'preflight', return_value=({}, [])), \
             patch.object(runner, 'ctl', return_value=snapshot), \
             patch.object(runner.subprocess, 'run') as process:
            process.return_value.returncode = 0
            self.assertEqual(runner.main(['--dry-run'] + argv), 0)
            return process.call_args.kwargs['env']

    def test_blind_stop_and_yolo_size_reach_the_approach(self):
        env = self.launched_env(['--approach', 'legacy', '--blind-stop-x', '0.30',
                                 '--yolo-imgsz', '1280'])
        self.assertEqual(env['SUGARBOX_BLIND_STOP_X'], '0.3')
        self.assertEqual(env['SUGARBOX_YOLO_IMGSZ'], '1280')

    def test_enoch_approach_is_the_default_and_never_drives_in_a_dry_run(self):
        env = self.launched_env([])
        self.assertEqual(env['SUGARBOX_TRAINING_CONTRACT'], '1')
        self.assertNotIn('SUGARBOX_CONTRACT_ALLOW_DRIVE', env)

    def test_enoch_approach_keeps_measuring_the_box_and_sees_far(self):
        # The first-sighting lock left 11 cm of odometry slip uncorrected (2026-10-12);
        # the 1.40 m rear calibration needs 1280 to find the box past ~1 m.
        env = self.launched_env([], {'SUGARBOX_TARGET_INITIAL_ONLY': '1'})
        self.assertNotIn('SUGARBOX_TARGET_INITIAL_ONLY', env)
        self.assertEqual(env['SUGARBOX_YOLO_IMGSZ'], '1280')

    def test_legacy_approach_sets_none_of_enochs_settings(self):
        env = self.launched_env(['--approach', 'legacy'], {
            'SUGARBOX_TRAINING_CONTRACT': '1', 'SUGARBOX_CONTRACT_ALLOW_DRIVE': '1',
            'SUGARBOX_TARGET_INITIAL_ONLY': '1'})
        for key in ('SUGARBOX_TRAINING_CONTRACT', 'SUGARBOX_CONTRACT_ALLOW_DRIVE',
                    'SUGARBOX_TARGET_INITIAL_ONLY'):
            self.assertNotIn(key, env)
        self.assertEqual(env['SUGARBOX_YOLO_IMGSZ'], '640')

    def test_gstreamer_prefers_the_bundled_binary_over_the_launcher(self):
        with tempfile.TemporaryDirectory() as folder:
            prefix = Path(folder)
            binary = prefix / 'Lib/site-packages/gstreamer_cli/bin/gst-launch-1.0.exe'
            launcher = prefix / 'Scripts/gst-launch-1.0.exe'
            for path in (binary, launcher):
                path.parent.mkdir(parents=True)
                path.write_bytes(b'')
            with patch.dict(runner.os.environ, {}, clear=True), \
                 patch.object(runner.shutil, 'which', return_value=None), \
                 patch.object(runner.sys, 'prefix', str(prefix)):
                self.assertEqual(runner.find_gstreamer(), binary)
                binary.unlink()
                self.assertEqual(runner.find_gstreamer(), launcher)

    def test_blind_finish_is_refused_with_enochs_approach(self):
        with patch.object(runner, 'ctl') as read, patch('sys.stderr'),              self.assertRaises(SystemExit):
            runner.main(['--blind-stop-x', '0.30'])
        read.assert_not_called()

    def test_real_enoch_run_opts_into_driving_then_hands_over_to_e1(self):
        snapshot = copy.deepcopy(self.snapshot)
        snapshot['servo_deg'] = [90, 139, 0, 0, 90, 30]
        snapshot['chassis']['arm_pose'] = 'travel'
        with patch.dict(runner.os.environ, {}, clear=True),              patch.object(runner, 'preflight', return_value=({}, [])),              patch.object(runner, 'ctl', return_value=snapshot),              patch.object(runner, 'stop'),              patch.object(runner.time, 'sleep'),              patch.object(runner, 'handoff', return_value={'confirmed': True}) as grasp,              patch.object(runner.subprocess, 'run') as process:
            process.return_value.returncode = 0
            self.assertEqual(runner.main(['--real', '--i-am-beside-the-robot']), 0)
            env = process.call_args.kwargs['env']
            self.assertEqual(env['SUGARBOX_CONTRACT_ALLOW_DRIVE'], '1')
            self.assertEqual(env['SUGARBOX_RUN_MODE'], 'DRIVE')
            grasp.assert_called_once()
            self.assertIs(grasp.call_args.kwargs['look'], runner.look)
            self.assertIs(grasp.call_args.kwargs['drive'], runner.drive)
            grasp.reset_mock()
            self.assertEqual(runner.main(['--real', '--i-am-beside-the-robot',
                                          '--approach', 'legacy']), 0)
            self.assertEqual(grasp.call_args.kwargs, {})     # legacy may not reverse

    def test_enoch_hands_over_at_1m_and_stops_the_box_at_28cm_by_default(self):
        env = self.launched_env([])
        self.assertEqual((env['SUGARBOX_CONTRACT_HANDOFF_M'], env['SUGARBOX_CONTRACT_FINAL_X_M']),
                         ('1.0', '0.28'))
        env = self.launched_env(['--handoff-m', '1.0', '--final-x-m', '0.34'])
        self.assertEqual((env['SUGARBOX_CONTRACT_HANDOFF_M'], env['SUGARBOX_CONTRACT_FINAL_X_M']),
                         ('1.0', '0.34'))
        legacy = self.launched_env(['--approach', 'legacy'])
        self.assertNotIn('SUGARBOX_CONTRACT_HANDOFF_M', legacy)
        self.assertNotIn('SUGARBOX_CONTRACT_FINAL_X_M', legacy)

    def test_obs_log_reaches_the_approach_only_when_asked(self):
        self.assertNotIn('SUGARBOX_OBS_LOG', self.launched_env([], {'SUGARBOX_OBS_LOG': 'old.jsonl'}))
        env = self.launched_env(['--obs-log', 'run.jsonl'])
        self.assertEqual(Path(env['SUGARBOX_OBS_LOG']), Path('run.jsonl').resolve())

    def test_settings_left_in_the_shell_do_not_leak_into_a_run(self):
        env = self.launched_env([], {'SUGARBOX_BLIND_STOP_X': '0.5', 'SUGARBOX_YOLO_IMGSZ': '960'})
        self.assertNotIn('SUGARBOX_BLIND_STOP_X', env)
        self.assertEqual(env['SUGARBOX_YOLO_IMGSZ'], '1280')

    def test_out_of_range_settings_are_refused_before_anything_runs(self):
        for argv in (['--blind-stop-x', '0.9'], ['--blind-stop-x', 'nan'],
                     ['--final-x-m', '0.5'], ['--handoff-m', '0.4'],
                     ['--handoff-m', '0.6', '--final-x-m', '0.5'],
                     ['--yolo-imgsz', '1000'], ['--yolo-imgsz', '64']):
            with self.subTest(argv=argv), \
                 patch.object(runner, 'ctl') as read, \
                 patch('sys.stderr'), self.assertRaises(SystemExit):
                runner.main(argv)
            read.assert_not_called()

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
