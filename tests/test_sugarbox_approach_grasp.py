"""No-hardware regression for approach arrival -> E1 -> fresh grasp."""
import copy
import json
import math
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('runner', Path(__file__).resolve().parents[1] /
                                        'integration/sugarbox_approach_grasp.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
# A 4 m square room around the origin, as ncu's scene layout gives room_polygon.
ROOM = [(-1.0, -1.0), (3.0, -1.0), (3.0, 3.0), (-1.0, 3.0)]


class FakeEngine:
    """The approach engine started early (start_engine): its environment, the go
    line it was sent, and how it ends once started."""

    def __init__(self, env, returncode=0, on_go=None, ended=None):
        self.env, self.exit_code, self.on_go = env, returncode, on_go
        self.stdin, self.lines, self.returncode = self, [], ended

    def write(self, text):
        self.lines.append(text)

    def close(self):
        pass

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        message = json.loads(self.lines[-1])
        if self.on_go is not None:
            self.on_go(self.env, message)
        self.returncode = self.exit_code
        return self.returncode

    def go_seed(self):
        return json.loads(self.lines[-1])['seed']


def stopped_at(x, y=0.0):
    """on_go for FakeEngine: the engine's arrival report with the box at (x, y)."""
    def report(env, message):
        Path(env['SUGARBOX_ARRIVAL_REPORT']).write_text(json.dumps(
            {'arrived': False, 'blocked': True, 'x': x, 'y': y}), encoding='utf-8')
    return report


def engines(returncode=0, on_go=None, ended=None):
    """(patch kwargs for runner.start_engine, the list of engines it started)."""
    started = []

    def start(env):
        started.append(FakeEngine(env, returncode, on_go, ended))
        return started[-1]
    return dict(side_effect=start), started


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

    def test_a_poll_between_two_e1_frames_does_not_wipe_the_last_reading(self):
        # 2026-10-18: E1 sent every 1.4 s, a detection counts as fresh for 1 s,
        # and the 1.1 s polls kept landing between frames: fresh, stale, fresh...
        # The first status read is settle()'s stop check.
        fresh = iter([True] + [True, False, True, False, True])

        def read(host, cmd):
            self.calls.append(cmd)
            if cmd != 'status':
                return {'ok': True, 'confirmed': True}
            snapshot = copy.deepcopy(self.snapshot)
            snapshot['detection']['fresh'] = next(fresh)
            return snapshot

        with patch('builtins.print'):
            self.assertTrue(self.run_flow(read=read)['confirmed'])
        self.assertEqual(self.calls.count('status'), 4)       # settle; fresh, stale, fresh -> grasp

    def test_readings_too_far_apart_in_time_do_not_agree(self):
        fresh = iter([True] + [True] + [False] * 5 + [True] + [False] * 20)

        def read(host, cmd):
            self.calls.append(cmd)
            if cmd != 'status':
                return {'ok': True, 'confirmed': True}
            snapshot = copy.deepcopy(self.snapshot)
            snapshot['detection']['fresh'] = next(fresh)
            return snapshot

        with patch('builtins.print'), self.assertRaises(RuntimeError):
            self.run_flow(read=read)                          # 5.5 s apart: never paired
        self.assertNotIn('grasp', self.calls)

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

    def test_e1_seeing_nothing_creeps_forward_then_grasps(self):
        # 2026-10-18, white ribbed mats: odometry ran fast over the blind last
        # 41 cm and parked 3-6 cm short; E1 saw only the floor.
        with patch('builtins.print'):
            result = self.adjust_flow(self.frame(seen=False, estimate=None), accept_after_moves=2)
        self.assertTrue(result['confirmed'])
        self.assertEqual(self.moves, [runner.BLIND_CREEP_M] * 2)
        drive_at = self.calls.index('drive')
        self.assertEqual(self.calls[drive_at - 2:drive_at], ['stow', 'status'])

    def test_blind_creep_is_bounded_and_forward_only(self):
        with patch('builtins.print'), self.assertRaises(RuntimeError):
            self.adjust_flow(self.frame(seen=False, estimate=None), accept_after_moves=99)
        self.assertEqual(self.moves, [runner.BLIND_CREEP_M] * runner.BLIND_CREEP_TRIES)
        self.assertNotIn('grasp', self.calls)
        self.assertTrue(0 < runner.BLIND_CREEP_M * runner.BLIND_CREEP_TRIES <= 0.06)

    def test_no_creep_without_fresh_proof_that_e1_sees_nothing(self):
        blank = self.frame(seen=False, estimate=None)
        cases = {'stale frames': [dict(blank, age=2.0)] * 12,
                 'no reports': [None] * 12,
                 'a box seen once': [blank, blank, self.frame(border=['left'])] + [blank] * 9,
                 'too few blank frames': [blank] * (runner.BLIND_CREEP_LOOKS - 1) + [None] * 9}
        for name, frames in cases.items():
            with self.subTest(case=name):
                self.setUp()
                frames = iter(frames + [None] * 20)
                self.moves = []

                def read(host, cmd):
                    if cmd in ('stow', 'home'):
                        self.snapshot['chassis']['arm_pose'] = 'travel' if cmd == 'stow' else 'e1'
                    return self.read(host, cmd)

                self.snapshot['detection'] = {'fresh': False, 'pos': [0.26, 0, 0.0325]}
                with patch('builtins.print'), self.assertRaises(RuntimeError):
                    runner.handoff('host', read=read, send_stop=lambda host: None,
                                   sleep=self.sleep, now=lambda: self.clock,
                                   look=lambda host: next(frames),
                                   drive=lambda host, m: self.moves.append(m))
                self.assertEqual(self.moves, [])

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
             patch.object(runner, 'start_engine', **engines(3, stopped_at(1.2))[0]):
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
             patch.object(runner, 'start_engine', **engines(4, stopped_at(0.95))[0]), \
             patch('builtins.print') as printed:
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
        fake = engines(0)
        with patch.dict(runner.os.environ, {}, clear=True),              patch.object(runner, 'preflight', return_value=({}, [])),              patch.object(runner, 'ctl', return_value=snapshot),              patch.object(runner, 'stop'),              patch.object(runner.time, 'sleep'),              patch.object(runner, 'handoff', return_value={'confirmed': True}) as grasp,              patch.object(runner, 'start_engine', **fake[0]):
            self.assertEqual(runner.main(['--real', '--i-am-beside-the-robot']), 0)
            env = fake[1][0].env
            self.assertEqual(env['SUGARBOX_CONTRACT_ALLOW_DRIVE'], '1')
            self.assertEqual(env['SUGARBOX_RUN_MODE'], 'DRIVE')
            grasp.assert_called_once()
            self.assertIs(grasp.call_args.kwargs['look'], runner.look)
            self.assertIs(grasp.call_args.kwargs['drive'], runner.drive)
            grasp.reset_mock()
            self.assertEqual(runner.main(['--real', '--i-am-beside-the-robot',
                                          '--approach', 'legacy']), 0)
            self.assertEqual(grasp.call_args.kwargs, {})     # legacy may not reverse

    def test_the_engine_loads_while_the_robot_gets_ready_and_waits_for_go(self):
        # 2026-10-19: ~25 s per stage to the first usable frame, all of it in series.
        with patch.object(runner.subprocess, 'Popen') as popen:
            runner.start_engine({'SUGARBOX_RUN_MODE': 'DRIVE'})
        kwargs = popen.call_args.kwargs
        self.assertEqual(kwargs['env']['SUGARBOX_WAIT_GO'], '1')
        self.assertEqual(kwargs['env']['SUGARBOX_RUN_MODE'], 'DRIVE')
        self.assertIs(kwargs['stdin'], runner.subprocess.PIPE)
        engine = FakeEngine({})
        runner.go(engine, (1.13251, -0.0004))
        self.assertEqual(json.loads(engine.lines[0]), {'go': True, 'seed': [1.1325, -0.0004]})

    def real_run(self, fake, stow_ok=True):
        snapshot = copy.deepcopy(self.snapshot)
        snapshot['servo_deg'] = [90, 139, 0, 0, 90, 30]
        snapshot['chassis']['arm_pose'] = 'travel' if stow_ok else 'e1'
        commands = []
        def read(host, cmd):
            commands.append(cmd)
            return snapshot if cmd == 'status' else {'ok': True}
        with patch.object(runner, 'preflight', return_value=({}, [])), \
             patch.object(runner, 'ctl', side_effect=read), \
             patch.object(runner, 'stop'), patch.object(runner.time, 'sleep'), \
             patch.object(runner, 'handoff', return_value={'confirmed': True}) as grasp, \
             patch.object(runner, 'start_engine', **fake[0]), \
             patch.object(runner, 'kill_tree') as killed, patch('builtins.print'):
            code = runner.main(['--real', '--i-am-beside-the-robot'])
        return code, grasp, killed, commands

    def test_a_miss_after_the_approach_reached_the_box_has_its_own_exit(self):
        # mission_loop guards that box: backed out, it is in the gripper's blind zone.
        snapshot = copy.deepcopy(self.snapshot)
        snapshot['servo_deg'] = [90, 139, 0, 0, 90, 30]
        snapshot['chassis']['arm_pose'] = 'travel'
        with patch.object(runner, 'handoff', side_effect=RuntimeError('grasp was not confirmed')), \
             patch.object(runner, 'preflight', return_value=({}, [])), \
             patch.object(runner, 'ctl', return_value=snapshot), \
             patch.object(runner, 'stop'), patch.object(runner.time, 'sleep'), \
             patch.object(runner, 'start_engine', **engines(0)[0]), patch('builtins.print'):
            self.assertEqual(runner.main(['--real', '--i-am-beside-the-robot']),
                             runner.AT_BOX_EXIT)
        # Stopped short of the box: an ordinary failure.
        code, grasp, _, _ = self.real_run(engines(4, stopped_at(1.2)))
        self.assertEqual(code, 2)
        grasp.assert_not_called()

    def test_a_run_that_stops_near_the_box_is_a_miss_at_the_box(self):
        # 2026-10-20 review: held or timed out within the last metre, the box may be
        # in the gripper's blind zone after the usual 0.20 m back-out.
        for on_go, near, where in ((stopped_at(0.35, -0.04), True, {'x': 0.35, 'y': -0.04}),
                                   (stopped_at(1.20), False, None),
                                   (None, True, None)):           # no report: unknown
            with self.subTest(where=where, near=near):
                code, grasp, _, _ = self.real_run(engines(4, on_go))
                self.assertEqual(code, runner.AT_BOX_EXIT if near else 2)
                if where is None:
                    self.assertFalse(runner.AT_BOX_REPORT.exists())
                else:
                    self.assertEqual(json.loads(runner.AT_BOX_REPORT.read_text()), where)
        # Never started (no motion toward the box): an ordinary failure.
        code, _, _, _ = self.real_run(engines(0, ended=1))
        self.assertEqual(code, 2)
        if runner.AT_BOX_REPORT.exists():
            runner.AT_BOX_REPORT.unlink()

    def test_an_engine_that_died_loading_is_never_started_or_handed_over(self):
        fake = engines(0, ended=1)
        code, grasp, killed, _ = self.real_run(fake)
        self.assertEqual(code, 2)
        self.assertEqual(fake[1][0].lines, [])          # never told to go
        grasp.assert_not_called()
        killed.assert_not_called()                      # it had already ended

    def test_a_stage_that_fails_before_go_ends_the_waiting_engine(self):
        fake = engines(0)
        code, grasp, killed, commands = self.real_run(fake, stow_ok=False)
        self.assertEqual(code, 2)
        self.assertIn('stow', commands)
        self.assertEqual(fake[1][0].lines, [])
        killed.assert_called_once_with(fake[1][0])      # with its camera receiver
        grasp.assert_not_called()

    def test_enoch_hands_over_at_1m_and_stops_the_box_at_34cm_by_default(self):
        # 0.28 until 2026-10-21: the user saw every stop too close (0.31, then 0.34).
        env = self.launched_env([])
        self.assertEqual((env['SUGARBOX_CONTRACT_HANDOFF_M'], env['SUGARBOX_CONTRACT_FINAL_X_M']),
                         ('1.0', '0.34'))
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


class PlacementTests(unittest.TestCase):
    """--target placement (2026-10-18): drive the held object to the placement box."""

    def setUp(self):
        self.calls = []
        self.status = {'ok': True, 'servo_deg': [90, 140, 0, 0, 90, 120],
                       'chassis': {'moving': False, 'arm_busy': False, 'motors': [0, 0, 0, 0],
                                   'board_rx_age_s': 0.01, 'stopped_for_s': 1,
                                   'arm_pose': 'travel', 'maintenance': False},
                       'odom': {'valid': True, 'failures': 0, 'twist': [0, 0, 0]}}
        self.holding = True
        self.report = {'arrived': True, 'reason': 'PURSUIT_DONE x=0.211 y=-0.029',
                       'seeded': False, 'x': 0.211, 'y': -0.029}

    def read(self, host, command):
        self.calls.append(command)
        if command == 'status':
            return copy.deepcopy(self.status)
        if command == 'health':
            return {'ok': True, 'holding': self.holding}
        if command in ('release', 'release-preview'):
            self.holding = False
        return {'ok': True, 'outcome': 'released'}

    def place(self, **kwargs):
        options = dict(report=self.report, final_x=0.212, aim_y=-0.028, empty_test=False)
        options.update(kwargs)
        return runner.place('host', options.pop('report'), options.pop('final_x'),
                            options.pop('aim_y'), options.pop('empty_test'), read=self.read,
                            send_stop=lambda host: self.calls.append('stop'),
                            sleep=lambda s: None, now=lambda: 0, **options)

    def test_checked_stop_releases_then_stows_and_confirms_empty(self):
        self.place()
        self.assertEqual(self.calls, ['stop', 'status', 'release', 'stow', 'status', 'health'])

    def test_empty_rehearsal_uses_release_preview_which_stows_itself(self):
        self.place(empty_test=True)
        self.assertEqual(self.calls, ['stop', 'status', 'release-preview', 'status', 'health'])

    def test_an_unchecked_stop_never_releases(self):
        for report in (None, dict(self.report, reason='PURSUIT_TIMEOUT x=0.4 y=0'),
                       dict(self.report, seeded=True), dict(self.report, y=0.05)):
            with self.subTest(report=report):
                self.calls = []
                with self.assertRaises(RuntimeError):
                    self.place(report=report)
                self.assertNotIn('release', self.calls)

    def test_still_holding_after_release_is_a_failure(self):
        def stuck(host, command):
            self.calls.append(command)
            return copy.deepcopy(self.status) if command == 'status' else {'ok': True,
                                                                            'holding': True}
        with self.assertRaises(RuntimeError):
            runner.place('host', self.report, 0.212, -0.028, False, read=stuck,
                         send_stop=lambda host: None, sleep=lambda s: None, now=lambda: 0)

    def test_taught_wall_distance_corrects_the_stop_before_releasing(self):
        readings, moves = iter([0.505, 0.472]), []
        self.place(wall_m=0.470, wall=lambda host: next(readings),
                   drive=lambda host, m: moves.append(round(m, 3)))
        self.assertEqual(moves, [0.035])
        self.assertLess(self.calls.index('stop', 1), self.calls.index('release'))

    def test_a_wall_reading_far_off_never_moves_or_releases(self):
        moves = []
        with self.assertRaises(RuntimeError):
            self.place(wall_m=0.470, wall=lambda host: 0.80, drive=lambda host, m: moves.append(m))
        self.assertEqual(moves, [])
        self.assertNotIn('release', self.calls)

    def localize(self, poses, turn_clear=True):
        poses = iter(poses)
        return lambda host: {'ready': True, 'age': 0.1,
                             'state': {'pose': list(next(poses)), 'turn_clear': turn_clear}}

    def test_face_box_turns_until_the_map_position_is_ahead(self):
        turns = []
        pose = runner.face_box('host', {'x': 0.0, 'y': 1.0},
                               localize=self.localize([(0, 0, 0), (0, 0, 1.50)]),
                               rotate=lambda host, r: turns.append(round(r, 3)))
        self.assertEqual(turns, [round(math.pi / 2, 3)])
        self.assertEqual(pose, (0.0, 0.0, 1.5))

    def test_face_box_never_turns_without_room_and_gives_up_after_its_tries(self):
        turns = []
        with self.assertRaises(RuntimeError):
            runner.face_box('host', {'x': 0.0, 'y': 1.0},
                            localize=self.localize([(0, 0, 0)], turn_clear=False),
                            rotate=lambda host, r: turns.append(r))
        self.assertEqual(turns, [])
        with self.assertRaises(RuntimeError):
            runner.face_box('host', {'x': 0.0, 'y': 1.0},
                            localize=self.localize([(0, 0, 0)] * 4),
                            rotate=lambda host, r: turns.append(r))
        self.assertEqual(len(turns), 3)

    def launched_env(self, argv):
        with patch.dict(runner.os.environ, {}, clear=True), \
             patch.object(runner, 'preflight', return_value=({}, [])), \
             patch.object(runner, 'ctl', side_effect=self.read), \
             patch.object(runner.subprocess, 'run') as process:
            process.return_value.returncode = 0
            self.assertEqual(runner.main(['--dry-run', '--target', 'placement'] + argv), 0)
            return process.call_args.kwargs['env']

    def test_handoff_only_skips_the_approach_and_stows_nothing_first(self):
        # 2026-10-18: the arm camera dropped at the box; after replugging, only E1
        # was left to do, from where the robot stood.
        self.holding = False
        self.status['servo_deg'] = [90, 74, 8, 9, 90, 30]
        with patch.object(runner, 'preflight', return_value=({}, [])), \
             patch.object(runner, 'ctl', side_effect=self.read), \
             patch.object(runner, 'stop'), patch.object(runner.time, 'sleep'), \
             patch.object(runner, 'handoff', return_value={'confirmed': True}) as grasp, \
             patch.object(runner.subprocess, 'run') as process:
            self.assertEqual(runner.main(['--real', '--i-am-beside-the-robot',
                                          '--handoff-only']), 0)
        process.assert_not_called()
        grasp.assert_called_once()
        self.assertIs(grasp.call_args.kwargs['look'], runner.look)
        self.assertNotIn('stow', self.calls)
        for argv in (['--handoff-only', '--target', 'placement'],
                     ['--handoff-only', '--approach', 'legacy']):
            with self.subTest(argv=argv), patch('sys.stderr'), self.assertRaises(SystemExit):
                runner.main(argv)

    def test_placement_settings_reach_the_approach(self):
        env = self.launched_env([])
        self.assertEqual((env['SUGARBOX_TARGET'], env['SUGARBOX_CONTRACT_AIM_Y_M'],
                          env['SUGARBOX_YOLO_IMGSZ'], env['SUGARBOX_CONTRACT_FINAL_X_M']),
                         ('placement', '-0.028', '640', str(runner.dock.PLACEMENT_FINAL_X_M)))
        self.assertTrue(env['SUGARBOX_ARRIVAL_REPORT'].endswith('.json'))
        self.assertNotIn('SUGARBOX_SEED_TARGET', env)
        # The corner box is often beyond the calibrated area after a grasp.
        self.assertEqual(env['SUGARBOX_FAR_SEED'], '1')

    def dry_env(self, argv, inherited=None):
        with patch.dict(runner.os.environ, inherited or {}, clear=True),              patch.object(runner, 'preflight', return_value=({}, [])),              patch.object(runner, 'ctl', side_effect=self.read),              patch.object(runner.subprocess, 'run') as process:
            process.return_value.returncode = 0
            self.assertEqual(runner.main(['--dry-run'] + argv), 0)
        return process.call_args.kwargs['env']

    def test_the_grasp_approach_drives_at_a_far_sighting_too(self):
        # 2026-10-19: the sugarbox may stand anywhere, so a box beyond the calibrated
        # strip is driven at as for the placement box (never stopped on).
        env = self.dry_env([])
        self.assertEqual(env['SUGARBOX_FAR_SEED'], '1')
        for key in ('SUGARBOX_TARGET', 'SUGARBOX_SEED_TARGET'):
            self.assertNotIn(key, env)
        # Where a run stopped tells a miss at the box from one short of it.
        self.assertIn('SUGARBOX_ARRIVAL_REPORT', env)
        env = self.dry_env(['--approach', 'legacy'], {'SUGARBOX_FAR_SEED': '1'})
        self.assertNotIn('SUGARBOX_FAR_SEED', env)

    def test_the_patrol_sighting_seeds_the_grasp_approach(self):
        env = self.dry_env(['--seed', '1.2', '-0.3'])
        self.assertEqual(env['SUGARBOX_SEED_TARGET'], '1.2000,-0.3000')
        for argv in (['--seed', '0.2', '0'], ['--seed', '3.6', '0'], ['--seed', '1', '2'],
                     ['--seed', 'nan', '0'], ['--seed', '1', '0', '--target', 'placement'],
                     ['--seed', '1', '0', '--approach', 'legacy'],
                     ['--seed', '1', '0', '--handoff-only']):
            with self.subTest(argv=argv), patch.object(runner, 'ctl') as read,                  patch('sys.stderr'), self.assertRaises(SystemExit):
                runner.main(argv)
            read.assert_not_called()

    def test_preflight_picks_and_checks_the_placement_model(self):
        args = runner.argparse.Namespace(target='placement', sam=None, homography=None,
                                         gstreamer=None, turn_to_box=False)
        paths, issues = runner.preflight(args)
        self.assertEqual(paths['SUGARBOX_YOLO_MODEL'], runner.PLACEMENT_MODEL)
        self.assertFalse([i for i in issues if 'placement model' in i], issues)
        args.target = 'sugarbox'
        self.assertEqual(runner.preflight(args)[0]['SUGARBOX_YOLO_MODEL'], runner.SUGARBOX_MODEL)

    def test_placement_only_settings_are_refused_elsewhere(self):
        for argv in (['--empty-test'], ['--turn-to-box'], ['--wall-m', '0.5'],
                     ['--final-x-m', '0.21'],
                     ['--target', 'placement', '--approach', 'legacy'],
                     ['--target', 'placement', '--final-x-m', '0.15'],
                     ['--target', 'placement', '--wall-m', '3']):
            with self.subTest(argv=argv), patch.object(runner, 'ctl') as read, \
                 patch('sys.stderr'), self.assertRaises(SystemExit):
                runner.main(argv)
            read.assert_not_called()

    def run_real(self, argv=(), box=None, report=None, pose=(0.0, 0.0, 0.0), backs=None,
                 face=None):
        report = self.report if report is None else report
        backs = [] if backs is None else backs
        def approach(env, message):
            if 'SUGARBOX_ARRIVAL_REPORT' in env:
                Path(env['SUGARBOX_ARRIVAL_REPORT']).write_text(json.dumps(report))
        fake = engines(0, on_go=approach)
        with patch.dict(runner.os.environ, {}, clear=True), \
             patch.object(runner, 'preflight', return_value=({}, [])), \
             patch.object(runner, 'ctl', side_effect=self.read), \
             patch.object(runner, 'stop'), patch.object(runner.time, 'sleep'), \
             patch.object(runner, 'face_box', **({'side_effect': face} if face else
                                                 {'return_value': pose})) as facing, \
             patch.object(runner, 'placement_layout', return_value=(box, ROOM)), \
             patch.object(runner, 'back_straight', side_effect=lambda h, m, walls, **kw: backs.append(
                 (round(m, 3), kw.get('speed'), walls, kw.get('margin')))), \
             patch.object(runner, 'place', return_value={'ok': True}) as placing, \
             patch.object(runner, 'handoff') as grasp, \
             patch.object(runner, 'start_engine', **fake[0]):
            code = runner.main(['--real', '--i-am-beside-the-robot', '--target', 'placement']
                               + list(argv))
            return code, fake[1][0], placing, facing, grasp

    def test_real_placement_reads_the_arrival_report_and_never_grasps(self):
        code, process, placing, facing, grasp = self.run_real()
        self.assertEqual(code, 0)
        self.assertEqual(placing.call_args.args[1], self.report)
        grasp.assert_not_called()
        facing.assert_not_called()
        self.assertIn('stow', self.calls)            # the grasp left the object at E1

    def test_turn_to_box_seeds_the_approach_only_from_a_verified_map(self):
        verified = {'x': 1.2, 'y': 0.0, 'verified': True}
        code, process, _, facing, _ = self.run_real(['--turn-to-box'], box=verified)
        self.assertEqual(code, 0)
        facing.assert_called_once()
        # Known only after the turn: it goes with the start, not in the environment.
        self.assertEqual(process.go_seed(), [1.1325, 0.0])
        self.assertNotIn('SUGARBOX_SEED_TARGET', process.env)
        code, process, _, _, _ = self.run_real(['--turn-to-box'], box=dict(verified, verified=False))
        self.assertEqual(code, 0)
        self.assertIsNone(process.go_seed())

    def test_a_box_too_near_to_measure_is_backed_away_from_first(self):
        backs = []
        near = {'x': 0.60, 'y': 0.0, 'verified': True}
        code, process, _, facing, _ = self.run_real(['--turn-to-box'], box=near, backs=backs)
        self.assertEqual(code, 0)
        # Capped, slow, checked on the map with the user's 0.25 m left behind.
        self.assertEqual(backs, [(0.40, 0.06, ROOM, runner.PLACEMENT_BACKOFF_MARGIN_M)])
        self.assertEqual(runner.PLACEMENT_BACKOFF_MARGIN_M, 0.25)
        self.assertEqual(facing.call_count, 2)          # faced again from the new spot
        backs.clear()
        code, _, _, facing, _ = self.run_real(['--turn-to-box'], box={'x': 1.2, 'y': 0.0,
                                                                      'verified': False},
                                              backs=backs)
        self.assertEqual((code, backs, facing.call_count), (0, [], 1))

    def test_no_room_to_turn_backs_the_way_it_came_then_turns(self):
        clear = iter([False, False, True, True])
        poses = iter([(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 1.5)])
        backs, turns = [], []
        pose = runner.face_box(
            'host', {'x': 0.0, 'y': 1.0},
            localize=lambda host: {'ready': True, 'age': 0.1, 'state': {
                'pose': list(next(poses)), 'turn_clear': next(clear)}},
            rotate=lambda host, r: turns.append(r), back=lambda host, m: backs.append(m))
        self.assertEqual(backs, [0.10, 0.10])
        self.assertEqual(len(turns), 1)
        self.assertEqual(pose, (0, 0, 1.5))
        backs.clear()
        with self.assertRaises(RuntimeError):
            runner.face_box('host', {'x': 0.0, 'y': 1.0},
                            localize=self.localize([(0, 0, 0)] * 5, turn_clear=False),
                            rotate=lambda host, r: turns.append(r),
                            back=lambda host, m: backs.append(m))
        self.assertEqual(backs, [0.10] * 3)             # 0.30 m in all, then stop

    def test_no_room_to_turn_may_back_all_the_way_to_where_the_patrol_stopped(self):
        # 2026-10-21 run 14: a 10 cm back-off was refused with the box 0.65 m
        # behind-right; the ground back to the patrol's stop was just driven over.
        backs = []
        with self.assertRaises(RuntimeError):         # never turns: backs to the limit
            runner.face_box('host', {'x': 0.0, 'y': 1.0},
                            localize=self.localize([(0, 0, 0)] * 9, turn_clear=False),
                            rotate=lambda host, r: None,
                            back=lambda host, m: backs.append(round(m, 3)), back_max=0.42)
        self.assertEqual(backs, [0.10, 0.10, 0.10, 0.10])   # the last 2 cm not worth a step
        # The spot is straight behind: its distance; aside or ahead: not a retrace.
        self.assertAlmostEqual(runner.dock.retrace_room((1.0, 0.0, 0.0), (0.55, 0.05)), 0.45)
        self.assertIsNone(runner.dock.retrace_room((1.0, 0.0, 0.0), (0.55, 0.40)))
        self.assertIsNone(runner.dock.retrace_room((1.0, 0.0, 0.0), (1.30, 0.0)))
        self.assertIsNone(runner.dock.retrace_room(None, (0.55, 0.0)))

    def test_came_from_lets_the_back_off_pass_the_unverified_box_slack(self):
        box = {'x': 0.30, 'y': -1.20, 'verified': False}     # behind-right, as in run 14
        captured = []
        with patch.object(runner.dock, 'read_localization'), \
             patch.object(runner.dock, 'localization_pose', return_value=(1.0, 0.0, 0.0)):
            for argv in (['--turn-to-box'], ['--turn-to-box', '--came-from', '0.55', '0.05']):
                code, *_ = self.run_real(argv, box=box, face=lambda host, b, back, back_max: (
                    captured.append(back_max) or (1.0, 0.0, 0.0)))
                self.assertEqual(code, 0)
        self.assertEqual(captured[0], runner.FACE_BACKOFF_MAX_M)
        self.assertAlmostEqual(captured[1], 0.45)
        with self.assertRaises(SystemExit), patch('sys.stderr'):
            runner.main(['--came-from', '0.5', '0.0'])        # only with --turn-to-box

    def test_backing_goes_in_steps_each_checked_on_the_map(self):
        # Nothing sees behind the robot (the LiDAR's rear half is blocked by the
        # body): the AMCL pose and the room's walls decide.
        moves, poses = [], iter([(1.0, 1.0, 0.0), (0.8, 1.0, 0.0)])
        localize = lambda host: {'ready': True, 'age': 0.1, 'state': {'pose': list(next(poses))}}
        drive = lambda host, m, limit, speed: moves.append((round(m, 3), limit, speed))
        runner.back_straight('host', 0.40, ROOM, speed=0.06, localize=localize, drive=drive)
        self.assertEqual(moves, [(-0.2, 0.2, 0.06), (-0.2, 0.2, 0.06)])
        moves.clear()
        near_wall = lambda host: {'ready': True, 'age': 0.1, 'state': {'pose': [-0.6, 1.0, 0.0]}}
        # 0.10 m from the west wall after 0.20 m: fine at 0.08, refused at 0.25.
        room_left = lambda host: {'ready': True, 'age': 0.1, 'state': {'pose': [-0.47, 1.0, 0.0]}}
        runner.back_straight('host', 0.20, ROOM, localize=room_left, drive=drive)
        self.assertEqual(len(moves), 1)
        moves.clear()
        with self.assertRaises(RuntimeError):
            runner.back_straight('host', 0.20, ROOM, localize=room_left, drive=drive, margin=0.25)
        self.assertEqual(moves, [])
        with self.assertRaises(RuntimeError):         # the rear edge would reach the wall
            runner.back_straight('host', 0.20, ROOM, localize=near_wall, drive=drive)
        for walls, localize in ((ROOM, lambda host: None), (None, near_wall)):
            with self.subTest(walls=walls), self.assertRaises(RuntimeError):
                runner.back_straight('host', 0.20, walls, localize=localize, drive=drive)
        self.assertEqual(moves, [])
        for metres in (0.0, -0.1, 0.7, float('nan')):
            with self.subTest(metres=metres), self.assertRaises(ValueError):
                runner.back_straight('host', metres, ROOM, localize=near_wall, drive=drive)

    def test_placement_needs_a_held_object_and_an_unlocked_service(self):
        for change in ('empty', 'locked'):
            with self.subTest(change=change):
                self.setUp()
                if change == 'empty':
                    self.holding = False
                else:
                    self.status['chassis']['maintenance'] = True
                with patch.object(runner, 'preflight', return_value=({}, [])), \
                     patch.object(runner, 'ctl', side_effect=self.read), \
                     patch.object(runner, 'stop') as stopping, \
                     patch.object(runner.subprocess, 'run') as process:
                    self.assertEqual(runner.main(['--real', '--i-am-beside-the-robot',
                                                  '--target', 'placement']), 2)
                    stopping.assert_not_called()
                    process.assert_not_called()


if __name__ == '__main__':
    unittest.main()
