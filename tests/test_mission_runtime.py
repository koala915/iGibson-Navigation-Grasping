"""Resident resource lifetime, fresh-frame and E1 regressions. No robot/network/GPU."""
import ast
import copy
import importlib.util
import io
import json
import multiprocessing as mp
from pathlib import Path
import queue
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0, str(ROOT))
from integration import mission_runtime as runtime
from integration.robot_rpc import RobotSession, SERVER

spec = importlib.util.spec_from_file_location('runtime_runner', ROOT / 'integration/sugarbox_approach_grasp.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def fake_stage(env):
    """Production worker protocol with lightweight resources and a fresh local target."""
    directory = Path(env['fixture'])
    module = SimpleNamespace(YOLO_MODEL_PATH=str(directory / env['target']),
        SAM2_MODEL_PATH=str(directory / 'sam'), RL_MODEL_PATH=str(directory / 'ppo'),
        VECNORMALIZE_PATH=str(directory / 'norm'), YOLO_IMGSZ=640)
    module.YOLO = lambda path: {'kind': 'yolo', 'path': path, 'uses': 0}
    module.SAM = lambda path: {'kind': 'sam', 'path': path, 'uses': 0}
    module.load_policy_and_vecnormalize = lambda: (object(), SimpleNamespace(training=False, norm_reward=False))
    module.warm_up_models = lambda yolo, sam: print('WARM ' + Path(yolo['path']).name, flush=True)
    local_state = []

    def main(resources=None, go_stream=None, headless=False):
        assert headless and not local_state
        local_state.append('new target')
        yolo, sam = resources.models(module)
        assert yolo['uses'] < 2  # the same cached model, but new stage-local state
        yolo['uses'] += 1
        resources.policy(module)
        message = json.loads(go_stream.readline())
        assert message.get('go') is True
        print('FAKE RUN ' + env['target'], flush=True)
        if env.get('hang'):
            time.sleep(10)
        return int(env.get('returncode', 0))
    module.main = main
    return module


def fake_worker(connection, frames):
    runtime.worker(connection, frames, stage_loader=fake_stage)


class FrameTests(unittest.TestCase):
    def test_only_new_frames_cross_a_stage_boundary_and_receipt_time_is_preserved(self):
        frames = runtime.SharedFrames()
        frame = np.zeros((480, 640, 3), np.uint8)
        frames.publish(frame, wall=100.0, monotonic=10.0)
        lease = runtime.FrameLease(frames, 10.0)
        self.assertEqual(lease.latest(), (None, 0.0))
        frame[0, 0, 0] = 42
        frames.publish(frame, wall=100.2, monotonic=10.2)
        received, stamp = lease.latest()
        self.assertEqual((received[0, 0, 0], stamp), (42, 10.2))
        self.assertEqual(lease.read()[2], 100.2)
        received_copy = received.copy()
        frames.publish(np.ones_like(frame), wall=101, monotonic=11)
        self.assertTrue(np.array_equal(received, received_copy))
        lease.close()
        self.assertEqual(lease.latest(), (None, 0.0))
        self.assertEqual(runtime.FrameLease(frames, 10.5).latest()[1], 11)

    def test_wrong_frame_format_is_refused(self):
        frames = runtime.SharedFrames()
        for frame in (np.zeros((10, 10, 3), np.uint8), np.zeros((480, 640, 3))):
            with self.assertRaises(ValueError):
                frames.publish(frame)

    def test_leases_close_without_closing_the_shared_receiver(self):
        resource = runtime.SharedFrames()
        a, b = runtime.FrameLease(resource, 0), runtime.FrameLease(resource, 0)
        a.close()
        resource.publish(np.zeros((480, 640, 3), np.uint8))
        self.assertIsNotNone(b.latest()[0])

    def test_a_lock_orphaned_by_worker_termination_has_a_bounded_failure(self):
        frames = runtime.SharedFrames()
        frames.lock = Mock()
        frames.lock.acquire.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'lock unavailable'):
            frames.snapshot()
        with self.assertRaisesRegex(RuntimeError, 'lock unavailable'):
            frames.publish(np.zeros((480,640,3), np.uint8))
        frames.lock.release.assert_not_called()
        frames.lock.acquire.assert_called_with(timeout=1.0)


class ResourceTests(unittest.TestCase):
    def test_assets_load_once_and_changes_require_a_new_mission(self):
        cache = runtime.Resources(None)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'weight'
            path.write_bytes(b'123')
            loader = Mock(return_value=object())
            a = cache.get('yolo', [path], loader)
            self.assertIs(cache.get('yolo', [path], loader), a)
            loader.assert_called_once()
            path.write_bytes(b'changed')
            with self.assertRaisesRegex(RuntimeError, 'asset changed'):
                cache.get('yolo', [path], loader)

    def test_inference_only_normalization_and_poisoned_worker_are_enforced(self):
        cache = runtime.Resources(None)
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder) / 'weight'
            p.touch()
            module = SimpleNamespace(RL_MODEL_PATH=p, VECNORMALIZE_PATH=p,
                load_policy_and_vecnormalize=lambda: (object(), SimpleNamespace(training=True, norm_reward=False)))
            with self.assertRaisesRegex(RuntimeError, 'inference-only'):
                cache.policy(module)
            cache.poisoned = True
            with self.assertRaisesRegex(RuntimeError, 'previous stage'):
                cache.get('yolo', [p], lambda: object())

    def test_new_stage_keeps_ros_reactor_alive_but_drops_its_subscriptions(self):
        source = ast.parse((ROOT/'integration/sugarbox_rl_approach_final2.py').read_text(encoding='utf-8'))
        cls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'RosSensorBridge')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'close')
        namespace = {}
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<ros-close>', 'exec'), namespace)
        bridge = SimpleNamespace(client=Mock(), scan_topic=Mock(), odom_topic=Mock())
        namespace['close'](bridge, terminate=False)
        bridge.scan_topic.unsubscribe.assert_called_once()
        bridge.odom_topic.unsubscribe.assert_called_once()
        bridge.client.close.assert_called_once()
        bridge.client.terminate.assert_not_called()
        namespace['close'](bridge)
        bridge.client.terminate.assert_called_once()

    def test_detection_receipt_is_read_atomically_with_the_position(self):
        import threading
        source = ast.parse((ROOT/'grasp/v23/x3plus_real_grasp.py').read_text(encoding='utf-8'))
        cls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'DetectionReceiver')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'snapshot_with_receipt')
        namespace = {}
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<receipt>', 'exec'), namespace)
        receiver = SimpleNamespace(_lock=threading.Lock(), _last_update_ts=12.0,
            _is_stale=lambda: False, _pos=np.array([0.26, 0, 0.03]), _height=0.06,
            _pose_stamp=('e1',), _default_pos=np.zeros(3), _default_height=0.0)
        data = namespace['snapshot_with_receipt'](receiver)
        self.assertEqual((data[2], data[4]), (True, 12.0))
        receiver._pos[:] = 1
        self.assertAlmostEqual(data[0][0], 0.26)
        receiver._is_stale = lambda: True
        data = namespace['snapshot_with_receipt'](receiver)
        self.assertEqual((data[2], data[4]), (False, 0.0))


class ProcessTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.path = Path(self.folder.name)
        for name in ('sugarbox', 'placement', 'sam', 'ppo', 'norm'):
            (self.path / name).touch()
        self.runtime = runtime.MissionRuntime('fake', 'not-started', self.path,
                                              process_target=fake_worker)
        self.addCleanup(self.runtime.close)
        self.env = {'fixture': str(self.path), 'target': 'sugarbox'}
        self.runtime.prepare([self.env, dict(self.env, target='placement')])
        # A fake receiver prevents this process test from opening UDP/GStreamer.
        self.runtime.receiver = Mock()

    def run_engine(self, **env):
        engine = self.runtime.start_engine(dict(self.env, **env))
        self.assertIsNone(engine.poll())       # no run before explicit go
        engine.write('{"go":true}\n')
        return engine, engine.wait(timeout=10)

    def test_warm_assets_survive_two_jobs_while_each_job_has_new_state(self):
        process = self.runtime.process.pid
        with patch('builtins.print'):
            _, first = self.run_engine()
            _, second = self.run_engine()
        self.assertEqual((first, second), (0, 0))
        self.assertEqual(self.runtime.process.pid, process)
        log = (self.path / 'runtime.log').read_text(encoding='utf-8')
        # start_engine intentionally starts a fresh engine log, prepare has its
        # own log in real missions. Model warm-up itself must never repeat.
        self.assertNotIn('WARM', log)

    def test_second_owner_is_refused_and_abort_ends_the_worker(self):
        engine = self.runtime.start_engine(self.env)
        with self.assertRaisesRegex(RuntimeError, 'already owns'):
            self.runtime.start_engine(self.env)
        engine.abort()
        self.assertFalse(self.runtime.process.is_alive())
        with self.assertRaisesRegex(RuntimeError, 'unavailable'):
            self.runtime.start_engine(self.env)

    def test_timeout_cancels_the_worker_and_cannot_replay(self):
        engine = self.runtime.start_engine(dict(self.env, hang=True))
        engine.write('{"go":true}\n')
        with self.assertRaises(runner.subprocess.TimeoutExpired):
            engine.wait(timeout=0.1)
        engine.abort()
        self.assertFalse(self.runtime.process.is_alive())

    def test_wrong_generation_cannot_start_an_engine(self):
        engine = self.runtime.start_engine(self.env)
        self.runtime.connection.send({'op': 'go', 'generation': engine.generation+1,
                                      'line': '{"go":true}\n'})
        with patch('builtins.print'), self.assertRaises(RuntimeError):
            engine.wait(timeout=10)
        self.assertFalse(self.runtime.process.is_alive())

    def test_runtime_run_stage_closes_every_engine_before_the_next_stage(self):
        starts = []
        fake_runner = SimpleNamespace()
        def main(argv):
            self.assertEqual(argv[:2], ['--host', 'fake'])
            engine = fake_runner._MISSION_RUNTIME.start_engine(self.env)
            starts.append(engine)
            engine.write('{"go":true}\n')
            return engine.wait(timeout=10)
        fake_runner.main = main
        with patch('builtins.print'), runtime.resident_session(fake_runner, self.runtime):
            self.assertEqual(self.runtime.run_stage(fake_runner, ['--target','sugarbox'], self.path/'one.log'), 0)
            self.assertIsNone(self.runtime.active)
            self.assertEqual(self.runtime.run_stage(fake_runner, ['--target','sugarbox'], self.path/'two.log'), 0)
        self.assertFalse(self.runtime.process.is_alive())
        self.assertIsNone(fake_runner._MISSION_RUNTIME)
        self.assertTrue((self.path/'one.engine.log').is_file())
        records = [json.loads(line) for line in (self.path/'timings.jsonl').read_text().splitlines()]
        self.assertEqual([r['event'] for r in records],
                         ['models_ready','approach','stage','approach','stage'])

    def test_one_receiver_survives_all_reader_leases(self):
        self.runtime.receiver = None
        receiver = Mock()
        with patch('integration.rear_stream.StreamReader', return_value=receiver) as factory:
            first = self.runtime.reader()
            first.close()
            second = self.runtime.reader()
            self.runtime.frames.publish(np.zeros((480,640,3), np.uint8),
                                        monotonic=second.since+0.001)
            self.assertIsNotNone(second.latest()[0])
            factory.assert_called_once()
            receiver.close.assert_not_called()
            second.close()
        self.runtime.close()
        receiver.close.assert_called_once()

    def test_worker_fault_is_terminal_and_never_reuses_the_previous_job(self):
        engine = self.runtime.start_engine(dict(self.env, returncode=None))
        engine.write('{"go":true}\n')
        with patch('builtins.print'), self.assertRaises(RuntimeError):
            engine.wait(timeout=10)
        with self.assertRaisesRegex(RuntimeError, 'unavailable'):
            self.runtime.start_engine(self.env)


class FakeProcess:
    def __init__(self, alter=lambda message: message):
        self.alter, self.lines, self.sent = alter, queue.Queue(), []
        self.stdin = SimpleNamespace(write=self.write, flush=lambda: None, close=lambda: self.lines.put(None))
        self.stdout = iter(self.lines.get, None)

    def write(self, text):
        request = json.loads(text)
        self.sent.append(request)
        reply = self.alter({'id': request['id'], 'reply': {'ok': True, 'confirmed': True}})
        if reply is not None:
            self.lines.put(json.dumps(reply)+'\n')

    def wait(self, timeout=None):
        return 0

    def kill(self):
        self.lines.put(None)


class RpcTests(unittest.TestCase):
    def session(self, alter=lambda message: message):
        process = FakeProcess(alter)
        factory = Mock(return_value=process)
        session = RobotSession('fake', popen=factory)
        self.addCleanup(session.close)
        return session, process, factory

    def test_read_and_action_commands_share_one_ssh_connection(self):
        session, process, factory = self.session()
        for command in ('status', 'home', 'health'):
            self.assertTrue(session.request('fake', command, 1)['ok'])
        factory.assert_called_once()
        self.assertEqual([r['id'] for r in process.sent], [1, 2, 3])

    def test_no_reply_or_a_wrong_one_closes_the_channel_and_is_never_replayed(self):
        for alter in (lambda r: None, lambda r: dict(r, id=999),
                      lambda r: {'id': r['id'], 'error': 'socket timed out'}):
            session, process, factory = self.session(alter)
            with self.assertRaises((RuntimeError, queue.Empty)):
                session.request('fake', 'grasp', 0.01)
            with self.assertRaises(RuntimeError):
                session.request('fake', 'grasp', 0.01)
            self.assertEqual(len(process.sent), 1)
            factory.assert_called_once()

    def test_a_refusal_or_missed_grasp_leaves_the_channel_for_the_next_command(self):
        # 2026-10-20 review: after a grasp missed at the box the loop must still read
        # the arm's state, or a recoverable miss becomes "arm state unknown".
        for reply in ({'ok': False}, {'ok': True, 'confirmed': False}):
            answers = iter([reply, {'ok': True}])
            session, process, factory = self.session(
                lambda r: dict(r, reply=next(answers)))
            with self.assertRaises(RuntimeError):
                session.request('fake', 'grasp', 1)
            self.assertTrue(session.request('fake', 'status', 1)['ok'])
            self.assertEqual([r['command'] for r in process.sent], ['grasp', 'status'])
            factory.assert_called_once()

    def channels(self, *alters):
        """A session whose n-th ssh channel answers through alters[n]."""
        processes = [FakeProcess(alter) for alter in alters]
        factory = Mock(side_effect=processes)
        session = RobotSession('fake', popen=factory)
        self.addCleanup(session.close)
        return session, processes, factory

    def test_a_lost_read_is_asked_again_on_a_new_channel_that_stays_in_use(self):
        # 2026-10-21: one WiFi drop used to close the session and end the mission.
        for lost in (lambda r: None, lambda r: dict(r, id=999),
                     lambda r: {'id': r['id'], 'error': 'socket timed out'}):
            session, (first, second), factory = self.channels(lost, lambda r: r)
            with patch('builtins.print'):
                self.assertTrue(session.request('fake', 'status', 0.05)['ok'])
            self.assertTrue(session.request('fake', 'home', 1)['ok'])
            self.assertEqual(factory.call_count, 2)
            self.assertEqual([r['command'] for r in first.sent], ['status'])
            self.assertEqual([r['command'] for r in second.sent], ['status', 'home'])

    def test_a_read_lost_twice_fails_but_leaves_the_session_open(self):
        lost = lambda r: None
        session, (_, _, third), factory = self.channels(lost, lost, lambda r: r)
        with patch('builtins.print'), self.assertRaises(queue.Empty):
            session.request('fake', 'health', 0.05)
        self.assertTrue(session.request('fake', 'stow', 1)['ok'])
        self.assertEqual([r['command'] for r in third.sent], ['stow'])
        self.assertEqual(factory.call_count, 3)

    def test_a_channel_that_ended_while_idle_is_replaced_before_any_command(self):
        session, (first, second), factory = self.channels(lambda r: r, lambda r: r)
        self.assertTrue(session.request('fake', 'status', 1)['ok'])
        first.stdin.close()                            # the ssh channel ends while idle
        deadline = time.monotonic() + 2
        while session.replies.empty() and time.monotonic() < deadline:
            time.sleep(0.01)                           # its end has reached the queue
        self.assertTrue(session.request('fake', 'stow', 1)['ok'])
        self.assertEqual([r['command'] for r in first.sent], ['status'])
        self.assertEqual([r['command'] for r in second.sent], ['stow'])

    def test_an_arm_command_whose_reply_is_lost_is_never_sent_again(self):
        session, (first,), factory = self.channels(lambda r: None)
        with self.assertRaises(queue.Empty):
            session.request('fake', 'stow', 0.05)
        with self.assertRaises(RuntimeError):          # unknown outcome: closed for good
            session.request('fake', 'status', 1)
        self.assertEqual([r['command'] for r in first.sent], ['stow'])
        factory.assert_called_once()

    def test_other_robot_is_refused_without_starting_a_connection(self):
        session, _, factory = self.session()
        with self.assertRaises(ValueError):
            session.request('other', 'status', 1)
        factory.assert_not_called()

    def test_remote_program_is_python36_and_uses_health_socket(self):
        ast.parse(SERVER, feature_version=(3, 6))
        connection = Mock()
        connection.makefile.return_value = io.StringIO('{"ok":true}\n')
        import socket
        output = io.StringIO()
        with patch.object(socket, 'AF_UNIX', 1, create=True), \
                patch.object(socket, 'socket', return_value=connection), \
                patch('sys.stdin', io.StringIO('{"id":1,"command":"health"}\n')), \
                patch('sys.stdout', output):
            exec(SERVER, {})
        connection.connect.assert_called_once_with('/tmp/grasp_health.sock')
        connection.settimeout.assert_called_once_with(2)
        self.assertTrue(json.loads(output.getvalue())['reply']['ok'])


class FastE1Tests(unittest.TestCase):
    def setUp(self):
        self.clock, self.calls = 0.0, []
        self.status = {'chassis': {'moving': False, 'arm_busy': False,
            'motors': [0, 0, 0, 0], 'board_rx_age_s': 0.01, 'stopped_for_s': 1, 'arm_pose': 'e1'},
            'odom': {'valid': True, 'failures': 0, 'twist': [0, 0, 0]},
            'detection': {'fresh': True, 'pos': [0.26, 0, 0.0325]},
            'sample_monotonic': 100, 'stationary_since_monotonic': 100}

    def sleep(self, seconds):
        self.clock += seconds

    def read(self, host, command):
        self.calls.append(command)
        if command != 'status':
            return {'ok': True, 'confirmed': True, 'e1_frame_receipts': True}
        result = copy.deepcopy(self.status)
        result['sample_monotonic'] = 100+self.clock
        result['detection']['receipt_monotonic'] = 100 + int(self.clock/0.4)*0.4
        return result

    def flow(self, read=None, look=None, drive=None):
        return runner.handoff('fake', read=read or self.read, send_stop=lambda host: None,
            sleep=self.sleep, now=lambda: self.clock, look=look, drive=drive)

    def test_two_new_post_motion_detections_can_finish_before_the_fixed_wait_path(self):
        self.assertTrue(self.flow()['confirmed'])
        self.assertLess(self.clock, 3.6)
        self.assertGreaterEqual(self.clock, 1.5)
        self.assertEqual(self.calls.count('grasp'), 1)

    def test_one_repeated_detection_is_never_two_agreeing_frames(self):
        def read(host, command):
            result = self.read(host, command)
            if command == 'status':
                result['sample_monotonic'] = 120+self.clock
                result['detection']['receipt_monotonic'] = 102
            return result
        with self.assertRaises(RuntimeError):
            self.flow(read)
        self.assertNotIn('grasp', self.calls)

    def test_missing_or_pre_motion_metadata_does_not_grasp(self):
        for receipt in (None, 100.1, float('nan'), True):
            self.setUp()
            def read(host, command):
                result = self.read(host, command)
                if command == 'status':
                    result['detection']['receipt_monotonic'] = receipt
                return result
            with self.subTest(receipt=receipt), self.assertRaises(RuntimeError):
                self.flow(read)
            self.assertNotIn('grasp', self.calls)

    def test_same_blank_frame_cannot_trigger_repeated_creep(self):
        def read(host, command):
            result = self.read(host, command)
            if command == 'status':
                result['detection']['fresh'] = False
            return result
        drive = Mock()
        with patch('builtins.print'), self.assertRaises(RuntimeError):
            self.flow(read, look=lambda host: {'t': 1.0, 'age': 0.1, 'seen': False}, drive=drive)
        drive.assert_not_called()

    def test_motion_epoch_change_is_a_failure_even_if_the_next_frame_agrees(self):
        def read(host, command):
            result = self.read(host, command)
            if command == 'status' and self.clock >= 2:
                result['stationary_since_monotonic'] = 99
            return result
        with self.assertRaisesRegex(RuntimeError, 'motion epoch'):
            self.flow(read)
        self.assertNotIn('grasp', self.calls)


class MissionWiringTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('resident_mission', ROOT/'integration/mission_loop.py')
        self.mission = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.mission)
        self.args = SimpleNamespace(host='fake', gstreamer=Path(__file__), homography=Path(__file__))

    def test_warm_up_happens_before_patrol_and_cleanup_restores_both_contexts(self):
        mission, order = self.mission, []
        resident = Mock()
        resident.host = 'fake'
        def prepare(envs):
            order.append(('prepare', len(envs)))
            for env in envs:
                self.assertEqual(Path(env['SUGARBOX_RL_MODEL']),
                                 ROOT/'integration/nav_best_model/doorway_ft_final.zip')
                self.assertEqual(Path(env['SUGARBOX_VECNORMALIZE']),
                                 ROOT/'integration/nav_best_model/doorway_ft_final_vecnormalize.pkl')
        resident.prepare.side_effect = prepare
        def run(args, pc, folder):
            order.append('patrol')
            self.assertIs(mission._runtime, resident)
            self.assertIs(mission.runner._MISSION_RUNTIME, resident)
            self.assertIsNotNone(mission.runner._ROBOT_SESSION)
            mission.run_stage('fake', ['--target','sugarbox'], Path(folder)/'stage.log')
            return 0
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(runtime, 'MissionRuntime', return_value=resident), \
                patch('integration.robot_rpc.RobotSession') as robot, \
                patch.object(mission, 'stream_running', return_value=True), \
                patch.object(mission.runner, 'preflight', return_value=({}, [])), \
                patch.object(mission.runner, 'stop') as stop, patch('builtins.print'):
            self.assertEqual(mission.with_runtime(self.args, 'fake', Path(folder), run), 0)
        self.assertEqual(order, [('prepare',2), 'patrol'])
        self.assertIsNone(mission._runtime)
        self.assertIsNone(mission.runner._MISSION_RUNTIME)
        self.assertIsNone(mission.runner._ROBOT_SESSION)
        robot.return_value.close.assert_called_once()
        resident.run_stage.assert_called_once()
        self.assertTrue(stop.called)

    def test_warm_up_failure_never_enters_patrol(self):
        mission = self.mission
        resident, run = Mock(), Mock()
        resident.prepare.side_effect = RuntimeError('model load failed')
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(runtime, 'MissionRuntime', return_value=resident), \
                patch('integration.robot_rpc.RobotSession'), \
                patch.object(mission, 'stream_running', return_value=True), \
                patch.object(mission.runner, 'preflight', return_value=({}, [])), \
                patch.object(mission.runner, 'stop'), patch('builtins.print'):
            self.assertEqual(mission.with_runtime(self.args, 'fake', Path(folder), run), 2)
        run.assert_not_called()
        self.assertIsNone(mission._runtime)
        resident.close.assert_called()

    def test_patrol_uses_the_resident_detector_without_a_second_model(self):
        resident = Mock()
        resident.detect.return_value = []
        with patch.object(self.mission, '_runtime', resident), \
                patch.object(self.mission, 'load_ground_calibration', return_value=(np.eye(3),0.2,[])):
            watcher = self.mission.SugarboxWatcher(Path('fixture'))
            self.assertIsNone(watcher.look(np.zeros((480,640,3), np.uint8)))
        resident.detect.assert_called_once()
        self.assertEqual(resident.detect.call_args.args[0], self.mission.runner.SUGARBOX_MODEL)


if __name__ == '__main__':
    mp.freeze_support()
    unittest.main()
