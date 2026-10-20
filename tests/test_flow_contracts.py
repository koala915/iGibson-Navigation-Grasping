"""Cross-stage safety regressions, with no robot, models, camera or network."""
import argparse
from collections import deque
import copy
import importlib.util
import io
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('flow_contract_fakes', ROOT / 'tests/test_mission_loop.py')
fakes = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fakes)
mission = fakes.mission
runner = mission.runner


class StationaryLink(fakes.FakeLink):
    pass


class PatrolSafetyTests(unittest.TestCase):
    def run_candidate(self, point, link=None, confirmed=None):
        clock = [100.0]
        with patch.object(mission, 'confirm_parked', return_value=confirmed), \
                patch.object(mission, 'start_patrol', side_effect=AssertionError(
                    'must not start through an unresolved visible candidate')) as start, \
                patch('builtins.print'):
            result = mission.patrol_until_sugarbox(
                'mock', 'scene', fakes.ClockReader(clock),
                fakes.FakeWatcher([point] * 30), link or StationaryLink(),
                walls=[(-1, -1), (2, -1), (2, 2), (-1, 2)], now=lambda: clock[0])
        start.assert_not_called()
        return result

    def test_visible_rejected_box_does_not_become_five_clear_frames(self):
        with self.assertRaises(mission.MissionError):
            self.run_candidate((1.0, 0.0))

    def test_no_map_exclusion_without_a_pose_matching_the_frame(self):
        class MovingLink(StationaryLink):
            def frame_pose(self, stamp, moving=True):
                return self.pose, 0.0, 0.5       # taken mid-turn: cannot be placed
        link = MovingLink(pose=(0.0, 0.2, 0.5))
        actual = (0.8, 0.6)
        parked_point = mission.dock.relative_point(link.pose, actual)
        clock = [100.0]
        with patch.object(mission, 'confirm_parked', return_value=(parked_point, False)), \
                patch.object(mission, 'start_patrol', side_effect=AssertionError(
                    'moving frame was incorrectly excluded as a wall clip')), \
                patch('builtins.print'):
            result = mission.patrol_until_sugarbox(
                'mock', 'scene', fakes.ClockReader(clock),
                fakes.FakeWatcher([(0.8, 0.4)] * 10), link,
                walls=[(-1, -1), (1, -1), (1, 1), (-1, 1)], now=lambda: clock[0])
        self.assertAlmostEqual(result['map'][0], actual[0])
        self.assertAlmostEqual(result['map'][1], actual[1])

    def test_an_unlocated_rejection_stays_stopped(self):
        class UnknownFrameLink(StationaryLink):
            def frame_pose(self, stamp, moving=True):
                return None
        with self.assertRaises(mission.MissionError):
            self.run_candidate((1.0, 0.0), UnknownFrameLink())


class SearchSafetyTests(unittest.TestCase):
    def test_search_rejects_no_clearance_stale_and_missing_localization(self):
        records = [
            None,
            {'ready': False, 'age': 0.0, 'state': {'pose': [0, 0, 0], 'turn_clear': True}},
            {'ready': True, 'age': 2.0, 'state': {'pose': [0, 0, 0], 'turn_clear': True}},
            {'ready': True, 'age': 0.0, 'state': {'pose': [0, 0, 0], 'turn_clear': False}},
            {'ready': True, 'age': 0.0, 'state': {'pose': [0, 0, 0]}},
        ]
        for record in records:
            with self.subTest(record=record):
                clock = [100.0]
                with patch.object(runner.dock, 'read_localization', return_value=record), \
                        patch('builtins.print'), \
                        self.assertRaises((mission.MissionError, RuntimeError, ValueError)):
                    mission.search_placement(
                        'mock', fakes.ClockReader(clock), fakes.FakeWatcher([]),
                        turn=lambda *a: self.fail('unsafe search turn requested'),
                        now=lambda: clock[0])


class ApproachFailureTests(unittest.TestCase):
    def run_failure(self, code, map_spot, sightings=None):
        args = argparse.Namespace(host='mock', scene='scene', homography=Path('h.json'),
                                  gstreamer=None, wall_m=None, max_deliveries=None)
        room = unittest.mock.Mock()
        sighting = {'point': (0.98, 0.0), 'map': map_spot}
        sightings = [sighting] if sightings is None else sightings
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(mission.dock, 'fetch_layout', return_value=fakes.LAYOUT), \
                patch.object(mission, 'stream_running', return_value=True), \
                patch.object(mission, 'SugarboxWatcher'), \
                patch('integration.rear_stream.StreamReader'), \
                patch.object(mission, 'patrol_until_sugarbox', side_effect=sightings) as patrol, \
                patch.object(mission, 'ensure_arm_camera', return_value=None), \
                patch.object(mission, 'unlock'), \
                patch.object(mission, 'approach_seed', return_value=(0.98, 0.0)), \
                patch.object(mission, 'run_stage', return_value=code) as stage, \
                patch.object(mission, 'arm_state', return_value='empty'), \
                patch.object(mission, 'arm_camera_state', return_value='ok'), \
                patch.object(mission, 'box_left_at', return_value=None), \
                patch.object(mission, 'to_travel'), \
                patch.object(runner, 'stop') as stop, patch('builtins.print'):
            result = mission.patrol_mission(args, 'mock', Path(directory), room=room,
                                            new_link=lambda host: StationaryLink())
        stop.assert_called_once()
        return result, patrol, stage, room

    def test_a_clean_failure_short_of_the_box_backs_out_and_patrols_on(self):
        # The user, 2026-10-20 (option C): the 10-19 table. With SeedLook the run-3
        # failure now means the camera looked from standing still and did not
        # measure the box, not that the robot drove at an estimate.
        result, patrol, stage, room = self.run_failure(2, (0.98, 0.0), [
            {'point': (0.98, 0.0), 'map': (0.98, 0.0)}, None])
        self.assertEqual((result, patrol.call_count, stage.call_count), (0, 2, 1))
        room.leave.assert_called_once_with()
        self.assertEqual(patrol.call_args_list[1].kwargs['skip'], [])   # once is not twice

    def test_even_an_at_box_miss_cannot_restart_without_a_hazard_position(self):
        result, patrol, stage, room = self.run_failure(runner.AT_BOX_EXIT, None)
        self.assertEqual((result, patrol.call_count, stage.call_count), (2, 1, 1))
        room.leave.assert_not_called()


class StageContractTests(unittest.TestCase):
    def setUp(self):
        self.args = argparse.Namespace(host='mock', wall_m=None,
            homography=ROOT / 'CUSTOM_CALIBRATION.json', gstreamer=ROOT / 'CUSTOM_GST.exe')

    def assert_assets(self, args):
        for option, value in [('--homography', self.args.homography),
                              ('--gstreamer', self.args.gstreamer)]:
            self.assertIn(option, args)
            self.assertEqual(Path(args[args.index(option) + 1]), value)

    def test_grasp_forwards_the_same_calibration_and_decoder(self):
        with patch.object(mission, 'ensure_arm_camera', return_value=None), \
                patch.object(mission, 'unlock'), \
                patch.object(mission, 'approach_seed', return_value=(1.0, 0.0)), \
                patch.object(mission, 'run_stage', return_value=0) as stage, \
                patch.object(mission, 'arm_state', return_value='holding'):
            mission.fetch_sugarbox(self.args, {'point': (1, 0), 'map': None}, ROOT, 1)
        self.assert_assets(stage.call_args.args[1])

    def test_placement_and_retry_forward_the_same_assets(self):
        with patch.object(mission, 'run_stage', side_effect=[2, 0]) as stage, \
                patch.object(mission, 'arm_state', side_effect=['holding', 'empty']), \
                patch.object(mission, 'to_travel'), \
                patch.object(mission, 'search_placement', return_value=True), patch('builtins.print'):
            mission.place_held(self.args, ROOT / 'layout.json', ROOT, 1,
                              lambda: fakes.ClockReader([0.0]), lambda: object())
        for call in stage.call_args_list:
            self.assert_assets(call.args[1])

    def test_resume_place_unlocks_before_starting_and_preserves_assets(self):
        self.args.resume_at = 'place'
        unlocked = []

        def ctl(host, command):
            return {'chassis': {'maintenance': not unlocked}} if command == 'status' else {'holding': True}

        def stage(host, argv, log=None):
            self.assertTrue(unlocked, 'placement started before maintenance unlock')
            self.assert_assets(argv)
            return 0

        with tempfile.TemporaryDirectory() as directory, \
                patch.object(mission, 'stream_running', return_value=True), \
                patch.object(runner, 'ctl', side_effect=ctl), \
                patch.object(runner, 'service_command', side_effect=lambda h, c: unlocked.append(c)), \
                patch.object(runner, 'stop'), patch.object(mission, 'run_stage', side_effect=stage), \
                patch('builtins.print'):
            self.assertEqual(mission.grasp_and_place(self.args, 'mock', Path(directory)), 0)
        self.assertEqual(unlocked, ['maintenance-unlock'])

    def test_no_patrol_grasp_and_placement_share_assets(self):
        self.args.resume_at = None
        calls = []

        def ctl(host, command):
            return {'chassis': {'maintenance': False}} if command == 'status' else {'holding': True}

        with tempfile.TemporaryDirectory() as directory, \
                patch.object(mission, 'stream_running', return_value=True), \
                patch.object(mission, 'ensure_arm_camera', return_value=None), \
                patch.object(runner, 'ctl', side_effect=ctl), patch.object(runner, 'stop'), \
                patch.object(mission, 'run_stage', side_effect=lambda h, a, log=None:
                             calls.append(a) or 0), patch('builtins.print'):
            self.assertEqual(mission.grasp_and_place(self.args, 'mock', Path(directory)), 0)
        self.assertEqual(len(calls), 2)
        for args in calls:
            self.assert_assets(args)

    def test_resume_never_starts_a_stage_when_unlock_is_refused(self):
        self.args.resume_at = 'place'
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(mission, 'stream_running', return_value=True), \
                patch.object(runner, 'ctl', return_value={'chassis': {'maintenance': True}}), \
                patch.object(runner, 'service_command', side_effect=RuntimeError('busy')), \
                patch.object(runner, 'stop'), patch.object(mission, 'run_stage') as stage, \
                patch('builtins.print'):
            self.assertEqual(mission.grasp_and_place(self.args, 'mock', Path(directory)), 2)
        stage.assert_not_called()


class FramePoseTests(unittest.TestCase):
    def link(self, samples, now=11.0):
        link = mission.VisionLink.__new__(mission.VisionLink)
        link.clock = lambda: now
        link.lock = threading.Lock()
        link.pose_history = deque(samples, maxlen=100)
        link.pose = samples[-1][1] if samples else None
        link.pose_at = samples[-1][0] if samples else None
        return link

    def test_a_still_robot_places_the_frame_exactly(self):
        pose = (0.0, 0.2, 0.0)
        samples = [(at, pose) for at in (9.9, 10.0, 10.1, 10.2, 10.3, 10.4, 10.5, 10.6)]
        self.assertEqual(self.link(samples).frame_pose(10.5), (pose, 0.0, 0.0))

    def test_motion_and_turning_within_the_frames_window_are_bounded(self):
        # The frame arrived at 10.5, so it was taken in 10.2..10.5.
        samples = [(10.1, (0.00, 0, 0.0)), (10.2, (0.01, 0, 0.0)), (10.3, (0.02, 0, 0.0)),
                   (10.4, (0.03, 0, 0.0)), (10.5, (0.04, 0, 0.3)), (10.6, (0.05, 0, 0.3))]
        pose, moved, turned = self.link(samples).frame_pose(10.5)
        self.assertAlmostEqual(moved, 0.02, places=6)
        self.assertAlmostEqual(turned, 0.3)
        # Turning 0.3 rad blurs a sighting 1 m away by 0.3 m: no exclusion.
        self.assertGreater(moved + 1.0 * turned, mission.SPOT_ERROR_MAX_M)

    def test_time_the_samples_do_not_cover_counts_at_full_speed(self):
        pose = (0.0, 0.0, 0.0)
        found = self.link([(10.4, pose), (10.45, pose)], now=10.5).frame_pose(10.5)
        self.assertIsNotNone(found)
        _, moved, turned = found
        # 10.2..10.4 and 10.45..10.5 uncovered: 0.25 s at the patrol's top speeds.
        self.assertAlmostEqual(moved, mission.PATROL_MAX_VX * 0.25)
        self.assertAlmostEqual(turned, mission.PATROL_MAX_WZ * 0.25)

    def test_stale_missing_future_and_gapped_frames_have_no_pose(self):
        pose = (0.0, 0.0, 0.0)
        self.assertIsNone(self.link([]).frame_pose(10.5))
        self.assertIsNone(self.link([(9.0, pose), (10.0, pose)]).frame_pose(10.5))  # stale
        self.assertIsNone(self.link([(9.0, pose), (10.9, pose)]).frame_pose(12.0))  # future
        self.assertIsNone(self.link([(9.0, pose), (10.9, pose)]).frame_pose(10.5))  # gap

    def test_yaw_wrap_is_not_mistaken_for_a_full_turn(self):
        poses = [(at, (0.0, 0.0, math.pi - 0.001 if i % 2 else -math.pi + 0.001))
                 for i, at in enumerate((10.1, 10.2, 10.3, 10.4, 10.5, 10.6))]
        self.assertLess(self.link(poses).frame_pose(10.5)[2], 0.01)

    def test_pose_age_is_not_reset_by_receipt_and_bad_reports_clear_the_cache(self):
        link = self.link([], now=10.0)
        good = {'t': 100.0, 'stamp': 99.6, 'ready': True, 'pose': [0, 0, 0]}
        link.proc = unittest.mock.Mock(stdout=io.StringIO(json.dumps(good) + '\n'))
        link._read()
        self.assertAlmostEqual(link.pose_at, 9.6)
        self.assertEqual(link.pose_now(), (0.0, 0.0, 0.0))
        link.clock = lambda: 10.2
        self.assertIsNone(link.pose_now())
        for bad in ({'ready': False}, dict(good, stamp=99.0),
                    dict(good, pose=[float('nan'), 0, 0]), None):
            with self.subTest(report=bad):
                link.pose, link.pose_at = (0, 0, 0), 10.2
                link.pose_history.append((10.2, link.pose))
                link.proc.stdout = io.StringIO(json.dumps(bad) + '\n')
                link._read()
                self.assertIsNone(link.pose_now())
                self.assertEqual(list(link.pose_history), [])


class ResidentProtocolTests(unittest.TestCase):
    def replies(self):
        return fakes.resident_reply('mock', 'status'), fakes.resident_reply('mock', 'health')

    def test_known_g2_protocol_is_accepted_but_missing_fields_are_not_invented(self):
        status, health = self.replies()
        self.assertIsNone(runner.service_protocol_problem(status, health))
        for name, bad in [('stack_version', 'v21'), ('service_pid', None), ('service_pid', True),
                          ('holding', None), ('chassis_tcp_ready', False), ('chassis', None),
                          ('ok', False)]:
            with self.subTest(field=name, value=bad):
                invalid = dict(health, **{name: bad})
                self.assertIn('protocol', runner.service_protocol_problem(status, invalid))
        status['chassis'].pop('maintenance')
        self.assertIn('protocol', runner.service_protocol_problem(status, health))
        self.assertIn('protocol', runner.service_protocol_problem(None, health))

    def test_incompatible_protocol_stops_mission_preflight_before_any_stage(self):
        args = ['--host', '10.0.0.1', '--no-patrol', '--real', '--i-am-beside-the-robot',
                '--gstreamer', __file__]
        status, health = self.replies()
        health.pop('stack_version')
        with patch.object(runner, 'ctl', side_effect=lambda h, c: health if c == 'health' else status), \
                patch.object(mission, 'ensure_arm_camera', return_value=None), \
                patch.object(Path, 'is_file', return_value=True), \
                patch.object(mission, 'run_stage') as stage, \
                patch.object(mission, 'pc_address') as address, patch('builtins.print'):
            self.assertEqual(mission.main(args), 2)
        stage.assert_not_called()
        address.assert_not_called()

    def test_standalone_preflight_also_checks_the_actual_service_contract(self):
        args = argparse.Namespace(target='sugarbox', sam=Path(__file__),
            homography=Path(__file__), gstreamer=Path(__file__), host='mock', turn_to_box=False)
        status, health = self.replies()
        for compatible in (True, False):
            reply = copy.deepcopy(health)
            if not compatible:
                reply.pop('holding')
            with self.subTest(compatible=compatible), \
                    patch.object(Path, 'is_file', return_value=True), \
                    patch.object(runner, 'MODEL_HASHES', {}), \
                    patch.object(runner, 'load_ground_calibration'), \
                    patch.object(runner.importlib.util, 'find_spec', return_value=True), \
                    patch.object(runner, 'ctl', side_effect=lambda h, c:
                                 reply if c == 'health' else status), \
                    patch.object(runner, 'service_command') as command:
                issues = runner.preflight(args)[1]
            self.assertEqual(bool(issues), not compatible)
            command.assert_not_called()


class GraspctlProtocolTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('flow_graspctl', ROOT / 'grasp/v23/graspctl.py')
        self.client = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.client)

    def invoke(self, args, reply):
        class FakeSocket:
            def settimeout(sock, value):
                sock.timeout = value

            def connect(sock, path):
                sock.path = path

            def sendall(sock, value):
                sock.sent = value

            def makefile(sock, mode):
                return io.StringIO(json.dumps(reply) + '\n')

            def close(sock):
                sock.closed = True
        sock = FakeSocket()
        with patch.object(self.client.socket, 'AF_UNIX', 1, create=True), \
                patch.object(self.client.socket, 'socket', return_value=sock), patch('builtins.print'):
            code = self.client.main(['graspctl.py'] + args)
        return code, sock

    def test_health_uses_bounded_independent_socket_and_explicit_override(self):
        code, sock = self.invoke(['health'], {'ok': True, 'holding': False})
        self.assertEqual((code, sock.path, sock.sent),
                         (0, '/tmp/grasp_health.sock', b'health\n'))
        self.assertEqual(sock.timeout, 2.0)
        self.assertTrue(sock.closed)
        code, sock = self.invoke(['health', '/tmp/custom.sock'], {'ok': True})
        self.assertEqual((code, sock.path), (0, '/tmp/custom.sock'))

    def test_preview_uses_arm_socket_and_cannot_bypass_a_service_refusal(self):
        for ok, expected in ((True, 0), (False, 1)):
            code, sock = self.invoke(['release-preview'], {'ok': ok})
            self.assertEqual(code, expected)
            self.assertEqual((sock.path, sock.sent),
                             ('/tmp/grasp_service.sock', b'release-preview\n'))

    def test_malformed_or_truthy_nonboolean_ok_is_never_success(self):
        for reply in ([], None, {'ok': 'yes'}, {'ok': 1}, {}):
            with self.subTest(reply=reply):
                self.assertEqual(self.invoke(['health'], reply)[0], 1)


if __name__ == '__main__':
    unittest.main()
