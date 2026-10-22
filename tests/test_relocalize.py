"""Explicitly requested relocalization (integration/relocalize.py), no robot."""
import json
import subprocess
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from integration import relocalize as reloc  # noqa: E402

LOCALIZED = {'ok': True, 'localized': True, 'seed_written': True, 'status': 'LOCALIZED',
             'seed': {'pose': [0.14, 0.16, -0.34]}, 'stable_marker_ids': [2]}


class FakeRobot:
    """Answers the ssh commands relocalize() sends, in order of need."""

    def __init__(self, captures=(True,), seed_report=LOCALIZED, camera_busy=0, capture_ids=(2,)):
        self.captures, self.seed_report = list(captures), seed_report
        self.camera_busy, self.commands = camera_busy, []
        self.capture_ids = list(capture_ids)

    def __call__(self, host, command, timeout):
        self.commands.append(command)
        out, code = '', 0
        if command.startswith('pkill'):
            code = 0
        elif command.startswith('fuser'):
            code = 0 if self.camera_busy > 0 else 1
            self.camera_busy -= 1
        elif command.startswith('cat ') and 'aruco_corner_landmarks' in command:
            out = json.dumps({'registration_id': 'coarse_wall_layout_20261021_v3'})
        elif 'g2_corner_camera_capture.py' in command:
            ok = self.captures.pop(0)
            out = json.dumps({'ok': True, 'ids': self.capture_ids}) if ok else ''
            code = 0 if ok else 1
        elif 'g2_scan_match.py' in command:
            code = 0
        elif 'g2_aruco_corner_seed.py' in command:
            out = json.dumps(self.seed_report)
        return subprocess.CompletedProcess(command, code, out, 'RuntimeError: V4L2 source timestamp')


def health(pose):
    return {'ready': True, 'age': 0.1, 'state': {'pose': list(pose)}}


class RelocalizeTests(unittest.TestCase):
    def run_reloc(self, robot, poses=((0.15, 0.16, -0.33),)):
        poses = iter(poses)
        with patch.object(reloc, 'ssh', side_effect=robot), \
             patch.object(reloc.dock, 'read_localization', side_effect=lambda host: health(next(poses))), \
             patch.object(reloc.time, 'sleep'):
            return reloc.relocalize('10.0.0.1', log=lambda line: None)

    def test_frees_the_camera_then_captures_matches_and_seeds_in_order(self):
        robot = FakeRobot()
        pose = self.run_reloc(robot)
        self.assertAlmostEqual(pose[0], 0.15)
        steps = [c.split()[0] if not c.startswith('cd ') else c.split()[4] for c in robot.commands]
        self.assertEqual(steps, ['pkill', 'fuser', 'cat', 'deploy/ros/g2_corner_camera_capture.py',
                                 'integration/g2_scan_match.py', 'integration/g2_aruco_corner_seed.py'])
        capture = robot.commands[3]
        self.assertIn('--registration-id coarse_wall_layout_20261021_v3', capture)
        self.assertIn('--real', robot.commands[5])
        self.assertIn('--reference-map-yaml ' + reloc.ACTIVE_MAP, robot.commands[4])

    def test_a_first_capture_glitch_is_retried_once(self):
        robot = FakeRobot(captures=(False, True))
        self.run_reloc(robot)
        self.assertEqual(sum('capture.py' in c for c in robot.commands), 2)
        with self.assertRaises(reloc.RelocalizeError):
            self.run_reloc(FakeRobot(captures=(False, False)))

    def test_ambiguous_markers_stop_with_what_to_do(self):
        report = {'ok': False, 'status': 'AMBIGUOUS', 'seed_written': False,
                  'stable_marker_ids': [0]}
        with self.assertRaises(reloc.RelocalizeError) as raised:
            self.run_reloc(FakeRobot(seed_report=report))
        self.assertIn('AMBIGUOUS', str(raised.exception))
        self.assertIn('another marker', str(raised.exception))
        self.assertIn('two markers together', str(raised.exception))
        self.assertIn('markers [0]', str(raised.exception))

    def test_failed_seed_report_uses_ids_seen_by_camera_when_ids_are_missing(self):
        for status in ('AMBIGUOUS', 'NO_MATCH'):
            report = {'ok': False, 'status': status, 'seed_written': False}
            with self.subTest(status=status), self.assertRaises(reloc.RelocalizeError) as raised:
                self.run_reloc(FakeRobot(seed_report=report, capture_ids=(5,)))
            message = str(raised.exception)
            self.assertIn(status, message)
            self.assertIn('markers [5]', message)
            self.assertNotIn('markers None', message)

    def test_explanation_preserves_report_ids_and_no_match_guidance(self):
        self.assertIn('markers [7]', reloc.explain(
            {'status': 'AMBIGUOUS', 'stable_marker_ids': [7]}, seen=[5]))
        self.assertIn('markers [5]', reloc.explain(
            {'status': 'NO_MATCH', 'stable_marker_ids': []}, seen=[5]))
        self.assertIn('verified markers', reloc.explain({'status': 'NO_MATCH'}, seen=[5]))
        self.assertEqual(reloc.explain({'status': 'ERROR', 'reason': 'bad input'}), 'ERROR: bad input')

    def test_the_service_must_be_ready_where_the_seed_put_it(self):
        far = (0.95, 0.83, 2.88)                     # the old, turned-around pose
        with patch.object(reloc, 'READY_WAIT_S', 0.0):
            with self.assertRaises(reloc.RelocalizeError) as raised:
                self.run_reloc(FakeRobot(), poses=[far] * 5)
        self.assertIn('from the seed', str(raised.exception))
        # A short wait for the service to pick the seed up is fine.
        pose = self.run_reloc(FakeRobot(), poses=[far, far, (0.14, 0.17, -0.30)])
        self.assertLess(abs(pose[2] + 0.30), 1e-9)

    def test_a_camera_still_in_use_is_not_taken(self):
        with patch.object(reloc.time, 'monotonic', side_effect=[0.0, 0.0, 10.0]), \
             patch.object(reloc, 'ssh', side_effect=FakeRobot(camera_busy=99)), \
             patch.object(reloc.time, 'sleep'), self.assertRaises(reloc.RelocalizeError):
            reloc.free_rear_camera('10.0.0.1')

    def test_last_json_takes_the_report_after_other_output(self):
        self.assertEqual(reloc.last_json('loading\n{"status": "LOCALIZED"}\n'),
                         {'status': 'LOCALIZED'})
        self.assertIsNone(reloc.last_json('Traceback\n'))


if __name__ == '__main__':
    unittest.main()
