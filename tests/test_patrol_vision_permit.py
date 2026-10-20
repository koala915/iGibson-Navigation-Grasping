"""Jetson end of the patrol's vision permit: no robot, socket or ROS."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    'permit', Path(__file__).resolve().parents[1] / 'integration/patrol_vision_permit_jetson.py')
permit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(permit)


class PermitTests(unittest.TestCase):
    def test_the_source_stamp_is_now_minus_the_frames_age(self):
        self.assertEqual(permit.permit_from_line('{"allow": true, "age": 0.4}', 100.0),
                         (True, 99.6))
        self.assertEqual(permit.permit_from_line('{"allow": false, "age": 0.4}', 100.0),
                         (False, 99.6))

    def test_anything_doubtful_denies(self):
        for line in ('junk', '{}', '{"allow": true}', '{"allow": true, "age": -0.1}',
                     '{"allow": true, "age": 9}', '{"allow": "yes", "age": 0.2}',
                     '{"allow": true, "age": null}'):
            with self.subTest(line=line):
                self.assertFalse(permit.permit_from_line(line, 100.0)[0])

    def test_the_permit_file_is_what_the_patrol_reads(self):
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder) / 'permit.json')
            permit.write_permit(path, 4443, 'boot-1', True, 99.6)
            self.assertEqual(json.loads(Path(path).read_text()),
                             {'owner_pid': 4443, 'boot_id': 'boot-1', 'source_stamp': 99.6,
                              'allow': True})
            self.assertEqual([p.name for p in Path(folder).iterdir()], ['permit.json'])

    def test_poses_come_from_the_localization_health_file(self):
        with tempfile.TemporaryDirectory() as folder:
            health = Path(folder) / 'health.json'
            health.write_text(json.dumps({'ready': True, 'stamp': 99.9,
                                          'state': {'pose': [1.0, 2.0, 0.5]}}))
            self.assertEqual(permit.pose_line(str(health), 100.0),
                             {'t': 100.0, 'ready': True, 'stamp': 99.9, 'pose': [1.0, 2.0, 0.5]})
            line = permit.pose_line(str(Path(folder) / 'missing.json'), 100.0)
            self.assertFalse(line['ready'])


    def test_a_watch_line_names_the_patrol_and_writes_no_permit(self):
        self.assertEqual(permit.watched_pid('{"watch": 4321}'), 4321)
        for line in ('{"allow": true, "age": 0.2}', '{"watch": "4321"}', '{"watch": true}',
                     '{"watch": -1}', 'junk', '[1]'):
            with self.subTest(line=line):
                self.assertIsNone(permit.watched_pid(line))

    def test_a_patrol_is_alive_until_it_exits_or_turns_zombie(self):
        with tempfile.TemporaryDirectory() as proc:
            for pid, state in ((10, 'S'), (11, 'R'), (12, 'Z')):
                (Path(proc) / str(pid)).mkdir()
                (Path(proc) / str(pid) / 'stat').write_text(
                    '{} (python (patrol)) {} 1 2 3'.format(pid, state))
            self.assertTrue(permit.process_alive(10, proc))
            self.assertTrue(permit.process_alive(11, proc))
            self.assertFalse(permit.process_alive(12, proc))
            self.assertFalse(permit.process_alive(13, proc))   # gone

if __name__ == '__main__':
    unittest.main()
