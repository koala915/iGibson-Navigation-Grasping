"""Placement target, map seed and arrival report in the approach engine.

Runs the engine's own TargetState, settings and report code (lifted from the
source, as the other approach tests do), without models, ROS or sockets.
"""
import ast
from dataclasses import dataclass
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from integration import sugarbox_training_contract as training_contract
from integration.sugarbox_ground_calibration import load_ground_calibration, pixel_to_ground

SOURCE = Path(__file__).resolve().parents[1] / 'integration/sugarbox_rl_approach_final2.py'
TREE = ast.parse(SOURCE.read_text(encoding='utf-8'))
SETTINGS = {'RUN_MODE', 'CONTRACT_HANDOFF_CENTER_M', 'TARGET_KIND', 'CONTRACT_FINAL_CENTER_X_M',
            'CONTRACT_FINAL_MIN_X_M', '_seed', 'SEED_TARGET', 'ARRIVAL_REPORT_PATH',
            'CONTRACT_AIM_Y_M', 'SEED_MIN_M', 'FAR_SEED', 'FAR_SEED_FRAMES',
            'FAR_SEED_MAX_ERROR_M', 'SEED_GIVE_UP_CENTER_M', 'TRAINING_CONTRACT'}
FUNCTIONS = {'get_age', 'arrival_record', 'write_arrival_report', 'coarse_ground'}
CLASSES = {'TargetState', 'FarSighting', 'SeedFacing'}


def engine(env=None):
    names = set(SETTINGS)
    classes = [n for n in TREE.body if isinstance(n, ast.ClassDef) and n.name in CLASSES]
    for node in classes:
        names.update(n.id for n in ast.walk(node) if isinstance(n, ast.Name) and n.id.isupper())
    body = []
    for node in TREE.body:
        if node in classes or (isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS):
            body.append(node)
        elif isinstance(node, ast.Assign) and any(
                isinstance(n, ast.Name) and n.id in names
                for t in node.targets for n in ast.walk(t)):
            body.append(node)
        elif isinstance(node, ast.If) and any(
                isinstance(n, ast.Name) and n.id in SETTINGS for n in ast.walk(node.test)):
            body.append(node)
    scope = {'math': math, 'json': json, 'dataclass': dataclass,
             'os': SimpleNamespace(environ=env or {}, replace=__import__('os').replace),
             'pixel_to_ground': pixel_to_ground, 'training_contract': training_contract}
    exec(compile(ast.Module(body=body, type_ignores=[]), str(SOURCE), 'exec'), scope)
    return scope


def sighting(x, y):
    return {'x': x, 'y': y, 'u': 320.0, 'v': 400.0,
            'det': {'conf': 0.9, 'class_name': 'placement_box'}}


class SeedTests(unittest.TestCase):
    def setUp(self):
        self.rt = engine()
        self.target = self.rt['TargetState']()
        self.target.seed(1.8, 0.2, 10.0)

    def test_a_seed_is_memory_to_drive_at_but_never_a_sighting(self):
        self.assertTrue(self.target.has_memory() and self.target.seeded)
        self.assertFalse(self.target.visual_recent(10.0))
        self.assertAlmostEqual(self.target.dist, math.hypot(1.8, 0.2))
        self.target.propagate_with_odom(0.2, 0.0, 10.4)               # odometry carries it
        self.assertAlmostEqual(self.target.x, 1.8 - 0.08)

    def test_the_camera_replaces_a_wrong_seed_outright(self):
        # 0.7 m from the seed: a remembered box would reject this as a mismatch.
        results = [self.target.consider_measurement(sighting(1.2, -0.4), 10.0 + 0.1 * i)
                   for i in range(3)]
        self.assertEqual([r[1] for r in results][:2], ['TARGET_ACQUIRING 1/3',
                                                      'TARGET_ACQUIRING 2/3'])
        self.assertEqual(results[2][:2], (True, 'TARGET_LOCKED'))
        self.assertFalse(self.target.seeded)
        self.assertEqual((self.target.x, self.target.y), (1.2, -0.4))  # no averaging with the seed

    def test_after_that_it_is_an_ordinary_remembered_box(self):
        for i in range(3):
            self.target.consider_measurement(sighting(1.2, -0.4), 10.0 + 0.1 * i)
        accepted, status, _, _ = self.target.consider_measurement(sighting(2.5, 1.0), 10.4)
        self.assertFalse(accepted)
        self.assertTrue(status.startswith('TARGET_MISMATCH'))

    def test_first_sighting_lock_mode_still_locks_over_a_seed(self):
        target = self.rt['TargetState'](freeze_visual_updates=True)
        target.seed(1.8, 0.2, 10.0)
        self.assertEqual(target.consider_measurement(sighting(1.2, -0.4), 10.1)[:2],
                         (True, 'TARGET_LOCKED'))
        self.assertFalse(target.seeded)

    def test_clear_forgets_the_seed(self):
        self.target.clear()
        self.assertFalse(self.target.seeded or self.target.has_memory())


class FarSeedTests(unittest.TestCase):
    """2026-10-18 dry run: the placement box sat in its corner, seen at conf 0.81 but
    outside the calibrated area, so the approach never locked and never moved."""

    def setUp(self):
        self.rt = engine({'SUGARBOX_TRAINING_CONTRACT': '1', 'SUGARBOX_FAR_SEED': '1'})
        self.H, self.front, _ = load_ground_calibration(ROOT / 'detection/rear_ground_homography.json')

    def test_the_corner_box_of_the_dry_run_gets_a_rough_position(self):
        # Box bottom at pixel (142, 214) in the 2026-10-18 frame.
        x, y = self.rt['coarse_ground'](142, 214, self.H, self.front)
        self.assertAlmostEqual(x, 1.62, delta=0.02)
        self.assertAlmostEqual(y, 0.47, delta=0.02)

    def test_pixels_without_a_plausible_floor_point_give_nothing(self):
        self.assertAlmostEqual(self.rt['coarse_ground'](320, 150, self.H, self.front)[0], 2.86,
                               delta=0.02)                       # 2.9 m straight ahead is fine
        # Above the horizon, and row 100 (6.5 m: past the 3.5 m a seed may be).
        for u, v in ((320, 60), (320, 0), (320, 100)):
            with self.subTest(u=u, v=v):
                self.assertIsNone(self.rt['coarse_ground'](u, v, self.H, self.front))

    def test_three_consistent_far_sightings_before_a_seed(self):
        far = self.rt['FarSighting']()
        self.assertIsNone(far.update((1.6, 0.45), 0.0))
        self.assertIsNone(far.update((1.7, 0.50), 0.4))
        x, y = far.update((1.65, 0.48), 0.8)
        self.assertAlmostEqual(x, 1.65)
        self.assertAlmostEqual(y, 0.4767, places=3)
        self.assertIsNone(far.update((2.4, -0.3), 1.2))         # a jump starts over
        self.assertIsNone(far.update(None, 2.5))                 # a long gap forgets
        self.assertIsNone(far.update((2.4, -0.3), 2.6))

    def test_reseeding_keeps_a_real_lock_that_is_half_way(self):
        # A box on the edge of the area alternates between far and measured frames.
        target = self.rt['TargetState']()
        target.seed(1.6, 0.4, 0.0, source='far_sighting')
        target.consider_measurement(sighting(1.30, 0.15), 0.4)
        target.consider_measurement(sighting(1.31, 0.15), 0.8)
        target.seed(1.55, 0.35, 1.2, source='far_sighting')
        self.assertEqual(target.consider_measurement(sighting(1.30, 0.16), 1.3)[:2],
                         (True, 'TARGET_LOCKED'))
        self.assertFalse(target.seeded)

    def test_give_up_is_past_the_handoff_but_while_the_camera_still_sees_the_box(self):
        rt = self.rt
        self.assertLess(rt['SEED_GIVE_UP_CENTER_M'], rt['CONTRACT_HANDOFF_CENTER_M'])
        self.assertGreater(rt['SEED_GIVE_UP_CENTER_M'], 0.64)


class SeedFacingTests(unittest.TestCase):
    """2026-10-18: a far sighting 25 deg to the left never entered the calibrated
    strip while the policy drove on; the seed is now faced first."""

    def setUp(self):
        self.rt = engine({'SUGARBOX_TRAINING_CONTRACT': '1', 'SUGARBOX_FAR_SEED': '1'})
        self.face = self.rt['SeedFacing']()
        self.target = self.rt['TargetState']()

    def at(self, degrees, seeded=True):
        self.target.seed(math.cos(math.radians(degrees)), math.sin(math.radians(degrees)), 0.0)
        self.target.seeded = seeded
        return self.face.command(self.target)

    def test_turns_in_place_toward_a_seed_off_to_the_side_until_it_is_ahead(self):
        self.assertEqual(self.at(25.0), self.rt['TURN_MIN_WZ'])       # left is +wz
        self.assertEqual(self.at(8.0), self.rt['TURN_MIN_WZ'])        # keeps turning
        self.assertEqual(self.at(4.0), 0.0)                           # faced: policy's turn
        self.assertEqual(self.at(10.0), 0.0)                          # small drift: no restart
        self.assertEqual(self.at(-15.0), -self.rt['TURN_MIN_WZ'])

    def test_a_measured_box_is_never_turned_to_by_this(self):
        self.assertEqual(self.at(40.0, seeded=False), 0.0)
        self.assertFalse(self.face.active)

    def test_the_turn_is_fast_enough_to_move_the_chassis_in_place(self):
        self.assertGreaterEqual(self.rt['TURN_MIN_WZ'], 1.0)

    def test_facing_branch_sits_before_the_policy_and_keeps_the_safety_filter(self):
        branch = next(n for n in ast.walk(TREE) if isinstance(n, ast.If)
                      and 'seed_facing.command(target)' in ast.unparse(n.test))
        body = ast.unparse(branch.body)
        self.assertIn('anti_jitter.apply_contract(', body)
        self.assertIn('policy_predict', ast.unparse(branch.orelse))     # the policy is the else


class SettingsTests(unittest.TestCase):
    def test_sugarbox_defaults_are_unchanged(self):
        rt = engine()
        self.assertEqual((rt['TARGET_KIND'], rt['CONTRACT_FINAL_CENTER_X_M'],
                          rt['CONTRACT_AIM_Y_M'], rt['SEED_TARGET']),
                         ('sugarbox', 0.28, -0.044, None))

    def test_only_a_placement_box_may_stop_inside_25cm(self):
        rt = engine({'SUGARBOX_TARGET': 'placement', 'SUGARBOX_CONTRACT_FINAL_X_M': '0.212',
                     'SUGARBOX_CONTRACT_AIM_Y_M': '-0.028'})
        self.assertEqual((rt['CONTRACT_FINAL_CENTER_X_M'], rt['CONTRACT_AIM_Y_M']),
                         (0.212, -0.028))
        for env in ({'SUGARBOX_CONTRACT_FINAL_X_M': '0.212'},
                    {'SUGARBOX_TARGET': 'placement', 'SUGARBOX_CONTRACT_FINAL_X_M': '0.15'},
                    {'SUGARBOX_TARGET': 'bin'}, {'SUGARBOX_CONTRACT_AIM_Y_M': '0.2'}):
            with self.subTest(env=env), self.assertRaises(ValueError):
                engine(env)

    def test_seed_must_be_ahead_and_in_reach(self):
        self.assertEqual(engine({'SUGARBOX_SEED_TARGET': '1.5,-0.3'})['SEED_TARGET'], (1.5, -0.3))
        for bad in ('1.5', '0.1,0', '4.0,0', '-1.0,0.1', '0.5,1.5', 'nan,0'):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                engine({'SUGARBOX_SEED_TARGET': bad})


class ArrivalReportTests(unittest.TestCase):
    def test_report_says_how_it_ended_and_is_strict_json(self):
        rt = engine({'SUGARBOX_TARGET': 'placement', 'SUGARBOX_CONTRACT_FINAL_X_M': '0.212'})
        target = rt['TargetState']()
        for i in range(3):
            target.consider_measurement(sighting(0.9, 0.01), 5.0 + 0.1 * i)
        record = rt['arrival_record'](True, False, 'PURSUIT_DONE x=0.210 y=-0.029', '', target,
                                      9.0, float('inf'), 0.48)
        self.assertEqual((record['arrived'], record['seeded'], record['target_kind']),
                         (True, False, 'placement'))
        self.assertTrue(record['reason'].startswith('PURSUIT_DONE'))
        self.assertIsNone(record['front_robust_m'])
        self.assertAlmostEqual(record['visual_age_s'], 3.8)
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder) / 'arrival.json')
            rt['write_arrival_report'](path, record)
            self.assertEqual(json.loads(Path(path).read_text(encoding='utf-8')), record)

    def test_a_blocked_run_reports_why(self):
        rt = engine()
        target = rt['TargetState']()
        target.seed(1.0, 0.0, 1.0)
        record = rt['arrival_record'](False, True, '', 'SEED_NOT_CONFIRMED: 0.98 m', target,
                                      2.0, 0.5, 0.5)
        self.assertEqual((record['arrived'], record['seeded']), (False, True))
        self.assertTrue(record['reason'].startswith('SEED_NOT_CONFIRMED'))
        self.assertIsNone(record['visual_age_s'])


if __name__ == '__main__':
    unittest.main()
