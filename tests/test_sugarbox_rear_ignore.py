"""The rear camera's view of the arm camera is never taken for the sugarbox."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

SOURCE = Path(__file__).resolve().parents[1] / 'integration/sugarbox_rl_approach_final2.py'
NAMES = {'overlap_fraction', 'select_best_box', 'REAR_IGNORE_REGIONS', 'REAR_IGNORE_MIN_OVERLAP'}


def runtime():
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    selected = [node for node in tree.body
                if (isinstance(node, ast.FunctionDef) and node.name in NAMES)
                or (isinstance(node, ast.Assign) and any(
                    isinstance(t, ast.Name) and t.id in NAMES for t in node.targets))]
    namespace = {}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(SOURCE), 'exec'), namespace)
    return namespace


class Tensor:
    def __init__(self, value):
        self.value = np.asarray(value, dtype=float)

    def __getitem__(self, index):
        return Tensor(self.value[index])

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.value

    def item(self):
        return float(self.value)


def result(*boxes):
    return SimpleNamespace(boxes=[SimpleNamespace(conf=Tensor([conf]), cls=Tensor([0]),
                                                  xyxy=Tensor([xyxy]))
                                  for xyxy, conf in boxes])


class RearIgnoreTests(unittest.TestCase):
    def setUp(self):
        self.rt = runtime()
        self.best = lambda *boxes: self.rt['select_best_box'](result(*boxes), {0: 'sugarbox'})

    def test_arm_camera_never_beats_the_real_box(self):
        # 2026-10-17 frame: the arm camera at 0.90, the real box at 0.87.
        arm_camera = ((308, 409, 391, 479), 0.90)
        box = ((451, 258, 471, 301), 0.87)
        self.assertEqual(self.best(arm_camera, box)['bbox'], (451, 258, 471, 301))
        self.assertIsNone(self.best(arm_camera))

    def test_a_box_merely_touching_the_region_still_counts(self):
        near = ((280, 330, 330, 400), 0.8)         # bottom edge dips into the region
        self.assertEqual(self.best(near)['bbox'], (280, 330, 330, 400))

    def test_overlap_is_measured_on_the_detection(self):
        overlap = self.rt['overlap_fraction']
        self.assertEqual(overlap((0, 0, 10, 10), (20, 20, 30, 30)), 0.0)
        self.assertAlmostEqual(overlap((0, 0, 10, 10), (5, 0, 30, 30)), 0.5)
        self.assertAlmostEqual(overlap((0, 0, 10, 10), (-5, -5, 30, 30)), 1.0)


if __name__ == '__main__':
    unittest.main()
