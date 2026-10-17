import ast
import math
from pathlib import Path
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "integration" / "sugarbox_rl_approach_final2.py"
tree = ast.parse(SOURCE.read_text(encoding="utf-8-sig"))
# Load the actual filter without initializing cameras, models, or motors.
nodes = []
for node in tree.body:
    if isinstance(node, ast.Assign) and any(
        isinstance(target, ast.Name) and target.id.startswith("AVOID_TURN_")
        for target in node.targets
    ):
        nodes.append(node)
    if isinstance(node, ast.ClassDef) and node.name == "AntiJitterSafetyFilter":
        nodes.append(node)
namespace = {"math": math}
exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec"), namespace)
Filter = namespace["AntiJitterSafetyFilter"]


class TurnFilterTests(unittest.TestCase):
    def setUp(self):
        self.filter = Filter()

    def turn(self, wz, front=0.35):
        return self.filter._stabilize_turn_direction(wz, front)[0]

    def test_same_direction_preserves_magnitude(self):
        self.assertEqual(self.turn(0.78), 0.78)
        self.assertEqual(self.turn(0.31), 0.31)

    def test_switch_waits_at_zero_in_both_directions(self):
        for sign in (-1, 1):
            self.filter.reset_motion()
            self.assertEqual(self.turn(sign * 0.78), sign * 0.78)
            self.assertEqual(self.turn(-sign * 0.4), 0.0)
            self.assertEqual(self.turn(-sign * 0.6), -sign * 0.6)

    def test_alternation_never_reverses_requested_direction(self):
        for wz in (0.78, -0.78, 0.78, -0.78, 0.4, -0.4):
            actual = self.turn(wz)
            self.assertGreaterEqual(actual * wz, 0.0)
            self.assertLessEqual(abs(actual), abs(wz))

    def test_small_or_zero_command_interrupts_confirmation(self):
        for small in (0.0, 0.05, -0.05):
            self.filter.reset_motion()
            self.turn(0.78)
            self.assertEqual(self.turn(-0.78), 0.0)
            self.assertEqual(self.turn(small), small)
            self.assertEqual(self.turn(-0.78), 0.0)
            self.assertEqual(self.turn(-0.78), -0.78)

    def test_return_to_old_direction_resets_confirmation(self):
        self.turn(0.78)
        self.assertEqual(self.turn(-0.78), 0.0)
        self.assertEqual(self.turn(0.4), 0.4)
        self.assertEqual(self.turn(-0.78), 0.0)

    def test_clearance_resets_lock_and_passes_command(self):
        self.turn(0.78)
        self.assertEqual(self.turn(-0.78, 0.55), -0.78)
        self.assertEqual(self.filter.avoid_turn_sign, 0.0)
        self.assertEqual(self.turn(-0.4), -0.4)


if __name__ == "__main__":
    unittest.main()
