"""Blind finish and motor packets: real runtime logic, no model/ROS/socket."""
import ast
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

SOURCE = Path(__file__).resolve().parents[1] / 'integration/sugarbox_rl_approach_final2.py'


def runtime(env=None):
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    classes = {'BlindFinishController', 'TargetState', 'MotorClient', 'AntiJitterSafetyFilter'}
    names = {'_blind', 'YOLO_IMGSZ', 'BLIND_STOP_X', 'MOTOR_SEND_INTERVAL'}
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name in classes:
            names.update(n.id for n in ast.walk(node)
                         if isinstance(n, ast.Name) and n.id.isupper())
    selected = []
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name in classes:
            selected.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name in {'get_age', 'clamp'}:
            selected.append(node)
        elif isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id in names for t in node.targets):
            selected.append(node)
        elif isinstance(node, ast.If) and any(
                isinstance(n, ast.Name) and n.id in {'BLIND_STOP_X', 'YOLO_IMGSZ'}
                for n in ast.walk(node.test)):
            selected.append(node)
    clock = SimpleNamespace(now=100.0)
    scope = {'math': math, 'np': np, 'json': json, 'dataclass': dataclass,
             'os': SimpleNamespace(environ=env or {}),
             'time': SimpleNamespace(time=lambda: clock.now,
                                     monotonic=lambda: clock.now, sleep=lambda _: None)}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(SOURCE), 'exec'), scope)
    scope['clock'] = clock
    scope['tree'] = tree
    return scope


class PacketSocket:
    def __init__(self):
        self.messages = []

    def sendall(self, data):
        self.messages.append(json.loads(data.decode('utf-8')))


class BlindFinishTests(unittest.TestCase):
    def setUp(self):
        self.rt = runtime({'SUGARBOX_BLIND_STOP_X': '0.26'})
        self.blind = self.rt['BlindFinishController']()

    def target(self, x=0.50, y=0.0):
        t = self.rt['TargetState']()
        measurement = {'x': x, 'y': y, 'u': 320, 'v': 430,
                       'det': {'conf': .9, 'class_name': 'sugarbox'}}
        for stamp in (9.0, 9.2, 9.4):
            t.consider_measurement(measurement, stamp)
        self.assertTrue(t.has_memory())
        return t

    def test_unset_stop_keeps_blind_finish_disabled(self):
        rt = runtime()
        self.assertIsNone(rt['BLIND_STOP_X'])
        self.assertFalse(rt['BlindFinishController']().eligible(self.target(), 10.1))

    def test_stop_configuration_rejects_nonfinite_and_out_of_range(self):
        for value in ('nan', 'inf', '0.049', '0.601'):
            with self.subTest(value=value), self.assertRaises(SystemExit):
                runtime({'SUGARBOX_BLIND_STOP_X': value})

    def test_yolo_size_configuration_keeps_default_and_accepts_1280(self):
        self.assertEqual(runtime()['YOLO_IMGSZ'], 640)
        self.assertEqual(runtime({'SUGARBOX_YOLO_IMGSZ': '1280'})['YOLO_IMGSZ'], 1280)
        for size in ('319', '641', '1952'):
            with self.subTest(size=size), self.assertRaises(SystemExit):
                runtime({'SUGARBOX_YOLO_IMGSZ': size})

    def test_only_confirmed_lost_near_forward_target_can_start(self):
        self.assertTrue(self.blind.eligible(self.target(), 10.1))
        for t, now in ((self.rt['TargetState'](), 10.1), (self.target(), 9.5),
                       (self.target(.61), 10.1), (self.target(.50, .10), 10.1)):
            with self.subTest(x=t.x, y=t.y, now=now):
                self.assertFalse(self.blind.eligible(t, now))

    def test_nonfinite_memory_cannot_start(self):
        for field in ('x', 'y', 'dist', 'bearing'):
            t = self.target()
            setattr(t, field, float('nan'))
            with self.subTest(field=field):
                self.assertFalse(self.blind.eligible(t, 10.1))

    def test_required_travel_over_limit_is_rejected_before_start(self):
        rt = runtime({'SUGARBOX_BLIND_STOP_X': '0.05'})
        self.assertFalse(rt['BlindFinishController']().eligible(self.target(.60), 10.1))

    def test_start_enforces_eligibility_and_is_once_per_approach(self):
        self.assertFalse(self.blind.start(self.target(), 9.5))
        self.assertTrue(self.blind.start(self.target(), 10.1))
        self.blind.reset()
        self.assertFalse(self.blind.start(self.target(), 11.0))
        self.blind.rearm()
        self.assertTrue(self.blind.start(self.target(), 11.0))

    def test_memory_and_distance_use_measured_motion_to_arrive(self):
        target = self.target()
        self.blind.start(target, 10.1)
        target.last_motion_update = 10.1
        arrived = False
        for i in range(1, 33):
            now = 10.1 + i * .1
            target.propagate_with_odom(.08, 0, now)
            arrived, vx, reason = self.blind.command(target, .08, now)
            if arrived:
                break
            self.assertEqual(vx, .08)
        self.assertTrue(arrived)
        self.assertLessEqual(target.x, .26 + 1e-9)
        self.assertEqual(vx, 0)
        self.assertFalse(self.blind.active)

    def test_stalled_motor_times_out_without_claiming_arrival(self):
        target = self.target()
        self.blind.start(target, 10.1)
        for now in (10.2, 10.5, 11.0, 12.0, 19.0):
            arrived, vx, reason = self.blind.command(target, 0, now)
        self.assertFalse(arrived)
        self.assertEqual(vx, 0)
        self.assertIn('TIMEOUT', reason)
        self.assertEqual(self.blind.travelled_m, 0)

    def test_reverse_and_long_gap_do_not_fabricate_forward_travel(self):
        target = self.target()
        self.blind.start(target, 10.1)
        self.blind.command(target, -.08, 10.2)
        self.blind.command(target, .08, 11.0)
        self.assertEqual(self.blind.travelled_m, 0)

    def test_distance_budget_does_not_mean_target_has_arrived(self):
        # Odometry ran past the plan (plus its slack) while the remembered box is
        # still ahead -- vision moved it back. That is not an arrival.
        target = self.target()
        self.blind.start(target, 10.1)
        self.blind.travelled_m = self.blind.budget_m + self.rt['BLIND_BUDGET_SLACK_M']
        arrived, vx, reason = self.blind.command(target, 0, 10.2)
        self.assertFalse(arrived)
        self.assertEqual(vx, 0)
        self.assertIn('BUDGET', reason)
        self.assertFalse(self.blind.active)

    def test_inactive_or_invalid_feedback_never_commands_forward(self):
        target = self.target()
        self.assertEqual(self.blind.command(target, .08, 10.1)[1], 0)
        self.blind.start(target, 10.1)
        arrived, vx, _ = self.blind.command(target, float('nan'), 10.2)
        self.assertFalse(arrived)
        self.assertEqual(vx, 0)

    def test_blind_creep_keeps_original_front_lidar_stop(self):
        for robust, minimum in ((.29, .29), (.50, .17)):
            with self.subTest(robust=robust, minimum=minimum):
                safety = self.rt['AntiJitterSafetyFilter']()
                vx, wz, reason = safety.apply_final_approach(.08, robust, minimum)
                self.assertEqual((vx, wz), (0, 0))
                self.assertTrue('BLOCK' in reason or 'STOP' in reason)

    def test_main_freshness_gates_prevent_blind_start(self):
        main = next(n for n in self.rt['tree'].body
                    if isinstance(n, ast.FunctionDef) and n.name == 'main')
        gate = next(n for n in ast.walk(main) if isinstance(n, ast.If)
                    and isinstance(n.test, ast.Name) and n.test.id == 'e_stop_latched')
        start = next(n for n in ast.walk(main) if isinstance(n, ast.If)
                     and any(isinstance(k, ast.Call) and isinstance(k.func, ast.Attribute)
                             and isinstance(k.func.value, ast.Name)
                             and k.func.value.id == 'blind_finish' and k.func.attr == 'eligible'
                             for k in ast.walk(n.test)))
        code = compile(ast.Module(body=[gate, start], type_ignores=[]), str(SOURCE), 'exec')
        # The gate reads the stale timeouts and other plain module constants.
        constants = {}
        for node in self.rt['tree'].body:
            if (isinstance(node, ast.Assign) and len(node.targets) == 1
                    and isinstance(node.targets[0], ast.Name) and node.targets[0].id.isupper()):
                try:
                    constants[node.targets[0].id] = ast.literal_eval(node.value)
                except ValueError:
                    pass
        cases = ({}, {'camera_age': 10}, {'scan_age': 10}, {'odom_age': 10},
                 {'e_stop_latched': True}, {'arrived_latched': True})
        for changed in cases:
            scope = dict(constants)
            scope.update(self.rt)
            scope.update(now=10.1, target=self.target(), should_stop=False,
                         camera_age=.01, scan_age=.01, odom_age=.01,
                         e_stop_latched=False, arrived_latched=False,
                         blind_finish=self.rt['BlindFinishController'](),
                         final_approach=SimpleNamespace(active=False),
                         center_controller=SimpleNamespace(reset=lambda: None),
                         anti_jitter=self.rt['AntiJitterSafetyFilter'](), last_vision_reason='VALID')
            scope.update(changed)
            with self.subTest(changed=changed):
                exec(code, scope)
                self.assertEqual(scope['blind_finish'].active, not bool(changed))


class FinalApproachOcclusionTests(unittest.TestCase):
    """2026-10-04: the box slid behind the gripper 11 cm into the final 15 cm; the
    search branch reset FINAL_FORWARD and arrival never came."""

    def searching_assignment(self):
        tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        return next(n for n in ast.walk(main) if isinstance(n, ast.Assign)
                    and any(isinstance(t, ast.Name) and t.id == 'target_searching'
                            for t in n.targets))

    def negated(self, node):
        return {f'{n.operand.value.id}.{n.operand.attr}' for n in ast.walk(node)
                if isinstance(n, ast.UnaryOp) and isinstance(n.op, ast.Not)
                and isinstance(n.operand, ast.Attribute) and isinstance(n.operand.value, ast.Name)}

    def test_search_cannot_take_over_a_running_final_approach(self):
        self.assertIn('final_approach.active', self.negated(self.searching_assignment().value))

    def test_search_cannot_take_over_a_running_blind_finish(self):
        self.assertIn('blind_finish.active', self.negated(self.searching_assignment().value))


class MotorTurnTests(unittest.TestCase):
    def setUp(self):
        self.rt = runtime()
        self.client = self.rt['MotorClient']('unused', 0)
        self.client.sock = PacketSocket()

    def send(self, vx, wz, force=True):
        self.client.send_velocity(vx, wz, force=force)
        return self.client.sock.messages[-1]

    def test_pure_turn_floor_preserves_both_directions(self):
        for wz in (.05, .40, .78, -.05, -.40, -.78):
            with self.subTest(wz=wz):
                packet = self.send(0, wz)
                self.assertEqual(packet['vx'], 0)
                self.assertEqual(packet['wz'], math.copysign(1.0, wz))

    def test_stops_and_moving_turns_are_unchanged(self):
        for vx, wz in ((0, 0), (.08, .4), (-.06, -.4), (0, 1.2), (0, -1.2)):
            with self.subTest(vx=vx, wz=wz):
                packet = self.send(vx, wz)
                self.assertEqual((packet['vx'], packet['wz']), (vx, wz))

    def test_stop_bypasses_rate_limit_and_cannot_turn(self):
        self.send(0, .4)
        count = len(self.client.sock.messages)
        self.client.send_velocity(0, -.4)
        self.assertEqual(len(self.client.sock.messages), count)
        self.client.stop(repeat=3)
        self.assertEqual(self.client.sock.messages[-3:],
                         [{'action': 'velocity', 'vx': 0, 'wz': 0}] * 3)

    def test_lidar_blocked_turn_stays_zero_in_motor_packet(self):
        for direction, side in ((.4, 'left'), (-.4, 'right')):
            with self.subTest(side=side):
                safety = self.rt['AntiJitterSafetyFilter']()
                kwargs = {side + '_robust': .19}
                vx, wz, reason, _ = safety.apply(0, direction, 1, 1, now=1, **kwargs)
                self.assertIn('SIDE_GUARD', reason)
                packet = self.send(vx, wz)
                self.assertEqual((packet['vx'], packet['wz']), (0, 0))


if __name__ == '__main__':
    unittest.main()
