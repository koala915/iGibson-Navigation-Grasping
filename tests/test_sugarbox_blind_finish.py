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
    classes = {'BlindFinishController', 'TargetState', 'MotorClient', 'AntiJitterSafetyFilter',
               'ProgressWatchdog', 'ObsLogger', 'FinalApproachController',
               'PursuitController'}
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
                 {'e_stop_latched': True}, {'arrived_latched': True}, {'blocked_latched': True})
        for changed in cases:
            scope = dict(constants)
            scope.update(self.rt)
            scope.update(now=10.1, target=self.target(), should_stop=False,
                         camera_age=.01, scan_age=.01, odom_age=.01,
                         e_stop_latched=False, arrived_latched=False,
                         blocked_latched=False, blocked_reason='NO_PROGRESS test',
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
        # Reverse is refused since 2026-10-06 (see ReverseOffTests).
        for vx, wz in ((0, 0), (.08, .4), (.06, -.4), (0, 1.2), (0, -1.2)):
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


class TurnSafetyTests(unittest.TestCase):
    """2026-10-05: centering and search turned in place with only the front hard
    stop; the side guard existed on the avoidance path alone."""

    def setUp(self):
        self.rt = runtime()
        self.safety = self.rt['AntiJitterSafetyFilter']()
        self.client = self.rt['MotorClient']('unused', 0)
        self.client.sock = PacketSocket()
        self.main = next(n for n in self.rt['tree'].body
                         if isinstance(n, ast.FunctionDef) and n.name == 'main')

    def turn(self, wz, right=1.0, left=1.0, front=1.0, front_min=1.0):
        return self.safety.apply_center_turn(wz, front, front_min,
                                             right_robust=right, left_robust=left)

    def test_turn_toward_a_near_side_stops_the_wheels(self):
        for wz, side in ((.6, 'left'), (-.6, 'right')):
            with self.subTest(side=side):
                self.setUp()
                vx, out, reason = self.turn(wz, **{side: .19})
                self.assertEqual((vx, out), (0, 0))
                self.assertIn('SIDE_GUARD', reason)
                self.client.send_velocity(vx, out, force=True)
                self.assertEqual(self.client.sock.messages[-1],
                                 {'action': 'velocity', 'vx': 0, 'wz': 0})

    def test_turn_away_from_a_near_side_is_allowed(self):
        self.assertEqual(self.turn(.6, right=.19)[:2], (0, .6))
        self.assertEqual(self.turn(-.6, left=.19)[:2], (0, -.6))

    def test_side_guard_releases_only_past_its_exit_distance(self):
        exit_m = self.rt['SIDE_TURN_GUARD_EXIT_M']
        self.assertEqual(self.turn(-.6, right=.19)[1], 0)
        self.assertEqual(self.turn(-.6, right=exit_m - .01)[1], 0)
        self.assertEqual(self.turn(-.6, right=exit_m + .01)[1], -.6)

    def test_front_hard_stop_still_wins(self):
        vx, wz, reason = self.turn(.6, front=.5, front_min=.17)
        self.assertEqual((vx, wz), (0, 0))
        self.assertTrue(reason.startswith('HARD_STOP'))

    def test_side_readings_cannot_be_left_out(self):
        with self.assertRaises(TypeError):
            self.safety.apply_center_turn(.6, 1.0, 1.0)

    def test_both_main_loop_turns_pass_side_readings(self):
        calls = [n for n in ast.walk(self.main) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Attribute) and n.func.attr == 'apply_center_turn']
        self.assertEqual(len(calls), 2)          # target search and centering
        for call in calls:
            self.assertEqual({k.arg: k.value.id for k in call.keywords},
                             {'right_robust': 'right_side_robust',
                              'left_robust': 'left_side_robust'})

    def test_every_moving_wheel_command_goes_through_a_safety_filter(self):
        parents = {child: node for node in ast.walk(self.main)
                   for child in ast.iter_child_nodes(node)}
        filters = {'apply', 'apply_center_turn', 'apply_final_approach', 'apply_contract'}
        sends = [n for n in ast.walk(self.main) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Attribute) and n.func.attr == 'send_velocity']
        checked = 0
        for call in sends:
            if all(isinstance(a, ast.Constant) and a.value == 0 for a in call.args):
                continue                          # an explicit stop
            node = call
            while not (isinstance(node, ast.If) and 'RUN_MODE' in ast.unparse(node.test)):
                node = parents[node]
            branch = parents[node]
            block = branch.body if node in branch.body else branch.orelse
            used = {n.func.attr for stmt in block for n in ast.walk(stmt)
                    if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                    and isinstance(n.func.value, ast.Name) and n.func.value.id == 'anti_jitter'}
            with self.subTest(line=call.lineno):
                self.assertTrue(used & filters, 'wheel command without a safety filter')
            checked += 1
        self.assertEqual(checked, 6)   # blind, search, centering, steer-in, final, avoidance

    def test_stale_scan_or_odometry_stops_every_branch(self):
        gate = next(n for n in ast.walk(self.main) if isinstance(n, ast.If)
                    and isinstance(n.test, ast.Name) and n.test.id == 'e_stop_latched')
        code = compile(ast.Module(body=[gate], type_ignores=[]), str(SOURCE), 'exec')
        target = SimpleNamespace(has_memory=lambda: True)
        for age, expected in (('scan_age', 'SCAN_STALE'), ('odom_age', 'ODOM_STALE')):
            scope = dict(self.rt)
            scope.update(e_stop_latched=False, arrived_latched=False, blocked_latched=False,
                         blocked_reason='', target=target,
                         camera_age=.01, scan_age=.01, odom_age=.01, should_stop=False,
                         last_vision_reason='VALID', CAMERA_STALE_TIMEOUT=.5,
                         SCAN_STALE_TIMEOUT=.3, ODOM_STALE_TIMEOUT=.5)
            scope[age] = 10
            with self.subTest(age=age):
                exec(code, scope)
                self.assertTrue(scope['should_stop'])
                self.assertTrue(scope['safety_reason'].startswith(expected))
        stop = next(n for n in ast.walk(self.main) if isinstance(n, ast.If)
                    and isinstance(n.test, ast.Name) and n.test.id == 'should_stop'
                    and 'send_velocity' in ast.unparse(n.body[-1]))
        sent = next(n for n in ast.walk(stop.body[-1]) if isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Attribute) and n.func.attr == 'send_velocity')
        self.assertEqual([a.value for a in sent.args], [0.0, 0.0])


class ReverseOffTests(unittest.TestCase):
    """2026-10-06: nothing watches behind the car, so no path may reverse it."""

    def setUp(self):
        self.rt = runtime()
        self.safety = self.rt['AntiJitterSafetyFilter']()
        self.client = self.rt['MotorClient']('unused', 0)
        self.client.sock = PacketSocket()

    def send(self, vx, wz):
        self.client.send_velocity(vx, wz, force=True)
        packet = self.client.sock.messages[-1]
        return packet['vx'], packet['wz']

    def test_reverse_is_off_in_code_and_no_environment_turns_it_on(self):
        self.assertIs(self.rt['ALLOW_REVERSE'], False)
        self.assertIs(runtime({'SUGARBOX_ALLOW_REVERSE': '1'})['ALLOW_REVERSE'], False)

    def test_motor_layer_turns_any_reverse_into_a_stop(self):
        for vx, wz in ((-.08, 0), (-.05, .6), (-1e-4, -.4), (float('nan'), .3), (.05, float('inf'))):
            with self.subTest(vx=vx, wz=wz):
                self.assertEqual(self.send(vx, wz), (0, 0))
        self.assertEqual(self.client.refused, 5)

    def test_motor_layer_still_passes_forward_and_turns(self):
        self.assertEqual(self.send(.08, 0), (.08, 0))
        self.assertEqual(self.send(0, .4), (0, 1.0))     # the pure-turn floor is unchanged
        self.assertEqual(self.client.refused, 0)

    def test_policy_reverse_keeps_a_real_turn(self):
        vx, wz, reason, _ = self.safety.apply(-.05, .6, 1, 1, now=1)
        self.assertEqual((vx, wz), (0, .6))
        self.assertIn('REVERSE_BLOCKED', reason)

    def test_policy_reverse_without_a_real_turn_holds_still(self):
        vx, wz, reason, _ = self.safety.apply(-.05, .05, 1, 1, now=1)
        self.assertEqual((vx, wz), (0, 0))
        self.assertIn('REVERSE_BLOCKED', reason)

    def test_policy_reverse_turn_still_obeys_the_side_guard(self):
        vx, wz, reason, _ = self.safety.apply(-.05, -.6, 1, 1, right_robust=.19, now=1)
        self.assertEqual((vx, wz), (0, 0))
        self.assertIn('SIDE_GUARD_R', reason)

    def test_hard_stop_holds_and_never_backs_up(self):
        for step in range(31):                            # three seconds, ten times the old wait
            vx, wz, reason, _ = self.safety.apply(.1, .4, .5, .17, odom_vx=0, now=1 + step / 10)
            with self.subTest(t=step / 10):
                self.assertEqual((vx, wz), (0, 0))
                self.assertIn('reverse=off', reason)
        self.assertFalse(self.safety.recovery_active)

    def test_forward_block_keeps_a_safe_turn(self):
        vx, wz, reason, _ = self.safety.apply(.1, .6, .29, .29, now=1)
        self.assertEqual((vx, wz), (0, .6))
        self.assertIn('FWD_BLOCK', reason)


class NoProgressTests(unittest.TestCase):
    def setUp(self):
        self.rt = runtime()
        self.window = self.rt['NO_PROGRESS_WINDOW_S']
        self.dog = self.rt['ProgressWatchdog']()
        tree = self.rt['tree']
        self.main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')

    def test_stuck_target_gives_up_after_the_window(self):
        self.assertFalse(self.dog.update(.60, 0))
        self.assertFalse(self.dog.update(.60, self.window - .1))
        self.assertTrue(self.dog.update(.60, self.window))

    def test_real_progress_restarts_the_window(self):
        self.dog.update(.60, 0)
        self.assertFalse(self.dog.update(.56, 8))          # 4 cm closer
        self.assertFalse(self.dog.update(.56, 8 + self.window - .1))
        self.assertTrue(self.dog.update(.56, 8 + self.window))

    def test_detection_noise_cannot_keep_a_stuck_approach_alive(self):
        for second, dist in enumerate((.60, .58, .61, .585, .595, .58, .60, .59, .58, .60)):
            self.assertFalse(self.dog.update(dist, second))
        self.assertTrue(self.dog.update(.59, self.window))

    def test_unreadable_distance_is_never_progress(self):
        self.dog.update(.60, 0)
        self.assertTrue(self.dog.update(float('nan'), self.window))

    def test_reset_starts_over(self):
        self.dog.update(.60, 0)
        self.dog.reset()
        self.assertFalse(self.dog.update(.60, self.window))

    def watchdog_block(self):
        return next(n for n in ast.walk(self.main) if isinstance(n, ast.If)
                    and ast.unparse(n.test).startswith("RUN_MODE == 'DRIVE' and target.has_memory()"))

    def run_block(self, **changed):
        calls = []
        scope = dict(RUN_MODE='DRIVE', e_stop_latched=False, arrived_latched=False,
                     blocked_latched=False, blocked_reason='', should_stop=False,
                     safety_reason='', last_safety_reason='FWD_BLOCK_HOLD front=0.290m',
                     now=50.0, NO_PROGRESS_WINDOW_S=self.window, NO_PROGRESS_MIN_GAIN_M=.03,
                     target=SimpleNamespace(has_memory=lambda: True, dist=.6),
                     progress=SimpleNamespace(update=lambda d, t: calls.append('update') or True,
                                              reset=lambda: calls.append('reset')))
        scope.update(changed)
        exec(compile(ast.Module(body=[self.watchdog_block()], type_ignores=[]),
                     str(SOURCE), 'exec'), scope)
        return scope, calls

    def test_main_loop_latches_stops_and_reports_what_held_it(self):
        scope, _ = self.run_block()
        self.assertTrue(scope['blocked_latched'] and scope['should_stop'])
        self.assertTrue(scope['blocked_reason'].startswith('NO_PROGRESS'))
        self.assertIn('FWD_BLOCK_HOLD', scope['blocked_reason'])

    def test_main_loop_watchdog_only_runs_while_driving_a_locked_target(self):
        for changed in ({'RUN_MODE': 'DRY_RUN'}, {'e_stop_latched': True}, {'arrived_latched': True}):
            with self.subTest(changed=changed):
                scope, calls = self.run_block(**changed)
                self.assertFalse(scope['blocked_latched'])
                self.assertNotIn('update', calls)
        scope, calls = self.run_block(target=SimpleNamespace(has_memory=lambda: False, dist=.6))
        self.assertEqual(calls, ['reset'])

    def test_a_blocked_run_exits_with_its_own_code(self):
        exits = [n for n in ast.walk(self.main) if isinstance(n, ast.If)
                 and ast.unparse(n.test) == 'EXIT_ON_ARRIVAL and (blocked_latched or arrived_latched)']
        self.assertEqual(len(exits), 1)
        self.assertIn('motor.stop', ast.unparse(exits[0].body))
        self.assertIsInstance(exits[0].body[-1], ast.Break)
        final = next(n for n in self.main.body if isinstance(n, ast.If)
                     and ast.unparse(n.test) == 'EXIT_ON_ARRIVAL')
        returns = [n.value.value for n in ast.walk(final.body[0]) if isinstance(n, ast.Return)]
        self.assertEqual(returns, [4])


class ContractModeTests(unittest.TestCase):
    """2026-10-12: enoch's training-matched approach is the primary approach."""

    def setUp(self):
        self.rt = runtime({'SUGARBOX_TRAINING_CONTRACT': '1'})
        self.safety = self.rt['AntiJitterSafetyFilter']()
        self.client = self.rt['MotorClient']('unused', 0)
        self.client.sock = PacketSocket()
        tree = self.rt['tree']
        self.main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        self.vision = next(n for n in tree.body
                           if isinstance(n, ast.FunctionDef) and n.name == 'vision_to_target')

    def safe(self, vx, wz, front=1.0, front_min=1.0, right=1.0, left=1.0):
        return self.safety.apply_contract(vx, wz, front, front_min,
                                          right_robust=right, left_robust=left)

    def send(self, vx, wz):
        self.client.send_velocity(vx, wz, force=True)
        packet = self.client.sock.messages[-1]
        return packet['vx'], packet['wz']

    # -- safety essentials ------------------------------------------------
    def test_hard_stop_stops_everything_including_reverse(self):
        for vx in (.1, -.05):
            self.setUp()
            self.assertEqual(self.safe(vx, .4, .5, .17)[:2], (0, 0))

    def test_front_block_keeps_the_turn_but_not_the_push(self):
        vx, wz, reason = self.safe(.1, .5, .29, .29)
        self.assertEqual((vx, wz), (0, .5))
        self.assertIn('FWD_BLOCK', reason)

    def test_side_guard_refuses_the_turn_and_never_pushes_instead(self):
        vx, wz, reason = self.safe(.1, -.5, right=.19)
        self.assertEqual((vx, wz), (0, 0))
        self.assertIn('SIDE_GUARD_R', reason)

    def test_reverse_is_allowed_but_bounded(self):
        vx, wz, reason = self.safe(-.15, .3)
        self.assertEqual((vx, wz), (-.08, .3))
        self.assertIn('REVERSE', reason)

    def test_no_slowdown_of_its_own(self):
        self.assertEqual(self.safe(.12, 0, .45, .45)[0], .12)   # training shaping already slows

    # -- motor layer --------------------------------------------------------
    def test_motor_passes_slow_reverse_and_refuses_faster(self):
        self.assertEqual(self.send(-.05, .3), (-.05, .3))
        self.assertEqual(self.send(-.12, 0), (0, 0))
        self.assertEqual(self.client.refused, 1)

    def test_motor_adds_no_turn_floor_in_this_mode(self):
        self.assertEqual(self.send(0, .3), (0, .3))

    def test_other_modes_still_refuse_any_reverse(self):
        legacy = runtime()['MotorClient']('unused', 0)
        legacy.sock = PacketSocket()
        legacy.send_velocity(-.05, .3, force=True)
        self.assertEqual(legacy.sock.messages[-1], {'action': 'velocity', 'vx': 0, 'wz': 0})

    # -- target ------------------------------------------------------------
    def test_first_sighting_locks_and_later_ones_are_ignored(self):
        t = self.rt['TargetState'](freeze_visual_updates=True)
        m = lambda x: {'x': x, 'y': 0.0, 'u': 320, 'v': 400, 'det': {'conf': .9, 'class_name': 'sugarbox'}}
        self.assertEqual(t.consider_measurement(m(1.2), 1.0)[1], 'TARGET_LOCKED')
        self.assertEqual(t.consider_measurement(m(0.7), 1.2)[1], 'TARGET_INITIAL_ONLY_ODOM')
        self.assertEqual(t.x, 1.2)
        t.clear()
        self.assertEqual(t.consider_measurement(m(0.9), 2.0)[1], 'TARGET_LOCKED')

    def test_vision_moves_the_target_to_the_centre_before_distance(self):
        body = self.vision.body
        convert = next(i for i, n in enumerate(body) if isinstance(n, ast.If)
                       and ast.unparse(n.test) == 'TRAINING_CONTRACT'
                       and 'center_relative_goal' in ast.unparse(n.body))
        distance = next(i for i, n in enumerate(body) if isinstance(n, ast.Assign)
                        and ast.unparse(n.targets[0]) == 'distance')
        self.assertLess(convert, distance)

    # -- main loop ---------------------------------------------------------
    def test_legacy_steps_are_off(self):
        tests = [ast.unparse(n.test) for n in ast.walk(self.main) if isinstance(n, ast.If)]
        assigns = [ast.unparse(n.value) for n in ast.walk(self.main) if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == 'target_searching' for t in n.targets)]
        self.assertTrue(any(t.startswith('not TRAINING_CONTRACT') and 'blind_finish.eligible' in t
                            for t in tests), 'blind finish')
        self.assertTrue(any(t.startswith('not TRAINING_CONTRACT') and 'not target_searching' in t
                            for t in tests), 'centring')
        self.assertTrue(assigns and assigns[0].startswith('not TRAINING_CONTRACT'), 'search')

    def test_policy_branch_uses_the_contract_filter(self):
        branch = next(n for n in ast.walk(self.main) if isinstance(n, ast.If)
                      and ast.unparse(n.test) == 'TRAINING_CONTRACT'
                      and 'apply_contract' in ast.unparse(n.body))
        self.assertIn('anti_jitter.apply(', ast.unparse(branch.orelse))

    def constant(self, name):
        return next(ast.literal_eval(n.value) for n in self.rt['tree'].body
                    if isinstance(n, ast.Assign) and ast.unparse(n.targets[0]) == name)

    def handoff(self, contract=True, dist=0.99, stop=False, active=False):
        block = next(n for n in ast.walk(self.main) if isinstance(n, ast.If)
                     and 'CONTRACT_HANDOFF_CENTER_M' in ast.unparse(n.test))
        started = []
        pursuit = SimpleNamespace(active=active, start=lambda now: started.append(now))
        scope = dict(TRAINING_CONTRACT=contract, should_stop=stop, arrived_latched=False,
                     safety_reason='', CONTRACT_HANDOFF_CENTER_M=1.00, pursuit=pursuit, now=5.0,
                     contract_action=SimpleNamespace(reset=lambda: None),
                     anti_jitter=SimpleNamespace(reset_motion=lambda: None), math=math,
                     target=SimpleNamespace(has_memory=lambda: True, dist=dist, bearing=.05, y=.04))
        exec(compile(ast.Module(body=[block], type_ignores=[]), str(SOURCE), 'exec'), scope)
        scope['started'] = started
        return scope

    def test_policy_hands_over_to_the_steer_in_at_1m(self):
        scope = self.handoff()
        self.assertEqual(scope['started'], [5.0])
        self.assertFalse(scope['arrived_latched'] or scope['should_stop'])
        self.assertTrue(scope['safety_reason'].startswith('HANDOFF'))
        self.assertEqual(self.handoff(active=True)['started'], [])        # not twice
        self.assertEqual(self.handoff(dist=1.01)['started'], [])
        self.assertEqual(self.handoff(contract=False)['started'], [])
        self.assertEqual(self.handoff(stop=True)['started'], [])          # never over a stop

    def test_after_arrival_it_never_returns_to_the_policy(self):
        block = next(n for n in ast.walk(self.main) if isinstance(n, ast.If)
                     and ast.unparse(n.test) == 'TRAINING_CONTRACT and arrived_latched')
        scope = dict(TRAINING_CONTRACT=True, arrived_latched=True, should_stop=False)
        exec(compile(ast.Module(body=[block], type_ignores=[]), str(SOURCE), 'exec'), scope)
        self.assertTrue(scope['should_stop'])

    def steer_in(self, box_x, box_y, moving=True, seconds=20.0):
        """Drive a unicycle on the controller's commands; box fixed on the floor."""
        pursuit = self.rt['PursuitController']()
        pursuit.start(0.0)
        px = py = th = 0.0
        t, dt, commands = 0.0, 0.1, []
        target = SimpleNamespace(x=box_x, y=box_y)
        while t < seconds:
            dx, dy = box_x - px, box_y - py
            target.x = math.cos(th) * dx + math.sin(th) * dy
            target.y = -math.sin(th) * dx + math.cos(th) * dy
            vx, wz, done, reason = pursuit.command(target, t)
            if done:
                return target, reason, commands
            commands.append((vx, wz))
            if moving:
                px += vx * math.cos(th) * dt
                py += vx * math.sin(th) * dt
                th += wz * dt
            t += dt
        self.fail('never finished')

    def test_steer_in_puts_an_offset_box_on_the_e1_line(self):
        # 2026-10-12: the policy kept an 8 cm offset all the way in. The E1 grasp
        # area at x 0.264 m spans y -0.049..+0.017 m; aim at its middle.
        for box_y in (+0.08, -0.10, 0.0):
            with self.subTest(box_y=box_y):
                target, reason, commands = self.steer_in(1.00, box_y)
                self.assertTrue(reason.startswith('PURSUIT_DONE'), reason)
                self.assertLessEqual(target.x, 0.28)
                self.assertGreater(target.x, 0.26)
                # E1's middle (-0.016 m) seen from the rear camera, 2.8 cm apart.
                self.assertAlmostEqual(target.y, -0.044, delta=0.01)
                self.assertTrue(all(vx > 0 and abs(wz) <= 0.60 for vx, wz in commands))
                # 2026-10-12: smaller turns did not move the chassis at this speed.
                self.assertTrue(all(wz == 0 or abs(wz) >= 0.30 for _, wz in commands))

    def test_steer_in_gives_up_to_e1_when_it_cannot_close_in(self):
        target, reason, _ = self.steer_in(1.00, 0.05, moving=False)
        self.assertTrue(reason.startswith('PURSUIT_TIMEOUT'), reason)

    def test_steer_in_branch_keeps_the_safety_essentials_and_latches_arrival(self):
        branch = next(n for n in ast.walk(self.main) if isinstance(n, ast.If)
                      and ast.unparse(n.test) == 'pursuit.active')
        code = ast.unparse(branch.body)
        self.assertIn('anti_jitter.apply_contract(', code)
        self.assertIn('arrived_latched = True', code)

    def test_final_forward_stops_after_its_own_distance(self):
        final = self.rt['FinalApproachController']()
        final.start(None, 0.0, travel_m=0.10)
        t, done = 0.0, None
        while t < 3.0 and done is None:
            t += 0.1
            _, vx, _, reason = final.command(None, 0.08, t)
            if reason.startswith('FINAL_DONE'):
                done = t
        self.assertAlmostEqual(done, 1.3, delta=0.11)                    # 0.10 m at 0.08 m/s

    def test_policy_sees_obstacles_15_percent_further(self):
        self.assertEqual(self.constant('CONTRACT_DISTANCE_SCALE'), 1.15)
        self.assertEqual(self.constant('CONTRACT_FRONT_CORRIDOR_HALF_WIDTH_M'), 0.17)

    def test_driving_this_mode_needs_its_own_opt_in(self):
        checks = [n for n in self.main.body[:4] if isinstance(n, ast.If)]
        code = compile(ast.Module(body=checks, type_ignores=[]), str(SOURCE), 'exec')
        base = dict(RUN_MODE='DRIVE', TRAINING_CONTRACT=True, CONTRACT_ALLOW_DRIVE=False,
                    TARGET_INITIAL_ONLY=False)
        with self.assertRaises(SystemExit):
            exec(code, dict(base))
        exec(code, dict(base, CONTRACT_ALLOW_DRIVE=True))
        exec(code, dict(base, RUN_MODE='DRY_RUN'))
        with self.assertRaises(SystemExit):
            exec(code, dict(base, TRAINING_CONTRACT=False, TARGET_INITIAL_ONLY=True))


class ObsLogTests(unittest.TestCase):
    """2026-10-12: a real run could not be replayed, so why the policy drove
    straight past a box 14 cm to its left stayed a guess."""

    def test_each_policy_tick_is_one_replayable_line(self):
        import tempfile
        rt = runtime()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'obs.jsonl'
            log = rt['ObsLogger'](str(path))
            target = SimpleNamespace(x=1.186, y=0.118)
            log.write(100.0, np.arange(55, dtype=np.float32) / 10, np.array([0.5, -0.25]), target)
            log.write(100.2, np.zeros(55, dtype=np.float32), np.array([1.0, 0.0]), target)
            log.close()
            rows = [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
        self.assertEqual(len(rows), 2)
        self.assertEqual(len(rows[0]['obs']), 55)
        self.assertAlmostEqual(rows[0]['obs'][54], 5.4, places=4)
        self.assertEqual(rows[0]['action'], [0.5, -0.25])
        self.assertEqual(rows[0]['target'], [1.186, 0.118])

    def test_off_by_default_and_writes_nothing(self):
        rt = runtime()
        log = rt['ObsLogger']('')
        log.write(100.0, np.zeros(55), np.zeros(2), SimpleNamespace(x=0.0, y=0.0))
        log.close()
        self.assertIsNone(log.file)


if __name__ == '__main__':
    unittest.main()
