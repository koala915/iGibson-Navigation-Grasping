#!/usr/bin/env python3
"""Drive a whole mission on stubs: patrol -> spot -> approach -> grasp -> bin -> patrol.

The per-module selftests each prove one piece. This proves they compose: the
real MissionFSM and the real MissionRunner._act run against fake hardware for a
complete cycle, and every state the mission is supposed to pass through is
actually visited in order.

It is not a physics simulation and does not pretend to be. Distances shrink
because the stub says so. What it does check is the part that has repeatedly
been wrong -- the bookkeeping between states: which flags are set, when the
tracker survives a reset, when the goal source flips, that the arm is folded
while driving and raised before the camera is used, and that the run ends where
it started rather than wedged in some state with no exit.

Run:
    python3 tests/test_mission_end_to_end.py
"""
from __future__ import annotations

import io
import math
import sys
import time
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

import numpy as np

_X3PLUS = Path(__file__).resolve().parent.parent
if str(_X3PLUS) not in sys.path:
    sys.path.insert(0, str(_X3PLUS))

from integration import map_goal_provider as mgp
from integration import mission_fsm as mfsm
from integration import mission_pipeline as mp
from integration import nav_rl_grasp_pipeline as nrgp
from integration import feedback_odom as fo


# ════════════════════════════════════════════════════════════════════════════
# Fakes
# ════════════════════════════════════════════════════════════════════════════

NAV_HOME = mp.NAV_HOME_DEG
C3 = mp.GRASP_HOME_DEG


class FakeCfg:
    home_deg = NAV_HOME
    grasp_home_deg = C3
    documented_x_range = (0.20, 0.28)
    documented_y_range = (-0.09, 0.08)
    envelope_tol_m = 0.001


class FakeMapper:
    def hw_deg_to_sim_arm(self, deg):
        return np.asarray([math.radians(d - 90.0) for d in deg[:5]], dtype=np.float64)

    def hw_deg_to_sim_grip(self, deg):
        return math.radians(float(deg) - 90.0)


class FakeController:
    """Records the arm poses it is asked for; never fails unless told to."""

    def __init__(self, *, grasp_ok=True, verify_ok=True):
        self.cfg = FakeCfg()
        self.mapper = FakeMapper()
        self._current_grip_rad = 0.0
        self.obj_provider = None
        self.moves = []           # labels, in order
        self.move_details = []
        self.ran_grasp = 0
        self.ran_release = 0
        self.grasp_ok = grasp_ok
        self.verify_ok = verify_ok

    def move_guarded_and_verified(self, arm, grip, *, label, **kw):
        self.moves.append(label)
        self.move_details.append((label, dict(kw)))
        return {"reached": True, "reason": "arrived", "iters": 1, "guard": ["pass"]}

    def move_home(self):
        self.moves.append("move_home")
        return True

    def run(self, max_steps=300):
        self.ran_grasp += 1
        return self.grasp_ok

    def run_release_only(self):
        self.ran_release += 1
        return "released"

    def close(self):
        pass


class FakeDevice:
    """The Rosmaster handle the self-check talks to.

    Implements exactly the two calls run_self_check makes: enabling auto
    reporting (v21's ServoController does not) and the liveness probe that
    proves the board is really streaming, without which get_motion_data()
    returns its initial zeros forever.
    """

    def __init__(self, *, reports=True):
        self.auto_report = None
        self._reports = reports

    def set_auto_report_state(self, enable, forever=False):
        self.auto_report = (enable, forever)

    def get_battery_voltage(self):
        return 12.4 if self._reports else 0.0


class FakeNav:
    """Chassis + cameras. The object closes in as the robot 'drives'."""

    def __init__(self, ncfg, *, start_dist=3.0, align_ok=True, reports=True):
        self.ncfg = ncfg
        self.device = FakeDevice(reports=reports)
        self.dry_run = True
        self.show = False
        # The object's real distance, which is NOT the tracker: the tracker is
        # an estimate that gets cleared on goal-source changes, while the camera
        # keeps seeing whatever is actually there. Conflating them hides exactly
        # the bug this suite exists to catch.
        self.obj_dist = start_dist
        self.tracker = _Tracker(start_dist)
        self.align_ok = align_ok
        self.stops = 0
        self.drives = 0
        self.arm_detects = 0
        self.reset_calls = []
        self.prev_action = np.zeros(2, dtype=np.float32)

    # chassis
    def stop(self):
        self.stops += 1

    def back_off(self):
        pass

    def _advance(self, step):
        self.obj_dist = max(0.2, self.obj_dist - step)
        self.tracker.close_in(step)

    def nav_tick(self, dist, bearing, dt, points):
        self.drives += 1
        self._advance(0.15)
        return mp.NavTick(0.3, 0.0, False, False, 4.0, float("inf"))

    def creep(self, vx, wz, dt):
        self.drives += 1
        self._advance(0.05)

    def reset_nav(self, clear_tracker=True):
        self.reset_calls.append(clear_tracker)
        if clear_tracker:
            self.tracker = _Tracker(float("inf"))

    # cameras -- these read the world, not the tracker
    def _detect_rear(self):
        return (True, self.obj_dist, 0.0)

    def _detect_arm(self):
        self.arm_detects += 1
        return (True, self.obj_dist, 0.0, 40.0, "sugarbox")

    def _arm_align(self, *, deadline=None, safety_check=None):
        if safety_check is not None and safety_check():
            return False
        self.obj_dist = 0.24
        self.tracker.set(0.24)
        return self.align_ok

    def latch_arm_object(self):
        return ([0.24, 0.0, 0.0325], 0.023, 0.065)

    def verify_grasp(self, pos):
        return True

    def open_cameras(self):
        pass

    def release(self):
        pass


class _Tracker:
    def __init__(self, d):
        self._d = d
        self._age = 0.0

    def set(self, d):
        self._d = d

    def update(self, forward_dist, offset_right):
        self._d = forward_dist
        self._age = 0.0

    def close_in(self, step):
        if math.isfinite(self._d):
            self._d = max(0.2, self._d - step)

    @property
    def has_fix(self):
        return math.isfinite(self._d)

    def dist(self):
        return self._d

    def bearing(self):
        return 0.0

    def fix_age(self, now=None):
        return self._age


class FakeLidar:
    def get_points(self):
        return [(a - 90.0, 4.0) for a in range(0, 181, 5)]

    def age(self):
        return 0.0

    def scan_info(self, cfg):
        return {"frame_id": "laser_link", "warnings": [], "angle_min_deg": -85.0,
                "angle_max_deg": 85.0, "forward_fraction": 0.94, "count": 340}

    def close(self):
        pass


class FakeOdom:
    """Always valid, fresh and stationary -- the FSM's health inputs."""
    valid = fresh = stationary = True
    reason = ""
    x = y = yaw = vx = vy = wz = 0.0


class FakeOdomPub:
    ticks = 5

    def state(self):
        return FakeOdom()

    def stop(self):
        pass

    def join(self, timeout=None):
        pass


class FakeRos:
    def __init__(self, goals, can_refresh=True):
        self.goals = goals
        self.can_refresh = can_refresh
        self.nomotion_calls = 0
        self.refreshes = 0
        self.clock = time.time         # tests drive this to a fake now

    def latest_pose(self):
        return self.goals.pose or mgp.MapPose(0.0, 0.0, 0.0, 1e9)

    def pose_quality_ok(self):
        return True, ""

    def request_nomotion_update(self, **kw):
        self.nomotion_calls += 1
        return self.can_refresh

    def refresh_pose(self, timeout_s=3.0, poll_s=0.05):
        self.refreshes += 1
        if self.can_refresh:
            # a real refresh lands as a new /amcl_pose, which restamps the fix
            pose = self.goals.pose
            if pose is not None:
                self.goals.set_pose(mgp.MapPose(pose.x, pose.y, pose.yaw, self.clock()))
        return self.can_refresh

    def rosapi_list(self, what, timeout_s=3.0):
        if what == "services":
            return ["/request_nomotion_update"] if self.can_refresh else []
        return ["/amcl", "/map_server"]

    def close(self):
        pass


class FakeArgs:
    real = False
    show = False
    max_steps = 300
    max_laps = 0
    no_deliver = False
    wait_start = False
    exit_on_pause = True
    detection_streak = 3
    detection_jump_m = 0.35
    blacklist_radius_m = 1.0
    handoff_aim_x = 0.24
    handoff_radius_m = 0.04
    nav_stop_dist = 0.75
    bin_rim_height = 0.0
    release_clearance = 0.03
    amcl_refresh_timeout = 3.0


def build_runner(*, grasp_ok=True, align_ok=True, deliver=True, can_refresh=True):
    try:
        from integration import nav_rl as nr
        ncfg = nr.NavRLConfig()
    except Exception:                                   # pragma: no cover
        ncfg = type("C", (), {"control_period_s": 1 / 6.0,
                              "lidar_stale_timeout_s": 0.7,
                              "safety_brake_dist": 0.25,
                              "motor_delay_steps": 2})()

    route = mgp.RouteSpec(
        waypoints=[mgp.Waypoint(f"p{i}", i * 0.8, 0.0) for i in range(8)],
        loop=True, bin_center=(6.0, 1.0), bin_approach=(6.0, 0.6, 0.0))
    goals = mgp.MapGoalProvider(route, arrival_radius_m=0.25, pose_max_age_s=1e12)
    goals.set_pose(mgp.MapPose(5.0, 5.0, 0.0, 1e9))       # far from every waypoint

    args = FakeArgs()
    args.no_deliver = not deliver
    runner = mp.MissionRunner(
        nav=FakeNav(ncfg, align_ok=align_ok), controller=FakeController(grasp_ok=grasp_ok),
        goals=goals, rio=FakeRos(goals, can_refresh=can_refresh),
        odom_pub=FakeOdomPub(), lidar=FakeLidar(),
        fsm=mfsm.MissionFSM(mfsm.MissionConfig(approach_to_align_m=args.nav_stop_dist),
                            deliver_enabled=deliver),
        args=args)
    runner.started = True
    return runner


def _step_map_pose(runner, t, step=0.4):
    """Move the robot in the MAP frame toward whatever goal it is driving to.

    The nav stub shrinks the camera distance to the object; this is the other
    half of the world -- without it the robot never arrives at a waypoint or at
    the bin, and the mission stalls in DELIVER for reasons that have nothing to
    do with the code under test.
    """
    pose = runner.goals.pose
    gx, gy, _yaw, _label = runner.goals.target()
    dx, dy = gx - pose.x, gy - pose.y
    d = math.hypot(dx, dy)
    if d <= 1e-9:
        return
    k = min(step, d) / d
    runner.goals.set_pose(mgp.MapPose(pose.x + dx * k, pose.y + dy * k,
                                      math.atan2(dy, dx), t))


def drive(runner, max_ticks=600):
    """Step the real FSM + real _act until PATROL is reached twice."""
    seen = []
    t = 1000.0
    laps_at_patrol = 0
    for _ in range(max_ticks):
        t += 0.2
        sense = runner._sense(t)
        tr = runner.fsm.step(sense)
        if not seen or seen[-1] is not tr.state:
            seen.append(tr.state)
            if tr.state is mfsm.State.PATROL:
                laps_at_patrol += 1
                if laps_at_patrol == 2:
                    return seen
        if tr.terminal or tr.completed or tr.state is mfsm.State.PAUSED:
            return seen
        runner._act(tr, t, 0.2)
        if tr.chassis_allowed and tr.state in mfsm.MAP_STATES:
            _step_map_pose(runner, t)
    return seen


# ════════════════════════════════════════════════════════════════════════════
# Tests
# ════════════════════════════════════════════════════════════════════════════

class MissionEndToEnd(unittest.TestCase):

    def test_odom_publisher_joins_and_ages_a_frozen_snapshot(self):
        cfg = fo.FeedbackOdomConfig(feedback_timeout_s=0.3)
        state = fo.OdomState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             100.0, True, True, True, "")
        reader = type("Reader", (), {})()
        reader.odom = type("Odom", (), {
            "cfg": cfg,
            "covariance": lambda self: [0.0] * 36,
        })()
        reader.poll = lambda now=None: state
        rio = type("Rio", (), {
            "publish_odom_and_tf": lambda self, *a, **kw: None,
        })()
        pub = mp.OdomPublisher(reader, rio, rate_hz=100.0)
        pub._state = state
        aged = pub.state(now=101.0)
        self.assertFalse(aged.fresh)
        self.assertFalse(aged.stationary)
        pub.start()
        pub.stop()
        pub.join(timeout=1.0)
        self.assertFalse(pub.is_alive())

    def test_closed_stdin_never_counts_as_operator_start(self):
        runner = build_runner()
        runner.started = False
        with mock.patch("builtins.input", side_effect=EOFError), redirect_stdout(io.StringIO()):
            runner._await_operator()
        self.assertFalse(runner.started)
        self.assertIn("confirmation unavailable", runner.fault)

    def test_real_integrated_mission_is_refused_before_loading_assets(self):
        args = mp.parse_args([
            "--real",
            "--i-confirm-serial-owner",
            "--lidar-orientation-evidence", "verified.json",
            "--i-confirm-arm-cam-pose",
            "--unlock-candidate-real",
        ])
        with self.assertRaisesRegex(SystemExit, "homography"):
            mp.build_and_run(args)

    def test_real_mission_reuses_the_v21_candidate_release_gate(self):
        args = mp.parse_args([
            "--real",
            "--i-confirm-serial-owner",
            "--lidar-orientation-evidence", "verified.json",
            "--i-confirm-arm-cam-pose",
        ])
        with self.assertRaisesRegex(SystemExit, "release gate"):
            mp.build_and_run(args)

    def test_real_mission_never_allows_skipping_model_hashes(self):
        args = mp.parse_args(["--real", "--skip-model-hash"])
        with self.assertRaisesRegex(SystemExit, "never permits"):
            mp.build_and_run(args)

    def test_a_long_arm_block_recovers_the_map_fix_instead_of_pausing(self):
        """A grasp holds the loop for minutes; DELIVER must not pause on that.

        AMCL only publishes when its filter updates, and it updates on motion.
        controller.run() blocks the mission loop for a whole episode, so nothing
        can keep the fix alive while the arm works -- and the state right after
        (DELIVER) treats a stale fix as blocking. Every successful grasp used to
        end in PAUSED, and the operator's Enter re-ran a self-check that failed
        on the same stale pose. The base is stopped there, so the fix is
        recovered on the spot instead.
        """
        runner = build_runner()
        runner.rio.clock = lambda: 200.0
        runner.goals.pose_max_age_s = 1.0
        runner.fsm.state = mfsm.State.DELIVER
        runner.goals.set_pose(mgp.MapPose(5.0, 5.0, 0.0, 1.0))   # 199 s old

        self.assertFalse(runner._sense(200.0).health.amcl_ok)
        self.assertEqual(runner.fsm.step(runner._sense(200.0)).state,
                         mfsm.State.PAUSED)

        runner.fsm.state = mfsm.State.DELIVER
        self.assertTrue(runner._recover_map_fix(140.0))
        self.assertEqual(runner.rio.refreshes, 1)
        self.assertTrue(runner._sense(200.0).health.amcl_ok)
        self.assertEqual(runner.fsm.step(runner._sense(200.0)).state,
                         mfsm.State.DELIVER)

    def test_real_refuses_to_start_when_amcl_cannot_be_refreshed(self):
        """No /request_nomotion_update means no recovery from standing still."""
        runner = build_runner(can_refresh=False)
        runner.args.real = True
        with redirect_stdout(io.StringIO()) as out:
            self.assertFalse(runner._check_amcl_refresh_path())
        self.assertIn("not advertised", out.getvalue())

        runner = build_runner()
        with redirect_stdout(io.StringIO()) as out:
            self.assertTrue(runner._check_amcl_refresh_path())
        self.assertIn("/request_nomotion_update available", out.getvalue())

    def test_stale_map_goal_pauses_patrol_instead_of_only_stopping(self):
        runner = build_runner()
        runner.fsm.state = mfsm.State.PATROL
        runner.goals.pose_max_age_s = 0.1
        runner.goals.set_pose(mgp.MapPose(5.0, 5.0, 0.0, 1.0))

        sense = runner._sense(10.0)
        self.assertFalse(sense.health.amcl_ok)
        transition = runner.fsm.step(sense)
        self.assertEqual(transition.state, mfsm.State.PAUSED)

    def _run(self, **kw):
        runner = build_runner(**kw)
        with redirect_stdout(io.StringIO()):
            states = drive(runner)
        return runner, states

    def test_full_cycle_reaches_every_state_and_returns_to_patrol(self):
        runner, states = self._run()
        S = mfsm.State
        for want in (S.SELF_CHECK, S.IDLE, S.PATROL, S.INVESTIGATE, S.APPROACH,
                     S.ALIGN, S.STATIONARY_GATE, S.LATCH, S.GRASP, S.VERIFY,
                     S.CARRY_HOME, S.DELIVER, S.PLACE_ALIGN, S.PLACE, S.RESUME):
            self.assertIn(want, states, f"{want.value} was never reached: {states}")
        self.assertIs(states[-1], S.PATROL, f"did not return to patrol: {states}")
        self.assertNotIn(S.PAUSED, states)
        self.assertNotIn(S.FAULT, states)

    def test_the_order_is_the_documented_one(self):
        _, states = self._run()
        S = mfsm.State
        order = [s for s in states if s in (S.PATROL, S.INVESTIGATE, S.APPROACH,
                                            S.ALIGN, S.GRASP, S.DELIVER, S.PLACE)]
        self.assertEqual(order, [S.PATROL, S.INVESTIGATE, S.APPROACH, S.ALIGN,
                                 S.GRASP, S.DELIVER, S.PLACE, S.PATROL], states)

    def test_grasp_and_release_each_run_exactly_once(self):
        runner, _ = self._run()
        self.assertEqual(runner.controller.ran_grasp, 1)
        self.assertEqual(runner.controller.ran_release, 1)

    def test_arm_is_folded_for_driving_and_raised_only_at_the_handoff(self):
        runner, _ = self._run()
        moves = runner.controller.moves
        self.assertIn("nav-home", moves, "the self-check never parked the arm")
        self.assertIn("nav-home->C3", moves, "the arm never raised to the grasp pose")
        self.assertLess(moves.index("nav-home"), moves.index("nav-home->C3"),
                        "raised to C3 before it was ever parked")
        # exactly one raise: mid-mission pose changes are how the arm ends up
        # somewhere nobody expects
        self.assertEqual(moves.count("nav-home->C3"), 1, moves)

    def test_arm_camera_is_not_consulted_while_the_arm_is_folded(self):
        # Its extrinsics belong to C3 and the ground model returns a plausible
        # number at any pose, so a consult here is silently wrong.
        runner, _ = self._run()
        self.assertEqual(runner.nav.arm_detects, 0, "arm cam used at the nav pose")

    def test_goal_source_switches_keep_the_camera_fix_but_clear_the_delay(self):
        runner, _ = self._run()
        # PATROL->INVESTIGATE must reset with clear_tracker False, DELIVER True
        self.assertIn(False, runner.nav.reset_calls,
                      "no reset preserved the tracker; approach can never start")
        self.assertIn(True, runner.nav.reset_calls,
                      "no reset cleared the tracker on the way to a map goal")

    def test_delivery_targets_the_bin_not_a_patrol_waypoint(self):
        runner, _ = self._run()
        # after PLACE the override is released; the recorded interrupt proves the
        # bin was actually selected at some point
        self.assertFalse(runner.goals.in_override)
        self.assertGreaterEqual(runner.goals.index, 0)
        self.assertEqual(runner.controller.ran_release, 1)

    def test_failed_alignment_gives_up_after_the_retry_budget(self):
        runner, states = self._run(align_ok=False)
        S = mfsm.State
        self.assertIn(S.RETRY, states, states)
        self.assertIn(S.RESUME, states, states)
        self.assertIs(states[-1], S.PATROL, f"did not resume patrol: {states}")
        self.assertEqual(runner.controller.ran_grasp, 0, "grasped despite bad align")
        self.assertTrue(runner._blacklist, "gave up without blacklisting the spot")

    def test_failed_controller_result_is_a_failed_attempt_not_an_infinite_grasp(self):
        runner, states = self._run(grasp_ok=False)
        S = mfsm.State
        self.assertEqual(runner.controller.ran_grasp,
                         runner.fsm.cfg.max_grasp_attempts)
        self.assertIn(S.RETRY, states, states)
        self.assertIn(S.RESUME, states, states)
        self.assertIs(states[-1], S.PATROL, states)
        self.assertTrue(runner._blacklist)

    def test_retry_records_nav_home_so_every_attempt_raises_back_to_c3(self):
        runner, _ = self._run(grasp_ok=False)
        self.assertEqual(runner.controller.moves.count("nav-home->C3"),
                         runner.fsm.cfg.max_grasp_attempts,
                         runner.controller.moves)

    def test_base_must_report_stationary_before_fine_align_or_arm_raise(self):
        fsm = mfsm.MissionFSM(mfsm.MissionConfig())
        fsm.state = mfsm.State.APPROACH
        fsm.entered_at = 10.0
        tr = fsm.step(mfsm.Sense(now=10.1, target_dist=0.5,
                                  target_fix_age=0.0, stationary=False))
        self.assertIs(tr.state, mfsm.State.ALIGN)
        self.assertIs(tr.action, mfsm.Action.SETTLE)
        tr = fsm.step(mfsm.Sense(now=10.2, target_dist=0.5,
                                  target_fix_age=0.0, stationary=False))
        self.assertIs(tr.action, mfsm.Action.SETTLE)
        tr = fsm.step(mfsm.Sense(now=10.3, target_dist=0.5,
                                  target_fix_age=0.0, stationary=True))
        self.assertIs(tr.action, mfsm.Action.FINE_ALIGN)

    def test_oversized_detection_never_corrupts_the_map_blacklist(self):
        runner = build_runner()
        runner._at_nav_home = False
        runner.nav.latch_arm_object = lambda: ([0.24, 0.0, 0.03], 999.0, 0.06)
        tr = mfsm.Transition(mfsm.State.ALIGN, mfsm.Action.FINE_ALIGN)
        with redirect_stdout(io.StringIO()):
            runner._act(tr, 100.0, 0.2)
        self.assertTrue(runner._align_failed)
        self.assertEqual(runner._blacklist, [])
        self.assertIsNone(runner._blacklisted_here())

    def test_detection_exception_breaks_the_consecutive_streak(self):
        runner = build_runner()
        runner._det_streak = 2
        runner._last_det = 0.0
        runner.nav._detect_rear = mock.Mock(side_effect=RuntimeError("camera lost"))
        with redirect_stdout(io.StringIO()):
            runner._run_detection(100.0)
        self.assertEqual(runner._det_streak, 0)

    def test_zero_blacklist_radius_really_disables_blacklisting(self):
        runner = build_runner()
        runner.args.blacklist_radius_m = 0.0
        pose = runner.goals.pose
        runner._blacklist.append((pose.x, pose.y))
        self.assertIsNone(runner._blacklisted_here())

    def test_pause_while_carrying_preserves_object_and_resumes_delivery(self):
        runner = build_runner()
        runner._grasp_finished = True
        runner._grasp_controller_confirmed = True
        runner._grasp_verified = True
        runner._latched = ([0.24, 0.0, 0.03], 0.03, 0.06)
        runner._at_nav_home = True
        runner.fsm.state = mfsm.State.DELIVER
        runner.fsm.entered_at = 10.0

        paused = runner.fsm.step(mfsm.Sense(
            now=10.1,
            health=mfsm.Health(amcl_ok=False),
            grasp_verified=True,
            arm_at_home=True,
        ))
        self.assertIs(paused.state, mfsm.State.PAUSED)
        with mock.patch("builtins.input", return_value=""), redirect_stdout(io.StringIO()):
            runner._handle_pause(paused.reason)

        self.assertTrue(runner._grasp_verified)
        self.assertIsNotNone(runner._latched)
        self.assertTrue(runner._at_nav_home)

        recheck = runner.fsm.step(mfsm.Sense(
            now=10.2, operator_cleared=True, grasp_verified=True,
            arm_at_home=True,
        ))
        self.assertIs(recheck.state, mfsm.State.SELF_CHECK)
        resumed = runner.fsm.step(mfsm.Sense(
            now=10.3, self_check_passed=True, grasp_verified=True,
            arm_at_home=True,
        ))
        self.assertIs(resumed.state, mfsm.State.DELIVER)
        self.assertIs(resumed.action, mfsm.Action.DRIVE_BIN)

    def test_carry_pause_selfcheck_reparks_without_opening_the_jaw(self):
        runner = build_runner()
        runner._grasp_verified = True
        runner._place_finished = False
        runner._at_nav_home = False
        runner._arm_at_home = False
        with redirect_stdout(io.StringIO()):
            self.assertTrue(runner.run_self_check())
        self.assertTrue(runner._at_nav_home)
        self.assertTrue(runner._arm_at_home)
        label, kwargs = runner.controller.move_details[-1]
        self.assertEqual(label, "nav-home")
        self.assertTrue(kwargs["grip_is_hold"])

    def test_no_deliver_stops_and_holds_without_the_bin(self):
        runner, states = self._run(deliver=False)
        S = mfsm.State
        self.assertIn(S.CARRY_HOME, states, states)
        self.assertNotIn(S.DELIVER, states, states)
        self.assertNotIn(S.PLACE, states, states)
        self.assertIs(states[-1], S.COMPLETE, states)
        self.assertEqual(runner.controller.ran_release, 0)

    def test_state_is_clean_when_patrol_resumes(self):
        # A stale latch makes the NEXT object's LATCH transition straight to
        # GRASP with the previous coordinates.
        runner, _ = self._run()
        self.assertIsNone(runner._latched)
        self.assertFalse(runner._align_failed)
        self.assertFalse(runner._handoff_ready)
        self.assertFalse(runner._grasp_finished)
        self.assertEqual(runner._det_streak, 0)
        self.assertTrue(runner._at_nav_home, "patrol resumed with the arm at C3")


class FineAlignSafetyRegression(unittest.TestCase):
    class Harness:
        def __init__(self, points):
            self.ncfg = nrgp.nr.NavRLConfig()
            self.lidar = type("Lidar", (), {"get_points": lambda _self: points})()
            self.show = False
            self.stops = 0
            self.drives = 0

        def stop(self):
            self.stops += 1

        def _detect_arm(self):
            return True, 0.5, 0.0, 40.0, "sugarbox"

        def _drive(self, action, speed):
            self.drives += 1

    def test_blocking_align_obeys_its_deadline(self):
        h = self.Harness([(0.0, 4.0)])
        with redirect_stdout(io.StringIO()):
            ok = nrgp.RLNavigator._arm_align(h, deadline=time.monotonic() - 0.01)
        self.assertFalse(ok)
        self.assertGreaterEqual(h.stops, 1)

    def test_blocking_align_obeys_live_health_and_lidar_brake(self):
        h = self.Harness([(0.0, 0.10)])
        checks = iter(("", "wheel feedback stale"))
        with mock.patch.object(nrgp.vgp, "decide_arm_action_by_distance",
                               return_value=("forward", 0.1, "move", None, 0.0)), \
             mock.patch.object(nrgp.vgp, "action_to_vxyz", return_value=(0.1, 0.0, 0.0)), \
             mock.patch.object(nrgp.vgp, "ARM_DECISION_INTERVAL", 0.0), \
             redirect_stdout(io.StringIO()):
            ok = nrgp.RLNavigator._arm_align(
                h, deadline=time.monotonic() + 1.0,
                safety_check=lambda: next(checks),
            )
        self.assertFalse(ok)
        self.assertEqual(h.drives, 0, "camera loop drove through the lidar brake")
        self.assertGreaterEqual(h.stops, 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
