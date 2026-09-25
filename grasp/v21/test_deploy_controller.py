"""Controller state-machine tests against a fake servo plant.

These exercise the deployment control layer without hardware and without the policy.
The plant deliberately behaves like the real one in the ways that have bitten us:

  * it moves at most ``max_delta_deg`` per command, so a distant target needs many
    commands — the controller must iterate, not fire once and assume arrival;
  * its gripper STALLS when an object is between the fingers, and closes all the way
    when there is nothing — the only grasp signal this robot has;
  * reads and writes can be made to fail, and a failure must stop the controller
    rather than let it carry on with an imagined pose.

Run:  python test_deploy_controller.py
"""
from __future__ import annotations

import contextlib
import io
import os
import sys
import threading
import time
from typing import List, Optional

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import deploy_contract as dc

FAILURES: List[str] = []
CHECKS = 0
OPEN_FK = []


def check(cond: bool, label: str, detail: str = "") -> None:
    global CHECKS
    CHECKS += 1
    if cond:
        print(f"  [ok]   {label}")
    else:
        print(f"  [FAIL] {label}" + (f"  — {detail}" if detail else ""))
        FAILURES.append(label)


# ───────────────────────────────────────────────────────────────────────────
# Fake plant
# ───────────────────────────────────────────────────────────────────────────

class FakeServoPlant:
    """Stands in for Arm_Lib. Tracks true angles and applies the real constraints."""

    def __init__(self, start_deg, max_delta_deg=8.0,
                 object_blocks_at_deg: Optional[float] = None):
        self.true = list(map(float, start_deg))
        self.max_delta = float(max_delta_deg)
        # Gripper angle at which an object stops the jaw. None = empty jaw.
        self.object_blocks_at = object_blocks_at_deg
        self.fail_writes = False
        self.fail_reads = False
        self.writes = 0
        self.reads = 0

    def write(self, deg6) -> None:
        self.writes += 1
        if self.fail_writes:
            raise RuntimeError("simulated servo write failure")
        for i, want in enumerate(deg6):
            delta = float(np.clip(float(want) - self.true[i], -self.max_delta, self.max_delta))
            self.true[i] += delta
        if self.object_blocks_at is not None:
            # Closing = increasing hw degrees on this robot (30 open → 180 closed).
            self.true[5] = min(self.true[5], self.object_blocks_at)

    def read(self, idx1based: int):
        self.reads += 1
        if self.fail_reads:
            raise RuntimeError("simulated servo read failure")
        return self.true[idx1based - 1]


class SlippingJawPlant(FakeServoPlant):
    """A jaw that keeps moving under load, just slower than it was told to.

    This is the case the absolute progress test cannot see. The object is
    gripped but creeps, so the encoder advances more than
    jaw_contact_min_progress_deg every iteration, `tracking` reads true, the
    contact streak resets, and the command squeezes harder for the rest of the
    close -- the clicking heard on hardware.
    """

    def __init__(self, start_deg, grip_at_deg, slip_deg_per_write=3.0,
                 total_slip_deg=18.0, **kw):
        super().__init__(start_deg, **kw)
        self.grip_at = float(grip_at_deg)
        self.slip = float(slip_deg_per_write)
        # A real object creeps a little and then holds. Letting it slide forever
        # would make the jaw reach the stop for an honest reason and the force
        # ceiling would look broken when it was working.
        self.hard_stop = float(grip_at_deg) + float(total_slip_deg)
        self.jaw_commands = []

    def write(self, deg6):
        self.jaw_commands.append(float(deg6[5]))
        before = self.true[5]
        super().write(deg6)
        if self.true[5] > self.grip_at:
            # Past the object: it yields self.slip per command at most, and stops
            # yielding altogether once it has given up total_slip_deg.
            self.true[5] = min(self.true[5], before + self.slip, self.hard_stop)


def build(plant, **cfg_over):
    """Controller wired to a fake plant, with no model and no PyBullet policy."""
    with contextlib.redirect_stdout(io.StringIO()):
        import x3plus_real_grasp as X
        cfg = X.DeployConfig()
        for k, v in cfg_over.items():
            setattr(cfg, k, v)

        # Construct as dry-run so no serial device is opened: on the Jetson the
        # Rosmaster driver is importable, and dry_run=False would make this test
        # claim /dev/myserial (three times) purely to build a shell it immediately
        # replaces. The two lines below restore the non-dry state the checks need,
        # so behaviour is unchanged — the fake plant supplies every reading anyway.
        servo = X.ServoController(cfg, dry_run=True)
        servo.dry_run = False
        servo.readings_are_simulated = False

        def _send(deg6, run_time_ms=None):
            safe = []
            for want, prev in zip(deg6, servo._last_deg):
                d = float(np.clip(want - prev, -cfg.max_delta_deg, cfg.max_delta_deg))
                safe.append(float(np.clip(prev + d, 0.0, 270.0)))
            try:
                plant.write(safe)
            except Exception as e:
                print(f"[ERROR] Servo write failed: {e}")
                return False
            servo._last_deg = safe[:]
            return True

        def _read():
            out, bad = [], []
            for i in range(1, 7):
                try:
                    out.append(float(plant.read(i)))
                except Exception as e:
                    bad.append(f"S{i}: {e}")
                    out.append(float("nan"))
            if bad:
                return X.ServoReadResult(out, False, "; ".join(bad))
            return X.ServoReadResult(out, True, "ok")

        servo.send_degrees = _send
        servo.read_degrees = _read

        ctrl = X.GraspController.__new__(X.GraspController)
        ctrl.cfg = cfg
        ctrl.servo = servo
        ctrl.mapper = X.JointMapper(cfg)
        ctrl.fk = X.FKComputer(cfg.urdf_path)
        # Each test builds a separate PyBullet DIRECT client. Desktop machines can
        # survive all of them until process exit, but a Jetson Nano is OOM-killed
        # around the fifteenth client. Track them so the runner can disconnect after
        # every case instead of turning a safety check into a memory stress test.
        OPEN_FK.append(ctrl.fk)
        ctrl.floor_guard = X.FloorGuard(cfg, ctrl.fk)
        ctrl._current_arm_rads = ctrl.mapper.hw_deg_to_sim_arm(list(cfg.home_deg[:5]))
        ctrl._current_grip_rad = ctrl.mapper.hw_deg_to_sim_grip(cfg.home_deg[5])
        ctrl._grip_hold_rad = None
        ctrl._hover_count = 0
        ctrl._guard_tally = {}
        ctrl._guard_deadlock_count = 0
        ctrl._guard_deadlock_z = None
        ctrl._wrist_z_offset = None
        ctrl._stage = 0
        ctrl._prev_action = np.zeros(6, dtype=np.float32)
    return ctrl, cfg


# ───────────────────────────────────────────────────────────────────────────

def test_long_move_needs_many_steps():
    print("\n[1] a long move is iterated until it actually arrives")
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30])
    ctrl, cfg = build(plant)
    # Ask for a pose ~40 deg away on S2.
    target = ctrl.mapper.hw_deg_to_sim_arm([90.0, 110.0, 9.79, 9.79, 90.0])
    with contextlib.redirect_stdout(io.StringIO()):
        res = ctrl.move_guarded_and_verified(
            target, ctrl._current_grip_rad, label="t", run_time_ms=1, settle_s=0.0)
    check(res["reached"], "long move reports arrival", res["reason"])
    check(res["iters"] >= 5, f"took {res['iters']} iterations (>1 command)",
          "a single 8 deg-limited command cannot cover 40 deg")
    check(abs(plant.true[1] - 110.0) <= 2.0,
          f"plant S2 actually reached {plant.true[1]:.1f} deg")

    with contextlib.redirect_stdout(io.StringIO()):
        reached_deg = ctrl.mapper.sim_arm_to_hw_deg(ctrl._current_arm_rads)
    check(abs(reached_deg[1] - plant.true[1]) < 1.0,
          "internal state matches the plant, not the command")


def test_small_home_residual_has_a_bounded_recovery_window():
    print("\n[1b] a 2.1 deg chassis-jostle residual is recoverable, but not unbounded")
    # S2 is stuck 2.1 deg from C3. This reproduces the hardware report without
    # pretending that a large or unknown arm offset is safe for camera geometry.
    plant = FakeServoPlant([90, 64.98, 9.79, 9.79, 90, 30], max_delta_deg=0.0)
    ctrl, cfg = build(plant)
    start_pose = (cfg.grasp_home_deg if cfg.grasp_home_deg is not None
                  else cfg.home_deg)
    target = ctrl.mapper.hw_deg_to_sim_arm(list(start_pose[:5]))
    grip = ctrl.mapper.hw_deg_to_sim_grip(start_pose[5])

    with contextlib.redirect_stdout(io.StringIO()):
        recovered = ctrl.move_guarded_and_verified(
            target, grip, label="startup-home", run_time_ms=1,
            settle_s=0.0, tol_deg=3.0)
    check(recovered["reached"],
          "3.0 deg startup window accepts the measured 2.1 deg residual",
          recovered["reason"])
    check(abs(recovered.get("residual_deg", 99.0) - 2.1) < 0.01,
          f"reported residual stays explicit ({recovered.get('residual_deg')} deg)")

    with contextlib.redirect_stdout(io.StringIO()):
        refused = ctrl.move_guarded_and_verified(
            target, grip, label="normal-home", run_time_ms=1,
            settle_s=0.0, tol_deg=2.0)
    check(not refused["reached"],
          "the normal 2.0 deg gate still refuses the same stalled pose")


def test_finger_error_delays_the_pads_ready_gate():
    print("")
    print("[1c] the measured finger error reaches pads_ready, not just the guard")
    # Found on hardware 2026-08-30. With a 6.5 cm box the gate went true while the
    # z error was still 23 mm: modelled pads at 53 mm against a 65 mm object top,
    # real pads about 15 mm higher and therefore ABOVE the object. pads_ready
    # forces the jaw to MIN_CLOSE_ACTION, so it shut during the approach and the
    # closed jaw jammed the arm short of the target. Correcting only the floor
    # guard leaves this untouched -- the two consumers fail in opposite
    # directions and both need the same physical number.
    with contextlib.redirect_stdout(io.StringIO()):
        import x3plus_real_grasp as _X
    home = list(_X.DeployConfig().home_deg)
    obj = np.array([0.2666, -0.0158, 0.0325], dtype=np.float64)
    height = 0.065

    plain, _ = build(FakeServoPlant(home, max_delta_deg=0.0))
    corrected, _ = build(FakeServoPlant(home, max_delta_deg=0.0),
                         floor_finger_error_m=0.015)
    wrist = dc.episode_wrist_z_offset(height, float(obj[2]))
    plain._wrist_z_offset = wrist
    corrected._wrist_z_offset = wrist
    with contextlib.redirect_stdout(io.StringIO()):
        a = plain._grasp_geometry(obj, height)
        b = corrected._grasp_geometry(obj, height)

    delta = b["pad_after_close_m"] - a["pad_after_close_m"]
    check(abs(delta - 0.015) < 1e-9,
          f"pad_after_close rises by exactly the finger error ({delta*1000:.2f} mm)")
    check(a["finger_error_m"] == 0.0 and abs(b["finger_error_m"] - 0.015) < 1e-12,
          "the geometry reports which correction it used")
    check(b["object_top_m"] == a["object_top_m"],
          "the object is unchanged -- only where the pads are believed to be")

    # The gate must be strictly harder to satisfy, never easier.
    check(b["pad_after_close_m"] > a["pad_after_close_m"],
          "the corrected pads sit HIGHER, so the gate fires later, not sooner")
    if a["pads_ready"]:
        check(not b["pads_ready"] or b["pad_after_close_m"] <= b["object_top_m"] - dc.STAGE1_MIN_ENGAGE_DEPTH,
              "a pose the model called ready is only still ready if the REAL pads "
              "clear the engagement depth")

    # Default must remain training-identical, or every existing run changes.
    default_cfg = _X.DeployConfig()
    check(default_cfg.floor_finger_error_m == 0.0,
          "the default is still 0.0, so nothing changes without an explicit opt-in")


def test_tcp_forward_error_moves_control_forward_without_moving_the_object():
    print("\n[1c2] TCP landing correction asks for another 10 mm forward")
    with contextlib.redirect_stdout(io.StringIO()):
        import x3plus_real_grasp as _X

    raw = np.array([0.250, -0.010, 0.080], dtype=np.float64)
    corrected = _X.control_tcp_position(raw, 0.010)
    check(abs(corrected[0] - 0.240) < 1e-12,
          "a +10mm FK forward error subtracts 10mm from control TCP X")
    check(np.allclose(corrected[1:], raw[1:]) and np.allclose(raw, [0.25, -0.01, 0.08]),
          "Y/Z and the caller's raw FK position stay unchanged")
    try:
        _X.control_tcp_position(raw, 0.021)
    except ValueError:
        check(True, "TCP correction is hard-bounded at 20mm")
    else:
        check(False, "TCP correction is hard-bounded at 20mm")

    class StubFK:
        @staticmethod
        def compute(_arm, _grip):
            return raw.copy(), np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)

    builder = _X.ObsBuilder(
        StubFK(), None, None, dc.OBS_28_INCREMENTAL,
        tcp_forward_error_m=0.010)
    obj = np.array([0.270, -0.010, 0.0325], dtype=np.float32)
    obs = builder.build(
        np.zeros(5, dtype=np.float32), -1.5, obj, 0,
        np.zeros(6, dtype=np.float32))
    check(abs(float(obs[6]) - 0.240) < 1e-6,
          "policy tcp_pos receives the corrected X")
    check(abs(float(obs[13]) - 0.270) < 1e-6,
          "policy object_pos remains the measured camera coordinate")
    check(abs(float(obs[16]) - 0.030) < 1e-6,
          "policy rel_pos asks for 10mm more forward travel")

    plant = FakeServoPlant(list(_X.DeployConfig().home_deg))
    ctrl, cfg = build(plant)
    cfg.tcp_forward_error_m = 0.010
    ctrl._wrist_z_offset = dc.episode_wrist_z_offset(0.065, 0.0325)
    raw_tcp, _ = ctrl.fk.compute(ctrl._current_arm_rads, ctrl._current_grip_rad)
    gate = ctrl._grasp_geometry(obj, 0.065)
    check(abs(float(gate["tcp"][0]) - (float(raw_tcp[0]) - 0.010)) < 1e-9,
          "close gate uses the same corrected TCP as the policy")


def test_a_slipping_object_is_contact_not_a_reason_to_squeeze():
    print("")
    print("[1d] a jaw that moves SLOWER than commanded is contact too")
    with contextlib.redirect_stdout(io.StringIO()):
        import x3plus_real_grasp as _X
    home = list(_X.DeployConfig().home_deg)

    def close(**over):
        plant = SlippingJawPlant(home, grip_at_deg=110.0, slip_deg_per_write=3.0)
        ctrl, cfg = build(plant, **over)
        arm = ctrl.mapper.hw_deg_to_sim_arm(home[:5])
        with contextlib.redirect_stdout(io.StringIO()):
            out = ctrl.move_guarded_and_verified(
                arm, ctrl.mapper.hw_deg_to_sim_grip(180.0), label="close",
                run_time_ms=1, settle_s=0.0, tol_deg=2.0,
                stop_on_jaw_contact=True)
        return plant, out

    # The encoder advances 3 deg per write, above the 2 deg absolute threshold,
    # so `tracking` reads true and the streak resets on every iteration. The
    # absolute detector only wins once the object stops yielding altogether --
    # by which time the command has squeezed a long way past first contact. That
    # over-squeeze is the clicking, and it is what the ratio test removes.
    OBJECT_AT = 110.0
    slow, slow_out = close()
    fast, fast_out = close(jaw_contact_track_fraction=0.5)
    slow_excess = max(slow.jaw_commands) - OBJECT_AT
    fast_excess = max(fast.jaw_commands) - OBJECT_AT
    check(slow_excess > 25.0,
          f"the absolute test alone squeezes {slow_excess:.0f} deg past contact")
    check(fast_excess < slow_excess / 2.0,
          f"the ratio test cuts that to {fast_excess:.0f} deg")

    plant, out = fast, fast_out
    check(bool(out.get("jaw_contact")),
          "the ratio test DOES detect it")
    check(out.get("jaw_contact_mode") == "slipping",
          f"...and says which test fired ({out.get('jaw_contact_mode')})")
    check("slower than commanded" in out.get("reason", ""),
          "the reason line distinguishes a slip from a stall")

    # Nothing about the ordinary blocked jaw changes.
    hard = FakeServoPlant(home, object_blocks_at_deg=110.0)
    ctrl, cfg = build(hard)
    arm = ctrl.mapper.hw_deg_to_sim_arm(home[:5])
    with contextlib.redirect_stdout(io.StringIO()):
        out = ctrl.move_guarded_and_verified(
            arm, ctrl.mapper.hw_deg_to_sim_grip(180.0), label="close",
            run_time_ms=1, settle_s=0.0, tol_deg=2.0, stop_on_jaw_contact=True)
    check(bool(out.get("jaw_contact")) and out.get("jaw_contact_mode") == "stalled",
          "a hard block is still reported as a stall, not a slip")


def test_stage0_policy_close_uses_the_slow_tracking_detector():
    print("")
    print("[1dd] Stage-0 policy close catches slip, not just a frozen encoder")
    with contextlib.redirect_stdout(io.StringIO()):
        import x3plus_real_grasp as _X
    home = list(_X.DeployConfig().home_deg)
    ctrl, cfg = build(
        FakeServoPlant(home),
        stage0_s6_stall_steps=2,
        jaw_contact_track_fraction=0.5)

    def reset():
        ctrl._stage0_s6_prev_deg = None
        ctrl._stage0_s6_prev_cmd_deg = None
        ctrl._stage0_s6_stall_count = 0

    # The previous command still had 8 degrees to travel. Advancing by only 3
    # degrees consumes 37.5% of it, twice in succession: this is exactly the
    # loaded/slipping pattern from the hardware log that the old unchanged-only
    # Stage-0 shortcut missed.
    reset()
    ctrl._observe_stage0_jaw(140.0, 148.0, True)
    one = ctrl._observe_stage0_jaw(143.0, 151.0, True)
    two = ctrl._observe_stage0_jaw(146.0, 154.0, True)
    check(one["mode"] == "slipping" and one["count"] == 1,
          "one slow sample starts, but does not yet confirm, contact")
    check(two["mode"] == "slipping" and two["count"] == 2,
          "two consecutive slow samples confirm the Stage-0 contact")
    check(abs(two["encoder_progress_deg"] - 3.0) < 1e-9
          and abs(two["requested_progress_deg"] - 8.0) < 1e-9,
          "the evidence reports 3 degrees moved out of 8 requested")

    # A free jaw consumes the request and must not be mistaken for an object.
    reset()
    ctrl._observe_stage0_jaw(140.0, 148.0, True)
    free = ctrl._observe_stage0_jaw(148.0, 156.0, True)
    check(not free["blocked"] and free["count"] == 0,
          "a normally tracking free jaw does not trigger")

    # The feature remains an explicit opt-in, and losing eligibility breaks a
    # streak so evidence cannot leak across unrelated policy motion.
    cfg.jaw_contact_track_fraction = 0.0
    reset()
    ctrl._observe_stage0_jaw(140.0, 148.0, True)
    disabled = ctrl._observe_stage0_jaw(143.0, 151.0, True)
    check(not disabled["blocked"], "fraction 0 preserves unchanged-only behaviour")
    cfg.jaw_contact_track_fraction = 0.5
    reset()
    ctrl._observe_stage0_jaw(140.0, 148.0, True)
    ctrl._observe_stage0_jaw(143.0, 151.0, True)
    ctrl._observe_stage0_jaw(146.0, 154.0, False)
    after_reset = ctrl._observe_stage0_jaw(149.0, 157.0, True)
    check(after_reset["count"] == 0,
          "an ineligible sample clears the contact streak")


def test_the_jaw_command_can_never_outrun_the_encoder():
    print("")
    print("[1e] the force ceiling bounds torque even when no detector fires")
    with contextlib.redirect_stdout(io.StringIO()):
        import x3plus_real_grasp as _X
    home = list(_X.DeployConfig().home_deg)

    # Detection switched off entirely, so only the ceiling is left.
    plant = SlippingJawPlant(home, grip_at_deg=110.0, slip_deg_per_write=3.0)
    ctrl, cfg = build(plant, jaw_contact_lag_deg=1e6)
    arm = ctrl.mapper.hw_deg_to_sim_arm(home[:5])
    with contextlib.redirect_stdout(io.StringIO()):
        ctrl.move_guarded_and_verified(
            arm, ctrl.mapper.hw_deg_to_sim_grip(180.0), label="close",
            run_time_ms=1, settle_s=0.0, tol_deg=2.0, stop_on_jaw_contact=True)
    capped = max(plant.jaw_commands)
    check(capped <= plant.true[5] + cfg.jaw_max_lag_deg + 1e-6,
          f"the command never exceeds encoder + {cfg.jaw_max_lag_deg:.0f} deg "
          f"(max {capped:.1f}, encoder {plant.true[5]:.1f})")

    # And with the ceiling removed the same run walks to the stop -- which is
    # what makes the check above meaningful rather than incidental.
    loose = SlippingJawPlant(home, grip_at_deg=110.0, slip_deg_per_write=3.0)
    ctrl2, _ = build(loose, jaw_contact_lag_deg=1e6, jaw_max_lag_deg=0.0)
    with contextlib.redirect_stdout(io.StringIO()):
        ctrl2.move_guarded_and_verified(
            ctrl2.mapper.hw_deg_to_sim_arm(home[:5]),
            ctrl2.mapper.hw_deg_to_sim_grip(180.0), label="close",
            run_time_ms=1, settle_s=0.0, tol_deg=2.0, stop_on_jaw_contact=True)
    uncapped = max(loose.jaw_commands)
    check(uncapped - loose.true[5] > cfg.jaw_max_lag_deg,
          f"without the ceiling the error grows past it "
          f"({uncapped - loose.true[5]:.0f} deg vs {cfg.jaw_max_lag_deg:.0f})")
    check(uncapped > capped,
          f"...and the command goes further ({uncapped:.0f} vs {capped:.0f} deg)")

    # A free jaw must still arrive; the ceiling may not throttle normal closing.
    free = FakeServoPlant(home)
    ctrl3, cfg3 = build(free)
    with contextlib.redirect_stdout(io.StringIO()):
        out = ctrl3.move_guarded_and_verified(
            ctrl3.mapper.hw_deg_to_sim_arm(home[:5]),
            ctrl3.mapper.hw_deg_to_sim_grip(180.0), label="close",
            run_time_ms=1, settle_s=0.0, tol_deg=2.0)
    check(out["reached"], "an unobstructed jaw still reaches its target")
    check(cfg3.jaw_max_lag_deg > cfg3.jaw_hold_bias_deg,
          "the ceiling stays above the deliberate hold bias")


def test_write_failure_stops_and_does_not_advance():
    print("\n[2] a write failure stops the move and leaves state untouched")
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30])
    ctrl, cfg = build(plant)
    before = np.array(ctrl._current_arm_rads, dtype=float).copy()
    plant.fail_writes = True
    target = ctrl.mapper.hw_deg_to_sim_arm([90.0, 110.0, 9.79, 9.79, 90.0])
    with contextlib.redirect_stdout(io.StringIO()):
        res = ctrl.move_guarded_and_verified(
            target, ctrl._current_grip_rad, label="t", run_time_ms=1, settle_s=0.0)
    check(not res["reached"], "move reports failure")
    check("write failed" in res["reason"], f"reason names the write failure: {res['reason']}")
    check(np.allclose(before, np.asarray(ctrl._current_arm_rads, dtype=float)),
          "internal arm state did not advance")


def test_read_failure_stops_and_does_not_advance():
    print("\n[3] a read failure stops the move and leaves state untouched")
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30])
    ctrl, cfg = build(plant)
    before = np.array(ctrl._current_arm_rads, dtype=float).copy()
    plant.fail_reads = True
    target = ctrl.mapper.hw_deg_to_sim_arm([90.0, 110.0, 9.79, 9.79, 90.0])
    with contextlib.redirect_stdout(io.StringIO()):
        res = ctrl.move_guarded_and_verified(
            target, ctrl._current_grip_rad, label="t", run_time_ms=1, settle_s=0.0)
    check(not res["reached"], "move reports failure")
    check("read failed" in res["reason"], f"reason names the read failure: {res['reason']}")
    check(np.allclose(before, np.asarray(ctrl._current_arm_rads, dtype=float)),
          "internal arm state did not advance")


def test_empty_jaw_closes_fully_and_is_rejected():
    print("\n[4] empty jaw closes to the stop → grasp REJECTED")
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30], object_blocks_at_deg=None)
    ctrl, cfg = build(plant)
    with contextlib.redirect_stdout(io.StringIO()):
        res = ctrl.move_guarded_and_verified(
            ctrl._current_arm_rads, dc.GRIPPER_ANGLE_CLOSED, label="close",
            run_time_ms=1, settle_s=0.0, max_iters=60, tol_deg=3.0)
        status, why = ctrl._grasp_looks_real()
    check(res["reached"], "close completed", res["reason"])
    check(abs(plant.true[5] - cfg.gripper_hw_closed) < 3.0,
          f"jaw reached the closed stop ({plant.true[5]:.1f} deg)")
    check(status == "rejected", f"grasp check rejects an empty jaw (got {status!r})", why)


def test_object_stall_is_accepted():
    print("\n[5] object stalls the jaw → grasp ACCEPTED")
    # 150 deg span; stall 30 deg short = 20%, well over the 6% threshold.
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30], object_blocks_at_deg=150.0)
    ctrl, cfg = build(plant)
    with contextlib.redirect_stdout(io.StringIO()):
        ctrl.move_guarded_and_verified(
            ctrl._current_arm_rads, dc.GRIPPER_ANGLE_CLOSED, label="close",
            run_time_ms=1, settle_s=0.0, max_iters=60, tol_deg=3.0)
        status, why = ctrl._grasp_looks_real()
    check(abs(plant.true[5] - 150.0) < 1.0,
          f"jaw stalled on the object at {plant.true[5]:.1f} deg")
    check(status == "confirmed",
          f"grasp check accepts a genuine stall (got {status!r})", why)


def test_open_jaw_is_not_a_grasp():
    print("\n[5b] jaw still open -> grasp REJECTED")
    # Nothing is commanded here: the jaw sits where a dropped object leaves it.
    # The check used to measure only distance from the CLOSED stop, so a wide
    # open jaw scored 100% and read as the most confident grasp possible --
    # which is how run_release_only() came to reach out and mime a drop over an
    # empty gripper on hardware (2026-09-20).
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30], object_blocks_at_deg=None)
    ctrl, cfg = build(plant)
    with contextlib.redirect_stdout(io.StringIO()):
        status, why = ctrl._grasp_looks_real()
    check(status == "rejected", f"an open jaw is not a grasp (got {status!r})", why)
    check("not around anything" in why, f"the reason names the open jaw: {why}")


def test_half_open_jaw_is_not_a_grasp():
    print("\n[5c] jaw only a third closed -> grasp REJECTED")
    # Between the two bounds: short of the closed stop (so the old lower bound
    # passed it) but nowhere near closed enough to be around an object.
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 75], object_blocks_at_deg=None)
    ctrl, cfg = build(plant)
    with contextlib.redirect_stdout(io.StringIO()):
        status, why = ctrl._grasp_looks_real()
    check(status == "rejected", f"a barely-closed jaw is not a grasp (got {status!r})", why)

def test_close_is_rate_limited_and_needs_many_commands():
    print("\n[6] a full close is rate-limited and needs many commands")
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30])
    ctrl, cfg = build(plant)
    span = abs(cfg.gripper_hw_closed - cfg.gripper_hw_open)
    expected_min = int(span / cfg.max_delta_deg) - 1
    with contextlib.redirect_stdout(io.StringIO()):
        res = ctrl.move_guarded_and_verified(
            ctrl._current_arm_rads, dc.GRIPPER_ANGLE_CLOSED, label="close",
            run_time_ms=1, settle_s=0.0, max_iters=60, tol_deg=3.0)
    check(res["iters"] >= expected_min,
          f"close took {res['iters']} commands (>= {expected_min} for {span:.0f} deg "
          f"at {cfg.max_delta_deg:.0f} deg/step)",
          "one send_degrees call cannot close the jaw")


def test_grasp_check_fails_closed_on_unreadable_servo():
    print("\n[7] unreadable gripper → grasp NOT confirmed")
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 150], object_blocks_at_deg=150.0)
    ctrl, cfg = build(plant)
    plant.fail_reads = True
    with contextlib.redirect_stdout(io.StringIO()):
        status, why = ctrl._grasp_looks_real()
    check(status == "rejected",
          f"grasp unconfirmed when the servo cannot be read (got {status!r})", why)


def test_emergency_raise_increases_clearance():
    print("\n[8] emergency raise increases min link z over many below-floor poses")
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30])
    ctrl, cfg = build(plant)
    rng = np.random.RandomState(11)
    limits = np.array(cfg.arm_sim_limits, dtype=np.float64)
    tested = improved = held = 0
    worst_gain = 1e9

    # Uniform joint sampling almost never puts the gripper through the floor (12 hits
    # in 600 tries), which is far too thin to conclude anything. Search for genuinely
    # below-floor poses instead: descend from a random start until the guard line is
    # crossed.
    def below_floor_poses(want=120, budget=6000):
        found = []
        for _ in range(budget):
            if len(found) >= want:
                break
            q = np.array([rng.uniform(lo, hi) for lo, hi in limits])
            for _ in range(25):
                z = ctrl.fk.min_gripper_link_z(q, -1.5)
                if z < ctrl.floor_guard.line:
                    found.append(q.copy())
                    break
                # step whichever of S2/S3/S4 lowers the gripper most
                best, best_z = None, z
                for j in (1, 2, 3):
                    for s in (+1.0, -1.0):
                        c = q.copy()
                        c[j] = float(np.clip(c[j] + s * 0.12, *limits[j]))
                        cz = ctrl.fk.min_gripper_link_z(c, -1.5)
                        if cz < best_z:
                            best, best_z = c, cz
                if best is None:
                    break
                q = best
        return found

    for q in below_floor_poses():
        z = ctrl.fk.min_gripper_link_z(q, -1.5)
        if z >= ctrl.floor_guard.line:
            continue
        tested += 1
        a, g, info = ctrl.floor_guard.project(q, -1.5, q.copy(), -1.5)
        z_after = ctrl.fk.min_gripper_link_z(a, g)
        if info["action"] == "emergency_raise":
            improved += 1
            worst_gain = min(worst_gain, z_after - z)
            if z_after <= z:
                check(False, "emergency raise gained height",
                      f"z {z*1000:.2f} → {z_after*1000:.2f} mm")
                return
        else:
            held += 1
            if z_after < z:
                check(False, "emergency hold did not lose height",
                      f"z {z*1000:.2f} → {z_after*1000:.2f} mm")
                return
    check(tested >= 30, f"exercised {tested} genuinely below-floor poses",
          "too few samples for this to mean anything")
    check(improved > 0, f"{improved} poses raised, {held} held")
    if improved:
        print(f"         smallest height gain among raises: {worst_gain*1000:+.3f} mm")


def test_emergency_raise_actually_reaches_the_plant():
    print("\n[10] emergency raise is SENT and the plant really rises")
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30])
    ctrl, cfg = build(plant)

    # Find a below-floor pose and put both the plant and the controller there.
    rng = np.random.RandomState(5)
    limits = np.array(cfg.arm_sim_limits, dtype=np.float64)
    start = None
    for _ in range(4000):
        q = np.array([rng.uniform(lo, hi) for lo, hi in limits])
        if ctrl.fk.min_gripper_link_z(q, -1.5) < ctrl.floor_guard.line:
            start = q
            break
    if start is None:
        check(False, "found a below-floor pose to start from")
        return

    with contextlib.redirect_stdout(io.StringIO()):
        hw = ctrl.mapper.sim_arm_to_hw_deg(start)
        hw.append(ctrl.mapper.sim_grip_to_hw_deg(-1.5))
    plant.true = [float(v) for v in hw]
    ctrl.servo._last_deg = [float(v) for v in hw]
    ctrl._current_arm_rads = np.asarray(start, dtype=np.float32)
    ctrl._current_grip_rad = -1.5

    z_before = ctrl.fk.min_gripper_link_z(
        ctrl.mapper.hw_deg_to_sim_arm(plant.true[:5]),
        ctrl.mapper.hw_deg_to_sim_grip(plant.true[5]))
    writes_before = plant.writes

    with contextlib.redirect_stdout(io.StringIO()):
        target = ctrl.mapper.hw_deg_to_sim_arm([90.0, 67.08, 9.79, 9.79, 90.0])
        res = ctrl.move_guarded_and_verified(
            target, -1.5, label="raise", run_time_ms=1, settle_s=0.0, max_iters=20)

    z_after = ctrl.fk.min_gripper_link_z(
        ctrl.mapper.hw_deg_to_sim_arm(plant.true[:5]),
        ctrl.mapper.hw_deg_to_sim_grip(plant.true[5]))

    check(plant.writes > writes_before,
          f"a command was actually written to the plant ({plant.writes - writes_before})",
          "the raise was reported but never sent")
    check(z_after > z_before,
          f"plant min_link_z rose {z_before*1000:+.2f} → {z_after*1000:+.2f} mm")
    # The only dishonest outcome would be claiming arrival while still below the line.
    check(not (res["reached"] and z_after < ctrl.floor_guard.line),
          f"never claims arrival while still below the line "
          f"(reached={res['reached']}, z={z_after*1000:+.2f} mm)",
          res["reason"])


def test_clamped_close_blocks_lift_and_stage2():
    print("\n[11] guard clamping during the close ⇒ no lift, no stage 2, no success")
    # Jaw physically free, but we place the arm low enough that closing would sweep
    # the links below the safety line, so the guard must clamp the close.
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30])
    ctrl, cfg = build(plant)

    rng = np.random.RandomState(3)
    limits = np.array(cfg.arm_sim_limits, dtype=np.float64)
    low = None
    for _ in range(6000):
        q = np.array([rng.uniform(lo, hi) for lo, hi in limits])
        z_open = ctrl.fk.min_gripper_link_z(q, -1.5)
        z_closed = ctrl.fk.sweep_min_z(q, -1.5, q, dc.GRIPPER_ANGLE_CLOSED, 8)
        if z_open >= ctrl.floor_guard.line > z_closed:
            low = q
            break
    if low is None:
        check(False, "found a pose where closing would breach the line")
        return

    with contextlib.redirect_stdout(io.StringIO()):
        hw = ctrl.mapper.sim_arm_to_hw_deg(low)
        hw.append(ctrl.mapper.sim_grip_to_hw_deg(-1.5))
    plant.true = [float(v) for v in hw]
    ctrl.servo._last_deg = [float(v) for v in hw]
    ctrl._current_arm_rads = np.asarray(low, dtype=np.float32)
    ctrl._current_grip_rad = -1.5

    with contextlib.redirect_stdout(io.StringIO()):
        verdict = ctrl.attempt_close()

    check(not verdict["may_lift"], "close verdict forbids lifting", verdict["reason"])
    check("guard" in verdict["reason"],
          f"reason names the guard: {verdict['reason']}")
    check("clamped" in verdict["guard_actions"],
          f"guard actions recorded: {verdict['guard_actions']}")
    check(ctrl._stage == 0, f"stage did not advance (stage={ctrl._stage})")

    z = ctrl.fk.min_gripper_link_z(
        ctrl.mapper.hw_deg_to_sim_arm(plant.true[:5]),
        ctrl.mapper.hw_deg_to_sim_grip(plant.true[5]))
    check(z >= ctrl.floor_guard.line - 1e-9,
          f"plant never went below the line during the clamped close "
          f"({z*1000:+.2f} mm >= {ctrl.floor_guard.line*1000:.1f} mm)")


def test_fully_closed_jaw_forbids_lift():
    print("\n[12] jaw closing all the way (nothing in it) ⇒ lift forbidden")
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30], object_blocks_at_deg=None)
    ctrl, cfg = build(plant)
    with contextlib.redirect_stdout(io.StringIO()):
        verdict = ctrl.attempt_close()
    check(not verdict["may_lift"],
          "an unobstructed full close is not treated as a grasp", verdict["reason"])
    check("nothing blocked it" in verdict["reason"],
          f"reason explains why: {verdict['reason']}")


def test_stall_permits_lift():
    print("\n[13] genuine stall ⇒ lift permitted")
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30], object_blocks_at_deg=150.0)
    ctrl, cfg = build(plant)
    with contextlib.redirect_stdout(io.StringIO()):
        verdict = ctrl.attempt_close()
    check(verdict["may_lift"], "a real stall permits the lift", verdict["reason"])
    check(verdict["guard_actions"] in (["pass"], ["disabled"]),
          f"guard stayed out of the way: {verdict['guard_actions']}")


def test_jaw_contact_parks_the_command():
    print("\n[13b] jaw contact parks the command at contact+bias — no stall grind")
    # The 2026-07-31 hardware report: an object gripped mid-close, and the command
    # kept walking toward 180 anyway — the servo ground its gears against a target
    # it could never reach, until the operator pulled the object out to protect it.
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30], object_blocks_at_deg=135.0)
    ctrl, cfg = build(plant)
    with contextlib.redirect_stdout(io.StringIO()):
        verdict = ctrl.attempt_close()
    check(verdict["may_lift"], "contact still counts as a grasp", verdict["reason"])
    check("contact" in verdict["reason"],
          f"reason reports the contact, not a bare stall: {verdict['reason']}")
    cmd = ctrl.servo._last_deg[5]
    check(cmd <= 135.0 + cfg.jaw_hold_bias_deg + 1e-6,
          f"command parked at {cmd:.1f} deg — within the hold bias of the 135.0 "
          f"block, nowhere near the 180 stop")
    check(cmd >= 135.0 + 2.0,
          f"command keeps a grip bias past contact ({cmd:.1f} deg), "
          f"not zero squeeze")
    check(ctrl._grip_hold_rad is not None, "the hold is recorded for stage 2")


def test_lift_and_return_keep_the_hold():
    print("\n[13c] stage 2 maintains the hold — grip force survives the ride home")
    # Both failure modes live here: re-commanding the encoder readback zeroes the
    # grip force (object slides out mid-lift), while re-commanding the closed stop
    # resumes the grind. Every jaw command from close to home must stay inside
    # (contact, contact + bias].
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30], object_blocks_at_deg=135.0)
    ctrl, cfg = build(plant)
    jaw_cmds = []
    orig_send = ctrl.servo.send_degrees

    def spy_send(deg6, run_time_ms=None):
        jaw_cmds.append(float(deg6[5]))
        return orig_send(deg6, run_time_ms=run_time_ms)

    ctrl.servo.send_degrees = spy_send
    with contextlib.redirect_stdout(io.StringIO()):
        verdict = ctrl.attempt_close()
        n_close = len(jaw_cmds)
        outcome = ctrl._scripted_lift_and_return()
    check(verdict["may_lift"], "close grips the object", verdict["reason"])
    check(outcome == "confirmed",
          f"stage 2 completes and still detects the object (got {outcome!r})")
    stage2 = jaw_cmds[n_close:]
    hold = ctrl.mapper.sim_grip_to_hw_deg(ctrl._grip_hold_rad)
    check(len(stage2) > 0 and all(abs(c - hold) < 1e-6 for c in stage2),
          f"every stage-2 jaw command is the hold ({hold:.1f} deg); "
          f"saw {sorted(set(round(c, 1) for c in stage2))}")
    check(all(c < 180.0 - 5.0 for c in stage2),
          "no stage-2 command resumes the walk toward the closed stop")
    check(abs(plant.true[5] - 135.0) < 1.0,
          f"the object is still squeezing the jaw at {plant.true[5]:.1f} deg at home")


def _hover_g(r_m, xy_m=0.020, z_m=0.012, pads=True):
    """A stage-0 geometry dict with only the fields the hover check reads."""
    return {"pads_ready": pads, "xy_error_m": xy_m, "z_error_m": z_m,
            "target_distance_m": r_m, "centred": False}


def test_hover_fallback_fires_only_on_a_sustained_near_miss():
    print("\n[13d] hover fallback: 1 mm outside the gate for 3 s earns the close")
    # Run 5, 2026-07-31: the policy grasped the object on its own and hovered at
    # r=27/26mm for 270 steps; max-steps then opened the jaw and gave it back.
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30])
    ctrl, cfg = build(plant)
    near = _hover_g(cfg.entry_radius + 0.001)          # the actual failure: +1 mm

    fired_early = False
    for _ in range(cfg.stage0_hover_steps - 1):
        fire, _why = ctrl._stage0_hover_check(near)
        fired_early = fired_early or fire
    check(not fired_early,
          f"no fire during the first {cfg.stage0_hover_steps - 1} near-miss steps")
    fire, why = ctrl._stage0_hover_check(near)
    check(fire, f"fires on step {cfg.stage0_hover_steps}", why)
    check("hover fallback" in why, f"reason names the fallback: {why!r}")

    # A pass through the band on the way in must not accumulate.
    ctrl._hover_count = 0
    for _ in range(cfg.stage0_hover_steps - 1):
        ctrl._stage0_hover_check(near)
    fire, _why = ctrl._stage0_hover_check(_hover_g(cfg.entry_radius + 0.020))
    check(not fire and ctrl._hover_count == 0,
          "a step outside the band resets the streak")

    # Only the radius may be forgiven — the other gate terms must be genuine.
    ctrl._hover_count = 0
    for g_bad, label in (
            (_hover_g(cfg.entry_radius + 0.001, xy_m=cfg.entry_xy_tol + 0.001),
             "xy out of tolerance"),
            (_hover_g(cfg.entry_radius + 0.001, z_m=cfg.entry_z_tol + 0.001),
             "z out of tolerance"),
            (_hover_g(cfg.entry_radius + 0.001, pads=False), "pads not ready"),
            (_hover_g(cfg.entry_radius + cfg.stage0_hover_extra_m + 0.001),
             "radius beyond the slack")):
        ctrl._hover_count = 0
        counted = False
        for _ in range(cfg.stage0_hover_steps + 5):
            fire, _why = ctrl._stage0_hover_check(g_bad)
            counted = counted or fire
        check(not counted, f"never fires with {label}")


def test_finger_error_correction_raises_only_the_pads():
    print("\n[13e] --floor-finger-error-mm raises the pads, not the whole linkage")
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30])
    ctrl, cfg = build(plant)
    arm = ctrl._current_arm_rads
    grip = ctrl._current_grip_rad
    base = ctrl.fk.min_gripper_link_z(arm, grip)
    corrected = ctrl.fk.min_gripper_link_z(arm, grip, 0.0167)
    check(corrected >= base, f"correction never lowers clearance "
                             f"({base*1000:.1f} -> {corrected*1000:.1f}mm)")
    check(corrected - base <= 0.0167 + 1e-9,
          "correction never exceeds the finger error itself — a non-pad link that is "
          "lowest caps the gain, which is exactly the ambiguity it must not paper over")
    low_idx, low_z = ctrl.fk.lowest_gripper_link(arm, grip)
    check(low_idx is not None, f"the lowest link is identified (link {low_idx}, "
                               f"pad={ctrl.fk.is_pad_link(low_idx)})")
    check(abs(low_z - base) < 1e-9, "lowest_gripper_link agrees with min_gripper_link_z")
    check(cfg.floor_finger_error_m == 0.0,
          "default stays 0.0 — the conservative metric is opt-out, not opt-in")


def test_guard_deadlock_is_bounded():
    print("\n[13f] a guard that intervenes forever without gaining height gives up")
    # Run 6, 2026-07-31: the jaw closed early, close_drop put the modelled pads at
    # 7.4 mm, and the 8 mm line fell between two adjacent encoder counts on S2. The
    # policy commanded down, the guard raised one count, repeat — 42 raises with
    # z_now bit-identical, 213 steps, no progress, until the operator interrupted.
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 30])
    ctrl, cfg = build(plant)
    stuck = {"action": "emergency_raise", "z_now": 0.0074, "z_after": 0.0124,
             "low_link": ctrl.fk.tcp_link_r, "low_is_pad": True}

    def feed(info, n):
        """Replay the main loop's deadlock bookkeeping n times; True if it trips."""
        for _ in range(n):
            if info["action"] in ("emergency_raise", "emergency_hold"):
                z = float(info["z_now"])
                gained = (ctrl._guard_deadlock_z is None
                          or z > ctrl._guard_deadlock_z + cfg.guard_deadlock_progress_m)
                ctrl._guard_deadlock_z = (z if ctrl._guard_deadlock_z is None
                                          else max(ctrl._guard_deadlock_z, z))
                ctrl._guard_deadlock_count = 0 if gained else ctrl._guard_deadlock_count + 1
                if ctrl._guard_deadlock_count >= cfg.guard_deadlock_steps:
                    return True
            else:
                ctrl._guard_deadlock_count = 0
        return False

    check(not feed(stuck, cfg.guard_deadlock_steps - 1),
          f"tolerates {cfg.guard_deadlock_steps - 1} interventions — intervening is "
          f"normal, only never gaining is not")
    check(feed(stuck, 2), f"trips at {cfg.guard_deadlock_steps} consecutive")

    # A guard that is actually working its way up must never trip.
    ctrl._guard_deadlock_count, ctrl._guard_deadlock_z = 0, None
    tripped = False
    for k in range(cfg.guard_deadlock_steps * 3):
        rising = dict(stuck, z_now=0.0074 + k * 0.002)
        tripped = tripped or feed(rising, 1)
    check(not tripped, "a guard gaining 2mm per step is progress, not a deadlock")

    # A single passing step clears the streak.
    ctrl._guard_deadlock_count, ctrl._guard_deadlock_z = 0, None
    feed(stuck, cfg.guard_deadlock_steps - 1)
    feed({"action": "pass"}, 1)
    check(ctrl._guard_deadlock_count == 0, "one clean step resets the streak")


def test_dry_run_never_reports_success():
    print("\n[14] simulated readings never report a grasp as successful")
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 150], object_blocks_at_deg=150.0)
    ctrl, cfg = build(plant)
    ctrl.servo.readings_are_simulated = True
    with contextlib.redirect_stdout(io.StringIO()):
        status, why = ctrl._grasp_looks_real()
    check(status == "unverified",
          f"status is 'unverified', not a pass (got {status!r})", why)
    check(status not in ("confirmed",), "simulated readings cannot confirm a grasp")


def test_unconfirmed_return_home_cannot_report_success():
    print("\n[15] unconfirmed return-home aborts instead of reporting delivery")
    plant = FakeServoPlant([90, 67.08, 9.79, 9.79, 90, 150], object_blocks_at_deg=150.0)
    ctrl, cfg = build(plant)
    ctrl._grasp_looks_real = lambda: ("confirmed", "fake object stall")
    calls = []

    def fake_move(arm_target, grip_target, *, label, **kwargs):
        calls.append(label)
        if label == "return-home":
            return {"reached": False, "reason": "test home stall", "iters": 3,
                    "guard": ["pass"]}
        return {"reached": True, "reason": "arrived", "iters": 1,
                "guard": ["pass"]}

    stopped = {"value": False}
    ctrl.move_guarded_and_verified = fake_move
    ctrl.servo.emergency_stop = lambda: stopped.__setitem__("value", True)
    with contextlib.redirect_stdout(io.StringIO()):
        status = ctrl._scripted_lift_and_return()
    check(status == "aborted", f"return-home failure yields {status!r}, not success")
    check(ctrl._outcome == "return_home_not_confirmed",
          f"outcome records return-home failure ({ctrl._outcome!r})")
    check(stopped["value"], "return-home failure triggers emergency stop")
    check("return-home" in calls, "test exercised the return-home leg")

def test_every_motion_path_is_guarded():
    print("\n[9] no command path bypasses the floor guard")
    import inspect
    import x3plus_real_grasp as X
    # Servo writes are legitimate only inside move_guarded_and_verified. Count call
    # sites in the class as a whole, then subtract those inside the primitive; the
    # remainder are paths that bypass the guard.
    cls_src = inspect.getsource(X.GraspController)
    prim_src = inspect.getsource(X.GraspController.move_guarded_and_verified)
    # _park_jaw_hold is the one deliberate exception: it moves the jaw command BACK
    # toward open, within the range the guard already approved while the close was
    # walking, with the arm unchanged — and a less-closed jaw only ever raises the
    # pads. Its own docstring carries the argument; hold it to exactly one call.
    park_src = inspect.getsource(X.GraspController._park_jaw_hold)

    def count(src):
        return sum(1 for ln in src.splitlines()
                   if "self.servo.send_degrees(" in ln.strip())

    total, inside = count(cls_src), count(prim_src) + count(park_src)
    outside = total - inside
    check(outside == 0,
          f"send_degrees call sites outside the primitives: {outside} "
          f"(total {total}, allowed {inside})",
          "every motion must go through move_guarded_and_verified or _park_jaw_hold")
    check(count(park_src) == 1,
          f"_park_jaw_hold contains exactly one send ({count(park_src)})")
    check("guard" in (X.GraspController._park_jaw_hold.__doc__ or ""),
          "_park_jaw_hold documents why it is allowed to bypass the guard")


def test_socket_payloads_are_validated():
    print("\n[10] socket detections are validated before they can reach the policy")
    import x3plus_real_grasp as X
    P = X.DetectionReceiver._parse_payload

    pos, height, stamp = P(b'{"x": 0.25, "y": 0.01, "z": 0.02, "height": 0.03}')
    check(abs(pos[0] - 0.25) < 1e-6 and abs(pos[1] - 0.01) < 1e-6
          and abs(pos[2] - 0.02) < 1e-6, f"well-formed payload parses: {pos.tolist()}")
    check(height is not None and abs(height - 0.03) < 1e-9,
          f"height is carried through: {height}")
    check(stamp is None, "a payload with no cam_pose yields no stamp")

    _, h_absent, _ = P(b'{"x": 0.25, "y": 0.0, "z": 0.02}')
    check(h_absent is None,
          "absent height stays None (contract decides: hard error or documented "
          "symmetric-object fallback) rather than being invented here")

    _, _, stamp = P(b'{"x": 0.25, "y": 0.0, "cam_pose": [90, 67.08, 9.79, 9.79, 90, 30],'
                    b' "cam_pose_name": "v21_c3_grasp_home"}')
    check(stamp is not None and stamp[1] == "v21_c3_grasp_home",
          f"a cam_pose stamp is carried through with its name: {stamp}")
    check(stamp is not None and abs(stamp[0][1] - 67.08) < 1e-9,
          "...and its joint angles survive intact")

    # A malformed stamp must raise, NOT read as absent: reading it as absent would
    # turn a sender bug into a silently skipped cross-process check, which is the
    # exact failure the stamp exists to prevent.
    for bad, why in [
        (b'{"x": 0.25, "y": 0.0, "cam_pose": "90,67,9,9,90"}', "cam_pose as a string"),
        (b'{"x": 0.25, "y": 0.0, "cam_pose": [90, 67]}', "cam_pose too short"),
        (b'{"x": 0.25, "y": 0.0, "cam_pose": [90,67,9,9,90,30,1]}', "cam_pose too long"),
        (b'{"x": 0.25, "y": 0.0, "cam_pose": [90,67,9,9,null]}', "cam_pose with a null"),
        (b'{"x": 0.25, "y": 0.0, "cam_pose": [90,67,9,9,NaN]}', "cam_pose with NaN"),
    ]:
        raised = False
        try:
            P(bad)
        except Exception:
            raised = True
        check(raised, f"rejected (not silently treated as unstamped): {why}")

    # Each of these used to be accepted by v21's inline parse. A NaN target compares
    # false against every bound, so nothing downstream would have stopped it.
    for bad, why in [
        (b'', "empty message"),
        (b'[1,2,3]', "JSON array, not an object"),
        (b'{"x": NaN, "y": 0.0, "z": 0.02}', "NaN position"),
        (b'{"x": Infinity, "y": 0.0, "z": 0.02}', "infinite position"),
        (b'{"y": 0.0, "z": 0.02}', "missing x"),
        (b'{"x": 0.25, "y": 0.0, "height": -0.01}', "negative height"),
        (b'{"x": 0.25, "y": 0.0, "height": 0.0}', "zero height"),
        (b'{"x": 0.25, "y": 0.0, "height": NaN}', "NaN height"),
    ]:
        raised = False
        try:
            P(bad)
        except Exception:
            raised = True
        check(raised, f"rejected: {why}")


def test_stale_detection_does_not_masquerade_as_a_fresh_one():
    print("\n[11] a detection that stopped arriving is not served as current")
    import x3plus_real_grasp as X
    rx = object.__new__(X.DetectionReceiver)
    rx._default_pos = np.array([0.26, 0.0, 0.02], dtype=np.float32)
    rx._default_height = None
    rx._pos = np.array([0.31, 0.05, 0.02], dtype=np.float32)
    rx._height = 0.04
    rx._stale_timeout_sec = 1.0
    rx._lock = threading.Lock()

    rx._pose_stamp = ((90.0, 67.08, 9.79, 9.79, 90.0, 30.0), "v21_c3_grasp_home")

    rx._last_update_ts = 0.0
    pos, height, fresh, stamp = rx.snapshot()
    check(not fresh, "before any detection arrives, snapshot reports fresh=False")
    check(abs(pos[0] - 0.26) < 1e-6,
          "...and hands back the DEFAULT, not the uninitialised buffer — otherwise "
          "the latch would freeze a placeholder and call it a detection")
    check(stamp is None,
          "...and withholds the pose stamp too, so a stale stamp cannot vouch for a "
          "default position")

    rx._last_update_ts = time.monotonic()
    pos, height, fresh, stamp = rx.snapshot()
    check(fresh and abs(pos[0] - 0.31) < 1e-6, "a just-received detection is fresh")
    check(height is not None and abs(height - 0.04) < 1e-9, "...and carries its height")
    check(stamp is not None and stamp[1] == "v21_c3_grasp_home",
          "...and its pose stamp comes from the same locked read as the position")

    rx._last_update_ts = time.monotonic() - 5.0
    pos, height, fresh, stamp = rx.snapshot()
    check(not fresh, "a detection older than the stale timeout is no longer fresh")
    check(abs(pos[0] - 0.26) < 1e-6, "...and get()/snapshot() fall back to the default")
    check(rx.get_height() is None,
          "...and the stale height is withdrawn, not left standing")


def test_latch_freezes_the_object_and_fails_closed_on_real_hardware():
    print("\n[12] --latch-obj freezes one detection; real mode refuses a stale latch")
    import x3plus_real_grasp as X

    class _Rx:
        """Fresh once, then keeps 'moving' — as the arm camera's readings do."""
        def __init__(self, fresh, stamp=None):
            self.fresh, self.n, self.stamp = fresh, 0, stamp
        def snapshot(self):
            self.n += 1
            if not self.fresh:
                return np.array([0.26, 0.0, 0.02], dtype=np.float32), None, False, None
            return (np.array([0.30 + 0.01 * self.n, 0.0, 0.02], dtype=np.float32),
                    0.03, True, self.stamp)

    g = object.__new__(X.GraspController)
    g.cfg = X.DeployConfig(latch_obj=True, latch_wait_sec=0.2)
    g.detection = _Rx(fresh=True)
    g.obj_provider = None
    g.real_servo = True
    g._fixed_obj = np.array([0.26, 0.0, 0.02], dtype=np.float32)
    g._fixed_height = None
    g._obj_latched = False
    g._latched_obj = None
    g._latched_height = None
    g._episode_object_height = None

    g._latch_object()
    first = g._get_obj_pos().copy()
    for _ in range(5):
        g._get_obj_pos()
    check(g._obj_latched, "latch records that it fired")
    check(np.allclose(g._get_obj_pos(), first),
          f"repeated reads return the same frozen position {first.tolist()} even though "
          f"the receiver keeps reporting new ones")
    check(g._latched_height is not None and abs(g._latched_height - 0.03) < 1e-9,
          "the height is latched together with the position, not re-read separately")

    # Same controller, no fresh detection, real hardware: must refuse rather than
    # glide the arm at a default nobody confirmed.
    g2 = object.__new__(X.GraspController)
    g2.cfg = X.DeployConfig(latch_obj=True, latch_wait_sec=0.05)
    g2.detection = _Rx(fresh=False)
    g2.obj_provider = None
    g2.real_servo = True
    g2._fixed_obj = np.array([0.26, 0.0, 0.02], dtype=np.float32)
    g2._fixed_height = None
    g2._obj_latched = False
    g2._latched_obj = None
    g2._latched_height = None
    raised = False
    try:
        g2._latch_object()
    except RuntimeError:
        raised = True
    check(raised,
          "real mode + latch timeout raises instead of moving toward the default")

    # Dry-run may proceed — verification must stay free — but says so.
    g2.real_servo = False
    g2._latch_object()
    check(g2._obj_latched and np.allclose(g2._get_obj_pos(), [0.26, 0.0, 0.02]),
          "dry-run falls back to the default with a warning instead of refusing")


def test_real_socket_requires_both_acknowledgements():
    print("\n[13] --real --socket refuses without --latch-obj and frame confirmation")
    import inspect
    import x3plus_real_grasp as X
    src = inspect.getsource(X.main)
    check("i_confirm_external_frame" in src and "args.socket" in src,
          "main() gates --real --socket on --i-confirm-external-frame")
    check("args.latch_obj" in src,
          "main() gates --real --socket on --latch-obj")
    # The gates must sit ABOVE the release gate / driver check, i.e. before anything
    # is loaded and long before the serial port is touched.
    i_frame = src.index("i_confirm_external_frame")
    i_latch = src.index("args.latch_obj")
    i_gate = src.index("_release_gate_ok")
    check(i_frame < i_gate and i_latch < i_gate,
          "both refusals happen before the model loads and before the port opens")


def test_no_method_reads_an_attribute_nobody_assigns():
    print("\n[17] every self.<attr> a method reads is assigned somewhere")
    import ast
    import x3plus_real_grasp as X

    src = open(X.__file__, encoding="utf-8").read()
    tree = ast.parse(src)
    cls = next(n for n in ast.walk(tree)
               if isinstance(n, ast.ClassDef) and n.name == "GraspController")

    assigned, read = set(), {}
    for node in ast.walk(cls):
        # self.x = ..., self.x += ..., and `for self.x in ...`
        targets = []
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, (ast.AugAssign, ast.AnnAssign, ast.For)):
            targets = [node.target]
        for tgt in targets:
            for sub in ast.walk(tgt):
                if (isinstance(sub, ast.Attribute)
                        and isinstance(sub.value, ast.Name)
                        and sub.value.id == "self"):
                    assigned.add(sub.attr)
        if (isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Load)
                and isinstance(node.value, ast.Name) and node.value.id == "self"):
            read.setdefault(node.attr, node.lineno)

    # Anything defined on the class itself (methods, class attributes) is fine.
    on_class = {n.name for n in cls.body
                if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
    for n in cls.body:
        if isinstance(n, ast.Assign):
            on_class.update(t.id for t in n.targets if isinstance(t, ast.Name))

    orphans = sorted((name, line) for name, line in read.items()
                     if name not in assigned and name not in on_class)
    # self.real_servo was exactly this: a constructor parameter consumed but never
    # stored, read only by the socket fail-closed branches -- so the AttributeError
    # could only surface at the moment the refusal was supposed to fire.
    check(not orphans,
          f"attributes read but never assigned: {orphans}" if orphans
          else "no attribute is read without being assigned somewhere")
    check("real_servo" in assigned,
          "self.real_servo is stored, not just taken as a parameter")


def test_detection_pose_stamp_is_checked_against_the_encoders():
    print("\n[16] a detection computed for the wrong arm pose is refused")
    import x3plus_real_grasp as X

    C3 = (90.0, 67.08, 9.79, 9.79, 90.0, 30.0)
    NAV = (90.0, 140.0, 0.0, 0.0, 90.0, 30.0)

    class _Servo:
        """Encoders that report a fixed pose, or fail to answer at all."""
        def __init__(self, deg, valid=True):
            self.deg, self.valid = deg, valid
        def read_degrees(self):
            return X.ServoReadResult(list(self.deg), self.valid,
                                     "ok" if self.valid else "bus silent")

    def controller(servo_deg, *, real, tol=1.0, valid=True):
        g = object.__new__(X.GraspController)
        g.cfg = X.DeployConfig(detection_pose_tol_deg=tol)
        g.real_servo = real
        g.servo = _Servo(servo_deg, valid=valid)
        return g

    def refused(g, stamp):
        try:
            g._verify_detection_pose(stamp)
        except RuntimeError as e:
            return str(e)
        return None

    check(refused(controller(C3, real=True), (C3, "v21_c3_grasp_home")) is None,
          "a stamp matching the encoders passes")

    # The v17 nav home is where the arm-camera constants were MEASURED, and what
    # the bridge still assumed on 2026-08-01 while the arm started at C3. Nothing
    # downstream can catch this: the coordinates look perfectly plausible.
    err = refused(controller(C3, real=True), (NAV, "v17_nav_home"))
    check(err is not None and "cam_pose does not match" in err,
          "real mode REFUSES a detection computed at a different arm pose")
    check(err is not None and "v17_nav_home" in err,
          "...and names the pose the sender used, so the fix is obvious")

    check(refused(controller(C3, real=False), (NAV, "v17_nav_home")) is None,
          "dry-run warns about the same mismatch instead of raising")

    # The gripper does not move the camera, and is legitimately somewhere else by
    # the time this side reads its encoders.
    check(refused(controller(tuple(C3[:5]) + (170.0,), real=True),
                  (C3, "v21_c3_grasp_home")) is None,
          "a different gripper angle is not treated as a pose mismatch")

    nudged = (90.0, 67.58, 9.79, 9.79, 90.0, 30.0)     # 0.5 deg off
    check(refused(controller(C3, real=True, tol=1.0), (nudged, "close")) is None,
          "0.5 deg of drift is within the 1.0 deg tolerance")
    far = (90.0, 69.08, 9.79, 9.79, 90.0, 30.0)        # 2.0 deg off
    check(refused(controller(C3, real=True, tol=1.0), (far, "far")) is not None,
          "2.0 deg of drift is refused")

    # An unverifiable stamp is not a passed check. Mirrors read_degrees() never
    # substituting _last_deg.
    err = refused(controller(C3, real=True, valid=False), (C3, "v21_c3_grasp_home"))
    check(err is not None and "cannot read the encoders" in err,
          "unreadable encoders fail closed rather than skipping the check")

    check(refused(controller(C3, real=True), None) is None,
          "an unstamped detection is not refused outright (sender compatibility)")

    # The wire protocol is duplicated in integration/arm_cam_geometry.py so this
    # script stays standalone on the Jetson. Pin the two definitions together: a
    # rename on either side would otherwise silently disable the whole check.
    acg_path = os.path.join(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))), "integration",
        "arm_cam_geometry.py")
    if os.path.exists(acg_path):
        with open(acg_path, encoding="utf-8") as fh:
            acg_src = fh.read()
        check(f'PAYLOAD_POSE_KEY = "{X.DETECTION_POSE_KEY}"' in acg_src,
              f"the sender uses the same cam_pose key ({X.DETECTION_POSE_KEY!r})")
        check(f'PAYLOAD_POSE_NAME_KEY = "{X.DETECTION_POSE_NAME_KEY}"' in acg_src,
              f"...and the same name key ({X.DETECTION_POSE_NAME_KEY!r})")
    else:
        check(False, f"integration/arm_cam_geometry.py not found at {acg_path}")


def run_test(fn) -> None:
    """Run one case and always release the PyBullet clients it constructed."""
    try:
        fn()
    finally:
        while OPEN_FK:
            OPEN_FK.pop().close()


def main() -> int:
    print("=" * 74)
    print("deployment controller state machine (fake servo plant)")
    print("=" * 74)
    tests = (
        test_long_move_needs_many_steps,
        test_small_home_residual_has_a_bounded_recovery_window,
        test_finger_error_delays_the_pads_ready_gate,
        test_a_slipping_object_is_contact_not_a_reason_to_squeeze,
        test_the_jaw_command_can_never_outrun_the_encoder,
        test_write_failure_stops_and_does_not_advance,
        test_read_failure_stops_and_does_not_advance,
        test_empty_jaw_closes_fully_and_is_rejected,
        test_object_stall_is_accepted,
        test_open_jaw_is_not_a_grasp,
        test_half_open_jaw_is_not_a_grasp,
        test_close_is_rate_limited_and_needs_many_commands,
        test_grasp_check_fails_closed_on_unreadable_servo,
        test_emergency_raise_increases_clearance,
        test_emergency_raise_actually_reaches_the_plant,
        test_clamped_close_blocks_lift_and_stage2,
        test_fully_closed_jaw_forbids_lift,
        test_stall_permits_lift,
        test_jaw_contact_parks_the_command,
        test_lift_and_return_keep_the_hold,
        test_hover_fallback_fires_only_on_a_sustained_near_miss,
        test_finger_error_correction_raises_only_the_pads,
        test_guard_deadlock_is_bounded,
        test_dry_run_never_reports_success,
        test_unconfirmed_return_home_cannot_report_success,
        test_every_motion_path_is_guarded,
        test_socket_payloads_are_validated,
        test_stale_detection_does_not_masquerade_as_a_fresh_one,
        test_latch_freezes_the_object_and_fails_closed_on_real_hardware,
        test_real_socket_requires_both_acknowledgements,
        test_no_method_reads_an_attribute_nobody_assigns,
        test_detection_pose_stamp_is_checked_against_the_encoders,
    )
    for test in tests:
        run_test(test)
    print("\n" + "=" * 74)
    if FAILURES:
        print(f"FAILED {len(FAILURES)}/{CHECKS} checks:")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    print(f"all {CHECKS} checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
