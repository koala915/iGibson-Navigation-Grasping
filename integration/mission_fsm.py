#!/usr/bin/env python3
"""Mission state machine: patrol -> spot trash -> approach -> grasp -> bin -> resume.

Implements the state table in SETMOTOR_ODOM_INTEGRATION.md section 7. The FSM is
deliberately PURE: it consumes a snapshot of sensed conditions and returns the
next state plus the action the pipeline should take. It never touches hardware,
so the whole mission -- including every failure path -- is testable offline.

    transition = fsm.step(Sense(...))
    #   transition.state           where we are now
    #   transition.action          what mission_pipeline must do this tick
    #   transition.chassis_allowed may the wheels move at all?
    #   transition.arm_allowed     may the servos move at all?
    #   transition.reset_nav       clear ActionDelay + GoalTracker before driving

Three invariants are enforced here rather than trusted to the caller, because
each of them has already bitten this project or is called out in the handoffs:

1. ``chassis_allowed`` and ``arm_allowed`` are never both true. The wheels and
   the arm share one Rosmaster serial bus and one centre of mass; the arm only
   moves once the base is confirmed stopped (section 6's stationary gate).
2. Any stale sensor while driving stops the robot and goes to PAUSED, which
   only a human can leave. Nothing here auto-resumes motion after a fault
   (INTEGRATION_CONTRACT.md section 8).
3. Changing the goal SOURCE (map waypoint <-> camera target) always sets
   ``reset_nav``. A stale ActionDelay or GoalTracker entry from the previous
   source is a command aimed at a target that no longer exists.

Run:
    python3 integration/mission_fsm.py --selftest
    python3 integration/mission_fsm.py --diagram
"""
from __future__ import annotations

import argparse
import dataclasses
import enum
from typing import Optional, Tuple


class State(str, enum.Enum):
    BOOT = "BOOT"
    SELF_CHECK = "SELF_CHECK"
    IDLE = "IDLE"
    PATROL = "PATROL"
    INVESTIGATE = "INVESTIGATE"
    APPROACH = "APPROACH"
    ALIGN = "ALIGN"
    STATIONARY_GATE = "STATIONARY_GATE"
    LATCH = "LATCH"
    GRASP = "GRASP"
    VERIFY = "VERIFY"
    RETRY = "RETRY"
    CARRY_HOME = "CARRY_HOME"
    DELIVER = "DELIVER"
    PLACE_ALIGN = "PLACE_ALIGN"
    PLACE = "PLACE"
    RESUME = "RESUME"
    COMPLETE = "COMPLETE"
    PAUSED = "PAUSED"
    FAULT = "FAULT"
    ESTOP = "ESTOP"


class Action(str, enum.Enum):
    HOLD = "HOLD"                    # do nothing this tick
    RUN_SELF_CHECK = "RUN_SELF_CHECK"
    WAIT_OPERATOR = "WAIT_OPERATOR"
    STOP = "STOP"                    # command zero and keep commanding zero
    DRIVE_PATROL = "DRIVE_PATROL"    # RL nav toward the current map waypoint
    DRIVE_TARGET = "DRIVE_TARGET"    # RL nav toward the camera target
    TURN_TO_TARGET = "TURN_TO_TARGET"  # slow bearing-only confirm turn
    FINE_ALIGN = "FINE_ALIGN"        # arm-camera visual servo
    SETTLE = "SETTLE"                # zero command, wait for the stationary gate
    LATCH = "LATCH"                  # freeze object pose + height
    RUN_GRASP = "RUN_GRASP"          # v21 GraspController.run()
    VERIFY = "VERIFY"                # did the object leave the floor?
    BACK_OFF = "BACK_OFF"            # short reverse before a retry
    DRIVE_BIN = "DRIVE_BIN"          # RL nav toward the bin approach point
    PLACE = "PLACE"                  # scripted extend + open + retract
    RESUME_PATROL = "RESUME_PATROL"  # re-arm the map goal provider


# States in which the wheels may be commanded non-zero.
DRIVING_STATES = frozenset({
    State.PATROL, State.INVESTIGATE, State.APPROACH, State.ALIGN,
    State.RETRY, State.DELIVER, State.PLACE_ALIGN,
})
# States in which the servos may be commanded.
ARM_STATES = frozenset({State.GRASP, State.PLACE, State.CARRY_HOME})
# States that need a trustworthy map pose. ALIGN/GRASP work in the robot frame,
# so a wobbling AMCL must not abort a grasp that is already under way.
MAP_STATES = frozenset({State.PATROL, State.DELIVER, State.PLACE_ALIGN, State.RESUME})
# States that need /scan, i.e. anything that can move the base.
SCAN_STATES = DRIVING_STATES
# States whose decision depends on wheel feedback even though the base is not
# driving: they gate the ARM on the base being stopped. Stale odometry there
# fails safe on its own (OdomState.stationary requires valid AND fresh, so the
# gate times out) but reports it as "never settled", which sends a human looking
# at the wrong thing. Naming them makes the real reason surface.
ODOM_STATES = DRIVING_STATES | MAP_STATES | frozenset({
    State.STATIONARY_GATE, State.LATCH,
})

# Pausing while holding an object must not drop the mission back to IDLE and
# patrol -- the jaw is still loaded. This maps the state the pause interrupted
# to the (state, action) the mission resumes with after the self-check.
#
# One dict rather than a membership test in PAUSED plus a lookup in SELF_CHECK:
# those were two lists twenty lines apart that had to agree, and the failure
# mode of them disagreeing was a KeyError at the worst possible moment -- while
# recovering a paused mission with an object in the gripper.
#
# PLACE resumes through PLACE_ALIGN so a release interrupted partway is
# re-entered through its stationary gate instead of being blindly re-run.
HELD_OBJECT_RESUME = {
    State.CARRY_HOME:  (State.CARRY_HOME,  Action.STOP),
    State.DELIVER:     (State.DELIVER,     Action.DRIVE_BIN),
    State.PLACE_ALIGN: (State.PLACE_ALIGN, Action.SETTLE),
    State.PLACE:       (State.PLACE_ALIGN, Action.SETTLE),
}


@dataclasses.dataclass
class MissionConfig:
    # ── detection trigger ──
    # Consecutive frames the same class must be seen before abandoning patrol.
    # One frame is a false positive waiting to happen; the robot would leave the
    # route for a shadow.
    detection_streak_needed: int = 3
    # Below this camera distance the target is worth driving to directly.
    investigate_to_approach_m: float = 2.5
    # RL nav hands over to the arm camera here (nav_rl_grasp_pipeline.NAV_STOP_DIST_M).
    approach_to_align_m: float = 0.75

    # ── timeouts (seconds) ──
    # Generous on purpose: the self-check waits on AMCL, and AMCL waits on a
    # human setting the initial pose in RViz. Faulting after a few seconds means
    # the operator can never satisfy it.
    self_check_timeout_s: float = 180.0
    investigate_timeout_s: float = 15.0
    approach_timeout_s: float = 120.0
    align_timeout_s: float = 60.0
    stationary_gate_timeout_s: float = 5.0
    grasp_timeout_s: float = 120.0
    carry_home_timeout_s: float = 20.0
    deliver_timeout_s: float = 240.0
    place_align_timeout_s: float = 45.0
    place_timeout_s: float = 45.0
    target_lost_timeout_s: float = 5.0

    # ── retries ──
    max_grasp_attempts: int = 3

    def validate(self) -> None:
        if self.detection_streak_needed < 1:
            raise ValueError("detection_streak_needed must be >= 1")
        if not 0.0 < self.approach_to_align_m < self.investigate_to_approach_m:
            raise ValueError(
                "distances must satisfy 0 < approach_to_align < investigate_to_approach, "
                f"got {self.approach_to_align_m} and {self.investigate_to_approach_m}")
        if self.max_grasp_attempts < 1:
            raise ValueError("max_grasp_attempts must be >= 1")
        for name in [f.name for f in dataclasses.fields(self) if f.name.endswith("_s")]:
            v = float(getattr(self, name))
            if not (v > 0.0) or v != v:
                raise ValueError(f"{name} must be > 0, got {v}")


@dataclasses.dataclass(frozen=True)
class Health:
    """Everything that can make the robot unsafe to move, in one place."""
    serial_ok: bool = True
    odom_valid: bool = True
    odom_fresh: bool = True
    scan_fresh: bool = True
    amcl_ok: bool = True          # fresh AND covariance within limits
    estop: bool = False
    fault: str = ""               # non-empty means an unrecoverable error

    def blocking_reason(self, state: "State") -> str:
        """Why this state cannot continue, or '' if it can."""
        if not self.serial_ok:
            return "serial/driver lost"
        if state in ODOM_STATES:
            if not self.odom_valid:
                return "wheel feedback invalid"
            if not self.odom_fresh:
                return "wheel feedback stale"
        if state in SCAN_STATES and not self.scan_fresh:
            return "/scan stale"
        if state in MAP_STATES and not self.amcl_ok:
            return "AMCL pose stale or diverged"
        return ""


@dataclasses.dataclass(frozen=True)
class Sense:
    """One tick's worth of the world, as the pipeline measured it."""
    now: float
    health: Health = dataclasses.field(default_factory=Health)

    # operator
    started: bool = False
    self_check_passed: bool = False
    operator_cleared: bool = False       # human acknowledged a PAUSED state

    # patrol (map frame)
    waypoint_reached: bool = False

    # detection / target (robot frame)
    detection_streak: int = 0
    target_visible: bool = False
    target_dist: float = float("inf")
    target_fix_age: float = float("inf")
    approach_ready: bool = True          # optional rear-camera close-range phase done
    approach_failed: bool = False

    # align / handoff
    handoff_ready: bool = False          # object inside the v21 trained envelope
    align_failed: bool = False           # visual servo gave up (e.g. unreachable)

    # chassis / arm
    stationary: bool = False
    arm_at_home: bool = False

    # grasp
    grasp_finished: bool = False         # GraspController.run() returned
    grasp_verified: bool = False         # object actually left the floor
    latched: bool = False

    # deliver
    bin_reached: bool = False
    place_finished: bool = False


@dataclasses.dataclass(frozen=True)
class Transition:
    state: State
    action: Action
    reason: str = ""
    reset_nav: bool = False           # clear ActionDelay + GoalTracker
    changed: bool = False             # state differs from the previous tick

    @property
    def chassis_allowed(self) -> bool:
        return self.state in DRIVING_STATES

    @property
    def arm_allowed(self) -> bool:
        return self.state in ARM_STATES

    @property
    def terminal(self) -> bool:
        return self.state in (State.FAULT, State.ESTOP)

    @property
    def completed(self) -> bool:
        return self.state is State.COMPLETE


class MissionFSM:
    """The mission controller. ``step()`` is a pure function of (state, Sense)."""

    def __init__(self, cfg: Optional[MissionConfig] = None,
                 *, deliver_enabled: bool = True):
        self.cfg = cfg or MissionConfig()
        self.cfg.validate()
        self.deliver_enabled = bool(deliver_enabled)
        self.state = State.BOOT
        self.entered_at = 0.0
        self.attempts = 0      # grasp attempts started (reached LATCH)
        self.failures = 0      # failed tries, including ones that never got there
        self.last_reason = ""
        self.pause_reason = ""
        self.paused_from: Optional[State] = None
        # (state, action) to re-enter with, from HELD_OBJECT_RESUME.
        self.resume_after_self_check: Optional[Tuple[State, Action]] = None
        self.history: list = []

    # ── helpers ──
    def _elapsed(self, now: float) -> float:
        return now - self.entered_at

    def _go(self, state: State, action: Action, reason: str, now: float,
            *, reset_nav: bool = False) -> Transition:
        changed = state is not self.state
        if changed:
            self.history.append((self.state, state, reason))
            self.state = state
            self.entered_at = now
        self.last_reason = reason
        return Transition(state, action, reason, reset_nav=reset_nav, changed=changed)

    def _pause(self, reason: str, now: float) -> Transition:
        if self.state is not State.PAUSED:
            self.paused_from = self.state
        self.pause_reason = reason
        return self._go(State.PAUSED, Action.STOP, reason, now, reset_nav=True)

    def _fault(self, reason: str, now: float) -> Transition:
        return self._go(State.FAULT, Action.STOP, reason, now, reset_nav=True)

    def _abandon_target(self, reason: str, now: float) -> Transition:
        """Give up on the current object and go back to the route."""
        self.attempts = self.failures = 0
        return self._go(State.RESUME, Action.RESUME_PATROL, reason, now, reset_nav=True)

    # ── the machine ──
    def step(self, s: Sense) -> Transition:
        now = s.now
        cfg = self.cfg
        st = self.state

        # ── pre-emptive safety, checked before any state logic ──
        if s.health.estop:
            # ESTOP latches. Section 8: no automatic recovery, and the arm is
            # NOT moved back to home -- whatever it is holding stays held.
            return self._go(State.ESTOP, Action.STOP, "ESTOP asserted", now,
                            reset_nav=True)
        if st is State.ESTOP:
            return self._go(State.ESTOP, Action.STOP, "ESTOP latched — manual reset only",
                            now)
        if s.health.fault:
            return self._fault(f"fault: {s.health.fault}", now)
        if st is State.FAULT:
            return self._go(State.FAULT, Action.STOP, "FAULT latched — manual reset only",
                            now)

        blocking = s.health.blocking_reason(st)
        if blocking:
            return self._pause(f"{st.value}: {blocking}", now)

        # ── states ──
        if st is State.BOOT:
            return self._go(State.SELF_CHECK, Action.RUN_SELF_CHECK,
                            "boot complete", now, reset_nav=True)

        if st is State.SELF_CHECK:
            if s.self_check_passed:
                resume = self.resume_after_self_check
                self.resume_after_self_check = None
                if resume is not None:
                    state, action = resume
                    return self._go(
                        state, action,
                        "self-check passed -- resuming held-object mission",
                        now, reset_nav=True)
                return self._go(State.IDLE, Action.STOP, "self-check passed", now,
                                reset_nav=True)
            if self._elapsed(now) > cfg.self_check_timeout_s:
                return self._fault(
                    f"self-check did not pass within {cfg.self_check_timeout_s:.0f}s",
                    now)
            return self._go(State.SELF_CHECK, Action.RUN_SELF_CHECK,
                            "waiting for self-check", now)

        if st is State.IDLE:
            if s.started:
                return self._go(State.PATROL, Action.DRIVE_PATROL,
                                "operator start", now, reset_nav=True)
            return self._go(State.IDLE, Action.WAIT_OPERATOR, "idle", now)

        if st is State.PATROL:
            if s.detection_streak >= cfg.detection_streak_needed:
                return self._go(
                    State.INVESTIGATE, Action.TURN_TO_TARGET,
                    f"trash seen on {s.detection_streak} consecutive frames", now,
                    reset_nav=True)   # goal source changes: map -> camera
            if s.waypoint_reached:
                return self._go(State.PATROL, Action.RESUME_PATROL,
                                "waypoint reached — advancing", now)
            return self._go(State.PATROL, Action.DRIVE_PATROL, "patrolling", now)

        if st is State.INVESTIGATE:
            if not s.target_visible and s.target_fix_age > cfg.target_lost_timeout_s:
                return self._abandon_target("target lost during investigate", now)
            if self._elapsed(now) > cfg.investigate_timeout_s:
                return self._abandon_target("investigate timed out", now)
            if s.target_visible and s.target_dist <= cfg.investigate_to_approach_m:
                return self._go(State.APPROACH, Action.DRIVE_TARGET,
                                f"target confirmed at {s.target_dist:.2f} m", now)
            return self._go(State.INVESTIGATE, Action.TURN_TO_TARGET,
                            "confirming target bearing", now)

        if st is State.APPROACH:
            if s.approach_failed:
                return self._retry_or_give_up("rear-camera final approach failed", now)
            if s.target_fix_age > cfg.target_lost_timeout_s:
                return self._abandon_target(
                    f"target lost for {s.target_fix_age:.1f}s during approach", now)
            if self._elapsed(now) > cfg.approach_timeout_s:
                return self._abandon_target("approach timed out", now)
            if s.target_dist <= cfg.approach_to_align_m and s.approach_ready:
                # Stop first.  The ALIGN state confirms wheel feedback is
                # stationary on a later tick before raising the arm to C3.
                return self._go(State.ALIGN, Action.SETTLE,
                                f"within {cfg.approach_to_align_m:.2f} m — fine align",
                                now, reset_nav=True)
            if s.target_dist <= cfg.approach_to_align_m:
                return self._go(State.APPROACH, Action.DRIVE_TARGET,
                                "finishing rear-camera centering", now)
            return self._go(State.APPROACH, Action.DRIVE_TARGET, "approaching", now)

        if st is State.ALIGN:
            # Deliberately no target_fix_age check here, unlike INVESTIGATE and
            # APPROACH. The visual servo runs as one blocking call and does its
            # own detection; the shared GoalTracker is NOT updated while it runs,
            # so the fix age climbs on its own and a "target lost" test would
            # fire on every alignment. Losing the object is reported through
            # align_failed instead, and align_timeout_s is the backstop.
            if s.align_failed:
                return self._retry_or_give_up("fine align failed", now)
            if self._elapsed(now) > cfg.align_timeout_s:
                return self._retry_or_give_up("fine align timed out", now)
            if not s.stationary:
                return self._go(
                    State.ALIGN, Action.SETTLE,
                    "waiting for the base to stop before raising the arm", now)
            if s.handoff_ready:
                return self._go(State.STATIONARY_GATE, Action.SETTLE,
                                "object inside the trained envelope", now)
            return self._go(State.ALIGN, Action.FINE_ALIGN, "fine aligning", now)

        if st is State.STATIONARY_GATE:
            # Section 6: the arm must not start while the base is still rolling.
            # A timeout here is a FAULT, not a retry -- we cannot confirm the
            # base stopped, so we must not raise the arm at all.
            if s.stationary:
                return self._go(State.LATCH, Action.LATCH, "base confirmed stationary",
                                now)
            if self._elapsed(now) > cfg.stationary_gate_timeout_s:
                return self._fault(
                    "base never confirmed stationary — refusing to move the arm", now)
            return self._go(State.STATIONARY_GATE, Action.SETTLE,
                            "waiting for the base to stop", now)

        if st is State.LATCH:
            if not s.stationary:
                return self._fault("base moved during latch", now)
            if s.latched:
                self.attempts += 1
                return self._go(State.GRASP, Action.RUN_GRASP,
                                f"target latched (attempt {self.attempts})", now)
            if self._elapsed(now) > cfg.stationary_gate_timeout_s:
                return self._retry_or_give_up("could not latch a valid target", now)
            return self._go(State.LATCH, Action.LATCH, "latching", now)

        if st is State.GRASP:
            if self._elapsed(now) > cfg.grasp_timeout_s:
                return self._fault("grasp policy exceeded its time budget", now)
            if s.grasp_finished:
                return self._go(State.VERIFY, Action.VERIFY, "grasp sequence finished",
                                now)
            return self._go(State.GRASP, Action.RUN_GRASP, "grasping", now)

        if st is State.VERIFY:
            if s.grasp_verified:
                return self._go(State.CARRY_HOME, Action.STOP,
                                "grasp verified — object lifted", now)
            return self._retry_or_give_up("grasp not verified", now)

        if st is State.RETRY:
            if self._elapsed(now) > cfg.align_timeout_s:
                return self._abandon_target("retry back-off timed out", now)
            if s.arm_at_home and s.stationary:
                return self._go(State.APPROACH, Action.DRIVE_TARGET,
                                f"retrying (failure {self.failures}/"
                                f"{cfg.max_grasp_attempts})", now, reset_nav=True)
            return self._go(State.RETRY, Action.BACK_OFF, "backing off for a retry", now)

        if st is State.CARRY_HOME:
            if self._elapsed(now) > cfg.carry_home_timeout_s:
                return self._pause("arm did not reach the carry pose", now)
            if s.arm_at_home:
                if not self.deliver_enabled:
                    self.attempts = self.failures = 0
                    return self._go(
                        State.COMPLETE, Action.HOLD,
                        "grasp verified — delivery disabled; holding", now,
                        reset_nav=True)
                return self._go(State.DELIVER, Action.DRIVE_BIN,
                                "carrying — heading to the bin", now, reset_nav=True)
            return self._go(State.CARRY_HOME, Action.STOP, "moving to the carry pose",
                            now)

        if st is State.DELIVER:
            if self._elapsed(now) > cfg.deliver_timeout_s:
                return self._pause("delivery timed out — still carrying", now)
            if s.bin_reached:
                return self._go(State.PLACE_ALIGN, Action.SETTLE,
                                "bin approach point reached", now)
            return self._go(State.DELIVER, Action.DRIVE_BIN, "driving to the bin", now)

        if st is State.PLACE_ALIGN:
            if self._elapsed(now) > cfg.place_align_timeout_s:
                return self._pause("could not settle at the bin", now)
            if s.stationary:
                return self._go(State.PLACE, Action.PLACE, "aligned at the bin", now)
            return self._go(State.PLACE_ALIGN, Action.SETTLE, "settling at the bin", now)

        if st is State.PLACE:
            if self._elapsed(now) > cfg.place_timeout_s:
                return self._pause("place sequence timed out", now)
            if s.place_finished and s.arm_at_home:
                self.attempts = self.failures = 0
                return self._go(State.RESUME, Action.RESUME_PATROL,
                                "object placed — arm home", now, reset_nav=True)
            return self._go(State.PLACE, Action.PLACE, "placing", now)

        if st is State.RESUME:
            return self._go(State.PATROL, Action.DRIVE_PATROL,
                            "resuming patrol at the next waypoint", now, reset_nav=True)

        if st is State.COMPLETE:
            return self._go(State.COMPLETE, Action.HOLD,
                            "mission complete — holding", now)

        if st is State.PAUSED:
            if s.operator_cleared:
                # A miss here means the pause happened somewhere the mission is
                # not carrying anything, so IDLE is the right place to land.
                self.resume_after_self_check = HELD_OBJECT_RESUME.get(
                    self.paused_from)
                self.paused_from = None
                return self._go(State.SELF_CHECK, Action.RUN_SELF_CHECK,
                                "operator cleared the pause", now, reset_nav=True)
            return self._go(State.PAUSED, Action.STOP,
                            f"paused: {self.pause_reason}", now)

        return self._fault(f"unhandled state {st!r}", now)

    def _retry_or_give_up(self, reason: str, now: float) -> Transition:
        """Count the FAILURE, not the attempt.

        Counting attempts only worked for failures that got as far as LATCH. An
        object the visual servo can never align on -- out of the trained
        envelope, too wide, simply not graspable -- never reaches LATCH, so the
        counter stayed at zero and APPROACH/ALIGN/RETRY cycled forever. Every
        path into here is a failed try, so this is where the budget belongs.
        """
        self.failures += 1
        if self.failures >= self.cfg.max_grasp_attempts:
            return self._abandon_target(
                f"{reason} — {self.failures} attempts exhausted, blacklisting", now)
        return self._go(State.RETRY, Action.BACK_OFF, reason, now, reset_nav=True)


# ════════════════════════════════════════════════════════════════════════════
# Selftest
# ════════════════════════════════════════════════════════════════════════════

def _drive(fsm: MissionFSM, sense_kw, *, ticks: int = 1, t0: float = 0.0,
           dt: float = 0.1) -> Transition:
    tr = None
    for i in range(ticks):
        tr = fsm.step(Sense(now=t0 + i * dt, **sense_kw))
    return tr


def _to_patrol(fsm: MissionFSM, t: float = 0.0) -> float:
    fsm.step(Sense(now=t))                                    # BOOT -> SELF_CHECK
    fsm.step(Sense(now=t + 0.1, self_check_passed=True))      # -> IDLE
    fsm.step(Sense(now=t + 0.2, started=True))                # -> PATROL
    assert fsm.state is State.PATROL
    return t + 0.3


def run_selftest() -> None:
    cfg = MissionConfig()

    # ── happy path: patrol all the way to resumed patrol ──
    fsm = MissionFSM(cfg)
    t = _to_patrol(fsm)

    tr = fsm.step(Sense(now=t, detection_streak=3))
    assert fsm.state is State.INVESTIGATE and tr.reset_nav, tr
    t += 0.1
    tr = fsm.step(Sense(now=t, target_visible=True, target_dist=2.0, target_fix_age=0.0))
    assert fsm.state is State.APPROACH, tr
    t += 0.1
    fsm.step(Sense(now=t, target_visible=True, target_dist=1.5, target_fix_age=0.0))
    assert fsm.state is State.APPROACH
    t += 0.1
    tr = fsm.step(Sense(now=t, target_visible=True, target_dist=0.6, target_fix_age=0.0))
    assert fsm.state is State.ALIGN and tr.reset_nav, tr
    t += 0.1
    tr = fsm.step(Sense(now=t, handoff_ready=True, stationary=True))
    assert fsm.state is State.STATIONARY_GATE, tr
    t += 0.1
    fsm.step(Sense(now=t, stationary=True))
    assert fsm.state is State.LATCH
    t += 0.1
    fsm.step(Sense(now=t, stationary=True, latched=True))
    assert fsm.state is State.GRASP and fsm.attempts == 1
    t += 0.1
    fsm.step(Sense(now=t, grasp_finished=True))
    assert fsm.state is State.VERIFY
    t += 0.1
    fsm.step(Sense(now=t, grasp_verified=True))
    assert fsm.state is State.CARRY_HOME
    t += 0.1
    fsm.step(Sense(now=t, arm_at_home=True))
    assert fsm.state is State.DELIVER
    t += 0.1
    fsm.step(Sense(now=t, bin_reached=True))
    assert fsm.state is State.PLACE_ALIGN
    t += 0.1
    fsm.step(Sense(now=t, stationary=True))
    assert fsm.state is State.PLACE
    t += 0.1
    tr = fsm.step(Sense(now=t, place_finished=True, arm_at_home=True))
    assert fsm.state is State.RESUME and fsm.attempts == 0, tr
    t += 0.1
    tr = fsm.step(Sense(now=t))
    assert fsm.state is State.PATROL and tr.reset_nav
    print("[fsm] full mission happy path OK (patrol -> grasp -> bin -> patrol)")

    # ── the two hardware invariants ──
    for state in State:
        drive = state in DRIVING_STATES
        arm = state in ARM_STATES
        assert not (drive and arm), f"{state} allows wheels AND arm at once"
    tr = Transition(State.GRASP, Action.RUN_GRASP)
    assert tr.arm_allowed and not tr.chassis_allowed
    tr = Transition(State.PATROL, Action.DRIVE_PATROL)
    assert tr.chassis_allowed and not tr.arm_allowed
    for state in (State.STATIONARY_GATE, State.LATCH, State.VERIFY,
                  State.PAUSED, State.FAULT, State.ESTOP, State.IDLE):
        tr = Transition(state, Action.STOP)
        assert not tr.chassis_allowed, f"{state} must not drive"
    print("[fsm] wheels/arm mutual exclusion OK")

    # ── retries: three attempts, then blacklist and resume patrol ──
    fsm = MissionFSM(cfg)
    t = _to_patrol(fsm)
    for attempt in range(1, cfg.max_grasp_attempts + 1):
        fsm.state, fsm.entered_at = State.LATCH, t
        fsm.step(Sense(now=t, stationary=True, latched=True))
        assert fsm.state is State.GRASP and fsm.attempts == attempt
        t += 0.1
        fsm.step(Sense(now=t, grasp_finished=True))
        t += 0.1
        tr = fsm.step(Sense(now=t, grasp_verified=False))
        if attempt < cfg.max_grasp_attempts:
            assert fsm.state is State.RETRY, (attempt, tr)
            t += 0.1
            fsm.step(Sense(now=t, arm_at_home=True, stationary=True))
            assert fsm.state is State.APPROACH
            t += 0.1
        else:
            assert fsm.state is State.RESUME and "blacklisting" in tr.reason, tr
            assert fsm.attempts == 0, "attempt counter must reset after giving up"
    print(f"[fsm] {cfg.max_grasp_attempts}-attempt retry then blacklist OK")

    # ── stale sensors while driving -> PAUSED, and PAUSED does not self-clear ──
    for field, state, needle in (
        ("scan_fresh", State.PATROL, "/scan stale"),
        ("odom_fresh", State.PATROL, "feedback stale"),
        ("odom_valid", State.PATROL, "feedback invalid"),
        ("amcl_ok", State.PATROL, "AMCL"),
        ("serial_ok", State.GRASP, "serial"),
        ("scan_fresh", State.APPROACH, "/scan stale"),
    ):
        fsm = MissionFSM(cfg)
        t = _to_patrol(fsm)
        fsm.state, fsm.entered_at = state, t
        tr = fsm.step(Sense(now=t, health=Health(**{field: False})))
        assert fsm.state is State.PAUSED, (field, state, tr)
        assert needle in tr.reason, (field, tr.reason)
        assert tr.action is Action.STOP and not tr.chassis_allowed
        # health restored on its own must NOT resume motion
        tr = fsm.step(Sense(now=t + 1.0))
        assert fsm.state is State.PAUSED, "PAUSED must not auto-resume"
        tr = fsm.step(Sense(now=t + 2.0, operator_cleared=True))
        assert fsm.state is State.SELF_CHECK, tr
    print("[fsm] stale-sensor pause + no auto-resume OK")

    # ── AMCL is required for patrol but must NOT abort an in-flight grasp ──
    fsm = MissionFSM(cfg)
    t = _to_patrol(fsm)
    fsm.state, fsm.entered_at = State.GRASP, t
    tr = fsm.step(Sense(now=t, health=Health(amcl_ok=False)))
    assert fsm.state is State.GRASP, "a wobbling AMCL must not abort a grasp"
    fsm.state, fsm.entered_at = State.ALIGN, t
    tr = fsm.step(Sense(now=t, health=Health(amcl_ok=False)))
    assert fsm.state is State.ALIGN, "ALIGN is robot-frame; AMCL is irrelevant"
    print("[fsm] AMCL scoped to map states only OK")

    # ── ESTOP latches from anywhere and cannot be cleared by the machine ──
    for state in (State.PATROL, State.GRASP, State.PLACE, State.PAUSED):
        fsm = MissionFSM(cfg)
        t = _to_patrol(fsm)
        fsm.state, fsm.entered_at = state, t
        tr = fsm.step(Sense(now=t, health=Health(estop=True)))
        assert fsm.state is State.ESTOP and tr.terminal, (state, tr)
        assert not tr.chassis_allowed and not tr.arm_allowed
        for extra in range(1, 4):
            tr = fsm.step(Sense(now=t + extra, operator_cleared=True, started=True))
            assert fsm.state is State.ESTOP, "ESTOP must latch"
    print("[fsm] ESTOP latches everywhere OK")

    # ── a fault latches too ──
    fsm = MissionFSM(cfg)
    t = _to_patrol(fsm)
    fsm.step(Sense(now=t, health=Health(fault="servo write refused")))
    assert fsm.state is State.FAULT
    tr = fsm.step(Sense(now=t + 1, operator_cleared=True))
    assert fsm.state is State.FAULT and tr.terminal
    print("[fsm] FAULT latches OK")

    # ── the stationary gate never lets the arm start on an unconfirmed base ──
    fsm = MissionFSM(cfg)
    t = _to_patrol(fsm)
    fsm.state, fsm.entered_at = State.STATIONARY_GATE, t
    for i in range(1, 10):
        tr = fsm.step(Sense(now=t + i * 1.0, stationary=False))
        if fsm.state is State.FAULT:
            break
    assert fsm.state is State.FAULT and "never confirmed stationary" in tr.reason, tr
    print("[fsm] stationary gate fails closed OK")

    # ── target lost / timeouts abandon the object and go back to the route ──
    for state, kw, needle in (
        (State.INVESTIGATE, dict(target_fix_age=99.0), "lost"),
        (State.APPROACH, dict(target_fix_age=99.0), "lost"),
    ):
        fsm = MissionFSM(cfg)
        t = _to_patrol(fsm)
        fsm.state, fsm.entered_at = state, t
        tr = fsm.step(Sense(now=t, **kw))
        assert fsm.state is State.RESUME and needle in tr.reason, (state, tr)
        assert tr.reset_nav
    fsm = MissionFSM(cfg)
    t = _to_patrol(fsm)
    fsm.state, fsm.entered_at = State.APPROACH, t
    tr = fsm.step(Sense(now=t + cfg.approach_timeout_s + 1,
                        target_visible=True, target_dist=1.0, target_fix_age=0.0))
    assert fsm.state is State.RESUME and "timed out" in tr.reason, tr
    print("[fsm] target-lost and approach-timeout recovery OK")

    # ── one-frame detections must not abandon the patrol route ──
    fsm = MissionFSM(cfg)
    t = _to_patrol(fsm)
    for streak in range(cfg.detection_streak_needed):
        fsm.step(Sense(now=t + streak * 0.1, detection_streak=streak))
        assert fsm.state is State.PATROL, f"streak {streak} should not trigger"
    fsm.step(Sense(now=t + 1.0, detection_streak=cfg.detection_streak_needed))
    assert fsm.state is State.INVESTIGATE
    print("[fsm] detection streak gate OK")

    # ── every goal-source change resets the nav filters ──
    fsm = MissionFSM(cfg)
    t = _to_patrol(fsm)
    tr = fsm.step(Sense(now=t, detection_streak=3))          # map -> camera
    assert tr.reset_nav, "PATROL->INVESTIGATE changes goal source"
    fsm.state, fsm.entered_at = State.CARRY_HOME, t
    tr = fsm.step(Sense(now=t, arm_at_home=True))            # camera -> map
    assert fsm.state is State.DELIVER and tr.reset_nav
    fsm.state, fsm.entered_at = State.PLACE, t
    tr = fsm.step(Sense(now=t, place_finished=True, arm_at_home=True))
    assert fsm.state is State.RESUME and tr.reset_nav
    print("[fsm] goal-source changes reset ActionDelay/tracker OK")

    # ── deliver disabled: stop after the verified grasp and hold ──
    fsm = MissionFSM(cfg, deliver_enabled=False)
    t = _to_patrol(fsm)
    fsm.state, fsm.entered_at = State.CARRY_HOME, t
    tr = fsm.step(Sense(now=t, arm_at_home=True))
    assert fsm.state is State.COMPLETE and tr.completed, tr
    print("[fsm] deliver_enabled=False path OK")

    # ── pausing while carrying resumes the mission, not IDLE ──
    # Every carrying state must come back to the held-object flow. Landing in
    # IDLE would strand an object in the jaw and start a fresh patrol with it.
    for paused_in, want_state in (
        (State.CARRY_HOME, State.CARRY_HOME),
        (State.DELIVER, State.DELIVER),
        (State.PLACE_ALIGN, State.PLACE_ALIGN),
        (State.PLACE, State.PLACE_ALIGN),      # re-enter via the stationary gate
    ):
        fsm = MissionFSM(cfg)
        t = _to_patrol(fsm)
        fsm.state, fsm.entered_at = paused_in, t
        fsm.step(Sense(now=t, health=Health(serial_ok=False)))
        assert fsm.state is State.PAUSED, paused_in
        fsm.step(Sense(now=t + 1, operator_cleared=True))
        assert fsm.state is State.SELF_CHECK, paused_in
        tr = fsm.step(Sense(now=t + 2, self_check_passed=True))
        assert fsm.state is want_state, (paused_in, fsm.state)
        assert "held-object" in tr.reason, tr
    # ...and a pause anywhere else still lands in IDLE
    fsm = MissionFSM(cfg)
    t = _to_patrol(fsm)
    fsm.step(Sense(now=t, health=Health(scan_fresh=False)))
    assert fsm.state is State.PAUSED
    fsm.step(Sense(now=t + 1, operator_cleared=True))
    tr = fsm.step(Sense(now=t + 2, self_check_passed=True))
    assert fsm.state is State.IDLE, tr
    # the resume table is the single source of truth for both sites
    for state, (dst, action) in HELD_OBJECT_RESUME.items():
        assert isinstance(dst, State) and isinstance(action, Action), state
        assert dst in ARM_STATES or dst in DRIVING_STATES, (state, dst)
    print(f"[fsm] paused-while-carrying resumes all {len(HELD_OBJECT_RESUME)} "
          f"carrying states, others land in IDLE OK")

    # ── config validation ──
    for bad in (dict(detection_streak_needed=0), dict(max_grasp_attempts=0),
                dict(approach_to_align_m=3.0), dict(align_timeout_s=0.0)):
        try:
            MissionConfig(**bad).validate()
        except ValueError:
            continue
        raise AssertionError(f"config {bad} should have been rejected")
    print("[fsm] config validation OK")

    print("[fsm] SELFTEST PASSED")


def print_diagram() -> None:
    print("""
  BOOT -> SELF_CHECK -> IDLE --(operator start)--> PATROL
                                                     |
        +--------------------------------------------+
        |  detection_streak >= N  (goal: map -> camera, reset nav)
        v
   INVESTIGATE --dist<=2.5m--> APPROACH --dist<=0.75m--> ALIGN
        |  lost/timeout            |  lost/timeout          | handoff_ready
        +--------> RESUME <--------+                        v
                     ^                            STATIONARY_GATE
                     |                                      | stationary
                     |                                      v
                     |                                    LATCH
                     |                                      v
                     |                                    GRASP
                     |                                      v
                     |                                   VERIFY
                     |                          fail        |  ok
                     |                     +----------------+
                     |                     v                v
                     |                   RETRY          CARRY_HOME --no-deliver--> COMPLETE
                     |               (<=3, else give up)     v
                     |                     |             DELIVER
                     |                     v                v
                     |                 APPROACH         PLACE_ALIGN
                     |                                      v
                     |                                    PLACE
                     +--------------------------------------+

  any state: sensor stale -> PAUSED (human only)   ESTOP/fault -> latched
""")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--diagram", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        run_selftest()
        return 0
    if args.diagram:
        print_diagram()
        return 0
    ap.print_help()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
