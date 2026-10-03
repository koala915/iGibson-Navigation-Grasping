#!/usr/bin/env python3
"""Pure-logic checks for the travel pose <-> E1 commands of the grasp service.

A fake controller records every guarded move, so the checks can pin the path
(through the waypoint validate_nav_to_grasp_transition.py found safe), what the
jaw is told on the way, and which commands refuse to start from where.
"""

from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import grasp_service as gs  # noqa: E402

checks = 0
failures = []

E1 = (90.0, 74.2, 8.6, 8.6, 90.0, 30.0)
TRAVEL = gs.TRAVEL_DEG
WAY = (90.0, gs.WAYPOINT_S2, 8.6, 8.6, 90.0)
HOLD = 0.7          # a hold command in sim radians; anything not the open value


def ok(condition, label):
    global checks
    checks += 1
    print("  [{}] {}".format("ok" if condition else "FAIL", label))
    if not condition:
        failures.append(label)


class Mapper:
    """Identity-ish: arm degrees pass through, the jaw maps 30 deg <-> -1.5 rad."""

    def hw_deg_to_sim_arm(self, deg):
        return list(deg[:5])

    def hw_deg_to_sim_grip(self, deg):
        return -1.5 if abs(deg - 30.0) < 1e-6 else deg / 100.0


class Servo:
    def __init__(self, deg):
        self.deg = list(deg)

    def read_degrees(self):
        return type("R", (), {"valid": True, "degrees": list(self.deg)})()


class Ctl:
    def __init__(self, at, holding=False, fail_leg=None):
        self.cfg = type("C", (), {"home_deg": E1, "gripper_hw_open": 30.0,
                                  "bin_rim_height": 0.05, "release_clearance": 0.03})()
        self.mapper = Mapper()
        self.servo = Servo(at)
        self.detection = None
        self._grip_hold_rad = HOLD if holding else None
        self.moves = []
        self.released = 0
        self.fail_leg = fail_leg

    def move_guarded_and_verified(self, arm, grip, *, label, grip_is_hold=False, **_kw):
        self.moves.append((tuple(round(a, 3) for a in arm), grip, label, grip_is_hold))
        if self.fail_leg is not None and len(self.moves) - 1 == self.fail_leg:
            return {"reached": False, "reason": "floor guard emergency_hold"}
        self.servo.deg = list(arm) + [self.servo.deg[5]]
        return {"reached": True, "reason": "arrived", "iters": 3}

    def run_release_only(self):
        self.released += 1
        return "released"


def svc(ctl, episode=None):
    return gs.GraspService(ctl, episode or (lambda c, max_steps: True), 10, "/tmp/unused.sock")


def arms(ctl):
    return [m[0] for m in ctl.moves]


def main():
    print("== stow: E1 -> travel through the waypoint ==")
    c = Ctl(E1)
    r = svc(c).cmd_stow()
    ok(r["ok"] and arms(c) == [WAY, TRAVEL[:5]], "two legs: waypoint (S2 %.0f, S3/S4 at E1), then travel"
       % gs.WAYPOINT_S2)
    ok(all(m[1] == -1.5 and not m[3] for m in c.moves), "an empty jaw is kept open")

    c = Ctl(E1, holding=True)
    svc(c).cmd_stow()
    ok(all(m[1] == HOLD and m[3] for m in c.moves),
       "a held object keeps the hold command on every leg, judged as a hold")

    c = Ctl(TRAVEL)
    r = svc(c).cmd_stow()
    ok(r.get("already") and c.moves == [], "already in the travel pose: nothing moves")

    c = Ctl((90.0, 60.0, 30.0, 20.0, 90.0, 30.0))
    r = svc(c).cmd_stow()
    ok(r["reason"] == "arm_not_at_e1" and c.moves == [],
       "from anywhere else it refuses: that path was never checked")

    c = Ctl(E1, fail_leg=0)
    r = svc(c).cmd_stow()
    ok(not r["ok"] and r["leg"] == 0 and len(c.moves) == 1,
       "a leg that does not arrive stops the sequence there")

    print("\n== home from travel ==")
    c = Ctl(TRAVEL, holding=True)
    r = svc(c).cmd_home()
    ok(arms(c)[:2] == [WAY, E1[:5]] and c.moves[0][3] and c.moves[1][3],
       "comes back through the waypoint still holding")
    ok(len(c.moves) == 3 and c.moves[2][2] == "service-home",
       "and only then opens the jaw at E1 with the usual home move")
    ok(r["ok"] and c._grip_hold_rad is None, "the hold is cleared after home")

    c = Ctl((90.0, 60.0, 30.0, 20.0, 90.0, 30.0))
    svc(c).cmd_home()
    ok([m[2] for m in c.moves] == ["service-home"],
       "from any other pose home is the old single guarded move, unchanged")

    print("\n== release from travel ==")
    c = Ctl(TRAVEL, holding=True)
    r = svc(c).cmd_release()
    ok(arms(c) == [WAY, E1[:5]] and all(m[3] for m in c.moves) and c.released == 1,
       "carries the object back to E1 through the waypoint, then runs the checked release")
    ok(r["ok"] and r["outcome"] == "released", "and reports the release outcome")

    c = Ctl(TRAVEL, holding=True, fail_leg=1)
    r = svc(c).cmd_release()
    ok(not r["ok"] and c.released == 0, "if it cannot get back to E1 it does not release")

    c = Ctl((90.0, 60.0, 30.0, 20.0, 90.0, 30.0), holding=True)
    r = svc(c).cmd_release()
    ok(r["reason"] == "arm_not_at_e1_or_travel" and c.released == 0 and c.moves == [],
       "from an unchecked pose release refuses without moving")

    c = Ctl(E1, holding=True)
    svc(c).cmd_release()
    ok(c.moves == [] and c.released == 1, "from E1 release is exactly what it was")

    print("\n== grasp only from E1 ==")
    ran = []
    c = Ctl(TRAVEL)
    r = svc(c, lambda ctl, max_steps: ran.append(1) or True).cmd_grasp()
    ok(r["reason"] == "arm_not_at_e1" and ran == [] and c.moves == [],
       "from the travel pose grasp refuses: a frame shot on the way up would pass the latch")
    c = Ctl(E1)
    r = svc(c, lambda ctl, max_steps: ran.append(1) or True).cmd_grasp()
    ok(r.get("confirmed") is True and ran == [1], "from E1 it grasps as before")

    print("\n== every arm command starts from the encoders ==")
    # The robot, 2026-09-28: after a restart the rate limiter still counted from
    # the default home (E1) while the arm sat in travel; home went out as one jump.
    c = Ctl(TRAVEL)
    c.servo._last_deg = list(E1)                    # what a fresh start leaves behind
    seen = {}
    real_move = c.move_guarded_and_verified

    def spy(arm, grip, **kw):
        seen.setdefault("first", list(c.servo._last_deg))
        return real_move(arm, grip, **kw)

    c.move_guarded_and_verified = spy
    s = svc(c)
    s.run_arm_command(s.cmd_home)
    ok(seen["first"][:5] == list(TRAVEL[:5]),
       "before the first move the rate limiter counts from the travel pose, not E1")
    ok(seen["first"][5] == E1[5], "the jaw's last command is left as it was")

    class DeadServo:
        _last_deg = list(E1)

        def read_degrees(self):
            return type("R", (), {"valid": False, "degrees": None, "reason": "bus"})()

    c = Ctl(TRAVEL)
    c.servo = DeadServo()
    r = svc(c).run_arm_command(svc(c).cmd_home)
    ok(r["reason"] == "servo_read_failed" and c.moves == [],
       "unreadable encoders: the arm command refuses without moving")

    print("\n== the chassis learns the arm pose after every arm command ==")
    import chassis_server as cs
    board = type("B", (), {"calls": [], "set_motor": lambda self, *v: self.calls.append(v)})()
    ch = cs.Chassis(board, lambda vx, wz: (20, 20, 20, 20) if vx else (0, 0, 0, 0),
                    log=lambda _m: None)
    fake_srv = type("S", (), {"chassis": ch})()
    c = Ctl(E1)
    s = gs.GraspService(c, lambda ctl, max_steps: True, 10, "/tmp/unused.sock",
                        chassis_server=fake_srv)
    s.run_arm_command(s.cmd_stow)
    ok(ch.status()["arm_pose"] == "travel", "after stow the chassis knows the arm is in travel")
    s.run_arm_command(s.cmd_home)
    ok(ch.status()["arm_pose"] == "e1", "after home it knows the arm is at E1")
    c.servo.deg = [90.0, 60.0, 30.0, 20.0, 90.0, 30.0]
    s.run_arm_command(lambda: {"ok": False})
    ok(ch.status()["arm_pose"] == "other", "an arm command that ends anywhere else is 'other'")

    print("\n== grasp waits for a frame shot after everything stopped ==")

    class FakeTime:
        def __init__(self):
            self.t, self.slept = 1000.0, 0.0

        def monotonic(self):
            return self.t

        def time(self):
            return self.t

        def sleep(self, dt):
            self.slept += dt
            self.t += dt

    class Recv:
        """Detections arriving at the given monotonic times."""

        def __init__(self, clock, arrivals):
            self.clock, self.arrivals = clock, sorted(arrivals)

        @property
        def _last_update_ts(self):
            past = [a for a in self.arrivals if a <= self.clock.t]
            return past[-1] if past else 0.0

        def snapshot(self):
            last = self._last_update_ts
            fresh = last > 0 and self.clock.t - last <= 1.0
            return [0.27, -0.02, 0.03], 0.065, fresh, None

    real_time = gs.time
    try:
        ft = gs.time = FakeTime()
        ran = []
        c = Ctl(E1)
        s = svc(c, lambda ctl, max_steps: ran.append(ft.t) or True)
        t0 = ft.t                                   # the arm has just stopped
        c.detection = Recv(ft, [t0 - 0.2 + k for k in range(10)])
        s.cmd_grasp()
        ok(abs(ft.slept - gs.GRASP_SETTLE_S) < 0.25 and ran and ran[0] >= t0 + gs.GRASP_SETTLE_S,
           "right after the arm stops, grasp waits %.1f s before it starts" % gs.GRASP_SETTLE_S)
        ok(c.detection._last_update_ts >= t0 + gs.FRAME_LAG_S,
           "and the detection it starts on arrived at least %.1f s after the stop" % gs.FRAME_LAG_S)

        ft = gs.time = FakeTime()
        ran = []
        c = Ctl(E1)
        s = svc(c, lambda ctl, max_steps: ran.append(1) or True)
        t0 = ft.t
        c.detection = Recv(ft, [t0 - 0.2, t0 + 0.8])      # then the camera goes quiet
        r = s.cmd_grasp()
        ok(r.get("reason") == "no_fresh_e1_detection" and ran == [],
           "with no detection after the stop it refuses, arm unmoved")

        saved = gs.GRASP_SETTLE_S
        gs.GRASP_SETTLE_S = 1.0                     # so a too-early one is still fresh
        ft = gs.time = FakeTime()
        ran = []
        c = Ctl(E1)
        s = svc(c, lambda ctl, max_steps: ran.append(ft.t) or True)
        t0 = ft.t
        c.detection = Recv(ft, [t0 + 0.5, t0 + 1.6])
        s.cmd_grasp()
        gs.GRASP_SETTLE_S = saved
        ok(ran and ran[0] >= t0 + 1.6,
           "a fresh detection from before the lag is skipped for the next one")

        ft = gs.time = FakeTime()
        ran = []
        board = type("B", (), {"calls": [], "set_motor": lambda self, *v: self.calls.append(v)})()
        ch = cs.Chassis(board, lambda vx, wz: (20, 20, 20, 20) if vx else (0, 0, 0, 0),
                        clock=ft.monotonic, log=lambda _m: None)
        ch.set_arm_pose("e1")
        c = Ctl(E1)
        s = gs.GraspService(c, lambda ctl, max_steps: ran.append(ft.t) or True, 10,
                            "/tmp/unused.sock", chassis_server=type("S", (), {"chassis": ch})())
        ft.t += 60.0                                # the arm has been still for a minute
        ch.command({"action": "velocity", "vx": 0.05, "wz": 0.0})
        ft.t += 0.3
        ch.command({"action": "stop"})              # a creep at E1 just ended
        t1 = ft.t
        ft.t += 0.6
        c.detection = Recv(ft, [t1 - 0.5 + k for k in range(10)])
        s.run_arm_command(s.cmd_grasp)
        ok(ran and ran[0] >= t1 + gs.GRASP_SETTLE_S,
           "a creep of the wheels counts too: the wait runs from when they stopped")
    finally:
        gs.time = real_time

    print()
    if failures:
        print("{} of {} checks FAILED:".format(len(failures), checks))
        for label in failures:
            print("  - {}".format(label))
        return 1
    print("all {} checks passed — the travel pose is only reached through the checked "
          "waypoint, and nothing reaches or grasps from a pose it was not checked from"
          .format(checks))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
