#!/usr/bin/env python3
"""Offline check of the travel pose <-> E1 transition, moved the way the controller moves.

DeployConfig.home_deg's comment has long said the nav -> E1 transition must follow
waypoints from this script; until 2026-09-28 the script did not exist. It answers
one question: does any part of the arm, or a held box, come closer to the robot
body or the floor part-way through the move than it already is at either end?
The two ends are poses the robot uses every day; the middle had never been looked
at.

Why a straight command is not enough: move_guarded_and_verified sends one command
per iteration and send_degrees clamps EACH joint to max_delta_deg (8 deg) from the
last command independently. Going E1 -> travel, S3/S4 (8.6 deg to go) finish in two
steps while S2 (66 deg) has barely started, so the arm folds before it rises and
a held box sweeps to 10.6 mm of the TG30 lidar. Going through a waypoint that
keeps S3/S4 at their E1 angles while S2 swings, and straightens them at a middle
S2, removes that.

Model: the full URDF with collision meshes (PyBullet uses their convex hulls, which
are larger than the meshes, so every distance here is conservative), a 7 cm cube
centred on the TCP for a held box, and a cylinder standing in for the real TG30 at
the measured spot (10 cm ahead of the chassis centre, scan plane 9.5 cm up); the
URDF's own laser_link is the stock lidar, not the one on this robot.

Result 2026-09-28 (python validate_nav_to_grasp_transition.py --sweep):
  direct E1 -> travel holding a box: box 10.6 mm from the TG30 (25.4 at the ends),
  finger links 22.5 mm (56.3 at the ends)
  via S2 = 100..135: nothing closer mid-path than at the ends; S2 = 140 puts
  arm_link3 0.6 mm into base_link's hull. 115 is used, the middle of the safe band.
  One transition via the waypoint is 10 guarded steps.

Needs pybullet, numpy, torch and stable-baselines3 (it imports the controller for
its own DeployConfig and JointMapper); runs on the dev machine, not the Nano.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pybullet as p

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import deploy_contract as dc   # noqa: E402
import x3plus_real_grasp as g  # noqa: E402

TRAVEL = (90.0, 140.0, 0.0, 0.0, 90.0, 30.0)     # docs/calibration/arm_pose.md
E1 = tuple(g.DeployConfig().home_deg)
WAYPOINT_S2 = 115.0
HOLD_JAW = 110.0             # jaw angle while holding the 6.5 cm box (approximate)
BOX_HALF = (0.035, 0.035, 0.035)
SUB = 12                     # samples inside one command's run time
NEAR_M = 0.030               # a pair is judged once it is within 30 mm
ARM_SIDE = ("arm_link3", "arm_link4", "arm_link5", "rlink1", "rlink2", "rlink3",
            "llink1", "llink2", "llink3", "mono_link")
BODY = ("base_link", "front_right_wheel", "front_left_wheel", "back_right_wheel",
        "back_left_wheel", "laser_link", "eyes")


def waypoint(s2: float, jaw: float):
    """Travel-side waypoint: S2 at `s2`, S3/S4 still at their E1 angles."""
    return (E1[0], float(s2), E1[2], E1[3], E1[4], jaw)


class Model:
    def __init__(self):
        cfg = g.DeployConfig()
        self.step = float(cfg.max_delta_deg)
        self.mapper = g.JointMapper(cfg)
        p.connect(p.DIRECT)
        self.robot = p.loadURDF(str(HERE.parent / "x3plus" / "yahboomcar.urdf"),
                                basePosition=list(dc.URDF_TO_TRAINING_FRAME),
                                useFixedBase=True, flags=p.URDF_IGNORE_VISUAL_SHAPES)
        n = p.getNumJoints(self.robot)
        self.link = {p.getJointInfo(self.robot, j)[12].decode(): j for j in range(n)}
        joint = {p.getJointInfo(self.robot, j)[1].decode(): j for j in range(n)}
        self.arm = [joint["arm_joint%d" % i] for i in range(1, 6)]
        self.grip = [(joint[k], m) for k, m in dc.GRIPPER_JOINT_MULTIPLIERS.items()]
        self.floor_z = min(p.getAABB(self.robot, self.link[w])[0][2] for w in BODY[1:5])
        foot = np.array(p.getBasePositionAndOrientation(self.robot)[0])
        self.tg30 = p.createMultiBody(0, p.createCollisionShape(
            p.GEOM_CYLINDER, radius=0.05, height=0.045), basePosition=list(foot + [0.10, 0, 0.095]))
        self.box = p.createMultiBody(0, p.createCollisionShape(p.GEOM_BOX, halfExtents=BOX_HALF),
                                     basePosition=[5, 5, 5])

    def set(self, hw):
        for i, a in zip(self.arm, self.mapper.hw_deg_to_sim_arm(list(hw[:5]))):
            p.resetJointState(self.robot, i, float(a))
        gr = self.mapper.hw_deg_to_sim_grip(hw[5])
        for i, m in self.grip:
            p.resetJointState(self.robot, i, gr * m)

    def path(self, a, b):
        """The controller's path: per-joint clamped steps, linear inside each step."""
        cur, out = list(a), [tuple(a)]
        for _ in range(200):
            if all(abs(x - y) < 1e-9 for x, y in zip(cur, b)):
                return out
            nxt = [c + max(-self.step, min(self.step, t - c)) for c, t in zip(cur, b)]
            out += [tuple(c + (n - c) * k / SUB for c, n in zip(cur, nxt)) for k in range(1, SUB + 1)]
            cur = nxt
        raise RuntimeError("path did not converge")

    def distances(self, hw, carry):
        """{pair: metres} for this pose, plus floor clearance under 'FLOOR:<link>'."""
        self.set(hw)
        movers = [(self.robot, self.link[n], n) for n in ARM_SIDE]
        if carry:
            r = np.array(p.getLinkState(self.robot, self.link["rlink3"])[0])
            l = np.array(p.getLinkState(self.robot, self.link["llink3"])[0])
            orn = p.getLinkState(self.robot, self.link["arm_link5"])[1]
            p.resetBasePositionAndOrientation(self.box, list((r + l) / 2.0), orn)
            movers.append((self.box, -1, "BOX"))
        out = {}
        for body, link, name in movers:
            out["FLOOR:" + name] = p.getAABB(body, link)[0][2] - self.floor_z
            targets = [(self.robot, self.link[b], b) for b in BODY] + [(self.tg30, -1, "TG30")]
            for tb, tl, tn in targets:
                pts = p.getClosestPoints(body, tb, 0.2, link, tl)
                out["%s-%s" % (name, tn)] = min((q[8] for q in pts), default=0.2)
        return out

    def check(self, legs, carry):
        """Worst (min along path) per pair across all legs, and the endpoint values."""
        samples = []
        for a, b in legs:
            pts = self.path(a, b)
            samples += pts if not samples else pts[1:]
        ds = [self.distances(hw, carry) for hw in samples]
        return {k: (min(d[k] for d in ds), min(ds[0][k], ds[-1][k])) for k in ds[0]}


def report(model, label, legs, carry, show=4):
    res = model.check(legs, carry)
    # A pair only matters once it is close: 80 mm shrinking to 75 mm is noise.
    worse = sorted((lo - ends, k, lo, ends) for k, (lo, ends) in res.items()
                   if lo < ends - 0.001 and lo < NEAR_M)
    near = sorted((lo, k) for k, (lo, _e) in res.items() if not k.startswith("FLOOR"))[:show]
    floor = min((lo, k[6:]) for k, (lo, _e) in res.items() if k.startswith("FLOOR"))
    print("%-30s floor %5.1f mm (%s) | nearest %s" % (
        label, floor[0] * 1000, floor[1], ", ".join("%s %.1f" % (k, v * 1000) for v, k in near)))
    for _gap, k, lo, ends in worse:
        print("   !! %s comes to %.1f mm mid-path (%.1f mm at the ends)" % (k, lo * 1000, ends * 1000))
    return not worse


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--waypoint-s2", type=float, default=WAYPOINT_S2)
    ap.add_argument("--sweep", action="store_true", help="also try S2 = 100..140 in 5 deg steps")
    args = ap.parse_args()
    m = Model()
    held = lambda hw: tuple(hw[:5]) + (HOLD_JAW,)
    print("travel %s  E1 %s  (per-joint step %.0f deg)\n" % (TRAVEL, E1, m.step))
    print("direct, as a single guarded move would do it:")
    for label, a, b, carry in (("  E1 -> travel, open", E1, TRAVEL, False),
                               ("  E1 -> travel, holding", held(E1), held(TRAVEL), True)):
        report(m, label, [(a, b)], carry)
    s2s = list(range(100, 141, 5)) if args.sweep else [args.waypoint_s2]
    ok_all = True
    for s2 in s2s:
        print("\nvia waypoint S2 = %.0f:" % s2)
        ok = True
        for label, a, b, carry in (("  E1 -> travel, open", E1, TRAVEL, False),
                                   ("  travel -> E1, open", TRAVEL, E1, False),
                                   ("  E1 -> travel, holding", held(E1), held(TRAVEL), True),
                                   ("  travel -> E1, holding", held(TRAVEL), held(E1), True)):
            w = waypoint(s2, a[5])
            ok &= report(m, label, [(a, w), (w, b)], carry)
        print("  => %s" % ("nothing comes closer mid-path than at the ends" if ok else "REJECTED"))
        if s2 == args.waypoint_s2:
            ok_all = ok
    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(main())
