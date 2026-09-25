"""Does the REAL-ROBOT floor guard hold? (No sim backstop allowed.)

The simulation guard can restore a violating state. Hardware cannot: once the gripper
is in the floor, it is in the floor. So the deployment guard has to be purely
preventive, and this test holds it to that standard — for every (current pose, target
pose) pair it is given, the command it returns must sweep to the target without any
gripper link crossing the safety line.

Adversaries:
  random        random current pose, random target
  straight_down targets driven hard toward the floor
  from_below    current pose already under the line (emergency-raise path)
  greedy        of 32 random targets, the one whose sweep dips lowest

Usage:  python test_deploy_floor_guard.py --trials 200
"""
from __future__ import annotations

import argparse
import contextlib
import io
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

FAILURES = []
CHECKS = 0


def check(cond, label, detail=""):
    global CHECKS
    CHECKS += 1
    if not cond:
        FAILURES.append(f"{label}  {detail}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=200)
    ap.add_argument("--seed", type=int, default=20260728)
    args = ap.parse_args()

    with contextlib.redirect_stdout(io.StringIO()):
        from x3plus_real_grasp import DeployConfig, FKComputer, FloorGuard
        cfg = DeployConfig()
        fk = FKComputer(cfg.urdf_path)
        guard = FloorGuard(cfg, fk)

    line = guard.line
    rng = np.random.RandomState(args.seed)
    limits = np.array(cfg.arm_sim_limits, dtype=np.float64)

    print("=" * 76)
    print(f"deployment floor guard  (margin {cfg.floor_safety_margin*1000:.0f} mm, "
          f"line z >= {line*1000:.1f} mm, sweep {cfg.floor_sweep_samples} samples)")
    print("requirement: the COMMAND the guard returns must never sweep below the line")
    print("=" * 76)

    def rand_pose(scale=0.6):
        return np.array([rng.uniform(lo * scale, hi * scale) for lo, hi in limits])

    def near_floor_pose(z_hi=0.060, tries=400):
        """A pose whose gripper is actually close to the floor.

        Uniform joint sampling almost never gets near the ground, so a guard test
        built on it never fires and proves nothing — the first version of this test
        passed 440 checks with the guard triggering zero times. Rejection-sample
        instead, and require the caller to handle failure rather than silently
        falling back to a high pose.
        """
        best, best_z = None, 1e9
        for _ in range(tries):
            q = rand_pose(rng.uniform(0.3, 0.95))
            z = fk.min_gripper_link_z(q, -1.5)
            if line <= z <= z_hi:
                return q
            if abs(z - z_hi) < abs(best_z - z_hi):
                best, best_z = q, z
        return best if best is not None and best_z <= 0.12 else None

    def descend_target(q):
        """A target that drives the gripper down from q, found by FK rather than by
        assuming which joint sign lowers the arm."""
        best, best_z = None, 1e9
        for _ in range(24):
            t = q + rng.uniform(-0.6, 0.6, size=5)
            t = np.clip(t, limits[:, 0], limits[:, 1])
            z = fk.min_gripper_link_z(t, 0.0)
            if z < best_z:
                best, best_z = t, z
        return best

    worst = {}

    def trial(name, arm_now, grip_now, arm_tgt, grip_tgt):
        a, g, info = guard.project(arm_now, grip_now, arm_tgt, grip_tgt)
        z = fk.sweep_min_z(arm_now, grip_now, a, g, cfg.floor_sweep_samples)
        # The emergency-raise path starts from an already-unsafe pose; it cannot make
        # the *start* safe, so judge it on whether it moved upward.
        if info["action"] == "emergency_raise":
            z_before = fk.min_gripper_link_z(arm_now, grip_now)
            z_after = fk.min_gripper_link_z(a, g)
            check(z_after > z_before, f"{name}: emergency raise moves up",
                  f"before {z_before*1000:.2f} mm, after {z_after*1000:.2f} mm")
            return
        worst[name] = min(worst.get(name, 1e9), z)
        check(z >= line - 1e-9, f"{name}: commanded sweep stays above the line",
              f"sweep min {z*1000:+.3f} mm < {line*1000:.1f} mm (action={info['action']})")

    skipped = 0
    near_poses = []
    for _ in range(args.trials):
        q = near_floor_pose()
        if q is None:
            skipped += 1
            continue
        near_poses.append(q)
        trial("random", q, -1.5, rand_pose(), rng.uniform(-1.5, 0.0))

    for q in near_poses:
        trial("descend", q, -1.5, descend_target(q), 0.0)

    # Closing the jaw alone drops the pads ~2.67 cm with no arm motion at all —
    # the guard must account for that too, not just arm targets.
    for q in near_poses:
        trial("jaw_close_only", q, -1.5, q.copy(), 0.0)

    # Start genuinely below the line: only the emergency raise is possible.
    raised = 0
    for _ in range(120):
        q = rand_pose(rng.uniform(0.5, 1.0))
        if fk.min_gripper_link_z(q, -1.5) < line:
            raised += 1
            trial("from_below", q, -1.5, rand_pose(), 0.0)

    for q in near_poses[:max(20, args.trials // 5)]:
        cands = [descend_target(q) for _ in range(8)]
        zs = [fk.sweep_min_z(q, -1.5, c, 0.0, 4) for c in cands]
        trial("greedy", q, -1.5, cands[int(np.argmin(zs))], 0.0)

    print(f"  near-floor poses generated: {len(near_poses)} (skipped {skipped})")

    for k, v in sorted(worst.items()):
        print(f"  {k:<15} worst commanded sweep = {v*1000:+8.3f} mm")
    print(f"  from_below trials that were genuinely below the line: {raised}")
    print(f"  guard clamped {guard.interventions}x, refused {guard.refusals}x")
    # A guard test in which the guard never fires proves nothing. Fail loudly rather
    # than report a green run that exercised no safety code.
    check(guard.interventions + guard.refusals > 0,
          "the guard actually fired during this test",
          "it never triggered — the adversaries never reached the floor, so this run "
          "is vacuous regardless of how many checks 'passed'")

    with contextlib.redirect_stdout(io.StringIO()):
        fk.close()

    print("=" * 76)
    if FAILURES:
        print(f"FAILED {len(FAILURES)}/{CHECKS}:")
        for f in FAILURES[:12]:
            print(f"  - {f}")
        return 1
    print(f"all {CHECKS} checks passed — deployment guard is preventive")
    return 0


if __name__ == "__main__":
    sys.exit(main())
