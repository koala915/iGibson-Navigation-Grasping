#!/usr/bin/env python3
"""One command that says whether this machine is ready, and what is left.

Two halves:

  --offline   everything checkable without the robot. Run it on any machine,
              including the one you are reading this on. If it is not green,
              nothing on the robot will work either.

  --onboard   run this ON the Jetson with ROS up. It checks the things that
              only exist there -- serial ownership, /scan, AMCL, the lidar's
              forward coverage -- and prints what is still unverified.

Neither half touches a motor or a servo. Both are safe to run at any time.

    python3 integration/preflight.py --offline
    python3 integration/preflight.py --onboard --ros-host 127.0.0.1
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

OK, WARN, BAD = "PASS", "WARN", "FAIL"
_MARK = {OK: "[ ok ]", WARN: "[warn]", BAD: "[FAIL]"}


def _default_route() -> Path:
    """route.yaml lives outside this repo, so look where it plausibly is.

    Working tree: the sibling Navigation handoff. Handed-off zip: the
    route_package/ copied in beside it, so the receiver can verify without
    hunting for the Route C package first.
    """
    candidates = [
        ROOT / "route_package" / "config" / "routes" / "route.yaml",
        ROOT.parent / "Navigation" / "02_route_c_external_map_handoff"
        / "config" / "routes" / "route.yaml",
    ]
    for c in candidates:
        if c.exists():
            return c
    return candidates[0]


class Report:
    """Collects check results, and prints them unless a machine is reading.

    ``quiet`` exists for the operator console, which runs this as a subprocess
    and needs the rows as data.  Scraping the pretty output would work until
    somebody reworded a line, so the same rows go out as JSON instead and the
    human format stays free to change.
    """

    def __init__(self, quiet=False):
        self.rows = []
        self.quiet = quiet
        self.sections = []
        self.manual = []

    def add(self, status, name, detail=""):
        self.rows.append((status, name, detail))
        if not self.quiet:
            print(f"  {_MARK[status]} {name}" + (f"\n         {detail}" if detail else ""))
        return status

    def counts(self):
        c = {OK: 0, WARN: 0, BAD: 0}
        for s, _, _ in self.rows:
            c[s] += 1
        return c

    def finish(self, title):
        c = self.counts()
        self.sections.append({
            "title": title,
            "pass": c[OK], "warn": c[WARN], "fail": c[BAD],
            "checks": [{"status": s, "name": n, "detail": d} for s, n, d in self.rows],
        })
        # Both halves can run in one invocation, and they share this report so
        # the JSON carries both. Reset here or the second section would repeat
        # the first one's rows and double every count.
        self.rows = []
        if not self.quiet:
            print(f"\n  {title}: {c[OK]} pass, {c[WARN]} warn, {c[BAD]} fail")
        return 0 if c[BAD] == 0 else 1

    def as_json(self):
        total = {"pass": 0, "warn": 0, "fail": 0}
        for s in self.sections:
            for k in total:
                total[k] += s[k]
        return {"schema": 1, "ok": total["fail"] == 0, "total": total,
                "sections": self.sections, "manual": self.manual}


# ════════════════════════════════════════════════════════════════════════════
# Offline
# ════════════════════════════════════════════════════════════════════════════

SUITES = [
    ("integration/map_goal_provider.py", ["--selftest"]),
    ("integration/feedback_odom.py", ["--selftest"]),
    ("integration/ros_io.py", ["--selftest"]),
    ("integration/mission_fsm.py", ["--selftest"]),
    ("integration/mission_pipeline.py", ["--selftest"]),
    ("integration/nav_rl.py", ["--selftest"]),
    ("integration/vision_grasp_pipeline.py", ["--selftest"]),
    ("integration/nav_rl_grasp_pipeline.py", ["--selftest"]),
    ("tests/test_grasp_home_homography.py", []),
    ("tests/test_model_package.py", []),
    ("tests/test_safety_guards.py", []),
    ("tests/test_mission_end_to_end.py", []),
    ("tests/test_stream_cam_capture.py", []),
    ("tests/test_vision_grasp_bridge_pose.py", []),
    # The offboard SAM2 adapter. This one pins the left/right sign flip, which
    # is the failure --target-source offboard cannot detect at runtime: both
    # conventions are plain floats in the same range, so a mirrored target just
    # steers the robot away from the trash, confidently.
    ("tests/test_trash_target.py", []),
    ("grasp/v21/test_deploy_controller.py", []),
    # jetson_verify.sh gates on 119 / 37 / 641; without this the offline half
    # only covers 119 and 641, and the half-duplex bus read goes unchecked
    # until someone is already standing next to the robot.
    ("grasp/v21/test_servo_read.py", []),
    ("grasp/v21/test_deploy_floor_guard.py", []),
    # The launcher-to-bridge wiring, for both stacks. Neither of these failures
    # is detectable at runtime from one side: the flags parse, the processes
    # start, and the mismatch appears after the arm has already moved. v23 adds
    # the pose stamp, the policy band and the weight pair to the same category.
    ("grasp/v21/test_one_command_launcher.py", []),
    ("grasp/v23/test_one_command_launcher.py", []),
    # grasp/v23's 119 / 37 / 641 are deliberately NOT here. They are the same
    # suites against the same code, and running the 641 twice roughly doubles
    # the offline pass for no new coverage. ./grasp/v23/jetson_verify.sh runs
    # them, and that is the gate before a v23 hardware run anyway.
]


def _run(script, args, timeout=300):
    # Directly executing a file under tests/ makes Python put tests/ rather
    # than the repository root first on sys.path.  Keep the same import context
    # as unittest discovery so preflight never omits a suite for that reason.
    inherited_path = os.environ.get("PYTHONPATH", "")
    pythonpath = str(ROOT) + (os.pathsep + inherited_path if inherited_path else "")
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONPATH=pythonpath)
    try:
        # Decode as UTF-8 explicitly: the suites print em-dashes and box
        # characters, and a cp950 console would otherwise raise mid-read and
        # report a passing suite as a crash.
        p = subprocess.run([sys.executable, str(ROOT / script), *args],
                           capture_output=True, text=True, timeout=timeout,
                           encoding="utf-8", errors="replace",
                           env=env, cwd=str(ROOT),
                           stdin=subprocess.DEVNULL)
        return p.returncode, (p.stdout or "") + (p.stderr or "")
    except subprocess.TimeoutExpired:
        return 124, "timed out"
    except Exception as exc:                                   # pragma: no cover
        return 1, str(exc)


def _last_line(out: str, code: int) -> str:
    """The most useful line of a failed run, or the exit code if it said nothing.

    A suite that dies without printing -- a segfault in a native library at
    import time is the one we actually hit on the Jetson -- leaves this empty.
    Indexing [-1] into that crashed preflight itself with a traceback, which
    looks like preflight is broken rather than like the suite failed.
    """
    lines = [l.strip() for l in out.splitlines() if l.strip()]
    return lines[-1][:160] if lines else f"exit code {code}, no output"


def check_offline(args, r=None) -> int:
    r = r if r is not None else Report()
    if not r.quiet:
        print("\n== OFFLINE: everything checkable without the robot ==\n")

    for script, extra in SUITES:
        if not (ROOT / script).exists():
            r.add(BAD, script, "missing")
            continue
        code, out = _run(script, extra)
        r.add(OK if code == 0 else BAD, script,
              "" if code == 0 else _last_line(out, code))

    # ── the grasp model pair must match its manifest ──
    try:
        sys.path.insert(0, str(HERE))
        import mission_pipeline as mp
        model, vec = mp.resolve_grasp_model(verify_hashes=True)
        r.add(OK, "grasp model pair matches manifest sha256",
              f"{Path(model).name} + {Path(vec).name}")
    except SystemExit as exc:
        r.add(BAD, "grasp model pair", str(exc)[:200])
    except Exception as exc:
        r.add(BAD, "grasp model pair", f"{type(exc).__name__}: {exc}"[:200])

    # ── the nav weights must be present and paired ──
    # The nav model and its VecNormalize are one unit; a missing half loads
    # nothing and a mismatched half normalises the observation wrongly.
    nav = HERE / "nav_best_model"
    z = nav / "ppo_nav_281440_steps.zip"
    p = nav / "ppo_nav_vecnormalize_281440_steps.pkl"
    paired = z.exists() and p.exists()
    r.add(OK if paired else BAD, "nav weights present and paired",
          "" if paired else f"missing {'zip' if not z.exists() else 'pkl'} in {nav}")

    # ── the route has to load and be drivable ──
    route = args.route
    if route and Path(route).exists():
        code, out = _run("integration/map_goal_provider.py",
                         ["--validate", "--route", route,
                          "--resample-m", str(args.resample_m)])
        if code == 0:
            detail = next((l.strip() for l in out.splitlines()
                           if "re-sampled" in l), "")[:160]
        else:
            # On failure "re-sampled" is exactly the line that is missing, so
            # searching for it reports a bare FAIL with no reason attached.
            detail = _last_line(out, code)
        r.add(OK if code == 0 else BAD, "route.yaml loads and validates", detail)
    else:
        r.add(WARN, "route.yaml", f"not found at {route!r}; pass --route")

    # ── arm homes must not have collapsed back into one ──
    try:
        import mission_pipeline as mp
        distinct = mp.NAV_HOME_DEG != mp.GRASP_HOME_DEG
        r.add(OK if distinct else BAD, "arm nav/grasp homes are distinct",
              f"nav={list(mp.NAV_HOME_DEG)} grasp={list(mp.GRASP_HOME_DEG)}")
    except Exception as exc:
        r.add(BAD, "arm homes", str(exc)[:120])

    return r.finish("OFFLINE")


# ════════════════════════════════════════════════════════════════════════════
# Onboard
# ════════════════════════════════════════════════════════════════════════════

def check_onboard(args, r=None) -> int:
    r = r if r is not None else Report()
    if not r.quiet:
        print("\n== ONBOARD: run this on the Jetson, with ROS up ==\n")

    # ── serial ownership ──
    # Existence first. fuser reports a missing device as "Specified filename
    # /dev/myserial does not exist." on stderr, and that line contains "/dev",
    # so the filter below drops it, leaves zero holders, and reports PASS -- an
    # unplugged chassis would read as a clean serial port.
    if not Path("/dev/myserial").exists():
        r.add(BAD, "serial /dev/myserial exists",
              "no /dev/myserial: chassis unplugged, powered off, or the udev "
              "rule did not fire")
    else:
        try:
            p = subprocess.run(["fuser", "-v", "/dev/myserial"], capture_output=True,
                               text=True, timeout=10)
            holders = [l for l in (p.stdout + p.stderr).splitlines() if "/dev" not in l]
            n = len([h for h in holders if h.strip() and "USER" not in h])
            r.add(OK if n <= 1 else BAD, "serial /dev/myserial has at most one owner",
                  (p.stdout + p.stderr).strip()[:200])
        except FileNotFoundError:
            r.add(WARN, "serial owner check", "fuser not available on this machine")
        except Exception as exc:
            r.add(WARN, "serial owner check", str(exc)[:120])

    # ── competing drivers ──
    try:
        p = subprocess.run(["ps", "-ef"], capture_output=True, text=True, timeout=10)
        bad = [l for l in p.stdout.splitlines()
               if any(k in l for k in ("rosmaster_main.py", "Mcnamu_driver.py",
                                       "ai_motor_server_B.py", "route_a_runtime"))
               and "grep" not in l]
        r.add(OK if not bad else BAD, "no competing chassis driver running",
              "\n         ".join(b[:150] for b in bad))
    except Exception as exc:
        r.add(WARN, "competing driver check", str(exc)[:120])

    # ── /scan + forward coverage + AMCL, all through the real adapters ──
    try:
        sys.path.insert(0, str(HERE))
        import nav_rl as nr
        import ros_io

        cfg = nr.NavRLConfig()
        if args.lidar_yaw_offset_deg is not None:
            cfg.lidar_yaw_offset_deg = args.lidar_yaw_offset_deg
        lidar = nr.make_lidar(cfg, "ros", ros_host=args.ros_host,
                              ros_port=args.ros_port)
        import time
        deadline = time.time() + 5.0
        while time.time() < deadline and not lidar.get_points():
            time.sleep(0.2)
        pts = lidar.get_points()
        r.add(OK if pts else BAD, "/scan is publishing",
              f"{len(pts)} points, age {lidar.age():.2f}s")

        info = lidar.scan_info(cfg)
        if info is None:
            r.add(BAD, "lidar forward coverage", "no scan to inspect")
        else:
            detail = (f"frame={info['frame_id']!r} "
                      f"window [{info.get('angle_min_deg', float('nan')):.0f}, "
                      f"{info.get('angle_max_deg', float('nan')):.0f}] deg, "
                      f"forward covered {info.get('forward_fraction', 0)*100:.0f}%")
            r.add(BAD if info["warnings"] else OK,
                  "lidar covers the policy's forward arc",
                  detail + ("\n         " + "\n         ".join(info["warnings"])
                            if info["warnings"] else ""))
            evidence = getattr(args, "lidar_orientation_evidence", None)
            if not evidence:
                r.add(BAD, "LiDAR/policy physical orientation evidence",
                      "missing --lidar-orientation-evidence; AMCL/TF cannot prove raw scan semantics")
            else:
                good, why = nr.validate_orientation_evidence(evidence, cfg, info)
                r.add(OK if good else BAD,
                      "LiDAR/policy physical orientation evidence", why)
        lidar.close()
    except Exception as exc:
        r.add(BAD, "/scan via rosbridge", f"{type(exc).__name__}: {exc}"[:200])

    try:
        rio = ros_io.RosBridgeIO(args.ros_host, args.ros_port)
        import time
        deadline = time.time() + 5.0
        while time.time() < deadline and rio.latest_pose() is None:
            time.sleep(0.2)
        pose = rio.latest_pose()
        good, why = rio.pose_quality_ok(
            max_age_s=getattr(args, "amcl_max_age", ros_io.DEFAULT_MAX_AMCL_AGE_S))
        if pose is None:
            r.add(BAD, "/amcl_pose is publishing",
                  "set a TIGHT 2D Pose Estimate in RViz (std 0.15 m / 7 deg)")
        else:
            r.add(OK if good else BAD, "AMCL pose is usable",
                  f"({pose.x:.2f}, {pose.y:.2f}) " + (why or "covariance ok"))

        # A grasp holds the base still for minutes, and AMCL only updates on
        # motion. Without a way to force an update, the first map state after
        # the grasp refuses to drive on the stale fix and the mission pauses.
        services = rio.rosapi_list("services")
        if services is None:
            r.add(BAD, "AMCL can be refreshed while stationary",
                  f"cannot verify {ros_io.NOMOTION_SERVICE}: rosapi did not "
                  "answer. Is rosapi running alongside rosbridge?")
        elif ros_io.NOMOTION_SERVICE not in services:
            r.add(BAD, "AMCL can be refreshed while stationary",
                  f"{ros_io.NOMOTION_SERVICE} is not advertised — a stationary "
                  "robot's fix goes stale and the mission pauses after the "
                  "first grasp")
        else:
            r.add(OK, "AMCL can be refreshed while stationary",
                  f"{ros_io.NOMOTION_SERVICE} advertised")
        rio.close()
    except Exception as exc:
        r.add(BAD, "/amcl_pose via rosbridge", f"{type(exc).__name__}: {exc}"[:200])

    manual = (
        "T1  push the robot 50 cm by hand, confirm /odom_setmotor grows to match",
        "T2  --probe: object in FRONT/LEFT/RIGHT lights the matching sector",
        "T2  --probe at both arm poses: a fixed 5-19 cm return dead ahead is the ARM",
        "T3  one patrol lap with --detection-streak 999 (never leaves the route)",
        "T4  place an object beside the route; confirm it does NOT bounce back to PATROL",
        "T5  grasp: base must be completely still while the arm moves",
        "T6  bin: log must say the bin approach point, not a patrol waypoint",
    )
    r.manual = list(manual)
    if not r.quiet:
        print("\n  Still needing a human, in this order — see TEST_PLAN.md:")
        for line in manual:
            print(f"    [ ] {line}")

    return r.finish("ONBOARD")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--offline", action="store_true")
    ap.add_argument("--onboard", action="store_true")
    ap.add_argument("--json", action="store_true",
                    help="emit the results as JSON on stdout and nothing else, "
                         "for the operator console")
    ap.add_argument("--route", default=str(_default_route()))
    ap.add_argument("--resample-m", type=float, default=0.75)
    ap.add_argument("--ros-host", default="127.0.0.1")
    ap.add_argument("--ros-port", type=int, default=9090)
    ap.add_argument("--lidar-yaw-offset-deg", type=float, default=0.0)
    ap.add_argument("--lidar-orientation-evidence", default=None,
                    help="verified Route B four-direction evidence/marker")
    ap.add_argument("--amcl-max-age", type=float,
                    default=5.0,
                    help="maximum local AMCL receipt age in seconds")
    args = ap.parse_args()

    if not math.isfinite(args.amcl_max_age) or args.amcl_max_age <= 0.0:
        ap.error("--amcl-max-age must be finite and > 0")

    if not (args.offline or args.onboard):
        args.offline = True
    r = Report(quiet=args.json)
    rc = 0
    if args.offline:
        rc |= check_offline(args, r)
    if args.onboard:
        rc |= check_onboard(args, r)
    if args.json:
        # Sole occupant of stdout, so the caller can json.loads() it directly.
        #
        # Written as bytes through the buffer rather than through the text
        # layer: check details quote the suites' own output, which contains
        # em-dashes, and a redirected stdout on Windows encodes as cp950 and
        # produces a document no JSON parser can read. Encoding here makes the
        # output UTF-8 on every platform without the caller having to set
        # PYTHONIOENCODING first.
        blob = json.dumps(r.as_json(), ensure_ascii=False).encode("utf-8")
        out = getattr(sys.stdout, "buffer", None)
        if out is None:                     # a stdout replaced by a test harness
            sys.stdout.write(blob.decode("utf-8") + "\n")
        else:
            out.write(blob + b"\n")
            out.flush()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
