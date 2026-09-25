#!/usr/bin/env python3
"""Self-driving grasp with RL navigation (LiDAR obstacle avoidance).

Same one-process design as vision_grasp_pipeline.py (one Rosmaster owns wheels
and arm), but the long-range approach is driven by the iGibson-trained PPO nav
policy instead of the hand-tuned rear-camera state machine:

    1. YOLO (rear mast cam, forward-looking) finds the object
       -> goal tracker keeps (dist, bearing) fresh, dead-reckoning between
          detections so the policy may turn away to avoid obstacles
    2. PPO nav policy (48-ray LiDAR obs) drives toward the goal AVOIDING
       obstacles; a geometric safety brake overlays it (policy alone still
       collides in ~20-27%% of sim episodes — never trust it raw)
    3. tracker dist <= --nav-stop-dist -> stop, settle -> visual fine-align
       with the arm camera (reused from vision_grasp_pipeline)
    4. handoff at ARM_BLIND_START_DIST -> latch target -> PPO grasp
    5. verify; on failure back off and retry (<= --max-retries)

Run:
    python nav_rl_grasp_pipeline.py --selftest              # pure logic, no hw
    python nav_rl_grasp_pipeline.py --no-lidar              # dry-run w/o lidar

The integrated --real path is intentionally refused until measured grasp-home
homography mapping is connected to final alignment and target latching.

HARDWARE PREREQUISITES (same as vision_grasp_pipeline.py, plus lidar):
    * port-7000 motor server / ROS base driver NOT running
    * camera streams on :8080; run ON the Jetson (default --jetson-ip 127.0.0.1)
    * YDLIDAR TG30 ROS driver publishing /scan + rosbridge on port 9090
      (verify with: python nav_rl.py --probe --lidar-backend ros)
"""
from __future__ import annotations

import argparse
import math
import os
import time
from typing import Optional

import numpy as np

try:  # direct script execution: python3 integration/nav_rl_grasp_pipeline.py
    import mission_status
    import nav_rl as nr
    import vision_grasp_pipeline as vgp
except ImportError:  # package import used by tests/tools
    from . import mission_status
    from . import nav_rl as nr
    from . import vision_grasp_pipeline as vgp

# nav-specific tunables (the plant params live in nav_rl.NavRLConfig and must
# NOT be tuned — these are pipeline glue only)
NAV_STOP_DIST_M = 0.75        # tracker dist to end RL nav (arm cam sees <=0.70)
NAV_STAY_STEPS = 4            # consecutive stopped steps before fine-align
NAV_DET_INTERVAL_S = 0.5      # YOLO cadence during RL nav (Jetson-friendly)
NAV_GOAL_LOST_TIMEOUT_S = 5.0  # no camera fix this long -> give up
NAV_MAX_DURATION_S = 120.0
NAV_LIDAR_GIVEUP_S = 3.0      # stale lidar this long -> abort approach
NAV_LOG_PERIOD_S = 1.0


class RLNavigator(vgp.Navigator):
    """vision_grasp_pipeline Navigator with the approach replaced by the RL policy."""

    def __init__(self, device, model, *, policy: Optional[nr.NavPolicy],
                 lidar, ncfg: nr.NavRLConfig, wz_sign: float = 1.0,
                 nav_stop_dist: float = NAV_STOP_DIST_M,
                 rl_only: bool = False, **kw):
        super().__init__(device, model, **kw)
        self.policy = policy
        self.lidar = lidar
        self.ncfg = ncfg
        self.wz_sign = wz_sign
        self.nav_stop_dist = nav_stop_dist
        self.rl_only = rl_only

    # ── raw chassis command (m/s, rad/s) ──
    def _drive_raw(self, vx: float, wz: float):
        vz = self.wz_sign * wz
        if self.dry_run or self.device is None:
            print(f"[nav-rl][dry] set_car_motion({vx:+.3f}, 0, {vz:+.3f})")
            return
        self.device.set_car_motion(float(vx), 0.0, float(vz))

    # ── approach = RL long range + visual fine align ──
    def approach(self) -> bool:
        self.open_cameras()
        if not self._rl_navigate():
            return False
        if self.rl_only:
            print("[nav-rl] --nav-only: RL segment done — skipping fine-align "
                  "(pure navigation validation).")
            return True
        return self._arm_align()

    def _rl_navigate(self) -> bool:
        cfg = self.ncfg
        if self.policy is None:
            raise RuntimeError("RL navigation requires a loaded policy")
        tracker = nr.GoalTracker()
        delay = nr.ActionDelay(cfg.motor_delay_steps)
        prev_action = np.zeros(2, dtype=np.float32)
        last_vx, last_wz = 0.0, 0.0
        stay = 0
        t_start = last_seen = last_log = time.time()
        last_det = 0.0
        lidar_stale_since = None
        t_prev = time.time()
        print(f"[nav-rl] RL approach start (stop at dist<={self.nav_stop_dist}m)")

        while True:
            now = time.time()
            dt = min(now - t_prev, 3 * cfg.control_period_s)
            t_prev = now

            if now - t_start > NAV_MAX_DURATION_S:
                print("[nav-rl] max nav duration exceeded — give up")
                self.stop()
                return False

            # ── lidar (safety-critical: stop on stale data) ──
            pts = self.lidar.get_points()
            if self.lidar.age() > cfg.lidar_stale_timeout_s:
                self.stop()
                last_vx = last_wz = 0.0
                if lidar_stale_since is None:
                    lidar_stale_since = now
                    print("[nav-rl] lidar stale — holding")
                elif now - lidar_stale_since > NAV_LIDAR_GIVEUP_S:
                    print("[nav-rl] lidar dead — give up approach")
                    return False
                time.sleep(cfg.control_period_s)
                continue
            lidar_stale_since = None
            rays = nr.scan_to_rays(pts, cfg)

            # ── goal fix from YOLO (rear mast cam; arm cam once close) ──
            if now - last_det >= NAV_DET_INTERVAL_S:
                last_det = now
                found, dist_f, off = self._detect_rear()
                if found and dist_f > 0:
                    tracker.update(dist_f, off)
                    last_seen = now
                elif tracker.has_fix and tracker.dist() < 0.9:
                    # No `dist_a > 0` filter: the arm camera's ground distance is
                    # measured from its own ground projection and is legitimately
                    # negative behind it, which at the C3 pose is most of the frame.
                    # _detect_arm() already reports found=False on unusable geometry.
                    found_a, dist_a, off_a, _w, _c = self._detect_arm()
                    if found_a:
                        tracker.update(dist_a, off_a)
                        last_seen = now

            if not tracker.has_fix:
                self.stop()
                last_vx = last_wz = 0.0
                if now - last_seen > NAV_GOAL_LOST_TIMEOUT_S:
                    print("[nav-rl] no target found — give up approach")
                    return False
                time.sleep(cfg.control_period_s)
                continue
            if now - last_seen > NAV_GOAL_LOST_TIMEOUT_S:
                print("[nav-rl] target lost during approach — give up")
                self.stop()
                return False

            d, b = tracker.dist(), tracker.bearing()

            # ── reached: stop and confirm ──
            if d <= self.nav_stop_dist:
                self.stop()
                last_vx = last_wz = 0.0
                stay += 1
                if stay >= NAV_STAY_STEPS:
                    print(f"[nav-rl] reached dist={d:.2f}m — hand to fine-align")
                    return True
                time.sleep(cfg.control_period_s)
                continue
            stay = 0

            # ── policy step (training plant mirrored in shape_action) ──
            obs = nr.build_nav_obs(d, b, abs(last_vx), last_wz, prev_action, rays)
            action = self.policy.predict(obs)
            executed = delay.push(action)
            vx, wz, stall = nr.shape_action(executed, obs, cfg)

            # ── geometric safety brake (raw points see below the ray floor) ──
            fmin = nr.front_min_raw(pts, cfg)
            braked = fmin < cfg.safety_brake_dist and vx > 0.0
            if braked:
                vx = 0.0

            self._drive_raw(vx, wz)
            tracker.predict(vx, wz, dt)
            prev_action = action
            last_vx, last_wz = vx, wz

            if now - last_log >= NAV_LOG_PERIOD_S:
                last_log = now
                print(f"[nav-rl] d={d:.2f}m b={math.degrees(b):+5.1f}deg "
                      f"act=[{action[0]:+.2f},{action[1]:+.2f}] "
                      f"vx={vx:+.2f} wz={wz:+.2f} minray={np.min(rays):.2f} "
                      f"front={fmin if fmin != float('inf') else 9.99:.2f}"
                      f"{' STALL' if stall else ''}{' BRAKE' if braked else ''}")
            if self.show:
                self._maybe_show()

            spent = time.time() - now
            if cfg.control_period_s - spent > 0:
                time.sleep(cfg.control_period_s - spent)

    def _arm_align(self, *, deadline=None, safety_check=None) -> bool:
        """Visual fine align 0.7m -> handoff (port of the parent ARM_ALIGN state)."""
        last_seen = time.time()
        print("[nav-rl] -> ARM_ALIGN (visual fine approach)")
        while True:
            now = time.time()
            if deadline is not None and time.monotonic() >= deadline:
                self.stop()
                print("[nav-rl] ARM_ALIGN timed out -- stopping")
                return False
            if safety_check is not None:
                reason = str(safety_check() or "")
                if reason:
                    self.stop()
                    print(f"[nav-rl] ARM_ALIGN safety stop: {reason}")
                    return False
            found, dist_arm, offset, box_w, class_name = self._detect_arm()
            if found:
                last_seen = now
                action, speed, status, _dstate, _wz = \
                    vgp.decide_arm_action_by_distance(True, offset, dist_arm)
                print(f"[nav-rl] ARM dist={dist_arm:.2f} off={offset:+.2f} {action} ({status})")
                if action == "handoff":
                    self.stop()
                    self._last_arm = {
                        "dist": dist_arm,
                        "offset": offset,
                        "box_w_px": box_w,
                        "class_name": class_name,
                    }
                    print(f"[nav-rl] HANDOFF: class={class_name} dist={dist_arm:.3f} "
                          f"off={offset:+.3f} box_w={box_w}px")
                    return True
                # Fine-align used to bypass the raw LiDAR brake because it owns
                # this blocking loop.  Apply the same forward obstacle gate as
                # the RL navigator before issuing any camera-driven motion.
                vx, _vy, _vz = vgp.action_to_vxyz(action, speed)
                if vx > 0.0:
                    front = nr.front_min_raw(self.lidar.get_points(), self.ncfg)
                    if front < self.ncfg.safety_brake_dist:
                        self.stop()
                        print(f"[nav-rl] ARM_ALIGN brake: front={front:.2f} m")
                        time.sleep(vgp.ARM_DECISION_INTERVAL)
                        continue
                self._drive(action, speed)
            else:
                self.stop()
                if now - last_seen > vgp.TARGET_LOST_TIMEOUT_S:
                    print("[nav-rl] target lost (ARM) — give up")
                    return False
            time.sleep(vgp.ARM_DECISION_INTERVAL)
            if self.show:
                self._maybe_show()


# ════════════════════════════════════════════════════════════════════════════
# Orchestrator — same retry skeleton as vision_grasp_pipeline.run_pipeline
# ════════════════════════════════════════════════════════════════════════════

def run_pipeline(args):
    lidar_backend = "none" if args.no_lidar else args.lidar_backend
    if lidar_backend == "none" and args.real:
        raise SystemExit("a no-lidar backend with --real is not allowed: the policy has "
                         "no obstacle input and the safety brake is blind.")
    if args.real and not getattr(args, "lidar_orientation_evidence", None):
        raise SystemExit("real navigation requires --lidar-orientation-evidence from "
                         "the Route B four-direction board gate")
    if args.real and not args.nav_only:
        raise SystemExit(
            "real integrated grasp is disabled until the final alignment/latch "
            "uses the calibrated grasp-home homography; --nav-only remains "
            "available for navigation testing"
        )
    if args.real and not args.nav_only and not args.i_confirm_camera_frame:
        raise SystemExit("real grasp requires a confirmed camera->base mapping")
    g = vgp._load_grasp_module()
    from ultralytics import YOLO
    from pathlib import Path

    model_path = (Path(__file__).resolve().parent.parent
                  / "detection" / "models" / "best.pt")
    print(f"[pipeline] loading YOLO: {model_path}")
    model = YOLO(str(model_path))

    ncfg = nr.NavRLConfig()
    if args.model: ncfg.model_path = args.model
    if args.vecnorm: ncfg.vecnorm_path = args.vecnorm
    if args.lidar_port: ncfg.lidar_port = args.lidar_port
    if args.lidar_yaw_offset_deg is not None:
        ncfg.lidar_yaw_offset_deg = args.lidar_yaw_offset_deg
    ncfg.lidar_forward_offset_m = args.lidar_forward_offset_m
    if args.lidar_dir is not None:
        ncfg.lidar_angle_dir = args.lidar_dir
    elif lidar_backend == "rplidar":
        ncfg.lidar_angle_dir = -1.0
    if args.control_period is not None: ncfg.control_period_s = args.control_period
    if args.motor_delay is not None: ncfg.motor_delay_steps = args.motor_delay
    nr.validate_config(ncfg)

    policy = nr.NavPolicy(ncfg)

    cfg = g.DeployConfig(serial_port=args.port)
    # v21 has no manual width-based closure sizing -- its jaw closes on CONTACT
    # (command-vs-encoder divergence during the close) and holds at contact + a
    # bias, regardless of the object's width. See vision_grasp_pipeline.py's
    # run_pipeline() for the fuller version of this note.
    if args.nav_home_deg is not None:
        cfg.home_deg = g.parse_deg6_csv(args.nav_home_deg)
    if args.grasp_home_deg is not None:
        cfg.grasp_home_deg = g.parse_deg6_csv(args.grasp_home_deg)
    class_z_m = vgp.parse_class_z(args.class_z, vgp.OBJ_Z_FIXED)
    class_height_m = vgp.parse_class_height(args.class_height)
    if class_height_m:
        print(f"[pipeline] class heights (drive v21 wrist_z_offset): {class_height_m}")
    else:
        print("[pipeline] no --class-height given: the grasp side will estimate height "
              "as 2*(centroid_z-ground_z).")
    vgp.warn_unknown_classes(model, [("--class-z", class_z_m),
                                     ("--class-height", class_height_m)])
    # Detection happens at cfg.home_deg (controller.move_home() runs before the RL
    # approach), so that is the pose H_ARM/THETA_ARM must have been measured at.
    vgp.check_arm_cam_pose(cfg.home_deg, real=args.real,
                           acknowledged=args.i_confirm_arm_cam_pose)
    if args.handoff_dist is not None:
        vgp.ARM_BLIND_START_DIST_M = args.handoff_dist

    rear_url = args.rear_stream if args.rear_stream is not None else \
        f"http://{args.jetson_ip}:8080/stream?topic=/back_cam/image_raw"
    arm_url = args.arm_stream if args.arm_stream is not None else \
        f"http://{args.jetson_ip}:8080/stream?topic=/arm_cam/image_raw"

    lidar = nr.make_lidar(
        ncfg, lidar_backend, ros_host=args.ros_host, ros_port=args.ros_port,
        scan_topic=args.scan_topic
    )
    if args.real:
        evidence = args.lidar_orientation_evidence
        deadline = time.time() + 5.0
        info = None
        while time.time() < deadline and info is None:
            info = getattr(lidar, "scan_info", lambda _cfg: None)(ncfg)
            if info is None:
                time.sleep(0.1)
        verified, why = nr.validate_orientation_evidence(evidence, ncfg, info)
        if not verified:
            lidar.close()
            raise SystemExit("real navigation refuses LiDAR orientation evidence: " + why)
        print("[pipeline] " + why)
    try:
        controller = g.GraspController(cfg, real_servo=args.real, use_socket=False)
    except Exception:
        lidar.close()
        raise
    try:
        nav = RLNavigator(
            controller.servo.device, model,
            policy=policy, lidar=lidar, ncfg=ncfg,
            wz_sign=args.wz_sign, nav_stop_dist=args.nav_stop_dist,
            rl_only=args.nav_only,
            show=args.show, dry_run=not args.real,
            rear_url=rear_url, arm_url=arm_url, class_z_m=class_z_m,
            class_height_m=class_height_m,
            cam_to_base_x=args.cam_x, cam_to_base_y=args.cam_y,
            sign_y=args.sign_y,
        )
    except Exception:
        lidar.close()
        controller.close()
        raise

    # Off unless --status-udp was given. Mode C is a straight sequence, so it
    # reports the step it is on using the mission FSM's own state names -- an
    # operator watching the console should not have to learn a second
    # vocabulary just because a different pipeline is driving.
    report = mission_status.SimpleReporter(getattr(args, "status_udp", None), mode="C")
    try:
        success = False
        for attempt in range(1, args.max_retries + 1):
            print("\n" + "=" * 60)
            print(f"[pipeline] ATTEMPT {attempt}/{args.max_retries}")
            print("=" * 60)

            print(f"[pipeline] moving arm to navigation home {list(cfg.home_deg)}")
            # move_home() is guarded (FloorGuard) and blocks until the encoders
            # confirm arrival, unlike v17's fire-and-forget servo.move_to_home() --
            # so no fixed-duration sleep is needed after it.
            report.say("SELF_CHECK", "RUN_SELF_CHECK",
                       f"attempt {attempt}/{args.max_retries}: arm to nav home",
                       attempt=attempt)
            controller.move_home()

            report.say("APPROACH", "DRIVE_TARGET", "RL navigation toward the object",
                       attempt=attempt)
            if not nav.approach():
                report.say("RESUME", "STOP", "no object reachable", attempt=attempt)
                print("[pipeline] no object reachable — stopping.")
                break

            if args.nav_only:
                report.say("COMPLETE", "STOP", "--nav-only: reached the stop distance",
                           attempt=attempt)
                print("[pipeline] --nav-only: RL navigation reached "
                      f"--nav-stop-dist — done (no fine-align, no grasp).")
                success = True
                break

            report.say("LATCH", "LATCH", "freezing the object pose in the base frame",
                       attempt=attempt)
            obj_pos, width, height = nav.latch_arm_object()
            if width is not None and width > vgp.MAX_GRASP_WIDTH_M:
                report.say("FAULT", "STOP",
                           f"object too wide: {width * 100:.1f} cm", attempt=attempt)
                print(f"[pipeline] OBJECT TOO LARGE: width={width * 100:.1f}cm > "
                      f"max {vgp.MAX_GRASP_WIDTH_M * 100:.1f}cm — aborting.")
                break

            # v21's obj_provider second element is the object's HEIGHT in metres --
            # NOT width. It drives wrist_z_offset and the engagement-depth geometry.
            # Comes from --class-height; None when undeclared, in which case v21
            # applies its own symmetric-object estimate. `width` is used only as the
            # too-wide-to-grasp gate above and must never be passed here.
            controller.obj_provider = (lambda p=obj_pos, h=height: (p, h))
            report.say("GRASP", "RUN_GRASP", "v21 grasp controller running",
                       attempt=attempt, grasp={"latched": True, "finished": False,
                                               "verified": False, "handoff_ready": True,
                                               "align_failed": False})
            grasp_seq_ok = controller.run(max_steps=args.max_steps)

            if not grasp_seq_ok:
                report.say("RETRY", "BACK_OFF",
                           "grasp sequence never reached done", attempt=attempt)
                print("[pipeline] grasp sequence did not complete (stage machine "
                      "never reached done) — counting as failure.")
            else:
                report.say("VERIFY", "VERIFY", "checking the object left the floor",
                           attempt=attempt)
                verification = nav.verify_grasp(obj_pos)
                if verification is True:
                    report.say("COMPLETE", "STOP",
                               f"grasp succeeded on attempt {attempt}", attempt=attempt,
                               lifted=1)
                    print(f"[pipeline] OK grasp succeeded on attempt {attempt}.")
                    success = True
                    break
                reason = ("object is still on the floor" if verification is False
                          else "grasp verification unavailable")
                report.say("RETRY", "BACK_OFF", reason, attempt=attempt)
            print("[pipeline] FAILED grasp — retreating and retrying.")
            controller.move_home()   # guarded; ensures gripper open for retry
            time.sleep(1.0)
            nav.back_off()

        if not success:
            report.say("FAULT", "STOP",
                       f"gave up after {args.max_retries} attempt(s)")
            print(f"\n[pipeline] gave up after {args.max_retries} attempt(s).")
    except KeyboardInterrupt:
        report.say("ESTOP", "STOP", "interrupted by the operator")
        print("\n[pipeline] interrupted — emergency stop.")
        try:
            controller.servo.emergency_stop()
        except Exception:
            pass
    finally:
        report.close()
        try:
            nav.stop()
        except Exception as e:
            print(f"[pipeline][WARN] chassis stop failed: {e}")
        try:
            nav.release()
        finally:
            try:
                lidar.close()
            finally:
                controller.close()


# ════════════════════════════════════════════════════════════════════════════
# Self-test — closed-loop toy sim: fake P-controller policy through the REAL
# delay/shaping/tracker plumbing must converge on the goal. No torch/cams/hw.
# ════════════════════════════════════════════════════════════════════════════

def run_selftest():
    nr.run_selftest()

    print("\n== closed-loop plumbing sim (fake P policy) ==")
    cfg = nr.NavRLConfig()
    tracker = nr.GoalTracker()
    delay = nr.ActionDelay(cfg.motor_delay_steps)
    tracker.update(2.0, -0.5)          # 2 m ahead, 0.5 m to the LEFT
    prev = np.zeros(2, dtype=np.float32)
    last_vx = last_wz = 0.0
    rays = np.full(48, 4.0, dtype=np.float32)
    dt = cfg.control_period_s
    for step in range(160):
        d, b = tracker.dist(), tracker.bearing()
        if d <= 0.35:
            break
        obs = nr.build_nav_obs(d, b, abs(last_vx), last_wz, prev, rays)
        fake_action = np.clip(np.array([d, 2.0 * b]), -1.0, 1.0)
        vx, wz, _ = nr.shape_action(delay.push(fake_action), obs, cfg)
        tracker.predict(vx, wz, dt)
        prev, last_vx, last_wz = fake_action, vx, wz
    print(f"  steps={step} final dist={tracker.dist():.3f} m "
          f"bearing={math.degrees(tracker.bearing()):+.1f} deg")
    assert tracker.dist() <= 0.35, "toy loop failed to converge"

    print("\n== safety brake precedence ==")
    obs = nr.build_nav_obs(2.0, 0.0, 0.0, 0.0, np.zeros(2), np.full(48, 4.0, np.float32))
    vx, wz, _ = nr.shape_action(np.array([1.0, 0.0]), obs, cfg)
    pts = [(0.0, 0.15)]                # something 15 cm dead ahead
    fmin = nr.front_min_raw(pts, cfg)
    assert fmin < cfg.safety_brake_dist and vx > 0
    print(f"  vx={vx:.2f} -> braked to 0.0 (front raw {fmin:.2f} m)")

    print("\n[selftest] OK")


def parse_args():
    p = argparse.ArgumentParser(description="X3Plus RL-nav self-driving grasp pipeline")
    p.add_argument("--real", action="store_true", help="drive real wheels + servos")
    p.add_argument("--show", action="store_true", help="show arm camera window")
    p.add_argument("--selftest", action="store_true", help="pure-logic self-test")
    p.add_argument("--status-udp", default=None, metavar="HOST:PORT",
                   help="publish step telemetry to the operator console "
                        f"(e.g. {mission_status.DEFAULT_ENDPOINT}). Fire-and-forget.")
    p.add_argument("--max-retries", type=int, default=3)
    p.add_argument("--max-steps", type=int, default=300)
    p.add_argument("--port", type=str, default="/dev/myserial", help="Rosmaster serial port")
    p.add_argument("--jetson-ip", type=str,
                   default=os.getenv("X3PLUS_JETSON_HOST", "127.0.0.1"),
                   help="camera stream host (pipeline normally runs ON the Jetson)")
    p.add_argument("--rear-stream", type=str, default=None,
                   help="override rear-camera source: URL or webcam index "
                        "(e.g. 0 for a dev-machine webcam dry-run)")
    p.add_argument("--arm-stream", type=str, default=None,
                   help="override arm-camera source: URL or webcam index")
    # nav
    p.add_argument("--nav-only", action="store_true",
                   help="pure RL navigation validation: stop right after the RL "
                        "segment reaches --nav-stop-dist (no arm-cam fine-align, "
                        "no grasp)")
    p.add_argument("--nav-stop-dist", type=float, default=NAV_STOP_DIST_M,
                   help="tracker dist (m) at which RL nav stops and fine-align starts")
    p.add_argument("--wz-sign", type=float, default=1.0, choices=(-1.0, 1.0),
                   help="flip if +wz does not turn the robot LEFT")
    p.add_argument("--control-period", type=float, default=None,
                   help="policy step period s (default 1/6 — matches training)")
    p.add_argument("--motor-delay", type=int, default=None,
                   help="action delay steps (default 2 — matches training)")
    p.add_argument("--model", type=str, default=None)
    p.add_argument("--vecnorm", type=str, default=None)
    # lidar
    p.add_argument("--lidar-backend", choices=("ros", "rplidar", "none"),
                   default="ros", help="scan source (X3Plus TG30 default: ros)")
    p.add_argument("--lidar-port", type=str, default=None)
    p.add_argument("--lidar-yaw-offset-deg", type=float, default=None)
    p.add_argument("--lidar-forward-offset-m", type=float, default=0.0)
    p.add_argument("--lidar-orientation-evidence", default=None,
                   help="verified Route B four-direction evidence/marker; required by --real")
    p.add_argument("--lidar-dir", type=float, default=None, choices=(-1.0, 1.0))
    p.add_argument("--ros-host", type=str,
                   default=os.getenv(
                       "X3PLUS_ROS_HOST",
                       os.getenv("X3PLUS_JETSON_HOST", "127.0.0.1"),
                   ), help="rosbridge host (pipeline normally runs on Jetson)")
    p.add_argument("--ros-port", type=int, default=9090)
    p.add_argument("--scan-topic", type=str, default="/scan")
    p.add_argument("--no-lidar", action="store_true",
                   help="legacy alias for --lidar-backend none (dry-run only)")
    # grasp passthrough
    p.add_argument("--handoff-dist", type=float, default=None)
    p.add_argument("--nav-home-deg", type=str, default=None)
    p.add_argument("--grasp-home-deg", type=str, default=None)
    p.add_argument("--class-z", action="append", default=[],
                   help="YOLO class CENTROID-Z override, NAME=CENTROID_Z_M. Repeatable.")
    p.add_argument("--class-height", action="append", default=[],
                   help="YOLO class object HEIGHT (full z extent), NAME=HEIGHT_M. "
                        "Repeatable. Drives the grasp side's wrist_z_offset.")
    p.add_argument("--i-confirm-arm-cam-pose", action="store_true",
                   help="Acknowledge H_ARM/THETA_ARM were re-measured at the pose "
                        "detection runs from. --real refuses without it when that "
                        "pose is not the one they were calibrated at.")
    p.add_argument("--cam-x", type=float, default=vgp.CAM_TO_BASE_X)
    p.add_argument("--cam-y", type=float, default=vgp.CAM_TO_BASE_Y)
    p.add_argument("--sign-y", type=float, choices=(-1.0, 1.0), default=vgp.SIGN_Y)
    p.add_argument("--i-confirm-camera-frame", action="store_true",
                   help="confirm Phase-3 camera->PPO base_link mapping was measured")
    return p.parse_args()


def main():
    args = parse_args()
    if args.selftest:
        run_selftest()
        return
    if args.max_retries <= 0:
        raise SystemExit("--max-retries must be > 0")
    if args.max_steps <= 0:
        raise SystemExit("--max-steps must be > 0")
    if not math.isfinite(args.nav_stop_dist) or args.nav_stop_dist <= 0.0:
        raise SystemExit("--nav-stop-dist must be finite and > 0")
    if args.control_period is not None and (
            not math.isfinite(args.control_period) or args.control_period <= 0.0):
        raise SystemExit("--control-period must be finite and > 0")
    if args.motor_delay is not None and args.motor_delay < 0:
        raise SystemExit("--motor-delay must be >= 0")
    if not 1 <= args.ros_port <= 65535:
        raise SystemExit("--ros-port must be in 1..65535")
    if not args.scan_topic.startswith("/"):
        raise SystemExit("--scan-topic must be an absolute ROS topic")
    if (not args.no_lidar and args.lidar_backend == "rplidar"
            and not args.lidar_port):
        raise SystemExit("--lidar-backend rplidar requires an explicit --lidar-port")
    if args.handoff_dist is not None and (
            not math.isfinite(args.handoff_dist) or args.handoff_dist <= 0.0):
        raise SystemExit("--handoff-dist must be finite and > 0")
    if not all(math.isfinite(v) for v in (args.cam_x, args.cam_y, args.sign_y)):
        raise SystemExit("--cam-x/--cam-y/--sign-y must be finite")
    if args.real and not args.nav_only and not args.i_confirm_camera_frame:
        raise SystemExit(
            "real grasp refused: pass calibrated --cam-x/--cam-y/--sign-y "
            "and --i-confirm-camera-frame"
        )
    run_pipeline(args)


if __name__ == "__main__":
    main()
