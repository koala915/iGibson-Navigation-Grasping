#!/usr/bin/env python3
"""Full mission: patrol the map, spot trash, drive to it, grasp it, bin it, resume.

One Python 3.8 process owns /dev/myserial and runs everything that touches the
robot (SETMOTOR_ODOM_INTEGRATION.md section 2.1). ROS Melodic runs only the
sensor/localisation side and must NOT open the chassis serial port:

    this process (sole /dev/myserial owner)      ROS Melodic (Python 2.7)
      GraspController -> servos (v21)              robot_state_publisher
      set_car_motion  -> wheels                    YDLIDAR TG30 -> /scan
      get_motion_data -> /odom_setmotor + TF       map_server -> /map
      nav PPO 55D->2D, grasp PPO 28D->6D           AMCL -> map->odom
      MissionFSM                                   rosbridge_server

Pieces, all separately testable:

    map_goal_provider.py  route.yaml + AMCL pose -> (dist, bearing)
    feedback_odom.py      get_motion_data()      -> odom pose
    ros_io.py             rosbridge publish/subscribe
    mission_fsm.py        the state machine
    nav_rl.py             55D obs, training plant, 48-ray lidar, safety brake
    vision_grasp_pipeline.py  cameras, YOLO, fine align, latch, verify
    grasp/v21/            the grasp policy that actually picks things up

Run:
    python3 integration/mission_pipeline.py --selftest        # no hw, no torch
    python3 integration/mission_pipeline.py --dry-run --route <route.yaml>

Real integrated motion is intentionally refused before any device is opened.
The final alignment/latch path still uses predicted C3 trigonometric geometry;
it must consume a measured grasp-home homography before --real can be restored.

HARDWARE PREREQUISITES
    * NOTHING else may hold /dev/myserial: no rosmaster_main.py, no
      Mcnamu_driver.py, no ai_motor_server_B.py, no Route A runtime node, no
      port-7000 motor server.  Check with: sudo fuser -v /dev/myserial
    * ROS side up: TG30 driver, robot_state_publisher, map_server, AMCL,
      rosbridge_server.  Set the AMCL initial pose in RViz with a TIGHT
      estimate (std 0.15 m / 7 deg) -- broad initialisation was measured
      jumping 1.573 m in the repeated corridor and does not converge.
    * Camera streams on :8080.
"""
from __future__ import annotations

import argparse
import dataclasses
import math
import os
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

try:  # direct script execution
    import nav_rl as nr
    import nav_rl_grasp_pipeline as nrgp
    import vision_grasp_pipeline as vgp
    import map_goal_provider as mgp
    import mission_fsm as mfsm
    import mission_status
    import ros_io
    import trash_target as tt
    from feedback_odom import FeedbackOdomConfig, FeedbackOdomReader
except ImportError:  # package import
    from . import nav_rl as nr
    from . import nav_rl_grasp_pipeline as nrgp
    from . import vision_grasp_pipeline as vgp
    from . import map_goal_provider as mgp
    from . import mission_fsm as mfsm
    from . import mission_status
    from . import ros_io
    from . import trash_target as tt
    from .feedback_odom import FeedbackOdomConfig, FeedbackOdomReader

# ── frame offsets, measured from the deployed FK and the URDF, not assumed ──
# The grasp policy frame is the URDF loaded at deploy_contract.URDF_TO_TRAINING_FRAME,
# so its origin does NOT sit on base_footprint. Measured with grasp/v21's own
# FKComputer (2026-08-04):
#
#   policy_x = base_footprint_x + 0.0199
#   arm_link1  -> policy 0.1181  = 0.0982 m ahead of base_footprint (URDF: 0.09825)
#   front axle -> base_link +0.08 = 0.080 m ahead of base_footprint
#   rear axle  -> base_link -0.08
#
# base_footprint is therefore the middle of the CHASSIS (midway between the two
# axles), not the middle of the front axle: front and rear wheels sit at +-0.08.
POLICY_X_OF_BASE_FOOTPRINT = 0.0199   # m, policy-frame x of base_footprint
FRONT_AXLE_FROM_FOOTPRINT = 0.08      # m, front wheel axle ahead of base_footprint

# ── the two arm homes (docs/calibration/arm_pose.md) ──
# Driving and grasping want different arm poses, and v21 collapsed them: its
# DeployConfig.home_deg IS the trained C3 grasp pose and grasp_home_deg defaults
# to None. That is right for a standalone grasp run, which starts already parked
# at the object, and wrong for a robot that patrols 58 m first -- at C3 the
# gripper sits 11.3 cm beyond the chassis front at 11.1 cm above the floor,
# below the 19.2 cm lidar plane, so nothing in the obstacle stack can see what
# it is about to hit.
#
# v21 anticipated this: run() takes its starting pose from grasp_home_deg when
# set, and _scripted_lift_and_return brings the arm back to home_deg afterwards,
# so setting both makes the object travel home at the nav pose too.
NAV_HOME_DEG = (90.0, 140.0, 0.0, 0.0, 90.0, 30.0)      # docs/calibration/arm_pose.md travel pose
GRASP_HOME_DEG = (90.0, 67.08, 9.79, 9.79, 90.0, 30.0)  # C3, the trained pose

ODOM_RATE_HZ = 20.0           # Route A's measured /odom_setmotor rate
DET_INTERVAL_S = 0.5          # YOLO cadence (Jetson-friendly), as in nav_rl_grasp
LOG_PERIOD_S = 1.0
AUTO_REPORT_WAIT_S = 3.0      # how long to wait for the board to start reporting
INVESTIGATE_WZ = 0.5          # rad/s, slow confirm turn
INVESTIGATE_VX = 0.10         # m/s, creep forward while confirming


# ════════════════════════════════════════════════════════════════════════════
# Odometry publisher thread
# ════════════════════════════════════════════════════════════════════════════

class OdomPublisher(threading.Thread):
    """Poll wheel feedback at 20 Hz and publish /odom_setmotor + TF.

    Runs in its own thread so odometry keeps flowing while a blocking action
    (the grasp policy, the place sequence) holds the main loop. That is safe
    without a serial lock because ``Rosmaster.get_motion_data()`` only reads
    the values the board's receive thread already cached -- it performs no
    serial I/O of its own, unlike every set_* call.
    """

    def __init__(self, reader: FeedbackOdomReader, rio, rate_hz: float = ODOM_RATE_HZ):
        super().__init__(name="odom-publisher", daemon=True)
        self.reader = reader
        self.rio = rio
        self.period = 1.0 / float(rate_hz)
        # ``threading.Thread.join()`` calls its private ``_stop()`` method.
        # Reusing that name for an Event makes every normal shutdown end in
        # TypeError after the odometry thread exits.
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._state = None
        self.ticks = 0
        self.publish_failures = 0

    def run(self) -> None:
        while not self._stop_event.is_set():
            t0 = time.time()
            try:
                state = self.reader.poll(now=t0)
                with self._lock:
                    self._state = state
                    self.ticks += 1
                self.rio.publish_odom_and_tf(state, self.reader.odom.covariance(),
                                             stamp=t0)
            except Exception as exc:                  # never kill the thread
                self.publish_failures += 1
                if self.publish_failures in (1, 10, 100):
                    print(f"[mission][odom] publish failed ({self.publish_failures}x): {exc}")
            slept = self.period - (time.time() - t0)
            if slept > 0:
                self._stop_event.wait(slept)

    def state(self, now: Optional[float] = None):
        with self._lock:
            state = self._state
        if state is None:
            return None
        # A publisher thread can die after storing a fresh snapshot. Consumers
        # must age the snapshot at read time rather than trusting the boolean it
        # carried when it was produced.
        now = time.time() if now is None else float(now)
        age = now - float(state.stamp)
        fresh = bool(state.valid and state.stamp > 0.0
                     and 0.0 <= age <= self.reader.odom.cfg.feedback_timeout_s)
        if fresh == state.fresh and (fresh or not state.stationary):
            return state
        reason = state.reason
        if state.valid and not fresh:
            reason = (f"odometry publisher snapshot stale ({age:.2f}s)"
                      if age >= 0.0 else "odometry publisher clock moved backwards")
        return dataclasses.replace(state, fresh=fresh,
                                   stationary=bool(state.stationary and fresh),
                                   reason=reason)

    def stop(self) -> None:
        self._stop_event.set()


# ════════════════════════════════════════════════════════════════════════════
# Navigator: one RL control tick, usable with any goal source
# ════════════════════════════════════════════════════════════════════════════

class NavTick(tuple):
    """(vx, wz, braked, stall, min_ray, front_raw)"""
    __slots__ = ()

    def __new__(cls, vx, wz, braked, stall, min_ray, front_raw):
        return tuple.__new__(cls, (vx, wz, braked, stall, min_ray, front_raw))

    vx = property(lambda s: s[0])
    wz = property(lambda s: s[1])
    braked = property(lambda s: s[2])
    stall = property(lambda s: s[3])
    min_ray = property(lambda s: s[4])
    front_raw = property(lambda s: s[5])


class MissionNavigator(nrgp.RLNavigator):
    """RLNavigator split into single ticks so the FSM owns the loop.

    Inherits _drive_raw, _detect_rear, _detect_arm, _arm_align, latch_arm_object,
    verify_grasp and back_off unchanged -- this class adds no new geometry, so
    the calibrated v21 vision path is exactly the one already under test.
    """

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.reset_nav()

    def reset_nav(self, clear_tracker: bool = True) -> None:
        """Clear every carry-over aimed at the previous goal.

        A fresh ActionDelay rather than a cleared one, so this is also the
        constructor path: there is no window in which the buffer exists but
        still holds commands meant for a target we have abandoned.

        ``clear_tracker`` is separate because the two things reset for opposite
        reasons. The ActionDelay always has to go: it holds commands computed
        for the old goal. The GoalTracker must NOT go when the camera target is
        what we are switching TO -- on PATROL -> INVESTIGATE the tracker holds
        the very fix that triggered the transition, and throwing it away leaves
        the next tick with fix_age = infinity, which reads as "target lost" and
        sends the robot straight back to patrol. Clear it only when entering a
        map-goal state, where the camera target is genuinely irrelevant.
        """
        self.delay = nr.ActionDelay(self.ncfg.motor_delay_steps)
        self.prev_action = np.zeros(2, dtype=np.float32)
        self.last_vx = self.last_wz = 0.0
        if clear_tracker:
            self.tracker = nr.GoalTracker()

    def nav_tick(self, goal_dist: float, goal_bearing: float, dt: float,
                 points) -> NavTick:
        """One 6 Hz policy step. Mirrors RLNavigator._rl_navigate exactly."""
        cfg = self.ncfg
        rays = nr.scan_to_rays(points, cfg)
        obs = nr.build_nav_obs(goal_dist, goal_bearing, abs(self.last_vx),
                               self.last_wz, self.prev_action, rays)
        action = self.policy.predict(obs)
        executed = self.delay.push(action)
        vx, wz, stall = nr.shape_action(executed, obs, cfg)

        # Geometric brake on RAW points: the 48-ray obs is floored at 0.33 m and
        # is blind below it. The policy alone still collides in 20-27% of sim
        # episodes, so this is not optional.
        front = nr.front_min_raw(points, cfg)
        braked = front < cfg.safety_brake_dist and vx > 0.0
        if braked:
            vx = 0.0

        self._drive_raw(vx, wz)
        self.tracker.predict(vx, wz, dt)
        self.prev_action = action
        self.last_vx, self.last_wz = vx, wz
        return NavTick(vx, wz, braked, stall, float(np.min(rays)), front)

    def creep(self, vx: float, wz: float, dt: float) -> None:
        """Open-loop slow motion for INVESTIGATE. Not policy-driven, so the
        ActionDelay is left untouched and reset before the policy resumes."""
        self._drive_raw(vx, wz)
        self.tracker.predict(vx, wz, dt)
        self.last_vx, self.last_wz = vx, wz


# ════════════════════════════════════════════════════════════════════════════
# Mission runner
# ════════════════════════════════════════════════════════════════════════════

class MissionRunner:
    def __init__(self, *, nav: MissionNavigator, controller, goals: mgp.MapGoalProvider,
                 rio, odom_pub: Optional[OdomPublisher], lidar,
                 fsm: mfsm.MissionFSM, args):
        self.nav = nav
        self.controller = controller
        self.goals = goals
        self.rio = rio
        self.odom_pub = odom_pub
        self.lidar = lidar
        self.fsm = fsm
        self.args = args

        self.started = not args.wait_start
        self.self_check_passed = False
        self.operator_cleared = False
        self.fault = ""
        # Off unless --status-udp was given, in which case every tick becomes a
        # datagram an operator console can subscribe to.  Constructed here so
        # the two operator prompts can announce themselves before they block.
        self.status = mission_status.StatusEmitter(
            getattr(args, "status_udp", None),
            period=getattr(args, "status_period", mission_status.DEFAULT_PERIOD_S))
        self._last_det = 0.0
        self._det_streak = 0
        # getattr so the selftests, which build runners from partial arg
        # namespaces, keep working and default to the onboard camera.
        self._target_source = getattr(args, "target_source", "onboard")
        self._trash_max_age_s = getattr(args, "trash_max_age",
                                        tt.DEFAULT_MAX_AGE_S)
        self._last_log = 0.0
        self._t_prev = time.time()
        self._latched: Optional[Tuple[list, Optional[float], Optional[float]]] = None
        self._grasp_finished = False
        self._grasp_controller_confirmed = False
        self._grasp_verified = False
        self._place_finished = False
        self._arm_at_home = True
        # Unknown until the self-check parks it; assumed False so the first
        # self-check always issues the move rather than trusting a leftover pose.
        self._at_nav_home = False
        self._align_failed = False
        self._handoff_ready = False
        self._blacklist: list = []

    def _amcl_quality(self) -> Tuple[bool, str]:
        """Apply the same local-age gate in self-check and every mission tick.

        A few offline fakes still expose the pre-age ``pose_quality_ok()``
        signature.  Keep those tests usable while the real RosBridgeIO always
        receives the configured age limit.
        """
        max_age = getattr(self.args, "amcl_max_age", ros_io.DEFAULT_MAX_AMCL_AGE_S)
        try:
            return self.rio.pose_quality_ok(max_age_s=max_age)
        except TypeError:
            return self.rio.pose_quality_ok()

    def _keep_amcl_fresh(self) -> None:
        """Keep a map fix alive while the arm works and the base stands still.

        AMCL updates on motion, so a stationary robot stops publishing. GRASP
        alone can hold still for 120 s, and the state right after it (DELIVER)
        treats a stale fix as a blocking fault -- so without this, the first
        successful grasp ended the mission in PAUSED, and the operator's Enter
        re-ran a self-check that failed on the same stale pose.
        """
        request = getattr(self.rio, "request_nomotion_update", None)
        if request is None:
            return
        try:
            request()
        except Exception as exc:                  # never take the loop down
            print(f"[mission] nomotion update failed: {exc}")

    def _recover_map_fix(self, blocked_for: float = 0.0) -> bool:
        """Re-acquire a map fix after standing still, before a map state needs it."""
        refresh = getattr(self.rio, "refresh_pose", None)
        if refresh is None:
            return True
        try:
            ok = refresh(timeout_s=self.args.amcl_refresh_timeout)
        except Exception as exc:
            print(f"[mission] AMCL refresh failed: {exc}")
            return False
        if not ok:
            where = (f"the arm held the loop for {blocked_for:.0f}s and "
                     if blocked_for > 0.0 else "")
            print(f"[mission][WARN] {where}AMCL did not refresh within "
                  f"{self.args.amcl_refresh_timeout:.1f}s. Is "
                  f"{ros_io.NOMOTION_SERVICE} advertised? A stationary robot "
                  "cannot get a fresh fix without it.")
        return ok

    def _check_amcl_refresh_path(self) -> bool:
        """Confirm something can refresh AMCL while the robot stands still.

        Reports "cannot verify" and "not there" as different things: rosapi may
        simply not be running, which is not evidence that the service is absent.
        """
        listing = getattr(self.rio, "rosapi_list", lambda *_a, **_k: None)
        try:
            services = listing("services")
        except Exception:
            services = None
        if services is None:
            print(f"[mission][check] {'FAIL' if self.args.real else 'warn'}: cannot "
                  f"verify {ros_io.NOMOTION_SERVICE} (is rosapi running alongside "
                  "rosbridge?). Without it a stationary robot's fix goes stale and "
                  "the first map state after a grasp pauses.")
            return not self.args.real
        if ros_io.NOMOTION_SERVICE not in services:
            print(f"[mission][check] {'FAIL' if self.args.real else 'warn'}: "
                  f"{ros_io.NOMOTION_SERVICE} is not advertised — AMCL is not "
                  "running, or is not the localiser. A grasp takes the base out "
                  "of motion for minutes and the fix cannot recover.")
            return not self.args.real
        print(f"[mission][check] {ros_io.NOMOTION_SERVICE} available "
              "(stationary fixes stay fresh)")
        nodes = None
        try:
            nodes = listing("nodes")
        except Exception:
            pass
        if nodes is not None and "/amcl_nomotion_keepalive" in nodes:
            print("[mission][check] Route B keepalive node is also running "
                  "(harmless: this process refreshes on demand)")
        return True



    # ── self check ──
    def run_self_check(self) -> bool:
        ok = True
        dev = self.nav.device
        if self.args.real and dev is None:
            print("[mission][check] FAIL: no Rosmaster device")
            return False

        # The board only streams motion feedback when auto-report is on. v21's
        # ServoController calls create_receive_threading() but not this, so
        # without it get_motion_data() returns its initial zeros forever and the
        # odometry would look like a robot that never moves -- which AMCL
        # believes, silently.
        if dev is not None:
            try:
                dev.set_auto_report_state(True, False)
                print("[mission][check] auto-report enabled")
            except Exception as exc:
                print(f"[mission][check] FAIL: set_auto_report_state: {exc}")
                return False
            deadline = time.time() + AUTO_REPORT_WAIT_S
            live = False
            while time.time() < deadline:
                try:
                    if float(dev.get_battery_voltage()) > 5.0:
                        live = True
                        break
                except Exception:
                    pass
                time.sleep(0.1)
            if live:
                print("[mission][check] board is reporting (battery telemetry seen)")
            else:
                print("[mission][check] FAIL: no telemetry within "
                      f"{AUTO_REPORT_WAIT_S:.0f}s — odometry would be fake zeros")
                ok = False

        if self.odom_pub is not None:
            deadline = time.time() + 2.0
            while time.time() < deadline and self.odom_pub.ticks < 3:
                time.sleep(0.1)
            st = self.odom_pub.state()
            if st is None or not st.valid:
                reason = "no samples" if st is None else st.reason
                print(f"[mission][check] {'FAIL' if self.args.real else 'warn'}: "
                      f"wheel feedback not valid ({reason})")
                ok = ok and not self.args.real
            else:
                print("[mission][check] wheel feedback valid")

        if self.lidar.age() > self.nav.ncfg.lidar_stale_timeout_s:
            print("[mission][check] FAIL: /scan stale — is the TG30 driver running?")
            ok = False
        else:
            print(f"[mission][check] /scan fresh ({len(self.lidar.get_points())} points)")
            # Coverage is only a metadata check.  A full 360-degree scan covers
            # both front and rear, so it cannot prove the physical meaning of a
            # raw index.  The Route B four-direction evidence gate does that.
            info = getattr(self.lidar, "scan_info", lambda _c: None)(self.nav.ncfg)
            if info:
                print(f"[mission][check] scan frame={info['frame_id']!r} "
                      f"window [{info.get('angle_min_deg', float('nan')):.0f}, "
                      f"{info.get('angle_max_deg', float('nan')):.0f}] deg, "
                      f"forward covered {info.get('forward_fraction', 0.0)*100:.0f}%")
                for w in info["warnings"]:
                    print(f"[mission][check] {'FAIL' if self.args.real else 'warn'}: {w}")
                if info["warnings"]:
                    ok = ok and not self.args.real
                if self.args.real:
                    evidence = getattr(self.args, "lidar_orientation_evidence", "")
                    verified, why = nr.validate_orientation_evidence(
                        evidence, self.nav.ncfg, info)
                    if not verified:
                        print(f"[mission][check] FAIL: LiDAR orientation evidence: {why}")
                        ok = False
                    else:
                        print(f"[mission][check] {why}")
            elif self.args.real:
                print("[mission][check] FAIL: no live scan metadata for orientation gate")
                ok = False

        # The mission keeps its own fix alive rather than relying on Route B's
        # keepalive node, which route_b_startup.sh starts only in start_dryrun.
        # That only works if AMCL actually offers the service, so confirm it
        # here -- silently having no way to refresh is how a stationary robot
        # ends up paused with a stale pose it cannot recover from.
        ok = self._check_amcl_refresh_path() and ok

        self._recover_map_fix(0.0)
        pose = self.rio.latest_pose()
        good, why = self._amcl_quality()
        if pose is None or not good:
            print(f"[mission][check] {'FAIL' if self.args.real else 'warn'}: "
                  f"AMCL not usable ({why}). Set a TIGHT 2D Pose Estimate in RViz.")
            ok = ok and not self.args.real
        else:
            self.goals.set_pose(pose)
            idx = self.goals.nearest_index(pose.x, pose.y)
            self.goals.reset(start_index=idx)
            wp = self.goals.current_waypoint()
            print(f"[mission][check] AMCL ok at ({pose.x:.2f}, {pose.y:.2f}); "
                  f"patrol starts at {wp.id} ({wp.x:.2f}, {wp.y:.2f})")

        # Park the arm at the travel pose before anything is allowed to drive.
        # Without this the arm starts wherever the last run left it -- which
        # after a completed grasp is C3, gripper outside the chassis.
        if ok and not self._at_nav_home:
            print("[mission][check] moving the arm to the nav travel pose "
                  f"{list(self.controller.cfg.home_deg)}")
            holding = self._grasp_verified and not self._place_finished
            if self.move_arm_to(self.controller.cfg.home_deg, "nav-home",
                                holding=holding):
                self._at_nav_home = True
            else:
                print(f"[mission][check] {'FAIL' if self.args.real else 'warn'}: "
                      f"the arm did not reach the travel pose")
                ok = ok and not self.args.real

        # _at_nav_home is set only after a verified move (or a successful v21
        # return-home sequence), so it is also authoritative for the FSM's
        # arm_at_home gate.  This matters after clearing a pause in CARRY_HOME:
        # without restoring the second flag, self-check parks the arm safely
        # but CARRY_HOME waits forever and pauses again.
        if ok and self._at_nav_home:
            self._arm_at_home = True

        self.self_check_passed = ok
        return ok

    # ── arm poses ──
    def move_arm_to(self, api_deg, label: str, *, holding: bool = False) -> bool:
        """Guarded, verified move to a six-value API-degree pose.

        Always through move_guarded_and_verified, never send_degrees: a bare
        send is capped at max_delta_deg (8 deg) per call, is not floor-swept per
        iteration, and updates the controller's internal pose to a target the
        arm may never have reached.
        """
        ctl = self.controller
        try:
            arm_rad = ctl.mapper.hw_deg_to_sim_arm(list(api_deg[:5]))
            grip_rad = (float(ctl._current_grip_rad) if holding
                        else ctl.mapper.hw_deg_to_sim_grip(float(api_deg[5])))
            res = ctl.move_guarded_and_verified(
                arm_rad, grip_rad, label=label, run_time_ms=400, settle_s=0.35,
                grip_is_hold=holding)
            if not res.get("reached"):
                print(f"[mission] arm move '{label}' did not arrive: {res.get('reason')}")
                return False
            return True
        except Exception as exc:
            print(f"[mission] arm move '{label}' raised: {exc}")
            return False

    # ── perception ──
    def _read_detection(self) -> Tuple[bool, float, float]:
        """``(found, dist_front, offset)`` from whichever source was selected.

        Both sources answer the same question -- where is the object, in metres
        ahead and metres to the RIGHT -- so the consistency gate, the
        blacklist and the FSM below are identical either way. The only thing
        that changes is who measured it.

        ``offboard`` is the SAM2 publisher on a development machine. If that
        link dies the adapter reports "not found" rather than the last thing it
        said, so the robot keeps patrolling instead of driving at a stale fix.
        """
        # getattr, not self._target_source: the selftests exercise single
        # methods on runners built without __init__, and the onboard camera is
        # the right default for a runner that never chose a source.
        if getattr(self, "_target_source", "onboard") == "offboard":
            target = self.rio.latest_trash_target()
            max_age = getattr(self, "_trash_max_age_s", tt.DEFAULT_MAX_AGE_S)
            return tt.to_rear_detection(target, time.time(), max_age_s=max_age)
        return self.nav._detect_rear()

    def _run_detection(self, now: float) -> None:
        """Rear-camera YOLO during patrol, with a spatial consistency gate.

        Counting bare detections is not enough. ``_detect_rear`` reports a
        distance and a lateral offset but no track id, so three frames in a row
        could be three different things -- or the same flicker appearing at
        random places in the image. Either would pull the robot off its route.
        Requiring successive fixes to land near each other is the check that is
        actually available, and it costs nothing.
        """
        if now - self._last_det < DET_INTERVAL_S:
            return
        self._last_det = now
        try:
            found, dist_f, off = self._read_detection()
        except Exception as exc:
            print(f"[mission][det] rear detection failed: {exc}")
            # An exception is a missed frame, not part of a consecutive streak.
            self._det_streak = 0
            return
        if not (found and dist_f > 0):
            self._det_streak = 0
            return

        blocked = self._blacklisted_here()
        if blocked is not None:
            # Seen, but this is somewhere we already failed. Keep patrolling.
            if self._det_streak != 0:
                print(f"[mission][det] ignoring a detection {blocked:.2f} m from a "
                      f"previous give-up point")
            self._det_streak = 0
            return

        # The robot keeps driving between detections, so the previous RAW fix is
        # not where the object should be now. Compare against the tracker, which
        # has been dead-reckoning that fix forward with the commanded motion --
        # that is the prediction the new detection has to agree with.
        tracker = self.nav.tracker
        if tracker.has_fix and self._det_streak > 0:
            d, b = tracker.dist(), tracker.bearing()
            pred_forward = d * math.cos(b)
            pred_right = -d * math.sin(b)      # tracker y is LEFT-positive
            moved = math.hypot(dist_f - pred_forward, off - pred_right)
            if moved > self.args.detection_jump_m:
                print(f"[mission][det] fix is {moved:.2f} m from where the tracker "
                      f"predicted (> {self.args.detection_jump_m:.2f}) — "
                      f"restarting the streak")
                self._det_streak = 1
            else:
                self._det_streak += 1
        else:
            self._det_streak = 1
        tracker.update(dist_f, off)

    def _refresh_close_fix(self, now: float) -> None:
        """Once inside ~0.9 m the arm camera is the better source (as in
        RLNavigator._rl_navigate)."""
        if now - self._last_det < DET_INTERVAL_S:
            return
        self._last_det = now
        try:
            found, dist_f, off = self.nav._detect_rear()
            if found and dist_f > 0:
                self.nav.tracker.update(dist_f, off)
                return
            # The arm camera is only usable at C3. While driving, the arm is at
            # the nav travel pose, where the same ground model still returns a
            # plausible distance -- silently wrong by tens of centimetres. Rear
            # camera plus the tracker's dead reckoning covers the last stretch
            # instead; the raise to C3 happens once, at the align handoff.
            if (not self._at_nav_home and self.nav.tracker.has_fix
                    and self.nav.tracker.dist() < 0.9):
                found_a, dist_a, off_a, _w, _c = self.nav._detect_arm()
                if found_a:
                    self.nav.tracker.update(dist_a, off_a)
        except Exception as exc:
            # A detection failure must not kill the control loop: the tracker
            # keeps dead-reckoning and the FSM's fix-age timeout is what decides
            # whether the target is really gone.
            print(f"[mission][det] close-range detection failed: {exc}")

    # ── sensing ──
    def _sense(self, now: float) -> mfsm.Sense:
        self._keep_amcl_fresh()
        odom = self.odom_pub.state() if self.odom_pub is not None else None
        pose = self.rio.latest_pose()
        amcl_good, _ = self._amcl_quality()
        if pose is not None and amcl_good:
            self.goals.set_pose(pose)

        # Evaluate the goal fix before constructing Health.  MapGoalProvider
        # has its own pose/route-age guard (normally shorter than the global
        # AMCL window); if that guard rejects the fix, leaving amcl_ok=True
        # would make PATROL/DELIVER stop the base but remain in the driving
        # state.  Marking health bad lets the FSM enter PAUSED with the real
        # reason and requires a fresh map-localized fix before resuming.
        fix = self.goals.get(now)
        amcl_usable = bool(pose is not None and amcl_good)
        if self.fsm.state in mfsm.MAP_STATES and not fix.valid:
            amcl_usable = False

        scan_fresh = self.lidar.age() <= self.nav.ncfg.lidar_stale_timeout_s
        health = mfsm.Health(
            serial_ok=(self.nav.device is not None) or not self.args.real,
            odom_valid=bool(odom.valid) if odom is not None else (not self.args.real),
            odom_fresh=bool(odom.fresh) if odom is not None else (not self.args.real),
            scan_fresh=scan_fresh,
            amcl_ok=amcl_usable,
            estop=False,
            fault=self.fault,
        )
        return mfsm.Sense(
            now=now,
            health=health,
            started=self.started,
            self_check_passed=self.self_check_passed,
            operator_cleared=self.operator_cleared,
            waypoint_reached=self.goals.arrived(fix, now),
            detection_streak=self._det_streak,
            target_visible=self.nav.tracker.has_fix,
            target_dist=self.nav.tracker.dist(),
            target_fix_age=self.nav.tracker.fix_age(now),
            handoff_ready=self._handoff_ready,
            align_failed=self._align_failed,
            stationary=bool(odom.stationary) if odom is not None else True,
            arm_at_home=self._arm_at_home,
            grasp_finished=self._grasp_finished,
            grasp_verified=self._grasp_verified,
            latched=self._latched is not None,
            bin_reached=self.goals.arrived(fix, now) and self.goals.in_override,
            place_finished=self._place_finished,
        )

    # ── envelope gate ──
    def envelope_ok(self, obj_pos) -> Tuple[bool, str]:
        """Is the object where the v21 policy was actually trained to grasp?

        The documented region is x 0.20-0.28 m, y -0.09..0.08 m in the arm base
        frame -- a box roughly 8 cm by 17 cm. The arm camera can see 0.205-0.317
        m, i.e. it overhangs at the far end, so a detection alone is not proof
        the policy can act on it. v21 re-checks this itself before running; this
        early copy exists so ALIGN keeps servoing instead of handing over a
        target that would be refused.
        """
        cfg = self.controller.cfg
        x, y = float(obj_pos[0]), float(obj_pos[1])
        xlo, xhi = cfg.documented_x_range
        ylo, yhi = cfg.documented_y_range
        tol = cfg.envelope_tol_m
        if not (xlo - tol <= x <= xhi + tol):
            return False, (f"x={x:.3f} outside documented [{xlo:.2f}, {xhi:.2f}] "
                           f"({self.describe_frames(obj_pos)})")
        if not (ylo - tol <= y <= yhi + tol):
            return False, f"y={y:+.3f} outside documented [{ylo:+.2f}, {yhi:+.2f}]"

        # Second, tighter gate: how far is it from where we are aiming? The
        # envelope is a box the policy was evaluated over; this is the "close
        # enough to hand over" tolerance, and it is the one to tune on the robot.
        aim = self.args.handoff_aim_x
        r = math.hypot(x - aim, y)
        if r > self.args.handoff_radius_m:
            return False, (f"{r*100:.1f} cm from the aim point "
                           f"(x={aim:.3f}, y=0) > {self.args.handoff_radius_m*100:.1f} cm "
                           f"({self.describe_frames(obj_pos)})")
        return True, ""

    @staticmethod
    def describe_frames(obj_pos) -> str:
        """The same point in all three frames people quote it in.

        Getting these confused is a 10-20 cm error that looks like a bad grasp,
        so every gate message carries all three rather than a bare number.
        """
        x, y = float(obj_pos[0]), float(obj_pos[1])
        bf = x - POLICY_X_OF_BASE_FOOTPRINT
        axle = bf - FRONT_AXLE_FROM_FOOTPRINT
        return (f"policy x={x:.3f} | base_footprint x={bf:.3f} | "
                f"front-axle x={axle:.3f} | y={y:+.3f} m")

    def _fine_align_blocking_reason(self) -> str:
        """Re-check fail-safe inputs inside the blocking visual servo loop."""
        if self.args.real and self.nav.device is None:
            return "serial/driver lost"
        if self.odom_pub is not None:
            odom = self.odom_pub.state()
            if odom is None or not odom.valid:
                return "wheel feedback invalid"
            if not odom.fresh:
                return "wheel feedback stale"
        if self.lidar.age() > self.nav.ncfg.lidar_stale_timeout_s:
            return "/scan stale"
        return ""

    # ── acting ──
    def _act(self, tr: mfsm.Transition, now: float, dt: float) -> None:
        A = mfsm.Action
        act = tr.action

        # The clear is a one-shot. Leaving it latched would make every later
        # PAUSED clear itself on the tick it happened.
        if self.operator_cleared and tr.state is not mfsm.State.PAUSED:
            self.operator_cleared = False

        if tr.reset_nav:
            # Entering a map-goal state means the camera target no longer
            # matters; entering a camera-goal state means it is the whole point.
            self.nav.reset_nav(clear_tracker=tr.state in mfsm.MAP_STATES)

        if act in (A.STOP, A.SETTLE, A.WAIT_OPERATOR, A.HOLD, A.RUN_SELF_CHECK):
            self.nav.stop()
            if act is A.RUN_SELF_CHECK and not self.self_check_passed:
                self.run_self_check()
            elif act is A.WAIT_OPERATOR and not self.started:
                self._await_operator()
            return

        if act is A.DRIVE_BIN and not self.goals.in_override:
            # Entering DELIVER: point the goal provider at the bin. Without this
            # the "drive to the bin" action drives to whatever patrol waypoint
            # was current, and the arrival test passes at the wrong place.
            self.goals.set_bin_target()
            bx, by, byaw, _ = self.goals.target()
            print(f"[mission] bin approach target: ({bx:.2f}, {by:.2f}, "
                  f"yaw={byaw if byaw is None else round(byaw, 2)}); "
                  f"patrol parked at index {self.goals.interrupted_index}")

        if act is A.DRIVE_PATROL or act is A.DRIVE_BIN:
            fix = self.goals.get(now)
            if not fix.valid:
                self.nav.stop()
                return
            self.nav.nav_tick(fix.dist, fix.bearing, dt, self.lidar.get_points())
            if act is A.DRIVE_PATROL:
                self._run_detection(now)
            return

        if act is A.RESUME_PATROL:
            if tr.state is mfsm.State.RESUME:
                # Done with this object -- delivered, or given up on. Every flag
                # about it has to go. Leaving them set is not cosmetic: a stale
                # `_latched` makes Sense.latched true on the FIRST tick of the
                # next LATCH, so the FSM skips straight to GRASP and the arm
                # reaches for the PREVIOUS object's coordinates. A stale
                # `_align_failed` makes the next ALIGN abort instantly.
                self.nav.stop()
                # Back to the travel pose before driving off. After a completed
                # grasp or release v21 already returns to cfg.home_deg (now the
                # nav pose), but the give-up paths leave the arm at C3 with the
                # gripper outside the chassis.
                if not self._at_nav_home:
                    self._at_nav_home = self.move_arm_to(
                        self.controller.cfg.home_deg, "C3->nav-home")
                    if not self._at_nav_home:
                        self.fault = "arm did not return to the travel pose"
                        return
                if not self._grasp_verified:
                    # Gave up on this one. Remember WHERE, or the next patrol
                    # pass sees the same object, confirms it again, and the robot
                    # loops on an ungraspable thing forever. The FSM already says
                    # "blacklisting"; this is what makes that true.
                    self._blacklist_here()
                idx = (self.goals.resume_patrol() if self.goals.in_override
                       else self.goals.advance())
                self._reset_target_state()
            else:
                # A plain waypoint advance, 83 times a lap. Do NOT stop (that
                # would stutter the whole patrol at 6 Hz) and do NOT clear the
                # detection streak -- the robot can be two frames into
                # confirming an object exactly as it passes a waypoint.
                idx = self.goals.advance()
            print(f"[mission] next waypoint: {self.goals.route.waypoints[idx].id}")
            return

        if act is A.TURN_TO_TARGET:
            self._refresh_close_fix(now)
            if not self.nav.tracker.has_fix:
                self.nav.stop()
                return
            b = self.nav.tracker.bearing()
            wz = max(-INVESTIGATE_WZ, min(INVESTIGATE_WZ, 2.0 * b))
            vx = INVESTIGATE_VX if abs(b) < math.radians(15.0) else 0.0
            front = nr.front_min_raw(self.lidar.get_points(), self.nav.ncfg)
            if front < self.nav.ncfg.safety_brake_dist:
                vx = 0.0
            self.nav.creep(vx, wz, dt)
            return

        if act is A.DRIVE_TARGET:
            self._refresh_close_fix(now)
            if not self.nav.tracker.has_fix:
                self.nav.stop()
                return
            self.nav.nav_tick(self.nav.tracker.dist(), self.nav.tracker.bearing(),
                              dt, self.lidar.get_points())
            return

        if act is A.FINE_ALIGN:
            # Blocking, by design: the visual servo owns the base until it either
            # lands the object inside the trained envelope or gives up. Odometry
            # keeps publishing from its own thread throughout.
            self.nav.stop()
            # This is where the arm camera starts being used, so this is where
            # the arm has to be at C3: the extrinsics, the visible ground band
            # and the trained envelope are all defined at that pose, and the
            # distance model returns a plausible number at any pose, so nothing
            # downstream would catch a mismatch. Raising here (base already
            # stopped) also keeps the wheels-or-arm-never-both rule intact.
            remaining = max(
                0.0,
                self.fsm.cfg.align_timeout_s - self.fsm._elapsed(now),
            )
            align_deadline = time.monotonic() + remaining
            if self._at_nav_home:
                if not self.move_arm_to(self.controller.cfg.grasp_home_deg,
                                        "nav-home->C3"):
                    self.fault = "arm did not reach the grasp pose for alignment"
                    return
                self._at_nav_home = False
                self._arm_at_home = False
            ok = False
            try:
                ok = self.nav._arm_align(
                    deadline=align_deadline,
                    safety_check=self._fine_align_blocking_reason,
                )
            except Exception as exc:
                print(f"[mission] fine align raised: {exc}")
            self.nav.stop()
            if not ok:
                self._align_failed = True
                return
            try:
                obj_pos, width, height = self.nav.latch_arm_object()
            except Exception as exc:
                print(f"[mission] latch preview failed: {exc}")
                self._align_failed = True
                return
            good, why = self.envelope_ok(obj_pos)
            if not good:
                print(f"[mission] aligned but outside the v21 envelope: {why}")
                self._align_failed = True
                return
            if width is not None and width > vgp.MAX_GRASP_WIDTH_M:
                print(f"[mission] object too wide: {width * 100:.1f} cm > "
                      f"{vgp.MAX_GRASP_WIDTH_M * 100:.1f} cm — abandoning")
                self._align_failed = True
                return
            self._handoff_ready = True
            return

        if act is A.LATCH:
            self.nav.stop()
            try:
                self._latched = self.nav.latch_arm_object()
            except Exception as exc:
                print(f"[mission] latch failed: {exc}")
                self._latched = None
            return

        if act is A.RUN_GRASP:
            if self._latched is None:
                self.fault = "RUN_GRASP with nothing latched"
                return
            obj_pos, _width, height = self._latched
            self.nav.stop()
            self._arm_at_home = False
            # v21's obj_provider second element is the object's HEIGHT (drives
            # wrist_z_offset), never its width. Passing width here silently
            # ruins the engagement depth.
            self.controller.obj_provider = (lambda p=obj_pos, h=height: (p, h))
            try:
                self._grasp_controller_confirmed = bool(
                    self.controller.run(max_steps=self.args.max_steps))
                # run() returning False is a completed failed attempt, not an
                # in-progress episode.  VERIFY owns the retry decision.
                self._grasp_finished = True
            except Exception as exc:
                self.fault = f"grasp policy raised: {exc}"
                return
            if self._grasp_controller_confirmed:
                # run() completing means stage 2 ran, and stage 2 ends with
                # _scripted_lift_and_return putting the arm back at cfg.home_deg
                # -- the nav travel pose now that the two homes are split. Without
                # recording that, CARRY_HOME waits for an arm_at_home that never
                # arrives and PAUSES after every successful grasp.
                self._arm_at_home = True
                self._at_nav_home = True
            return

        if act is A.VERIFY:
            self.nav.stop()
            obj_pos = self._latched[0] if self._latched else None
            try:
                verification = (self.nav.verify_grasp(obj_pos)
                                if self._grasp_controller_confirmed
                                and obj_pos is not None else False)
                self._grasp_verified = verification is True
                if verification is None:
                    print("[mission] grasp verification UNKNOWN — no valid visual evidence")
            except Exception as exc:
                print(f"[mission] verify failed ({exc}) — treating as a failed grasp")
                self._grasp_verified = False
            if self._grasp_verified:
                print("[mission] grasp VERIFIED — object is off the floor")
            return

        if act is A.BACK_OFF:
            try:
                if not self.controller.move_home():
                    # The arm is somewhere unknown; reversing now could drag it
                    # through whatever it failed to clear.
                    self.fault = "arm did not reach home before the retry back-off"
                    return
                self._arm_at_home = True
                self._at_nav_home = True
            except Exception as exc:
                self.fault = f"move_home failed during retry: {exc}"
                return
            self.nav.back_off()
            self.nav.stop()
            self._latched = None
            self._grasp_finished = self._grasp_controller_confirmed = False
            self._grasp_verified = False
            self._handoff_ready = self._align_failed = False
            return

        if act is A.PLACE:
            self.nav.stop()
            self._place_finished = self._run_place()
            return

    def _announce(self, state, action, reason: str, *, waiting: str = "") -> None:
        """Publish a state the tick loop will not reach on its own.

        The loop publishes what the FSM returned; these are the moments the
        loop is about to stop running -- blocked on a human -- and so has no
        transition to report.
        """
        # getattr, not self.status: the selftests exercise single methods on
        # runners built without __init__, and a missing publisher must degrade
        # to silence rather than taking the caller down with it.
        status = getattr(self, "status", None)
        if status is None or not status.enabled:
            return
        # changed=True so the heartbeat throttle cannot drop it. This is the
        # last frame before the loop blocks on a human, and a console that goes
        # quiet without saying why looks exactly like one whose robot crashed.
        status.publish(mfsm.Transition(state, action, reason, changed=True),
                       waiting=waiting, laps=getattr(self.goals, "laps", 0))

    def _await_operator(self) -> None:
        """Block in IDLE until a human says go.

        The self-check has passed, which means the wheels are live and the very
        next state drives. Starting that the instant the process launches is a
        surprise nobody wants standing next to the robot. Falls through if there
        is no terminal (cron, nohup, a pipe), the mission stays stopped and
        enters FAULT; loss of stdin is not operator consent.
        """
        print("\n" + "=" * 60)
        print("  Self-check passed. The robot will start PATROLLING.")
        print("  Clear the area, then press Enter (Ctrl+C to abort).")
        print("=" * 60)
        # Say so before blocking. The tick loop is about to stop publishing, and
        # a console that went quiet with no explanation looks like a crash --
        # exactly when a human is standing next to a robot deciding whether to
        # hit the power switch.
        self._announce(mfsm.State.IDLE, mfsm.Action.WAIT_OPERATOR,
                       "waiting for the operator to start", waiting="start")
        try:
            input()
        except (EOFError, OSError):
            self.fault = "operator start confirmation unavailable (stdin closed)"
            print("[mission] no terminal attached — refusing to start")
            self.started = False
            return
        self.started = True

    # ── blacklist: objects we already failed on ──
    def _blacklist_here(self) -> None:
        """Record the map pose where we gave up on an object."""
        pose = self.goals.pose
        if pose is None:
            print("[mission] gave up on the target but have no AMCL pose to "
                  "blacklist — it may be picked up again next pass")
            return
        self._blacklist.append((pose.x, pose.y))
        print(f"[mission] blacklisted ({pose.x:.2f}, {pose.y:.2f}); "
              f"{len(self._blacklist)} point(s) suppressed")

    def _blacklisted_here(self) -> Optional[float]:
        """Distance to the nearest give-up point, if we are inside the radius."""
        if self.args.blacklist_radius_m <= 0.0:
            return None
        pose = self.goals.pose
        if pose is None or not self._blacklist:
            return None
        d = min(math.hypot(pose.x - bx, pose.y - by) for bx, by in self._blacklist)
        return d if d <= self.args.blacklist_radius_m else None

    def _handle_pause(self, reason: str) -> None:
        """Hold in PAUSED until a human clears it, then re-verify from scratch.

        Without this the state is a dead end: the FSM has a documented recovery
        path but ``operator_cleared`` was never set by anything, so a robot that
        paused for a stale sensor stopped forever.

        Clearing deliberately resets ``self_check_passed`` and ``started`` too.
        Something was wrong enough to stop the robot; resuming straight into
        PATROL on the strength of a self-check that passed before the fault is
        exactly the wrong reading of "the operator said continue".
        """
        print("\n" + "=" * 60)
        print(f"  PAUSED: {reason}")
        print("  The base is stopped. Fix the cause (RViz pose, sensor, obstacle),")
        print("  then press Enter to re-run the self-check. Ctrl+C to quit.")
        print("=" * 60)
        self._announce(mfsm.State.PAUSED, mfsm.Action.STOP, reason, waiting="pause")
        try:
            input()
        except (EOFError, OSError):
            # No terminal: hold rather than silently resume. A headless run that
            # paused needs a human anyway.
            print("[mission] no terminal attached — holding in PAUSED")
            time.sleep(5.0)
            return
        self.operator_cleared = True
        self.self_check_passed = False
        self.started = not self.args.wait_start
        holding = self._grasp_verified and not self._place_finished
        if holding:
            # Preserve both the jaw hold and the delivery bookkeeping.  If the
            # pose is already known-good, the self-check need not move the arm.
            # If it is not, run_self_check moves it with grip_is_hold=True.
            pass
        else:
            self._reset_target_state()
            self._at_nav_home = False    # re-park during the new check

    def _reset_target_state(self) -> None:
        self._latched = None
        self._grasp_finished = self._grasp_controller_confirmed = False
        self._grasp_verified = False
        self._handoff_ready = self._align_failed = False
        self._place_finished = False
        self._det_streak = 0

    def _run_place(self) -> bool:
        """Drop the object: grasp/v21's scripted release, run in place.

        No bin pose, no aiming, no measured over-the-bin arm pose. The opening
        is ~30 cm across, so once the chassis is at the bin waypoint the object
        can simply fall: reach forward, stop as soon as FK says the pad would
        drop below the rim, open, come home.

        ``run_release_only()`` deliberately does not go to home first (the arm is
        holding something) and refuses if the jaw check says nothing is held --
        which also catches an object dropped somewhere between the grasp and the
        bin, instead of miming a release over an empty jaw.

        There is no drop verification. The jaw-stall proxy reads "holding" right
        up until the jaw is commanded open, so it cannot tell a successful drop
        from a failed one. A "released" outcome means the motion ran.
        """
        try:
            outcome = self.controller.run_release_only()
        except Exception as exc:
            self.fault = f"release sequence raised: {exc}"
            return False

        if outcome == "released":
            # v21's release ends with a guarded move to cfg.home_deg, which is
            # the nav travel pose now that the two homes are split -- so the arm
            # is already folded and RESUME need not move it again.
            self._arm_at_home = self._at_nav_home = True
            print("[mission] release motion complete (the drop itself is not sensed)")
            return True
        if outcome == "rejected":
            # Nothing in the jaw. Not a fault: the object is gone, the arm is
            # safe, and the right move is to carry on patrolling. The arm has
            # not moved, so leave _at_nav_home alone and let RESUME fold it.
            self._arm_at_home = True
            print("[mission] nothing in the jaw to release — resuming patrol")
            return True
        self.fault = f"release aborted: {outcome}"
        return False

    # ── main loop ──
    def run(self) -> int:
        cfg = self.nav.ncfg
        print("[mission] starting. Ctrl+C stops the base and releases the port.")
        try:
            while True:
                now = time.time()
                dt = min(now - self._t_prev, 3 * cfg.control_period_s)
                self._t_prev = now

                sense = self._sense(now)
                tr = self.fsm.step(sense)

                if self.status.enabled:
                    # Cheap enough to do every tick: one dict and one datagram.
                    # Sampling it would make the console lag the robot, which
                    # is the opposite of the point.
                    self.status.publish(tr, sense, self.goals.get(now),
                                        self.goals.pose, laps=self.goals.laps)

                if tr.changed:
                    print(f"[mission] {tr.state.value:<16} {tr.reason}")
                elif now - self._last_log >= LOG_PERIOD_S:
                    self._last_log = now
                    fix = self.goals.get(now)
                    print(f"[mission] {tr.state.value:<16} "
                          f"goal={fix.goal_id}:{fix.dist:.2f}m "
                          f"tgt={sense.target_dist:.2f}m "
                          f"streak={sense.detection_streak} "
                          f"{'STOP' if not tr.chassis_allowed else ''}")

                if tr.terminal:
                    self.nav.stop()
                    print(f"[mission] TERMINAL {tr.state.value}: {tr.reason}")
                    return 2
                if tr.completed:
                    self.nav.stop()
                    print(f"[mission] COMPLETE: {tr.reason}")
                    return 0
                if tr.state is mfsm.State.PAUSED:
                    self.nav.stop()
                    if self.args.exit_on_pause:
                        print(f"[mission] PAUSED: {tr.reason} (--exit-on-pause)")
                        return 3
                    self._handle_pause(tr.reason)
                    continue

                self._act(tr, now, dt)

                if self.args.max_laps and self.goals.laps >= self.args.max_laps:
                    self.nav.stop()
                    print(f"[mission] completed {self.goals.laps} lap(s) — done.")
                    return 0

                spent = time.time() - now
                if spent > self.goals.pose_max_age_s:
                    # This tick blocked -- a grasp episode, a release, a
                    # move_home. Nothing could keep the fix alive meanwhile, so
                    # the map states that come next would refuse to drive on it.
                    # The base is stopped here, so recover the fix now instead
                    # of pausing and sending a human to RViz.
                    self._recover_map_fix(spent)
                if cfg.control_period_s - spent > 0:
                    time.sleep(cfg.control_period_s - spent)
        except KeyboardInterrupt:
            print("\n[mission] interrupted")
            return 130
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        try:
            self.nav.stop()
        except Exception as exc:
            print(f"[mission][WARN] chassis stop failed: {exc}")
        # After the wheels: a console watching this run should see the stop
        # before the stream ends.
        status = getattr(self, "status", None)
        if status is not None:
            try:
                status.close()
            except Exception as exc:
                print(f"[mission][WARN] status cleanup failed: {exc}")
        if self.odom_pub is not None:
            try:
                self.odom_pub.stop()
            except Exception as exc:
                print(f"[mission][WARN] odom stop failed: {exc}")
            try:
                self.odom_pub.join(timeout=2.0)
                if self.odom_pub.is_alive():
                    print("[mission][WARN] odom publisher did not stop within 2.0s")
            except Exception as exc:
                print(f"[mission][WARN] odom cleanup failed: {exc}")
        for name, fn in (("cameras", self.nav.release), ("lidar", self.lidar.close),
                         ("ros", self.rio.close), ("grasp", self.controller.close)):
            try:
                fn()
            except Exception as exc:
                print(f"[mission][WARN] {name} cleanup failed: {exc}")


# ════════════════════════════════════════════════════════════════════════════
# Wiring
# ════════════════════════════════════════════════════════════════════════════

def resolve_grasp_model(verify_hashes: bool = True) -> Tuple[str, str]:
    """Read the grasp model pair out of grasp/v21/manifest.json and check it.

    DeployConfig ships a deliberate placeholder ("SELECTED_MODEL_REQUIRED.zip"),
    so something must name the weights. Taking them from the manifest rather
    than hardcoding two more paths buys the pairing rule the manifest states:
    "These two files are one unit. Never load this model with another
    VecNormalize, or this VecNormalize with another model." A wrong-but-loadable
    VecNormalize does not raise -- it silently feeds the policy mis-normalised
    observations, and the arm just grasps in the wrong place.
    """
    import hashlib
    import json

    v21 = Path(__file__).resolve().parent.parent / "grasp" / "v21"
    manifest_path = v21 / "manifest.json"
    if not manifest_path.exists():
        raise SystemExit(f"[mission] missing grasp manifest: {manifest_path}")
    with open(manifest_path, "r", encoding="utf-8") as fh:
        manifest = json.load(fh)

    artifacts = manifest.get("artifacts") or {}
    out = []
    for key in ("model", "vecnormalize"):
        entry = artifacts.get(key) or {}
        rel = entry.get("file")
        if not rel:
            raise SystemExit(f"[mission] manifest has no artifacts.{key}.file")
        path = v21 / rel
        if not path.exists():
            raise SystemExit(f"[mission] manifest names a missing file: {path}")
        want = entry.get("sha256")
        if verify_hashes and want:
            h = hashlib.sha256()
            with open(path, "rb") as fh:
                for chunk in iter(lambda: fh.read(1 << 20), b""):
                    h.update(chunk)
            if h.hexdigest() != want:
                raise SystemExit(
                    f"[mission] {path.name} does not match the manifest sha256.\n"
                    f"  expected {want}\n  got      {h.hexdigest()}\n"
                    f"  Refusing to run: the model/VecNormalize pairing is the one "
                    f"thing that fails silently rather than loudly.")
        out.append(str(path))

    status = manifest.get("status", "?")
    print(f"[mission] grasp model: {manifest.get('package')} (status={status}), "
          f"sha256 verified" if verify_hashes else "[mission] grasp model resolved")
    if status != "approved":
        print(f"[mission] NOTE: this grasp package is '{status}', not approved — "
              f"see manifest status_note before quoting results.")
    return out[0], out[1]


def build_and_run(args) -> int:
    if args.real:
        if args.skip_model_hash:
            raise SystemExit("[mission] --real never permits --skip-model-hash")
        missing = [f for f, v in (
            ("--i-confirm-serial-owner", args.i_confirm_serial_owner),
            ("--lidar-orientation-evidence", args.lidar_orientation_evidence),
            ("--i-confirm-arm-cam-pose", args.i_confirm_arm_cam_pose),
        ) if not v]
        if missing:
            raise SystemExit(
                "[mission] --real refuses to start without: " + ", ".join(missing) +
                "\n  serial owner:      sudo fuser -v /dev/myserial   (exactly one PID)"
                "\n  lidar orientation: run the Route B four-direction board gate and pass"
                " --lidar-orientation-evidence <verified.json>"
                "\n  arm cam pose:      the C3 extrinsics must be MEASURED, not predicted"
            )
        # Validate the exact v21 model pair and incremental contract before
        # loading navigation, YOLO, cameras or opening the serial port. The
        # standalone launcher has always done this; the integrated entry used
        # to bypass it by constructing GraspController directly.
        g = vgp._load_grasp_module()
        gmodel, gvec = (args.grasp_model, args.grasp_vecnorm)
        if not (gmodel and gvec):
            gmodel, gvec = resolve_grasp_model(verify_hashes=True)
        release_cfg = g.DeployConfig(serial_port=args.port)
        release_cfg.model_path, release_cfg.vecnorm_path = gmodel, gvec
        if not g._release_gate_ok(release_cfg, args.unlock_candidate_real):
            raise SystemExit("[mission] grasp model release gate refused --real")
        # The standalone vision pipelines already enforce this boundary. This
        # integrated entry used their Navigator class directly and therefore
        # bypassed the orchestrator gate, despite still using the old
        # near-vertical trigonometric C3 mapping. Keep the wheels and arm locked
        # out until a measured grasp-home homography is actually wired into the
        # final align/latch path.
        raise SystemExit(
            "[mission] --real integrated grasp is disabled: final alignment/latch "
            "does not yet consume a measured grasp-home homography. Use dry-run "
            "for mission integration or a versioned standalone grasp launcher "
            "for supervised hardware work."
        )

    else:
        g = vgp._load_grasp_module()
        gmodel, gvec = (args.grasp_model, args.grasp_vecnorm)
        if not (gmodel and gvec):
            gmodel, gvec = resolve_grasp_model(verify_hashes=not args.skip_model_hash)

    route = mgp.load_route(args.route, annotations_path=args.annotations,
                           arrival_radius_m=args.arrival_radius,
                           resample_m=(args.resample_m or None))
    print(f"[mission] route: {len(route.waypoints)} waypoints, loop={route.loop}, "
          f"bin approach={route.bin_approach}")
    goals = mgp.MapGoalProvider(route, arrival_radius_m=args.arrival_radius,
                                pose_max_age_s=args.pose_max_age)

    ncfg = nr.NavRLConfig()
    if args.model: ncfg.model_path = args.model
    if args.vecnorm: ncfg.vecnorm_path = args.vecnorm
    if args.lidar_dir < 0: ncfg.lidar_angle_dir = -1.0
    if args.lidar_yaw_offset_deg is not None:
        ncfg.lidar_yaw_offset_deg = args.lidar_yaw_offset_deg
    ncfg.lidar_forward_offset_m = args.lidar_forward_offset_m
    if args.control_period: ncfg.control_period_s = args.control_period
    nr.validate_config(ncfg)
    if args.real:
        verified, why = nr.validate_orientation_evidence(
            args.lidar_orientation_evidence, ncfg)
        if not verified:
            raise SystemExit(f"[mission] --real refuses orientation evidence: {why}")
    policy = nr.NavPolicy(ncfg)

    from ultralytics import YOLO
    model_path = Path(__file__).resolve().parent.parent / "detection" / "models" / "best.pt"
    print(f"[mission] loading YOLO: {model_path}")
    model = YOLO(str(model_path))

    gcfg = g.DeployConfig(serial_port=args.port)
    gcfg.model_path, gcfg.vecnorm_path = gmodel, gvec
    gcfg.release_enable = not args.no_deliver
    gcfg.bin_rim_height = args.bin_rim_height
    gcfg.release_clearance = args.release_clearance
    gcfg.release_extend_steps = args.release_extend_steps
    # home_deg = where the arm rides while driving; grasp_home_deg = the trained
    # pose run() starts the policy from. v21 leaves the second None and puts C3
    # in the first, which parks the gripper outside the chassis for the whole
    # patrol. Split them.
    gcfg.home_deg = (g.parse_deg6_csv(args.nav_home_deg)
                     if args.nav_home_deg else NAV_HOME_DEG)
    gcfg.grasp_home_deg = (g.parse_deg6_csv(args.grasp_home_deg)
                           if args.grasp_home_deg else GRASP_HOME_DEG)
    print(f"[mission] arm nav home   : {list(gcfg.home_deg)}")
    print(f"[mission] arm grasp home : {list(gcfg.grasp_home_deg)}  (C3)")
    class_z_m = vgp.parse_class_z(args.class_z, vgp.OBJ_Z_FIXED)
    class_height_m = vgp.parse_class_height(args.class_height)
    # Check the pose the arm CAMERA is used at, which is now the grasp home, not
    # the nav home. Passing home_deg here would validate the travel pose against
    # C3's extrinsics and pass or fail for the wrong reason.
    vgp.check_arm_cam_pose(gcfg.grasp_home_deg, real=args.real,
                           acknowledged=args.i_confirm_arm_cam_pose)

    lidar = nr.make_lidar(ncfg, args.lidar_backend, ros_host=args.ros_host,
                          ros_port=args.ros_port, scan_topic=args.scan_topic)
    rio = None
    controller = None
    odom_pub = None
    try:
        rio = ros_io.make_ros_io(
            args.ros_backend, host=args.ros_host, port=args.ros_port,
            # Only subscribe when the offboard source is actually selected;
            # an unused subscription is a thread and a queue for nothing.
            trash_topic=(ros_io.TRASH_TOPIC
                         if args.target_source == "offboard" else None))
        controller = g.GraspController(gcfg, real_servo=args.real, use_socket=False)
        nav = MissionNavigator(
            controller.servo.device, model,
            policy=policy, lidar=lidar, ncfg=ncfg,
            wz_sign=args.wz_sign, nav_stop_dist=args.nav_stop_dist,
            show=args.show, dry_run=not args.real,
            rear_url=args.rear_stream or
                f"http://{args.jetson_ip}:8080/stream?topic=/back_cam/image_raw",
            arm_url=args.arm_stream or
                f"http://{args.jetson_ip}:8080/stream?topic=/arm_cam/image_raw",
            class_z_m=class_z_m, class_height_m=class_height_m,
        )
        nav.open_cameras()
        reader = FeedbackOdomReader(controller.servo.device, FeedbackOdomConfig())
        odom_pub = OdomPublisher(reader, rio, rate_hz=args.odom_rate)
        odom_pub.start()

        fcfg = mfsm.MissionConfig(
            detection_streak_needed=args.detection_streak,
            approach_to_align_m=args.nav_stop_dist,
        )
        fsm = mfsm.MissionFSM(fcfg, deliver_enabled=not args.no_deliver)
        runner = MissionRunner(nav=nav, controller=controller, goals=goals, rio=rio,
                               odom_pub=odom_pub, lidar=lidar, fsm=fsm, args=args)
        return runner.run()
    except Exception:
        if odom_pub is not None:
            odom_pub.stop()
        for fn in (getattr(lidar, "close", None), getattr(rio, "close", None),
                   getattr(controller, "close", None)):
            if fn is not None:
                try:
                    fn()
                except Exception:
                    pass
        raise


# ════════════════════════════════════════════════════════════════════════════
# Selftest — the whole mission on fakes: no torch, no cameras, no hardware
# ════════════════════════════════════════════════════════════════════════════

class _FakePolicy:
    """Proportional controller in the policy's action space."""

    def predict(self, obs55):
        dist, sin_b, cos_b = float(obs55[0]), float(obs55[1]), float(obs55[2])
        bearing = math.atan2(sin_b, cos_b)
        return np.array([np.clip(dist, 0.0, 1.0) * (1.0 if abs(bearing) < 0.5 else 0.2),
                         np.clip(2.0 * bearing, -1.0, 1.0)], dtype=np.float32)


class _FakeLidar:
    def __init__(self):
        self.points = [(a - 90.0, 4.0) for a in range(0, 181, 2)]

    def get_points(self):
        return list(self.points)

    def age(self):
        return 0.0

    def close(self):
        pass


def run_selftest() -> None:
    print("== mission pipeline selftest (no hardware, no torch) ==")
    ncfg = nr.NavRLConfig()

    # ── nav_tick drives a toy robot to a map waypoint ──
    class _Nav(MissionNavigator):
        def __init__(self):
            self.policy, self.lidar, self.ncfg = _FakePolicy(), _FakeLidar(), ncfg
            self.wz_sign, self.dry_run, self.device, self.show = 1.0, True, None, False
            self.commands = []
            MissionNavigator.reset_nav(self)

        def _drive_raw(self, vx, wz):
            self.commands.append((vx, wz))

        def stop(self):
            self.commands.append((0.0, 0.0))

    route = mgp.RouteSpec(
        waypoints=[mgp.Waypoint("p0", 0.0, 0.0), mgp.Waypoint("p1", 2.0, 0.0),
                   mgp.Waypoint("p2", 2.0, 2.0)],
        loop=True, bin_center=(3.0, 2.0), bin_approach=(2.6, 2.0, 0.0),
    )
    goals = mgp.MapGoalProvider(route, arrival_radius_m=0.25)
    nav = _Nav()

    # closed loop: integrate the commands we issue and check we reach p1
    x, y, yaw = 0.0, 0.0, 0.0
    goals.reset(start_index=1)
    dt = ncfg.control_period_s
    for step in range(400):
        t = 1000.0 + step * dt
        goals.set_pose(mgp.MapPose(x, y, yaw, t))
        fix = goals.get(t)
        assert fix.valid, fix
        if goals.arrived(fix, t):
            break
        tick = nav.nav_tick(fix.dist, fix.bearing, dt, nav.lidar.get_points())
        x += tick.vx * math.cos(yaw) * dt
        y += tick.vx * math.sin(yaw) * dt
        yaw = mgp.wrap_angle(yaw + tick.wz * dt)
    assert goals.arrived(now=t), f"never arrived: ({x:.2f}, {y:.2f})"
    print(f"  reached {goals.current_waypoint().id} in {step} ticks "
          f"at ({x:.2f}, {y:.2f})")

    # ── the safety brake outranks the policy, on raw points ──
    # Prime the 2-step ActionDelay on a clear path first, otherwise the plant is
    # still executing the zeros it started with and vx would be 0 for reasons
    # that have nothing to do with braking.
    nav2 = _Nav()
    clear = nav2.lidar.get_points()
    for _ in range(ncfg.motor_delay_steps + 1):
        primed = nav2.nav_tick(3.0, 0.0, dt, clear)
    assert primed.vx > 0.0 and not primed.braked, primed
    blocked = [(0.0, 0.15)] + [(a - 90.0, 4.0) for a in range(0, 181, 5)]
    tick = nav2.nav_tick(3.0, 0.0, dt, blocked)
    assert tick.braked and tick.vx == 0.0, tick
    assert tick.min_ray >= ncfg.lidar_min_dist, \
        "the 48-ray obs is floored, which is exactly why the brake reads raw points"
    print(f"  safety brake fired: front={tick.front_raw:.2f} m -> vx=0 "
          f"(obs min ray would have read {tick.min_ray:.2f} m)")

    # ── reset_nav clears everything aimed at the previous goal ──
    nav3 = _Nav()
    for _ in range(5):
        nav3.nav_tick(2.0, 0.3, dt, nav3.lidar.get_points())
    nav3.tracker.update(1.0, 0.0)
    assert nav3.last_vx != 0.0 and nav3.tracker.has_fix
    nav3.reset_nav()
    assert nav3.last_vx == 0.0 and nav3.last_wz == 0.0
    assert not nav3.tracker.has_fix
    assert float(np.max(np.abs(nav3.prev_action))) == 0.0
    assert float(np.max(np.abs(nav3.delay.push(np.zeros(2, np.float32))))) == 0.0
    print("  reset_nav cleared ActionDelay, prev_action and the tracker")

    # ── the handoff gate: trained envelope AND radius from the aim point ──
    class _Cfg:
        documented_x_range = (0.20, 0.28)
        documented_y_range = (-0.09, 0.08)
        envelope_tol_m = 0.001

    class _Ctl:
        cfg = _Cfg()

    class _Args:
        handoff_aim_x = 0.24
        handoff_radius_m = 0.04

    runner = MissionRunner.__new__(MissionRunner)
    runner.controller = _Ctl()
    runner.args = _Args()
    for pos, want, needle in (
        ((0.24, 0.00, 0.02), True, ""),          # dead on the aim point
        ((0.27, 0.02, 0.02), True, ""),          # 3.6 cm away, inside the radius
        ((0.31, 0.00, 0.02), False, "x="),       # camera sees it, never trained
        ((0.19, 0.00, 0.02), False, "x="),
        ((0.24, 0.12, 0.02), False, "y="),
        ((0.20, -0.09, 0.02), False, "from the aim point"),  # in the box, too far
        ((0.28, 0.00, 0.02), False, "from the aim point"),
    ):
        ok, why = MissionRunner.envelope_ok(runner, pos)
        assert ok is want, (pos, ok, why)
        if not want:
            assert needle in why, (pos, why)
    print("  handoff gate: trained box 0.20-0.28 m AND within "
          f"{_Args.handoff_radius_m*100:.0f} cm of the aim point")

    # frame arithmetic must be stated, not assumed: the same object in the three
    # frames people quote (measured from grasp/v21's FK, 2026-08-04)
    desc = MissionRunner.describe_frames((0.24, 0.0, 0.02))
    assert "base_footprint x=0.220" in desc, desc
    assert "front-axle x=0.140" in desc, desc
    print(f"  frame conversion: {desc}")

    # ── DELIVER must actually re-point the goal provider at the bin ──
    # Without this the "drive to the bin" action drives to whatever patrol
    # waypoint happened to be current, and the arrival test passes there.
    dgoals = mgp.MapGoalProvider(
        mgp.RouteSpec(waypoints=[mgp.Waypoint(f"p{i}", i * 0.8, 0.0) for i in range(6)],
                      loop=True, bin_center=(3.0, 1.0), bin_approach=(3.0, 0.6, 0.0)),
        arrival_radius_m=0.25)
    dgoals.reset(start_index=2)

    class _StubNav:
        def stop(self): pass
        def reset_nav(self, clear_tracker=True): pass
        def nav_tick(self, *a, **k): pass

    dr = MissionRunner.__new__(MissionRunner)
    dr.goals, dr.nav = dgoals, _StubNav()
    dr.lidar = type("L", (), {"get_points": lambda s: []})()
    dr.args = type("A", (), {"blacklist_radius_m": 1.0})()
    dr._det_streak = 0
    dr._grasp_verified = True          # delivered successfully
    dr._blacklist = []
    dr._at_nav_home = True             # v21 already returns there after a grasp
    dr.operator_cleared = False

    assert not dgoals.in_override
    MissionRunner._act(dr, mfsm.Transition(mfsm.State.DELIVER, mfsm.Action.DRIVE_BIN),
                       1.0, 0.1)
    assert dgoals.in_override and dgoals.target()[:2] == (3.0, 0.6), dgoals.target()
    assert dgoals.interrupted_index == 2
    dgoals.set_pose(mgp.MapPose(3.0, 0.6, 0.0, 10.0))
    assert dgoals.arrived(now=10.0)
    MissionRunner._act(dr, mfsm.Transition(mfsm.State.RESUME, mfsm.Action.RESUME_PATROL),
                       11.0, 0.1)
    assert not dgoals.in_override and dgoals.index == 3, dgoals.index
    assert dr._det_streak == 0, "coming back from the bin should start a fresh look"
    # ...but a plain patrol advance must not wipe a confirmation in progress
    dr._det_streak = 2
    MissionRunner._act(dr, mfsm.Transition(mfsm.State.PATROL, mfsm.Action.RESUME_PATROL),
                       12.0, 0.1)
    assert dgoals.index == 4 and dr._det_streak == 2, \
        "a waypoint advance stopped the robot or wiped the detection streak"
    assert dr._blacklist == [], "a delivered object must not be blacklisted"
    print("  DELIVER re-points the goal to the bin, RESUME returns to the next waypoint")

    # ── giving up must clear the target state AND blacklist the spot ──
    # Leaving _latched set makes Sense.latched true on the first tick of the
    # NEXT object's LATCH, so the FSM skips the latch entirely and the arm
    # reaches for the previous object's coordinates. Not blacklisting makes the
    # robot re-confirm the same ungraspable thing on every patrol lap, forever.
    gr = MissionRunner.__new__(MissionRunner)
    gr.goals, gr.nav = dgoals, _StubNav()
    gr.lidar = dr.lidar
    gr.args = type("A", (), {"blacklist_radius_m": 1.0, "detection_jump_m": 0.35})()
    gr._blacklist = []
    gr._at_nav_home = True
    gr.operator_cleared = False
    gr._det_streak = 3
    gr._latched = ([0.24, 0.0, 0.02], 0.023, 0.065)
    gr._grasp_finished = True
    gr._grasp_verified = False            # gave up
    gr._align_failed = True
    gr._handoff_ready = True
    gr._place_finished = False
    dgoals.set_pose(mgp.MapPose(2.5, 0.0, 0.0, 50.0))
    MissionRunner._act(gr, mfsm.Transition(mfsm.State.RESUME, mfsm.Action.RESUME_PATROL),
                       50.0, 0.1)
    assert gr._latched is None, "a stale latch would make the NEXT grasp use old coords"
    assert not gr._align_failed and not gr._handoff_ready
    assert not gr._grasp_finished and not gr._grasp_verified
    assert gr._det_streak == 0
    assert gr._blacklist == [(2.5, 0.0)], gr._blacklist
    print("  giving up clears every target flag and blacklists the spot")

    # ...and a detection back at that spot is ignored instead of looping forever
    gr.nav = _Nav()
    gr._last_det = 0.0
    gr.nav._detect_rear = lambda: (True, 1.5, 0.0)
    dgoals.set_pose(mgp.MapPose(2.7, 0.0, 0.0, 60.0))     # 0.2 m from the give-up
    MissionRunner._run_detection(gr, 60.0)
    assert gr._det_streak == 0, "detection inside the blacklist radius must not count"
    dgoals.set_pose(mgp.MapPose(4.6, 0.0, 0.0, 61.0))     # 2.1 m away, outside
    gr._last_det = 0.0
    MissionRunner._run_detection(gr, 61.0)
    assert gr._det_streak == 1, "outside the radius detections must count again"
    print("  blacklist suppresses re-detection nearby, releases further away")

    # ── the bin override remembers where patrol was ──
    goals.reset(start_index=1)
    goals.set_bin_target()
    assert goals.in_override and goals.interrupted_index == 1
    goals.set_pose(mgp.MapPose(2.6, 2.0, 0.0, 1.0))
    assert goals.arrived(now=1.0), "sitting on the bin approach point"
    assert goals.resume_patrol() == 2, goals.index
    assert not goals.in_override
    print("  bin override -> resume at the NEXT waypoint OK")

    # ── FSM + goal-source switching agree ──
    fsm = mfsm.MissionFSM(mfsm.MissionConfig(approach_to_align_m=0.75))
    t = 0.0
    fsm.step(mfsm.Sense(now=t))
    fsm.step(mfsm.Sense(now=t + 0.1, self_check_passed=True))
    fsm.step(mfsm.Sense(now=t + 0.2, started=True))
    assert fsm.state is mfsm.State.PATROL
    tr = fsm.step(mfsm.Sense(now=t + 0.3, detection_streak=3))
    assert tr.reset_nav and tr.chassis_allowed
    print("  FSM patrol -> investigate switches goal source with reset_nav")

    # ── PATROL -> INVESTIGATE must NOT throw away the fix that triggered it ──
    # This is the untested leg of the mission. Clearing the tracker here leaves
    # the next tick with fix_age = infinity, the FSM reads "target lost", and the
    # robot bounces straight back to patrol -- it could never reach an object it
    # had just seen. Regression test, because nothing on the robot would show
    # this as anything but "it ignored the trash".
    nav4 = _Nav()
    nav4.tracker.update(1.8, 0.10)
    assert nav4.tracker.has_fix
    nav4.reset_nav(clear_tracker=(mfsm.State.INVESTIGATE in mfsm.MAP_STATES))
    assert nav4.tracker.has_fix, \
        "PATROL->INVESTIGATE cleared the camera fix; the target is instantly 'lost'"
    assert nav4.tracker.fix_age() < 1.0
    assert float(np.max(np.abs(nav4.prev_action))) == 0.0, \
        "the ActionDelay must still be cleared — it is aimed at the map waypoint"
    # ...and entering a map-goal state DOES clear it
    nav4.reset_nav(clear_tracker=(mfsm.State.DELIVER in mfsm.MAP_STATES))
    assert not nav4.tracker.has_fix, "DELIVER is a map goal; the camera fix must go"
    print("  tracker survives PATROL->INVESTIGATE, cleared on ->DELIVER")

    # the FSM would abandon immediately on a cleared tracker — prove the failure
    # mode this guards against is real, not hypothetical
    fsm2 = mfsm.MissionFSM(mfsm.MissionConfig())
    fsm2.state, fsm2.entered_at = mfsm.State.INVESTIGATE, 100.0
    tr = fsm2.step(mfsm.Sense(now=100.1, target_visible=False,
                              target_fix_age=float("inf")))
    assert fsm2.state is mfsm.State.RESUME and "lost" in tr.reason, tr
    print("  (confirmed: a cleared tracker makes INVESTIGATE abandon on tick 1)")

    # ── detection streak needs spatial agreement, not just three hits ──
    class _StreakRunner(MissionRunner):
        def __init__(self, nav, jump_m):
            self.nav = nav
            self._det_streak = 0
            self._last_det = 0.0
            self.args = type("A", (), {"detection_jump_m": jump_m,
                                       "blacklist_radius_m": 1.0})()
            self._blacklist = []
            self.goals = mgp.MapGoalProvider(
                mgp.RouteSpec(waypoints=[mgp.Waypoint("a", 0.0, 0.0),
                                         mgp.Waypoint("b", 1.0, 0.0)], loop=False))

        def _detect(self):
            return self._detections.pop(0)

    nav5 = _Nav()
    r5 = _StreakRunner(nav5, 0.35)
    # three consistent sightings of a stationary object (robot is not moving in
    # this harness, so the tracker prediction stays put)
    r5.nav._detect_rear = lambda: (True, 2.00, 0.05)
    for i in range(3):
        r5._last_det = 0.0
        MissionRunner._run_detection(r5, 100.0 + i)
    assert r5._det_streak == 3, r5._det_streak
    # now a flicker somewhere else entirely
    r5.nav._detect_rear = lambda: (True, 2.00, 1.20)
    r5._last_det = 0.0
    MissionRunner._run_detection(r5, 200.0)
    assert r5._det_streak == 1, \
        f"a fix 1.15 m off the prediction should restart the streak, got {r5._det_streak}"
    # a miss resets it outright
    r5.nav._detect_rear = lambda: (False, -1.0, 0.0)
    r5._last_det = 0.0
    MissionRunner._run_detection(r5, 300.0)
    assert r5._det_streak == 0
    print("  detection streak: 3 consistent hits confirm, a jump restarts, a miss resets")

    # ── PAUSED must be recoverable, and must re-verify on the way out ──
    # The FSM has always had the recovery edge; nothing in the pipeline set
    # operator_cleared, so a robot that paused for a stale sensor stopped
    # forever. And clearing must not walk straight back into PATROL on a
    # self-check that passed before the fault.
    pr = MissionRunner.__new__(MissionRunner)
    pr.args = type("A", (), {"wait_start": True, "exit_on_pause": False})()
    pr.operator_cleared = False
    pr.self_check_passed = True
    pr.started = True
    pr._at_nav_home = True
    pr._grasp_verified = False
    pr._place_finished = False
    pr._latched = None
    pr._grasp_finished = False
    pr._grasp_controller_confirmed = False
    pr._handoff_ready = False
    pr._align_failed = False
    pr._det_streak = 0
    import builtins as _b
    import contextlib as _ctx
    import io as _io

    def _with_input(fn):
        """Drive _handle_pause with a scripted stdin, never the ambient one."""
        real_in, _b.input = _b.input, fn
        real_sleep, time.sleep = time.sleep, lambda *_a: None
        try:
            with _ctx.redirect_stdout(_io.StringIO()):
                MissionRunner._handle_pause(pr, "test: /scan stale")
        finally:
            _b.input, time.sleep = real_in, real_sleep

    def _no_tty(*_a):
        raise EOFError

    _with_input(_no_tty)     # no terminal -> hold, do NOT silently resume
    assert pr.operator_cleared is False and pr.self_check_passed is True, \
        "with no terminal the pause must hold, not clear itself"

    _with_input(lambda *_a: "")   # a terminal + Enter -> clear and demote
    assert pr.operator_cleared is True
    assert pr.self_check_passed is False, "clearing must force a fresh self-check"
    assert pr.started is False, "clearing must require the start confirmation again"
    assert pr._at_nav_home is False, "the arm must be re-parked after a pause"

    # and the FSM accepts it, then the flag must not stay latched
    pfsm = mfsm.MissionFSM(mfsm.MissionConfig())
    pfsm.state, pfsm.entered_at = mfsm.State.PAUSED, 0.0
    assert pfsm.step(mfsm.Sense(now=1.0)).state is mfsm.State.PAUSED
    tr = pfsm.step(mfsm.Sense(now=2.0, operator_cleared=True))
    assert tr.state is mfsm.State.SELF_CHECK, tr
    print("  PAUSED is recoverable and forces a fresh self-check + start confirm")

    # ── the two arm homes must stay distinct ──
    # v21 ships them collapsed (home_deg IS C3, grasp_home_deg is None), which
    # is right for a standalone grasp and wrong for a robot that patrols first.
    # If a future config edit collapses them again the symptom is subtle -- the
    # robot simply drives 58 m with the gripper outside its own footprint --
    # so pin it here.
    assert NAV_HOME_DEG != GRASP_HOME_DEG, \
        "the travel pose and the trained grasp pose must not be the same"
    assert len(NAV_HOME_DEG) == 6 and len(GRASP_HOME_DEG) == 6
    assert GRASP_HOME_DEG == (90.0, 67.08, 9.79, 9.79, 90.0, 30.0), \
        "grasp home must stay C3: the extrinsics, the visible ground band and " \
        "the trained envelope are all defined at that pose"
    # The nav pose has to fold the arm UP relative to C3. S2 is the shoulder and
    # larger API degrees raise it, so this is the one ordering that matters.
    assert NAV_HOME_DEG[1] > GRASP_HOME_DEG[1], \
        "the travel pose must raise the shoulder above the grasp pose"
    print(f"  arm homes distinct: nav S2={NAV_HOME_DEG[1]:.0f} deg vs "
          f"grasp(C3) S2={GRASP_HOME_DEG[1]:.2f} deg")

    # ── every Action the FSM can emit must have a handler ──
    # A missing branch is silent: _act just returns, the robot does nothing, and
    # the FSM sits in that state until its timeout. Cheap to check, so check it.
    import inspect
    src = inspect.getsource(MissionRunner._act)
    unhandled = [a.name for a in mfsm.Action if f"A.{a.name}" not in src]
    assert not unhandled, f"FSM actions with no handler in _act: {unhandled}"
    print(f"  all {len(list(mfsm.Action))} FSM actions have a handler")

    print("[mission] SELFTEST PASSED")


# ════════════════════════════════════════════════════════════════════════════
# CLI
# ════════════════════════════════════════════════════════════════════════════

def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--selftest", action="store_true", help="offline logic tests")
    p.add_argument("--target-source", choices=("onboard", "offboard"),
                   default="onboard",
                   help="where patrol detections come from. 'onboard' (default) "
                        "runs YOLO on the robot's rear camera. 'offboard' "
                        f"subscribes to {ros_io.TRASH_TOPIC}, published by "
                        "detection/rear_cam_sam2_publisher.py on a development "
                        "machine (YOLO + SAM2, mask bottom point). The offboard "
                        "source needs --ros-backend ros.")
    p.add_argument("--trash-max-age", type=float, default=tt.DEFAULT_MAX_AGE_S,
                   metavar="SECONDS",
                   help="offboard detections older than this are treated as no "
                        "detection (default %(default)s). Aged by local arrival "
                        "time, not the publisher's clock.")
    p.add_argument("--status-udp", default=None, metavar="HOST:PORT",
                   help="publish JSON status datagrams (e.g. "
                        f"{mission_status.DEFAULT_ENDPOINT}). Fire-and-forget: "
                        "the mission is unaffected if nothing is listening.")
    p.add_argument("--status-period", type=float,
                   default=mission_status.DEFAULT_PERIOD_S, metavar="SECONDS",
                   help="heartbeat interval between state changes (default "
                        "%(default)s). State changes are always sent immediately; "
                        "raise this to spend less on telemetry.")
    p.add_argument("--real", action="store_true", help="drive real hardware")
    p.add_argument("--dry-run", action="store_true",
                   help="run the full loop with hardware output suppressed")
    p.add_argument("--show", action="store_true")

    p.add_argument("--route", default=str(mgp.DEFAULT_ROUTE_HINT))
    p.add_argument("--annotations", default=None)
    p.add_argument("--arrival-radius", type=float, default=mgp.DEFAULT_ARRIVAL_RADIUS_M)
    # Route C's route.yaml declares 0.75 m spacing but really has 0.049 m in
    # places (orthogonal A* on the 0.049 m/cell grid). 0 disables re-sampling,
    # which for that file means the route is rejected at load.
    p.add_argument("--resample-m", type=float, default=0.75,
                   help="re-space the patrol polyline at this arc length "
                        "(0 = use route.yaml as-is)")
    p.add_argument("--pose-max-age", type=float, default=mgp.DEFAULT_POSE_MAX_AGE_S)
    p.add_argument("--amcl-max-age", type=float,
                   default=ros_io.DEFAULT_MAX_AMCL_AGE_S,
                   help="maximum local receipt age for AMCL health (seconds)")
    p.add_argument("--amcl-refresh-timeout", type=float, default=3.0,
                   help="how long to wait for a fresh fix after the arm blocks "
                        "the loop (seconds)")
    p.add_argument("--max-laps", type=int, default=0, help="0 = patrol forever")
    p.add_argument("--no-deliver", action="store_true",
                   help="stop after a verified grasp instead of going to the bin")

    p.add_argument("--model", default=None)
    p.add_argument("--vecnorm", default=None)
    p.add_argument("--nav-stop-dist", type=float, default=nrgp.NAV_STOP_DIST_M)
    p.add_argument("--detection-streak", type=int, default=3)
    p.add_argument("--control-period", type=float, default=None)
    p.add_argument("--wz-sign", type=float, default=1.0, choices=(-1.0, 1.0))

    p.add_argument("--lidar-backend", default="ros", choices=("ros", "rplidar", "none"))
    p.add_argument("--lidar-dir", type=float, default=1.0, choices=(-1.0, 1.0))
    p.add_argument("--lidar-yaw-offset-deg", type=float, default=0.0)
    p.add_argument("--lidar-forward-offset-m", type=float, default=0.0,
                   help="LiDAR origin ahead(+)/behind(-) of base footprint")
    p.add_argument("--scan-topic", default="/scan")
    p.add_argument("--ros-backend", default="ros", choices=("ros", "none"))
    p.add_argument("--ros-host", default="127.0.0.1")
    p.add_argument("--ros-port", type=int, default=9090)
    p.add_argument("--odom-rate", type=float, default=ODOM_RATE_HZ)

    p.add_argument("--port", default="/dev/myserial")
    p.add_argument("--jetson-ip", default=os.getenv("X3PLUS_JETSON_HOST", "127.0.0.1"))
    p.add_argument("--rear-stream", default=None)
    p.add_argument("--arm-stream", default=None)
    p.add_argument("--class-z", action="append", default=[])
    p.add_argument("--class-height", action="append", default=[])
    p.add_argument("--nav-home-deg", default=None,
                   help=f"arm pose while driving, six API degrees "
                        f"(default {','.join(str(v) for v in NAV_HOME_DEG)})")
    p.add_argument("--grasp-home-deg", default=None,
                   help=f"trained pose the grasp policy starts from "
                        f"(default C3 {','.join(str(v) for v in GRASP_HOME_DEG)})")
    p.add_argument("--max-steps", type=int, default=300)
    p.add_argument("--grasp-model", default=None,
                   help="override the grasp .zip (default: from grasp/v21/manifest.json)")
    p.add_argument("--grasp-vecnorm", default=None,
                   help="override the grasp VecNormalize .pkl — must be the "
                        "matching half of --grasp-model")
    p.add_argument("--skip-model-hash", action="store_true",
                   help="skip the manifest sha256 check (slow disks only)")
    p.add_argument("--unlock-candidate-real", action="store_true",
                   help="allow the v21 candidate release status for a supervised "
                        "real experiment; integrity and incremental contract checks "
                        "remain mandatory")

    # Release: grasp/v21's _scripted_release. No bin pose and no aiming -- it
    # reaches forward from wherever the arm is, stops as soon as FK says the pad
    # would drop below the rim, opens, and comes home.
    # 0 by default: the release just runs. Set it only if you want the reach to
    # stop short for a tall bin; with 0 the only remaining check is that the jaw
    # is not opened at floor level, which is a floor sanity margin, not a bin one.
    p.add_argument("--bin-rim-height", type=float, default=0.0,
                   help="bin rim height above the floor (m). 0 = ignore the bin")
    p.add_argument("--release-clearance", type=float, default=0.03,
                   help="pad bottom must stay this far above rim height (m)")
    p.add_argument("--release-extend-steps", type=int, default=6)

    # Handoff gate: how close the object must be to the aim point before the
    # arm takes over. See MISSION.md for the three frames these numbers live in.
    p.add_argument("--handoff-aim-x", type=float, default=0.24,
                   help="policy-frame x we drive the object to (default 0.24 = "
                        "centre of v21's documented 0.20-0.28 band)")
    p.add_argument("--handoff-radius-m", type=float, default=0.04,
                   help="object must be within this of the aim point to hand over")
    p.add_argument("--detection-jump-m", type=float, default=0.35,
                   help="a patrol detection further than this from the tracker's "
                        "prediction restarts the confirmation streak")

    p.add_argument("--blacklist-radius-m", type=float, default=1.0,
                   help="after giving up on an object, ignore detections within "
                        "this radius of where we gave up (0 disables)")
    p.add_argument("--wait-start", action="store_true", default=None,
                   help="wait for Enter before patrolling (default: on for --real)")
    p.add_argument("--start-immediately", dest="wait_start", action="store_false",
                   help="skip the confirmation prompt")
    p.add_argument("--exit-on-pause", action="store_true")

    p.add_argument("--i-confirm-serial-owner", action="store_true")
    p.add_argument("--lidar-orientation-evidence", default=None,
                   help="verified Route B four-direction evidence/marker; required by --real")
    p.add_argument("--i-confirm-lidar-orientation", action="store_true",
                   help="deprecated compatibility flag; no longer proves orientation")
    p.add_argument("--i-confirm-arm-cam-pose", action="store_true")

    args = p.parse_args(argv)
    if args.real and args.dry_run:
        p.error("--real and --dry-run are mutually exclusive")
    if bool(args.grasp_model) != bool(args.grasp_vecnorm):
        p.error("--grasp-model and --grasp-vecnorm must be given together: they "
                "are one unit and mixing halves fails silently")
    if not math.isfinite(args.nav_stop_dist) or args.nav_stop_dist <= 0.0:
        p.error("--nav-stop-dist must be finite and > 0")
    if not math.isfinite(args.amcl_max_age) or args.amcl_max_age <= 0.0:
        p.error("--amcl-max-age must be finite and > 0")
    if not math.isfinite(args.amcl_refresh_timeout) or args.amcl_refresh_timeout <= 0.0:
        p.error("--amcl-refresh-timeout must be finite and > 0")
    if not math.isfinite(args.lidar_forward_offset_m):
        p.error("--lidar-forward-offset-m must be finite")
    if args.detection_streak < 1:
        p.error("--detection-streak must be >= 1")
    if args.max_laps < 0:
        p.error("--max-laps must be >= 0")
    for name in ("release_clearance", "handoff_radius_m",
                 "handoff_aim_x", "detection_jump_m"):
        v = getattr(args, name)
        if not math.isfinite(v) or v <= 0.0:
            p.error(f"--{name.replace('_', '-')} must be finite and > 0")
    if not math.isfinite(args.bin_rim_height) or args.bin_rim_height < 0.0:
        p.error("--bin-rim-height must be finite and >= 0")
    if not math.isfinite(args.blacklist_radius_m) or args.blacklist_radius_m < 0.0:
        p.error("--blacklist-radius-m must be finite and >= 0")
    if args.wait_start is None:
        # Confirm before driving on hardware; dry-runs just go.
        args.wait_start = bool(args.real)
    if args.release_extend_steps < 1:
        p.error("--release-extend-steps must be >= 1")
    if args.status_udp:
        # Fail on a typo now rather than raising out of the runner's
        # constructor after the hardware has already been opened.
        try:
            mission_status.parse_endpoint(args.status_udp)
        except ValueError as exc:
            p.error(f"--status-udp: {exc}")
    if not math.isfinite(args.status_period) or args.status_period < 0.0:
        p.error("--status-period must be finite and >= 0")
    if args.target_source == "offboard":
        if args.ros_backend != "ros":
            # NullRosIO's latest_trash_target always returns None, so this
            # combination patrols forever and never sees anything. Fail here
            # rather than let it look like the publisher is at fault.
            p.error("--target-source offboard needs --ros-backend ros; "
                    f"the target arrives over rosbridge on {ros_io.TRASH_TOPIC}")
        if not math.isfinite(args.trash_max_age) or args.trash_max_age <= 0.0:
            p.error("--trash-max-age must be finite and > 0")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.selftest:
        run_selftest()
        return 0
    if not (args.real or args.dry_run):
        print("[mission] neither --real nor --dry-run given; assuming --dry-run.")
        args.dry_run = True

    return build_and_run(args)


if __name__ == "__main__":
    raise SystemExit(main())
