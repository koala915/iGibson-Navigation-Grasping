#!/usr/bin/env python3
"""Standalone Jetson Nano deployment for X3Plus 6D grasp policy.

No ROS required. Uses PyBullet DIRECT (headless) for forward kinematics only.

Features
--------
- Loads trained weights from the same folder automatically
- TCP socket (port 5555) for object detection input — or defaults to 0.25 m front
- Yahboom Rosmaster_Lib servo control via UART
- 28D observation matching student training env exactly
- Automatic 3-stage grasp management

Usage
-----
    python x3plus_real_grasp.py                     # dry-run, default obj at 0.25 m
    python x3plus_real_grasp.py --real              # real servo mode
    python x3plus_real_grasp.py --real --socket     # real servo + listen for detection

Sim-to-real calibration note (FINAL — verified on hardware 2026-05-25)
-----------------------------
  All arm joints S1-S5: hw(API) = 90 + sim_deg  (arm_hw_invert = all False).
  Rosmaster_Lib mirrors S2/S3/S4 internally (API = 180 - physical; the Yahboom
  App shows the PHYSICAL angle). Do NOT re-apply per-joint inversion on top of
  the API — that double flip makes the policy move backwards (see progress.md
  2026-05-25). S6 gripper: hw=30° = open (fingers spread), hw=180° = closed.

  All mappings are defined in JointMapper and can be adjusted without touching
  the control logic.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Optional, Tuple

import numpy as np

if "KMP_DUPLICATE_LIB_OK" not in os.environ:
    os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
os.environ.setdefault("OMP_NUM_THREADS", "1")

import pybullet as p
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
import gymnasium as gym

# ── Yahboom Rosmaster_Lib (cp -r /usr/local/lib/python3.6/dist-packages/Rosmaster_Lib . on Jetson) ──
try:
    from Rosmaster_Lib import Rosmaster as _RosmasterCls
    _ROSMASTER_AVAILABLE = True
except ImportError:
    _RosmasterCls = None
    _ROSMASTER_AVAILABLE = False
    print("[WARNING] Rosmaster_Lib not found. Servo output will be printed only.")


# ═══════════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class DeployConfig:
    # ── model paths (relative to this script or absolute) ──
    # v17 = wider-spawn training (x 0.17-0.33 / y ±0.15) on the scripted-Stage-2
    # env. Deterministic wide-range eval 30 eps ≈97-100%; full-workspace
    # map_range 2000 eps = 89.2%. Stage-2 arm output is UNTRAINED in this model —
    # the real robot must script the return (already done here: Stage 2 drives
    # home_deg[:5] + closed gripper). Model and vecnorm must stay paired (v17+v17).
    model_path: str = "trained_6d_models_v17/ppo_6d_final_ready_for_real_robot.zip"
    vecnorm_path: str = "trained_6d_models_v17/vecnormalize_6d_final.pkl"
    urdf_path: str = "x3plus/yahboomcar.urdf"

    # ── object detection ──
    # No-detection fallback: straight ahead (+X), same direction as simulation setup.
    default_obj_pos: Tuple[float, float, float] = (0.25, 0.0, 0.02)
    socket_host: str = "0.0.0.0"
    socket_port: int = 5555
    serial_port: str = "/dev/myserial"  # stable symlink to the ch341 Rosmaster board
    detection_stale_timeout_sec: float = 1.0

    # ── object latching (opt-in via --latch-obj) ──
    # The arm camera (URDF mono_link) is mounted on arm_link4, so it MOVES with
    # the arm. With latch enabled, first settle at the PPO grasp-home pose, then
    # discard any detection received during arm motion and capture ONE new
    # position + width. The frozen grasp-home observation is reused for the
    # whole episode, so later (now-invalid) detections cannot mislead the policy.
    latch_obj: bool = False
    latch_wait_sec: float = 5.0   # how long to wait for a fresh detection before fallback

    # ── control timing ──
    control_hz: float = 10.0        # inference rate
    servo_run_time_ms: int = 100    # time for servo to reach target (ms)
    # Stage-0 approach smoothness: use a run_time LONGER than the control
    # period (100ms at 10Hz) so each new streamed target re-plans the servo
    # trajectory MID-MOTION (remaining distance over 250ms) instead of
    # finish-and-stop every 100ms. At the extended reach pose 3°/joint ≈
    # 3.5cm of TCP, so the stop-start bursts read as a forward lurch; with
    # blending the same path executes velocity-continuously. The physical
    # position trails the commanded stream slightly — the Stage-1 settle +
    # wait guarantees arrival before the gripper closes.
    stage0_run_time_ms: int = 250

    # ── stage thresholds ──
    # Stage 0→1 fires on the SCALAR TCP-to-object distance, so a pose still on
    # its way down can satisfy it: at the C3 grasp home the approach comes in
    # diagonally and crosses 0.05 while the gripper is still ~1.4cm too high.
    # Measured on hardware 2026-07-26 (object x=0.28 y=0 z=0.035): locking at
    # 0.05 froze the arm with the TCP 5.4cm above the object instead of the
    # converged 4.0cm, and the grasp missed. 0.042 forces the descent to finish
    # first — swept over the measured deployable region (x 0.21-0.27 by
    # y -0.08..+0.07) it still triggers everywhere, with tighter lock distances
    # (0.038 vs 0.044). Override per-run with --stage0-dist-threshold.
    # NOTE: this value is grasp-home-pose specific; re-measure if the pose changes.
    stage0_dist_threshold: float = 0.042
    grip_close_threshold: float = 0.90    # model grip output > this → close
    min_dist_rebound_m: float = 0.05      # trigger Stage 1 if dist rebounds this far from minimum
    stage2_dist_threshold: float = 0.05   # dist to home for "done"

    # ── safety ──
    max_delta_deg: float = 3.0            # max per-step servo change (deg)
    # Navigation/cruise home: keep this pose stable while cruising and aligning.
    home_deg: Tuple[float,...] = (90.0, 140.0, 0.0, 0.0, 90.0, 30.0)
    nav_home_wait_sec: float = 2.0

    # PPO training home: use this only as the initial pose for the grasp policy.
    # = v17 training env home_arm_pose (0.0, -1.0, -1.4, -1.0, 0.0) rad converted
    # via hw(API) = 90 + sim_deg. The API mirrors S2/S3/S4, so the Yahboom App
    # will show the PHYSICAL pose ≈ [90, 147.3, 170.2, 147.3, 90] (top-down:
    # camera looks at the ground ahead, gripper hovers over the spawn zone).
    # S6=30 = open. Dry-run + App readout check before the first --real run.
    grasp_home_deg: Tuple[float,...] = (90.0, 32.704, 9.786, 32.704, 90.0, 30.0)
    preposition_wait_sec: float = 4.0    # wait for servos to settle after move_to_grasp_home()
    # Physical reach measured from the TCP at grasp_home_deg. Targets outside
    # this 3-D radius are rejected before any policy-driven arm motion.
    grasp_home_max_target_distance_m: float = 0.15

    # ── Stage 0→1 settle before closing ──
    # At the extended reach pose the Jacobian is large, so the last rate-limited
    # Stage 0 step (3°/joint) moves the TCP several cm. At 100ms/step the servos
    # lag behind and only finish arriving once Stage 1 holds a fixed pose — seen
    # as a forward lurch right as the gripper starts closing (observed 2026-07-05,
    # v17). Ease into the lock pose over a longer run_time and wait, so the arm is
    # physically at rest before S6 closes.
    stage1_settle_run_time_ms: int = 600  # smooth move into the locked arm pose
    # Must exceed stage1_settle_run_time_ms so the first close command is not
    # issued while the settle move is still executing.
    stage1_settle_wait_sec: float = 0.7   # pause for the arm to arrive before closing

    # ── smooth approach (opt-in via --smooth-approach) ──
    # The control loop is open-loop w.r.t. the hardware: observations are
    # built from the rate-limited COMMANDED state, the policy is deterministic
    # and the object position is fixed (or latched) for the whole episode.
    # The entire Stage-0 command sequence is therefore known in advance:
    # roll the policy out virtually to find the lock pose, then travel there
    # in ONE smoothly planned servo move instead of 10 Hz streamed bursts.
    # Identical grasp geometry, far smoother execution. Requires a static
    # object source (--obj-x/y/z, --latch-obj, or a latching obj_provider).
    smooth_approach: bool = False
    smooth_approach_deg_per_sec: float = 15.0  # upper speed for segmented glide
    smooth_approach_min_ms: int = 800
    smooth_approach_max_ms: int = 2000  # Rosmaster_Lib/board limit per segment
    # Reject real glides whose virtual lock pose sits effectively on a joint
    # stop.  A 1° margin blocks the observed ground-cap bad-Z plan (S2=0°)
    # while retaining the trained obj-z=0.02 plan (S2≈2.7°).
    smooth_approach_limit_margin_deg: float = 1.0

    # ── gripper sim range (must match training env) ──
    # Physical: hw=30° = fingers spread (OPEN), hw=180° = fingers together (CLOSED).
    # Confirmed 2026-05-22 from dry-run: Stage 1 force-close should drive S6 30→180.
    gripper_sim_open: float = -1.5   # rad
    gripper_sim_closed: float = 0.0  # rad
    gripper_hw_open: float = 30.0    # degrees (fingers spread)
    gripper_hw_closed: float = 180.0 # degrees (fingers together)

    # ── width-based gripper control (opt-in via --width-grip) ──
    # When enabled and a detected object width (metres) is available, Stage 1
    # closes the gripper to an angle derived from the width instead of always
    # slamming to gripper_hw_closed. Wider object → smaller close angle (the
    # fingers meet the object sooner, avoiding over-squeeze); a near-zero width
    # → full close. If disabled, or no width is known, behaviour is unchanged
    # (full close to gripper_hw_closed).
    grip_width_control: bool = False        # set True by --width-grip CLI flag
    grip_max_object_width_m: float = 0.06   # widest graspable object → grip_min_close_deg
    grip_min_close_deg: float = 120.0       # close angle used for the widest object

    # ── arm sim range (URDF limits in radians) ──
    arm_sim_limits: Tuple[Tuple[float, float], ...] = (
        (-1.5708, 1.5708),   # S1
        (-1.5708, 1.5708),   # S2
        (-1.5708, 1.5708),   # S3
        (-1.5708, 1.5708),   # S4
        (-1.5708, 3.14159),  # S5
    )
    # Invert flag: True = hw = center - sim_deg (inverted).
    # S2/S3/S4 were True when API==physical. After discovering API mirror
    # (API = 180−physical for S2/S3/S4), the mirror itself acts as an inversion,
    # so all joints are now False (API value = center + sim_deg).
    arm_hw_invert: Tuple[bool, ...] = (False, False, False, False, False)
    arm_hw_center: Tuple[float, ...] = (90.0, 90.0, 90.0, 90.0, 90.0)
    arm_hw_range: Tuple[Tuple[float, float], ...] = (
        (0.0, 180.0),   # S1
        (0.0, 180.0),   # S2
        (0.0, 180.0),   # S3
        (0.0, 180.0),   # S4
        (0.0, 270.0),   # S5
    )


def arm_limit_margin_violations(
    arm_deg: List[float],
    limits: Tuple[Tuple[float, float], ...],
    margin_deg: float,
) -> List[str]:
    """Return S-joint labels at/inside the requested hardware-limit margin."""
    if (not math.isfinite(float(margin_deg))) or margin_deg < 0.0:
        raise ValueError("joint-limit margin must be finite and >= 0")
    if len(arm_deg) != len(limits):
        raise ValueError("arm degrees and hardware limits must have equal length")
    violations = []
    for index, (deg, (lo, hi)) in enumerate(zip(arm_deg, limits), start=1):
        if not all(math.isfinite(float(v)) for v in (deg, lo, hi)) or lo > hi:
            raise ValueError("arm degrees and hardware limits must be finite and ordered")
        if deg <= lo + margin_deg or deg >= hi - margin_deg:
            violations.append(f"S{index}={deg:.1f}° (limit {lo:.1f}..{hi:.1f}°)")
    return violations


def validate_grasp_home_reach(
    target_pos: np.ndarray,
    grasp_home_tcp: np.ndarray,
    max_distance_m: float,
) -> float:
    """Return target distance from grasp-home TCP or reject an unreachable target."""
    target = np.asarray(target_pos, dtype=np.float64).reshape(-1)
    home = np.asarray(grasp_home_tcp, dtype=np.float64).reshape(-1)
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError(f"target position must be 3 finite values, got {target}")
    if home.shape != (3,) or not np.all(np.isfinite(home)):
        raise ValueError(f"grasp-home TCP must be 3 finite values, got {home}")
    if not math.isfinite(float(max_distance_m)) or float(max_distance_m) <= 0.0:
        raise ValueError("grasp-home maximum target distance must be finite and > 0")

    distance_m = float(np.linalg.norm(target - home))
    if distance_m > float(max_distance_m) + 1e-9:
        raise RuntimeError(
            f"target is {distance_m:.3f}m from grasp-home TCP, beyond the "
            f"{max_distance_m:.3f}m reachable radius; move the chassis/object "
            "closer and detect again at grasp home"
        )
    return distance_m


# ═══════════════════════════════════════════════════════════════════════════
# Joint Mapper — sim radians ↔ hardware degrees
# ═══════════════════════════════════════════════════════════════════════════

class JointMapper:
    def __init__(self, cfg: DeployConfig):
        self.cfg = cfg

    def sim_arm_to_hw_deg(self, sim_angles_rad: np.ndarray) -> List[float]:
        """Convert 5 arm joint angles (radians, sim) → hardware degrees."""
        sim_angles_rad = np.asarray(sim_angles_rad, dtype=np.float32)
        if sim_angles_rad.shape != (5,) or not np.all(np.isfinite(sim_angles_rad)):
            raise ValueError(f"arm angles must be finite shape (5,), got {sim_angles_rad}")
        hw = []
        for i, (rad, invert, center, (hw_lo, hw_hi), (sim_lo, sim_hi)) in enumerate(zip(
            sim_angles_rad,
            self.cfg.arm_hw_invert,
            self.cfg.arm_hw_center,
            self.cfg.arm_hw_range,
            self.cfg.arm_sim_limits,
        )):
            sim_deg = math.degrees(float(rad))
            if invert:
                deg = center - sim_deg
            else:
                deg = center + sim_deg
            hw.append(float(np.clip(deg, hw_lo, hw_hi)))
        return hw

    def sim_grip_to_hw_deg(self, grip_rad: float) -> float:
        """Convert gripper joint angle (radians, sim) → hardware degree."""
        cfg = self.cfg
        if not math.isfinite(float(grip_rad)):
            raise ValueError(f"gripper angle must be finite, got {grip_rad}")
        grip_rad = float(np.clip(grip_rad, cfg.gripper_sim_open, cfg.gripper_sim_closed))
        ratio = (grip_rad - cfg.gripper_sim_open) / (cfg.gripper_sim_closed - cfg.gripper_sim_open)
        hw = cfg.gripper_hw_open + ratio * (cfg.gripper_hw_closed - cfg.gripper_hw_open)
        return float(np.clip(hw, min(cfg.gripper_hw_open, cfg.gripper_hw_closed),
                                  max(cfg.gripper_hw_open, cfg.gripper_hw_closed)))

    def width_to_close_deg(self, width_m: Optional[float]) -> float:
        """Pick a Stage-1 gripper close angle (hw degrees) from object width.

        Returns gripper_hw_closed (full close) when width control is disabled or
        no width is known, so the default behaviour is unchanged. Otherwise
        linearly interpolates: width 0 → full close, width >= max → min close.
        """
        cfg = self.cfg
        full_close = cfg.gripper_hw_closed
        if not cfg.grip_width_control or width_m is None:
            return full_close
        if not math.isfinite(float(width_m)) or float(width_m) < 0.0:
            raise ValueError(f"object width must be finite and non-negative, got {width_m}")
        w = float(np.clip(width_m, 0.0, cfg.grip_max_object_width_m))
        max_w = max(cfg.grip_max_object_width_m, 1e-6)
        ratio = w / max_w   # 0 (narrow) … 1 (widest)
        close_deg = full_close - ratio * (full_close - cfg.grip_min_close_deg)
        lo = min(cfg.grip_min_close_deg, full_close)
        hi = max(cfg.grip_min_close_deg, full_close)
        return float(np.clip(close_deg, lo, hi))

    def hw_deg_to_sim_arm(self, hw_degs: List[float]) -> np.ndarray:
        """Convert 5 hardware degrees → sim radians (for observation building)."""
        hw_degs = list(hw_degs)
        if len(hw_degs) != 5 or not all(math.isfinite(float(v)) for v in hw_degs):
            raise ValueError(f"hardware arm angles must be 5 finite values, got {hw_degs}")
        sim = []
        for i, (deg, invert, center, (hw_lo, hw_hi), (sim_lo, sim_hi)) in enumerate(zip(
            hw_degs,
            self.cfg.arm_hw_invert,
            self.cfg.arm_hw_center,
            self.cfg.arm_hw_range,
            self.cfg.arm_sim_limits,
        )):
            if invert:
                sim_deg = center - deg
            else:
                sim_deg = deg - center
            rad = math.radians(sim_deg)
            sim.append(float(np.clip(rad, sim_lo, sim_hi)))
        return np.array(sim, dtype=np.float32)

    def hw_deg_to_sim_grip(self, hw_deg: float) -> float:
        """Convert hardware gripper degree → sim radians."""
        cfg = self.cfg
        if not math.isfinite(float(hw_deg)):
            raise ValueError(f"hardware gripper angle must be finite, got {hw_deg}")
        ratio = (hw_deg - cfg.gripper_hw_open) / (cfg.gripper_hw_closed - cfg.gripper_hw_open + 1e-9)
        rad = cfg.gripper_sim_open + ratio * (cfg.gripper_sim_closed - cfg.gripper_sim_open)
        return float(np.clip(rad, cfg.gripper_sim_open, cfg.gripper_sim_closed))

    def norm_action_to_sim_angles(self, action: np.ndarray) -> Tuple[np.ndarray, float]:
        """Convert normalized 6D policy action → (5 arm radians, gripper radian).

        Uses the same _map_norm_to_range logic as robot_grasp_env.py.
        """
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (6,) or not np.all(np.isfinite(action)):
            raise ValueError(f"policy action must be finite shape (6,), got {action}")
        arm_rads = np.zeros(5, dtype=np.float32)
        for i, (norm_val, (lo, hi)) in enumerate(zip(action[:5], self.cfg.arm_sim_limits)):
            norm_val = float(np.clip(norm_val, -1.0, 1.0))
            arm_rads[i] = lo + (norm_val + 1.0) * 0.5 * (hi - lo)

        grip_lo = min(self.cfg.gripper_sim_open, self.cfg.gripper_sim_closed)
        grip_hi = max(self.cfg.gripper_sim_open, self.cfg.gripper_sim_closed)
        grip_norm = float(np.clip(action[5], -1.0, 1.0))
        grip_rad = grip_lo + (grip_norm + 1.0) * 0.5 * (grip_hi - grip_lo)

        return arm_rads, float(grip_rad)


# ═══════════════════════════════════════════════════════════════════════════
# Servo Controller (Yahboom Rosmaster_Lib wrapper)
# ═══════════════════════════════════════════════════════════════════════════

class ServoController:
    def __init__(self, cfg: DeployConfig, dry_run: bool = False):
        self.cfg = cfg
        self._device = None
        self._has_servo = False

        if not dry_run:
            if not _ROSMASTER_AVAILABLE:
                raise RuntimeError(
                    "--real requires Rosmaster_Lib, but the driver is unavailable. "
                    "Install/localize it and run startup_device_check.py first."
                )
            try:
                self._device = _RosmasterCls(com=cfg.serial_port)
                self._device.create_receive_threading()
                self._has_servo = True
                print(f"[INFO] Rosmaster_Lib ready on {cfg.serial_port}.")
            except Exception as e:
                self._close_device()
                raise RuntimeError(
                    f"--real could not initialize Rosmaster_Lib on {cfg.serial_port}: {e}"
                ) from e

        self.dry_run = dry_run
        self._last_deg = list(cfg.home_deg)   # [S1..S5, S6]

    @property
    def device(self):
        """The underlying Rosmaster handle (or None in dry-run / no hardware).

        Exposed so a unified pipeline can drive the chassis (set_car_motion) on
        the SAME serial connection that owns the arm servos.
        """
        return self._device

    def _send_direct(self, deg6: List[float], run_time_ms: int, tag: str = "HOME"):
        """Send degrees bypassing rate limiter. For home/emergency/settle use."""
        deg6, run_time_ms = self._validate_command(deg6, run_time_ms)
        hw_limits = list(self.cfg.arm_hw_range) + [(
            min(self.cfg.gripper_hw_closed, self.cfg.gripper_hw_open),
            max(self.cfg.gripper_hw_closed, self.cfg.gripper_hw_open),
        )]
        safe_deg = [float(np.clip(d, lo, hi)) for d, (lo, hi) in zip(deg6, hw_limits)]
        limits = [(0,180),(0,180),(0,180),(0,180),(0,270),(0,180)]
        oob = [i+1 for i,(v,(lo,hi)) in enumerate(zip(safe_deg,limits)) if not (lo<=v<=hi)]
        flag = f" *** OOB S{oob} ***" if oob else ""
        label = f"{'DRY' if self.dry_run else 'REAL'}-{tag}"
        print(f"[{label}] S1={safe_deg[0]:.1f}° S2={safe_deg[1]:.1f}° "
              f"S3={safe_deg[2]:.1f}° S4={safe_deg[3]:.1f}° "
              f"S5={safe_deg[4]:.1f}° S6={safe_deg[5]:.1f}° t={run_time_ms}ms{flag}")

        if self.dry_run:
            self._last_deg = safe_deg[:]
            return
        try:
            self._device.set_uart_servo_angle_array(
                angle_s=[safe_deg[0], safe_deg[1], safe_deg[2],
                         safe_deg[3], safe_deg[4], safe_deg[5]],
                run_time=run_time_ms,
            )
        except Exception as e:
            raise RuntimeError(f"servo write failed: {e}") from e
        self._last_deg = safe_deg[:]

    def move_to_home(self):
        self._send_direct(list(self.cfg.home_deg), run_time_ms=2000)

    def move_to_grasp_home(self):
        self._send_direct(list(self.cfg.grasp_home_deg), run_time_ms=2000)

    def settle_to(self, deg6: List[float], run_time_ms: int):
        """Smoothly move (no rate limit) to deg6 over run_time_ms. Used at the
        Stage 0→1 lock so the arm reaches its final pose gently before closing."""
        self._send_direct(deg6, run_time_ms=run_time_ms, tag="SETTLE")

    def glide_to(
        self,
        deg6: List[float],
        deg_per_sec: float,
        min_segment_ms: int,
        max_segment_ms: int,
    ) -> int:
        """Move through <=2 s linear segments without exceeding angular speed.

        Rosmaster clamps each command to 2000 ms. Splitting a long glide keeps
        the physical motion at or below ``deg_per_sec`` instead of silently
        accelerating a 3-5 second plan into one 2 second board command.
        Returns the number of generated segments.
        """
        target, _ = self._validate_command(deg6, 0)
        if not math.isfinite(float(deg_per_sec)) or deg_per_sec <= 0.0:
            raise ValueError("glide deg_per_sec must be finite and > 0")
        min_segment_ms = int(min_segment_ms)
        max_segment_ms = min(int(max_segment_ms), 2000)
        if min_segment_ms < 0 or max_segment_ms <= 0 or min_segment_ms > max_segment_ms:
            raise ValueError("invalid glide segment duration bounds")

        start = np.asarray(self._last_deg, dtype=np.float64)
        target_arr = np.asarray(target, dtype=np.float64)
        max_jump = float(np.max(np.abs(target_arr - start)))
        if max_jump == 0.0:
            return 0
        max_jump_per_segment = float(deg_per_sec) * max_segment_ms / 1000.0
        segment_count = max(1, int(math.ceil(max_jump / max_jump_per_segment)))

        previous = start
        for index in range(1, segment_count + 1):
            segment_target = start + (target_arr - start) * (index / segment_count)
            segment_jump = float(np.max(np.abs(segment_target - previous)))
            run_ms = max(min_segment_ms, int(math.ceil(segment_jump / deg_per_sec * 1000.0)))
            if run_ms > max_segment_ms:
                raise RuntimeError("internal glide planning exceeded the hardware segment limit")
            self._send_direct(segment_target.tolist(), run_ms, tag=f"APPROACH {index}/{segment_count}")
            wait_s = run_ms / 1000.0
            if index < segment_count:
                # Re-plan just before the current segment finishes so adjacent
                # board commands remain a continuous trajectory.
                wait_s = max(0.0, wait_s - 0.05)
            time.sleep(wait_s)
            previous = segment_target
        return segment_count

    def send_degrees(self, deg6: List[float], run_time_ms: Optional[int] = None):
        """Send 6-DOF servo command with per-step rate limiting.

        deg6: [S1, S2, S3, S4, S5, S6] in hardware degrees
        """
        if run_time_ms is None:
            run_time_ms = self.cfg.servo_run_time_ms
        deg6, run_time_ms = self._validate_command(deg6, run_time_ms)

        # Rate limiting
        hw_limits = list(self.cfg.arm_hw_range) + [(
            min(self.cfg.gripper_hw_closed, self.cfg.gripper_hw_open),
            max(self.cfg.gripper_hw_closed, self.cfg.gripper_hw_open),
        )]
        safe_deg = []
        for target, prev, (lo, hi) in zip(deg6, self._last_deg, hw_limits):
            delta = float(np.clip(target - prev, -self.cfg.max_delta_deg, self.cfg.max_delta_deg))
            safe_deg.append(float(np.clip(prev + delta, lo, hi)))

        limits = [(0,180),(0,180),(0,180),(0,180),(0,270),(0,180)]
        oob = [i+1 for i,(v,(lo,hi)) in enumerate(zip(safe_deg,limits)) if not (lo<=v<=hi)]
        flag = f" *** OOB S{oob} ***" if oob else ""
        label = "DRY" if self.dry_run else "REAL"
        print(f"[{label}] S1={safe_deg[0]:.1f}° S2={safe_deg[1]:.1f}° "
              f"S3={safe_deg[2]:.1f}° S4={safe_deg[3]:.1f}° "
              f"S5={safe_deg[4]:.1f}° S6={safe_deg[5]:.1f}° "
              f"t={run_time_ms}ms{flag}")

        if self.dry_run:
            self._last_deg = safe_deg[:]
            return

        try:
            # run_time unit: milliseconds (verified against Rosmaster_Lib docs)
            self._device.set_uart_servo_angle_array(
                angle_s=[
                    safe_deg[0], safe_deg[1], safe_deg[2],
                    safe_deg[3], safe_deg[4], safe_deg[5],
                ],
                run_time=run_time_ms,
            )
        except Exception as e:
            raise RuntimeError(f"servo write failed: {e}") from e
        self._last_deg = safe_deg[:]

    @staticmethod
    def _validate_command(deg6: List[float], run_time_ms: int) -> Tuple[List[float], int]:
        values = [float(v) for v in deg6]
        if len(values) != 6:
            raise ValueError(f"servo command needs 6 angles, got {len(values)}")
        if not all(math.isfinite(v) for v in values):
            raise ValueError(f"servo command contains non-finite angle: {values}")
        run_time_ms = int(run_time_ms)
        if run_time_ms < 0:
            raise ValueError(f"servo run time must be non-negative, got {run_time_ms}")
        # Rosmaster_Lib clamps values above 2000 ms internally. Clamp here too so
        # the software timing/logs describe the command the board actually sees.
        return values, min(run_time_ms, 2000)

    def read_degrees(self) -> List[float]:
        """Read current servo positions. Falls back to last command on failure."""
        if self.dry_run or not self._has_servo:
            return self._last_deg[:]
        out = []
        for i in range(1, 7):
            try:
                angle = self._device.get_uart_servo_angle(i)
                if angle is None or angle < 0:
                    angle = self._last_deg[i - 1]
            except Exception:
                angle = self._last_deg[i - 1]
            out.append(float(angle))
        return out

    def emergency_stop(self):
        """Hold the measured pose instead of starting another large motion."""
        hold_deg = self.read_degrees()
        print("[SAFETY] Emergency hold — commanding the current servo pose.")
        self._send_direct(hold_deg, run_time_ms=100, tag="HOLD")

    def close(self):
        self._close_device()

    def _close_device(self):
        device, self._device = self._device, None
        self._has_servo = False
        if device is None:
            return
        try:
            device.cancel_receive_threading()
        except Exception:
            pass
        serial_obj = getattr(device, "ser", None)
        if serial_obj is not None:
            try:
                serial_obj.close()
            except Exception:
                pass


# ═══════════════════════════════════════════════════════════════════════════
# Forward Kinematics via PyBullet DIRECT
# ═══════════════════════════════════════════════════════════════════════════

class FKComputer:
    """Headless PyBullet instance used only for forward kinematics."""

    def __init__(self, urdf_path: str):
        self.physics_client = p.connect(p.DIRECT)
        if self.physics_client < 0:
            raise RuntimeError("PyBullet DIRECT connection failed")
        try:
            p.setGravity(0, 0, -9.81, physicsClientId=self.physics_client)

            urdf_abs = str(Path(__file__).parent / urdf_path)
            if not Path(urdf_abs).exists():
                raise FileNotFoundError(f"URDF not found: {urdf_abs}")

            # IGNORE_VISUAL_SHAPES: the .obj visual meshes are ~88 MB and nothing
            # here renders (DIRECT mode, no getCameraImage). Halves loadURDF with
            # the FK geometry unchanged. NOT the collision flag -- that one does not
            # raise, it silently shifts the reported pad geometry.
            self.body_id = p.loadURDF(
                urdf_abs,
                basePosition=[0, 0, 0],
                useFixedBase=True,
                flags=p.URDF_IGNORE_VISUAL_SHAPES,
                physicsClientId=self.physics_client,
            )

            # Discover arm joint indices (same logic as x3plus_ground_grasp_env.py)
            target_arm = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
            target_grip = ["grip_joint"]
            self.name2id = {}
            self.short2full = {}
            for j in range(p.getNumJoints(self.body_id, physicsClientId=self.physics_client)):
                info = p.getJointInfo(self.body_id, j, physicsClientId=self.physics_client)
                full = info[1].decode("utf-8")
                self.name2id[full] = j
                for t in target_arm + target_grip:
                    if full.endswith(t):
                        self.short2full[t] = full

            missing = [name for name in target_arm if name not in self.short2full]
            if missing:
                raise RuntimeError(f"URDF missing required arm joints: {missing}")
            self.arm_indices = [self.name2id[self.short2full[t]] for t in target_arm]
            grip_full = self.short2full.get("grip_joint")
            self.grip_index = self.name2id[grip_full] if grip_full else None
            self.ee_link = self.arm_indices[-1]   # arm_joint5 child link
        except Exception:
            p.disconnect(self.physics_client)
            self.physics_client = -1
            raise

        print(f"[FK] PyBullet DIRECT ready. ee_link={self.ee_link}, arm={self.arm_indices}, grip={self.grip_index}")

    def compute(self, arm_rads: np.ndarray, grip_rad: float) -> Tuple[np.ndarray, np.ndarray]:
        """Set joint states and return (tcp_pos, tcp_quat)."""
        for idx, rad in zip(self.arm_indices, arm_rads):
            p.resetJointState(self.body_id, idx, float(rad), physicsClientId=self.physics_client)

        if self.grip_index is not None:
            p.resetJointState(self.body_id, self.grip_index, float(grip_rad), physicsClientId=self.physics_client)

        p.stepSimulation(physicsClientId=self.physics_client)

        state = p.getLinkState(
            self.body_id, self.ee_link,
            computeForwardKinematics=True,
            physicsClientId=self.physics_client,
        )
        tcp_pos = np.array(state[0], dtype=np.float32)
        tcp_quat = np.array(state[1], dtype=np.float32)   # (x, y, z, w)
        return tcp_pos, tcp_quat

    def close(self):
        if self.physics_client >= 0:
            p.disconnect(self.physics_client)
            self.physics_client = -1


# ═══════════════════════════════════════════════════════════════════════════
# Object Detection Receiver (TCP socket)
# ═══════════════════════════════════════════════════════════════════════════

class DetectionReceiver:
    """Listen for JSON object-position messages on a TCP socket.

    Expected JSON format:
        {"x": 0.20, "y": 0.00, "z": 0.02, "w": 0.03}

    "w" (object width in metres) is optional and only used when the grasp
    script runs with --width-grip. The sender (camera node) connects, sends the
    JSON, and can disconnect. The latest received values are kept until a new
    one arrives.
    """

    MAX_MESSAGE_BYTES = 4096

    def __init__(self, host: str, port: int, default_pos: Tuple[float, float, float], stale_timeout_sec: float = 1.0):
        self._default_pos = np.array(default_pos, dtype=np.float32)
        self._pos = self._default_pos.copy()
        self._width: Optional[float] = None
        self._stale_timeout_sec = float(max(0.0, stale_timeout_sec))
        self._last_update_ts = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server.bind((host, port))
            self._server.listen(5)
            self._server.settimeout(1.0)
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
        except Exception:
            self._server.close()
            raise
        print(f"[DetectionReceiver] Listening on {host}:{port} | default={list(default_pos)}")

    def _loop(self):
        while not self._stop.is_set():
            conn = None
            try:
                conn, addr = self._server.accept()
                conn.settimeout(2.0)
                data = b""
                while True:
                    chunk = conn.recv(1024)
                    if not chunk:
                        break
                    if len(data) + len(chunk) > self.MAX_MESSAGE_BYTES:
                        raise ValueError(
                            f"detection message exceeds {self.MAX_MESSAGE_BYTES} bytes"
                        )
                    data += chunk
                pos, width = self._parse_payload(data)
                with self._lock:
                    self._pos = pos
                    self._width = width
                    self._last_update_ts = time.monotonic()
                print(f"[DetectionReceiver] New obj pos from {addr}: {pos.tolist()}"
                      + (f" w={width:.3f}m" if width is not None else ""))
            except socket.timeout:
                continue
            except OSError as e:
                if self._stop.is_set():
                    break
                print(f"[DetectionReceiver] Socket error: {e}")
            except Exception as e:
                print(f"[DetectionReceiver] Parse error: {e}")
            finally:
                if conn is not None:
                    try:
                        conn.close()
                    except OSError:
                        pass

    @staticmethod
    def _parse_payload(data: bytes) -> Tuple[np.ndarray, Optional[float]]:
        if not data:
            raise ValueError("empty detection message")
        msg = json.loads(data.decode("utf-8"))
        if not isinstance(msg, dict):
            raise ValueError("detection payload must be a JSON object")
        pos = np.array(
            [float(msg["x"]), float(msg["y"]), float(msg.get("z", 0.02))],
            dtype=np.float32,
        )
        if not np.all(np.isfinite(pos)):
            raise ValueError(f"detection position must be finite, got {pos.tolist()}")
        width_raw = msg.get("w", None)
        width = float(width_raw) if width_raw is not None else None
        if width is not None and (not math.isfinite(width) or width < 0.0):
            raise ValueError(f"detection width must be finite and non-negative, got {width}")
        return pos, width

    def _is_stale(self) -> bool:
        return (self._stale_timeout_sec > 0.0
                and self._last_update_ts > 0.0
                and (time.monotonic() - self._last_update_ts) > self._stale_timeout_sec)

    def get(self) -> np.ndarray:
        with self._lock:
            if self._is_stale():
                return self._default_pos.copy()
            return self._pos.copy()

    def get_width(self) -> Optional[float]:
        """Latest detected object width (m), or None if unknown/stale."""
        with self._lock:
            if self._is_stale():
                return None
            return self._width

    def snapshot(self) -> Tuple[np.ndarray, Optional[float], bool]:
        """Atomically return (pos, width, is_fresh).

        is_fresh is True only when a real detection has been received and is not
        stale (used by the latch logic to avoid freezing the default position).
        """
        with self._lock:
            fresh = self._last_update_ts > 0.0 and not self._is_stale()
            if fresh:
                return self._pos.copy(), self._width, True
            return self._default_pos.copy(), None, False

    def discard_pending(self) -> None:
        """Forget detections captured before the camera reaches its valid pose."""
        with self._lock:
            self._pos = self._default_pos.copy()
            self._width = None
            self._last_update_ts = 0.0

    def close(self):
        self._stop.set()
        try:
            self._server.close()
        except OSError:
            pass
        self._thread.join(timeout=2.0)


# ═══════════════════════════════════════════════════════════════════════════
# Observation Builder
# ═══════════════════════════════════════════════════════════════════════════

class ObsBuilder:
    """Build the 28D observation matching robot_grasp_env.py (student 6D env).

    Observation layout (28D):
      [0:5]   arm joint angles in sim radians
      [5]     gripper joint angle in sim radians
      [6:9]   TCP position (metres)
      [9:13]  TCP quaternion (x, y, z, w)
      [13:16] object position (metres)
      [16:19] relative position = obj_pos - tcp_pos
      [19:22] stage one-hot [stage0, stage1, stage2]
      [22:28] previous action (6D)
    """

    def __init__(self, fk: FKComputer, mapper: JointMapper, vec_normalize: Optional[VecNormalize]):
        self.fk = fk
        self.mapper = mapper
        self.vec_normalize = vec_normalize

    def build(
        self,
        arm_sim_rads: np.ndarray,
        grip_sim_rad: float,
        obj_pos: np.ndarray,
        stage: int,
        prev_action: np.ndarray,
    ) -> np.ndarray:
        tcp_pos, tcp_quat = self.fk.compute(arm_sim_rads, grip_sim_rad)
        rel_pos = obj_pos - tcp_pos

        stage_onehot = np.zeros(3, dtype=np.float32)
        stage_onehot[int(np.clip(stage, 0, 2))] = 1.0

        obs = np.concatenate([
            arm_sim_rads.astype(np.float32),            # 5
            np.array([grip_sim_rad], dtype=np.float32), # 1
            tcp_pos.astype(np.float32),                  # 3
            tcp_quat.astype(np.float32),                 # 4
            obj_pos.astype(np.float32),                  # 3
            rel_pos.astype(np.float32),                  # 3
            stage_onehot,                                # 3
            prev_action.astype(np.float32),              # 6
        ])

        assert obs.shape == (28,), f"Expected 28D obs, got {obs.shape}"
        if not np.all(np.isfinite(obs)):
            raise RuntimeError("28D observation contains NaN or infinity")

        if self.vec_normalize is not None and self.vec_normalize.norm_obs:
            obs = self.vec_normalize.normalize_obs(obs.reshape(1, -1)).reshape(-1)
            if not np.all(np.isfinite(obs)):
                raise RuntimeError("normalized 28D observation contains NaN or infinity")

        return obs.astype(np.float32)


# ═══════════════════════════════════════════════════════════════════════════
# Mock environment for VecNormalize loading (no PyBullet needed)
# ═══════════════════════════════════════════════════════════════════════════

class _MockEnv28(gym.Env):
    observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(28,), dtype=np.float32)
    action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(6,), dtype=np.float32)

    def step(self, a):
        return np.zeros(28, dtype=np.float32), 0.0, False, False, {}

    def reset(self, **kw):
        return np.zeros(28, dtype=np.float32), {}


# ═══════════════════════════════════════════════════════════════════════════
# Main Controller
# ═══════════════════════════════════════════════════════════════════════════

class GraspController:
    def __init__(self, cfg: DeployConfig, real_servo: bool, use_socket: bool,
                 obj_provider: Optional[Callable[[], Tuple[np.ndarray, Optional[float]]]] = None):
        self.cfg = cfg
        self.real_servo = bool(real_servo)
        self.script_dir = Path(__file__).parent
        self.fk: Optional[FKComputer] = None
        self.servo: Optional[ServoController] = None
        self.detection: Optional[DetectionReceiver] = None
        self._validate_config()
        # External object source (unified pipeline): callable returning
        # (pos_xyz: np.ndarray, width_m: Optional[float]). When set it overrides
        # both the TCP socket and the fixed object (and the latch logic), so the
        # navigator can feed the object it located in-process. Default None keeps
        # the original --socket / --obj-x behaviour unchanged.
        self.obj_provider = obj_provider

        # ── Load model ────────────────────────────────────────────────────
        model_path = self._resolve_weight_file(
            cfg.model_path,
            fallback_names=["ppo_6d_final_ready_for_real_robot.zip"],
        )
        vecnorm_path = self._resolve_weight_file(
            cfg.vecnorm_path,
            fallback_names=["vecnormalize_6d_final.pkl"],
        )
        print(f"[Init] Loading model: {model_path}")
        self.model = PPO.load(str(model_path), device="cpu")
        obs_shape = tuple(self.model.observation_space.shape)
        action_shape = tuple(self.model.action_space.shape)
        if obs_shape != (28,) or action_shape != (6,):
            raise RuntimeError(
                f"grasp model spaces must be obs=(28,), action=(6,), got "
                f"obs={obs_shape}, action={action_shape}"
            )

        print(f"[Init] Loading VecNormalize: {vecnorm_path}")
        mock_venv = DummyVecEnv([lambda: _MockEnv28()])
        try:
            loaded = VecNormalize.load(str(vecnorm_path), mock_venv)
            stats_shape = tuple(np.asarray(loaded.obs_rms.mean).shape)
            if stats_shape != (28,):
                raise RuntimeError(f"VecNormalize observation stats must be (28,), got {stats_shape}")
            vec_norm = VecNormalize(mock_venv, norm_obs=True, norm_reward=False)
            vec_norm.obs_rms = loaded.obs_rms
            vec_norm.training = False
            vec_norm.norm_reward = False
        except Exception as e:
            if real_servo:
                raise RuntimeError(f"VecNormalize load failed in --real mode: {e}") from e
            print(f"[WARNING] VecNormalize load failed: {e}. Running without normalization.")
            vec_norm = None

        # ── FK computer ───────────────────────────────────────────────────
        try:
            self.fk = FKComputer(cfg.urdf_path)

            # ── Mapper & obs builder ──────────────────────────────────────
            self.mapper = JointMapper(cfg)
            self.obs_builder = ObsBuilder(self.fk, self.mapper, vec_norm)

            # ── Servo controller ──────────────────────────────────────────
            self.servo = ServoController(cfg, dry_run=not real_servo)

            # ── Object detection ──────────────────────────────────────────
            self._fixed_obj = np.array(cfg.default_obj_pos, dtype=np.float32)
            if use_socket:
                self.detection = DetectionReceiver(
                    cfg.socket_host,
                    cfg.socket_port,
                    cfg.default_obj_pos,
                    stale_timeout_sec=cfg.detection_stale_timeout_sec,
                )
        except Exception:
            self.close()
            raise

        # ── State ─────────────────────────────────────────────────────────
        self._stage = 0
        self._stage1_steps_left = 0
        self._stage1_arm_hold: Optional[List[float]] = None
        self._stage1_close_deg = cfg.gripper_hw_closed
        self._obj_latched = False
        self._latched_obj: Optional[np.ndarray] = None
        self._latched_width: Optional[float] = None
        self._grasp_succeeded = False
        self._prev_action = np.zeros(6, dtype=np.float32)
        self._current_arm_rads = self.mapper.hw_deg_to_sim_arm(list(cfg.grasp_home_deg[:5]))
        self._current_grip_rad = self.mapper.hw_deg_to_sim_grip(cfg.grasp_home_deg[5])

        print("[Init] All systems ready.")

    def _validate_config(self) -> None:
        cfg = self.cfg
        numeric_positive = {
            "control_hz": cfg.control_hz,
            "max_delta_deg": cfg.max_delta_deg,
            "grip_max_object_width_m": cfg.grip_max_object_width_m,
            "smooth_approach_deg_per_sec": cfg.smooth_approach_deg_per_sec,
            "grasp_home_max_target_distance_m": cfg.grasp_home_max_target_distance_m,
        }
        for name, value in numeric_positive.items():
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f"{name} must be finite and > 0, got {value}")
        margin = cfg.smooth_approach_limit_margin_deg
        if not math.isfinite(float(margin)) or float(margin) < 0.0:
            raise ValueError(
                "smooth_approach_limit_margin_deg must be finite and >= 0, "
                f"got {margin}"
            )
        for name, pose in (("home_deg", cfg.home_deg), ("grasp_home_deg", cfg.grasp_home_deg)):
            if len(pose) != 6 or not all(math.isfinite(float(v)) for v in pose):
                raise ValueError(f"{name} must contain 6 finite angles, got {pose}")
        default_obj = np.asarray(cfg.default_obj_pos, dtype=np.float32)
        if default_obj.shape != (3,) or not np.all(np.isfinite(default_obj)):
            raise ValueError(f"default_obj_pos must contain 3 finite values, got {cfg.default_obj_pos}")
        if (cfg.smooth_approach_min_ms < 0
                or cfg.smooth_approach_max_ms <= 0
                or cfg.smooth_approach_min_ms > min(cfg.smooth_approach_max_ms, 2000)):
            raise ValueError("smooth approach duration bounds are invalid for the 2000 ms board limit")

    def _predict_action(self, obs: np.ndarray) -> np.ndarray:
        action, _ = self.model.predict(obs, deterministic=True)
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape != (6,) or not np.all(np.isfinite(action)):
            raise RuntimeError(f"grasp policy returned invalid action: {action}")
        return np.clip(action, -1.0, 1.0)

    def _abs(self, path: str) -> Path:
        p_abs = Path(path)
        if not p_abs.is_absolute():
            p_abs = self.script_dir / path
        if not p_abs.exists():
            raise FileNotFoundError(f"File not found: {p_abs}")
        return p_abs

    def _resolve_weight_file(self, path: str, fallback_names: List[str]) -> Path:
        """Resolve model/stat files from deploy folder or known subfolder.

        Search order:
        1) Provided absolute path
        2) script_dir / provided path
        3) script_dir / basename(provided path)
        4) script_dir / trained_6d_models_v17 / basename(provided path)
        5) fallback_names in script_dir and trained_6d_models_v17

        Arbitrary glob fallback is intentionally forbidden: a model and its
        VecNormalize statistics must be selected as a verified pair.
        """
        raw = Path(path)
        candidates: List[Path] = []

        if raw.is_absolute():
            candidates.append(raw)
        else:
            candidates.append(self.script_dir / raw)
            candidates.append(self.script_dir / raw.name)
            candidates.append(self.script_dir / "trained_6d_models_v17" / raw.name)

        for name in fallback_names:
            candidates.append(self.script_dir / name)
            candidates.append(self.script_dir / "trained_6d_models_v17" / name)

        for c in candidates:
            if c.exists():
                return c

        raise FileNotFoundError(
            f"Could not resolve weight file from '{path}'. "
            "Refusing to select an arbitrary fallback because the PPO model and "
            "VecNormalize statistics must remain an intentional pair."
        )

    def _get_obj_pos(self) -> np.ndarray:
        if self.obj_provider is not None:
            pos = np.asarray(self.obj_provider()[0], dtype=np.float32).reshape(-1)
            if pos.shape != (3,) or not np.all(np.isfinite(pos)):
                raise RuntimeError(f"object provider returned invalid XYZ: {pos}")
            return pos.copy()
        if self.cfg.latch_obj and self._obj_latched and self._latched_obj is not None:
            return self._latched_obj.copy()
        if self.detection is not None:
            return self.detection.get()
        return self._fixed_obj.copy()

    def _get_obj_width(self) -> Optional[float]:
        """Latest detected object width (m), or None when unavailable."""
        if self.obj_provider is not None:
            width = self.obj_provider()[1]
            if width is None:
                return None
            width = float(width)
            if not math.isfinite(width) or width < 0.0:
                raise RuntimeError(f"object provider returned invalid width: {width}")
            return width
        if self.cfg.latch_obj and self._obj_latched:
            return self._latched_width
        if self.detection is not None:
            return self.detection.get_width()
        return None

    def _prepare_grasp_home(self) -> None:
        """Move once to grasp home, settle, then latch a post-settle detection."""
        self._obj_latched = False
        print(f"[Start] Moving to PPO grasp home {list(self.cfg.grasp_home_deg)}...")
        self.servo.move_to_grasp_home()
        time.sleep(self.cfg.preposition_wait_sec)
        self._sync_joint_state_from_servos()

        if self.cfg.latch_obj:
            if self.detection is not None:
                # A packet received while the arm/camera was moving is not in
                # the calibrated grasp-home camera frame. Require a later one.
                self.detection.discard_pending()
            self._latch_object()

    def _latch_object(self):
        """Capture object position + width ONCE at the grasp-home pose and freeze them.

        Solves the moving-arm-camera problem: the arm camera's distance model is
        valid only at the calibrated grasp-home pose, so we read a single
        post-settle detection here and reuse it for the whole episode.
        """
        if self.detection is None:
            # No detection source — the fixed object is already static.
            self._latched_obj = self._fixed_obj.copy()
            self._latched_width = None
            self._obj_latched = True
            print(f"[Latch] No detection source; using fixed object {self._latched_obj.tolist()}")
            return

        print(f"[Latch] Waiting up to {self.cfg.latch_wait_sec:.1f}s for "
              "one fresh socket detection...")
        deadline = time.time() + self.cfg.latch_wait_sec
        pos, width, fresh = self.detection.snapshot()
        while not fresh and time.time() < deadline:
            time.sleep(0.05)
            pos, width, fresh = self.detection.snapshot()

        self._latched_obj = pos
        self._latched_width = width
        self._obj_latched = True
        if fresh:
            print(f"[Latch] Object latched at grasp-home pose: pos={pos.tolist()}"
                  + (f" w={width:.3f}m" if width is not None else " (no width)"))
        else:
            if self.real_servo:
                raise RuntimeError(
                    "no fresh socket detection arrived before latch timeout; "
                    "refusing to move the real arm toward the default position"
                )
            print(f"[Latch] WARNING: no detection within {self.cfg.latch_wait_sec}s; "
                  f"falling back to {pos.tolist()}")

    def _sync_joint_state_from_servos(self):
        """Read current servo angles and convert to sim space."""
        hw_deg = self.servo.read_degrees()   # [S1..S5, S6]
        self._current_arm_rads = self.mapper.hw_deg_to_sim_arm(hw_deg[:5])
        self._current_grip_rad = self.mapper.hw_deg_to_sim_grip(hw_deg[5])

    def _virtual_rollout_to_lock(self, max_steps: int):
        """Replay the Stage-0 loop purely in software (no servo commands).

        Because obs come from the rate-limited commanded state (not measured
        feedback), the policy is deterministic and the object position is
        static for the episode, this reproduces EXACTLY the command sequence
        the streamed loop would send. Returns (lock_deg6, prev_action,
        n_steps, lock_dist) at the Stage-1 lock, or None if Stage 1 never
        triggers (caller falls back to the normal streamed approach).

        NOTE: the rate-limit math below must mirror ServoController.send_degrees().
        """
        arm_rads = self._current_arm_rads.copy()
        grip_rad = self._current_grip_rad
        prev_action = self._prev_action.copy()
        last_deg = list(self.servo._last_deg)
        obj_pos = self._get_obj_pos()
        initial_dist = None
        min_dist = float("inf")
        hw_limits = list(self.cfg.arm_hw_range) + [(
            min(self.cfg.gripper_hw_closed, self.cfg.gripper_hw_open),
            max(self.cfg.gripper_hw_closed, self.cfg.gripper_hw_open),
        )]
        for step in range(max_steps):
            obs = self.obs_builder.build(arm_rads, grip_rad, obj_pos, 0, prev_action)
            action = self._predict_action(obs)
            current_tcp, _ = self.fk.compute(arm_rads, grip_rad)
            dist = float(np.linalg.norm(obj_pos - current_tcp))
            if initial_dist is None:
                initial_dist = dist
            min_dist = min(min_dist, dist)
            grip_cmd = float(action[5])
            made_progress = min_dist < initial_dist * 0.6
            diverging = (step > 10 and made_progress
                         and dist > min_dist + self.cfg.min_dist_rebound_m)
            triggered = (dist < self.cfg.stage0_dist_threshold
                         or grip_cmd > self.cfg.grip_close_threshold
                         or diverging)
            # Mirror of send_degrees() rate limiting, applied to virtual state.
            # (The real loop still sends the triggering iteration's action and
            # locks on the NEXT iteration, so apply the send before returning.)
            arm_sim, grip_sim = self.mapper.norm_action_to_sim_angles(action)
            deg6 = self.mapper.sim_arm_to_hw_deg(arm_sim) + [self.mapper.sim_grip_to_hw_deg(grip_sim)]
            for i, (target, prevd, (lo, hi)) in enumerate(zip(deg6, last_deg, hw_limits)):
                delta = float(np.clip(target - prevd,
                                      -self.cfg.max_delta_deg, self.cfg.max_delta_deg))
                last_deg[i] = float(np.clip(prevd + delta, lo, hi))
            arm_rads = self.mapper.hw_deg_to_sim_arm(last_deg[:5])
            grip_rad = self.mapper.hw_deg_to_sim_grip(last_deg[5])
            prev_action = action.copy()
            if triggered:
                lock_tcp, _ = self.fk.compute(arm_rads, grip_rad)
                lock_dist = float(np.linalg.norm(obj_pos - lock_tcp))
                return list(last_deg), prev_action, step + 1, lock_dist
        return None

    def run(self, max_steps: int = 300):
        if max_steps <= 0:
            raise ValueError(f"max_steps must be > 0, got {max_steps}")
        print("\n" + "="*60)
        print("X3Plus Real Grasp Controller")
        print(f"  Stage 0  → align gripper above object")
        print(f"  Stage 1  → close gripper (grasp)")
        print(f"  Stage 2  → lift and return home")
        print("="*60 + "\n")

        self._stage = 0
        self._stage1_steps_left = 0
        self._stage1_arm_hold = None
        self._stage1_close_deg = self.cfg.gripper_hw_closed  # set per-grasp at 0→1 (width-aware)
        self._grasp_succeeded = False
        self._prev_action = np.zeros(6, dtype=np.float32)
        self._min_dist_seen = float('inf')
        self._min_dist_step = 0
        self._initial_dist = None

        # grasp_home_deg is both the calibrated detection pose and the PPO
        # training start. Move there exactly once, settle, then latch one new
        # camera observation before any policy-driven motion.
        self._prepare_grasp_home()

        dt = 1.0 / self.cfg.control_hz
        obj_pos = self._get_obj_pos()
        home_tcp, _ = self.fk.compute(self._current_arm_rads, self._current_grip_rad)
        target_distance_m = validate_grasp_home_reach(
            obj_pos,
            home_tcp,
            self.cfg.grasp_home_max_target_distance_m,
        )
        print(f"[Start] Grasp home TCP (URDF): {home_tcp.round(4).tolist()}")
        print(f"[Start] Object position: {obj_pos.tolist()}")
        print(f"[Reach] Target is {target_distance_m:.3f}m from grasp-home TCP "
              f"(limit {self.cfg.grasp_home_max_target_distance_m:.3f}m)")
        print(f"[Start] Running for up to {max_steps} steps at {self.cfg.control_hz} Hz\n")

        # ── Smooth approach: pre-compute the lock pose, glide there once ──
        if self.cfg.smooth_approach:
            if self.detection is not None and not self.cfg.latch_obj:
                print("[Smooth][WARN] --smooth-approach plans the whole approach "
                      "from ONE object snapshot, but --socket without --latch-obj "
                      "is a live source — a stale/default detection would send "
                      "the glide to a phantom position. Add --latch-obj.")
            rollout = self._virtual_rollout_to_lock(max_steps)
            if rollout is None:
                message = "[Smooth] Virtual rollout never triggered Stage 1"
                if self.real_servo:
                    raise RuntimeError(
                        message + " — refusing real-arm fallback to streamed motion."
                    )
                print(message + " — falling back to the streamed approach.")
            else:
                lock_deg6, prev_action, n_steps, lock_dist = rollout
                limit_hits = arm_limit_margin_violations(
                    lock_deg6[:5],
                    self.cfg.arm_hw_range,
                    self.cfg.smooth_approach_limit_margin_deg,
                )
                if self.real_servo and limit_hits:
                    raise RuntimeError(
                        "[Smooth] virtual lock pose is too close to a hardware "
                        "joint limit; refusing real glide: " + ", ".join(limit_hits)
                    )
                # All 6 channels (incl. S6): the gripper may travel far when the
                # rollout half-closes it, and it should glide, not slew.
                max_jump = max(abs(t - c) for t, c
                               in zip(lock_deg6, self.servo._last_deg))
                max_segment_ms = min(self.cfg.smooth_approach_max_ms, 2000)
                planned_segments = max(1, math.ceil(
                    max_jump / (self.cfg.smooth_approach_deg_per_sec
                                * max_segment_ms / 1000.0)
                ))
                print(f"[Smooth] Lock pose after {n_steps} virtual steps, "
                      f"dist@lock={lock_dist:.3f}m → {planned_segments} glide segment(s) to "
                      f"{[round(d, 1) for d in lock_deg6[:5]]}")
                self.servo.glide_to(
                    lock_deg6,
                    self.cfg.smooth_approach_deg_per_sec,
                    self.cfg.smooth_approach_min_ms,
                    self.cfg.smooth_approach_max_ms,
                )
                time.sleep(0.25)
                # Adopt the rollout end state so Stage 1 continues seamlessly
                # (the Stage-1 branch will lock at this pose, settle and close).
                self._current_arm_rads = self.mapper.hw_deg_to_sim_arm(self.servo._last_deg[:5])
                self._current_grip_rad = self.mapper.hw_deg_to_sim_grip(self.servo._last_deg[5])
                self._prev_action = prev_action
                self._stage = 1
                obj_w = self._get_obj_width()
                self._stage1_close_deg = self.mapper.width_to_close_deg(obj_w)
                if self.cfg.grip_width_control:
                    print(f"[Smooth] width={'n/a' if obj_w is None else f'{obj_w:.3f}m'} "
                          f"→ close S6 to {self._stage1_close_deg:.1f}°")
                self._stage1_steps_left = math.ceil(
                    abs(self.cfg.gripper_hw_open - self._stage1_close_deg) / self.cfg.max_delta_deg
                ) + 2

        for step in range(max_steps):
            t0 = time.time()

            obj_pos = self._get_obj_pos()
            obs = self.obs_builder.build(
                self._current_arm_rads,
                self._current_grip_rad,
                obj_pos,
                self._stage,
                self._prev_action,
            )

            # ── Model inference ───────────────────────────────────────────
            action = self._predict_action(obs)

            # ── Stage management ──────────────────────────────────────────
            # Use CURRENT arm state for distance, not the action target
            current_tcp, _ = self.fk.compute(self._current_arm_rads, self._current_grip_rad)
            dist_to_obj = float(np.linalg.norm(obj_pos - current_tcp))
            if self._initial_dist is None:
                self._initial_dist = dist_to_obj
            if dist_to_obj < self._min_dist_seen:
                self._min_dist_seen = dist_to_obj
                self._min_dist_step = step
            arm_sim, grip_sim = self.mapper.norm_action_to_sim_angles(action)

            # Stage transitions
            if self._stage == 0:
                grip_cmd = float(action[5])
                made_progress = (self._initial_dist is not None
                                and self._min_dist_seen < self._initial_dist * 0.6)
                diverging = (step > 10
                             and made_progress
                             and dist_to_obj > self._min_dist_seen + self.cfg.min_dist_rebound_m)
                if dist_to_obj < self.cfg.stage0_dist_threshold or grip_cmd > self.cfg.grip_close_threshold or diverging:
                    reason = ("dist_thresh" if dist_to_obj < self.cfg.stage0_dist_threshold
                              else "grip_thresh" if grip_cmd > self.cfg.grip_close_threshold
                              else f"diverging(min={self._min_dist_seen:.3f}m)")
                    print(f"\n[Stage] 0→1  dist={dist_to_obj:.3f}m  grip_cmd={grip_cmd:.2f}  reason={reason}")
                    self._stage = 1
                    # Width-aware close target: full close by default; if --width-grip
                    # and a detection width is known, stop the fingers earlier.
                    obj_w = self._get_obj_width()
                    self._stage1_close_deg = self.mapper.width_to_close_deg(obj_w)
                    if self.cfg.grip_width_control:
                        print(f"[Stage] 1: width={'n/a' if obj_w is None else f'{obj_w:.3f}m'} "
                              f"→ close S6 to {self._stage1_close_deg:.1f}°")
                    self._stage1_steps_left = math.ceil(
                        abs(self.cfg.gripper_hw_open - self._stage1_close_deg) / self.cfg.max_delta_deg
                    ) + 2

            elif self._stage == 1:
                # On first entry, snapshot arm hw degrees and lock for whole Stage 1.
                # Policy's arm action is ignored — only S6 drives toward the
                # (width-aware) close target self._stage1_close_deg.
                if self._stage1_arm_hold is None:
                    self._stage1_arm_hold = list(self.servo._last_deg[:5])
                    print(f"\n[Stage] 1: Arm locked at "
                          f"{[round(d,1) for d in self._stage1_arm_hold]}")
                    # Ease into the lock pose (gripper still open) and let the arm
                    # physically arrive before closing, so servo catch-up lag does
                    # not show up as a forward lurch as the gripper starts closing.
                    settle_deg = self._stage1_arm_hold + [self.servo._last_deg[5]]
                    self.servo.settle_to(settle_deg, self.cfg.stage1_settle_run_time_ms)
                    time.sleep(self.cfg.stage1_settle_wait_sec)

                deg6 = self._stage1_arm_hold + [self._stage1_close_deg]
                self.servo.send_degrees(deg6, run_time_ms=self.cfg.servo_run_time_ms)
                self._current_arm_rads = self.mapper.hw_deg_to_sim_arm(self.servo._last_deg[:5])
                self._current_grip_rad = self.mapper.hw_deg_to_sim_grip(self.servo._last_deg[5])
                self._prev_action = action.copy()
                self._stage1_steps_left -= 1
                print(f"\r[Stage] 1: Closing gripper... steps_left={self._stage1_steps_left} "
                      f"S6={self.servo._last_deg[5]:.1f}° dist={dist_to_obj:.3f}m",
                      end="", flush=True)
                if self._stage1_steps_left <= 0:
                    self._stage = 2
                    print("\n[Stage] 1→2  Grasped. Returning home...")
                elapsed = time.time() - t0
                if dt - elapsed > 0:
                    time.sleep(dt - elapsed)
                continue

            elif self._stage == 2:
                # Bypass policy: command home arm pose; force gripper to stay CLOSED
                # (home_deg[5]=30° = OPEN under new convention, so override with the
                # width-aware close angle held since Stage 1)
                hw_home_with_grip = list(self.cfg.home_deg[:5]) + [self._stage1_close_deg]
                self.servo.send_degrees(hw_home_with_grip, run_time_ms=self.cfg.servo_run_time_ms)
                self._current_arm_rads = self.mapper.hw_deg_to_sim_arm(self.servo._last_deg[:5])
                self._current_grip_rad = self.mapper.hw_deg_to_sim_grip(self.servo._last_deg[5])
                self._prev_action = action.copy()

                home_arm_rads = self.mapper.hw_deg_to_sim_arm(list(self.cfg.home_deg[:5]))
                dist_home = float(np.linalg.norm(home_arm_rads - self._current_arm_rads))
                print(f"\r[Stage] 2: Returning home... dist={dist_home:.3f}rad", end="", flush=True)
                if dist_home < self.cfg.stage2_dist_threshold:
                    print("\n[Done] Returned to home. Object held.")
                    # Final snap to home arm pose; gripper stays CLOSED (holding object)
                    self.servo.send_degrees(
                        list(self.cfg.home_deg[:5]) + [self._stage1_close_deg],
                        run_time_ms=1500,
                    )
                    self._grasp_succeeded = True
                    break

                elapsed = time.time() - t0
                if dt - elapsed > 0:
                    time.sleep(dt - elapsed)
                continue

            # ── Convert action → servo degrees ────────────────────────────
            arm_sim, grip_sim = self.mapper.norm_action_to_sim_angles(action)
            hw_arm = self.mapper.sim_arm_to_hw_deg(arm_sim)
            hw_grip = self.mapper.sim_grip_to_hw_deg(grip_sim)
            deg6 = hw_arm + [hw_grip]

            # Stage 0 only reaches here (Stages 1/2 send + continue above).
            # Longer run_time → mid-motion re-planning → no stop-start stutter.
            self.servo.send_degrees(deg6, run_time_ms=self.cfg.stage0_run_time_ms)
            # Use rate-limited degrees so obs matches what servo actually reached
            self._current_arm_rads = self.mapper.hw_deg_to_sim_arm(self.servo._last_deg[:5])
            self._current_grip_rad = self.mapper.hw_deg_to_sim_grip(self.servo._last_deg[5])
            self._prev_action = action.copy()

            # Status
            print(f"\r[Step {step+1:3d}] Stage={self._stage} "
                  f"dist={dist_to_obj:.3f}m(min={self._min_dist_seen:.3f}) "
                  f"tcp={current_tcp.round(3).tolist()} "
                  f"grip={float(action[5]):.2f} "
                  f"S1-6={[round(d,1) for d in deg6]}",
                  end="", flush=True)

            elapsed = time.time() - t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)
        else:
            print(f"\n[End] Max steps ({max_steps}) reached.")

        # Return to home (only on failure — successful grasp already at home with object held)
        print("\n[End] Moving back to home...")
        if self._grasp_succeeded:
            print("[End] Grasp succeeded — keeping gripper closed, arm already at home.")
        else:
            self.servo.move_to_home()  # opens gripper (home_deg[5]=30°) for retry/reset
            time.sleep(2.0)

        # True only means the 3-stage grasp SEQUENCE completed (arm returned home
        # with gripper closed). Whether the object was actually captured is
        # confirmed separately by re-detection (see pipeline verify_grasp).
        return self._grasp_succeeded

    def close(self):
        detection, self.detection = self.detection, None
        servo, self.servo = self.servo, None
        fk, self.fk = self.fk, None
        if detection is not None:
            try:
                detection.close()
            except Exception as e:
                print(f"[WARN] detection cleanup failed: {e}")
        if servo is not None:
            try:
                servo.close()
            except Exception as e:
                print(f"[WARN] servo cleanup failed: {e}")
        if fk is not None:
            try:
                fk.close()
            except Exception as e:
                print(f"[WARN] PyBullet cleanup failed: {e}")


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════

def parse_deg6_csv(value: str) -> Tuple[float, ...]:
    vals = [float(x.strip()) for x in value.split(",") if x.strip()]
    if len(vals) != 6:
        raise argparse.ArgumentTypeError(f"expected 6 comma-separated degrees, got {len(vals)}")
    return tuple(vals)


def parse_args():
    p = argparse.ArgumentParser(description="X3Plus real-robot grasp deployment")
    p.add_argument("--model", type=str, default=None,
                   help="Path to .zip model (default: trained_6d_models_v17/ppo_6d_final_ready_for_real_robot.zip)")
    p.add_argument("--vecnorm", type=str, default=None,
                   help="Path to vecnormalize .pkl")
    p.add_argument("--stage0-dist-threshold", type=float, default=None,
                   help="TCP-to-object distance (m) that locks the arm and closes the "
                        "gripper (default 0.042). Too large and the arm freezes while "
                        "still descending; too small and Stage 1 may never trigger. "
                        "Re-measure on hardware after changing the grasp home pose.")
    p.add_argument("--real", action="store_true",
                   help="Actually send commands to servos (default: dry-run print only)")
    p.add_argument(
        "--pose-only",
        choices=("grasp-home", "nav-home"),
        default=None,
        help="Move only to the selected camera-calibration pose, wait for it to "
             "settle, and exit while leaving the servos at that pose. Does not "
             "load or run PPO/FK/detection.",
    )
    p.add_argument("--socket", action="store_true",
                   help="Listen for object detection on TCP socket (port 5555)")
    p.add_argument("--i-confirm-external-frame", action="store_true",
                   help="confirm socket XYZ is calibrated in the PPO/URDF base_link frame")
    p.add_argument("--width-grip", action="store_true",
                   help="Use detected object width (JSON 'w') to pick the Stage-1 "
                        "gripper close angle instead of always full-closing")
    p.add_argument("--latch-obj", action="store_true",
                   help="Capture object position/width ONCE after settling at grasp home and "
                        "freeze it for the episode (recommended: arm camera moves "
                        "with the arm, so live detections drift after motion)")
    p.add_argument("--latch-wait-sec", type=float, default=30.0,
                   help="Seconds to wait at grasp home for a fresh post-settle socket "
                        "detection (default 30)")
    p.add_argument("--obj-x", type=float, default=0.25,
                   help="Object X position in metres (forward, default 0.25)")
    p.add_argument("--obj-y", type=float, default=0.00,
                   help="Object Y position in metres (lateral, default 0.00)")
    p.add_argument("--obj-z", type=float, default=0.02,
                   help="Object Z position in metres (height, default 0.02)")
    p.add_argument("--max-steps", type=int, default=300)
    p.add_argument("--hz", type=float, default=10.0)
    p.add_argument("--port", type=str, default="/dev/myserial",
                   help="Serial port for Rosmaster (stable symlink to the ch341 board)")
    p.add_argument("--nav-home-deg", type=parse_deg6_csv, default=None,
                   help="Navigation/cruise home servo degrees as S1,...,S6")
    p.add_argument("--grasp-home-deg", type=parse_deg6_csv, default=None,
                   help="PPO grasp initial servo degrees as S1,...,S6")
    p.add_argument("--smooth-approach", action="store_true",
                   help="Pre-compute the Stage-0 lock pose by rolling the policy "
                         "out virtually, then glide there in hardware-limited segments "
                        "instead of 10 Hz streamed bursts. Same grasp geometry, "
                        "smoother motion. Needs a static object source "
                        "(--obj-x/y/z or --latch-obj).")
    return p.parse_args()


def run_pose_only(cfg: DeployConfig, real_servo: bool, pose_name: str) -> None:
    """Safely command one calibration pose without constructing GraspController."""
    if pose_name not in ("grasp-home", "nav-home"):
        raise ValueError(f"unsupported pose-only target: {pose_name}")

    pose_deg = cfg.grasp_home_deg if pose_name == "grasp-home" else cfg.home_deg
    settle_sec = (
        cfg.preposition_wait_sec if pose_name == "grasp-home"
        else cfg.nav_home_wait_sec
    )
    mode = "REAL" if real_servo else "DRY-RUN"
    print(f"[PoseOnly][{mode}] Commanding {pose_name} pose: {list(pose_deg)}")
    print("[PoseOnly] PPO, FK, socket detection and object reach checks are disabled.")

    servo: Optional[ServoController] = None
    try:
        servo = ServoController(cfg, dry_run=not real_servo)
        if pose_name == "grasp-home":
            servo.move_to_grasp_home()
        else:
            servo.move_to_home()
        print(f"[PoseOnly] Waiting {settle_sec:.1f}s for the arm to settle...")
        time.sleep(settle_sec)
        if real_servo:
            print(f"[PoseOnly] Settled at {pose_name}. No return command will be sent; "
                  "the servos will remain holding this pose after the program exits.")
        else:
            print(f"[PoseOnly][DRY-RUN] Verified {pose_name}; no hardware moved. "
                  "A --real run will leave the servos holding this pose.")
    except KeyboardInterrupt:
        print("\n[PoseOnly] Interrupted; holding the current servo pose.")
        if servo is not None:
            servo.emergency_stop()
    except Exception as exc:
        print(f"\n[PoseOnly][ABORT] {exc}")
        if servo is not None:
            servo.emergency_stop()
        raise SystemExit(1)
    finally:
        if servo is not None:
            servo.close()


def main():
    args = parse_args()
    if args.pose_only is not None:
        incompatible = [
            flag for flag, enabled in (
                ("--socket", args.socket),
                ("--width-grip", args.width_grip),
                ("--latch-obj", args.latch_obj),
                ("--smooth-approach", args.smooth_approach),
            )
            if enabled
        ]
        if incompatible:
            raise SystemExit(
                "--pose-only cannot be combined with grasp/detection flags: "
                + ", ".join(incompatible)
            )
        # Keep this branch before all PPO/object validation. Pose-only is a
        # servo calibration utility and must not construct GraspController.
        pose_cfg = DeployConfig(serial_port=args.port)
        if args.nav_home_deg is not None:
            pose_cfg.home_deg = args.nav_home_deg
        if args.grasp_home_deg is not None:
            pose_cfg.grasp_home_deg = args.grasp_home_deg
        run_pose_only(pose_cfg, real_servo=args.real, pose_name=args.pose_only)
        return

    if (args.model is None) != (args.vecnorm is None):
        raise SystemExit("--model and --vecnorm must be supplied together as a verified pair")
    if not math.isfinite(args.hz) or args.hz <= 0.0:
        raise SystemExit("--hz must be finite and > 0")
    if args.max_steps <= 0:
        raise SystemExit("--max-steps must be > 0")
    if not math.isfinite(args.latch_wait_sec) or args.latch_wait_sec <= 0.0:
        raise SystemExit("--latch-wait-sec must be finite and > 0")
    if args.real and args.socket and not args.i_confirm_external_frame:
        raise SystemExit(
            "real socket grasp refused: confirm the sender outputs calibrated "
            "PPO/URDF base_link XYZ with --i-confirm-external-frame"
        )
    if args.real and args.socket and not args.latch_obj:
        raise SystemExit(
            "real socket grasp requires --latch-obj so the moving arm camera "
            "cannot overwrite the target during the episode"
        )
    cfg = DeployConfig(
        control_hz=args.hz,
        default_obj_pos=(args.obj_x, args.obj_y, args.obj_z),
        serial_port=args.port,
    )
    if args.model:
        cfg.model_path = args.model
    if args.vecnorm:
        cfg.vecnorm_path = args.vecnorm
    if args.nav_home_deg is not None:
        cfg.home_deg = args.nav_home_deg
    if args.grasp_home_deg is not None:
        cfg.grasp_home_deg = args.grasp_home_deg
    if args.stage0_dist_threshold is not None:
        if not math.isfinite(args.stage0_dist_threshold) or args.stage0_dist_threshold <= 0.0:
            raise SystemExit("--stage0-dist-threshold must be finite and > 0")
        cfg.stage0_dist_threshold = args.stage0_dist_threshold
    cfg.grip_width_control = args.width_grip
    cfg.latch_obj = args.latch_obj
    cfg.latch_wait_sec = args.latch_wait_sec
    cfg.smooth_approach = args.smooth_approach

    controller = GraspController(cfg, real_servo=args.real, use_socket=args.socket)
    try:
        controller.run(max_steps=args.max_steps)
    except KeyboardInterrupt:
        print("\n[Interrupted] Emergency stop.")
        controller.servo.emergency_stop()
    except Exception as exc:
        print(f"\n[ABORT] {exc}")
        controller.servo.emergency_stop()
        raise SystemExit(1)
    finally:
        controller.close()


if __name__ == "__main__":
    main()
