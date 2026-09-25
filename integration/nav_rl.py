#!/usr/bin/env python3
"""RL navigation runtime for X3Plus — iGibson-trained PPO nav policy on the real robot.

Weights: `integration/nav_best_model/` (from igibson_x3_test
`nav_best_model/COLLISION_SAFE_BEST_collision_guard_hardfail_0.600`, hard-fail
success rate 0.600). PPO MLP + VecNormalize, 55-D obs, 2-D action.

This module mirrors the TRAINING plant exactly — the policy learned against
these actuation quirks, so the real robot must reproduce them
(source: igibson_x3_test/training/navigation_env_room.py X3PlusNavEnv.step()):

    1. motor delay: the executed action is the one commanded 2 steps ago
    2. stall assist: aligned to a far goal + clear path + standing still
       -> forced minimum forward throttle (0.28)
    3. near-obstacle slowdown: min(lidar) < 0.66 m scales linear x0.24..1,
       angular x0.9..1 (uses the SAME lidar array the policy saw)
    4. throttle -> velocity: forward x0.5 m/s, reverse x0.15 m/s, yaw x1.2 rad/s
       (no mecanum strafe: vy = 0)

Observation layout (55-D, order is load-bearing — see nav_best_model README):

    [0]    dist        robot->goal planar distance (m)
    [1:3]  sin/cos     goal bearing in robot frame (LEFT positive)
    [3:5]  lin_speed, ang_vel_z   executed velocities of the previous step
    [5:7]  prev_lin, prev_ang     previous RAW policy action (pre-delay)
    [7:55] lidar x48   ray distances from ROBOT CENTER, right (-90 deg) to
                       left (+90 deg), clipped to [0.33, 4.0]; no return -> 4.0

Pure logic (runs on a dev machine, no torch/SB3/lidar):
    scan_to_rays / build_nav_obs / shape_action / ActionDelay / GoalTracker

Heavy/hardware pieces (lazy imports):
    NavPolicy (SB3 PPO + VecNormalize pickle), RosLaserScanSource (roslibpy),
    optional legacy RPLidarSource (rplidar package)

CLI:
    python nav_rl.py --selftest                       # pure logic check
    python nav_rl.py --probe                          # verify ROS /scan orientation
    python nav_rl.py --bench                          # load model, measure predict Hz
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import os
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, List, Mapping, Optional, Sequence, Tuple

import numpy as np

_HERE = Path(__file__).resolve().parent
# Default = ppo_nav_281440_steps: training-PC eval (seed 123 x 30, 2026-07-10)
# scored it 0.600 success / 0.267 collision vs best_model's 0.467 / 0.367 —
# better on both axes. best_model.zip is kept only as a fallback checkpoint.
DEFAULT_MODEL = _HERE / "nav_best_model" / "ppo_nav_281440_steps.zip"
DEFAULT_VECNORM = _HERE / "nav_best_model" / "ppo_nav_vecnormalize_281440_steps.pkl"


# ════════════════════════════════════════════════════════════════════════════
# Config — defaults are the TRAINING values (NavTaskConfig merged with the
# P2_J_collision_guard_rewardparams.json overrides). Do not "tune" the plant
# params without retraining; they are part of the dynamics the policy knows.
# ════════════════════════════════════════════════════════════════════════════

@dataclasses.dataclass
class NavRLConfig:
    model_path: str = str(DEFAULT_MODEL)
    vecnorm_path: str = str(DEFAULT_VECNORM)

    # ── plant: velocity scaling (X3PlusNavEnv.step) ──
    max_linear_vel_forward: float = 0.5
    max_linear_vel_reverse: float = 0.15
    max_angular_vel: float = 1.2

    # ── plant: motor delay + control rate ──
    # Sim policy step = action_repeat(5) x sim.step(1/30 s) ~= 0.167 s (6 Hz).
    motor_delay_steps: int = 2
    control_period_s: float = 1.0 / 6.0

    # ── plant: stall assist (JSON override values) ──
    stall_assist_enabled: bool = True
    stall_assist_min_throttle: float = 0.28
    stall_dist_threshold: float = 0.9
    stall_heading_cos_threshold: float = 0.95
    stall_clearance_threshold: float = 0.6
    stall_speed_threshold: float = 0.04

    # ── plant: near-obstacle slowdown (JSON override values) ──
    near_obstacle_slowdown_dist: float = 0.66
    near_obstacle_min_linear_scale: float = 0.24
    near_obstacle_min_angular_scale: float = 0.9

    # ── lidar ray model (must equal training _get_lidar_data) ──
    lidar_num_rays: int = 48
    lidar_fov_deg: float = 180.0
    lidar_max_dist: float = 4.0
    # robot_r = max(collision_threshold 0.22 + 0.02, robot_safety_radius 0.33)
    lidar_min_dist: float = 0.33

    # ── real lidar mounting (calibrate with --probe) ──
    # The verified X3Plus sensor is a YDLIDAR TG30 published by the factory ROS
    # driver on /scan. lidar_port is only used by the optional legacy direct
    # Slamtec/RPLidar backend and has no default, so this TG30 cannot be opened
    # accidentally with the wrong protocol. Never point it at /dev/myserial.
    lidar_port: str = ""
    lidar_yaw_offset_deg: float = 0.0   # lidar 0 deg vs robot forward
    lidar_angle_dir: float = 1.0        # ROS /scan is CCW; direct RPLIDAR needs -1
    lidar_forward_offset_m: float = 0.0  # lidar center ahead(+)/behind(-) robot center

    # ── safety overlay (NOT in training — geometric last-resort brake) ──
    # Uses RAW point distances (the 48-ray obs is floored at 0.33 m and cannot
    # see closer). Blocks forward motion only; turning/reverse stay available.
    safety_brake_dist: float = 0.25
    safety_brake_halfangle_deg: float = 50.0
    lidar_stale_timeout_s: float = 0.7

    # ── goal / handoff (sim task params, used by the pipeline) ──
    goal_threshold: float = 0.35
    success_stay_steps: int = 15


def validate_config(cfg: NavRLConfig) -> None:
    """Reject values that would make the control loop unsafe or ill-defined."""
    positive = {
        "control_period_s": cfg.control_period_s,
        "max_linear_vel_forward": cfg.max_linear_vel_forward,
        "max_linear_vel_reverse": cfg.max_linear_vel_reverse,
        "max_angular_vel": cfg.max_angular_vel,
        "lidar_fov_deg": cfg.lidar_fov_deg,
        "lidar_max_dist": cfg.lidar_max_dist,
        "safety_brake_dist": cfg.safety_brake_dist,
        "safety_brake_halfangle_deg": cfg.safety_brake_halfangle_deg,
        "lidar_stale_timeout_s": cfg.lidar_stale_timeout_s,
    }
    for name, value in positive.items():
        if not math.isfinite(float(value)) or float(value) <= 0.0:
            raise ValueError(f"{name} must be finite and > 0, got {value}")
    if cfg.motor_delay_steps < 0:
        raise ValueError(f"motor_delay_steps must be >= 0, got {cfg.motor_delay_steps}")
    if cfg.lidar_num_rays < 2:
        raise ValueError(f"lidar_num_rays must be >= 2, got {cfg.lidar_num_rays}")
    if not 0.0 < cfg.lidar_min_dist < cfg.lidar_max_dist:
        raise ValueError(
            f"lidar distances must satisfy 0 < min < max, got "
            f"{cfg.lidar_min_dist}, {cfg.lidar_max_dist}"
        )
    if cfg.lidar_fov_deg > 360.0:
        raise ValueError(f"lidar_fov_deg must be <= 360, got {cfg.lidar_fov_deg}")
    if cfg.safety_brake_halfangle_deg > 180.0:
        raise ValueError(
            "safety_brake_halfangle_deg must be <= 180, got "
            f"{cfg.safety_brake_halfangle_deg}"
        )
    if cfg.lidar_angle_dir not in (-1.0, 1.0):
        raise ValueError(f"lidar_angle_dir must be -1 or +1, got {cfg.lidar_angle_dir}")
    for name, value in {
        "lidar_yaw_offset_deg": cfg.lidar_yaw_offset_deg,
        "lidar_forward_offset_m": cfg.lidar_forward_offset_m,
    }.items():
        if not math.isfinite(float(value)):
            raise ValueError(f"{name} must be finite, got {value}")


# ════════════════════════════════════════════════════════════════════════════
# Pure math
# ════════════════════════════════════════════════════════════════════════════

def wrap_angle(a: float) -> float:
    """Wrap to [-pi, pi] (same convention as the training env)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _transform_scan_sample(ang_deg: float, dist: float,
                           cfg: NavRLConfig) -> Optional[Tuple[float, float]]:
    """Map one raw scan sample into the robot frame.

    ``scan_to_rays`` and the geometric brake used to carry out this transform
    independently.  That made a corrected yaw offset reach the policy while
    the brake still watched the old sector.  Keep the transform in one pure
    helper so both consumers (and their tests) use exactly the same convention.
    """
    try:
        ang_deg, dist = float(ang_deg), float(dist)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(ang_deg) or not math.isfinite(dist) or dist <= 0.02:
        return None
    a = wrap_angle(cfg.lidar_angle_dir * math.radians(ang_deg)
                   + math.radians(cfg.lidar_yaw_offset_deg))
    if cfg.lidar_forward_offset_m:
        x = dist * math.cos(a) + cfg.lidar_forward_offset_m
        y = dist * math.sin(a)
        dist = math.hypot(x, y)
        a = math.atan2(y, x)
    return a, dist


def scan_to_rays(points: Sequence[Tuple[float, float]], cfg: NavRLConfig) -> np.ndarray:
    """Convert raw lidar points to the training 48-ray observation.

    points: (angle_deg_in_lidar_frame, distance_m) samples of one revolution.
    Returns float32[48]: distances from ROBOT CENTER, ray 0 at -90 deg (right)
    to ray 47 at +90 deg (left), min-pooled per ray sector, clipped to
    [lidar_min_dist, lidar_max_dist]; empty sector -> lidar_max_dist (no hit),
    matching the sim rayTestBatch normalisation value = hitfrac*(4-r)+r.
    """
    n = cfg.lidar_num_rays
    rays = np.full(n, cfg.lidar_max_dist, dtype=np.float32)
    half_fov = math.radians(cfg.lidar_fov_deg) / 2.0
    step = math.radians(cfg.lidar_fov_deg) / (n - 1)
    for ang_deg, dist in points:
        sample = _transform_scan_sample(ang_deg, dist, cfg)
        if sample is None:
            continue
        a, dist = sample
        if abs(a) > half_fov + step / 2.0:
            continue                 # behind the 180 deg front window
        idx = int(round((a + half_fov) / step))
        if 0 <= idx < n:
            d = min(max(dist, cfg.lidar_min_dist), cfg.lidar_max_dist)
            if d < rays[idx]:
                rays[idx] = d
    return rays


def front_min_raw(points: Sequence[Tuple[float, float]], cfg: NavRLConfig) -> float:
    """Minimum RAW distance in the front safety cone (for the geometric brake)."""
    half = math.radians(cfg.safety_brake_halfangle_deg)
    best = float("inf")
    for ang_deg, dist in points:
        sample = _transform_scan_sample(ang_deg, dist, cfg)
        if sample is None:
            continue
        a, dist = sample
        if abs(a) <= half and dist < best:
            best = dist
    return best


def build_nav_obs(goal_dist: float, goal_bearing: float,
                  lin_speed: float, ang_vel_z: float,
                  prev_action: np.ndarray, rays: np.ndarray) -> np.ndarray:
    """Assemble the 55-D observation in the exact training order."""
    return np.concatenate([
        [goal_dist, math.sin(goal_bearing), math.cos(goal_bearing),
         lin_speed, ang_vel_z, prev_action[0], prev_action[1]],
        rays,
    ]).astype(np.float32)


def shape_action(delayed_action: np.ndarray, obs: np.ndarray,
                 cfg: NavRLConfig) -> Tuple[float, float, bool]:
    """Mirror of X3PlusNavEnv.step() actuation on the DELAYED action.

    All shaping inputs come from the observation the policy just saw (in sim,
    prev_dist/prev_angle/base_speed/min lidar at actuation time all equal the
    values inside that observation).
    Returns (vx m/s, wz rad/s LEFT-positive, stall_assist_fired).
    """
    throttle = float(np.clip(delayed_action[0], -1.0, 1.0))
    ang_raw = float(np.clip(delayed_action[1], -1.0, 1.0))
    wz = ang_raw * cfg.max_angular_vel

    dist, cos_t, lin_speed = float(obs[0]), float(obs[2]), float(obs[3])
    min_lidar = float(np.min(obs[7:7 + cfg.lidar_num_rays]))

    stall = (cfg.stall_assist_enabled
             and dist > cfg.stall_dist_threshold
             and cos_t > cfg.stall_heading_cos_threshold
             and min_lidar > cfg.stall_clearance_threshold
             and lin_speed < cfg.stall_speed_threshold
             and throttle < cfg.stall_assist_min_throttle)
    if stall:
        throttle = cfg.stall_assist_min_throttle

    if min_lidar < cfg.near_obstacle_slowdown_dist:
        ratio = max(0.0, min(1.0, min_lidar / max(cfg.near_obstacle_slowdown_dist, 1e-6)))
        throttle *= (cfg.near_obstacle_min_linear_scale
                     + (1.0 - cfg.near_obstacle_min_linear_scale) * ratio)
        wz *= (cfg.near_obstacle_min_angular_scale
               + (1.0 - cfg.near_obstacle_min_angular_scale) * ratio)

    vmax = cfg.max_linear_vel_forward if throttle >= 0 else cfg.max_linear_vel_reverse
    return throttle * vmax, wz, stall


class ActionDelay:
    """Training motor delay: execute the action commanded delay_steps ago."""

    def __init__(self, delay_steps: int):
        self.delay_steps = int(delay_steps)
        if self.delay_steps < 0:
            raise ValueError(f"delay_steps must be >= 0, got {delay_steps}")
        self._buf: deque = deque(maxlen=self.delay_steps + 1)

    def push(self, action: np.ndarray) -> np.ndarray:
        self._buf.append(np.asarray(action, dtype=np.float32).copy())
        if len(self._buf) > self.delay_steps:
            return self._buf[0]
        return np.zeros(2, dtype=np.float32)

    def reset(self):
        self._buf.clear()


class GoalTracker:
    """Goal position in the robot frame: camera fixes + dead-reckoned prediction.

    Frame: x forward, y LEFT (bearing = atan2(y, x), left positive — same as
    the sim's atan2(rel_y, rel_x) - yaw). Camera offsets are RIGHT-positive
    (vision_grasp_pipeline.estimate_offset_x), hence the sign flip in update().
    """

    def __init__(self):
        self._rel: Optional[np.ndarray] = None
        self._last_fix = 0.0

    @property
    def has_fix(self) -> bool:
        return self._rel is not None

    def fix_age(self, now: Optional[float] = None) -> float:
        return (time.time() if now is None else now) - self._last_fix

    def update(self, forward_dist: float, offset_right: float):
        self._rel = np.array([forward_dist, -offset_right], dtype=np.float64)
        self._last_fix = time.time()

    def predict(self, vx: float, wz: float, dt: float):
        """Advance the robot by (vx, wz) for dt; goal moves in the robot frame."""
        if self._rel is None:
            return
        x, y = self._rel
        x -= vx * dt                       # robot moved forward
        phi = wz * dt                      # robot yawed left by phi
        c, s = math.cos(phi), math.sin(phi)
        self._rel = np.array([c * x + s * y, -s * x + c * y], dtype=np.float64)

    def dist(self) -> float:
        return float(np.hypot(*self._rel)) if self._rel is not None else float("inf")

    def bearing(self) -> float:
        if self._rel is None:
            return 0.0
        return math.atan2(self._rel[1], self._rel[0])


# ════════════════════════════════════════════════════════════════════════════
# Policy (heavy deps — lazy)
# ════════════════════════════════════════════════════════════════════════════

class NavPolicy:
    """PPO nav policy + manual VecNormalize obs normalisation.

    Mirrors the grasp deployment: the vecnormalize pickle is loaded directly
    and only obs_rms/clip_obs are used (training=False semantics, no env).
    """

    def __init__(self, cfg: NavRLConfig):
        import pickle
        from stable_baselines3 import PPO

        validate_config(cfg)

        print(f"[nav-rl] loading policy: {cfg.model_path}")
        try:
            self.model = PPO.load(cfg.model_path, device="cpu")
        except Exception:
            # cross-version pickles: schedule lambdas may not deserialise;
            # they are irrelevant for inference.
            self.model = PPO.load(cfg.model_path, device="cpu", custom_objects={
                "lr_schedule": (lambda _: 0.0),
                "clip_range": (lambda _: 0.0),
            })
        obs_shape = tuple(self.model.observation_space.shape)
        action_shape = tuple(self.model.action_space.shape)
        if obs_shape != (55,) or action_shape != (2,):
            raise RuntimeError(
                f"nav model spaces must be obs=(55,), action=(2,), got "
                f"obs={obs_shape}, action={action_shape}"
            )

        print(f"[nav-rl] loading vecnormalize: {cfg.vecnorm_path}")
        with open(cfg.vecnorm_path, "rb") as f:
            vn = pickle.load(f)
        self._mean = np.asarray(vn.obs_rms.mean, dtype=np.float64)
        self._var = np.asarray(vn.obs_rms.var, dtype=np.float64)
        self._eps = float(vn.epsilon)
        self._clip = float(vn.clip_obs)
        if self._mean.shape != (55,):
            raise RuntimeError(f"vecnormalize stats shape {self._mean.shape} != (55,)")
        if (not np.all(np.isfinite(self._mean))
                or not np.all(np.isfinite(self._var))
                or np.any(self._var < 0.0)):
            raise RuntimeError("vecnormalize statistics contain invalid mean/variance values")

    def predict(self, obs55: np.ndarray) -> np.ndarray:
        obs55 = np.asarray(obs55, dtype=np.float32)
        if obs55.shape != (55,) or not np.all(np.isfinite(obs55)):
            raise ValueError(f"navigation observation must be finite shape (55,), got {obs55}")
        norm = np.clip((obs55 - self._mean) / np.sqrt(self._var + self._eps),
                       -self._clip, self._clip).astype(np.float32)
        action, _ = self.model.predict(norm, deterministic=True)
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape != (2,) or not np.all(np.isfinite(action)):
            raise RuntimeError(f"navigation policy returned invalid action: {action}")
        return np.clip(action, -1.0, 1.0)


# ════════════════════════════════════════════════════════════════════════════
# Lidar sources
# ════════════════════════════════════════════════════════════════════════════

def laser_scan_to_points(message: Mapping[str, Any]) -> List[Tuple[float, float]]:
    """Convert one ROS ``sensor_msgs/LaserScan`` message to finite points.

    Returned angles follow ROS REP-103: 0 deg is forward and positive angles
    are counter-clockwise/left. ``inf``/``nan`` and values outside the
    message's declared range are omitted; an all-``inf`` scan is therefore a
    valid empty point list, not a stale scan.
    """
    if not isinstance(message, Mapping):
        raise TypeError("LaserScan message must be a mapping")
    ranges = message.get("ranges")
    if (not isinstance(ranges, Sequence)
            or isinstance(ranges, (str, bytes, bytearray))):
        raise ValueError("LaserScan ranges must be a sequence")

    try:
        angle_min = float(message["angle_min"])
        angle_increment = float(message["angle_increment"])
        range_min = float(message["range_min"])
        range_max = float(message["range_max"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"LaserScan metadata is incomplete or invalid: {exc}") from exc

    if not math.isfinite(angle_min):
        raise ValueError("LaserScan angle_min must be finite")
    if not math.isfinite(angle_increment) or angle_increment == 0.0:
        raise ValueError("LaserScan angle_increment must be finite and non-zero")
    if (not math.isfinite(range_min) or range_min < 0.0
            or not math.isfinite(range_max) or range_max <= range_min):
        raise ValueError("LaserScan range limits must satisfy 0 <= min < max")

    if not ranges:
        raise ValueError("LaserScan ranges must not be empty")
    points: List[Tuple[float, float]] = []
    usable_samples = 0
    for i, raw_dist in enumerate(ranges):
        try:
            dist = float(raw_dist)
        except (TypeError, ValueError):
            continue
        # Positive infinity is the ROS convention for a valid ray with no
        # return. NaN, -inf and finite values outside the declared sensor range
        # do not establish that the sensor produced a usable sample.
        if math.isinf(dist) and dist > 0.0:
            usable_samples += 1
            continue
        if not math.isfinite(dist) or dist < range_min or dist > range_max:
            continue
        usable_samples += 1
        angle_deg = math.degrees(angle_min + i * angle_increment)
        points.append((angle_deg, dist))
    if usable_samples == 0:
        raise ValueError("LaserScan contains no usable ranges")
    return points


def describe_scan(message: Mapping[str, Any], cfg: NavRLConfig) -> dict:
    """What frame the scan is in and whether it can see where we drive.

    This exists because the 180 deg question has two conflicting records and
    getting it wrong is silent. The TG30 on this robot is mounted flipped:
    Route B's TF puts yaw=pi on ``laser_link -> laser`` and the Jetson's own
    launch says "``/scan`` frame_id is ``laser``". AMCL reads ``/scan`` through
    TF and is therefore correct either way. This module does NOT use TF -- it
    converts the message's own angles directly -- so if the message really is
    in ``laser``, every ray here is 180 deg out and "forward" is the robot's
    rear. Nothing about that reads as an error: the policy simply sees a clear
    path ahead and the geometric brake never fires.

    The coverage check below is only a metadata check.  A full 360-degree scan
    covers both the front and rear regardless of the offset, so coverage cannot
    prove the physical meaning of a raw index.  That requires the Route B
    four-direction board gate.  This function must never be treated as a
    substitute for that evidence.
    """
    header = message.get("header") if isinstance(message, Mapping) else None
    frame = ""
    if isinstance(header, Mapping):
        frame = str(header.get("frame_id") or "")

    info = {"frame_id": frame, "warnings": []}
    try:
        angle_min = math.degrees(float(message["angle_min"]))
        angle_inc = math.degrees(float(message["angle_increment"]))
        n = len(message["ranges"])
    except (KeyError, TypeError, ValueError):
        info["warnings"].append("scan metadata unreadable; cannot check coverage")
        return info

    angle_max = angle_min + angle_inc * max(n - 1, 0)
    lo, hi = min(angle_min, angle_max), max(angle_min, angle_max)
    info.update(angle_min_deg=lo, angle_max_deg=hi, span_deg=hi - lo, count=n)

    # Compute overlap on the circle, including scans whose transformed window
    # crosses -180/+180.  The previous linear min/max comparison reported a
    # false partial overlap for a valid 360-degree scan with a 180-degree offset.
    half = cfg.lidar_fov_deg / 2.0
    raw_span = abs(angle_inc) * max(n - 1, 0)
    if raw_span >= 360.0 - 1e-6:
        covered = min(360.0, cfg.lidar_fov_deg)
    else:
        # a = dir*raw + offset is affine, so the image of the raw window is
        # bounded by the images of its two endpoints.  Adding raw_span to the
        # transformed start silently assumed dir=+1: with a mirrored lidar the
        # transformed angles DECREASE with index, and coverage came out
        # inverted -- a correctly aimed +-85 deg scan reported 3% (and failed
        # --real) while the same scan pointing backwards reported 92%.
        start = math.degrees(wrap_angle(
            cfg.lidar_angle_dir * math.radians(angle_min)
            + math.radians(cfg.lidar_yaw_offset_deg)))
        delta = cfg.lidar_angle_dir * angle_inc * max(n - 1, 0)
        start, end = (start, start + delta) if delta >= 0.0 else (start + delta, start)
        covered = 0.0
        for k in range(-2, 3):
            target_lo = -half + 360.0 * k
            target_hi = half + 360.0 * k
            covered = max(covered,
                          max(0.0, min(end, target_hi)
                              - max(start, target_lo)))
    info["forward_coverage_deg"] = covered
    info["forward_fraction"] = covered / max(cfg.lidar_fov_deg, 1e-9)

    if covered <= 0.0:
        info["warnings"].append(
            f"the transformed scan window does NOT overlap the policy's "
            f"forward window ±{half:.0f} deg. Every "
            f"forward ray will read as no-hit (clear) and the geometric brake "
            f"will never fire. This is what a 180 deg mounting error looks like."
        )
    elif info["forward_fraction"] < 0.9:
        info["warnings"].append(
            f"only {info['forward_fraction']*100:.0f}% of the policy's forward "
            f"{cfg.lidar_fov_deg:.0f} deg is actually sampled; the rest defaults "
            f"to no-hit"
        )
    if frame == "laser" and abs(cfg.lidar_yaw_offset_deg) < 1e-6:
        info["warnings"].append(
            "scan frame_id is 'laser', which on this robot is rotated 180 deg "
            "from laser_link (see Route B tf_confirmed.yaml), but "
            "lidar_yaw_offset_deg is 0. Either pass --lidar-yaw-offset-deg 180 "
            "or confirm with --probe that front really is front."
        )
    return info


def load_orientation_evidence(path: str) -> dict:
    """Load the Route B orientation marker/evidence without executing it.

    The handoff marker is intentionally a tiny ``key=value`` file so it can be
    written on the Jetson without Python dependencies.  JSON evidence from the
    four-direction diagnostic is accepted as well.  This parser only checks
    identity and completeness; it never turns a marker into physical proof.
    """
    if not path:
        raise ValueError("orientation evidence path is empty")
    p = Path(path).expanduser()
    if not p.is_file() or not p.stat().st_size:
        raise ValueError(f"orientation evidence is missing or empty: {p}")
    try:
        raw = p.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        raise ValueError(f"cannot read orientation evidence {p}: {exc}") from exc
    try:
        value = json.loads(raw)
        if not isinstance(value, Mapping):
            raise ValueError("JSON evidence must be an object")
        data = dict(value)
    except (ValueError, TypeError):
        data = {}
        for line in raw.splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            data[key.strip()] = value.strip()
    data["_path"] = str(p)
    return data


def validate_orientation_evidence(path: str, cfg: NavRLConfig,
                                   scan_info: Optional[Mapping[str, Any]] = None
                                   ) -> Tuple[bool, str]:
    """Validate the evidence identity against the live scan configuration.

    A valid return means only that the operator's evidence belongs to this
    runtime configuration.  The four-direction board test remains the source
    of the physical 0/180 decision and must be performed before creating the
    marker.
    """
    try:
        data = load_orientation_evidence(path)
    except ValueError as exc:
        return False, str(exc)
    required = ("verified_at", "scan_frame", "policy_source_angle_offset_deg",
                "evidence_log")
    missing = [key for key in required if not str(data.get(key, "")).strip()]
    if missing:
        return False, "orientation evidence missing: " + ", ".join(missing)
    status = str(data.get("result", data.get("status", ""))).strip().upper()
    if not status:
        return False, "orientation evidence missing result/status"
    if status not in ("PASS", "VERIFIED", "MANUAL_PASS"):
        return False, f"orientation evidence status is {status!r}, not PASS"
    try:
        evidence_offset = float(data["policy_source_angle_offset_deg"])
    except (TypeError, ValueError):
        return False, "orientation evidence offset is not numeric"
    if not math.isfinite(evidence_offset):
        return False, "orientation evidence offset is not finite"
    # 180 and -180 are equivalent; arbitrary offsets are not accepted by the
    # Route B contract until a new physical gate is designed.
    if min(abs(evidence_offset), abs(abs(evidence_offset) - 180.0)) > 1e-3:
        return False, ("orientation evidence offset must be 0 or 180 degrees, "
                       f"got {evidence_offset}")
    if abs(((evidence_offset - cfg.lidar_yaw_offset_deg + 180.0) % 360.0)
           - 180.0) > 1e-3:
        return False, ("orientation evidence offset does not match runtime "
                       f"lidar_yaw_offset_deg={cfg.lidar_yaw_offset_deg}")
    if scan_info:
        live_frame = str(scan_info.get("frame_id") or "")
        if live_frame and live_frame != str(data["scan_frame"]):
            return False, (f"orientation evidence frame={data['scan_frame']!r} "
                           f"but live /scan frame={live_frame!r}")
    return True, "orientation evidence identity matches live configuration"


def _laser_scan_stamp_seconds(message: Mapping[str, Any]) -> Optional[float]:
    """Return the ROS1 header timestamp when present, without using it as age."""
    header = message.get("header")
    if not isinstance(header, Mapping):
        return None
    stamp = header.get("stamp")
    if not isinstance(stamp, Mapping):
        return None
    try:
        secs = float(stamp["secs"])
        nsecs = float(stamp["nsecs"])
    except (KeyError, TypeError, ValueError):
        return None
    value = secs + nsecs * 1e-9
    return value if math.isfinite(value) and value >= 0.0 else None


class FakeLidar:
    """All-clear scan — bench/dry-run use ONLY (no obstacle avoidance!)."""

    def get_points(self) -> List[Tuple[float, float]]:
        return [(a, 3.9) for a in range(0, 360, 2)]

    def age(self) -> float:
        return 0.0

    def close(self):
        pass


class RPLidarSource:
    """Slamtec RPLIDAR via the `rplidar`(-roboticia) package, background thread.

    Keeps the latest full revolution. get_points() -> [(angle_deg, dist_m)].
    """

    def __init__(self, port: str, baud: int = 115200):
        from rplidar import RPLidar  # pip install rplidar-roboticia
        self._lidar = RPLidar(port, baudrate=baud)
        self._points: List[Tuple[float, float]] = []
        self._ts = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        try:
            for scan in self._lidar.iter_scans(max_buf_meas=800):
                if self._stop.is_set():
                    break
                points = [(ang, dist / 1000.0) for _q, ang, dist in scan]
                with self._lock:
                    self._points = points
                    self._ts = time.monotonic()
        except Exception as e:
            if not self._stop.is_set():
                print(f"[nav-rl][lidar] stream error: {e}")

    def get_points(self) -> List[Tuple[float, float]]:
        with self._lock:
            return list(self._points)

    def age(self) -> float:
        with self._lock:
            ts = self._ts
        return time.monotonic() - ts if ts else float("inf")

    def close(self):
        self._stop.set()
        try:
            self._lidar.stop()
            self._lidar.stop_motor()
            self._lidar.disconnect()
        except Exception:
            pass
        self._thread.join(timeout=2.0)


class RosLaserScanSource:
    """Latest-only ROS1 ``/scan`` subscriber through rosbridge WebSocket.

    This keeps the Python 3.8 inference process independent of Melodic's
    Python 2.7 ``rospy``. A message is considered fresh when its callback is
    received locally; the navigation loop already stops on ``age()`` timeout.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 9090,
                 topic: str = "/scan", connect_timeout_s: float = 5.0):
        if not host:
            raise ValueError("ROS bridge host must not be empty")
        if not 1 <= int(port) <= 65535:
            raise ValueError(f"ROS bridge port out of range: {port}")
        if not topic.startswith("/"):
            raise ValueError(f"ROS scan topic must be absolute, got {topic!r}")

        try:
            import roslibpy
        except ImportError as exc:
            raise RuntimeError(
                "roslibpy is required for the ROS /scan backend; run "
                "'python3 -m pip install roslibpy' inside grasp_venv"
            ) from exc

        self._points: List[Tuple[float, float]] = []
        self._ts = 0.0
        self._ros_stamp: Optional[float] = None
        self._lock = threading.Lock()
        self._last_error = ""
        self._raw_sample: Optional[Mapping[str, Any]] = None
        self._client = roslibpy.Ros(host=host, port=int(port))
        self._topic = None
        try:
            self._client.run(timeout=float(connect_timeout_s))
            if not self._client.is_connected:
                raise ConnectionError("rosbridge connection did not become ready")
            self._topic = roslibpy.Topic(
                self._client, topic, "sensor_msgs/LaserScan", queue_length=1
            )
            self._topic.subscribe(self._on_scan)
        except Exception as exc:
            self.close()
            raise RuntimeError(
                f"cannot subscribe to ROS scan at ws://{host}:{port}{topic}: {exc}"
            ) from exc

    def _on_scan(self, message: Mapping[str, Any]) -> None:
        try:
            points = laser_scan_to_points(message)
        except (TypeError, ValueError) as exc:
            error = str(exc)
            if error != self._last_error:
                print(f"[nav-rl][lidar] invalid ROS LaserScan: {error}")
                self._last_error = error
            return
        with self._lock:
            self._points = points
            self._ts = time.monotonic()
            self._ros_stamp = _laser_scan_stamp_seconds(message)
            self._last_error = ""
            if self._raw_sample is None:
                # Keep one message so the frame/coverage check can run without
                # a second subscription. Ranges are dropped -- only the header
                # and the angle metadata matter, and holding 1000+ floats per
                # source for the life of the run is pointless.
                self._raw_sample = {
                    "header": dict(message.get("header") or {}),
                    "angle_min": message.get("angle_min"),
                    "angle_increment": message.get("angle_increment"),
                    "ranges": [0.0] * len(message.get("ranges") or []),
                }

    def get_points(self) -> List[Tuple[float, float]]:
        with self._lock:
            return list(self._points)

    def age(self) -> float:
        with self._lock:
            ts = self._ts
        return time.monotonic() - ts if ts else float("inf")

    def ros_stamp(self) -> Optional[float]:
        with self._lock:
            return self._ros_stamp

    def scan_info(self, cfg: NavRLConfig) -> Optional[dict]:
        """Frame and forward-coverage report for the first scan seen."""
        with self._lock:
            sample = self._raw_sample
        return None if sample is None else describe_scan(sample, cfg)

    def close(self) -> None:
        topic, self._topic = self._topic, None
        if topic is not None:
            try:
                topic.unsubscribe()
            except Exception:
                pass
        client, self._client = getattr(self, "_client", None), None
        if client is not None:
            try:
                client.terminate()
            except Exception:
                pass


def make_lidar(cfg: NavRLConfig, kind: str = "ros", *,
               ros_host: str = "127.0.0.1", ros_port: int = 9090,
               scan_topic: str = "/scan"):
    kind = kind.lower()
    if kind == "none":
        print("[nav-rl] WARNING: --no-lidar — rays forced clear, NO obstacle "
              "avoidance and NO safety brake. Bench/dry-run only.")
        return FakeLidar()
    if kind == "ros":
        return RosLaserScanSource(ros_host, ros_port, scan_topic)
    if kind == "rplidar":
        if not cfg.lidar_port:
            raise ValueError(
                "--lidar-backend rplidar requires an explicit --lidar-port; "
                "do not use it for the X3Plus YDLIDAR TG30"
            )
        return RPLidarSource(cfg.lidar_port)
    raise ValueError(f"unsupported lidar backend: {kind!r}")


# ════════════════════════════════════════════════════════════════════════════
# CLI: selftest / probe / bench
# ════════════════════════════════════════════════════════════════════════════

def _approx(a, b, tol=1e-4):
    assert abs(a - b) <= tol, f"{a} != {b} (tol {tol})"


def run_selftest():
    cfg = NavRLConfig()
    validate_config(cfg)

    print("== wrap_angle ==")
    _approx(wrap_angle(math.pi + 0.1), -math.pi + 0.1)
    _approx(wrap_angle(-3 * math.pi), -math.pi)

    print("== LaserScan conversion: finite ranges + ROS CCW angles ==")
    pts = laser_scan_to_points({
        "angle_min": -math.pi / 2,
        "angle_increment": math.pi / 2,
        "range_min": 0.01,
        "range_max": 50.0,
        "ranges": [1.0, float("inf"), 0.5],
    })
    assert len(pts) == 2
    _approx(pts[0][0], -90.0); _approx(pts[1][0], 90.0)

    print("== scan_to_rays: dead-ahead 1 m, ROS default (dir=+1) ==")
    rays = scan_to_rays([(0.0, 1.0)], cfg)
    ctr = np.argmin(rays)
    assert 22 <= ctr <= 25, f"center ray index {ctr}"
    _approx(float(rays[ctr]), 1.0)
    assert float(np.max(rays)) == cfg.lidar_max_dist

    print("== scan_to_rays: clipping ==")
    rays = scan_to_rays([(0.0, 0.10), (10.0, 9.0)], cfg)
    _approx(float(np.min(rays)), cfg.lidar_min_dist)          # 0.33 floor
    assert float(np.max(rays)) == cfg.lidar_max_dist          # 4.0 ceiling

    print("== scan_to_rays: ROS right/left mapping (dir=+1) ==")
    rays = scan_to_rays([(-85.0, 1.5)], cfg)
    assert np.argmin(rays) <= 3, f"expected right edge, got {np.argmin(rays)}"
    rays = scan_to_rays([(85.0, 1.5)], cfg)
    assert np.argmin(rays) >= 44, f"expected left edge, got {np.argmin(rays)}"

    print("== scan_to_rays: legacy direct RPLIDAR clockwise mapping (dir=-1) ==")
    direct_cfg = dataclasses.replace(cfg, lidar_angle_dir=-1.0)
    rays = scan_to_rays([(85.0, 1.5)], direct_cfg)
    assert np.argmin(rays) <= 3, f"expected right edge, got {np.argmin(rays)}"

    print("== scan_to_rays: forward mount offset ==")
    c2 = dataclasses.replace(cfg, lidar_forward_offset_m=0.10)
    rays = scan_to_rays([(0.0, 0.50)], c2)
    _approx(float(np.min(rays)), 0.60, 1e-3)

    print("== front_min_raw sees below the 0.33 ray floor ==")
    _approx(front_min_raw([(0.0, 0.18), (120.0, 0.05)], cfg), 0.18)

    print("== build_nav_obs ordering ==")
    obs = build_nav_obs(2.0, math.radians(30), 0.4, -0.5,
                        np.array([0.7, -0.2]), np.full(48, 4.0, dtype=np.float32))
    assert obs.shape == (55,)
    _approx(float(obs[0]), 2.0); _approx(float(obs[1]), math.sin(math.radians(30)))
    _approx(float(obs[3]), 0.4); _approx(float(obs[4]), -0.5)
    _approx(float(obs[5]), 0.7); _approx(float(obs[6]), -0.2)

    print("== shape_action: clear path passthrough ==")
    vx, wz, st = shape_action(np.array([1.0, 0.5]), obs, cfg)
    _approx(vx, 0.5); _approx(wz, 0.6); assert not st

    print("== shape_action: reverse scaling ==")
    vx, _, _ = shape_action(np.array([-1.0, 0.0]), obs, cfg)
    _approx(vx, -0.15)

    print("== shape_action: near-obstacle slowdown (min ray 0.33) ==")
    obs_block = obs.copy(); obs_block[7:55] = cfg.lidar_min_dist
    vx, wz, _ = shape_action(np.array([1.0, 1.0]), obs_block, cfg)
    ratio = cfg.lidar_min_dist / cfg.near_obstacle_slowdown_dist
    lin_scale = 0.24 + 0.76 * ratio
    ang_scale = 0.9 + 0.1 * ratio
    _approx(vx, 0.5 * lin_scale, 1e-3); _approx(wz, 1.2 * ang_scale, 1e-3)

    print("== shape_action: stall assist fires ==")
    obs_st = build_nav_obs(2.0, 0.0, 0.0, 0.0, np.zeros(2),
                           np.full(48, 4.0, dtype=np.float32))
    vx, _, st = shape_action(np.array([0.0, 0.0]), obs_st, cfg)
    assert st; _approx(vx, 0.28 * 0.5)

    print("== ActionDelay (2 steps) ==")
    d = ActionDelay(2)
    a = [np.array([i + 1.0, 0.0]) for i in range(3)]
    assert float(d.push(a[0])[0]) == 0.0
    assert float(d.push(a[1])[0]) == 0.0
    assert float(d.push(a[2])[0]) == 1.0   # executes the action from 2 steps ago

    print("== GoalTracker ==")
    t = GoalTracker()
    t.update(1.0, 0.0)                     # 1 m dead ahead
    t.predict(0.5, 0.0, 0.2)               # drive 10 cm
    _approx(t.dist(), 0.9)
    t2 = GoalTracker(); t2.update(1.0, 0.0)
    t2.predict(0.0, math.pi / 2, 1.0)      # rotate 90 deg LEFT in place
    _approx(t2.bearing(), -math.pi / 2, 1e-6)   # goal now to the RIGHT
    t3 = GoalTracker(); t3.update(1.0, 0.3)     # offset 0.3 RIGHT
    assert t3.bearing() < 0                 # right => negative bearing

    _selftest_describe_scan()

    print("\n[selftest] OK")


def _selftest_describe_scan():
    """The 180 deg mounting check: coverage, not frame names."""
    cfg = NavRLConfig()

    def msg(frame, lo_deg, hi_deg, n=340):
        inc = math.radians(hi_deg - lo_deg) / max(n - 1, 1)
        return {"header": {"frame_id": frame},
                "angle_min": math.radians(lo_deg),
                "angle_increment": inc,
                "ranges": [1.0] * n}

    # A correctly-oriented forward window: no complaints.
    info = describe_scan(msg("laser_link", -85.0, 85.0), cfg)
    assert info["frame_id"] == "laser_link"
    assert info["forward_fraction"] > 0.9, info
    assert not info["warnings"], info["warnings"]

    # The same 170 deg window on a sensor mounted backwards. The robot's front
    # is simply not in the data, so every forward ray reads clear and the brake
    # never fires -- the failure this whole check exists for.
    info = describe_scan(msg("laser", 95.0, 265.0), cfg)
    assert info["forward_coverage_deg"] == 0.0, info
    assert any("does NOT overlap" in w for w in info["warnings"]), info["warnings"]

    # frame 'laser' with no offset is suspicious even when coverage looks fine,
    # because on this robot that frame is the flipped one.
    info = describe_scan(msg("laser", -85.0, 85.0), cfg)
    assert any("rotated 180" in w for w in info["warnings"]), info["warnings"]
    # ...and declaring the offset clears that specific complaint
    shifted = dataclasses.replace(cfg, lidar_yaw_offset_deg=180.0)
    info = describe_scan(msg("laser", 95.0, 265.0), shifted)
    assert info["forward_fraction"] > 0.9, info
    assert not info["warnings"], info["warnings"]

    # A narrow window is reported as partial rather than silently padded.
    info = describe_scan(msg("laser_link", -40.0, 40.0), cfg)
    assert 0.4 < info["forward_fraction"] < 0.5, info
    assert any("only" in w for w in info["warnings"]), info["warnings"]

    # Unreadable metadata must not raise in the middle of a scan callback.
    info = describe_scan({"header": {"frame_id": "x"}}, cfg)
    assert info["warnings"] and "unreadable" in info["warnings"][0]
    print("== describe_scan: 180deg mounting / forward-coverage check ==")


def run_probe(cfg: NavRLConfig, *, backend: str = "ros",
              ros_host: str = "127.0.0.1", ros_port: int = 9090,
              scan_topic: str = "/scan"):
    """Print per-30deg-sector minima so mounting/orientation can be verified.

    Put your hand ~0.4 m from the robot's FRONT, then LEFT, then RIGHT, and
    check the matching sector reacts. If front/left/right are shifted, adjust
    --lidar-yaw-offset-deg; if left/right are swapped, flip --lidar-dir.
    """
    lidar = make_lidar(
        cfg, backend, ros_host=ros_host, ros_port=ros_port,
        scan_topic=scan_topic
    )
    reported = False
    try:
        while True:
            pts = lidar.get_points()
            age = lidar.age()
            if not pts and not math.isfinite(age):
                print("[probe] waiting for scan...")
                time.sleep(0.5)
                continue
            if not reported:
                info = getattr(lidar, "scan_info", lambda _c: None)(cfg)
                if info:
                    reported = True
                    print(f"[probe] scan frame_id = {info['frame_id']!r}, "
                          f"window [{info.get('angle_min_deg', float('nan')):.1f}, "
                          f"{info.get('angle_max_deg', float('nan')):.1f}] deg, "
                          f"{info.get('count', 0)} samples")
                    print(f"[probe] policy forward window is covered "
                          f"{info.get('forward_fraction', 0.0)*100:.0f}%")
                    for w in info["warnings"]:
                        print(f"[probe] WARNING: {w}")
            rays = scan_to_rays(pts, cfg)
            sect = [f"{np.min(rays[i:i + 8]):.2f}" for i in range(0, 48, 8)]
            print(f"[probe] backend={backend} age={age:.2f}s pts={len(pts):4d} "
                  f"right->left sector minima: {sect}  "
                  f"front_raw={front_min_raw(pts, cfg):.2f}m")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        lidar.close()


def run_bench(cfg: NavRLConfig):
    policy = NavPolicy(cfg)
    obs = build_nav_obs(2.0, 0.1, 0.0, 0.0, np.zeros(2),
                        np.full(48, 4.0, dtype=np.float32))
    a = policy.predict(obs)
    print(f"[bench] action for clear 2 m goal: {a}")
    n, t0 = 200, time.time()
    for _ in range(n):
        policy.predict(obs)
    hz = n / (time.time() - t0)
    print(f"[bench] predict rate: {hz:.0f} Hz (control loop needs "
          f"{1.0 / cfg.control_period_s:.1f} Hz)")


def main():
    p = argparse.ArgumentParser(description="X3Plus RL nav runtime tools")
    p.add_argument("--selftest", action="store_true")
    p.add_argument("--probe", action="store_true", help="live lidar sector monitor")
    p.add_argument("--bench", action="store_true", help="model load + predict-rate test")
    p.add_argument("--lidar-backend", choices=("ros", "rplidar", "none"),
                   default="ros", help="scan source (X3Plus TG30 default: ros)")
    p.add_argument("--lidar-port", type=str, default=None)
    p.add_argument("--lidar-yaw-offset-deg", type=float, default=None)
    p.add_argument("--lidar-dir", type=float, default=None, choices=(-1.0, 1.0))
    p.add_argument("--ros-host", type=str,
                   default=os.getenv(
                       "X3PLUS_ROS_HOST",
                       os.getenv("X3PLUS_JETSON_HOST", "127.0.0.1"),
                   ), help="rosbridge host (pipeline normally runs on Jetson)")
    p.add_argument("--ros-port", type=int, default=9090,
                   help="rosbridge WebSocket port")
    p.add_argument("--scan-topic", type=str, default="/scan")
    p.add_argument("--model", type=str, default=None)
    p.add_argument("--vecnorm", type=str, default=None)
    args = p.parse_args()

    cfg = NavRLConfig()
    if args.lidar_port: cfg.lidar_port = args.lidar_port
    if args.lidar_yaw_offset_deg is not None: cfg.lidar_yaw_offset_deg = args.lidar_yaw_offset_deg
    if args.lidar_dir is not None:
        cfg.lidar_angle_dir = args.lidar_dir
    elif args.lidar_backend == "rplidar":
        cfg.lidar_angle_dir = -1.0
    if args.model: cfg.model_path = args.model
    if args.vecnorm: cfg.vecnorm_path = args.vecnorm

    validate_config(cfg)
    if not 1 <= args.ros_port <= 65535:
        p.error("--ros-port must be in 1..65535")
    if not args.scan_topic.startswith("/"):
        p.error("--scan-topic must be an absolute ROS topic")
    if args.probe and args.lidar_backend == "none":
        p.error("--probe requires --lidar-backend ros or rplidar")
    if args.lidar_backend == "rplidar" and not args.lidar_port:
        p.error("--lidar-backend rplidar requires an explicit --lidar-port")

    if args.selftest:
        run_selftest()
    elif args.probe:
        run_probe(cfg, backend=args.lidar_backend, ros_host=args.ros_host,
                  ros_port=args.ros_port, scan_topic=args.scan_topic)
    elif args.bench:
        run_bench(cfg)
    else:
        p.print_help()


if __name__ == "__main__":
    main()
