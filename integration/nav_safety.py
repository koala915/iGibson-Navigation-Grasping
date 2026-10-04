#!/usr/bin/env python3
"""Pure post-policy safety for X3Plus navigation commands.

The PPO observation remains the trained 48-ray/55-D contract.  This module
only inspects dense raw scan points after policy inference and may reduce or
override the command that is about to reach the chassis.
"""
from __future__ import annotations

import dataclasses
import math
from typing import Optional, Sequence, Tuple


@dataclasses.dataclass
class SafetyConfig:
    enabled: bool = False
    front_half_angle_deg: float = 22.0
    front_percentile: float = 5.0
    front_min_points: int = 8
    hard_stop_enter_m: float = 0.18
    hard_stop_exit_m: float = 0.22
    forward_block_enter_m: float = 0.30
    forward_block_exit_m: float = 0.36
    slow_start_m: float = 0.55
    turn_hold_enter_m: float = 0.45
    turn_hold_exit_m: float = 0.55
    turn_min_abs_wz: float = 0.12
    turn_switch_confirm_ticks: int = 2
    side_max_angle_deg: float = 125.0
    side_front_exclude_deg: float = 25.0
    side_percentile: float = 5.0
    side_min_points: int = 8
    side_guard_enter_m: float = 0.20
    side_guard_exit_m: float = 0.24
    side_vx_slow_enter_m: float = 0.16
    side_vx_cap_mps: float = 0.06
    recovery_enabled: bool = False
    recovery_wait_s: float = 0.30
    recovery_reverse_vx: float = -0.08
    recovery_target_m: float = 0.06
    recovery_max_s: float = 1.50
    recovery_settle_s: float = 0.20
    angular_max_delta_per_step: float = 0.0

    def validate(self) -> None:
        if not 0.0 < self.hard_stop_enter_m < self.hard_stop_exit_m:
            raise ValueError("hard-stop thresholds must satisfy 0 < enter < exit")
        if not self.hard_stop_exit_m < self.forward_block_enter_m:
            raise ValueError("forward block must start beyond the hard-stop release")
        if not self.forward_block_enter_m < self.forward_block_exit_m < self.slow_start_m:
            raise ValueError("forward thresholds must satisfy enter < exit < slow_start")
        if not 0.0 < self.side_guard_enter_m < self.side_guard_exit_m:
            raise ValueError("side thresholds must satisfy 0 < enter < exit")
        if self.turn_switch_confirm_ticks < 1:
            raise ValueError("turn_switch_confirm_ticks must be >= 1")
        if self.recovery_reverse_vx >= 0.0:
            raise ValueError("recovery_reverse_vx must be negative")
        if self.angular_max_delta_per_step < 0.0:
            raise ValueError("angular_max_delta_per_step must be >= 0")


@dataclasses.dataclass(frozen=True)
class SectorMetrics:
    robust: float = float("inf")
    minimum: float = float("inf")
    count: int = 0


@dataclasses.dataclass(frozen=True)
class SafetyDecision:
    vx: float
    wz: float
    braked: bool
    reason: str
    front: SectorMetrics
    right: SectorMetrics
    left: SectorMetrics


class AngularActionRateLimiter:
    """Optional limiter for the executed PPO angular action only."""

    def __init__(self, max_delta: float = 0.0):
        self.max_delta = float(max_delta)
        self.last = 0.0

    @property
    def enabled(self) -> bool:
        return self.max_delta > 0.0

    def reset(self, value: float = 0.0) -> None:
        self.last = max(-1.0, min(1.0, float(value)))

    def apply(self, action):
        import numpy as np

        out = np.asarray(action, dtype=np.float32).reshape(-1).copy()
        if out.shape != (2,):
            raise ValueError("navigation action must have shape (2,)")
        if not self.enabled:
            self.last = float(out[1])
            return out
        delta = max(-self.max_delta,
                    min(self.max_delta, float(out[1]) - self.last))
        self.last = max(-1.0, min(1.0, self.last + delta))
        out[1] = self.last
        return out


def _percentile(values, percentile: float) -> float:
    if not values:
        return float("inf")
    ordered = sorted(float(v) for v in values)
    if len(ordered) == 1:
        return ordered[0]
    rank = max(0.0, min(100.0, float(percentile))) * (len(ordered) - 1) / 100.0
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return ordered[lo]
    frac = rank - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


def _robot_samples(points, nav_cfg):
    """Yield (robot_angle_deg, distance) using nav_rl's canonical transform."""
    try:
        from . import nav_rl
    except ImportError:  # direct script execution
        import nav_rl

    for angle_deg, distance in points:
        sample = nav_rl._transform_scan_sample(angle_deg, distance, nav_cfg)
        if sample is None:
            continue
        angle_rad, distance = sample
        yield math.degrees(angle_rad), float(distance)


def scan_metrics(points: Sequence[Tuple[float, float]], nav_cfg,
                 cfg: SafetyConfig) -> Tuple[SectorMetrics, SectorMetrics, SectorMetrics]:
    front_values = []
    right_values = []
    left_values = []
    for angle_deg, distance in _robot_samples(points, nav_cfg):
        absolute = abs(angle_deg)
        if absolute <= cfg.front_half_angle_deg:
            front_values.append(distance)
        elif cfg.side_front_exclude_deg < absolute <= cfg.side_max_angle_deg:
            (left_values if angle_deg > 0.0 else right_values).append(distance)

    def summarize(values, percentile, min_points):
        if not values:
            return SectorMetrics()
        minimum = min(values)
        robust = (_percentile(values, percentile)
                  if len(values) >= min_points else minimum)
        return SectorMetrics(robust, minimum, len(values))

    return (
        summarize(front_values, cfg.front_percentile, cfg.front_min_points),
        summarize(right_values, cfg.side_percentile, cfg.side_min_points),
        summarize(left_values, cfg.side_percentile, cfg.side_min_points),
    )


class NavigationSafety:
    """Stateful hysteresis and one-shot recovery around raw LiDAR sectors."""

    def __init__(self, cfg: Optional[SafetyConfig] = None):
        self.cfg = cfg or SafetyConfig()
        self.cfg.validate()
        self.angular_limiter = AngularActionRateLimiter(
            self.cfg.angular_max_delta_per_step)
        self.reset()

    def reset(self) -> None:
        self.hard_stopped = False
        self.forward_blocked = False
        self.right_guard = False
        self.left_guard = False
        self.turn_sign = 0
        self.pending_turn_sign = 0
        self.pending_turn_ticks = 0
        self.hard_stop_since: Optional[float] = None
        self.recovery_active = False
        self.recovery_used = False
        self.recovery_started_at = 0.0
        self.recovery_last_time = 0.0
        self.recovery_travel_m = 0.0
        self.recovery_settle_until = 0.0
        self.angular_limiter.reset()

    def limit_action(self, action):
        return self.angular_limiter.apply(action)

    @staticmethod
    def _latch(active: bool, value: float, enter: float, exit_: float) -> bool:
        if active:
            return not (math.isfinite(value) and value >= exit_)
        return math.isfinite(value) and value < enter

    def _stable_turn(self, wz: float, front: float) -> Tuple[float, str]:
        cfg = self.cfg
        if not math.isfinite(front) or front >= cfg.turn_hold_exit_m:
            self.turn_sign = self.pending_turn_sign = self.pending_turn_ticks = 0
            return wz, ""
        if abs(wz) < cfg.turn_min_abs_wz or front >= cfg.turn_hold_enter_m:
            return wz, ""
        requested = 1 if wz > 0.0 else -1
        if self.turn_sign == 0:
            self.turn_sign = requested
            return wz, ""
        if requested == self.turn_sign:
            self.pending_turn_sign = self.pending_turn_ticks = 0
            return wz, ""
        if self.pending_turn_sign != requested:
            self.pending_turn_sign = requested
            self.pending_turn_ticks = 1
        else:
            self.pending_turn_ticks += 1
        if self.pending_turn_ticks >= cfg.turn_switch_confirm_ticks:
            self.turn_sign = requested
            self.pending_turn_sign = self.pending_turn_ticks = 0
            return wz, "TURN_SWITCH_CONFIRMED"
        return math.copysign(abs(wz), self.turn_sign), "TURN_SIGN_HOLD"

    def apply(self, desired_vx: float, desired_wz: float,
              points: Sequence[Tuple[float, float]], nav_cfg,
              *, odom_vx: float = 0.0, now: Optional[float] = None) -> SafetyDecision:
        now = float(now if now is not None else 0.0)
        cfg = self.cfg
        front, right, left = scan_metrics(points, nav_cfg, cfg)
        if not cfg.enabled:
            try:
                from . import nav_rl
            except ImportError:  # direct script execution
                import nav_rl
            legacy_front = nav_rl.front_min_raw(points, nav_cfg)
            braked = desired_vx > 0.0 and legacy_front < nav_cfg.safety_brake_dist
            front = SectorMetrics(front.robust, legacy_front, front.count)
            return SafetyDecision(0.0 if braked else float(desired_vx),
                                  float(desired_wz), braked,
                                  "LEGACY_FRONT_BRAKE" if braked else "OK",
                                  front, right, left)

        self.hard_stopped = self._latch(
            self.hard_stopped, front.minimum,
            cfg.hard_stop_enter_m, cfg.hard_stop_exit_m)
        self.forward_blocked = self._latch(
            self.forward_blocked, front.robust,
            cfg.forward_block_enter_m, cfg.forward_block_exit_m)
        self.right_guard = self._latch(
            self.right_guard, right.robust,
            cfg.side_guard_enter_m, cfg.side_guard_exit_m)
        self.left_guard = self._latch(
            self.left_guard, left.robust,
            cfg.side_guard_enter_m, cfg.side_guard_exit_m)

        if self.recovery_active:
            dt = max(0.0, min(0.25, now - self.recovery_last_time))
            self.recovery_last_time = now
            if math.isfinite(float(odom_vx)):
                self.recovery_travel_m += max(0.0, -float(odom_vx)) * dt
            elapsed = now - self.recovery_started_at
            if (self.recovery_travel_m >= cfg.recovery_target_m
                    or elapsed >= cfg.recovery_max_s):
                self.recovery_active = False
                self.recovery_settle_until = now + cfg.recovery_settle_s
                return SafetyDecision(0.0, 0.0, True, "HARD_RECOVERY_DONE",
                                      front, right, left)
            return SafetyDecision(cfg.recovery_reverse_vx, 0.0, True,
                                  "HARD_RECOVERY_BACKUP", front, right, left)

        if now < self.recovery_settle_until:
            return SafetyDecision(0.0, 0.0, True, "HARD_RECOVERY_SETTLE",
                                  front, right, left)

        if not self.hard_stopped:
            self.hard_stop_since = None
            self.recovery_used = False
        if self.hard_stopped:
            if self.hard_stop_since is None:
                self.hard_stop_since = now
            elapsed = now - self.hard_stop_since
            if (cfg.recovery_enabled and not self.recovery_used
                    and elapsed >= cfg.recovery_wait_s):
                self.recovery_active = True
                self.recovery_used = True
                self.recovery_started_at = self.recovery_last_time = now
                self.recovery_travel_m = 0.0
                self.angular_limiter.reset()
                return SafetyDecision(cfg.recovery_reverse_vx, 0.0, True,
                                      "HARD_RECOVERY_START", front, right, left)
            return SafetyDecision(0.0, 0.0, True, "HARD_STOP_HOLD",
                                  front, right, left)

        vx, wz = float(desired_vx), float(desired_wz)
        reasons = []
        wz, turn_reason = self._stable_turn(wz, front.robust)
        if turn_reason:
            reasons.append(turn_reason)
        if self.right_guard and wz < 0.0:
            wz = 0.0
            reasons.append("SIDE_GUARD_R")
        if self.left_guard and wz > 0.0:
            wz = 0.0
            reasons.append("SIDE_GUARD_L")
        side_nearest = min(right.robust, left.robust)
        if vx > cfg.side_vx_cap_mps and side_nearest < cfg.side_vx_slow_enter_m:
            vx = cfg.side_vx_cap_mps
            reasons.append("SIDE_VX_CAP")
        if vx > 0.0 and front.robust < cfg.slow_start_m:
            scale = ((front.robust - cfg.forward_block_enter_m)
                     / (cfg.slow_start_m - cfg.forward_block_enter_m))
            vx *= max(0.0, min(1.0, scale))
            reasons.append("FRONT_SLOW")
        if self.forward_blocked and vx > 0.0:
            vx = 0.0
            reasons.append("FWD_BLOCK_HOLD")
        return SafetyDecision(vx, wz, vx != desired_vx,
                              " | ".join(reasons) or "OK", front, right, left)
