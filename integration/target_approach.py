#!/usr/bin/env python3
"""Pure close-range rear-camera alignment used inside mission APPROACH."""
from __future__ import annotations

import dataclasses
import math
from typing import Optional


@dataclasses.dataclass
class ApproachConfig:
    enabled: bool = False
    frame_width: int = 640
    frame_height: int = 480
    bottom_trigger_ratio: float = 0.78
    center_tolerance_px: float = 30.0
    center_kp: float = 0.0045
    center_min_wz: float = 0.50
    center_max_wz: float = 0.78
    center_max_jump_px: float = 180.0
    center_confirm_frames: int = 2
    center_timeout_s: float = 8.0
    final_vx: float = 0.08
    final_travel_m: float = 0.15
    final_timeout_s: float = 3.5

    def validate(self) -> None:
        if self.frame_width <= 0 or self.frame_height <= 0:
            raise ValueError("frame dimensions must be positive")
        if not 0.0 < self.bottom_trigger_ratio <= 1.0:
            raise ValueError("bottom_trigger_ratio must be in (0, 1]")
        if self.center_confirm_frames < 1:
            raise ValueError("center_confirm_frames must be >= 1")
        if self.final_vx <= 0.0 or self.final_travel_m <= 0.0:
            raise ValueError("final approach speed and travel must be positive")


@dataclasses.dataclass(frozen=True)
class ApproachCommand:
    active: bool
    vx: float = 0.0
    wz: float = 0.0
    done: bool = False
    failed: bool = False
    reason: str = ""


class TargetApproachController:
    IDLE = "IDLE"
    CENTER = "CENTER"
    FINAL = "FINAL"
    DONE = "DONE"
    FAILED = "FAILED"

    def __init__(self, cfg: Optional[ApproachConfig] = None):
        self.cfg = cfg or ApproachConfig()
        self.cfg.validate()
        self.reset()

    def reset(self) -> None:
        self.phase = self.IDLE
        self.started_at = 0.0
        self.last_target_stamp = 0.0
        self.last_bbox_u: Optional[float] = None
        self.last_wz = 0.0
        self.confirmed = 0
        self.last_update_at = 0.0
        self.travelled_m = 0.0

    @property
    def ready(self) -> bool:
        return not self.cfg.enabled or self.phase == self.DONE

    @property
    def failed(self) -> bool:
        return self.phase == self.FAILED

    def _pixel(self, target):
        bbox = getattr(target, "bbox_xyxy", None) if target is not None else None
        if not getattr(target, "valid", False) or bbox is None or len(bbox) != 4:
            return None
        if getattr(target, "on_floor", None) is False:
            return None
        x1, _y1, x2, y2 = [float(v) for v in bbox]
        values = (x1, x2, y2)
        if not all(math.isfinite(v) for v in values):
            return None
        return 0.5 * (x1 + x2), y2

    def _turn(self, error_px: float) -> float:
        cfg = self.cfg
        if abs(error_px) <= cfg.center_tolerance_px:
            return 0.0
        raw = -cfg.center_kp * error_px
        magnitude = max(cfg.center_min_wz, min(cfg.center_max_wz, abs(raw)))
        return math.copysign(magnitude, raw)

    def step(self, target, odom_vx: float, now: float) -> Optional[ApproachCommand]:
        if not self.cfg.enabled:
            return None
        now = float(now)
        pixel = self._pixel(target)

        if self.phase == self.IDLE:
            if pixel is None or pixel[1] < self.cfg.frame_height * self.cfg.bottom_trigger_ratio:
                return None
            self.phase = self.CENTER
            self.started_at = now
            self.last_update_at = now

        if self.phase == self.CENTER:
            if now - self.started_at >= self.cfg.center_timeout_s:
                self.phase = self.FAILED
                return ApproachCommand(True, failed=True, reason="CENTER_TIMEOUT")
            if pixel is None:
                self.last_wz = 0.0
                return ApproachCommand(True, reason="CENTER_WAIT_TARGET")

            stamp = float(getattr(target, "stamp_unix", 0.0))
            if stamp <= self.last_target_stamp + 1e-9:
                return ApproachCommand(True, wz=self.last_wz, reason="CENTER_HOLD")
            self.last_target_stamp = stamp
            u, _bottom = pixel
            if self.last_bbox_u is not None and abs(u - self.last_bbox_u) > self.cfg.center_max_jump_px:
                self.last_wz = 0.0
                self.confirmed = 0
                return ApproachCommand(True, reason="CENTER_REJECT_JUMP")
            self.last_bbox_u = u
            error = u - self.cfg.frame_width * 0.5
            if abs(error) <= self.cfg.center_tolerance_px:
                self.last_wz = 0.0
                self.confirmed += 1
                if self.confirmed >= self.cfg.center_confirm_frames:
                    self.phase = self.FINAL
                    self.started_at = self.last_update_at = now
                    self.travelled_m = 0.0
                    return ApproachCommand(True, reason="CENTER_DONE")
                return ApproachCommand(True, reason="CENTER_CONFIRM")
            self.confirmed = 0
            self.last_wz = self._turn(error)
            return ApproachCommand(True, wz=self.last_wz, reason="CENTER_TRACK")

        if self.phase == self.FINAL:
            dt = max(0.0, min(0.5, now - self.last_update_at))
            self.last_update_at = now
            if math.isfinite(float(odom_vx)):
                self.travelled_m += max(0.0, float(odom_vx)) * dt
            if self.travelled_m >= self.cfg.final_travel_m:
                self.phase = self.DONE
                return ApproachCommand(True, done=True, reason="FINAL_DONE")
            if now - self.started_at >= self.cfg.final_timeout_s:
                self.phase = self.FAILED
                return ApproachCommand(True, failed=True, reason="FINAL_TIMEOUT")
            return ApproachCommand(True, vx=self.cfg.final_vx, reason="FINAL_FORWARD")

        if self.phase == self.DONE:
            return ApproachCommand(True, done=True, reason="FINAL_DONE")
        return ApproachCommand(True, failed=True, reason="APPROACH_FAILED")
