#!/usr/bin/env python3
"""Wheel-feedback odometry: Rosmaster get_motion_data() -> pose in the odom frame.

AMCL cannot localise without odometry, and the patrol states cannot run without
AMCL. This is the missing publisher: the Python 3.8 process that already owns
/dev/myserial (GraspController.servo.device) reads the board's own velocity
feedback and integrates it.

Ported from Route A's MotionFeedbackOdom. The angular constants below remain
Route A's third and final generation.  The linear constant was re-measured on
this robot on 2026-09-22 with motor effort 30: scale 0.65 reported 14.6 cm for
about 22 cm of ruler travel, while scale 0.98 reported 20.9 cm for about
21--22 cm.  Do not reuse the abandoned command-based odometry: this integrates
what the board REPORTS, not what we asked for.

Route A's measured residuals with these constants:
    50 cm square, closed loop   ->  7.83 cm position, 2.92 deg heading
    90 deg left  (effort 30)    ->  +96.06 deg
    90 deg right (effort 30)    ->  -94.68 deg
    200 static reads            ->  zero drift

Two things this module refuses to do, both deliberate:

* It never integrates a failed or stale read. A dropped serial frame must not
  become "the robot did not move" -- that is indistinguishable from a real stop
  and would silently corrupt AMCL. Failures mark the pose invalid instead.
* It never reports `stationary` on invalid feedback, because the arm's
  stationary gate depends on it (SETMOTOR_ODOM_INTEGRATION.md section 6).

Run:
    python3 integration/feedback_odom.py --selftest
"""
from __future__ import annotations

import argparse
import dataclasses
import math
import time
from typing import Optional, Tuple

# ════════════════════════════════════════════════════════════════════════════
# Config
# ════════════════════════════════════════════════════════════════════════════

@dataclasses.dataclass
class FeedbackOdomConfig:
    # Linear scale re-measured on 2026-09-22 at the normal motor effort 30.
    # Angular scales are Route A generation 3 (route_a_parameters.yaml) and
    # still require a fresh turn calibration in the current deployment.
    linear_scale: float = 0.98
    angular_left_scale: float = 0.501
    angular_right_scale: float = 0.501

    # A read older than this cannot be integrated (route_a_parameters.yaml).
    feedback_timeout_s: float = 0.30

    # Largest plausible step. The board reports m/s; a dt this long means the
    # loop stalled and integrating it would teleport the pose.
    max_dt_s: float = 0.5

    # Reject physically impossible feedback rather than integrating garbage.
    max_speed_mps: float = 1.5
    max_yaw_rate_radps: float = 4.0

    # ── stationary gate (SETMOTOR_ODOM_INTEGRATION.md section 6) ──
    # Starting values. Re-measure the static feedback noise on the real robot
    # and tighten before trusting the arm gate.
    stationary_vx_mps: float = 0.02
    stationary_vy_mps: float = 0.02
    stationary_wz_radps: float = 0.05
    stationary_streak: int = 3

    # ── covariance published with /odom_setmotor ──
    # Wheel odometry on mecanum wheels slips; these are honest-but-loose values
    # so AMCL weights the scan more than the odometry. The Route A square-loop
    # residual (7.8 cm over ~2 m) is the empirical basis.
    var_xy: float = 0.05 ** 2
    var_yaw: float = math.radians(5.0) ** 2


def validate_config(cfg: FeedbackOdomConfig) -> None:
    positive = {
        "linear_scale": cfg.linear_scale,
        "angular_left_scale": cfg.angular_left_scale,
        "angular_right_scale": cfg.angular_right_scale,
        "feedback_timeout_s": cfg.feedback_timeout_s,
        "max_dt_s": cfg.max_dt_s,
        "max_speed_mps": cfg.max_speed_mps,
        "max_yaw_rate_radps": cfg.max_yaw_rate_radps,
        "var_xy": cfg.var_xy,
        "var_yaw": cfg.var_yaw,
    }
    for name, value in positive.items():
        if not math.isfinite(float(value)) or float(value) <= 0.0:
            raise ValueError(f"{name} must be finite and > 0, got {value}")
    if cfg.stationary_streak < 1:
        raise ValueError(f"stationary_streak must be >= 1, got {cfg.stationary_streak}")
    for name in ("stationary_vx_mps", "stationary_vy_mps", "stationary_wz_radps"):
        v = float(getattr(cfg, name))
        if not math.isfinite(v) or v <= 0.0:
            raise ValueError(f"{name} must be finite and > 0, got {v}")


# ════════════════════════════════════════════════════════════════════════════
# State
# ════════════════════════════════════════════════════════════════════════════

@dataclasses.dataclass(frozen=True)
class OdomState:
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float
    stamp: float
    valid: bool
    fresh: bool
    stationary: bool
    reason: str = ""

    def pose(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.yaw


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ════════════════════════════════════════════════════════════════════════════
# Integrator
# ════════════════════════════════════════════════════════════════════════════

class MotionFeedbackOdom:
    """Integrate body-frame velocity feedback into an odom-frame pose.

    Feed it whatever ``Rosmaster.get_motion_data()`` returns -- ``(vx, vy, vz)``
    where vz is the yaw rate, all in the body frame. Scales are applied here,
    not on the board.
    """

    def __init__(self, cfg: Optional[FeedbackOdomConfig] = None):
        self.cfg = cfg or FeedbackOdomConfig()
        validate_config(self.cfg)
        self._x = self._y = self._yaw = 0.0
        self._vx = self._vy = self._wz = 0.0
        self._stamp = 0.0
        self._valid = False
        self._reason = "no feedback yet"
        self._still_streak = 0
        self.success_count = 0
        self.failure_count = 0

    # ── inputs ──
    def update(self, vx_raw: float, vy_raw: float, wz_raw: float,
               dt: float, stamp: Optional[float] = None) -> OdomState:
        """Integrate one successful feedback read."""
        cfg = self.cfg
        stamp = time.time() if stamp is None else float(stamp)

        if not all(math.isfinite(float(v)) for v in (vx_raw, vy_raw, wz_raw, dt)):
            return self.record_failure("non-finite feedback", stamp)
        if dt <= 0.0:
            # Duplicate or out-of-order sample: keep the pose, do not integrate.
            return self._state(stamp, note="non-positive dt (sample skipped)")
        if dt > cfg.max_dt_s:
            return self.record_failure(
                f"control loop stalled ({dt:.3f}s > {cfg.max_dt_s:.3f}s)", stamp)

        vx = float(vx_raw) * cfg.linear_scale
        vy = float(vy_raw) * cfg.linear_scale
        # Left and right turns have separate scales because the mecanum base is
        # not symmetric in practice; Route A measured both. They are equal today
        # (0.501) but the split must survive, so a future re-calibration of one
        # direction does not silently change the other.
        wz_scale = cfg.angular_left_scale if wz_raw >= 0.0 else cfg.angular_right_scale
        wz = float(wz_raw) * wz_scale

        if abs(vx) > cfg.max_speed_mps or abs(vy) > cfg.max_speed_mps:
            return self.record_failure(
                f"implausible linear feedback ({vx:.2f}, {vy:.2f}) m/s", stamp)
        if abs(wz) > cfg.max_yaw_rate_radps:
            return self.record_failure(
                f"implausible yaw feedback ({wz:.2f}) rad/s", stamp)

        # Exact arc integration over the step: the body twist is held constant
        # for dt, so the pose follows a circular arc, not a straight line.
        # Midpoint yaw is used for the (small) mecanum lateral term.
        yaw0 = self._yaw
        dyaw = wz * dt
        if abs(dyaw) < 1e-9:
            dx_body, dy_body = vx * dt, vy * dt
        else:
            s, c = math.sin(dyaw), math.cos(dyaw)
            dx_body = (vx * s + vy * (c - 1.0)) / wz
            dy_body = (vx * (1.0 - c) + vy * s) / wz

        cy, sy = math.cos(yaw0), math.sin(yaw0)
        self._x += cy * dx_body - sy * dy_body
        self._y += sy * dx_body + cy * dy_body
        self._yaw = wrap_angle(yaw0 + dyaw)
        self._vx, self._vy, self._wz = vx, vy, wz
        self._stamp = stamp
        self._valid = True
        self._reason = ""
        self.success_count += 1

        if (abs(vx) < cfg.stationary_vx_mps and abs(vy) < cfg.stationary_vy_mps
                and abs(wz) < cfg.stationary_wz_radps):
            self._still_streak += 1
        else:
            self._still_streak = 0
        return self._state(stamp)

    def record_failure(self, error: str, stamp: Optional[float] = None) -> OdomState:
        """A read failed. Freeze the pose, invalidate it, and forbid `stationary`.

        Never integrate zeros here: "the serial read failed" and "the robot is
        holding still" produce identical wheel data but must not produce
        identical odometry.
        """
        self._valid = False
        self._reason = str(error)
        self._still_streak = 0
        self._vx = self._vy = self._wz = 0.0
        self.failure_count += 1
        return self._state(time.time() if stamp is None else float(stamp))

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        """Re-origin the odom frame. Only legal while stopped and before AMCL
        has begun consuming this pose (SETMOTOR_ODOM_INTEGRATION.md 4.3)."""
        if not all(math.isfinite(float(v)) for v in (x, y, yaw)):
            raise ValueError(f"odom reset needs finite values, got ({x}, {y}, {yaw})")
        self._x, self._y, self._yaw = float(x), float(y), wrap_angle(float(yaw))
        self._vx = self._vy = self._wz = 0.0
        self._still_streak = 0

    # ── outputs ──
    def age(self, now: Optional[float] = None) -> float:
        if not self._stamp:
            return float("inf")
        return (time.time() if now is None else now) - self._stamp

    def is_fresh(self, now: Optional[float] = None) -> bool:
        return self.age(now) <= self.cfg.feedback_timeout_s

    def is_stationary(self, now: Optional[float] = None) -> bool:
        return (self._valid and self.is_fresh(now)
                and self._still_streak >= self.cfg.stationary_streak)

    def state(self, now: Optional[float] = None) -> OdomState:
        return self._state(time.time() if now is None else now, refresh_stamp=False)

    def _state(self, now: float, *, refresh_stamp: bool = True,
               note: str = "") -> OdomState:
        fresh = self.is_fresh(now)
        return OdomState(
            x=self._x, y=self._y, yaw=self._yaw,
            vx=self._vx, vy=self._vy, wz=self._wz,
            stamp=self._stamp,
            valid=self._valid,
            fresh=fresh,
            stationary=(self._valid and fresh
                        and self._still_streak >= self.cfg.stationary_streak),
            reason=note or self._reason,
        )

    def covariance(self) -> list:
        """Row-major 6x6 for nav_msgs/Odometry (x y z roll pitch yaw)."""
        cov = [0.0] * 36
        big = 1e6                      # planar robot: z/roll/pitch are meaningless
        cov[0] = cov[7] = self.cfg.var_xy
        cov[14] = cov[21] = cov[28] = big
        cov[35] = self.cfg.var_yaw
        return cov


# ════════════════════════════════════════════════════════════════════════════
# Driver-facing reader
# ════════════════════════════════════════════════════════════════════════════

class FeedbackOdomReader:
    """Polls the Rosmaster handle and keeps a MotionFeedbackOdom up to date.

    The handle is the SAME object GraspController.servo.device holds -- one
    process, one serial owner. Reads go through the caller's serial lock when
    one is supplied, because servo writes share the bus.
    """

    def __init__(self, device, cfg: Optional[FeedbackOdomConfig] = None,
                 serial_lock=None):
        self.device = device
        self.odom = MotionFeedbackOdom(cfg)
        self.serial_lock = serial_lock
        self._last_poll: Optional[float] = None

    def poll(self, now: Optional[float] = None) -> OdomState:
        now = time.time() if now is None else float(now)
        dt = 0.0 if self._last_poll is None else now - self._last_poll
        self._last_poll = now

        if self.device is None:
            return self.odom.record_failure("no Rosmaster device", now)
        try:
            if self.serial_lock is not None:
                with self.serial_lock:
                    raw = self.device.get_motion_data()
            else:
                raw = self.device.get_motion_data()
        except Exception as exc:
            return self.odom.record_failure(f"get_motion_data failed: {exc}", now)

        try:
            vx, vy, wz = (float(raw[0]), float(raw[1]), float(raw[2]))
        except (TypeError, IndexError, ValueError) as exc:
            return self.odom.record_failure(f"malformed feedback {raw!r}: {exc}", now)

        if dt <= 0.0:
            # First poll: establish the clock without integrating a bogus step.
            return self.odom._state(now, note="first sample (no integration)")
        return self.odom.update(vx, vy, wz, dt, stamp=now)


# ════════════════════════════════════════════════════════════════════════════
# Selftest
# ════════════════════════════════════════════════════════════════════════════

def run_selftest() -> None:
    def approx(a, b, tol=1e-6):
        assert abs(a - b) <= tol, f"{a} != {b} (tol {tol})"

    cfg = FeedbackOdomConfig()

    # ── straight line: scale must be applied ──
    o = MotionFeedbackOdom(cfg)
    t = 100.0
    for _ in range(10):
        t += 0.05
        o.update(1.0, 0.0, 0.0, 0.05, stamp=t)      # board says 1.0 m/s
    st = o.state(now=t)
    approx(st.x, cfg.linear_scale * 1.0 * 0.5, 1e-9)
    approx(st.y, 0.0, 1e-12)
    approx(st.yaw, 0.0, 1e-12)
    assert st.valid and st.fresh and not st.stationary
    print("[odom] linear scale + integration OK")

    # ── pure rotation: left/right scales, both directions ──
    o = MotionFeedbackOdom(cfg); t = 200.0
    for _ in range(10):
        t += 0.05
        o.update(0.0, 0.0, 1.0, 0.05, stamp=t)
    approx(o.state(now=t).yaw, 0.501 * 0.5, 1e-9)
    o = MotionFeedbackOdom(cfg); t = 300.0
    for _ in range(10):
        t += 0.05
        o.update(0.0, 0.0, -1.0, 0.05, stamp=t)
    approx(o.state(now=t).yaw, -0.501 * 0.5, 1e-9)
    print("[odom] angular scales (both directions) OK")

    # ── arc integration: quarter circle must land on the circle, not the chord ──
    # Effective v/w use the configured linear/angular calibration scales.
    o = MotionFeedbackOdom(cfg); t = 400.0
    v_raw, w_raw = 1.0, 1.0
    v, w = cfg.linear_scale, cfg.angular_left_scale
    R = v / w
    quarter = (math.pi / 2.0) / w
    steps = 2000
    dt = quarter / steps
    for _ in range(steps):
        t += dt
        o.update(v_raw, 0.0, w_raw, dt, stamp=t)
    st = o.state(now=t)
    approx(st.x, R, 1e-3)               # after 90 deg: (R, R) on the circle
    approx(st.y, R, 1e-3)
    approx(st.yaw, math.pi / 2.0, 1e-3)
    chord = math.hypot(R, R)
    assert abs(chord - R * math.sqrt(2)) < 1e-9
    print(f"[odom] exact-arc integration OK (R={R:.4f} m)")

    # ── a failed read must NOT look like standing still ──
    o = MotionFeedbackOdom(cfg); t = 500.0
    for _ in range(5):
        t += 0.05
        o.update(0.0, 0.0, 0.0, 0.05, stamp=t)
    assert o.is_stationary(now=t), "5 zero-velocity reads should be stationary"
    x_before = o.state(now=t).x
    st = o.record_failure("simulated serial timeout", stamp=t)
    assert not st.valid and not st.stationary, st
    assert "timeout" in st.reason
    approx(o.state(now=t).x, x_before, 1e-12)      # pose frozen, not advanced
    print("[odom] failed read invalidates and never reports stationary OK")

    # ── freshness ──
    o = MotionFeedbackOdom(cfg); t = 600.0
    o.update(0.1, 0.0, 0.0, 0.05, stamp=t)
    assert o.is_fresh(now=t + 0.2)
    assert not o.is_fresh(now=t + 0.4)
    assert not o.is_stationary(now=t + 0.4), "stale feedback must not be stationary"
    print("[odom] freshness gating OK")

    # ── rejections ──
    def rejected(fn, needle):
        st = fn()
        assert not st.valid and needle in st.reason, f"expected {needle!r}, got {st.reason!r}"

    o = MotionFeedbackOdom(cfg); t = 700.0
    o.update(0.1, 0.0, 0.0, 0.05, stamp=t)
    rejected(lambda: o.update(99.0, 0.0, 0.0, 0.05, stamp=t + 0.05), "implausible linear")
    o.update(0.1, 0.0, 0.0, 0.05, stamp=t + 0.10)
    rejected(lambda: o.update(0.0, 0.0, 99.0, 0.05, stamp=t + 0.15), "implausible yaw")
    o.update(0.1, 0.0, 0.0, 0.05, stamp=t + 0.20)
    rejected(lambda: o.update(0.1, 0.0, 0.0, 5.0, stamp=t + 5.2), "stalled")
    o.update(0.1, 0.0, 0.0, 0.05, stamp=t + 5.3)
    rejected(lambda: o.update(float("nan"), 0.0, 0.0, 0.05, stamp=t + 5.35), "non-finite")
    print("[odom] implausible/stalled/non-finite reads rejected OK")

    # ── a stalled loop must not teleport the pose ──
    o = MotionFeedbackOdom(cfg); t = 800.0
    o.update(0.5, 0.0, 0.0, 0.05, stamp=t)
    x_before = o.state(now=t).x
    o.update(0.5, 0.0, 0.0, 3.0, stamp=t + 3.0)     # 3 s gap
    approx(o.state(now=t + 3.0).x, x_before, 1e-12)
    print("[odom] stalled loop does not teleport OK")

    # ── reader wiring against a stub driver ──
    class _Stub:
        def __init__(self): self.calls = 0
        def get_motion_data(self):
            self.calls += 1
            if self.calls == 3:
                raise OSError("simulated bus error")
            return (1.0, 0.0, 0.0)

    stub = _Stub()
    r = FeedbackOdomReader(stub)
    t = 900.0
    s0 = r.poll(now=t)
    assert "first sample" in s0.reason, s0
    s1 = r.poll(now=t + 0.05)
    assert s1.valid and s1.x > 0.0
    s2 = r.poll(now=t + 0.10)
    assert not s2.valid and "bus error" in s2.reason, s2
    s3 = r.poll(now=t + 0.15)
    assert s3.valid, "reader must recover after a transient failure"
    assert FeedbackOdomReader(None).poll(now=t).reason == "no Rosmaster device"
    print("[odom] reader stub/error/recovery OK")

    # ── covariance shape ──
    cov = MotionFeedbackOdom(cfg).covariance()
    assert len(cov) == 36 and cov[0] > 0 and cov[35] > 0 and cov[14] > 1e5
    print("[odom] covariance OK")

    # ── config validation ──
    for bad in (dict(linear_scale=0.0), dict(feedback_timeout_s=-1.0),
                dict(stationary_streak=0), dict(max_dt_s=float("nan"))):
        try:
            validate_config(FeedbackOdomConfig(**bad))
        except ValueError:
            continue
        raise AssertionError(f"config {bad} should have been rejected")
    print("[odom] config validation OK")

    print("[odom] SELFTEST PASSED")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        run_selftest()
        return 0
    ap.print_help()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
