#!/usr/bin/env python3
"""Arm-camera → grasp-frame geometry. One source of truth for every consumer.

The arm camera (URDF ``mono_link``) is bolted to ``arm_link4``, so it MOVES WITH
THE ARM. Its extrinsics — height above the floor, optical-axis depression, and
where its ground projection sits in the base frame — are mounting geometry that
is valid at exactly ONE arm pose. Only the intrinsics and distortion
coefficients are pose-independent (docs/calibration/CAMERA_CALIBRATION_PARAMETERS.md section 6).

Before this module the constants lived in three separate copies —
``integration/vision_grasp_bridge.py``, ``integration/vision_grasp_pipeline.py``
and ``detection/arm_cam.py`` — each with its own hard-coded ``H`` and
``THETA``. That is why the 2026-08-01 arm-pose guard landed in the pipeline and
not in the bridge: there was no single place to put it. Everything that turns an
arm-camera pixel into a grasp target now goes through here.

──────────────────────────────────────────────────────────────────────────────
Why the model needed fixing for v21
──────────────────────────────────────────────────────────────────────────────
The v17 constants were measured at the navigation home, where the camera looks
36.4 deg down and clearly *forward*. v21 starts the policy at the C3 grasp home,
where URDF FK puts the camera 11.7 cm lower and pitched to within a degree of
straight down. Three things in the old code break there, all silently:

1.  ``if dist > 0`` filters.  Ground distance is measured forward from the
    camera's own ground projection and is legitimately NEGATIVE for anything
    behind that point. At the C3 pose 64 % of the image rows land there, so the
    old filters would have discarded two thirds of the workspace and reported
    "no detection" while staring straight at the object.

2.  Lateral offset and metric width used the GROUND distance as the projection
    depth. The pin-hole relation is ``X = (u - cx) * Z / fx`` where ``Z`` is
    depth along the OPTICAL AXIS, not distance along the floor. The two differ
    by ``cos(theta+alpha)``: a 20 % under-estimate at the nav home, and a
    collapse to ~0 at C3, where every object would be reported dead-centre with
    zero width.

3.  ``cam_x`` was 0.0 ("not calibrated yet"). At the nav home that is a modest
    offset next to a ~45 cm ground distance. At C3 the ground distance is only
    +-4 cm, so ``cam_x`` IS the answer — leaving it at zero puts the target on
    top of the arm base instead of 25 cm in front of it.

──────────────────────────────────────────────────────────────────────────────
The model
──────────────────────────────────────────────────────────────────────────────
For an undistorted pixel (u, v), with theta the optical-axis depression measured
from horizontal *towards base +X* and H the camera height above the floor::

    alpha = atan((v - cy) / fy)          # extra depression of this pixel's row
    total = theta + alpha                # depression of the ray, from horizontal
    Z     = H * cos(alpha) / sin(total)  # depth along the optical axis  (> 0)
    d     = H / tan(total)               # SIGNED ground distance, forward-positive
    lat   = (u - cx) * Z / fx            # metres, image-right positive
    size  = px * Z / fx                  # metric size of a pixel span

    obj_x = cam_x + d
    obj_y = cam_y + sign_y * lat

``total`` measures the ray's angle below horizontal. total < 90 deg is a ray
landing in front of the camera, total = 90 deg is the point directly beneath it
(d = 0, perfectly well defined), and total > 90 deg is behind it (d < 0, tan is
negative and the arithmetic already has the sign right). The ray only fails to
meet the floor at total <= 0 or total >= 180 deg, which is what
``GroundGeometryError`` reports.

Note that ``theta`` is measured towards base +X. URDF FK reports the C3 optical
axis as 86.7 deg below horizontal with its horizontal component pointing
BACKWARD; measured towards +X that same ray is 93.3 deg. Feeding 86.7 into this
model would mirror the workspace about the camera.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

# ── Intrinsics: pose-independent, measured 2026-07-08 (Phase 1) ───────────────
# arm_cam_intrinsics.json, reprojection RMS 0.495 px.
FX = 919.08
FY = 919.41
CX = 212.23
CY = 168.42
DIST = (-0.3764, -0.0748, -0.0015, 0.0035, 0.4793)  # k1 k2 p1 p2 k3

# The image size those numbers were solved at. Intrinsics are only meaningful
# together with it: a stream that resizes or crops invalidates all five
# (docs/calibration/CAMERA_CALIBRATION_PARAMETERS.md line 13 says so explicitly), silently, and
# every coordinate downstream comes out confidently wrong.
#
# Note how far off-centre the principal point is -- 212 in a 640-wide frame, not
# 320. So the FRAME IS NOT SYMMETRIC ABOUT THE OPTICAL AXIS: u - cx spans -212 to
# +427 px, and the ground the camera sees extends about twice as far to one side
# as the other. Assuming the frame is ~424 px wide because cx is ~212 gets the
# lateral reach badly wrong, which is a mistake this file made until 2026-08-02.
IMG_W = 640
IMG_H = 480

# A ray closer than this to the horizon is treated as not hitting the floor at
# all: |d| grows without bound there and a 1 px error becomes metres.
MAX_ABS_GROUND_DISTANCE_M = 2.0

# How far the live arm pose may drift from the pose an extrinsic set was
# measured at before its numbers stop applying. The jaw joint (index 5) is
# excluded — opening or closing the gripper does not move the camera.
POSE_TOL_DEG = 1.0


class GroundGeometryError(ValueError):
    """The pixel's ray does not meet the floor in a usable way."""


@dataclass(frozen=True)
class ArmCamPose:
    """Arm-camera extrinsics tagged with the arm pose they belong to.

    ``theta_deg``  optical-axis depression from horizontal, measured towards
                   base +X. >90 means the axis has tipped past vertical and
                   leans back over the robot.
    ``h_m``        camera height above the floor.
    ``cam_x_m``    the camera's ground projection in the base frame, +X forward.
    ``cam_y_m``    same, +Y left.
    ``sign_y``     +1 or -1: does image-right map to base +Y or -Y.

    ``distance_model_measured`` / ``base_offset_measured`` record whether the
    numbers came off the hardware or out of the URDF. A predicted set is a
    legitimate starting point for a careful first run, but callers must not let
    one reach the real arm without the operator saying so out loud — hence
    ``require_measured()``.
    """

    name: str
    arm_deg: Tuple[float, ...]
    theta_deg: float
    h_m: float
    cam_x_m: float
    cam_y_m: float
    sign_y: float
    distance_model_measured: bool
    base_offset_measured: bool
    source: str

    @property
    def fully_measured(self) -> bool:
        return self.distance_model_measured and self.base_offset_measured

    def matches(self, arm_deg: Sequence[float], tol_deg: float = POSE_TOL_DEG) -> bool:
        """True if ``arm_deg`` is the pose these extrinsics were taken at.

        Compares the five arm joints only; the gripper does not move the camera.
        """
        cur = [float(v) for v in arm_deg]
        if len(cur) < 5:
            return False
        return all(abs(a - b) <= tol_deg for a, b in zip(cur[:5], self.arm_deg[:5]))

    def describe(self) -> str:
        flags = []
        if not self.distance_model_measured:
            flags.append("theta/H PREDICTED")
        if not self.base_offset_measured:
            flags.append("cam_x/cam_y PREDICTED")
        suffix = f"  [{', '.join(flags)}]" if flags else "  [measured]"
        return (f"{self.name}: arm={list(self.arm_deg)} theta={self.theta_deg:.3f}deg "
                f"H={self.h_m:.4f}m cam_x={self.cam_x_m:+.4f} cam_y={self.cam_y_m:+.4f} "
                f"sign_y={self.sign_y:+.0f}{suffix}")

    def require_measured(self, *, real: bool, acknowledged: bool = False) -> bool:
        """Refuse to drive the real arm off predicted extrinsics unless told to.

        Returns True when the set is fully measured. Otherwise prints what is
        missing and, under ``real`` without ``acknowledged``, raises SystemExit.
        Nothing downstream can catch a wrong extrinsic: the model returns a
        confident number for any theta, and the arm then grasps precisely in the
        wrong place.
        """
        if self.fully_measured:
            return True
        msg = (
            f"arm-camera extrinsics for {self.name!r} are not measured.\n"
            f"    {self.describe()}\n"
            f"    source: {self.source}\n"
            "    These came from URDF FK plus the mounting error solved at the nav\n"
            "    home, not from the hardware. Run docs/calibration/CALIBRATION_PLAN.md Phase 1/2 at\n"
            "    this pose (intrinsics and distortion carry over unchanged; only\n"
            "    theta/H/cam_x/cam_y need redoing) and pass the results with\n"
            "    --cam-theta/--cam-h/--cam-x/--cam-y."
        )
        if real and not acknowledged:
            raise SystemExit("[arm-cam] REFUSED: " + msg)
        print("[arm-cam] WARN: " + msg)
        return False

    def replace(self, **kw) -> "ArmCamPose":
        """A copy with overrides, re-flagged as measured for whatever was supplied.

        An operator passing --cam-theta/--cam-h has, by definition, measured
        them, so the override clears the corresponding PREDICTED flag; anything
        not supplied keeps the flag it had.
        """
        fields = dict(
            name=self.name, arm_deg=self.arm_deg, theta_deg=self.theta_deg,
            h_m=self.h_m, cam_x_m=self.cam_x_m, cam_y_m=self.cam_y_m,
            sign_y=self.sign_y,
            distance_model_measured=self.distance_model_measured,
            base_offset_measured=self.base_offset_measured, source=self.source,
        )
        touched_distance = any(kw.get(k) is not None for k in ("theta_deg", "h_m"))
        touched_offset = any(kw.get(k) is not None for k in ("cam_x_m", "cam_y_m"))
        for key, value in kw.items():
            if value is not None:
                fields[key] = value
        if touched_distance:
            fields["distance_model_measured"] = True
        if touched_offset:
            fields["base_offset_measured"] = True
        if touched_distance or touched_offset:
            fields["source"] = self.source + " + operator override"
        return ArmCamPose(**fields)


# ── Pose registry ─────────────────────────────────────────────────────────────
# v17 navigation home. theta and H measured 2026-07-08 (Phase 2 median, std
# 0.18 deg); the ground model was validated 0.25-0.50 m to 0.43 cm. cam_x/cam_y
# were never solved (Phase 3 was never run), so they are deployment-FK values in
# the policy frame -- see V21_C3_GRASP_HOME below for why that frame, and not the
# raw base_link one verify_camera_grasp_frame.py prints.
V17_NAV_HOME = ArmCamPose(
    name="v17_nav_home",
    arm_deg=(90.0, 140.0, 0.0, 0.0, 90.0, 30.0),
    theta_deg=36.40,
    h_m=0.332,
    cam_x_m=0.1783,
    cam_y_m=-0.0056,
    sign_y=1.0,
    distance_model_measured=True,
    base_offset_measured=False,
    source="Phase 1/2 hardware calibration 2026-07-08; cam_x/cam_y from URDF FK",
)

# v21 C3 grasp home — the pose the policy is trained to start from, and the one
# the whole mode-B flow detects from.
#
# Derivation. The positions below come from the DEPLOYMENT FK
# (x3plus_real_grasp.FKComputer), i.e. the URDF loaded at
# deploy_contract.URDF_TO_TRAINING_FRAME — the same frame the policy's object
# coordinates live in. That matters: integration/verify_camera_grasp_frame.py
# walks the raw URDF from base_link and reports mono_link at x=+0.2563, which is
# 19.9 mm short of the policy frame. Taking cam_x from that tool put the target
# two centimetres behind the object, and the entry tolerance is 22 mm.
#
#     mono_link, policy frame, C3       : (+0.2762, -0.0056, +0.2183)
#     mono_link, policy frame, nav home : (+0.1783, -0.0056, +0.3355)
#     optical axis at C3  : (-0.0583, 0, -0.9983) -> 93.340 deg from horizontal
#                           measured towards base +X
#     optical axis at nav : 40.000 deg
#
# Two corrections turn FK into a prediction of the real numbers. Both are
# DIFFERENCES anchored on a hardware measurement at the nav home, so any constant
# frame offset cancels and the result does not depend on which frame the FK
# reports in:
#     theta: the nav home was MEASURED at 36.40 against 40.000 from FK, so the
#            camera is mounted 3.600 deg less steep than the URDF claims. A
#            mounting error is fixed in the link frame, and the pitch axis is
#            base +Y at both poses, so it carries over unchanged:
#            93.340 - 3.600 = 89.740 deg.
#     H:     H is a PHYSICAL height above the real floor. The nav home was
#            MEASURED at 0.332 m, and the FK puts the camera 0.1172 m lower at C3
#            than at the nav home, so 0.332 - 0.1172 = 0.2148 m. (Note this is
#            NOT "FK z plus base_link's height above the floor": the URDF's
#            base_footprint->base_link offset is 0.0815 m, and the camera also
#            sits ~12.6 mm lower than that arithmetic predicts. Differencing
#            sidesteps both.)
#
# cam_x/cam_y are taken from the FK directly rather than by differencing, so
# unlike theta and H they carry the full modelling error, which is exactly why
# Phase 2 solves them from base-frame observations instead.
#
# Sanity: sweeping raw pixels across the whole frame through undistort + this
# model puts the visible ground band at x = 0.200..0.318 m and y = -0.059..+0.047
# (the v21 notes quote 0.196..0.330 for this pose), and they make the distance
# model 4.4x LESS sensitive to a theta error than at the nav home
# (-3.75 mm/deg vs -16.45 mm/deg) because the camera is nearly vertical.
#
# STILL A PREDICTION. Confirm on hardware before trusting it with the real arm.
V21_C3_GRASP_HOME = ArmCamPose(
    name="v21_c3_grasp_home",
    arm_deg=(90.0, 67.08, 9.79, 9.79, 90.0, 30.0),
    theta_deg=89.740,
    h_m=0.2148,
    cam_x_m=0.2762,
    cam_y_m=-0.0056,
    sign_y=1.0,
    distance_model_measured=False,
    base_offset_measured=False,
    source="deployment FK (policy frame) + nav-home mounting error "
           "(theta -3.600 deg, camera 0.1172 m lower than at nav home); "
           "NOT measured at this pose",
)

# v23 E1 grasp home — the pose the v23 policy starts from (grasp/v23/).
#
# Derived exactly the same way as C3 above, from the same deployment FK and the
# same two hardware-anchored differences. Running that derivation on C3
# reproduces its stored numbers to four decimals, which is why the E1 row is
# trusted to the same degree as the C3 one — and no further.
#
#     mono_link, policy frame, E1 : (+0.272307, -0.005559, +0.234046)
#     optical axis at E1          : (+0.024432, 0, -0.999701) -> 88.600 deg
#     theta : 88.600 - 3.600 (nav-home mounting error) = 85.000 deg
#     H     : 0.332 - 0.1014 (FK drop from nav home)   = 0.2306 m
#
# The one thing that is genuinely different in kind, not just in value: the
# optical axis CROSSES VERTICAL between C3 and E1. At C3 the axis leans back
# over the robot (x component -0.0583, theta > 90); at E1 it leans forward
# (+0.0244, theta < 90). The trigonometric ground-distance model in this module
# divides by the tangent of the angle between the ray and the ground, so as the
# camera approaches vertical the recovered lateral offset collapses toward zero
# and the depth becomes arbitrarily sensitive to a theta error. C3 was already
# in that regime (the module's own docstring says the offsets "collapse to ~0");
# E1, at 1.4 deg off vertical against C3's 3.3, is further into it.
#
# So these numbers exist to STAMP a detection with the pose it was taken at, and
# to let ground_hit()'s callers reason about the frame. They are not a licence to
# map pixels trigonometrically at E1. vision_grasp_bridge refuses to, and
# require_measured() below refuses a real run off them regardless.
#
# STILL A PREDICTION, and less of the workspace was ever checked here than at C3:
# nothing in this row has been on the hardware.
# sign_y is the ONE component here that has been measured, 2026-08-29, from the
# E1 calibration pass:
#
#     base y +0.0165 (2 cm left)  -> u ~ 105     y -0.0035 (centre) -> u ~ 223
#     base y -0.0235 (2 cm right) -> u ~ 342     y -0.0535 (5 cm right) -> u ~ 510
#
# Base +y lands on image-LEFT, so image-right is base -y, so sign_y = -1. The
# same pass confirms the consequence independently: a box 3 cm to the LEFT runs
# off the left edge while one 5 cm to the RIGHT is still comfortably inside,
# which is what CX=212.23 in a 640-wide frame predicts (the ground reaches 2.0x
# further to image-right than image-left) -- but only with this sign.
#
# ⚠ V21_C3_GRASP_HOME above still carries sign_y=+1.0 and has never been checked
# on hardware either. It is the same camera on the same link, so it is probably
# also -1; it is left alone here because changing it would change v21's live
# behaviour on the strength of a measurement taken at a different pose. Measure
# it at C3 before touching it.
#
# This does not affect the runtime grasp path. At a grasp home the bridge maps
# through the measured homography and uses this row only for the pose stamp
# (stamp_payload reads name and arm_deg). sign_y feeds the trigonometric model
# and usable_placement_window, and with the wrong sign the latter reports the
# reachable window MIRRORED -- which is how this was caught.
V23_E1_GRASP_HOME = ArmCamPose(
    name="v23_e1_grasp_home",
    arm_deg=(90.0, 74.2, 8.6, 8.6, 90.0, 30.0),
    theta_deg=85.000,
    h_m=0.2306,
    cam_x_m=0.2723,
    cam_y_m=-0.0056,
    sign_y=-1.0,
    distance_model_measured=False,
    base_offset_measured=False,
    source="deployment FK (policy frame) + nav-home mounting error "
           "(theta -3.600 deg, camera 0.1014 m lower than at nav home); "
           "theta/H/cam_x/cam_y NOT measured at this pose. sign_y=-1 IS "
           "measured (E1 calibration pass 2026-08-29). Nearly vertical "
           "(1.4 deg): use the measured homography, not this distance model.",
)

POSES: Dict[str, ArmCamPose] = {
    V17_NAV_HOME.name: V17_NAV_HOME,
    V21_C3_GRASP_HOME.name: V21_C3_GRASP_HOME,
    V23_E1_GRASP_HOME.name: V23_E1_GRASP_HOME,
}

# Poses that count as "grasp-home" — an arm pose the policy starts its episode
# from, where the camera looks almost straight down and pixel->base must go
# through a measured homography. One entry per deployed policy generation.
GRASP_HOME_POSES: Tuple[ArmCamPose, ...] = (V21_C3_GRASP_HOME, V23_E1_GRASP_HOME)
GRASP_HOME_POSE_NAMES = frozenset(pose.name for pose in GRASP_HOME_POSES)

# What the v21 stack detects from unless told otherwise. v23 callers pass
# --pose v23_e1_grasp_home explicitly; the default is NOT switched, because
# every existing mode A/B/C path still runs the C3 policy and a silently
# changed default would stamp their detections with the wrong pose. The grasp
# side compares that stamp against its encoders and would refuse every frame.
DEFAULT_POSE = V21_C3_GRASP_HOME


def get_pose(name: str) -> ArmCamPose:
    try:
        return POSES[name]
    except KeyError:
        raise KeyError(
            f"unknown arm-camera pose {name!r}; known: {sorted(POSES)}") from None


def pose_for_arm_deg(arm_deg: Sequence[float],
                     tol_deg: float = POSE_TOL_DEG) -> Optional[ArmCamPose]:
    """The registered extrinsic set matching this arm pose, or None."""
    for pose in POSES.values():
        if pose.matches(arm_deg, tol_deg):
            return pose
    return None


# ── Pixel → ground ────────────────────────────────────────────────────────────

def frame_size_mismatch(width: int, height: int) -> Optional[str]:
    """None when a frame matches the calibrated size, else what is wrong.

    Intrinsics only mean anything at the size they were solved at. A stream that
    delivers a resized or cropped frame keeps working -- YOLO still detects, the
    ground model still returns a number -- while every coordinate is scaled by
    the wrong focal length and offset from the wrong principal point. On this
    robot the camera devices are also known to move between /dev/video indices
    between boots, so pointing the arm pipeline at the rear camera is a live
    hazard, and the two have different frame sizes and very different intrinsics.
    """
    if int(width) == IMG_W and int(height) == IMG_H:
        return None
    return (f"frame is {int(width)}x{int(height)} but the arm-camera intrinsics "
            f"were calibrated at {IMG_W}x{IMG_H}. fx/fy/cx/cy do not transfer "
            f"across a resize or a crop (docs/calibration/CAMERA_CALIBRATION_PARAMETERS.md), so "
            f"every coordinate computed from this frame would be wrong. Either "
            f"stream at {IMG_W}x{IMG_H}, or re-run Phase 1 at the size you are "
            f"actually using. If this is a {IMG_W}x{IMG_H} camera reporting "
            f"something else, check you are reading the ARM camera and not the "
            f"rear one -- they swap /dev/video indices between boots.")


# A bbox that runs into the frame edge is CLIPPED. Left/right/bottom clipping
# invalidates the bottom-centre or width directly. Top-only clipping is different:
# the bottom and side edges can remain observable, so a measured grasp-home
# homography may opt in to using it while every other geometry path stays strict.
BORDER_MARGIN_PX = 2.0


def bbox_border_hits(x1: float, y1: float, x2: float, y2: float,
                     margin_px: float = BORDER_MARGIN_PX) -> Tuple[str, ...]:
    """Return frame edges touched by a bbox, in deterministic order."""
    hits = []
    if x1 <= margin_px:
        hits.append("left")
    if y1 <= margin_px:
        hits.append("top")
    if x2 >= IMG_W - 1 - margin_px:
        hits.append("right")
    if y2 >= IMG_H - 1 - margin_px:
        hits.append("bottom")
    return tuple(hits)


def bbox_touches_border(x1: float, y1: float, x2: float, y2: float,
                        margin_px: float = BORDER_MARGIN_PX) -> Optional[str]:
    """None when the box is wholly inside the frame, else which edge it hits.

    The whole ground model rests on the bbox bottom-centre being the point where
    the object touches the floor. Clip that box against the frame and the
    assumption quietly stops holding: at the C3 pose an object whose true bottom
    edge is 60 px below the frame reports 3.6 cm too far forward, and one clipped
    90 px at the side reports 2.2 cm off in y. Both are larger than the 22 mm
    stage-0 entry tolerance, both look like ordinary healthy detections, and the
    forward case lands INSIDE the reach envelope, so that gate does not catch it
    either. The only honest response is to refuse the detection and let the
    operator move the object into frame.
    """
    hits = bbox_border_hits(x1, y1, x2, y2, margin_px)
    if not hits:
        return None
    if hits == ("top",):
        return ("bbox touches only the top frame edge. The full silhouette is "
                "clipped, but the bottom and both side edges remain visible. "
                "Default is fail-closed; a measured grasp-home homography may "
                "explicitly allow this case")
    return (f"bbox touches the {'/'.join(hits)} frame edge, so it is clipped and "
            f"its bottom-centre is the image border rather than where the object "
            f"meets the floor — move the object further into view")


def usable_placement_window(pose: "ArmCamPose", object_height_m: float,
                            object_width_m: float = 0.023,
                            step_mm: int = 1):
    """Where an object of this size fits ENTIRELY in frame, in the base frame.

    Not the same as the camera's ground footprint, and much smaller. Seen from
    near vertical, an object's top face projects further from the nadir than its
    base, so its silhouette is larger than its footprint and it runs off the edge
    of the frame sooner. A clipped box is refused (bbox_touches_border), so this
    is the range in which a detection can actually be produced -- and therefore
    the range calibration placements must live in.

    Taller means smaller: for the C3 pose a flat marker fits over x 0.218-0.304
    while a 6.5 cm sugarbox only fits over x 0.239-0.292.

    Returns (x_lo, x_hi, y_lo, y_hi); the y range is measured at mid-x.
    """
    th = math.radians(pose.theta_deg)

    def project(x, y, z):
        dx, dy, dz = x - pose.cam_x_m, y - pose.cam_y_m, z - pose.h_m
        xc = dy * pose.sign_y
        yc = -dx * math.sin(th) - dz * math.cos(th)
        zc = dx * math.cos(th) - dz * math.sin(th)
        if zc <= 1e-9:
            return None
        return CX + FX * xc / zc, CY + FY * yc / zc

    def fits(x0, y0):
        half = object_width_m / 2.0
        us, vs = [], []
        for sx in (-half, half):
            for sy in (-half, half):
                for sz in (0.0, object_height_m):
                    q = project(x0 + sx, y0 + sy, sz)
                    if q is None:
                        return False
                    us.append(q[0])
                    vs.append(q[1])
        return bbox_touches_border(min(us), min(vs), max(us), max(vs)) is None

    step = step_mm / 1000.0
    xs = [i * step for i in range(int(0.15 / step), int(0.36 / step))
          if fits(i * step, 0.0)]
    if not xs:
        raise GroundGeometryError(
            f"an object {object_height_m*100:.1f} cm tall never fits entirely in "
            f"frame at pose {pose.name!r}")
    x_mid = (min(xs) + max(xs)) / 2.0
    ys = [i * step for i in range(int(-0.12 / step), int(0.14 / step))
          if fits(x_mid, i * step)]
    return min(xs), max(xs), min(ys), max(ys)


def undistort_pixel(u: float, v: float, iters: int = 8) -> Tuple[float, float]:
    """Undistort one arm-cam pixel (plumb-bob k1,k2,p1,p2,k3).

    Same fixed-point iteration as cv2.undistortPoints(..., P=K), written out so
    this module imports without OpenCV. Distortion moves the bbox bottom-center
    by ~9 px and theta was solved on undistorted pixels, so every pixel MUST go
    through here before the ground model.
    """
    k1, k2, p1, p2, k3 = DIST
    xd = (u - CX) / FX
    yd = (v - CY) / FY
    x, y = xd, yd
    for _ in range(iters):
        r2 = x * x + y * y
        radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x = (xd - dx) / radial
        y = (yd - dy) / radial
    return CX + x * FX, CY + y * FY


@dataclass(frozen=True)
class GroundHit:
    """Where one undistorted pixel's ray meets the floor."""

    obj_x: float        # base frame, metres
    obj_y: float
    ground_dist_m: float   # SIGNED, forward-positive from the camera's ground point
    lateral_m: float       # SIGNED, image-right positive, before sign_y is applied
    depth_m: float         # along the optical axis; the pin-hole projection depth
    total_angle_deg: float

    def metric_size(self, pixels: float) -> float:
        """Metric size of a pixel span at this depth (e.g. bbox width → metres)."""
        return abs(float(pixels)) * self.depth_m / FX


def ground_hit(u_undistorted: float, v_undistorted: float, pose: ArmCamPose,
               max_abs_dist_m: float = MAX_ABS_GROUND_DISTANCE_M,
               z_plane: float = 0.0) -> GroundHit:
    """Project one undistorted pixel onto a horizontal plane in the base frame.

    ``z_plane`` is the height of that plane. It defaults to 0 (the floor), which
    is what a pixel known to be a ground-contact point wants. Pass the object's
    half-height to intersect at its centroid instead -- see
    ``silhouette_centre_target()`` for why that is the better estimator here.

    Raises GroundGeometryError when the ray runs along the horizon instead of
    meeting the floor. A ground distance of zero (or a negative one) is NOT an
    error: it is the point directly beneath a near-vertical camera (or behind
    it), and at the C3 pose that is most of the workspace.
    """
    h_eff = pose.h_m - float(z_plane)
    if h_eff <= 0.0:
        raise GroundGeometryError(
            f"plane at z={z_plane} is at or above the camera ({pose.h_m} m)")
    alpha = math.atan((float(v_undistorted) - CY) / FY)
    total = math.radians(pose.theta_deg) + alpha
    total_deg = math.degrees(total)
    if not (0.0 < total_deg < 180.0):
        raise GroundGeometryError(
            f"ray at {total_deg:.2f} deg below horizontal never meets the floor "
            f"(theta={pose.theta_deg:.2f}, alpha={math.degrees(alpha):+.2f})")
    sin_total = math.sin(total)
    tan_total = math.tan(total)
    if sin_total <= 0.0 or not math.isfinite(tan_total) or abs(tan_total) < 1e-9:
        raise GroundGeometryError(
            f"degenerate ground geometry at total angle {total_deg:.2f} deg")
    depth = h_eff * math.cos(alpha) / sin_total
    dist = h_eff / tan_total
    if not math.isfinite(depth) or not math.isfinite(dist):
        raise GroundGeometryError("non-finite ground solution")
    if abs(dist) > max_abs_dist_m:
        raise GroundGeometryError(
            f"ground distance {dist:+.3f} m is beyond the usable band "
            f"(+-{max_abs_dist_m} m); the ray is too close to the horizon")
    lateral = (float(u_undistorted) - CX) * depth / FX   # image-right positive
    return GroundHit(
        obj_x=pose.cam_x_m + dist,
        obj_y=pose.cam_y_m + pose.sign_y * lateral,
        ground_dist_m=dist,
        lateral_m=lateral,
        depth_m=depth,
        total_angle_deg=total_deg,
    )


def ground_hit_from_raw(u: float, v: float, pose: ArmCamPose,
                        max_abs_dist_m: float = MAX_ABS_GROUND_DISTANCE_M,
                        z_plane: float = 0.0) -> GroundHit:
    """undistort_pixel() + ground_hit(), which is what every caller actually wants."""
    u_u, v_u = undistort_pixel(u, v)
    return ground_hit(u_u, v_u, pose, max_abs_dist_m, z_plane)


def silhouette_centre_target(x1: float, y1: float, x2: float, y2: float,
                             pose: ArmCamPose, object_height_m: float) -> GroundHit:
    """Where a detected object's CENTRE is, from its bounding box.

    The usual recipe -- take the bbox bottom-centre and intersect it with the
    floor -- assumes the lowest pixel of the silhouette is where the object meets
    the ground. That holds for a forward-looking camera. It does not hold at the
    C3 pose, where the camera is within a degree of vertical:

      * An object INSIDE the camera's nadir has its top face project FURTHER from
        the nadir than its base, so the bottom of the bbox is the top face, not
        the base at all.
      * Even outside the nadir, where the bottom of the bbox really is the base,
        it is the base's far EDGE rather than the object's centre.

    Modelled on a 2.3 x 2.3 x 6.5 cm box across the C3 workspace, the bottom-centre
    recipe is off by -11 to -45 mm -- up to twice the 22 mm stage-0 entry
    tolerance, and it varies with position, so calibration cannot absorb it.

    Intersecting the ray through the SILHOUETTE CENTRE with the object's
    half-height plane instead is within 4.3 mm over the same sweep, because for a
    convex object the silhouette centre is very nearly the centroid's ray and the
    centroid sits at half height. It needs the object's height, which the v21
    contract already requires for wrist_z_offset -- so it costs nothing new.

    Estimator comparison over base x 0.21-0.31, worst |error|:
        bbox bottom-centre -> ground   45.2 mm   (the old recipe)
        bbox centre -> top plane        8.3 mm
        bbox centre -> half height      4.3 mm   <- this
        bbox centre -> ground          16.9 mm
    """
    if not math.isfinite(object_height_m) or object_height_m <= 0.0:
        raise GroundGeometryError(
            f"object height must be finite and positive, got {object_height_m}")
    return ground_hit_from_raw((x1 + x2) / 2.0, (y1 + y2) / 2.0, pose,
                               z_plane=object_height_m / 2.0)


def silhouette_width_to_object_width(pixel_span: float, hit: "GroundHit",
                                     pose: "ArmCamPose",
                                     object_height_m: float) -> float:
    """Metric object width from its silhouette span, corrected for its height.

    The widest part of a box's silhouette is its TOP face, which is closer to the
    camera than the object's mid-height and therefore magnified more. Scaling the
    pixel span by the mid-height depth -- the depth the position estimate uses --
    over-reports the width by exactly ``(H - h/2) / (H - h)``: for the 6.5 cm
    sugarbox at the C3 pose that is 2.80 cm against a true 2.3 cm, a constant
    +22%. Undo it by re-scaling to the top-face depth.

    Matters because the width gate refuses objects wider than the jaw opens, and
    a 22% over-report rejects things that would in fact fit; the v17 stack also
    sizes its close angle from this number.
    """
    if not math.isfinite(object_height_m) or object_height_m <= 0.0:
        return hit.metric_size(pixel_span)
    top_h = pose.h_m - object_height_m
    mid_h = pose.h_m - object_height_m / 2.0
    if top_h <= 0.0 or mid_h <= 0.0:
        return hit.metric_size(pixel_span)
    return hit.metric_size(pixel_span) * (top_h / mid_h)


# ── Cross-process pose agreement ──────────────────────────────────────────────
# The bridge knows which pose its numbers assume but cannot read the servos; the
# grasp process owns the serial port but has no idea how the coordinates it is
# handed were computed. Neither can catch a mismatch alone. So the bridge stamps
# the payload and the grasp side checks the stamp against its own encoders at
# latch time — the one moment both facts exist together.

PAYLOAD_POSE_KEY = "cam_pose"
PAYLOAD_POSE_NAME_KEY = "cam_pose_name"


def stamp_payload(payload: dict, pose: ArmCamPose) -> dict:
    """Add the pose stamp the grasp side verifies against its encoders."""
    payload[PAYLOAD_POSE_KEY] = [round(float(v), 3) for v in pose.arm_deg]
    payload[PAYLOAD_POSE_NAME_KEY] = pose.name
    return payload


def parse_pose_stamp(msg: dict) -> Optional[Tuple[Tuple[float, ...], Optional[str]]]:
    """Read a pose stamp out of a detection payload.

    Returns (arm_deg, name) or None when the sender did not stamp one. Raises
    ValueError on a stamp that is present but malformed — a garbled stamp must
    not read as an absent one, or a sender bug turns into a skipped check.
    """
    if PAYLOAD_POSE_KEY not in msg:
        return None
    raw = msg[PAYLOAD_POSE_KEY]
    if not isinstance(raw, (list, tuple)):
        raise ValueError(f"{PAYLOAD_POSE_KEY} must be a list of joint degrees")
    if not 5 <= len(raw) <= 6:
        raise ValueError(
            f"{PAYLOAD_POSE_KEY} must have 5 or 6 entries, got {len(raw)}")
    try:
        deg = tuple(float(v) for v in raw)
    except (TypeError, ValueError):
        raise ValueError(f"{PAYLOAD_POSE_KEY} entries must be numbers") from None
    if not all(math.isfinite(v) for v in deg):
        raise ValueError(f"{PAYLOAD_POSE_KEY} entries must be finite")
    name = msg.get(PAYLOAD_POSE_NAME_KEY)
    return deg, (str(name) if name is not None else None)


def pose_stamp_disagreement(stamp_deg: Sequence[float],
                            actual_deg: Sequence[float],
                            tol_deg: float = POSE_TOL_DEG) -> Optional[str]:
    """None when the stamp agrees with the measured arm pose, else why not.

    Compares the five arm joints; the gripper is excluded because it does not
    move the camera, and it is legitimately in a different position by the time
    the grasp side reads its encoders.
    """
    stamp = [float(v) for v in stamp_deg]
    actual = [float(v) for v in actual_deg]
    if len(stamp) < 5 or len(actual) < 5:
        return (f"need at least 5 joint angles to compare, got "
                f"stamp={len(stamp)} actual={len(actual)}")
    worst_i, worst = -1, 0.0
    for i, (a, b) in enumerate(zip(stamp[:5], actual[:5])):
        delta = abs(a - b)
        if delta > worst:
            worst_i, worst = i, delta
    if worst <= tol_deg:
        return None
    return (f"joint S{worst_i + 1} differs by {worst:.2f} deg "
            f"(tolerance {tol_deg} deg): the detection was computed for arm pose "
            f"{[round(v, 2) for v in stamp[:5]]} but the arm is actually at "
            f"{[round(v, 2) for v in actual[:5]]}")


def _selftest() -> int:
    """Geometry self-checks. Run: python integration/arm_cam_geometry.py"""
    failures = []

    def check(label, cond, detail=""):
        if cond:
            print(f"  ok   {label}")
        else:
            failures.append(label)
            print(f"  FAIL {label} {detail}")

    print("arm_cam_geometry self-test")

    # The principal ray at the nav home must reproduce the validated model.
    hit = ground_hit(CX, CY, V17_NAV_HOME)
    expect = V17_NAV_HOME.h_m / math.tan(math.radians(V17_NAV_HOME.theta_deg))
    check("nav-home principal ray reproduces H/tan(theta)",
          abs(hit.ground_dist_m - expect) < 1e-9, f"{hit.ground_dist_m} vs {expect}")

    # Depth exceeds ground distance whenever the camera is tilted at all, and the
    # ratio is exactly cos(total). This is the bug the old code had.
    ratio = hit.ground_dist_m / hit.depth_m
    check("depth > ground distance, ratio == cos(total)",
          abs(ratio - math.cos(math.radians(hit.total_angle_deg))) < 1e-9,
          f"ratio={ratio}")
    check("old d-based lateral would have been ~20% low at the nav home",
          0.78 < ratio < 0.82, f"ratio={ratio}")

    # C3: the point under the camera is d == 0, and rows below it go negative.
    c3 = V21_C3_GRASP_HOME
    straight_down = ground_hit(CX, CY, c3)
    check("C3 principal ray lands within a centimetre of the camera's own foot",
          abs(straight_down.ground_dist_m) < 0.01,
          f"d={straight_down.ground_dist_m:+.4f}")
    check("C3 principal ray still gives a healthy projection depth",
          0.20 < straight_down.depth_m < 0.23, f"Z={straight_down.depth_m:.4f}")
    check("C3 obj_x is dominated by cam_x, not by the ground distance",
          abs(straight_down.obj_x - c3.cam_x_m) < 0.01,
          f"obj_x={straight_down.obj_x:.4f}")

    lower = ground_hit(CX, 400.0, c3)
    check("C3 rows below the principal point give NEGATIVE ground distance",
          lower.ground_dist_m < 0, f"d={lower.ground_dist_m:+.4f}")
    check("...and that is still a usable target in front of the base",
          0.15 < lower.obj_x < 0.25, f"obj_x={lower.obj_x:.4f}")

    # A near-vertical camera must NOT collapse lateral offset to zero.
    off_axis = ground_hit(CX + 100.0, CY, c3)
    check("C3 lateral offset survives a near-vertical camera",
          abs(off_axis.obj_y - c3.cam_y_m) > 0.015,
          f"lateral={off_axis.obj_y - c3.cam_y_m:+.4f}")
    check("C3 metric width is not zero either",
          off_axis.metric_size(100.0) > 0.015,
          f"width={off_axis.metric_size(100.0):.4f}")

    # Horizon handling.
    try:
        ground_hit(CX, CY, V17_NAV_HOME.replace(theta_deg=0.0))
    except GroundGeometryError:
        print("  ok   a horizontal camera is rejected, not silently answered")
    else:
        failures.append("horizontal camera not rejected")
        print("  FAIL horizontal camera not rejected")

    # Pose stamping.
    stamped = stamp_payload({"x": 0.25}, c3)
    parsed = parse_pose_stamp(stamped)
    check("stamp round-trips", parsed is not None and parsed[1] == c3.name)
    check("stamp agrees with the same pose",
          pose_stamp_disagreement(c3.arm_deg, c3.arm_deg) is None)
    check("stamp disagrees with the nav home",
          pose_stamp_disagreement(c3.arm_deg, V17_NAV_HOME.arm_deg) is not None)
    check("a different jaw angle is not a disagreement",
          pose_stamp_disagreement(c3.arm_deg, tuple(c3.arm_deg[:5]) + (170.0,)) is None)
    check("no stamp reads as None, not as agreement",
          parse_pose_stamp({"x": 0.25}) is None)
    try:
        parse_pose_stamp({PAYLOAD_POSE_KEY: "90,67,9,9,90"})
    except ValueError:
        print("  ok   a malformed stamp raises instead of reading as absent")
    else:
        failures.append("malformed stamp accepted")
        print("  FAIL malformed stamp accepted")

    check("C3 extrinsics are flagged as not measured", not c3.fully_measured)
    check("an override clears the flag it supplies",
          c3.replace(theta_deg=88.0).distance_model_measured)
    check("...and leaves the other flag alone",
          not c3.replace(theta_deg=88.0).base_offset_measured)

    print(f"\n{'PASS' if not failures else 'FAIL'}: "
          f"{len(failures)} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
