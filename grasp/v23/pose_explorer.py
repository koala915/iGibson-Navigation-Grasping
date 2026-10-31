#!/usr/bin/env python3
"""Hold the arm camera open and step through candidate grasp-home poses.

WHY THIS EXISTS
---------------
The E1 calibration pass (2026-08-29) measured what a 6.5 cm sugarbox can
actually do at that pose: usable placement is roughly 前2.0-5.5 cm x 左2.0-右5.5
cm, about 26 cm2. The v23 policy is trained on a 101 cm2 band, so vision can
only ever feed it a third of what it expects. That is not a calibration
shortfall -- it is the pose. The box is 6.5 cm tall and the camera is 23 cm up
looking nearly straight down, so the box's TOP face projects 1.4x further from
the nadir than its base, and ``bbox_touches_border`` refuses the detection long
before the object's ground contact point nears the edge of the ground footprint.

Choosing a better pose is therefore a training-side decision, and it needs an
eye on the real camera, not a model. Two independent reasons not to trust the
model here:

  * The predicted window over-shoots. At E1 it says the far edge is x=0.305;
    you measured 前6.0 (x=0.2887) clipping the top. That is 2 cm optimistic.
  * Recovering the true pose from the measured homography does not work either.
    The calibrated patch is 3 x 7 cm seen from 23 cm, so the projective terms
    are almost zero, the fit is effectively affine, and decomposing it into R,t
    returns nonsense (it put the camera at 10.9 cm and 27 deg off vertical
    against an FK 23.4 cm and 1.4 deg).

So: this script drives the arm to each candidate, keeps ONE camera handle open
across all of them, and lets you look. The numbers it prints are the same
predictions, labelled as such. Your eyes are the measurement.

CHASSIS DRIFT
-------------
The 2026-08-29 session lost its data to this. Three consecutive poses came back
showing a different patch of floor, with the object gone, while the operator had
touched nothing; two poses later it was back. The base is on unbraked mecanum
wheels and the arm is heavy enough to push the robot while it travels, so "the
camera moved" and "the robot moved" are indistinguishable from one frame.

They are not indistinguishable from two. Revisiting a pose and phase-correlating
against the first visit measures the shift directly: identical pose, identical
overlay, so anything that moved is the robot. Every capture is now numbered
rather than overwritten, revisits report the shift, and a large one is called
out rather than left for someone to notice in the pictures afterwards.

Bracket a session with the same pose (the default start and the automatic return
are both E1), and check the number before trusting anything measured in between.

SAFETY
------
No policy is loaded and no grasp is attempted. The jaw stays open at 30 deg for
the whole session. Every move goes through ``GraspController.move_guarded_and_
verified`` -- the real one, borrowed unbound, so the preventive FloorGuard, the
8 deg/command rate limit and the fail-closed encoder reads are the same code
that runs during a grasp, not a re-implementation. Candidate poses are further
filtered so FK puts the finger pads at least 5 cm above the floor at every one.

USAGE
-----
    python3 pose_explorer.py --check          # no hardware, no camera
    python3 pose_explorer.py --list           # print the candidate table only
    python3 pose_explorer.py --i-am-beside-the-robot
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

if sys.version_info < (3, 8):
    sys.exit(
        "[FATAL] 需要 Python >= 3.8，目前是 {}（Jetson 的系統 python3 是 3.6.9）。\n"
        "        先啟用虛擬環境再重跑：source ~/grasp_venv/bin/activate".format(
            sys.version.split()[0]))

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
INTEGRATION = REPO_ROOT / "integration"
for _p in (str(HERE), str(INTEGRATION)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                    # noqa: E402

ARM_CAMERA = ("/dev/v4l/by-id/"
              "usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera-video-index0")

# Frames discarded before each capture, and how long to wait first. The arm has
# just stopped, the driver has a queue of frames from where it used to be, and
# the auto-exposure is still chasing the new scene. Both numbers are deliberately
# generous: a capture happens once per pose, so a second of latency costs
# nothing, and a stale frame costs the whole session.
CAMERA_FLUSH_FRAMES = 10
CAMERA_SETTLE_S = 0.7

# The object the window is judged against. Taller object -> smaller window, and
# height dominates: the same sweep gives 54 cm2 at 6.5 cm and 90 cm2 at 3.0 cm.
# Keep this at whatever you will actually grasp.
OBJECT_HEIGHT_M = 0.065
OBJECT_WIDTH_M = 0.023

# The ground mark from the E1 calibration pass: the vertical projection of
# gripper_center at E1, base (0.2287, -0.0035). Offsets are printed against it
# so a reading here can be compared with the numbers already measured.
MARK_X, MARK_Y = 0.2287, -0.0035

# Finger pads must clear the floor by this much at the home pose itself.
MIN_PAD_CLEARANCE_M = 0.05

# S1/S5 stay at 90 and the jaw at 30 (open) for every candidate: rotating the
# base or the wrist moves the camera without changing how much ground it sees,
# and it would invalidate the comparison between rows.
CANDIDATES = [
    # (name, S2, S3=S4, note)
    ("C3", 67.08, 9.79, "v21 baseline — the only pose with a real grasp logged"),
    ("E1", 74.20, 8.60, "v23 current — measured window ~26 cm2 with a 6.5 cm box"),
    ("F1", 78.00, 8.00, ""),
    ("F2", 81.00, 7.50, ""),
    ("F3", 83.00, 7.50, ""),
    ("F4", 85.00, 7.00, ""),
    ("F5", 86.00, 6.50, "near the sweep's peak predicted usable area"),
    ("F6", 87.50, 6.00, "top of the first sweep; check the arm is not folded awkwardly"),
    ("G1", 86.50, 5.00, "best found once every joint is kept 5 deg inside its "
                        "URDF limit. Beats F6 mostly at the NEAR edge."),
    ("G2", 90.00, 6.00, "S2 straight up (sim 0). Answers 'what about 90?' -- it "
                        "works, S2=90 is not special; all-90 is (see below)."),
]

# ⚠ "S2=S3=S4=90" -- API 90 is sim ZERO for every joint, so all-90 stands the arm
# straight up: camera at 43 cm, gripper_center at 51 cm, and the optical axis
# pointing at the CEILING (theta -90). It sees no ground at all. S2=90 on its own
# is fine and is included above as G2; it is S3/S4 at 90 that unfolds the arm.
#
# Sweeping S2 65-115 and S3=S4 0-25 for a 3 cm object, the best USABLE area
# (window intersected with the arm's evaluated reach) is ~135 cm2 near S2=99 with
# S3=S4=0 -- but S3=S4=0 IS the URDF joint limit, with nothing left to move
# against, which is not somewhere to start an episode. Requiring every joint to
# stay 5 deg inside its limit, the best is G1 at ~117 cm2, and F6 already reaches
# ~104. The last stretch is bought entirely by pushing S3/S4 into their stop.

# The widest region this arm has ever been shown to grasp in (v21's formal
# evaluation, 97/100 on a held-out seed). Used only to score how much of a
# candidate's camera window is somewhere the arm plausibly works.
REACH_X = (0.20, 0.33)
REACH_Y = (-0.10, 0.10)


def api_deg(entry):
    """Full six-servo API tuple for a candidate row."""
    _, s2, s34, _ = entry
    return (90.0, float(s2), float(s34), float(s34), 90.0, 30.0)


# ═══════════════════════════════════════════════════════════════════════════
# Geometry (prediction only — see the module docstring)
# ═══════════════════════════════════════════════════════════════════════════

class Geometry:
    """FK camera/gripper geometry, and the nav-home-anchored extrinsic guess."""

    NAV_HOME = (90.0, 140.0, 0.0, 0.0, 90.0, 30.0)
    NAV_H_MEASURED_M = 0.332      # ruler, Phase 1/2 hardware calibration
    NAV_THETA_MEASURED_DEG = 36.40

    def __init__(self, cfg, mapper, fk, acg, pybullet):
        self.cfg, self.mapper, self.fk, self.acg, self.p = cfg, mapper, fk, acg, pybullet
        self.cam_link = fk.name2id["mono_joint"]
        nav_pos, nav_theta, _, _, _ = self._fk(self.NAV_HOME)
        # A mounting error is fixed in the link frame, so differencing against
        # the one MEASURED pose cancels any constant frame offset.
        self.theta_correction = self.NAV_THETA_MEASURED_DEG - nav_theta
        self.nav_cam_z = float(nav_pos[2])

    def _fk(self, api):
        arm = self.mapper.hw_deg_to_sim_arm(list(api[:5]))
        grip = self.mapper.hw_deg_to_sim_grip(float(api[5]))
        self.fk._set_state(arm, grip)
        st = self.p.getLinkState(self.fk.body_id, self.cam_link,
                                 computeForwardKinematics=True,
                                 physicsClientId=self.fk.physics_client)
        pos = np.array(st[4], dtype=np.float64)
        R = np.array(self.p.getMatrixFromQuaternion(st[5])).reshape(3, 3)
        axis = R[:, 2]
        theta_fk = math.degrees(math.atan2(-float(axis[2]), float(axis[0])))
        tcp, _ = self.fk.compute(arm, grip)
        pad = self.fk.pad_bottom_z(arm, grip)
        return pos, theta_fk, np.array(tcp, dtype=np.float64), float(pad), axis

    def describe(self, name, api):
        pos, theta_fk, tcp, pad, axis = self._fk(api)
        pose = self.acg.ArmCamPose(
            name=name, arm_deg=tuple(api),
            theta_deg=theta_fk + self.theta_correction,
            h_m=self.NAV_H_MEASURED_M - (self.nav_cam_z - float(pos[2])),
            cam_x_m=float(pos[0]), cam_y_m=float(pos[1]),
            # Measured at E1 on 2026-08-29: base +y lands on image-LEFT.
            sign_y=-1.0,
            distance_model_measured=False, base_offset_measured=False,
            source="pose_explorer FK + nav-home difference; PREDICTED")
        info = {
            "name": name, "api_deg": [round(float(v), 2) for v in api],
            "cam_fk_xyz": [round(float(v), 4) for v in pos],
            "cam_h_predicted_m": round(pose.h_m, 4),
            "theta_predicted_deg": round(pose.theta_deg, 3),
            "tilt_off_vertical_deg": round(
                math.degrees(math.acos(min(1.0, abs(float(axis[2]))))), 2),
            "gripper_center_xyz": [round(float(v), 4) for v in tcp],
            "pad_bottom_z_m": round(pad, 4),
            "window": None,
        }
        try:
            xlo, xhi, ylo, yhi = self.acg.usable_placement_window(
                pose, OBJECT_HEIGHT_M, OBJECT_WIDTH_M)
        except self.acg.GroundGeometryError:
            info["window_note"] = (f"a {OBJECT_HEIGHT_M*100:.1f} cm object never "
                                   f"fits entirely in frame at this pose")
            return pose, info
        ox = max(0.0, min(xhi, REACH_X[1]) - max(xlo, REACH_X[0]))
        oy = max(0.0, min(yhi, REACH_Y[1]) - max(ylo, REACH_Y[0]))
        info["window"] = {
            "x_m": [round(xlo, 4), round(xhi, 4)],
            "y_m": [round(ylo, 4), round(yhi, 4)],
            "forward_cm_from_mark": [round((xlo - MARK_X) * 100, 1),
                                     round((xhi - MARK_X) * 100, 1)],
            "left_cm_from_mark": [round((ylo - MARK_Y) * 100, 1),
                                  round((yhi - MARK_Y) * 100, 1)],
            "area_cm2": round((xhi - xlo) * (yhi - ylo) * 1e4, 1),
            "area_within_reach_cm2": round(ox * oy * 1e4, 1),
        }
        return pose, info

    def project(self, pose, x, y, z=0.0):
        """Base point -> pixel, under the PREDICTED extrinsics. None if behind."""
        th = math.radians(pose.theta_deg)
        dx, dy, dz = x - pose.cam_x_m, y - pose.cam_y_m, z - pose.h_m
        xc = dy * pose.sign_y
        yc = -dx * math.sin(th) - dz * math.cos(th)
        zc = dx * math.cos(th) - dz * math.sin(th)
        if zc <= 1e-9:
            return None
        return (self.acg.CX + self.acg.FX * xc / zc,
                self.acg.CY + self.acg.FY * yc / zc)


def format_row(info):
    w = info.get("window")
    if not w:
        return (f"  {info['name']:4s} S2={info['api_deg'][1]:6.2f} "
                f"S3=S4={info['api_deg'][2]:5.2f}  "
                f"h={info['cam_h_predicted_m']:.3f} tilt={info['tilt_off_vertical_deg']:4.1f}  "
                f"{info.get('window_note', 'no window')}")
    f0, f1 = w["forward_cm_from_mark"]
    l0, l1 = w["left_cm_from_mark"]
    return (f"  {info['name']:4s} S2={info['api_deg'][1]:6.2f} "
            f"S3=S4={info['api_deg'][2]:5.2f}  "
            f"h={info['cam_h_predicted_m']:.3f} tilt={info['tilt_off_vertical_deg']:4.1f}  "
            f"area={w['area_cm2']:5.1f} reach={w['area_within_reach_cm2']:5.1f} cm2  "
            f"前{f0:+5.1f}..{f1:+5.1f}  右{-l0:4.1f}..左{l1:+5.1f} cm  "
            f"gc_z={info['gripper_center_xyz'][2]:.3f}")


# ═══════════════════════════════════════════════════════════════════════════
# Camera
# ═══════════════════════════════════════════════════════════════════════════

def annotate(frame, geo, pose, info, acg, cv2):
    """Draw the predicted ground grid and the pose summary onto a frame copy."""
    img = frame.copy()
    h, w = img.shape[:2]

    # Principal point. It is at u=212 in a 640-wide frame, not 320, which is why
    # the reachable ground is about twice as wide to image-right as to
    # image-left. Worth seeing rather than remembering.
    cx, cy = int(round(acg.CX)), int(round(acg.CY))
    cv2.drawMarker(img, (cx, cy), (0, 200, 255), cv2.MARKER_CROSS, 22, 2)
    cv2.putText(img, "optical axis", (cx + 10, cy - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)

    # Predicted ground grid, in base coordinates, at the object's BOTTOM plane.
    for fwd_cm in range(0, 11, 2):
        pts = []
        for left_cm in [x * 0.5 for x in range(-14, 9)]:
            q = geo.project(pose, MARK_X + fwd_cm / 100.0, MARK_Y + left_cm / 100.0)
            if q is not None and -2000 < q[0] < 2000 and -2000 < q[1] < 2000:
                pts.append((int(round(q[0])), int(round(q[1]))))
        for a, b in zip(pts, pts[1:]):
            cv2.line(img, a, b, (80, 180, 80), 1)
        if pts:
            cv2.putText(img, f"+{fwd_cm}", (pts[-1][0] + 4, pts[-1][1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (80, 180, 80), 1)
    for left_cm in (-6, -4, -2, 0, 2):
        pts = []
        for fwd_cm in [x * 0.5 for x in range(0, 21)]:
            q = geo.project(pose, MARK_X + fwd_cm / 100.0, MARK_Y + left_cm / 100.0)
            if q is not None and -2000 < q[0] < 2000 and -2000 < q[1] < 2000:
                pts.append((int(round(q[0])), int(round(q[1]))))
        for a, b in zip(pts, pts[1:]):
            cv2.line(img, a, b, (80, 180, 80), 1)
        if pts:
            tag = "mid" if left_cm == 0 else (f"L{left_cm}" if left_cm > 0
                                              else f"R{-left_cm}")
            cv2.putText(img, tag, (pts[0][0] - 8, pts[0][1] + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (80, 180, 80), 1)

    lines = [
        f"{info['name']}  API S2={info['api_deg'][1]:.2f} S3=S4={info['api_deg'][2]:.2f}",
        f"cam h={info['cam_h_predicted_m']:.3f}m tilt={info['tilt_off_vertical_deg']:.1f}deg"
        f"  gc_z={info['gripper_center_xyz'][2]:.3f}m",
    ]
    wd = info.get("window")
    if wd:
        f0, f1 = wd["forward_cm_from_mark"]
        l0, l1 = wd["left_cm_from_mark"]
        lines.append(f"PREDICTED window {wd['area_cm2']:.0f}cm2: "
                     f"fwd {f0:+.1f}..{f1:+.1f}  R{-l0:.1f}..L{l1:+.1f} cm")
    else:
        lines.append("PREDICTED: object never fits entirely in frame")
    lines.append("grid = PREDICTED ground, 2cm steps. Trust your eyes, not it.")
    y0 = 16
    for i, text in enumerate(lines):
        cv2.putText(img, text, (6, y0 + i * 16), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (0, 0, 0), 3)
        cv2.putText(img, text, (6, y0 + i * 16), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (255, 255, 255), 1)
    return img


# ═══════════════════════════════════════════════════════════════════════════
# Motion — borrows the controller's own guarded move, no re-implementation
# ═══════════════════════════════════════════════════════════════════════════

class Mover:
    """The minimum state ``move_guarded_and_verified`` reads.

    The method is taken UNBOUND off GraspController rather than copied, so the
    floor guard, the per-command rate limit and the fail-closed encoder reads
    are literally the ones a real grasp uses. If GraspController ever needs more
    state than this, ``check_mover_contract`` below fails on a laptop instead of
    raising AttributeError beside a powered arm.
    """

    def __init__(self, cfg, servo, mapper, floor_guard, arm_rads, grip_rad):
        self.cfg = cfg
        self.servo = servo
        self.mapper = mapper
        self.floor_guard = floor_guard
        self._current_arm_rads = np.asarray(arm_rads, dtype=np.float64)
        self._current_grip_rad = float(grip_rad)

    def _park_jaw_hold(self, *a, **kw):
        """Reached only on the stage-1 jaw-contact branch, which this tool never
        enables (stop_on_jaw_contact defaults False and nothing here sets it).

        Present anyway, and loud. Without it the contract check below has to
        whitelist the name, and a future change that DID take that branch would
        surface as an AttributeError next to a powered arm rather than a refusal.
        """
        raise RuntimeError(
            "pose_explorer reached the jaw-contact hold path. This tool never "
            "grasps and has no business closing the jaw on anything — stop and "
            "check why stop_on_jaw_contact was enabled.")

    def move_to(self, X, api, *, tol_deg=2.0):
        arm = self.mapper.hw_deg_to_sim_arm(list(api[:5]))
        grip = self.mapper.hw_deg_to_sim_grip(float(api[5]))
        return X.GraspController.move_guarded_and_verified(
            self, arm, grip, label="pose-explorer", run_time_ms=400,
            settle_s=0.35, tol_deg=tol_deg)


MOVER_ATTRS = ("cfg", "servo", "mapper", "floor_guard",
               "_current_arm_rads", "_current_grip_rad", "_park_jaw_hold")


def check_mover_contract(X):
    """Every attribute move_guarded_and_verified touches must exist on Mover."""
    import inspect
    src = inspect.getsource(X.GraspController.move_guarded_and_verified)
    used = set()
    for token in src.replace("(", " ").replace(")", " ").replace(",", " ").split():
        if token.startswith("self."):
            used.add(token[5:].split(".")[0].split("[")[0])
    missing = sorted(a for a in used if a not in MOVER_ATTRS)
    return missing


# ═══════════════════════════════════════════════════════════════════════════

def parse_args_from(argv):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--camera", default=ARM_CAMERA)
    p.add_argument("--port", default="/dev/myserial")
    p.add_argument("--snapshot-dir", default=str(Path.home() / "pose_explorer"))
    p.add_argument("--show", action="store_true",
                   help="also open a live window (needs a display)")
    p.add_argument("--object-height", type=float, default=OBJECT_HEIGHT_M,
                   help="object height in metres the window is judged against "
                        "(default %(default)s). This is the dominant term.")
    p.add_argument("--list", action="store_true",
                   help="print the candidate table and exit; no hardware")
    p.add_argument("--check", action="store_true",
                   help="check imports, camera, port and the borrowed-move "
                        "contract; open nothing, move nothing")
    p.add_argument("--i-am-beside-the-robot", action="store_true",
                   help="required to move the arm. The arm will travel between "
                        "poses; stay beside it with a hand on the power switch.")
    p.add_argument("--selftest", action="store_true")
    return p.parse_args(argv)


def build_geometry():
    import pybullet
    import x3plus_real_grasp as X
    import arm_cam_geometry as acg
    cfg = X.DeployConfig()
    mapper = X.JointMapper(cfg)
    fk = X.FKComputer(cfg.urdf_path)
    return X, acg, cfg, mapper, fk, Geometry(cfg, mapper, fk, acg, pybullet)


def print_table(geo, X):
    print("\n候選姿勢（S1/S5 固定 90，夾爪固定 30 開）")
    print("  面積是 PREDICTED —— E1 實測顯示遠端會高估約 2 cm，所以每一列都要用眼睛確認。")
    print(f"  物體高度 {OBJECT_HEIGHT_M*100:.1f} cm、寬 {OBJECT_WIDTH_M*100:.1f} cm；"
          f"前/左是相對 E1 地面記號 ({MARK_X:.4f}, {MARK_Y:+.4f})\n")
    infos = []
    for entry in CANDIDATES:
        api = api_deg(entry)
        _, info = geo.describe(entry[0], api)
        info["note"] = entry[3]
        infos.append(info)
        flag = "" if info["pad_bottom_z_m"] >= MIN_PAD_CLEARANCE_M else "  ⚠ pads low"
        print(format_row(info) + flag)
        if entry[3]:
            print(f"        {entry[3]}")
    return infos


def main(argv=None):
    args = parse_args_from(argv)
    global OBJECT_HEIGHT_M
    OBJECT_HEIGHT_M = float(args.object_height)

    if args.selftest:
        return selftest()

    print("[explorer] 載入 FK（不載入 PPO 權重，這支不夾東西）...")
    X, acg, cfg, mapper, fk, geo = build_geometry()

    missing = check_mover_contract(X)
    if missing:
        print(f"[FATAL] GraspController.move_guarded_and_verified 需要 Mover 沒有的欄位: "
              f"{missing}。不要在這個狀態下驅動手臂。")
        return 2
    print("[check] 借用的 move_guarded_and_verified 只用到 Mover 提供的狀態")

    infos = print_table(geo, X)

    if args.list:
        return 0

    unsafe = [i["name"] for i in infos if i["pad_bottom_z_m"] < MIN_PAD_CLEARANCE_M]
    if unsafe:
        print(f"\n[FATAL] 這些候選在 home 時指墊離地不足 "
              f"{MIN_PAD_CLEARANCE_M*100:.0f} cm: {unsafe}")
        return 2

    if not Path(args.port).exists():
        print(f"[FATAL] Rosmaster port 不存在: {args.port}")
        return 2
    print(f"[check] Rosmaster port: {args.port}")

    if args.check:
        import vision_grasp_bridge as vgb
        cap, frame, err = vgb.open_capture_checked(str(args.camera))
        if cap is None:
            print(f"[FATAL] 相機打不開: {err}")
            return 2
        h, w = frame.shape[:2]
        cap.release()
        note = acg.frame_size_mismatch(w, h) if hasattr(acg, "frame_size_mismatch") else None
        print(f"[check] 相機 OK，{w}x{h}")
        if note:
            print(f"[FATAL] {note}")
            return 2
        print("[check] PASS：沒有開硬體、沒有移動任何東西。")
        return 0

    if not args.i_am_beside_the_robot:
        print("\n[REFUSED] 這支會讓手臂在候選姿勢之間移動。")
        print("          確認你人在旁邊、手放電源開關，然後加 "
              "--i-am-beside-the-robot 重跑。")
        return 3

    return run_interactive(args, X, acg, cfg, mapper, fk, geo, infos)


def run_interactive(args, X, acg, cfg, mapper, fk, geo, infos):
    import cv2
    import vision_grasp_bridge as vgb

    snap_dir = Path(args.snapshot_dir)
    snap_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n[explorer] 開相機 {args.camera} …（整場只開這一次）")
    cap, frame, err = vgb.open_capture_checked(str(args.camera))
    if cap is None:
        print(f"[FATAL] 相機打不開: {err}")
        return 2
    fh, fw = frame.shape[:2]
    print(f"[explorer] 相機 OK {fw}x{fh}")
    # Belt and braces. Several V4L2 backends ignore this, which is why grab()
    # flushes explicitly rather than relying on it.
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    servo = X.ServoController(cfg, dry_run=False)
    rd = servo.read_degrees()
    if not rd.valid:
        print(f"[FATAL] 讀不到伺服機（{rd.reason}）。不送任何指令。")
        cap.release()
        return 2
    print(f"[explorer] 目前手臂 = {[round(d, 1) for d in rd.degrees]}")

    mover = Mover(cfg, servo, mapper, X.FloorGuard(cfg, fk),
                  mapper.hw_deg_to_sim_arm(rd.degrees[:5]),
                  mapper.hw_deg_to_sim_grip(rd.degrees[5]))

    by_name = {i["name"]: i for i in infos}
    order = [i["name"] for i in infos]
    idx = order.index("E1") if "E1" in order else 0
    keepers = []
    log = []

    def grab(flush=CAMERA_FLUSH_FRAMES, settle_s=CAMERA_SETTLE_S):
        """A frame of the scene as it is NOW, not whatever the driver has queued.

        cap.read() returns the OLDEST buffered frame. Between poses this process
        spends seconds on a guarded move and then blocks on input(), so the V4L2
        queue fills and the next read hands back a picture of the PREVIOUS pose.
        The 2026-08-29 sessions were captured this way: every snapshot was
        labelled with the pose the arm had just arrived at and showed the one
        before it. Revisiting a pose then compared two frames of two different
        scenes and returned a near-zero correlation, which the drift check
        reported as the chassis having moved. It had not.

        So: sleep for the scene and the auto-exposure to settle, discard the
        queue, and only then keep a frame.
        """
        time.sleep(max(0.0, settle_s))
        for _ in range(max(0, flush)):
            cap.read()
        for _ in range(5):
            ok, f = cap.read()
            if ok and f is not None and getattr(f, "size", 0) > 0:
                return f
            time.sleep(0.05)
        return None

    visits = {}          # pose name -> number of captures so far
    first_gray = {}      # pose name -> RAW grayscale of the first capture
    drifts = []

    def px_per_cm(pose):
        """Image scale near the ground mark, from the PREDICTED extrinsics.

        Only used to put the measured pixel shift into familiar units. The shift
        itself is measured, and is reported in pixels too.
        """
        a = geo.project(pose, MARK_X, MARK_Y)
        b = geo.project(pose, MARK_X + 0.01, MARK_Y)
        if a is None or b is None:
            return None
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        return d if d > 1e-6 else None

    def check_drift(name, frame, pose):
        """Compare a revisit against this pose's first capture.

        Raw frames only. The overlay is a deterministic function of the pose, so
        correlating annotated frames would lock onto the identical grid and
        report zero however far the robot had rolled.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if name not in first_gray:
            first_gray[name] = gray
            return None
        try:
            (dx, dy), response = cv2.phaseCorrelate(
                np.float32(first_gray[name]), np.float32(gray))
        except Exception as exc:
            print(f"[explorer] 漂移比對失敗: {exc}")
            return None
        shift_px = math.hypot(dx, dy)
        scale = px_per_cm(pose)
        cm = shift_px / scale if scale else None
        entry = {"pose": name, "shift_px": round(shift_px, 1),
                 "dx_px": round(dx, 1), "dy_px": round(dy, 1),
                 "confidence": round(float(response), 3),
                 "approx_cm": None if cm is None else round(cm, 2)}
        drifts.append(entry)
        cm_txt = "" if cm is None else f" ≈ {cm:.2f} cm"
        print(f"[explorer] 漂移檢查 {name}: 畫面位移 {shift_px:.1f} px"
              f"{cm_txt}  (dx={dx:+.1f}, dy={dy:+.1f}, 相關度 {response:.2f})")
        # Low confidence and a SMALL shift is not motion. Phase correlation
        # returns a small meaningless offset for two unrelated images, so a
        # rolled chassis shows up as a large shift held CONFIDENTLY. Reading the
        # two cases as one is what produced the wrong "the chassis moved"
        # conclusion on 2026-08-29; the giveaway was that the same four poses
        # failed every round, which no amount of rolling can do.
        if response < 0.20 and (cm is None or cm < 1.0):
            entry["verdict"] = "uncorrelated_small_shift"
            print("[explorer] ⚠ 兩張畫面幾乎不相關，但位移很小 —— 這通常是"
                  "抓到了舊影格，不是車子移動。若同一批姿勢每輪都這樣，"
                  "就是相機緩衝，不是底盤。")
        elif response < 0.20:
            entry["verdict"] = "uncorrelated_large_shift"
            print("[explorer] ⚠ 幾乎不相關且位移很大 —— 場景真的換了。"
                  "檢查車子、物體、以及畫面是不是抓錯姿勢。")
        elif cm is not None and cm >= 1.0:
            entry["verdict"] = "chassis_moved"
            print(f"[explorer] ⚠ 底盤移動了約 {cm:.1f} cm（相關度夠高，這是真的）。"
                  "這一輪之間量到的東西都不可信，把輪子固定好再重跑。")
        elif cm is not None and cm >= 0.3:
            entry["verdict"] = "small_movement"
            print(f"[explorer] 注意：約 {cm:.1f} cm 的位移。小，但不是零。")
        else:
            entry["verdict"] = "stable"
            print("[explorer] 底盤沒有明顯移動。")
        return entry

    def goto(name, entry_lookup):
        nonlocal idx
        entry = entry_lookup[name]
        api = api_deg(entry)
        print(f"\n[explorer] → {name}  API {list(api)}")
        res = mover.move_to(X, api, tol_deg=2.0)
        print(f"[explorer] reached={res['reached']} ({res['reason']}, "
              f"{res.get('iters')} iters, guard={sorted(set(res.get('guard', [])))})")
        if not res["reached"]:
            print("[explorer] 沒到位，先不要相信這一格的畫面。")
        pose, info = geo.describe(name, api)
        info["note"] = entry[3]
        by_name[name] = info
        print(format_row(info))
        f = grab()
        if f is None:
            print("[explorer] 抓不到影格。")
            return res, info, None
        check_drift(name, f, pose)
        visits[name] = visits.get(name, 0) + 1
        img = annotate(f, geo, pose, info, acg, cv2)
        # Numbered, not overwritten. The previous version wrote pose_E1.jpg on
        # every visit, so the start-of-session frame -- the one a drift check
        # compares against -- was destroyed by the return at the end.
        path = snap_dir / f"pose_{name}_{visits[name]:02d}.jpg"
        cv2.imwrite(str(path), img)
        print(f"[explorer] 快照 → {path}")
        if args.show:
            try:
                cv2.imshow("arm cam — pose explorer", img)
                cv2.waitKey(1)
            except Exception as exc:
                print(f"[explorer] --show 失敗（沒有顯示器？）: {exc}")
        log.append({"pose": name, "reached": bool(res["reached"]),
                    "reason": res["reason"], "geometry": info})
        return res, info, img

    lookup = {e[0]: e for e in CANDIDATES}
    print("\n" + "=" * 70)
    print("指令：  n 下一個   p 上一個   <名稱> 直接跳（C3/E1/F1..F6）")
    print("        r 重拍     k 記下這個姿勢（會問一句備註）")
    print("        m 90,80,7,7,90,30  手動姿勢（只有 S2/S3/S4 建議改）")
    print("        q 離開（回 E1 並存下紀錄）")
    print("=" * 70)

    goto(order[idx], lookup)
    while True:
        try:
            cmd = input("\n[explorer] > ").strip()
        except (EOFError, KeyboardInterrupt):
            cmd = "q"
        if not cmd:
            continue
        low = cmd.lower()
        if low == "q":
            break
        elif low == "n":
            idx = (idx + 1) % len(order)
            goto(order[idx], lookup)
        elif low == "p":
            idx = (idx - 1) % len(order)
            goto(order[idx], lookup)
        elif low == "r":
            goto(order[idx], lookup)
        elif low == "k":
            try:
                note = input("       備註（為什麼這個好？）: ").strip()
            except (EOFError, KeyboardInterrupt):
                note = ""
            keepers.append({"pose": order[idx],
                            "api_deg": list(api_deg(lookup[order[idx]])),
                            "note": note,
                            "geometry": by_name[order[idx]]})
            print(f"[explorer] 記下 {order[idx]}（目前 {len(keepers)} 個）")
        elif low.startswith("m "):
            try:
                vals = [float(v) for v in cmd[2:].replace(" ", "").split(",")]
                if len(vals) != 6:
                    raise ValueError("需要 6 個角度")
            except ValueError as exc:
                print(f"[explorer] 看不懂：{exc}")
                continue
            name = "M%d" % (sum(1 for k in lookup if k.startswith("M")) + 1)
            lookup[name] = (name, vals[1], vals[2], "manual")
            _, mi = geo.describe(name, tuple(vals))
            if mi["pad_bottom_z_m"] < MIN_PAD_CLEARANCE_M:
                print(f"[explorer] 拒絕：指墊只離地 "
                      f"{mi['pad_bottom_z_m']*100:.1f} cm，低於 "
                      f"{MIN_PAD_CLEARANCE_M*100:.0f} cm。")
                continue
            order.append(name)
            idx = len(order) - 1
            # Manual poses may set S3 != S4, so drive the tuple as given.
            print(f"\n[explorer] → {name}  API {vals}")
            res = mover.move_to(X, tuple(vals), tol_deg=2.0)
            print(f"[explorer] reached={res['reached']} ({res['reason']})")
            f = grab()
            if f is not None:
                pose, mi = geo.describe(name, tuple(vals))
                img = annotate(f, geo, pose, mi, acg, cv2)
                check_drift(name, f, pose)
                visits[name] = visits.get(name, 0) + 1
                p = snap_dir / f"pose_{name}_{visits[name]:02d}.jpg"
                cv2.imwrite(str(p), img)
                by_name[name] = mi
                print(format_row(mi))
                print(f"[explorer] 快照 → {p}")
        elif cmd.upper() in lookup:
            idx = order.index(cmd.upper()) if cmd.upper() in order else idx
            goto(cmd.upper(), lookup)
        else:
            print("[explorer] 不認得。n / p / r / k / q / 姿勢名稱 / m a,b,c,d,e,f")

    print("\n[explorer] 回 E1，並和開場那張比對底盤有沒有移動 …")
    goto("E1", lookup)
    cap.release()
    if args.show:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    out = snap_dir / "session.json"
    real = [d for d in drifts if d.get("verdict") in ("stable", "small_movement",
                                                      "chassis_moved")]
    stale = [d for d in drifts if d.get("verdict", "").startswith("uncorrelated")]
    worst = max((d["approx_cm"] or 0.0) for d in real) if real else 0.0
    out.write_text(json.dumps(
        {"object_height_m": OBJECT_HEIGHT_M, "mark_base_xy": [MARK_X, MARK_Y],
         "visited": log, "keepers": keepers, "drift_checks": drifts,
         "worst_drift_cm": round(worst, 2)},
        indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"[explorer] 紀錄 → {out}")
    if real:
        print(f"[explorer] 底盤漂移：最大 {worst:.2f} cm"
              f"（{len(real)} 次可信比對）")
        if worst >= 1.0:
            print("[explorer] ⚠ 這一輪的姿勢比較不可信 —— 車子在過程中移動了。")
    if stale:
        print(f"[explorer] ⚠ {len(stale)} 次比對相關度過低，已排除在漂移統計外。"
              f"影響到的姿勢：{sorted(set(d['pose'] for d in stale))}")
        print("[explorer]   若同一批姿勢每輪都這樣，那是抓到舊影格，不是底盤。"
              f"目前 flush={CAMERA_FLUSH_FRAMES}、settle={CAMERA_SETTLE_S}s，"
              "可以再調高。")
    if not drifts:
        print("[explorer] 沒有重複造訪任何姿勢，所以沒有量到底盤漂移。"
              "下次至少回同一個姿勢一次。")
    if keepers:
        print("\n你標記的姿勢：")
        for k in keepers:
            print(f"  {k['pose']}  API {k['api_deg']}   {k['note']}")
        print("\n把上面這幾行給訓練端當新的 grasp home 候選。")
    else:
        print("\n沒有標記任何姿勢。")
    return 0


def selftest():
    checks = []

    def ok(cond, label):
        checks.append((bool(cond), label))
        print(f"  [{'ok' if cond else 'FAIL'}] {label}")

    print("pose_explorer selftest (no hardware, no camera)")
    X, acg, cfg, mapper, fk, geo = build_geometry()

    ok(not check_mover_contract(X),
       "move_guarded_and_verified only touches state Mover supplies")

    e1 = [c for c in CANDIDATES if c[0] == "E1"][0]
    ok(api_deg(e1) == tuple(cfg.home_deg),
       "the E1 row equals DeployConfig.home_deg")

    _, info = geo.describe("E1", api_deg(e1))
    ok(abs(info["cam_fk_xyz"][2] - 0.2340) < 5e-4,
       f"E1 FK camera z {info['cam_fk_xyz'][2]} matches the manifest 0.2340")
    ok(abs(info["gripper_center_xyz"][2] - 0.1558) < 5e-4,
       f"E1 gripper_center z {info['gripper_center_xyz'][2]} matches 0.1558")
    ok(abs(geo.theta_correction + 3.600) < 1e-3,
       f"nav-home theta correction reproduces -3.600 ({geo.theta_correction:.3f})")

    c3 = [c for c in CANDIDATES if c[0] == "C3"][0]
    _, c3i = geo.describe("C3", api_deg(c3))
    ok(abs(c3i["cam_h_predicted_m"] - 0.2148) < 5e-4,
       f"the C3 row reproduces the registry's measured-anchored h 0.2148 "
       f"({c3i['cam_h_predicted_m']})")

    ok(all(geo.describe(c[0], api_deg(c))[1]["pad_bottom_z_m"]
           >= MIN_PAD_CLEARANCE_M for c in CANDIDATES),
       f"every candidate keeps the pads >= {MIN_PAD_CLEARANCE_M*100:.0f} cm up")

    ok(all(api_deg(c)[5] == 30.0 for c in CANDIDATES),
       "every candidate leaves the jaw open at 30 deg")
    ok(all(api_deg(c)[0] == 90.0 and api_deg(c)[4] == 90.0 for c in CANDIDATES),
       "S1 and S5 are fixed at 90 across the table")

    # sign_y: base +y must land LEFT of base -y in the image, as measured.
    pose, _ = geo.describe("E1", api_deg(e1))
    left = geo.project(pose, MARK_X, MARK_Y + 0.02)
    right = geo.project(pose, MARK_X, MARK_Y - 0.02)
    ok(left and right and left[0] < right[0],
       "base +y projects to image-LEFT, matching the 2026-08-29 measurement")

    names = [c[0] for c in CANDIDATES]
    ok(len(names) == len(set(names)), "candidate names are unique")

    fk.close()
    bad = [label for good, label in checks if not good]
    print()
    if bad:
        print(f"{len(bad)} of {len(checks)} checks FAILED:")
        for label in bad:
            print(f"  - {label}")
        return 1
    print(f"all {len(checks)} checks passed — pose explorer geometry and the "
          f"borrowed guarded move agree with the deployed stack")
    return 0


if __name__ == "__main__":
    sys.exit(main())
