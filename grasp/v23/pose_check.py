#!/usr/bin/env python3
"""C3 gripper geometry check — measures the one thing FK cannot: the real finger.

Why this exists
---------------
The C3 ruler gate measured gripper_center at 14.4 cm against an FK prediction of
14.39 — the arm pose is right. But the finger pad bottom measured 12.8 against a
prediction of 11.10, and a photograph of the gripper ruled out the easy
explanation (a narrow tip below the flat pad face that the ruler missed): the
fingers are blunt-ended curved parts with nothing below them.

So the model and the hardware disagree about the finger by ~1.7 cm at the OPEN
jaw angle. The open jaw is not when grasping happens, and the disagreement can
behave completely differently once the jaw closes:

  * If it vanishes when closed, the cause is the open-angle mapping
    (hw_deg_to_sim_grip(30) -> -1.5 rad) putting the modelled fingers at a
    different splay than the real ones. Grip depth is then unaffected, because
    grip depth is set at the closed angle.
  * If it persists when closed, the real finger is genuinely ~1.7 cm shorter
    than rlink2.STL, and every grasp lands that much shallower on the object
    than training intended — survivable on the 66 mm wood block, fatal on the
    13 mm bottle cap.

The same two readings also measure close_drop, the distance gripper_center sinks
as the jaw closes. The controller actively adds close_drop back into its target
(dc.GRASP_CLOSE_DROP = 26.7 mm, FK says 28.5 mm at C3), so an error there biases
the height of every single grasp. It has never been measured on hardware.

Safety
------
This script NEVER moves the arm. It refuses to run unless the arm is already
parked at C3, and the only servo it drives is S6. At C3 the modelled pad bottom
is 111 mm open and 82 mm closed, both far above the 8 mm floor line, and the jaw
travels in place.

Usage
-----
    # park the arm first
    python3 x3plus_real_grasp.py --model models/... --vecnorm models/... \
        --contract obs_28_incremental --object-height 0.066 \
        --obj-x 0.24 --obj-y 0.00 --obj-z 0.033 \
        --max-steps 0 --real --unlock-candidate-real

    python3 pose_check.py              # FK table only, no hardware
    python3 pose_check.py --real       # drive S6 and prompt for readings
"""
from __future__ import annotations

import argparse
import sys
import time

import x3plus_real_grasp as X


ARM_TOL_DEG = 3.0


def fk_table(fk, mapper, cfg):
    """Predicted heights at C3 for both jaw angles, in cm."""
    arm = mapper.hw_deg_to_sim_arm(list(cfg.home_deg[:5]))
    rows = {}
    for label, hw in (("open", 30.0), ("closed", 180.0)):
        g = mapper.hw_deg_to_sim_grip(hw)
        tcp, _ = fk.compute(arm, g)
        rows[label] = {
            "gripper_center_cm": float(tcp[2]) * 100.0,
            "finger_bottom_cm": fk.pad_bottom_z(arm, g) * 100.0,
        }
    rows["close_drop_cm"] = (rows["open"]["gripper_center_cm"]
                             - rows["closed"]["gripper_center_cm"])
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--real", action="store_true",
                    help="Drive S6. Without this the script only prints the FK table.")
    ap.add_argument("--port", type=str, default="/dev/myserial")
    ap.add_argument("--run-time-ms", type=int, default=200,
                    help="Per-segment jaw travel time. The 8 deg/command rate limit "
                         "means the jaw walks to its target in ~19 segments.")
    ap.add_argument("--settle-s", type=float, default=0.15,
                    help="Pause after each segment before reading the encoder back.")
    args = ap.parse_args()

    if args.real and not X._ROSMASTER_AVAILABLE:
        print("[FATAL] --real requires Rosmaster_Lib, but the driver is not importable.")
        return 2

    cfg = X.DeployConfig(serial_port=args.port)
    mapper = X.JointMapper(cfg)
    fk = X.FKComputer(cfg.urdf_path)
    rows = fk_table(fk, mapper, cfg)

    print()
    print("=" * 68)
    print("C3 夾爪幾何量測 — FK 預測值（公分，離地）")
    print("=" * 68)
    print(f"{'':16s}{'gripper_center':>16s}{'手指最低點':>14s}")
    for label in ("open", "closed"):
        r = rows[label]
        tag = "張開 (S6=30)" if label == "open" else "閉合 (S6=180)"
        print(f"  {tag:14s}{r['gripper_center_cm']:14.2f}{r['finger_bottom_cm']:16.2f}")
    print(f"\n  close_drop（閉合時 gripper_center 下沉）= {rows['close_drop_cm']:.2f} cm"
          f"   契約常數 {X.dc.GRASP_CLOSE_DROP*100:.2f} cm")
    print()
    print("  張開那列已量過：gripper_center 14.4（預測 14.39，命中）、")
    print("  手指最低點 12.8（預測 11.10，差 1.7）。這支要問的是閉合那列。")
    print("=" * 68)

    if not args.real:
        print("\n（未加 --real，只印表格，不驅動任何伺服機。）")
        fk.close()
        return 0

    servo = X.ServoController(cfg, dry_run=False)
    try:
        rd = servo.read_degrees()
        if not rd.valid:
            print(f"\n[SAFETY] 讀不到伺服機（{rd.reason}）。不送任何指令。")
            return 1

        # The arm must already be at C3. This script has no guarded-move primitive
        # and is not going to grow one -- refusing is the correct answer, not
        # driving the arm there with a raw command.
        drift = [abs(a - b) for a, b in zip(rd.degrees[:5], cfg.home_deg[:5])]
        print(f"\n目前手臂 S1-S5 = {[round(d, 1) for d in rd.degrees[:5]]}")
        print(f"C3 應為        = {[round(d, 1) for d in cfg.home_deg[:5]]}")
        if max(drift) > ARM_TOL_DEG:
            print(f"\n[SAFETY] 手臂不在 C3（最大偏差 {max(drift):.1f}° > {ARM_TOL_DEG}°）。")
            print("         這支不會移動手臂。請先跑 x3plus_real_grasp.py --max-steps 0 "
                  "--real --unlock-candidate-real")
            return 1
        print(f"  → 在 C3（最大偏差 {max(drift):.1f}°）")

        def jaw(hw_deg: float, label: str) -> bool:
            """Drive S6 to hw_deg, iterating because send_degrees rate-limits.

            send_degrees clamps every joint to max_delta_deg (8) per command, so a
            single call moves the jaw 8 degrees, not the 150 this needs. The
            controller has the same problem and solves it the same way: keep
            commanding the target and let the limiter walk there. The arm joints are
            re-sent unchanged every iteration, so the limiter is a no-op for them.
            """
            target = list(cfg.home_deg[:5]) + [hw_deg]
            # Budget for the full S6 range rather than trying to predict the segment
            # count. The limiter walks from ServoController's own _last_deg, not from
            # the encoder, and the two disagree whenever a previous run left the jaw
            # somewhere else -- so any cleverer estimate is a guess that strands the
            # gripper mid-travel. The loop exits the moment the encoder confirms
            # arrival, so a generous cap costs nothing.
            max_iters = int(180.0 / cfg.max_delta_deg) + 6
            for i in range(max_iters):
                if not servo.send_degrees(target, run_time_ms=args.run_time_ms):
                    print(f"[SAFETY] S6 指令被拒（{label}，第 {i+1} 次）。停止。")
                    return False
                time.sleep(args.settle_s)
                rd = servo.read_degrees()
                if rd.valid and abs(rd.degrees[5] - hw_deg) <= 2.0:
                    print(f"  S6 → {hw_deg:.0f}（{i+1} 段，讀回 {rd.degrees[5]:.1f}）")
                    return True
            rd = servo.read_degrees()
            got = f"{rd.degrees[5]:.1f}" if rd.valid else "讀不到"
            print(f"[SAFETY] S6 在 {max_iters} 段內沒到 {hw_deg:.0f}（讀回 {got}）。停止。")
            return False

        print("\n--- 步驟 1：閉合夾爪（手臂不動）---")
        input("按 Enter 閉合，或 Ctrl+C 中止 > ")
        if not jaw(180.0, "closed"):
            return 1
        r = rows["closed"]
        print(f"\n請量兩個數字（夾爪現在是閉合的）：")
        print(f"  a) 兩指 pad 中點離地   → FK 預測 {r['gripper_center_cm']:.2f} cm")
        print(f"  b) 手指最低點離地     → FK 預測 {r['finger_bottom_cm']:.2f} cm")
        print(f"\n  判讀：(b) 若接近 {r['finger_bottom_cm']:.1f} → 張開時的 1.7 cm 差異在閉合時消失，")
        print(f"        夾取深度不受影響（原因是開角映射，不是手指長度）。")
        print(f"        (b) 若接近 {r['finger_bottom_cm']+1.7:.1f} → 差異是常數，手指真的比模型短，")
        print(f"        每一夾都會淺 1.7 cm，瓶蓋一定夾不到。")
        print(f"  (a) 則直接量出 close_drop：14.4 −(a) 應為 {rows['close_drop_cm']:.2f} cm")
        input("\n量完按 Enter 張開夾爪 > ")

        print("\n--- 步驟 2：張開夾爪，回到起始狀態 ---")
        if not jaw(30.0, "open"):
            return 1
        print("完成。夾爪已回到張開，手臂全程沒有移動。")
        return 0
    except KeyboardInterrupt:
        print("\n[Interrupted] 送出張開指令後結束。")
        try:
            servo.send_degrees(list(cfg.home_deg[:5]) + [30.0], run_time_ms=args.run_time_ms)
        except Exception:
            pass
        return 130
    finally:
        fk.close()


if __name__ == "__main__":
    sys.exit(main())
