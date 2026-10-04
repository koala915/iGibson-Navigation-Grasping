import socket
import sys
import json
import time
import csv
import math
import os
import threading
import numpy as np
from pathlib import Path

MOTION_ALLOWED = "--real" in sys.argv[1:]


# ============================================================
# 1. Jetson motor server 設定
# ============================================================

JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "yahboom.local")
MOTOR_PORT = 7000


# ============================================================
# 2. 已量好的 vx 校正結果
# ============================================================

# 你剛剛量到的結果：
# vx = 0.006995 * speed
KX = 0.006995


# ============================================================
# 3. Wz 校正設定
# ============================================================

# 要測哪些 speed
TEST_SPEEDS = [30, 35, 40, 45, 50]

# 每個 speed 測幾次 90 度
TRIALS_PER_SPEED = 4

# 每次目標角度：90 度
TARGET_ANGLE_DEG = 90.0
TARGET_ANGLE_RAD = math.radians(TARGET_ANGLE_DEG)

# 在旋轉期間，每隔多久重送一次 turn_left
COMMAND_INTERVAL = 0.05

# 停車重送次數
STOP_REPEAT = 8
STOP_INTERVAL = 0.03

# 輸出檔案
RAW_CSV_PATH = Path("wz_90deg_calibration_raw.csv")
RESULT_JSON_PATH = Path("wz_90deg_calibration_result.json")


# ============================================================
# 4. socket motor server
# 格式完全照你原本可動程式：
# {"action": "...", "speed": ...}
# ============================================================

def connect_motor_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((JETSON_IP, MOTOR_PORT))
    sock.settimeout(1.0)
    return sock


def send_motor_action(sock, action, speed):
    if not MOTION_ALLOWED and action != "stop":
        raise RuntimeError("motor calibration requires the explicit --real flag")
    msg = {
        "action": str(action),
        "speed": int(speed)
    }

    data = (json.dumps(msg) + "\n").encode("utf-8")
    sock.sendall(data)


def stop_motor(sock, repeat=STOP_REPEAT, interval=STOP_INTERVAL):
    for _ in range(repeat):
        try:
            send_motor_action(sock, "stop", 0)
        except Exception:
            pass
        time.sleep(interval)


# ============================================================
# 5. keep-alive 旋轉控制
# ============================================================

def keep_sending_action(sock, action, speed, stop_event):
    """
    背景執行緒：
    在 stop_event 被觸發前，一直重送 turn_left。
    這是為了避免車只跑一小段就 timeout 自動停。
    """
    while not stop_event.is_set():
        try:
            send_motor_action(sock, action, speed)
        except Exception as e:
            print(f"[WARN] send failed: {e}")
            break

        time.sleep(COMMAND_INTERVAL)


def run_one_90deg_trial(sock, speed, trial_index):
    """
    單次 90 度測試：
    1. 開始 turn_left
    2. 你看到 90 度，按 Enter
    3. 程式停車
    4. 用時間算 rad/s
    """

    print("\n----------------------------------------")
    print(f"[TEST] speed = {speed}, trial = {trial_index}")
    print("[ACTION] turn_left")
    print("看到車子轉到 90 度時，直接按 Enter 停止")
    print("----------------------------------------")

    input("把車放好，按 Enter 開始旋轉...")

    stop_motor(sock)
    time.sleep(0.3)

    stop_event = threading.Event()

    t0 = time.perf_counter()

    sender_thread = threading.Thread(
        target=keep_sending_action,
        args=(sock, "turn_left", speed, stop_event)
    )
    sender_thread.daemon = True
    sender_thread.start()

    input("到 90 度時按 Enter：")

    t1 = time.perf_counter()

    stop_event.set()
    sender_thread.join(timeout=1.0)

    stop_motor(sock)

    elapsed = t1 - t0

    if elapsed <= 0:
        wz = 0.0
    else:
        wz = TARGET_ANGLE_RAD / elapsed

    print(f"[RESULT] speed={speed}, time={elapsed:.3f} s, wz={wz:.5f} rad/s")

    return {
        "speed": speed,
        "trial": trial_index,
        "target_angle_deg": TARGET_ANGLE_DEG,
        "target_angle_rad": TARGET_ANGLE_RAD,
        "elapsed_s": elapsed,
        "wz_rad_s": wz
    }


# ============================================================
# 6. 擬合 Wz：wz = KZ * speed
# ============================================================

def fit_wz(rows):
    xs = np.array([float(r["speed"]) for r in rows], dtype=float)
    ys = np.array([float(r["wz_rad_s"]) for r in rows], dtype=float)

    if len(xs) < 2:
        return None

    # 完整線性：wz = k * speed + b
    k, b = np.polyfit(xs, ys, 1)

    # 強制過原點：wz = k0 * speed
    k0 = float(np.sum(xs * ys) / np.sum(xs * xs))

    pred = k * xs + b
    pred0 = k0 * xs

    rmse = float(np.sqrt(np.mean((ys - pred) ** 2)))
    rmse0 = float(np.sqrt(np.mean((ys - pred0) ** 2)))

    return {
        "k": float(k),
        "b": float(b),
        "k_through_origin": float(k0),
        "rmse": rmse,
        "rmse_through_origin": rmse0,
        "samples": len(xs)
    }


def save_raw_csv(rows):
    with RAW_CSV_PATH.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "speed",
                "trial",
                "target_angle_deg",
                "target_angle_rad",
                "elapsed_s",
                "wz_rad_s"
            ]
        )
        writer.writeheader()
        writer.writerows(rows)


def save_result_json(result):
    import json

    with RESULT_JSON_PATH.open("w", encoding="utf-8") as f:
        json.dump(result, f, indent=4, ensure_ascii=False)


def print_result(result):
    print("\n===================================================")
    print("Wz 校正完成")
    print("===================================================")

    kz = result["wz"]["k_through_origin"]
    k = result["wz"]["k"]
    b = result["wz"]["b"]

    print(f"\n你現在已有：")
    print(f"vx = {KX:.6f} * speed    (m/s)")

    print(f"\n這次量到：")
    print(f"建議先用： wz = {kz:.6f} * speed    (rad/s)")
    print(f"完整線性： wz = {k:.6f} * speed + {b:.6f}")
    print(f"RMSE through origin = {result['wz']['rmse_through_origin']:.6f}")

    print("\n反推 speed：")
    print(f"speed_x = desired_vx / {KX:.6f}")
    print(f"speed_z = desired_wz / {kz:.6f}")

    print("\n檔案位置：")
    print(f"raw csv   = {RAW_CSV_PATH.resolve()}")
    print(f"result json = {RESULT_JSON_PATH.resolve()}")

    print("\n之後可以先用這個：")
    print(f"KX = {KX:.8f}")
    print(f"KZ = {kz:.8f}")


# ============================================================
# 7. 主程式
# ============================================================

def main():
    if not MOTION_ALLOWED:
        print("[SAFETY] Calibration not started. Re-run with --real in a clear test area.")
        return
    print("[INFO] Wz 90-degree calibration")
    print("[INFO] 這版不用量角器")
    print("[INFO] 車子開始轉，你看到 90 度就按 Enter")
    print("[INFO] 每個 speed 會測 4 次")
    print()
    print(f"[INFO] raw csv 會存在：{RAW_CSV_PATH.resolve()}")
    print(f"[INFO] result json 會存在：{RESULT_JSON_PATH.resolve()}")
    print()

    rows = []

    sock = connect_motor_server()
    print("[INFO] motor server connected.")

    try:
        stop_motor(sock)

        for speed in TEST_SPEEDS:
            print("\n===================================================")
            print(f"開始測 speed = {speed}")
            print("===================================================")

            for trial in range(1, TRIALS_PER_SPEED + 1):
                row = run_one_90deg_trial(sock, speed, trial)
                rows.append(row)

        stop_motor(sock)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected.")

    finally:
        try:
            stop_motor(sock)
        except Exception:
            pass

        try:
            sock.close()
        except Exception:
            pass

    if len(rows) == 0:
        print("[WARN] 沒有資料，不產生結果。")
        return

    save_raw_csv(rows)

    wz_fit = fit_wz(rows)

    result = {
        "KX_from_previous_vx_calibration": KX,
        "target_angle_deg": TARGET_ANGLE_DEG,
        "test_speeds": TEST_SPEEDS,
        "trials_per_speed": TRIALS_PER_SPEED,
        "command_interval_s": COMMAND_INTERVAL,
        "wz": wz_fit
    }

    save_result_json(result)

    print_result(result)


if __name__ == "__main__":
    main()
