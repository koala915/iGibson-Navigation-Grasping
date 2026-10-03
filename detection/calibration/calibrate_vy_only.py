import socket
import sys
import json
import time
import csv
import os
import numpy as np
from pathlib import Path

MOTION_ALLOWED = "--real" in sys.argv[1:]


# ============================================================
# 1. Jetson motor server 設定
# ============================================================

JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "yahboom.local")
MOTOR_PORT = 7000


# ============================================================
# 2. Vy 校正設定
# ============================================================

# 橫移 action
# 如果 left 不會動，改成 "right"
VY_ACTION = "right"

# 希望每次橫移幾秒
TEST_DURATION = 2.0

# 要測的 speed
TEST_SPEEDS = [30, 35, 40, 45, 50]

# 在 TEST_DURATION 期間，每隔多久重送一次 action
# 避免只送一次後底層 timeout 自己停掉
COMMAND_INTERVAL = 0.05

# stop 重送
STOP_REPEAT = 8
STOP_INTERVAL = 0.03

# 輸出檔案
RAW_CSV_PATH = Path("vy_only_calibration_raw.csv")
RESULT_JSON_PATH = Path("vy_only_calibration_result.json")


# ============================================================
# 3. socket motor server
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
# 4. 單次 Vy 測試
# ============================================================

def run_one_vy_test(sock, speed):
    print("\n----------------------------------------")
    print(f"[TEST] vy 橫移")
    print(f"[ACTION] {VY_ACTION}, speed = {speed}")
    print(f"[INFO] 連續送指令時間 = {TEST_DURATION:.2f} s")
    print(f"[INFO] 重送間隔 = {COMMAND_INTERVAL:.3f} s")
    print("----------------------------------------")

    input("把車放好，按 Enter 開始...")

    stop_motor(sock)
    time.sleep(0.3)

    print(f"[RUN] keep sending {VY_ACTION}, speed={speed}")

    t0 = time.perf_counter()
    send_count = 0

    while True:
        elapsed = time.perf_counter() - t0

        if elapsed >= TEST_DURATION:
            break

        send_motor_action(sock, VY_ACTION, speed)
        send_count += 1

        time.sleep(COMMAND_INTERVAL)

    actual_motion_time = time.perf_counter() - t0

    print("[SEND] stop")
    stop_motor(sock)

    print(f"[INFO] 實際連續送指令時間：約 {actual_motion_time:.3f} s")
    print(f"[INFO] 總共送出 {send_count} 次 {VY_ACTION}")

    print("\n請量車子實際橫向移動距離。")
    print("單位：cm")
    print("如果方向跟你定義的正方向相反，就輸入負值。")
    measured_cm = float(input("實際橫移距離 cm = "))

    vy_mps = (measured_cm / 100.0) / actual_motion_time

    print(f"[RESULT] speed={speed} -> vy={vy_mps:.5f} m/s")

    return {
        "action": VY_ACTION,
        "motor_command": speed,
        "duration_s": TEST_DURATION,
        "actual_motion_time_s": actual_motion_time,
        "send_count": send_count,
        "measured_cm": measured_cm,
        "vy_mps": vy_mps
    }


# ============================================================
# 5. 擬合 vy = KY * speed
# ============================================================

def fit_vy(rows):
    xs = np.array([float(r["motor_command"]) for r in rows], dtype=float)
    ys = np.array([float(r["vy_mps"]) for r in rows], dtype=float)

    if len(xs) < 2:
        return None

    # 完整線性：vy = k * speed + b
    k, b = np.polyfit(xs, ys, 1)

    # 強制過原點：vy = k0 * speed
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
                "action",
                "motor_command",
                "duration_s",
                "actual_motion_time_s",
                "send_count",
                "measured_cm",
                "vy_mps"
            ]
        )

        writer.writeheader()
        writer.writerows(rows)


def save_result_json(result):
    with RESULT_JSON_PATH.open("w", encoding="utf-8") as f:
        json.dump(result, f, indent=4, ensure_ascii=False)


def print_result(result):
    print("\n===================================================")
    print("Vy 校正完成")
    print("===================================================")

    vy_result = result["vy"]

    ky = vy_result["k_through_origin"]
    k = vy_result["k"]
    b = vy_result["b"]

    print(f"\n建議先用： vy = {ky:.6f} * speed    (m/s)")
    print(f"完整線性： vy = {k:.6f} * speed + {b:.6f}")
    print(f"RMSE through origin = {vy_result['rmse_through_origin']:.6f}")

    print("\n反推 speed：")
    print(f"speed_y = desired_vy / {ky:.6f}")

    print("\n檔案位置：")
    print(f"raw csv     = {RAW_CSV_PATH.resolve()}")
    print(f"result json = {RESULT_JSON_PATH.resolve()}")

    print("\n之後可用：")
    print(f"KY = {ky:.8f}")


# ============================================================
# 6. 主程式
# ============================================================

def main():
    if not MOTION_ALLOWED:
        print("[SAFETY] Calibration not started. Re-run with --real in a clear test area.")
        return
    print("[INFO] Vy only calibration")
    print("[INFO] 這支只測橫移 vy")
    print("[INFO] 傳送格式完全照你舊程式：")
    print('[INFO] {"action": "...", "speed": ...}')
    print()
    print(f"[INFO] VY_ACTION = {VY_ACTION}")
    print(f"[INFO] raw csv 會存在：{RAW_CSV_PATH.resolve()}")
    print(f"[INFO] result json 會存在：{RESULT_JSON_PATH.resolve()}")
    print()

    rows = []

    sock = connect_motor_server()
    print("[INFO] motor server connected.")

    try:
        stop_motor(sock)

        for speed in TEST_SPEEDS:
            row = run_one_vy_test(sock, speed)
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

    vy_fit = fit_vy(rows)

    if vy_fit is None:
        print("[WARN] 資料不足，無法擬合。")
        return

    result = {
        "vy_action": VY_ACTION,
        "test_duration_s": TEST_DURATION,
        "test_speeds": TEST_SPEEDS,
        "command_interval_s": COMMAND_INTERVAL,
        "vy": vy_fit
    }

    save_result_json(result)
    print_result(result)


if __name__ == "__main__":
    main()
