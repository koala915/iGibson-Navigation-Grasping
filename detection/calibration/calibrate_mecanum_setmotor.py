import socket
import sys
import json
import time
import csv
import math
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
# 2. 校正設定
# ============================================================

# 希望車子連續動幾秒
TEST_DURATION = 2.0

# speed 太慢就改這裡
TEST_SPEEDS = [30, 35, 40, 45, 50]

# 在 TEST_DURATION 期間，每隔多久重送一次 action
# 你的車會自己跑一下就停，所以一定要 keep-alive 重送
COMMAND_INTERVAL = 0.05

# 停車指令重送次數
STOP_REPEAT = 8
STOP_INTERVAL = 0.03

RAW_CSV_PATH = Path("action_speed_calibration_raw.csv")
RESULT_JSON_PATH = Path("action_speed_calibration_result.json")


# ============================================================
# 3. socket motor server
# 完全照你舊程式格式：
# {"action": "...", "speed": ...}
# 你原本可動的程式就是 send_motor_action(sock, action, speed)
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
# 4. axis 對應 motor action
# ============================================================

def axis_to_action(axis):
    """
    vx：前進速度校正
    vy：左右平移速度校正
    wz：旋轉速度校正

    注意：
    你的舊程式確定有 forward / turn_left / turn_right / stop。
    但是 left / right 不一定有支援。
    如果 vy 測試不會動，代表 Jetson server 沒寫 left/right。
    """

    if axis == "vx":
        return "forward"

    elif axis == "vy":
        return "left"

    elif axis == "wz":
        return "turn_left"

    else:
        raise ValueError(f"Unknown axis: {axis}")


# ============================================================
# 5. 單次測試：重點是 keep-alive
# ============================================================

def run_one_test(sock, axis, speed):
    action = axis_to_action(axis)

    print("\n----------------------------------------")
    print(f"[TEST] axis = {axis}")
    print(f"[ACTION] {action}, speed = {speed}")
    print(f"[INFO] 連續送指令時間 = {TEST_DURATION:.2f} s")
    print(f"[INFO] 重送間隔 = {COMMAND_INTERVAL:.3f} s")
    print("----------------------------------------")

    input("把車放好，按 Enter 開始...")

    # 測試前先停穩
    stop_motor(sock)
    time.sleep(0.3)

    print(f"[RUN] keep sending {action}, speed={speed}")

    t0 = time.perf_counter()
    send_count = 0

    while True:
        elapsed = time.perf_counter() - t0

        if elapsed >= TEST_DURATION:
            break

        send_motor_action(sock, action, speed)
        send_count += 1

        time.sleep(COMMAND_INTERVAL)

    actual_motion_time = time.perf_counter() - t0

    print("[SEND] stop")
    stop_motor(sock)

    print(f"[INFO] 實際連續送指令時間：約 {actual_motion_time:.3f} s")
    print(f"[INFO] 總共送出 {send_count} 次 {action}")

    if axis in ["vx", "vy"]:
        print("\n請量車子實際移動距離。")
        print("單位：cm")
        print("如果方向跟你定義的正方向相反，就輸入負值。")
        measured = float(input("實際移動距離 cm = "))

        velocity = (measured / 100.0) / actual_motion_time
        unit = "m/s"

    elif axis == "wz":
        print("\n請量車子實際旋轉角度。")
        print("單位：degree")
        print("如果方向跟你定義的正方向相反，就輸入負值。")
        measured = float(input("實際旋轉角度 degree = "))

        velocity = math.radians(measured) / actual_motion_time
        unit = "rad/s"

    else:
        raise ValueError(axis)

    print(f"[RESULT] {axis}, action={action}, speed={speed} -> {velocity:.5f} {unit}")

    return {
        "axis": axis,
        "action": action,
        "motor_command": speed,
        "duration_s": TEST_DURATION,
        "actual_motion_time_s": actual_motion_time,
        "send_count": send_count,
        "measured_value": measured,
        "velocity": velocity,
        "unit": unit
    }


# ============================================================
# 6. 擬合 motor_command -> 實際速度
# ============================================================

def fit_axis(rows, axis):
    xs = []
    ys = []

    for r in rows:
        if r["axis"] == axis:
            xs.append(float(r["motor_command"]))
            ys.append(float(r["velocity"]))

    xs = np.array(xs, dtype=float)
    ys = np.array(ys, dtype=float)

    if len(xs) < 2:
        return None

    # 一般線性模型：velocity = k * speed + b
    k, b = np.polyfit(xs, ys, 1)

    # 強制過原點模型：velocity = k0 * speed
    # 對馬達比較合理，speed=0 時速度應該接近 0
    k0 = float(np.sum(xs * ys) / np.sum(xs * xs))

    pred = k * xs + b
    pred0 = k0 * xs

    rmse = float(np.sqrt(np.mean((ys - pred) ** 2)))
    rmse0 = float(np.sqrt(np.mean((ys - pred0) ** 2)))

    return {
        "axis": axis,
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
                "axis",
                "action",
                "motor_command",
                "duration_s",
                "actual_motion_time_s",
                "send_count",
                "measured_value",
                "velocity",
                "unit"
            ]
        )

        writer.writeheader()
        writer.writerows(rows)


def save_result_json(result):
    with RESULT_JSON_PATH.open("w", encoding="utf-8") as f:
        json.dump(result, f, indent=4, ensure_ascii=False)


def print_result(result):
    print("\n===================================================")
    print("校正完成")
    print("===================================================")

    for axis in ["vx", "vy", "wz"]:
        axis_result = result.get(axis)

        if axis_result is None:
            print(f"\n[{axis}] 沒有足夠資料，略過。")
            continue

        k0 = axis_result["k_through_origin"]
        k = axis_result["k"]
        b = axis_result["b"]

        unit = "m/s" if axis in ["vx", "vy"] else "rad/s"

        print(f"\n[{axis}]")
        print(f"建議先用： {axis} = {k0:.6f} * speed    ({unit})")
        print(f"完整線性： {axis} = {k:.6f} * speed + {b:.6f}")
        print(f"RMSE through origin = {axis_result['rmse_through_origin']:.6f}")

    print("\n之後反推 speed：")
    print("speed_x = desired_vx / KX")
    print("speed_y = desired_vy / KY")
    print("speed_z = desired_wz / KZ")

    print(f"\n原始資料：{RAW_CSV_PATH}")
    print(f"校正結果：{RESULT_JSON_PATH}")


# ============================================================
# 7. 主程式
# ============================================================

def main():
    if not MOTION_ALLOWED:
        print("[SAFETY] Calibration not started. Re-run with --real in a clear test area.")
        return
    print("[INFO] action + speed 校正程式 keep-alive 版")
    print("[INFO] 傳送格式完全照你舊程式：")
    print('[INFO] {"action": "...", "speed": ...}')
    print()
    print("[INFO] 這版會在 TEST_DURATION 期間一直重送 action")
    print("[INFO] 解決：只送一次 forward 時，車自己跑一小段就停的問題")
    print()

    rows = []

    sock = connect_motor_server()
    print("[INFO] motor server connected.")

    try:
        stop_motor(sock)

        # 先測一定會動的 vx / wz
        for axis in ["vx", "wz"]:
            print("\n===================================================")
            print(f"開始測 {axis}")
            print("===================================================")

            for speed in TEST_SPEEDS:
                row = run_one_test(sock, axis, speed)
                rows.append(row)

        # vy 不一定有支援，手動決定要不要測
        print("\n===================================================")
        print("vy 橫移測試")
        print("===================================================")
        print("注意：如果 Jetson server 沒有 left/right，vy 會不動。")
        test_vy = input("要測 vy 嗎？輸入 y 才測：").strip().lower()

        if test_vy == "y":
            for speed in TEST_SPEEDS:
                row = run_one_test(sock, "vy", speed)
                rows.append(row)

        stop_motor(sock)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected. Stop motor.")

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

    result = {
        "test_duration_s": TEST_DURATION,
        "test_speeds": TEST_SPEEDS,
        "command_interval_s": COMMAND_INTERVAL,
        "vx": fit_axis(rows, "vx"),
        "vy": fit_axis(rows, "vy"),
        "wz": fit_axis(rows, "wz")
    }

    save_result_json(result)
    print_result(result)


if __name__ == "__main__":
    main()
