import cv2
import os
import time
import json
import socket
import sys
from datetime import datetime

ENABLE_MOTION = "--real" in sys.argv[1:]


# ============================================================
# 基本設定
# ============================================================

JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "yahboom.local")
MOTOR_PORT = 7000

URL_ARM = f"http://{JETSON_IP}:8080/stream?topic=/arm_cam/image_raw"

# 照片存放資料夾
SAVE_DIR = r"C:\Users\user\Downloads\arm_cam_dataset"

# 畫面大小
FRAME_W = 640
FRAME_H = 480

# 鍵盤控制速度
MANUAL_SPEED = 30

# motor server 有 timeout，所以要 keep-alive
COMMAND_INTERVAL = 0.05


# ============================================================
# 建立資料夾
# ============================================================

# ============================================================
# Motor Socket：只連一次，不用 cmd_vel
# ============================================================

def connect_motor_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((JETSON_IP, MOTOR_PORT))
    sock.settimeout(1.0)
    return sock


def send_motor_action(sock, action, speed):
    if not ENABLE_MOTION:
        return
    msg = {
        "action": str(action),
        "speed": int(speed)
    }

    data = (json.dumps(msg) + "\n").encode("utf-8")
    sock.sendall(data)


def stop_motor(sock, repeat=8, interval=0.03):
    for _ in range(repeat):
        try:
            send_motor_action(sock, "stop", 0)
        except Exception:
            pass
        time.sleep(interval)


def keepalive_send(sock, action, speed, last_send_time):
    now = time.time()

    if now - last_send_time >= COMMAND_INTERVAL:
        send_motor_action(sock, action, speed)
        return now

    return last_send_time


# ============================================================
# 存圖函式
# ============================================================

def save_frame(frame):
    now = datetime.now()
    filename = now.strftime("arm_%Y%m%d_%H%M%S_%f")[:-3] + ".jpg"
    save_path = os.path.join(SAVE_DIR, filename)

    ok = cv2.imwrite(save_path, frame)

    if ok:
        print(f"[SAVE] {save_path}")
        return True
    else:
        print("[ERROR] Failed to save image.")
        return False


# ============================================================
# 主程式
# ============================================================

def main():
    os.makedirs(SAVE_DIR, exist_ok=True)
    motor_sock = None
    if ENABLE_MOTION:
        print("[INFO] Connecting motor server...")
        motor_sock = connect_motor_server()
        print("[INFO] motor server connected.")
    else:
        print("[SAFETY] Photo-only mode; add --real to enable keyboard motion.")

    print("[INFO] Opening arm camera stream...")
    cap = cv2.VideoCapture(URL_ARM)

    if not cap.isOpened():
        print("[ERROR] Cannot open arm camera stream.")
        print("[CHECK] 確認 Jetson IP、arm camera stream、網路連線是否正常")
        try:
            stop_motor(motor_sock)
            if motor_sock is not None:
                motor_sock.close()
        except Exception:
            pass
        return

    print("[INFO] Start arm cam manual capture + keyboard control.")
    print(f"[INFO] Save directory: {SAVE_DIR}")
    print("==================================================")
    print("W: forward")
    print("S: backward")
    print("A: turn_left")
    print("D: turn_right")
    print("Q: curve_left")
    print("E: curve_right")
    print("SPACE: stop")
    print("K: save image now")
    print("ESC: quit")
    print("==================================================")

    capture_count = 0

    current_action = "stop"
    current_speed = 0
    last_motor_send_time = 0

    try:
        while True:
            ok, frame = cap.read()

            if not ok or frame is None:
                print("[WARN] Cannot read frame.")
                time.sleep(0.1)
                continue

            frame = cv2.resize(frame, (FRAME_W, FRAME_H))

            now = time.time()

            # motor keep-alive
            try:
                last_motor_send_time = keepalive_send(
                    motor_sock,
                    current_action,
                    current_speed,
                    last_motor_send_time
                )
            except Exception as e:
                print(f"[MOTOR ERROR] keepalive failed: {e}")

            display = frame.copy()

            cv2.putText(
                display,
                "ARM CAM MANUAL CAPTURE + KEYBOARD CONTROL",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 255, 255),
                2
            )

            cv2.putText(
                display,
                f"Saved: {capture_count}",
                (15, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            cv2.putText(
                display,
                f"ACTION: {current_action} | SPEED: {current_speed}",
                (15, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2
            )

            cv2.putText(
                display,
                "W/S/A/D move | Q/E curve | SPACE stop",
                (15, 135),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2
            )

            cv2.putText(
                display,
                "K save image | ESC quit",
                (15, 165),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2
            )

            cv2.imshow("Arm Cam Capture + Keyboard Control", display)

            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                print("[INFO] ESC pressed. Quit.")
                break

            elif key in [ord("w"), ord("W")]:
                current_action = "forward"
                current_speed = MANUAL_SPEED
                send_motor_action(motor_sock, current_action, current_speed)
                last_motor_send_time = time.time()
                print(f"[KEY] forward speed={current_speed}")

            elif key in [ord("s"), ord("S")]:
                current_action = "backward"
                current_speed = MANUAL_SPEED
                send_motor_action(motor_sock, current_action, current_speed)
                last_motor_send_time = time.time()
                print(f"[KEY] backward speed={current_speed}")

            elif key in [ord("a"), ord("A")]:
                current_action = "turn_left"
                current_speed = MANUAL_SPEED
                send_motor_action(motor_sock, current_action, current_speed)
                last_motor_send_time = time.time()
                print(f"[KEY] turn_left speed={current_speed}")

            elif key in [ord("d"), ord("D")]:
                current_action = "turn_right"
                current_speed = MANUAL_SPEED
                send_motor_action(motor_sock, current_action, current_speed)
                last_motor_send_time = time.time()
                print(f"[KEY] turn_right speed={current_speed}")

            elif key in [ord("q"), ord("Q")]:
                current_action = "curve_left"
                current_speed = MANUAL_SPEED
                send_motor_action(motor_sock, current_action, current_speed)
                last_motor_send_time = time.time()
                print(f"[KEY] curve_left speed={current_speed}")

            elif key in [ord("e"), ord("E")]:
                current_action = "curve_right"
                current_speed = MANUAL_SPEED
                send_motor_action(motor_sock, current_action, current_speed)
                last_motor_send_time = time.time()
                print(f"[KEY] curve_right speed={current_speed}")

            elif key == 32:
                current_action = "stop"
                current_speed = 0
                stop_motor(motor_sock)
                last_motor_send_time = time.time()
                print("[KEY] stop")

            elif key in [ord("k"), ord("K")]:
                if save_frame(frame):
                    capture_count += 1
                print("[INFO] Manual capture.")

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected.")

    finally:
        print("[INFO] Stopping motor and closing resources...")

        try:
            stop_motor(motor_sock, repeat=10, interval=0.03)
        except Exception as e:
            print(f"[WARN] stop failed: {e}")

        try:
            motor_sock.close()
        except Exception:
            pass

        try:
            cap.release()
        except Exception:
            pass
        cv2.destroyAllWindows()

        print(f"[INFO] Total saved images: {capture_count}")
        print(f"[INFO] Images saved in: {SAVE_DIR}")
        print("[INFO] Program ended safely.")


if __name__ == "__main__":
    main()
