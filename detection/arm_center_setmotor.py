import cv2
import time
import json
import os
import socket
import sys
import threading
from ultralytics import YOLO

# ========= 1. 模型路徑 =========
YOLO_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v3i.yolov11 (1)\runs\detect\train6\weights\best.pt"

# ========= 2. Jetson 網路設定 =========
JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "yahboom.local")

# ARM camera 串流
TOPIC_ARM = "/arm_cam/image_raw"
URL_ARM = f"http://{JETSON_IP}:8080/stream?topic={TOPIC_ARM}"

# set_motor server
MOTOR_PORT = 7000

# ========= 3. 畫面設定 =========
FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2

# ========= 4. 對中控制參數 =========
# Legacy experiment safety gate: non-stop commands require an explicit --real.
ENABLE_MOTION = "--real" in sys.argv[1:]

# 你目前覺得好用的設定
TURN_SPEED = 30
ANGLE_DEADZONE = 0.05

# 轉向脈衝
TURN_PULSE_TIME = 0.07
STOP_AFTER_TURN_TIME = 0.10

# 一般前進脈衝
FORWARD_SPEED = 30
FORWARD_PULSE_TIME = 0.20
STOP_AFTER_FORWARD_TIME = 0.05

# ========= 5. 盲走補償參數 =========
# 當紙球已經接近 ARM 鏡頭可視極限時，進入 blind push。
# 這兩個值要現場調。
BLIND_START_BOTTOM_RATIO = 0.95
BLIND_START_HEIGHT_RATIO = 0.4

# 盲目前進補償
BLIND_FORWARD_SPEED = 30
BLIND_FORWARD_TIME = 3.0

# ========= 6. YOLO 設定 =========
YOLO_CONF = 0.3

# ========= 7. 狀態 =========
e_stop_active = False

# 狀態機：
# ALIGN：視覺對中與靠近
# BLIND_PUSH：盲目前進一小段
# READY_GRAB：停車，準備夾取
state = "ALIGN"


# ========= 8. 只保留最新畫面的讀取器 =========
class LatestFrameReader:
    """
    背景執行緒一直讀 camera stream。
    主程式每次只拿最新 frame，避免 OpenCV buffer 堆積造成延遲。
    """

    def __init__(self, url, width=640, height=480):
        self.url = url
        self.width = width
        self.height = height

        self.cap = cv2.VideoCapture(url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.latest_frame = None
        self.latest_time = 0.0
        self.running = True
        self.lock = threading.Lock()

        self.thread = threading.Thread(target=self._reader_loop)
        self.thread.daemon = True
        self.thread.start()

    def _reader_loop(self):
        while self.running:
            ret, frame = self.cap.read()

            if ret:
                frame = cv2.resize(frame, (self.width, self.height))

                with self.lock:
                    self.latest_frame = frame
                    self.latest_time = time.time()
            else:
                time.sleep(0.01)

    def read(self):
        with self.lock:
            if self.latest_frame is None:
                return False, None, 0.0

            return True, self.latest_frame.copy(), self.latest_time

    def release(self):
        self.running = False
        time.sleep(0.1)
        self.cap.release()


# ========= 9. TCP 馬達控制 =========
def connect_motor_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((JETSON_IP, MOTOR_PORT))
    return sock


def send_motor_action(sock, action, speed):
    if not ENABLE_MOTION and action != "stop":
        action, speed = "stop", 0
    msg = {
        "action": str(action),
        "speed": int(speed)
    }
    data = (json.dumps(msg) + "\n").encode("utf-8")
    sock.sendall(data)


def stop_motor(sock, repeat=5, interval=0.03):
    try:
        for _ in range(repeat):
            send_motor_action(sock, "stop", 0)
            time.sleep(interval)
    except Exception:
        pass


# ========= 10. YOLO 目標選擇 =========
def select_largest_box(results):
    """
    如果畫面裡有多個偵測框，選面積最大的那個。
    """
    if results.boxes is None or len(results.boxes) == 0:
        return None

    best_box = None
    best_area = -1

    for box in results.boxes:
        x1 = int(box.xyxy[0][0])
        y1 = int(box.xyxy[0][1])
        x2 = int(box.xyxy[0][2])
        y2 = int(box.xyxy[0][3])

        area = max(1, x2 - x1) * max(1, y2 - y1)

        if area > best_area:
            best_area = area
            best_box = box

    return best_box


# ========= 11. 根據目標位置決定動作 =========
def decide_align_action(target_found, goal_angle, box_h_ratio, box_bottom_ratio):
    """
    goal_angle:
        > 0 代表目標在畫面左邊
        < 0 代表目標在畫面右邊
    """

    if not target_found:
        return "stop", 0, "TARGET LOST"

    # 先判斷左右對中
    if goal_angle > ANGLE_DEADZONE:
        return "turn_left", TURN_SPEED, "ALIGN: TURN LEFT"

    elif goal_angle < -ANGLE_DEADZONE:
        return "turn_right", TURN_SPEED, "ALIGN: TURN RIGHT"

    # 已經對中，判斷是否到 ARM 可視極限附近
    if (
        box_bottom_ratio >= BLIND_START_BOTTOM_RATIO
        and box_h_ratio >= BLIND_START_HEIGHT_RATIO
    ):
        return "blind_push", BLIND_FORWARD_SPEED, "ENTER BLIND PUSH"

    # 對中了，但還不夠近：短脈衝前進
    return "forward", FORWARD_SPEED, "CENTERED: FORWARD PULSE"


# ========= 12. 脈衝控制 =========
def send_pulse_control(sock, action, speed):
    """
    有影像延遲時不能連續長時間轉或前進。
    這裡用短脈衝控制：動一下，停一下，再重新看畫面。
    """

    if action in ["turn_left", "turn_right"]:
        send_motor_action(sock, action, speed)
        time.sleep(TURN_PULSE_TIME)

        send_motor_action(sock, "stop", 0)
        time.sleep(STOP_AFTER_TURN_TIME)

    elif action == "forward":
        send_motor_action(sock, "forward", speed)
        time.sleep(FORWARD_PULSE_TIME)

        send_motor_action(sock, "stop", 0)
        time.sleep(STOP_AFTER_FORWARD_TIME)

    else:
        send_motor_action(sock, "stop", 0)


def execute_blind_push(sock):
    """
    紙球進入 ARM 可視極限後，不再依賴視覺。
    直接往前補一小段，讓紙球進入夾爪可達範圍。
    """
    print("[BLIND_PUSH] Start blind forward compensation")

    send_motor_action(sock, "forward", BLIND_FORWARD_SPEED)
    time.sleep(BLIND_FORWARD_TIME)

    send_motor_action(sock, "stop", 0)
    time.sleep(0.1)

    print("[BLIND_PUSH] Done. READY_GRAB")


# ========= 13. 畫面顯示 =========
def draw_dashboard(
    frame,
    target_found,
    goal_angle,
    motor_action,
    motor_speed,
    status_text,
    frame_age,
    inference_time,
    box_h_ratio,
    box_bottom_ratio,
    current_state
):
    # 中心線
    cv2.line(frame, (CENTER_X, 0), (CENTER_X, FRAME_H), (255, 255, 255), 1)

    # deadzone 區域
    deadzone_px = int(CENTER_X * ANGLE_DEADZONE)
    left_bound = CENTER_X - deadzone_px
    right_bound = CENTER_X + deadzone_px

    cv2.line(frame, (left_bound, 0), (left_bound, FRAME_H), (0, 255, 255), 1)
    cv2.line(frame, (right_bound, 0), (right_bound, FRAME_H), (0, 255, 255), 1)

    # blind push 觸發線
    blind_y = int(FRAME_H * BLIND_START_BOTTOM_RATIO)
    cv2.line(frame, (0, blind_y), (FRAME_W, blind_y), (0, 0, 255), 2)

    # 上方面板
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (635, 215), (0, 0, 0), -1)
    frame[:] = cv2.addWeighted(overlay, 0.45, frame, 0.55, 0)

    cv2.putText(
        frame,
        "ARM CENTER + BLIND PUSH MODE",
        (25, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.68,
        (0, 255, 255),
        2
    )

    target_text = "FOUND" if target_found else "LOST"

    cv2.putText(
        frame,
        f"STATE: {current_state}",
        (25, 72),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (0, 255, 255),
        2
    )

    cv2.putText(
        frame,
        f"TARGET: {target_text}",
        (25, 102),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (0, 255, 0) if target_found else (0, 0, 255),
        2
    )

    cv2.putText(
        frame,
        f"ANGLE: {goal_angle:.3f}",
        (25, 132),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (255, 255, 255),
        2
    )

    cv2.putText(
        frame,
        f"MOTOR: {motor_action} | SPEED: {motor_speed}",
        (250, 102),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (0, 255, 255),
        2
    )

    cv2.putText(
        frame,
        f"STATUS: {status_text}",
        (250, 132),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (255, 255, 255),
        2
    )

    cv2.putText(
        frame,
        f"box_h: {box_h_ratio:.3f} | bottom: {box_bottom_ratio:.3f}",
        (25, 162),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1
    )

    cv2.putText(
        frame,
        f"frame_age: {frame_age:.3f}s | yolo: {inference_time:.3f}s",
        (25, 187),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1
    )

    cv2.putText(
        frame,
        "Q: Quit | S: E-STOP | R: Reset to ALIGN",
        (25, 210),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        1
    )


# ========= 14. 主程式 =========
def main():
    global e_stop_active
    global state

    if not ENABLE_MOTION:
        print("[SAFETY] Vision-only mode; add --real to permit motor motion.")

    print("[INFO] 載入 YOLO...")
    yolo_model = YOLO(YOLO_MODEL_PATH)

    print(f"[INFO] 連線至 Jetson motor server {JETSON_IP}:{MOTOR_PORT} ...")
    motor_sock = connect_motor_server()
    print("[INFO] motor server connected.")

    print("[INFO] 開啟 ARM camera stream，用 LatestFrameReader 丟掉舊畫面...")
    cap_arm = LatestFrameReader(URL_ARM, FRAME_W, FRAME_H)

    target_dt = 0.05

    # 避免同一張 frame 被處理太多次
    last_processed_frame_time = 0.0

    try:
        while True:
            loop_start = time.time()

            ret, frame, frame_time = cap_arm.read()

            if not ret:
                print("[WARN] ARM camera 還沒有畫面，停車...")
                stop_motor(motor_sock, repeat=1, interval=0.01)
                time.sleep(0.1)
                continue

            # 如果拿到的是同一張舊 frame，就不要重複下判斷
            if frame_time == last_processed_frame_time:
                time.sleep(0.01)
                continue

            last_processed_frame_time = frame_time
            frame_age = time.time() - frame_time

            # 如果 frame 太舊，先停車，不根據舊畫面動作
            if frame_age > 0.8:
                print(f"[WARN] frame too old: {frame_age:.2f}s, stop")
                stop_motor(motor_sock, repeat=1, interval=0.01)

                cv2.putText(
                    frame,
                    f"FRAME TOO OLD: {frame_age:.2f}s",
                    (30, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 0, 255),
                    2
                )

                cv2.imshow("ARM Center + Blind Push", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("s"):
                    e_stop_active = not e_stop_active
                    stop_motor(motor_sock)
                elif key == ord("r"):
                    state = "ALIGN"
                    print("[RESET] state -> ALIGN")

                continue

            # ========= READY_GRAB 狀態 =========
            if state == "READY_GRAB":
                stop_motor(motor_sock, repeat=1, interval=0.01)

                display = frame.copy()

                cv2.putText(
                    display,
                    "READY TO GRAB",
                    (140, 240),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.4,
                    (0, 255, 0),
                    4
                )

                cv2.putText(
                    display,
                    "Press R to reset ALIGN | Q to quit",
                    (100, 290),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2
                )

                cv2.imshow("ARM Center + Blind Push", display)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("r"):
                    state = "ALIGN"
                    print("[RESET] state -> ALIGN")
                elif key == ord("s"):
                    e_stop_active = not e_stop_active
                    stop_motor(motor_sock)

                continue

            # ========= YOLO 推論 =========
            yolo_start = time.time()

            results = yolo_model.predict(
                source=frame,
                conf=YOLO_CONF,
                imgsz=640,
                verbose=False
            )[0]

            inference_time = time.time() - yolo_start

            annotated = results.plot()

            target_found = False
            goal_angle = 0.0
            motor_action = "stop"
            motor_speed = 0
            status_text = "NO TARGET"

            box_h_ratio = 0.0
            box_bottom_ratio = 0.0

            box = select_largest_box(results)

            if box is not None:
                target_found = True

                x1 = int(box.xyxy[0][0])
                y1 = int(box.xyxy[0][1])
                x2 = int(box.xyxy[0][2])
                y2 = int(box.xyxy[0][3])

                box_w = max(1, x2 - x1)
                box_h = max(1, y2 - y1)

                box_h_ratio = box_h / FRAME_H
                box_bottom_ratio = y2 / FRAME_H

                cx_box = int((x1 + x2) / 2)
                cy_box = int((y1 + y2) / 2)

                # 目標偏左為正，偏右為負
                goal_angle = (CENTER_X - cx_box) / CENTER_X

                # 畫目標中心與偏移線
                cv2.circle(annotated, (cx_box, cy_box), 6, (0, 255, 255), -1)
                cv2.line(annotated, (CENTER_X, cy_box), (cx_box, cy_box), (0, 255, 255), 2)

                motor_action, motor_speed, status_text = decide_align_action(
                    target_found=True,
                    goal_angle=goal_angle,
                    box_h_ratio=box_h_ratio,
                    box_bottom_ratio=box_bottom_ratio
                )

            else:
                motor_action, motor_speed, status_text = decide_align_action(
                    target_found=False,
                    goal_angle=0.0,
                    box_h_ratio=0.0,
                    box_bottom_ratio=0.0
                )

            if e_stop_active or not ENABLE_MOTION:
                motor_action = "stop"
                motor_speed = 0
                status_text = "E-STOP" if e_stop_active else "MOTION DISABLED"

            # ========= 狀態機控制 =========
            if state == "ALIGN":
                if motor_action == "blind_push" and not e_stop_active and ENABLE_MOTION:
                    state = "BLIND_PUSH"
                    print("[STATE] ALIGN -> BLIND_PUSH")

                    execute_blind_push(motor_sock)

                    state = "READY_GRAB"
                    print("[STATE] BLIND_PUSH -> READY_GRAB")

                    motor_action = "stop"
                    motor_speed = 0
                    status_text = "READY TO GRAB"

                else:
                    send_pulse_control(motor_sock, motor_action, motor_speed)

            else:
                stop_motor(motor_sock, repeat=1, interval=0.01)

            draw_dashboard(
                frame=annotated,
                target_found=target_found,
                goal_angle=goal_angle,
                motor_action=motor_action,
                motor_speed=motor_speed,
                status_text=status_text,
                frame_age=frame_age,
                inference_time=inference_time,
                box_h_ratio=box_h_ratio,
                box_bottom_ratio=box_bottom_ratio,
                current_state=state
            )

            print(
                f"State: {state} | "
                f"Target: {target_found} | "
                f"Angle: {goal_angle:.3f} | "
                f"box_h: {box_h_ratio:.3f} | "
                f"bottom: {box_bottom_ratio:.3f} | "
                f"Motor: {motor_action} | "
                f"Speed: {motor_speed} | "
                f"frame_age: {frame_age:.3f}s | "
                f"YOLO: {inference_time:.3f}s | "
                f"Status: {status_text}"
            )

            cv2.imshow("ARM Center + Blind Push", annotated)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                print("[INFO] 按下 Q，結束程式...")
                break

            elif key == ord("s"):
                e_stop_active = not e_stop_active

                if e_stop_active:
                    print("[E-STOP] 急停啟動")
                    stop_motor(motor_sock)
                else:
                    print("[E-STOP] 急停解除")

            elif key == ord("r"):
                state = "ALIGN"
                print("[RESET] state -> ALIGN")

            loop_time = time.time() - loop_start

            if loop_time < target_dt:
                time.sleep(target_dt - loop_time)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected.")

    finally:
        print("[INFO] 停車並關閉資源...")

        try:
            stop_motor(motor_sock)
            motor_sock.close()
        except Exception as e:
            print(f"[WARN] motor socket close failed: {e}")

        cap_arm.release()
        cv2.destroyAllWindows()

        print("[INFO] 程式已結束")


if __name__ == "__main__":
    main()
