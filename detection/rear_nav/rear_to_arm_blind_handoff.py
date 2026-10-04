import cv2
import time
import json
import os
import socket
import sys
import threading
import math
import numpy as np
from ultralytics import YOLO


# ============================================================
# 1. 模型路徑
# ============================================================

YOLO_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v8i.yolov11(0517)\runs\detect\train\weights\best.pt"


# ============================================================
# 2. Jetson 網路設定
# ============================================================

JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "yahboom.local")

TOPIC_REAR = "/back_cam/image_raw"
TOPIC_ARM = "/arm_cam/image_raw"

URL_REAR = f"http://{JETSON_IP}:8080/stream?topic={TOPIC_REAR}"
URL_ARM = f"http://{JETSON_IP}:8080/stream?topic={TOPIC_ARM}"

MOTOR_PORT = 7000


# ============================================================
# 3. 畫面設定
# ============================================================

FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2


# ============================================================
# 4. 後鏡頭距離校正
# ============================================================

THETA_REAR = 14.8
H_REAR = 0.50

# 後鏡頭到車頭前緣距離，單位 m
REAR_CAM_TO_FRONT_M = 0.20

FY_REAR = 956.0294
CY_REAR = 221.3649
FX_REAR = 957.6253
CX_REAR = 320.0


# ============================================================
# 5. ARM 鏡頭距離校正
# ============================================================

# 你剛校正出來的 ARM camera theta
THETA_ARM = 27.3

# ARM camera 離地高度，單位 m
# 如果你實際量到不是 0.35，要改這裡
H_ARM = 0.35

FY_ARM = 956.0294
CY_ARM = 221.3649
FX_ARM = 957.6253
CX_ARM = 320.0


# ============================================================
# 6. 車速校正
# ============================================================

# vx = 0.006995 * speed
KX = 0.006995

# wz ≈ 0.02384 * speed
KZ = 0.02384


# ============================================================
# 7. 整體控制設定
# ============================================================

# Legacy experiment safety gate: non-stop commands require an explicit --real.
ENABLE_MOTION = "--real" in sys.argv[1:]

YOLO_CONF = 0.3
IMG_SIZE = 640

# motor server 有 timeout，所以要 keep-alive
COMMAND_INTERVAL = 0.05

# REAR / ARM 都每 0.5 秒重新判斷一次
REAR_DECISION_INTERVAL = 0.5
ARM_DECISION_INTERVAL = 0.5

MAX_FRAME_AGE = 0.8

STOP_WHEN_TARGET_LOST = True


# ============================================================
# 8. REAR camera 控制參數
# ============================================================

FAR_VX_MPS = 0.22
MID_VX_MPS = 0.18
NEAR_VX_MPS = 0.13

MIN_SPEED = 28
MAX_SPEED = 48

# 這些距離都是「紙球到車頭距離」
FAR_DIST_M = 1.60
MID_DIST_M = 1.10
NEAR_DIST_M = 0.90

# 車頭距離小於這個值，進入 rear blind
REAR_BLIND_START_DIST_M = 0.90

# 希望 rear blind 後，紙球大概到車頭這個距離
ARM_EXPECTED_CAPTURE_DIST_M = 0.70

MIN_REAR_BLIND_DIST_M = 0.12
MAX_REAR_BLIND_DIST_M = 0.38

REAR_BLIND_VX_MPS = 0.13

# offset_x_m：右正左負
OFFSET_FORWARD_DEADZONE_M = 0.06
OFFSET_TURN_THRESHOLD_M = 0.28
REAR_BLIND_MAX_OFFSET_M = 0.13

TURN_SPEED_RATIO = 0.85


# ============================================================
# 9. ARM camera 控制參數
# ============================================================

# ARM 段希望慢一點
ARM_FAR_VX_MPS = 0.10
ARM_MID_VX_MPS = 0.07
ARM_NEAR_VX_MPS = 0.045

ARM_MIN_SPEED = 24
ARM_MAX_SPEED = 38

# ARM 距離分段，單位 m
ARM_FAR_DIST_M = 0.70
ARM_MID_DIST_M = 0.45
ARM_NEAR_DIST_M = 0.30

# ARM 距離小於這個值，開始最後 blind push
ARM_BLIND_START_DIST_M = 0.24

# ARM 左右偏移門檻，單位 m
ARM_OFFSET_FORWARD_DEADZONE_M = 0.035
ARM_OFFSET_TURN_THRESHOLD_M = 0.12

# 用 offset / dist 算角度，再算 desired_wz
ARM_KP_WZ = 0.5
ARM_MAX_WZ = 0.35
ARM_MIN_TURN_SPEED = 24

# 最後 blind push
ARM_BLIND_FORWARD_SPEED = 30
ARM_BLIND_FORWARD_TIME = 3.0


# ============================================================
# 10. 狀態
# ============================================================

e_stop_active = False

# 狀態：
# REAR_FOLLOW
# REAR_BLIND
# ARM_ALIGN
# ARM_BLIND_PUSH
# READY_GRAB
state = "REAR_FOLLOW"

last_action = "stop"
last_speed = 0
last_command_time = 0.0

last_rear_decision_time = 0.0
last_arm_decision_time = 0.0

last_rear_info = {
    "target_found": False,
    "dist_front_m": -1.0,
    "dist_rear_m": -1.0,
    "offset_x_m": 0.0,
    "goal_angle": 0.0,
    "box_h_ratio": 0.0,
    "box_bottom_ratio": 0.0,
    "action": "stop",
    "speed": 0,
    "status_text": "INIT",
    "distance_state": "INIT",
    "inference_time": 0.0,
}

last_arm_info = {
    "target_found": False,
    "dist_arm_m": -1.0,
    "offset_x_m": 0.0,
    "goal_angle": 0.0,
    "desired_wz": 0.0,
    "box_h_ratio": 0.0,
    "box_bottom_ratio": 0.0,
    "action": "stop",
    "speed": 0,
    "status_text": "INIT",
    "distance_state": "INIT",
    "inference_time": 0.0,
}

last_rear_target = {
    "dist_front_m": None,
    "dist_rear_m": None,
    "offset_x_m": 0.0,
    "goal_angle": 0.0,
    "time": 0.0,
    "blind_dist_m": 0.0,
    "blind_time_s": 0.0,
}


# ============================================================
# 11. 只保留最新畫面
# ============================================================

class LatestFrameReader:
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


# ============================================================
# 12. motor socket
# ============================================================

def connect_motor_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((JETSON_IP, MOTOR_PORT))
    sock.settimeout(1.0)
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


def stop_motor(sock, repeat=8, interval=0.03):
    for _ in range(repeat):
        try:
            send_motor_action(sock, "stop", 0)
        except Exception:
            pass
        time.sleep(interval)


def keepalive_send(sock, action, speed):
    global last_command_time

    now = time.time()

    if now - last_command_time >= COMMAND_INTERVAL:
        send_motor_action(sock, action, speed)
        last_command_time = now


# ============================================================
# 13. YOLO box
# ============================================================

def select_largest_box(results):
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


# ============================================================
# 14. 距離與偏移公式
# ============================================================

def estimate_ground_distance(y2, theta_deg, h_m, fy, cy):
    """
    用相機 theta + YOLO box 底部 y2 算地面前方距離。
    """
    theta = math.radians(theta_deg)
    alpha = math.atan((y2 - cy) / fy)

    total_angle = theta + alpha

    if total_angle <= 0:
        return -1.0

    return h_m / math.tan(total_angle)


def rear_dist_to_front_dist(dist_rear_m):
    if dist_rear_m <= 0:
        return -1.0

    return max(0.0, dist_rear_m - REAR_CAM_TO_FRONT_M)


def estimate_offset_x(cx_box, dist_m, fx, cx_cam):
    """
    offset_x > 0：目標在右邊
    offset_x < 0：目標在左邊
    """
    if dist_m <= 0:
        return 0.0

    return dist_m * (cx_box - cx_cam) / fx


# ============================================================
# 15. speed 轉換
# ============================================================

def speed_from_vx(vx_mps, min_speed=MIN_SPEED, max_speed=MAX_SPEED):
    if KX <= 1e-9:
        return min_speed

    speed = vx_mps / KX
    return int(round(np.clip(speed, min_speed, max_speed)))


def speed_from_wz(wz_rad_s, min_speed=MIN_SPEED, max_speed=MAX_SPEED):
    if KZ <= 1e-9:
        return min_speed

    speed = abs(wz_rad_s) / KZ
    speed = int(round(np.clip(speed, min_speed, max_speed)))
    return speed


# ============================================================
# 16. REAR 決策
# ============================================================

def get_rear_distance_state(dist_front_m):
    if dist_front_m <= 0:
        return "INVALID"

    if dist_front_m <= REAR_BLIND_START_DIST_M:
        return "BLIND_READY"

    if dist_front_m <= NEAR_DIST_M:
        return "NEAR"

    if dist_front_m <= MID_DIST_M:
        return "MID"

    if dist_front_m <= FAR_DIST_M:
        return "FAR"

    return "VERY_FAR"


def choose_rear_base_speed(distance_state):
    if distance_state in ["VERY_FAR", "FAR"]:
        return speed_from_vx(FAR_VX_MPS)

    if distance_state == "MID":
        return speed_from_vx(MID_VX_MPS)

    if distance_state in ["NEAR", "BLIND_READY"]:
        return speed_from_vx(NEAR_VX_MPS)

    return 0


def compute_rear_blind_plan(dist_front_m):
    raw_blind_dist = dist_front_m - ARM_EXPECTED_CAPTURE_DIST_M

    blind_dist_m = float(np.clip(
        raw_blind_dist,
        MIN_REAR_BLIND_DIST_M,
        MAX_REAR_BLIND_DIST_M
    ))

    if REAR_BLIND_VX_MPS <= 1e-6:
        blind_time_s = 1.0
    else:
        blind_time_s = blind_dist_m / REAR_BLIND_VX_MPS

    blind_time_s = float(np.clip(blind_time_s, 0.6, 3.2))

    return blind_dist_m, blind_time_s


def decide_rear_action_by_offset(target_found, offset_x_m, dist_front_m):
    if not target_found:
        return "stop", 0, "TARGET LOST", "NO_TARGET"

    distance_state = get_rear_distance_state(dist_front_m)
    base_speed = choose_rear_base_speed(distance_state)

    if distance_state == "INVALID":
        return "stop", 0, "INVALID DIST", distance_state

    if distance_state == "BLIND_READY":
        if abs(offset_x_m) <= REAR_BLIND_MAX_OFFSET_M:
            return "stop", 0, "ENTER REAR BLIND", distance_state

        turn_speed = int(round(base_speed * TURN_SPEED_RATIO))
        turn_speed = int(np.clip(turn_speed, MIN_SPEED, MAX_SPEED))

        if offset_x_m < 0:
            return "turn_left", turn_speed, "BLIND NEAR BUT LEFT OFFSET", distance_state
        else:
            return "turn_right", turn_speed, "BLIND NEAR BUT RIGHT OFFSET", distance_state

    if abs(offset_x_m) <= OFFSET_FORWARD_DEADZONE_M:
        return "forward", base_speed, "CENTER BY OFFSET: FORWARD", distance_state

    if abs(offset_x_m) >= OFFSET_TURN_THRESHOLD_M:
        turn_speed = int(round(base_speed * TURN_SPEED_RATIO))
        turn_speed = int(np.clip(turn_speed, MIN_SPEED, MAX_SPEED))

        if offset_x_m < 0:
            return "turn_left", turn_speed, "LARGE LEFT OFFSET: TURN LEFT", distance_state
        else:
            return "turn_right", turn_speed, "LARGE RIGHT OFFSET: TURN RIGHT", distance_state

    if offset_x_m < 0:
        return "curve_left", base_speed, "LEFT OFFSET: CURVE LEFT", distance_state
    else:
        return "curve_right", base_speed, "RIGHT OFFSET: CURVE RIGHT", distance_state


# ============================================================
# 17. ARM 決策：用距離 + offset 算 desired_wz
# ============================================================

def get_arm_distance_state(dist_arm_m):
    if dist_arm_m <= 0:
        return "INVALID"

    if dist_arm_m <= ARM_BLIND_START_DIST_M:
        return "ARM_BLIND_READY"

    if dist_arm_m <= ARM_NEAR_DIST_M:
        return "ARM_NEAR"

    if dist_arm_m <= ARM_MID_DIST_M:
        return "ARM_MID"

    if dist_arm_m <= ARM_FAR_DIST_M:
        return "ARM_FAR"

    return "ARM_VERY_FAR"


def choose_arm_forward_speed(distance_state):
    if distance_state in ["ARM_VERY_FAR", "ARM_FAR"]:
        return speed_from_vx(ARM_FAR_VX_MPS, ARM_MIN_SPEED, ARM_MAX_SPEED)

    if distance_state == "ARM_MID":
        return speed_from_vx(ARM_MID_VX_MPS, ARM_MIN_SPEED, ARM_MAX_SPEED)

    if distance_state in ["ARM_NEAR", "ARM_BLIND_READY"]:
        return speed_from_vx(ARM_NEAR_VX_MPS, ARM_MIN_SPEED, ARM_MAX_SPEED)

    return 0


def decide_arm_action_by_distance(target_found, offset_x_m, dist_arm_m):
    """
    ARM camera 使用：
    - dist_arm_m 算距離
    - offset_x_m / dist_arm_m 算角度誤差
    - desired_wz = KP * atan(offset / dist)
    - 用 KZ 反推 turn speed
    """

    if not target_found:
        return "stop", 0, "ARM TARGET LOST", "NO_TARGET", 0.0

    distance_state = get_arm_distance_state(dist_arm_m)

    if distance_state == "INVALID":
        return "stop", 0, "ARM INVALID DIST", distance_state, 0.0

    # 已經很近，進入最後 blind push
    if distance_state == "ARM_BLIND_READY":
        if abs(offset_x_m) <= ARM_OFFSET_TURN_THRESHOLD_M:
            return "arm_blind_push", 0, "ENTER ARM BLIND PUSH", distance_state, 0.0

    # 算角度誤差
    if dist_arm_m > 0:
        angle_error = math.atan2(offset_x_m, dist_arm_m)
    else:
        angle_error = 0.0

    desired_wz = ARM_KP_WZ * angle_error
    desired_wz = float(np.clip(desired_wz, -ARM_MAX_WZ, ARM_MAX_WZ))

    # 目標在中心附近，前進
    if abs(offset_x_m) <= ARM_OFFSET_FORWARD_DEADZONE_M:
        speed = choose_arm_forward_speed(distance_state)
        return "forward", speed, "ARM CENTER: FORWARD", distance_state, desired_wz

    # 偏移明顯，原地轉向修正
    turn_speed = speed_from_wz(
        desired_wz,
        min_speed=ARM_MIN_TURN_SPEED,
        max_speed=ARM_MAX_SPEED
    )

    if offset_x_m < 0:
        return "turn_left", turn_speed, "ARM OFFSET LEFT: TURN LEFT", distance_state, desired_wz
    else:
        return "turn_right", turn_speed, "ARM OFFSET RIGHT: TURN RIGHT", distance_state, desired_wz


# ============================================================
# 18. blind 動作
# ============================================================

def execute_rear_blind(sock, blind_time_s):
    global last_action
    global last_speed
    global last_command_time

    speed = speed_from_vx(REAR_BLIND_VX_MPS)

    print("\n[REAR_BLIND] Start blind forward")
    print(f"[REAR_BLIND] speed={speed}, time={blind_time_s:.2f}s")

    stop_motor(sock, repeat=3, interval=0.03)
    time.sleep(0.1)

    t0 = time.time()
    last_command_time = 0.0

    while True:
        if e_stop_active:
            break

        elapsed = time.time() - t0

        if elapsed >= blind_time_s:
            break

        keepalive_send(sock, "forward", speed)
        time.sleep(0.005)

    stop_motor(sock, repeat=8, interval=0.03)

    last_action = "stop"
    last_speed = 0

    print("[REAR_BLIND] Done. Switch to ARM_ALIGN.")


def execute_arm_blind_push(sock):
    print("\n[ARM_BLIND_PUSH] Start")
    stop_motor(sock, repeat=3, interval=0.03)
    time.sleep(0.1)

    t0 = time.time()

    while True:
        if e_stop_active:
            break

        if time.time() - t0 >= ARM_BLIND_FORWARD_TIME:
            break

        send_motor_action(sock, "forward", ARM_BLIND_FORWARD_SPEED)
        time.sleep(COMMAND_INTERVAL)

    stop_motor(sock, repeat=8, interval=0.03)
    print("[ARM_BLIND_PUSH] Done. READY_GRAB")


# ============================================================
# 19. Dashboard
# ============================================================

def draw_common_overlay(
    frame,
    title,
    state_text,
    target_found,
    goal_angle,
    dist_main_m,
    dist_second_m,
    offset_x_m,
    desired_wz,
    box_h_ratio,
    box_bottom_ratio,
    action,
    speed,
    status_text,
    frame_age,
    inference_time,
    next_decision_remain
):
    cv2.line(frame, (CENTER_X, 0), (CENTER_X, FRAME_H), (255, 255, 255), 1)

    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (635, 315), (0, 0, 0), -1)
    frame[:] = cv2.addWeighted(overlay, 0.45, frame, 0.55, 0)

    cv2.putText(frame, title, (22, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2)

    target_text = "FOUND" if target_found else "LOST"
    color = (0, 255, 0) if target_found else (0, 0, 255)

    cv2.putText(frame, f"STATE: {state_text} | TARGET: {target_text}",
                (22, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.56, color, 2)

    main_text = f"{dist_main_m:.2f} m" if dist_main_m is not None and dist_main_m > 0 else "N/A"
    second_text = f"{dist_second_m:.2f} m" if dist_second_m is not None and dist_second_m > 0 else "N/A"

    cv2.putText(frame, f"main_dist: {main_text} | second_dist: {second_text}",
                (22, 102), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1)

    cv2.putText(frame, f"offset_x: {offset_x_m:.2f} m | angle: {goal_angle:.3f} | wz: {desired_wz:.3f}",
                (22, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1)

    cv2.putText(frame, f"box_h: {box_h_ratio:.3f} | bottom: {box_bottom_ratio:.3f}",
                (22, 158), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)

    cv2.putText(frame, f"action: {action} | speed: {speed}",
                (22, 188), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2)

    cv2.putText(frame, f"status: {status_text}",
                (22, 218), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)

    cv2.putText(frame, f"age: {frame_age:.3f}s | yolo: {inference_time:.3f}s",
                (22, 246), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1)

    cv2.putText(frame, f"rear_dt: {REAR_DECISION_INTERVAL:.1f}s | arm_dt: {ARM_DECISION_INTERVAL:.1f}s | next: {next_decision_remain:.2f}s",
                (22, 272), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1)

    cv2.putText(frame, f"THETA_REAR: {THETA_REAR:.1f} | THETA_ARM: {THETA_ARM:.1f}",
                (22, 298), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1)

    cv2.putText(frame, "Q: Quit | S: E-STOP | R: Reset to REAR_FOLLOW",
                (22, 462), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (255, 255, 255), 2)


def draw_object_distance_label(frame, x1, y1, primary_dist_m, secondary_dist_m, offset_x_m, label_prefix):
    if primary_dist_m > 0:
        cv2.putText(
            frame,
            f"{label_prefix}: {primary_dist_m * 100:.1f} cm",
            (x1, max(25, y1 - 45)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2
        )

        if secondary_dist_m is not None and secondary_dist_m > 0:
            cv2.putText(
                frame,
                f"Raw: {secondary_dist_m * 100:.1f} cm",
                (x1, max(50, y1 - 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1
            )

        if offset_x_m > 0:
            side_text = "Right"
        elif offset_x_m < 0:
            side_text = "Left"
        else:
            side_text = "Center"

        cv2.putText(
            frame,
            f"{side_text}: {abs(offset_x_m) * 100:.1f} cm",
            (x1, min(FRAME_H - 10, y1 + 25)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 100, 255),
            2
        )
    else:
        cv2.putText(
            frame,
            "Dist invalid",
            (x1, max(25, y1 - 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2
        )


# ============================================================
# 20. 主程式
# ============================================================

def main():
    global state
    global e_stop_active
    global last_action
    global last_speed

    if not ENABLE_MOTION:
        print("[SAFETY] Vision-only mode; add --real to permit motor motion.")
    global last_command_time
    global last_rear_decision_time
    global last_arm_decision_time
    global last_rear_info
    global last_arm_info
    global last_rear_target

    print("[INFO] Loading YOLO...")
    yolo_model = YOLO(YOLO_MODEL_PATH)

    print(f"[INFO] Connecting motor server {JETSON_IP}:{MOTOR_PORT}...")
    motor_sock = connect_motor_server()
    print("[INFO] motor server connected.")

    print("[INFO] Opening cameras...")
    cap_rear = LatestFrameReader(URL_REAR, FRAME_W, FRAME_H)
    cap_arm = LatestFrameReader(URL_ARM, FRAME_W, FRAME_H)

    try:
        while True:
            if state in ["REAR_FOLLOW", "REAR_BLIND"]:
                active_cap = cap_rear
                active_name = "REAR"
            else:
                active_cap = cap_arm
                active_name = "ARM"

            ret, frame, frame_time = active_cap.read()

            if not ret:
                print(f"[WARN] {active_name} camera no frame.")
                stop_motor(motor_sock, repeat=1, interval=0.01)
                time.sleep(0.1)
                continue

            frame_age = time.time() - frame_time

            if frame_age > MAX_FRAME_AGE:
                print(f"[WARN] {active_name} frame too old: {frame_age:.2f}s")
                last_action = "stop"
                last_speed = 0
                stop_motor(motor_sock, repeat=1, interval=0.01)
                time.sleep(0.03)
                continue

            # ==================================================
            # REAR_BLIND
            # ==================================================
            if state == "REAR_BLIND":
                execute_rear_blind(motor_sock, last_rear_target["blind_time_s"])
                state = "ARM_ALIGN"
                last_arm_decision_time = 0.0
                continue

            # ==================================================
            # ARM_BLIND_PUSH
            # ==================================================
            if state == "ARM_BLIND_PUSH":
                execute_arm_blind_push(motor_sock)
                state = "READY_GRAB"
                continue

            # ==================================================
            # READY_GRAB
            # ==================================================
            if state == "READY_GRAB":
                stop_motor(motor_sock, repeat=1, interval=0.01)

                display = frame.copy()
                cv2.putText(display, "READY TO GRAB", (145, 235),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.35, (0, 255, 0), 4)
                cv2.putText(display, "Press R to restart rear follow | Q to quit",
                            (80, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255, 255, 255), 2)

                cv2.imshow("Rear-to-Arm Distance-Wz Handoff", display)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("r"):
                    state = "REAR_FOLLOW"
                    last_rear_decision_time = 0.0
                    print("[RESET] state -> REAR_FOLLOW")
                elif key == ord("s"):
                    e_stop_active = not e_stop_active
                    stop_motor(motor_sock)

                continue

            # ==================================================
            # REAR_FOLLOW：每 0.5 秒判斷一次
            # ==================================================
            if state == "REAR_FOLLOW":
                now = time.time()
                need_decision = (now - last_rear_decision_time) >= REAR_DECISION_INTERVAL

                if need_decision:
                    yolo_start = time.time()
                    results = yolo_model.predict(source=frame, conf=YOLO_CONF, imgsz=IMG_SIZE, verbose=False)[0]
                    inference_time = time.time() - yolo_start

                    annotated = results.plot()
                    box = select_largest_box(results)

                    target_found = False
                    goal_angle = 0.0
                    dist_rear_m = -1.0
                    dist_front_m = -1.0
                    offset_x_m = 0.0
                    box_h_ratio = 0.0
                    box_bottom_ratio = 0.0
                    action = "stop"
                    speed = 0
                    status_text = "NO TARGET"
                    distance_state = "NO_TARGET"

                    if box is not None:
                        target_found = True

                        x1 = int(box.xyxy[0][0])
                        y1 = int(box.xyxy[0][1])
                        x2 = int(box.xyxy[0][2])
                        y2 = int(box.xyxy[0][3])

                        box_w = max(1, x2 - x1)
                        box_h = max(1, y2 - y1)

                        cx_box = int((x1 + x2) / 2)
                        cy_box = int((y1 + y2) / 2)

                        box_h_ratio = box_h / FRAME_H
                        box_bottom_ratio = y2 / FRAME_H
                        goal_angle = (CENTER_X - cx_box) / CENTER_X

                        dist_rear_m = estimate_ground_distance(y2, THETA_REAR, H_REAR, FY_REAR, CY_REAR)
                        dist_front_m = rear_dist_to_front_dist(dist_rear_m)
                        offset_x_m = estimate_offset_x(cx_box, dist_rear_m, FX_REAR, CX_REAR)

                        cv2.circle(annotated, (cx_box, cy_box), 6, (0, 255, 255), -1)
                        cv2.line(annotated, (CENTER_X, cy_box), (cx_box, cy_box), (0, 255, 255), 2)

                        draw_object_distance_label(
                            frame=annotated,
                            x1=x1,
                            y1=y1,
                            primary_dist_m=dist_front_m,
                            secondary_dist_m=dist_rear_m,
                            offset_x_m=offset_x_m,
                            label_prefix="Front"
                        )

                        action, speed, status_text, distance_state = decide_rear_action_by_offset(
                            target_found=True,
                            offset_x_m=offset_x_m,
                            dist_front_m=dist_front_m
                        )

                        if distance_state == "BLIND_READY" and abs(offset_x_m) <= REAR_BLIND_MAX_OFFSET_M:
                            blind_dist_m, blind_time_s = compute_rear_blind_plan(dist_front_m)

                            last_rear_target = {
                                "dist_front_m": dist_front_m,
                                "dist_rear_m": dist_rear_m,
                                "offset_x_m": offset_x_m,
                                "goal_angle": goal_angle,
                                "time": time.time(),
                                "blind_dist_m": blind_dist_m,
                                "blind_time_s": blind_time_s,
                            }

                            print("\n[HANDOFF] Rear target saved.")
                            print(
                                f"[HANDOFF] front_dist={dist_front_m:.2f}m | "
                                f"cam_dist={dist_rear_m:.2f}m | "
                                f"offset_x={offset_x_m:.2f}m | angle={goal_angle:.3f}"
                            )
                            print(f"[HANDOFF] blind_dist={blind_dist_m:.2f}m blind_time={blind_time_s:.2f}s")
                            print("[HANDOFF] state -> REAR_BLIND")

                            state = "REAR_BLIND"
                            action = "stop"
                            speed = 0
                            status_text = "ENTER REAR BLIND"
                            stop_motor(motor_sock, repeat=5, interval=0.03)

                    else:
                        if STOP_WHEN_TARGET_LOST:
                            action = "stop"
                            speed = 0
                            status_text = "TARGET LOST - STOP"

                    if e_stop_active:
                        action = "stop"
                        speed = 0
                        status_text = "E-STOP"

                    if not ENABLE_MOTION:
                        action = "stop"
                        speed = 0
                        status_text = "MOTION DISABLED"

                    last_action = action
                    last_speed = speed
                    last_rear_decision_time = time.time()

                    last_rear_info = {
                        "target_found": target_found,
                        "dist_front_m": dist_front_m,
                        "dist_rear_m": dist_rear_m,
                        "offset_x_m": offset_x_m,
                        "goal_angle": goal_angle,
                        "box_h_ratio": box_h_ratio,
                        "box_bottom_ratio": box_bottom_ratio,
                        "action": action,
                        "speed": speed,
                        "status_text": status_text,
                        "distance_state": distance_state,
                        "inference_time": inference_time,
                    }

                else:
                    annotated = frame.copy()
                    inference_time = 0.0

                if state == "REAR_FOLLOW":
                    keepalive_send(motor_sock, last_action, last_speed)

                remain = max(0.0, REAR_DECISION_INTERVAL - (time.time() - last_rear_decision_time))

                draw_common_overlay(
                    frame=annotated,
                    title="REAR DISTANCE CURVE FOLLOW",
                    state_text=state,
                    target_found=last_rear_info["target_found"],
                    goal_angle=last_rear_info["goal_angle"],
                    dist_main_m=last_rear_info["dist_front_m"],
                    dist_second_m=last_rear_info["dist_rear_m"],
                    offset_x_m=last_rear_info["offset_x_m"],
                    desired_wz=0.0,
                    box_h_ratio=last_rear_info["box_h_ratio"],
                    box_bottom_ratio=last_rear_info["box_bottom_ratio"],
                    action=last_action,
                    speed=last_speed,
                    status_text=last_rear_info["status_text"],
                    frame_age=frame_age,
                    inference_time=last_rear_info.get("inference_time", inference_time),
                    next_decision_remain=remain
                )

                print(
                    f"State:{state} | Cam:REAR | "
                    f"target:{last_rear_info['target_found']} | "
                    f"front_dist:{last_rear_info['dist_front_m']:.2f} | "
                    f"cam_dist:{last_rear_info['dist_rear_m']:.2f} | "
                    f"offset_x:{last_rear_info['offset_x_m']:.2f} | "
                    f"action:{last_action} speed:{last_speed} | "
                    f"next:{remain:.2f}s | {last_rear_info['status_text']}"
                )

                cv2.imshow("Rear-to-Arm Distance-Wz Handoff", annotated)

            # ==================================================
            # ARM_ALIGN：每 0.5 秒判斷一次，使用距離 + wz
            # ==================================================
            elif state == "ARM_ALIGN":
                now = time.time()
                need_decision = (now - last_arm_decision_time) >= ARM_DECISION_INTERVAL

                if need_decision:
                    yolo_start = time.time()
                    results = yolo_model.predict(source=frame, conf=YOLO_CONF, imgsz=IMG_SIZE, verbose=False)[0]
                    inference_time = time.time() - yolo_start

                    annotated = results.plot()
                    box = select_largest_box(results)

                    target_found = False
                    goal_angle = 0.0
                    dist_arm_m = -1.0
                    offset_x_m = 0.0
                    desired_wz = 0.0
                    box_h_ratio = 0.0
                    box_bottom_ratio = 0.0
                    action = "stop"
                    speed = 0
                    status_text = "NO TARGET"
                    distance_state = "NO_TARGET"

                    if box is not None:
                        target_found = True

                        x1 = int(box.xyxy[0][0])
                        y1 = int(box.xyxy[0][1])
                        x2 = int(box.xyxy[0][2])
                        y2 = int(box.xyxy[0][3])

                        box_w = max(1, x2 - x1)
                        box_h = max(1, y2 - y1)

                        cx_box = int((x1 + x2) / 2)
                        cy_box = int((y1 + y2) / 2)

                        box_h_ratio = box_h / FRAME_H
                        box_bottom_ratio = y2 / FRAME_H
                        goal_angle = (CENTER_X - cx_box) / CENTER_X

                        dist_arm_m = estimate_ground_distance(y2, THETA_ARM, H_ARM, FY_ARM, CY_ARM)
                        offset_x_m = estimate_offset_x(cx_box, dist_arm_m, FX_ARM, CX_ARM)

                        cv2.circle(annotated, (cx_box, cy_box), 6, (0, 255, 255), -1)
                        cv2.line(annotated, (CENTER_X, cy_box), (cx_box, cy_box), (0, 255, 255), 2)

                        draw_object_distance_label(
                            frame=annotated,
                            x1=x1,
                            y1=y1,
                            primary_dist_m=dist_arm_m,
                            secondary_dist_m=None,
                            offset_x_m=offset_x_m,
                            label_prefix="ARM"
                        )

                        action, speed, status_text, distance_state, desired_wz = decide_arm_action_by_distance(
                            target_found=True,
                            offset_x_m=offset_x_m,
                            dist_arm_m=dist_arm_m
                        )

                        if action == "arm_blind_push":
                            state = "ARM_BLIND_PUSH"
                            action = "stop"
                            speed = 0
                            stop_motor(motor_sock, repeat=5, interval=0.03)

                    else:
                        if STOP_WHEN_TARGET_LOST:
                            action = "stop"
                            speed = 0
                            status_text = "ARM TARGET LOST - STOP"

                    if e_stop_active:
                        action = "stop"
                        speed = 0
                        status_text = "E-STOP"

                    if not ENABLE_MOTION:
                        action = "stop"
                        speed = 0
                        status_text = "MOTION DISABLED"

                    last_action = action
                    last_speed = speed
                    last_arm_decision_time = time.time()

                    last_arm_info = {
                        "target_found": target_found,
                        "dist_arm_m": dist_arm_m,
                        "offset_x_m": offset_x_m,
                        "goal_angle": goal_angle,
                        "desired_wz": desired_wz,
                        "box_h_ratio": box_h_ratio,
                        "box_bottom_ratio": box_bottom_ratio,
                        "action": action,
                        "speed": speed,
                        "status_text": status_text,
                        "distance_state": distance_state,
                        "inference_time": inference_time,
                    }

                else:
                    annotated = frame.copy()
                    inference_time = 0.0

                if state == "ARM_ALIGN":
                    keepalive_send(motor_sock, last_action, last_speed)

                remain = max(0.0, ARM_DECISION_INTERVAL - (time.time() - last_arm_decision_time))

                draw_common_overlay(
                    frame=annotated,
                    title="ARM DISTANCE + WZ ALIGN",
                    state_text=state,
                    target_found=last_arm_info["target_found"],
                    goal_angle=last_arm_info["goal_angle"],
                    dist_main_m=last_arm_info["dist_arm_m"],
                    dist_second_m=None,
                    offset_x_m=last_arm_info["offset_x_m"],
                    desired_wz=last_arm_info["desired_wz"],
                    box_h_ratio=last_arm_info["box_h_ratio"],
                    box_bottom_ratio=last_arm_info["box_bottom_ratio"],
                    action=last_action,
                    speed=last_speed,
                    status_text=last_arm_info["status_text"],
                    frame_age=frame_age,
                    inference_time=last_arm_info.get("inference_time", inference_time),
                    next_decision_remain=remain
                )

                print(
                    f"State:{state} | Cam:ARM | "
                    f"target:{last_arm_info['target_found']} | "
                    f"dist:{last_arm_info['dist_arm_m']:.2f} | "
                    f"offset_x:{last_arm_info['offset_x_m']:.2f} | "
                    f"wz:{last_arm_info['desired_wz']:.3f} | "
                    f"action:{last_action} speed:{last_speed} | "
                    f"next:{remain:.2f}s | {last_arm_info['status_text']}"
                )

                cv2.imshow("Rear-to-Arm Distance-Wz Handoff", annotated)

            # ==================================================
            # keyboard
            # ==================================================
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                print("[INFO] Q pressed.")
                break

            elif key == ord("s"):
                e_stop_active = not e_stop_active
                if e_stop_active:
                    print("[E-STOP] ON")
                    stop_motor(motor_sock)
                else:
                    print("[E-STOP] OFF")

            elif key == ord("r"):
                state = "REAR_FOLLOW"
                e_stop_active = False
                last_rear_decision_time = 0.0
                last_arm_decision_time = 0.0
                stop_motor(motor_sock)
                print("[RESET] state -> REAR_FOLLOW")

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected.")

    finally:
        print("[INFO] Stopping and closing resources...")

        try:
            stop_motor(motor_sock, repeat=10, interval=0.03)
        except Exception as e:
            print(f"[WARN] stop failed: {e}")

        try:
            motor_sock.close()
        except Exception:
            pass

        try:
            cap_rear.release()
        except Exception:
            pass

        try:
            cap_arm.release()
        except Exception:
            pass

        cv2.destroyAllWindows()
        print("[INFO] Program ended safely.")


if __name__ == "__main__":
    main()
