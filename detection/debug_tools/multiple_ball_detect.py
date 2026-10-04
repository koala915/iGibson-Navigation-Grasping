import cv2
import time
import math
import json
import os
import socket
import sys
import threading
import numpy as np
from ultralytics import YOLO

ENABLE_MOTION = "--real" in sys.argv[1:]


# ============================================================
# 1. 基本設定
# ============================================================

YOLO_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v3i.yolov11 (1)\runs\detect\train7\weights\best.pt"

JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "yahboom.local")
MOTOR_PORT = 7000

URL_REAR = f"http://{JETSON_IP}:8080/stream?topic=/back_cam/image_raw"

FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2

# 畫質差、白色雜訊多，不要太低
YOLO_CONF = 0.25
IMG_SIZE = 640
YOLO_INTERVAL = 0.5

COMMAND_INTERVAL = 0.05
MANUAL_SPEED = 30


# ============================================================
# 2. 後鏡頭距離校正
# ============================================================

THETA_REAR = 13.92
H_REAR = 0.50

FY_REAR = 956.0294
CY_REAR = 221.3649
FX_REAR = 957.6253
CX_REAR = 320.0

REAR_CAM_TO_FRONT_M = 0.20


# ============================================================
# 3. 車速校正，用來更新記憶地圖
# ============================================================

KX = 0.006995
KZ = 0.02384

# 實車會打滑、起步慢、轉向不準，所以先不要 100% 相信 KX / KZ
MOTION_SCALE = 0.70
TURN_SCALE = 0.70

USE_MOTION_UPDATE = True


# ============================================================
# 4. Target Memory 參數
# ============================================================

# 最多只追兩顆紙球
MAX_TRACKED_TARGETS = 2

# YOLO 最多允許幾個候選框，最後還是只取前 2 個
MAX_YOLO_CANDIDATES = 10

# 過濾太小、太怪的框
MIN_BOX_AREA = 300
MIN_BOX_ASPECT = 0.45
MAX_BOX_ASPECT = 2.20

# 同一顆球合併半徑：放大一點，避免同一顆球一直開新 ID
BASE_SAME_TARGET_RADIUS = 0.45
MAX_SAME_TARGET_RADIUS = 1.00

# 重複 target 合併，避免實際 2 顆球但記憶跳到 ID6、ID7
ENABLE_DUPLICATE_MERGE = True
DUPLICATE_MERGE_RADIUS = 0.35

# 記憶位置不確定度
INITIAL_SIGMA = 0.15
MIN_SIGMA = 0.08
MAX_SIGMA = 0.90

# 移動越多，記憶誤差越大
SIGMA_GROW_PER_M = 0.18
SIGMA_GROW_PER_RAD = 0.28
SIGMA_GROW_PER_SEC = 0.002

# 保守刪除：只有「很明確應該看得到」，但 YOLO 超過 5 秒沒看到才刪
VISIBLE_MISSING_DELETE_SEC = 5.0

# should_visible 保守條件
EXPECTED_VISIBLE_MAX_X = 1.60
EXPECTED_VISIBLE_MIN_X = 0.25
EXPECTED_VISIBLE_MAX_ABS_Y = 0.45
CAMERA_HALF_FOV_DEG = 18
MAX_SIGMA_FOR_VISIBLE_DELETE = 0.35

MIN_CONFIDENCE_TO_SELECT = 1


# ============================================================
# 5. Radar 顯示
# ============================================================

RADAR_W = 500
RADAR_H = 480
RADAR_SCALE = 100

RADAR_MAX_X = 8.0
RADAR_MAX_Y = 4.0


# ============================================================
# 6. Camera Reader：只保留最新畫面
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
# 7. Motor Socket：只連一次，不用 cmd_vel
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
# 8. 座標換算
# ============================================================

def estimate_ground_distance(y_pixel, theta_deg, h_m, fy, cy):
    theta = math.radians(theta_deg)
    alpha = math.atan((y_pixel - cy) / fy)
    total_angle = theta + alpha

    if total_angle <= 0.01:
        return None

    dist_m = h_m / math.tan(total_angle)

    if dist_m <= 0 or dist_m > 10:
        return None

    return dist_m


def rear_dist_to_front_dist(dist_rear_m):
    return max(0.0, dist_rear_m - REAR_CAM_TO_FRONT_M)


def estimate_offset_x(cx_pixel, dist_m, fx, cx):
    """
    offset_x_m > 0：畫面右邊
    offset_x_m < 0：畫面左邊
    """
    return (cx_pixel - cx) * dist_m / fx


def detection_to_robot_xy(cx_box, y2_box):
    """
    robot 座標：
    x = 車頭前方距離
    y = 左正右負
    """
    dist_rear_m = estimate_ground_distance(
        y2_box,
        THETA_REAR,
        H_REAR,
        FY_REAR,
        CY_REAR
    )

    if dist_rear_m is None:
        return None

    dist_front_m = rear_dist_to_front_dist(dist_rear_m)

    offset_x_m = estimate_offset_x(
        cx_box,
        dist_rear_m,
        FX_REAR,
        CX_REAR
    )

    robot_x = dist_front_m
    robot_y = -offset_x_m

    return robot_x, robot_y, dist_rear_m, dist_front_m, offset_x_m


def select_top_two_detections_from_yolo(boxes):
    """
    從 YOLO 輸出的 boxes 裡面，挑出最適合的前兩個紙球。

    流程：
    1. 過濾太小、太扁、太怪的框
    2. 換算成機器人座標
    3. 過濾太遠、太偏的框
    4. 用 conf + area 排序
    5. 最多回傳 2 個
    """
    candidates = []

    for box in boxes:
        xyxy = box.xyxy[0].cpu().numpy()
        conf = float(box.conf[0].cpu().numpy())

        x1, y1, x2, y2 = xyxy
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)

        w = max(1, x2 - x1)
        h = max(1, y2 - y1)
        area = w * h
        aspect = w / h

        if area < MIN_BOX_AREA:
            continue

        if aspect < MIN_BOX_ASPECT or aspect > MAX_BOX_ASPECT:
            continue

        cx_box = int((x1 + x2) / 2)
        y2_box = int(y2)

        converted = detection_to_robot_xy(cx_box, y2_box)

        if converted is None:
            continue

        robot_x, robot_y, dist_rear_m, dist_front_m, offset_x_m = converted

        # 太遠先不要，畫質差時遠處很容易誤判
        if dist_front_m > 3.0:
            continue

        # 太旁邊先不要，避免邊緣白色雜訊
        if abs(robot_y) > 1.8:
            continue

        area_score = min(area / 5000.0, 1.5)
        score = conf * 0.75 + area_score * 0.25

        det = {
            "x": robot_x,
            "y": robot_y,
            "dist_rear_m": dist_rear_m,
            "dist_front_m": dist_front_m,
            "offset_x_m": offset_x_m,
            "bbox": (x1, y1, x2, y2),
            "conf": conf,
            "area": area,
            "aspect": aspect,
            "score": score
        }

        candidates.append(det)

    candidates.sort(key=lambda d: d["score"], reverse=True)

    return candidates[:MAX_TRACKED_TARGETS]


# ============================================================
# 9. Target Memory Map
# ============================================================

class TargetMemory:
    def __init__(self):
        self.targets = []
        self.next_id = 1

    def update_with_detections(self, detections_xy):
        now = time.time()
        matched_ids = set()
        used_target_ids = set()

        for det in detections_xy:
            x = det["x"]
            y = det["y"]

            matched_target = None
            min_dist = 999.0

            for target in self.targets:
                if target["status"] != "active":
                    continue

                if target["id"] in used_target_ids:
                    continue

                d = math.sqrt((x - target["x"]) ** 2 + (y - target["y"]) ** 2)

                # sigma 越大，代表舊記憶越不準，所以允許合併範圍變大
                gate = BASE_SAME_TARGET_RADIUS + target["sigma"]
                gate = min(gate, MAX_SAME_TARGET_RADIUS)

                if d < gate and d < min_dist:
                    matched_target = target
                    min_dist = d

            if matched_target is not None:
                self.update_existing_target(matched_target, det, now)
                matched_ids.add(matched_target["id"])
                used_target_ids.add(matched_target["id"])
            else:
                self.add_new_target(det, now)

        if ENABLE_DUPLICATE_MERGE:
            self.merge_duplicate_targets()

        # 最多只保留兩顆紙球
        self.limit_to_two_targets()

        self.handle_not_seen_targets(matched_ids, now)

    def update_existing_target(self, target, det, now):
        x = det["x"]
        y = det["y"]

        # sigma 越大，舊資料越不可信，新觀測權重提高
        a = 0.35 + min(target["sigma"], 0.5) * 0.45
        a = max(0.35, min(a, 0.60))

        target["x"] = target["x"] * (1 - a) + x * a
        target["y"] = target["y"] * (1 - a) + y * a

        target["last_seen"] = now
        target["confidence"] += 1
        target["bbox"] = det["bbox"]
        target["det_conf"] = det["conf"]

        # 重新看到後，清除 visible missing timer
        target["visible_missing_since"] = None
        target["miss_total_count"] = 0

        # 重新看到後，不確定度下降
        target["sigma"] = max(MIN_SIGMA, target["sigma"] * 0.50)

    def add_new_target(self, det, now):
        new_target = {
            "id": self.next_id,
            "x": det["x"],
            "y": det["y"],
            "confidence": 1,
            "last_seen": now,
            "first_seen": now,
            "status": "active",
            "bbox": det["bbox"],
            "det_conf": det["conf"],
            "sigma": INITIAL_SIGMA,
            "visible_missing_since": None,
            "miss_total_count": 0,
            "collected": False
        }

        self.targets.append(new_target)
        self.next_id += 1

    def merge_duplicate_targets(self):
        """
        合併因為抖動或座標更新誤差造成的重複 ID。
        避免實際只有 2 顆球，記憶卻跑到 ID6、ID7。
        """
        active = [t for t in self.targets if t["status"] == "active"]
        removed_ids = set()

        for i in range(len(active)):
            a = active[i]

            if a["id"] in removed_ids:
                continue

            for j in range(i + 1, len(active)):
                b = active[j]

                if b["id"] in removed_ids:
                    continue

                d = math.sqrt((a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2)

                merge_gate = DUPLICATE_MERGE_RADIUS + 0.30 * max(a["sigma"], b["sigma"])
                merge_gate = min(merge_gate, 0.55)

                if d < merge_gate:
                    if (b["confidence"] > a["confidence"]) or (
                        b["confidence"] == a["confidence"] and b["id"] < a["id"]
                    ):
                        keep = b
                        drop = a
                    else:
                        keep = a
                        drop = b

                    total_conf = max(1, keep["confidence"] + drop["confidence"])
                    wk = keep["confidence"] / total_conf
                    wd = drop["confidence"] / total_conf

                    keep["x"] = keep["x"] * wk + drop["x"] * wd
                    keep["y"] = keep["y"] * wk + drop["y"] * wd
                    keep["confidence"] += drop["confidence"]
                    keep["sigma"] = min(MAX_SIGMA, max(keep["sigma"], drop["sigma"]) * 0.85)
                    keep["last_seen"] = max(keep["last_seen"], drop["last_seen"])

                    if keep.get("visible_missing_since") is None:
                        keep["visible_missing_since"] = drop.get("visible_missing_since")

                    removed_ids.add(drop["id"])

                    print(
                        f"[MERGE] ID{drop['id']} -> ID{keep['id']} | "
                        f"d={d:.2f} gate={merge_gate:.2f}"
                    )

        if len(removed_ids) > 0:
            self.targets = [
                t for t in self.targets
                if t["id"] not in removed_ids
            ]

    def limit_to_two_targets(self):
        """
        memory 最多只保留兩顆 active target。
        如果多於兩顆，保留：
        1. confidence 高
        2. sigma 小
        3. 比較靠近正前方
        4. 比較近
        """
        active = [t for t in self.targets if t["status"] == "active"]

        if len(active) <= MAX_TRACKED_TARGETS:
            return

        def keep_score(t):
            return (
                t["confidence"] * 2.0
                - t["sigma"] * 3.0
                - abs(t["y"]) * 0.8
                - t["x"] * 0.2
            )

        active_sorted = sorted(active, key=keep_score, reverse=True)
        keep_ids = set(t["id"] for t in active_sorted[:MAX_TRACKED_TARGETS])

        removed = []

        new_targets = []
        for t in self.targets:
            if t["status"] != "active":
                continue

            if t["id"] in keep_ids:
                new_targets.append(t)
            else:
                removed.append(t["id"])

        if removed:
            print(f"[LIMIT] remove extra targets: {removed}, keep only {MAX_TRACKED_TARGETS}")

        self.targets = new_targets

    def handle_not_seen_targets(self, matched_ids, now):
        """
        刪除邏輯：
        只有當目標非常明確應該在 rear cam 視野內，
        但 YOLO 連續超過 VISIBLE_MISSING_DELETE_SEC 秒沒看到，才刪掉。
        """
        kept_targets = []

        for target in self.targets:
            if target["status"] != "active":
                continue

            if target["id"] in matched_ids:
                kept_targets.append(target)
                continue

            should_visible = self.should_be_visible_in_camera(target)

            if should_visible:
                target["miss_total_count"] += 1

                if target["visible_missing_since"] is None:
                    target["visible_missing_since"] = now
                    print(f"[VISIBLE-MISS-START] ID{target['id']} should be visible.")

                missing_time = now - target["visible_missing_since"]

                print(
                    f"[VISIBLE-MISS] ID{target['id']} should be visible but YOLO missing. "
                    f"missing_time={missing_time:.2f}s"
                )

                if missing_time >= VISIBLE_MISSING_DELETE_SEC:
                    print(
                        f"[DELETE] ID{target['id']} deleted. "
                        f"Visible but missing > {VISIBLE_MISSING_DELETE_SEC:.1f}s"
                    )
                    continue

            else:
                # 不在目前鏡頭朝向範圍內，不能因為 YOLO 沒看到就刪
                target["visible_missing_since"] = None

            kept_targets.append(target)

        self.targets = kept_targets

    def should_be_visible_in_camera(self, target):
        """
        保守版 should_visible：
        只有非常明確在鏡頭正前方，才允許啟動刪除倒數。
        """
        x = target["x"]
        y = target["y"]
        sigma = target["sigma"]

        # 記憶誤差太大，不刪
        if sigma > MAX_SIGMA_FOR_VISIBLE_DELETE:
            return False

        # 太近或太遠都不刪，因為距離估算和視野都不穩
        if x <= EXPECTED_VISIBLE_MIN_X:
            return False

        if x > EXPECTED_VISIBLE_MAX_X:
            return False

        # 偏太旁邊，不刪
        if abs(y) > EXPECTED_VISIBLE_MAX_ABS_Y:
            return False

        bearing_deg = math.degrees(math.atan2(y, x))

        # 不用 sigma 去放寬 FOV，避免太容易誤判「應該看得到」
        return abs(bearing_deg) <= CAMERA_HALF_FOV_DEG

    def update_by_motion(self, action, speed, dt, kx=KX, kz=KZ, imu_dtheta=None):
        """
        根據目前送出的 action 更新記憶地圖。
        如果未來有 IMU，可以把 imu_dtheta 傳進來。
        """
        if not USE_MOTION_UPDATE:
            return

        vx = 0.0
        wz = 0.0

        if action == "forward":
            vx = kx * speed * MOTION_SCALE
            wz = 0.0

        elif action == "backward":
            vx = -kx * speed * MOTION_SCALE
            wz = 0.0

        elif action == "turn_left":
            vx = 0.0
            wz = kz * speed * TURN_SCALE

        elif action == "turn_right":
            vx = 0.0
            wz = -kz * speed * TURN_SCALE

        elif action == "curve_left":
            vx = kx * speed * 0.8 * MOTION_SCALE
            wz = kz * speed * 0.35 * TURN_SCALE

        elif action == "curve_right":
            vx = kx * speed * 0.8 * MOTION_SCALE
            wz = -kz * speed * 0.35 * TURN_SCALE

        else:
            vx = 0.0
            wz = 0.0

        dx = vx * dt

        if imu_dtheta is not None:
            dtheta = imu_dtheta
        else:
            dtheta = wz * dt

        self.apply_robot_motion(dx, dtheta)

    def apply_robot_motion(self, dx, dtheta):
        c = math.cos(dtheta)
        s = math.sin(dtheta)

        motion_amount = abs(dx)
        turn_amount = abs(dtheta)

        for target in self.targets:
            if target["status"] != "active":
                continue

            x = target["x"]
            y = target["y"]

            # 機器人前進，球在相對座標中變近
            x = x - dx

            # 機器人轉向，地圖做反向旋轉
            x_new = c * x + s * y
            y_new = -s * x + c * y

            target["x"] = x_new
            target["y"] = y_new

            # 移動與轉向造成記憶誤差增加
            target["sigma"] += SIGMA_GROW_PER_M * motion_amount
            target["sigma"] += SIGMA_GROW_PER_RAD * turn_amount
            target["sigma"] = min(target["sigma"], MAX_SIGMA)

    def grow_uncertainty_by_time(self, dt):
        for target in self.targets:
            if target["status"] != "active":
                continue

            target["sigma"] += SIGMA_GROW_PER_SEC * dt
            target["sigma"] = min(target["sigma"], MAX_SIGMA)

    def mark_collected(self, target_id):
        new_targets = []

        for target in self.targets:
            if target["id"] == target_id:
                print(f"[COLLECTED] ID{target_id} removed from memory.")
                continue

            new_targets.append(target)

        self.targets = new_targets

    def select_best_target(self):
        candidates = []

        for target in self.targets:
            if target["status"] != "active":
                continue

            if target["confidence"] < MIN_CONFIDENCE_TO_SELECT:
                continue

            x = target["x"]
            y = target["y"]
            sigma = target["sigma"]

            if x <= 0:
                continue

            # score 越小越優先
            score = x + 1.5 * abs(y) + 0.8 * sigma
            candidates.append((score, target))

        if len(candidates) == 0:
            return None

        candidates.sort(key=lambda item: item[0])
        return candidates[0][1]

    def debug_print(self):
        items = []

        for t in self.targets:
            if t["status"] != "active":
                continue

            missing = None
            if t["visible_missing_since"] is not None:
                missing = round(time.time() - t["visible_missing_since"], 1)

            items.append({
                "id": t["id"],
                "x": round(t["x"], 2),
                "y": round(t["y"], 2),
                "sig": round(t["sigma"], 2),
                "conf": t["confidence"],
                "vis": self.should_be_visible_in_camera(t),
                "miss": missing
            })

        print("[MEMORY]", items)


# ============================================================
# 10. 視覺化
# ============================================================

def draw_detections(frame, detections_xy, selected_target, last_yolo_age, current_action, current_speed):
    for det in detections_xy:
        x1, y1, x2, y2 = det["bbox"]
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)

        text = f"x={det['x']:.2f} y={det['y']:.2f} conf={det['conf']:.2f}"

        cv2.putText(
            frame,
            text,
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2
        )

    if selected_target is not None:
        missing_text = "none"

        if selected_target["visible_missing_since"] is not None:
            missing_text = f"{time.time() - selected_target['visible_missing_since']:.1f}s"

        msg = (
            f"SELECT ID{selected_target['id']} "
            f"x={selected_target['x']:.2f} "
            f"y={selected_target['y']:.2f} "
            f"sigma={selected_target['sigma']:.2f} "
            f"miss={missing_text}"
        )

        cv2.putText(
            frame,
            msg,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2
        )
    else:
        cv2.putText(
            frame,
            "NO TARGET",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 0, 255),
            2
        )

    cv2.putText(
        frame,
        f"ACTION={current_action} SPEED={current_speed}",
        (15, 65),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2
    )

    cv2.putText(
        frame,
        f"YOLO_CONF={YOLO_CONF:.2f} | used boxes={len(detections_xy)} | last update={last_yolo_age:.2f}s",
        (15, FRAME_H - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2
    )


def robot_xy_to_radar_px(x, y):
    origin_x = RADAR_W // 2
    origin_y = RADAR_H - 45

    px = int(origin_x - y * RADAR_SCALE)
    py = int(origin_y - x * RADAR_SCALE)

    return px, py


def draw_radar(memory, selected_target):
    radar = np.zeros((RADAR_H, RADAR_W, 3), dtype=np.uint8)

    origin_x = RADAR_W // 2
    origin_y = RADAR_H - 45

    cv2.circle(radar, (origin_x, origin_y), 10, (255, 255, 255), -1)
    cv2.putText(
        radar,
        "ROBOT",
        (origin_x - 35, origin_y + 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1
    )

    cv2.line(radar, (origin_x, origin_y), (origin_x, 20), (80, 80, 80), 1)

    # 保守刪除用 FOV 線
    for sign in [-1, 1]:
        ang = math.radians(sign * CAMERA_HALF_FOV_DEG)
        length = 2.0
        end_x = int(origin_x - math.sin(ang) * length * RADAR_SCALE)
        end_y = int(origin_y - math.cos(ang) * length * RADAR_SCALE)
        cv2.line(radar, (origin_x, origin_y), (end_x, end_y), (60, 60, 120), 1)

    for d in [0.5, 1.0, 1.5, 2.0, 3.0, 4.0]:
        r = int(d * RADAR_SCALE)
        cv2.circle(radar, (origin_x, origin_y), r, (45, 45, 45), 1)
        cv2.putText(
            radar,
            f"{d:.1f}m",
            (origin_x + 8, origin_y - r),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (120, 120, 120),
            1
        )

    now = time.time()

    for target in memory.targets:
        x = target["x"]
        y = target["y"]

        # 只是顯示限制，不代表從 memory 刪掉
        if x < -3.0 or x > RADAR_MAX_X:
            continue

        if abs(y) > RADAR_MAX_Y:
            continue

        px, py = robot_xy_to_radar_px(x, y)

        is_selected = (
            selected_target is not None and
            target["id"] == selected_target["id"]
        )

        should_visible = memory.should_be_visible_in_camera(target)

        if is_selected:
            color = (0, 0, 255)
            radius = 9
        elif should_visible:
            color = (0, 255, 255)
            radius = 7
        else:
            color = (180, 180, 180)
            radius = 6

        cv2.circle(radar, (px, py), radius, color, -1)

        sigma_px = int(target["sigma"] * RADAR_SCALE)
        cv2.circle(radar, (px, py), sigma_px, (80, 80, 80), 1)

        missing_text = "none"
        if target["visible_missing_since"] is not None:
            missing_text = f"{now - target['visible_missing_since']:.1f}s"

        label = f"ID{target['id']} ({x:.2f},{y:.2f})"
        cv2.putText(
            radar,
            label,
            (px + 8, py - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            color,
            1
        )

        info = f"vis={should_visible} miss={missing_text} sig={target['sigma']:.2f}"
        cv2.putText(
            radar,
            info,
            (px + 8, py + 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.32,
            (150, 150, 150),
            1
        )

    cv2.putText(
        radar,
        "Target Memory Map - Max 2 Balls",
        (15, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2
    )

    cv2.putText(
        radar,
        "Only top 2 YOLO candidates are tracked",
        (15, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (180, 180, 180),
        1
    )

    cv2.putText(
        radar,
        "W/S/A/D move | Q/E curve | SPACE stop | X collect | C clear | M motion",
        (15, RADAR_H - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.33,
        (180, 180, 180),
        1
    )

    return radar


# ============================================================
# 11. 主程式
# ============================================================

def main():
    global YOLO_CONF
    global USE_MOTION_UPDATE
    global MOTION_SCALE
    global TURN_SCALE

    if not ENABLE_MOTION:
        print("[SAFETY] Detection-only mode; add --real to permit motor motion.")

    print("[INFO] Loading YOLO model...")
    model = YOLO(YOLO_MODEL_PATH)

    print("[INFO] Model classes:")
    print(model.names)

    print(f"[INFO] Connecting motor server {JETSON_IP}:{MOTOR_PORT}...")
    motor_sock = connect_motor_server()
    print("[INFO] motor server connected.")

    print("[INFO] Opening rear camera stream...")
    reader = LatestFrameReader(URL_REAR, FRAME_W, FRAME_H)

    memory = TargetMemory()

    last_yolo_time = 0.0
    last_yolo_update_time = 0.0
    last_time = time.time()
    last_motor_send_time = 0.0
    last_debug_time = 0.0

    last_detections_xy = []

    current_action = "stop"
    current_speed = 0

    print("\n================ Keyboard Control ================")
    print("W: forward")
    print("S: backward")
    print("A: turn_left")
    print("D: turn_right")
    print("Q: curve_left")
    print("E: curve_right")
    print("SPACE: stop")
    print("X: collect selected target")
    print("C: clear memory")
    print("M: toggle motion update")
    print("1/2: decrease/increase MOTION_SCALE")
    print("3/4: decrease/increase TURN_SCALE")
    print("+/-: adjust YOLO_CONF")
    print("ESC: quit")
    print("==================================================\n")

    try:
        while True:
            ok, frame, frame_time = reader.read()

            if not ok or frame is None:
                print("[WARN] No frame.")
                time.sleep(0.05)
                continue

            now = time.time()
            dt = now - last_time
            last_time = now

            # 用目前 action 更新記憶地圖
            memory.update_by_motion(current_action, current_speed, dt)

            # 時間造成的不確定度增加
            memory.grow_uncertainty_by_time(dt)

            # 馬達 keep-alive
            try:
                last_motor_send_time = keepalive_send(
                    motor_sock,
                    current_action,
                    current_speed,
                    last_motor_send_time
                )
            except Exception as e:
                print(f"[MOTOR ERROR] keepalive failed: {e}")

            # YOLO 每 0.5 秒跑一次
            if now - last_yolo_time >= YOLO_INTERVAL:
                last_yolo_time = now
                last_yolo_update_time = now

                new_detections_xy = []

                results = model.predict(
                    frame,
                    imgsz=IMG_SIZE,
                    conf=YOLO_CONF,
                    iou=0.8,
                    max_det=MAX_YOLO_CANDIDATES,
                    verbose=False
                )

                if len(results) > 0 and results[0].boxes is not None:
                    boxes = results[0].boxes
                    print(f"[YOLO] raw boxes = {len(boxes)}")

                    new_detections_xy = select_top_two_detections_from_yolo(boxes)

                    print(f"[YOLO] used boxes = {len(new_detections_xy)} / max {MAX_TRACKED_TARGETS}")

                    for det in new_detections_xy:
                        print(
                            f"[USE] conf={det['conf']:.2f} "
                            f"area={det['area']} "
                            f"aspect={det['aspect']:.2f} "
                            f"x={det['x']:.2f} "
                            f"y={det['y']:.2f} "
                            f"score={det['score']:.2f}"
                        )

                last_detections_xy = new_detections_xy
                memory.update_with_detections(new_detections_xy)

            selected_target = memory.select_best_target()

            if now - last_debug_time >= 1.0:
                memory.debug_print()
                print(
                    f"[PARAM] motion_update={USE_MOTION_UPDATE} "
                    f"motion_scale={MOTION_SCALE:.2f} "
                    f"turn_scale={TURN_SCALE:.2f} "
                    f"yolo_conf={YOLO_CONF:.2f}"
                )
                last_debug_time = now

            last_yolo_age = time.time() - last_yolo_update_time if last_yolo_update_time > 0 else 999

            draw_detections(
                frame,
                last_detections_xy,
                selected_target,
                last_yolo_age,
                current_action,
                current_speed
            )

            radar = draw_radar(memory, selected_target)

            cv2.imshow("Rear Cam + YOLO", frame)
            cv2.imshow("Target Memory Radar", radar)

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

            elif key in [ord("c"), ord("C")]:
                memory = TargetMemory()
                last_detections_xy = []
                print("[KEY] memory cleared")

            elif key in [ord("x"), ord("X")]:
                if selected_target is not None:
                    memory.mark_collected(selected_target["id"])
                    last_detections_xy = []
                else:
                    print("[WARN] No selected target to collect.")

            elif key in [ord("m"), ord("M")]:
                USE_MOTION_UPDATE = not USE_MOTION_UPDATE
                print(f"[KEY] USE_MOTION_UPDATE = {USE_MOTION_UPDATE}")

            elif key == ord("1"):
                MOTION_SCALE = max(0.10, MOTION_SCALE - 0.10)
                print(f"[KEY] MOTION_SCALE = {MOTION_SCALE:.2f}")

            elif key == ord("2"):
                MOTION_SCALE = min(1.50, MOTION_SCALE + 0.10)
                print(f"[KEY] MOTION_SCALE = {MOTION_SCALE:.2f}")

            elif key == ord("3"):
                TURN_SCALE = max(0.10, TURN_SCALE - 0.10)
                print(f"[KEY] TURN_SCALE = {TURN_SCALE:.2f}")

            elif key == ord("4"):
                TURN_SCALE = min(1.50, TURN_SCALE + 0.10)
                print(f"[KEY] TURN_SCALE = {TURN_SCALE:.2f}")

            elif key == ord("+") or key == ord("="):
                YOLO_CONF += 0.05
                if YOLO_CONF > 0.95:
                    YOLO_CONF = 0.95
                print(f"[INFO] YOLO_CONF = {YOLO_CONF:.2f}")

            elif key == ord("-") or key == ord("_"):
                YOLO_CONF -= 0.05
                if YOLO_CONF < 0.05:
                    YOLO_CONF = 0.05
                print(f"[INFO] YOLO_CONF = {YOLO_CONF:.2f}")

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
            reader.release()
        except Exception:
            pass

        cv2.destroyAllWindows()
        print("[INFO] Program ended safely.")


if __name__ == "__main__":
    main()
