# Upstream: enoch20050427/iGibson-Navigation-Grasping PR #6, cf0b3ad0cb648c9fd1f1609e958ea2250e587e64.
# Local changes: configurable assets; opt-in stopped arrival exit for E1 handoff.
import cv2
import json
import math
import os
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import gymnasium as gym
import numpy as np
import roslibpy
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from ultralytics import YOLO, SAM

if __package__:
    from .sugarbox_lidar_geometry import (
        LidarGeometry, nearest_policy_ranges, robot_scan_samples,
    )
    from .sugarbox_ground_calibration import (
        load_ground_calibration, pixel_to_ground, resolve_asset_paths,
        inside_calibration_hull as _inside_calibration_hull,
    )
else:
    from sugarbox_lidar_geometry import (
        LidarGeometry, nearest_policy_ranges, robot_scan_samples,
    )
    from sugarbox_ground_calibration import (
        load_ground_calibration, pixel_to_ground, resolve_asset_paths,
        inside_calibration_hull as _inside_calibration_hull,
    )



# ============================================================
# 0. 執行模式
# ============================================================
#
# VISION
#   YOLO + SAM2 + Homography
#   只看 sugar box 相對位置
#
# DRY_RUN
#   YOLO + SAM2 + Homography
#   + /scan
#   + /odom_setmotor
#   + PPO
#   但不送馬達
#
# DRIVE
#   完整執行並送 TCP velocity
#
# 第一次先 DRY_RUN。
# 確認左右方向和 PPO action 正確後改 DRIVE。
#
RUN_MODE = os.environ.get(
    "SUGARBOX_RUN_MODE",
    "DRY_RUN"
).strip().upper()


# ============================================================
# 1. 資產路徑
# ============================================================

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSET_PATHS = resolve_asset_paths(REPO_ROOT)
BASE_DIR = ASSET_PATHS["base_dir"]
YOLO_MODEL_PATH = ASSET_PATHS["yolo"]
SAM2_MODEL_PATH = ASSET_PATHS["sam"]
HOMOGRAPHY_PATH = ASSET_PATHS["homography"]

# 你原本 PPO
RL_MODEL_PATH = os.environ.get("SUGARBOX_RL_MODEL", os.path.join(os.path.dirname(__file__), "nav_best_model", "doorway_ft_final.zip"))

VECNORMALIZE_PATH = os.environ.get("SUGARBOX_VECNORMALIZE", os.path.join(os.path.dirname(__file__), "nav_best_model", "doorway_ft_final_vecnormalize.pkl"))


# ============================================================
# 2. Jetson / ROS / Camera / Motor
# ============================================================

# 預設使用 Jetson 的 mDNS 主機名，避免 DHCP 換 IP 後需要改程式。
# 若現場網路不支援 mDNS，可先在 PowerShell 指定當次 IP：
#
# $env:ROUTE_B_IP="10.230.78.252"
#
# 就不用改程式
#
JETSON_IP = os.environ.get(
    "ROUTE_B_IP",
    "yahboom.local"
)

ROSBRIDGE_PORT = 9090
MOTOR_PORT = 7000

REAR_TOPIC = "/back_cam/image_raw"  # 保留名稱供舊設定參考；正式影像已改走 UDP/H.264
SCAN_TOPIC = "/scan"
ODOM_TOPIC = "/odom_setmotor"

# Windows GStreamer 安裝位置。可在 PowerShell 覆蓋：
#   $env:GSTREAMER_LAUNCH_EXE="D:\\msvc_x86_64\\bin\\gst-launch-1.0.exe"
GSTREAMER_LAUNCH_EXE = os.environ.get(
    "GSTREAMER_LAUNCH_EXE",
    r"D:\msvc_x86_64\bin\gst-launch-1.0.exe"
)

# Jetson sender -> Windows 的 RTP/H.264 UDP port
CAMERA_UDP_PORT = int(
    os.environ.get(
        "CAMERA_UDP_PORT",
        "5600"
    )
)

# Python 不需要吃滿 30 FPS raw BGR。YOLO/SAM2 約 5 FPS，
# 所以 GStreamer receiver 先降到 10 FPS，避免 stdout raw-frame pipe
# 搶 CPU / GIL，造成 rosbridge /scan callback 被餓死。
CAMERA_PIPE_FPS = int(
    os.environ.get(
        "CAMERA_PIPE_FPS",
        "30"
    )
)


# ============================================================
# 3. Camera / YOLO / SAM2
# ============================================================

FRAME_W = 640
FRAME_H = 480

YOLO_CONF = 0.30
YOLO_IMGSZ = 640

# SAM mask 至少要有多少 pixel
MIN_MASK_PIXELS = 100

# 不直接取單一最低 pixel。
# 取 mask 最下方 5 pixel 高度的區域，
# 再用 x median，避免 SAM 邊界單點雜訊。
BOTTOM_BAND_PX = 5
MIN_BOTTOM_PIXELS = 3

# SAM mask bottom 和 YOLO bbox bottom
# 差太多代表 segmentation 很可能怪掉。
MAX_SAM_BOTTOM_TO_BBOX_BOTTOM_PX = 30


# SAM2 不需要每個 camera frame 都重新跑
VISION_INTERVAL = 0.20

# Visual confirmation and persistent odom target memory.
#
# A target is acquired only after several spatially-consistent detections. Once
# acquired, its robot-relative position is propagated with wheel odometry even
# while it is outside the camera view. A later detection may correct the memory
# only when it lands close to the odom-predicted position.
TARGET_VISUAL_RECENT_S = 0.60
TARGET_ACQUIRE_CONFIRM_FRAMES = 3
TARGET_REACQUIRE_CONFIRM_FRAMES = 3
TARGET_CONFIRM_MAX_GAP_S = 0.75
TARGET_ACQUIRE_MAX_ERROR_M = 0.25
TARGET_REACQUIRE_BASE_ERROR_M = 0.50
TARGET_REACQUIRE_RANGE_GAIN = 0.15
TARGET_REACQUIRE_MAX_ERROR_M = 0.70

# When the remembered target is nearby but still invisible, stop translating
# and turn slowly toward its odom-predicted bearing until vision confirms it.
TARGET_SEARCH_START_DIST_M = 0.85
TARGET_SEARCH_MIN_WZ = 0.25
TARGET_SEARCH_MAX_WZ = 0.40
TARGET_SEARCH_KP = 0.90
TARGET_SEARCH_TIMEOUT_S = 8.0

CAMERA_STALE_TIMEOUT = 0.50

# ============================================================
# Camera latency compensation
# ============================================================
# 正式影像已改成：
#   Jetson /dev/video1 -> NVENC H.264 -> RTP/UDP -> Windows GStreamer
#
# 這條路徑實測幾乎同步，並且 Windows reader thread 只保存最新 frame，
# 不再使用 web_video_server / HTTP MJPEG 的大 buffer。
#
# 實測 UDP/H.264 live preview 幾乎同步，因此 transport/decode delay
# 不再加入任何人工固定延遲（0 ms）。
# 仍保留 queue + inference 的實際時間做 odom latency compensation。
# UDP/H.264 live preview has been visually verified to be effectively real-time.
# Do not add an artificial transport delay here.
CAMERA_STREAM_DELAY_COMP_S = 0.0

# 保存 Windows 收到的 /odom_setmotor 歷史，用來把「舊畫面中的 target」
# 從估計拍攝時間 propagate 到現在。
ODOM_HISTORY_S = 3.0
MAX_VISION_COMPENSATION_S = 1.20

# latency log 不要每一幀洗版。
VISION_LATENCY_PRINT_INTERVAL = 1.0


# 位置稍微平滑
#
# 1.0 = 完全不平滑
# 越小越平滑
#
TARGET_EMA_ALPHA = 0.45


# ============================================================
# 4. Ground Homography / Floor Gate
# ============================================================

# 這裡沒有加入 SegFormer。
#
# on_floor 的定義：
#
# SAM2 mask 最底部
#       ↓
# rear_ground_homography
#       ↓
# 能得到合理的前方地面座標
#
MIN_FORWARD_M = 0.05
MAX_FORWARD_M = 4.00
MAX_ABS_LEFT_M = 1.50

CALIBRATION_MARGIN_PX = 10.0


# ============================================================
# 5. PPO observation contract
# ============================================================

OBS_DIM = 55

NUM_LIDAR_RAYS = 48
LIDAR_MAX_M = 4.0

# 只修改送進 PPO 的 LiDAR 距離，不修改外部安全層使用的 raw /scan。
# > 1.0 代表讓 PPO 覺得障礙物稍微更遠，降低過早左右避障的傾向。
# 48-ray observation 維度完全不變，所以不需要重訓 observation shape。
# 注意：這會讓 observation 和訓練分布有些差異，因此先從 1.25 做保守測試。
PPO_LIDAR_DISTANCE_SCALE = 1.15

# index 0  = right  -90°
# index 47 = left   +90°
#
POLICY_ANGLES_DEG = np.linspace(
    -90.0,
    90.0,
    NUM_LIDAR_RAYS,
    dtype=np.float32
)


# ============================================================
# 6. TG30 measured ROS mounting
# ============================================================

# /scan uses ROS CCW angles in frame "laser". The measured TF is
# base_link -> laser: yaw 180 deg, x +0.10 m, y 0. A rigid rotation
# preserves handedness; a front/back reflection swaps robot left/right.
# Explicit overrides describe a separately measured mounting, not a brake bypass.
LIDAR_GEOMETRY = LidarGeometry(
    yaw_offset_deg=float(os.environ.get("SUGARBOX_LIDAR_YAW_OFFSET_DEG", "180")),
    forward_offset_m=float(os.environ.get("SUGARBOX_LIDAR_FORWARD_OFFSET_M", "0.10")),
)


# ============================================================
# 7. PPO loop
# ============================================================

CONTROL_HZ = 6.0
CONTROL_PERIOD = 1.0 / CONTROL_HZ

SCAN_STALE_TIMEOUT = 0.30
ODOM_STALE_TIMEOUT = 0.50


# ============================================================
# 8. PPO action scaling
# ============================================================

# 原模型：
#
# if a0 >= 0:
#     vx = a0 * 0.5
# else:
#     vx = a0 * 0.15
#
# wz = a1 * 1.2
#

POLICY_MAX_FWD = 0.50
POLICY_MAX_REV = 0.15
POLICY_MAX_WZ = 1.20


# ============================================================
# 9. 第一次真車限速
# ============================================================

TEST_MAX_FWD = 0.15
TEST_MAX_REV = 0.08
TEST_MAX_WZ = 0.78


# ============================================================
# 10. Motor delay
# ============================================================

# 你 PPO 訓練紀錄中有：
#
# motor_delay_steps = 2
#
MOTOR_DELAY_STEPS = 0


# ============================================================
# 11. 到 sugar box 停車
# ============================================================

STOP_DISTANCE_M = 0.45  # legacy only; no longer used to declare ARRIVED before visual centering


# ============================================================
# 12. Dense raw LiDAR safety + anti-jitter
# ============================================================

# 這層不改 PPO observation。
# PPO 仍然吃原本 48-ray / 55D observation；
# 這裡只在 PPO 輸出之後做「最後一道真車安全層」。
#
# 之前實車抖動的三個主要來源：
#   1) 0.35 m 單一門檻硬切換
#   2) PPO wz 每幀快速變化 / 換向
#   3) Jetson 端最低馬達輸出 18，會把很小的 vx/wz 放大
#
# 因此這版加入：
#   - 前方距離 robust percentile（避免單一 LiDAR noise）
#   - 0.35 / 0.45 m hysteresis
#   - 0.80 -> 0.35 m 漸進降速
#   - PPO command EMA + slew-rate limit
#   - vx / wz deadzone hysteresis
#
# 額外安全層只看車頭較窄的 +/-20 deg，避免太早把側邊障礙當成正前方。
FRONT_SAFETY_HALF_ANGLE_DEG = 22.0

# 不用單一最小 ray 決定「一般減速 / block」。
# 取前方 sector 的第 5 percentile：
# 單一雜訊點不會立刻觸發，但真正障礙物佔多束時會反映出來。
FRONT_ROBUST_PERCENTILE = 5.0
FRONT_ROBUST_MIN_POINTS = 8

# Emergency hard stop 仍然使用 strict minimum，立即停車。
# hard stop 現在只留給非常近的緊急情況；exit threshold 避免邊界抖動。
FRONT_HARD_STOP_ENTER_M = 0.18
FRONT_HARD_STOP_EXIT_M = 0.22

# Forward block hysteresis：
# < 0.26 m 才禁止正向 vx；
# 必須 > 0.32 m 才解除。wz 仍交給 PPO。
FRONT_BLOCK_ENTER_M = 0.30
FRONT_BLOCK_EXIT_M = 0.36

# 只在 0.50 m 內才開始額外縮小正向速度；更遠完全不干預 PPO。
FRONT_SLOW_START_M = 0.55

# 近障礙物時只做「轉向方向防抖」，而且現在到 0.42 m 內才啟用。
# PPO 的 |wz| 大小仍然直接使用；只有左右方向若每幀反覆翻轉，
# 需要連續兩個 control tick 都要求新方向才允許切換。
# 這避免在障礙物前 LEFT/RIGHT/LEFT/RIGHT 原地抖很久，
# 又不會讓舊的 wz 大小殘留造成繞過後繼續轉。
AVOID_TURN_HOLD_ENTER_M = 0.45
AVOID_TURN_HOLD_EXIT_M = 0.55
AVOID_TURN_MIN_ABS_WZ = 0.12
AVOID_TURN_SWITCH_CONFIRM_TICKS = 2

# ============================================================
# 12b. 250-degree side-clearance safety (outside PPO)
# ============================================================
# PPO observation stays exactly the trained 48 rays over -90..+90 deg.
# We only widen the *external safety layer* to -125..+125 deg so obstacles
# that slide alongside the chassis remain visible after they leave the
# narrow front sector.
SIDE_SAFETY_MAX_ANGLE_DEG = 125.0
SIDE_SAFETY_FRONT_EXCLUDE_DEG = 25.0
SIDE_ROBUST_PERCENTILE = 5.0
SIDE_ROBUST_MIN_POINTS = 8

# If an obstacle is this close on one side, do not allow PPO to keep
# steering farther toward that side. Hysteresis avoids edge chatter.
SIDE_TURN_GUARD_ENTER_M = 0.20
SIDE_TURN_GUARD_EXIT_M = 0.24

# If the side clearance becomes very small, also cap forward speed.
# This is intentionally a soft cap rather than a full stop.
SIDE_VX_SLOW_ENTER_M = 0.16
SIDE_VX_CAP_MPS = 0.06

# ============================================================
# 12c. Hard-stop simple reverse recovery
# ============================================================
# 只使用可信的 LiDAR 範圍（-125..+125 deg）判斷前方/側邊。
# 車後方被車體遮住，因此 recovery 不使用任何 rear LiDAR。
#
# 行為刻意保持非常簡單：
#   HARD_STOP 先原地停住 0.30 s
#   -> 若仍然 HARD_STOP，倒退約 6 cm
#   -> 停 0.20 s
#   -> 控制權交回 PPO
#
# 同一次 HARD_STOP episode 最多只自動倒退一次；如果倒完仍然卡住，
# 就保持停止，不會連續瘋狂後退。必須等 HARD_STOP 真正解除後，
# 之後再次進入新的 HARD_STOP 才能再觸發一次 recovery。
HARD_RECOVERY_WAIT_S = 0.30
HARD_RECOVERY_REVERSE_VX = -0.08
HARD_RECOVERY_TARGET_M = 0.06
HARD_RECOVERY_MAX_S = 1.50
HARD_RECOVERY_SETTLE_S = 0.20


# ============================================================
# 13. PPO output path: NO temporal smoothing
# ============================================================

# 這版刻意取消 Windows 端所有 PPO command temporal filter：
#   - 不做 EMA
#   - 不做 slew-rate limit
#   - 不做 vx / wz deadzone hysteresis
#   - 不做 BLOCK_RELEASE / WZ reversal sync
#
# PPO action 經 action_to_velocity() 後，除了 LiDAR safety
# （slow / forward block / hard stop）之外，直接送到馬達。
# 目的是恢復原本 PPO 的即時避障反應；即使有些抖動也先接受。


# ============================================================
# 14. Bottom-of-frame FINAL VISUAL CENTERING
# ============================================================
# sugar box 接近畫面底部後，最後置中改成連續 pixel P-control：
#
#   error_px = bbox_center_x - image_center_x
#   wz       = -Kp * error_px
#
# 不再用 odom yaw，也不再使用 TURN PULSE / WAIT / REPULSE。
# 每次有新的 YOLO/SAM vision 結果就直接更新 wz；兩次 vision 間維持
# 上一次的轉向命令。bbox 越偏，轉得越快；進入中心 tolerance 後立即停。
#
# 水平置中使用 YOLO bbox center，而不是 SAM bottom-contact u，因為前者
# 對「物體在畫面左/右」通常更穩定。SAM bottom 仍保留給 Homography / 距離。

# bbox bottom 超過這個比例才進 final centering。
BOTTOM_CENTER_TRIGGER_V_RATIO = 0.78

# 640px 寬時，中心 ±30px 視為置中。
VISUAL_CENTER_TOL_PX = 30.0

# pixel proportional controller。
# 例如偏 150px -> 0.675rad/s；偏 100px -> 0.45，但會受 minimum 限制。
VISUAL_CENTER_KP = 0.0045

# 因實車有靜摩擦，非零旋轉不能降得太小；最大值則避免大幅甩動。
VISUAL_CENTER_MIN_WZ = 0.50
VISUAL_CENTER_MAX_WZ = 0.78

# 單次 vision 的 bbox center 若瞬間跳太多，先停止並等下一張，避免反向亂甩。
VISUAL_CENTER_MAX_JUMP_PX = 180.0

# 連續幾張 fresh vision 都進中心才算完成。
VISUAL_CENTER_CONFIRM_FRAMES = 2

# FINAL visual centering 最長時間，異常時保持停止。
VISUAL_CENTER_TIMEOUT_S = 8.0

# CENTER 狀態機本身跑快一點。真正的 wz 只在 fresh vision result 時更新，
# 但兩張 vision 之間會持續送上一次的命令，因此不再有 pulse 卡頓。
CENTER_CONTROL_HZ = 20.0
CENTER_CONTROL_PERIOD = 1.0 / CENTER_CONTROL_HZ

# ============================================================
# 15. Final forward after visual centering
# ============================================================
# 畫面真正置中後，再往前約 15 cm，完成後才設定 ARRIVED_LATCHED。
# 這裡不再用 Homography 距離當作提前到站條件，避免明明還沒到畫面底部
# 就因 dist <= 0.45 m 被提前鎖死。
FINAL_APPROACH_VX = 0.08
FINAL_APPROACH_TRAVEL_M = float(os.environ.get("SUGARBOX_FINAL_TRAVEL_M", "0.15"))
if not math.isfinite(FINAL_APPROACH_TRAVEL_M) or not 0.0 < FINAL_APPROACH_TRAVEL_M <= 0.15:
    raise ValueError("SUGARBOX_FINAL_TRAVEL_M must be in (0, 0.15]")
EXIT_ON_ARRIVAL = os.environ.get("SUGARBOX_EXIT_ON_ARRIVAL", "0") == "1"
FINAL_APPROACH_MAX_DURATION_S = 3.5


# ============================================================
# 16. Motor command
# ============================================================

MOTOR_SEND_INTERVAL = 0.10


# ============================================================
# 17. Display
# ============================================================

PRINT_INTERVAL = 0.50

# HUD 不需要跟主迴圈一起跑到數百 Hz。限制到 15 FPS，
# 避免 cv2 copy / draw / imshow 反過來拖慢 ROS callback。
DISPLAY_HZ = 30.0
DISPLAY_PERIOD = 1.0 / DISPLAY_HZ

SHOW_MASK = True
MASK_ALPHA = 0.30


# ============================================================
# 工具
# ============================================================

def clamp(x, lo, hi):
    return max(
        lo,
        min(hi, x)
    )


def wrap_pi(angle):
    return (
        angle + math.pi
    ) % (
        2.0 * math.pi
    ) - math.pi


def get_age(now, stamp):

    if stamp <= 0.0:
        return float("inf")

    return now - stamp


# ============================================================
# Camera Reader
# ============================================================

class LatestFrameReader:

    def __init__(
        self,
        width=640,
        height=480,
        udp_port=5600,
        gst_exe=GSTREAMER_LAUNCH_EXE
    ):

        self.width = int(width)
        self.height = int(height)
        self.udp_port = int(udp_port)
        self.gst_exe = gst_exe

        self.frame_bytes = (
            self.width
            * self.height
            * 3
        )

        if not os.path.exists(
            self.gst_exe
        ):
            raise RuntimeError(
                "找不到 Windows GStreamer："
                f"{self.gst_exe}\n"
                "可用環境變數 GSTREAMER_LAUNCH_EXE 指定 gst-launch-1.0.exe。"
            )

        # GStreamer 不經 OpenCV backend。
        # fdsink 將解碼後的 BGR raw frame 寫到 stdout，
        # Python reader thread 持續把 pipe 讀乾淨並只保存最後一張。
        #
        # queue leaky=downstream + max-size-buffers=1：
        # 若下游瞬間跟不上，舊 frame 優先丟棄，不累積控制延遲。
        gst_args = [
            self.gst_exe,
            "-q",
            "udpsrc",
            f"port={self.udp_port}",
            "caps=application/x-rtp,media=video,encoding-name=H264,payload=96",
            "!",
            "rtph264depay",
            "!",
            "h264parse",
            "!",
            "avdec_h264",
            "!",
            "videoconvert",
            "!",
            f"video/x-raw,format=BGR,width={self.width},height={self.height}",
            "!",
            "queue",
            "max-size-buffers=1",
            "max-size-bytes=0",
            "max-size-time=0",
            "leaky=downstream",
            "!",
            "fdsink",
            "fd=1",
            "sync=false",
        ]

        print(
            "[CAMERA] GStreamer UDP/H264 receiver: "
            f"port={self.udp_port}"
        )

        print(
            "[CAMERA] gst-launch = "
            f"{self.gst_exe}"
        )

        self.process = subprocess.Popen(
            gst_args,
            stdout=subprocess.PIPE,
            # -q 只會在錯誤時往 stderr 印訊息；保留它方便現場除錯。
            stderr=None,
            bufsize=0,
        )

        if self.process.stdout is None:
            raise RuntimeError(
                "GStreamer stdout pipe 建立失敗"
            )

        self.lock = threading.Lock()

        self.latest_frame = None
        self.latest_time = 0.0
        self.frame_counter = 0

        self.running = True

        self.thread = threading.Thread(
            target=self._loop,
            daemon=True
        )

        self.thread.start()


    def _read_exact(
        self,
        size
    ):

        data = bytearray()

        while (
            self.running
            and
            len(data) < size
        ):

            chunk = self.process.stdout.read(
                size - len(data)
            )

            if not chunk:
                return None

            data.extend(
                chunk
            )

        if len(data) != size:
            return None

        return bytes(
            data
        )


    def _loop(self):

        while self.running:

            raw = self._read_exact(
                self.frame_bytes
            )

            if raw is None:
                if self.running:
                    time.sleep(
                        0.01
                    )
                continue

            frame = np.frombuffer(
                raw,
                dtype=np.uint8
            ).reshape(
                self.height,
                self.width,
                3
            )

            receive_time = time.time()

            with self.lock:

                # frame 由 immutable bytes backing，下一幀會建立新物件；
                # 這裡不需要再 copy 0.9 MB。
                self.latest_frame = frame
                self.latest_time = receive_time
                self.frame_counter += 1


    def read(self):

        with self.lock:

            if self.latest_frame is None:

                return (
                    False,
                    None,
                    0.0
                )

            return (
                True,
                self.latest_frame,
                self.latest_time
            )


    def release(self):

        self.running = False

        try:
            if self.process.stdout is not None:
                self.process.stdout.close()
        except Exception:
            pass

        try:
            self.process.terminate()
        except Exception:
            pass

        try:
            self.process.wait(
                timeout=1.0
            )
        except Exception:
            try:
                self.process.kill()
            except Exception:
                pass

        try:
            self.thread.join(
                timeout=0.5
            )
        except Exception:
            pass


# ============================================================
# Homography JSON
# ============================================================

def inside_calibration_hull(u, v, hull):
    return _inside_calibration_hull(u, v, hull, margin_px=CALIBRATION_MARGIN_PX)


# ============================================================
# YOLO
# ============================================================

def select_best_box(
    yolo_result,
    model_names
):

    boxes = yolo_result.boxes

    if boxes is None:
        return None

    if len(boxes) == 0:
        return None

    detections = []

    for box in boxes:

        conf = float(

            box.conf[0]
            .detach()
            .cpu()
            .item()
        )

        cls_id = int(

            box.cls[0]
            .detach()
            .cpu()
            .item()
        )

        xyxy = (

            box.xyxy[0]
            .detach()
            .cpu()
            .numpy()
            .astype(float)
        )

        (
            x1,
            y1,
            x2,
            y2
        ) = xyxy.tolist()

        if isinstance(
            model_names,
            dict
        ):

            class_name = str(
                model_names.get(
                    cls_id,
                    cls_id
                )
            )

        elif (
            isinstance(
                model_names,
                (
                    list,
                    tuple
                )
            )
            and
            cls_id < len(
                model_names
            )
        ):

            class_name = str(
                model_names[
                    cls_id
                ]
            )

        else:

            class_name = str(
                cls_id
            )

        area = (
            max(
                1.0,
                x2 - x1
            )
            *
            max(
                1.0,
                y2 - y1
            )
        )

        detections.append({

            "conf": conf,

            "cls_id": cls_id,

            "class_name": class_name,

            "bbox": (
                x1,
                y1,
                x2,
                y2
            ),

            "area": area
        })

    detections.sort(

        key=lambda d: (
            d["conf"],
            d["area"]
        ),

        reverse=True
    )

    return detections[0]


# ============================================================
# SAM2
# ============================================================

def run_sam2_with_bbox(
    sam_model,
    frame,
    bbox
):

    (
        x1,
        y1,
        x2,
        y2
    ) = bbox

    prompt = [

        int(
            round(x1)
        ),

        int(
            round(y1)
        ),

        int(
            round(x2)
        ),

        int(
            round(y2)
        )
    ]

    results = sam_model(

        frame,

        bboxes=[
            prompt
        ],

        verbose=False
    )

    if results is None:
        return None

    if len(results) == 0:
        return None

    result = results[0]

    if result.masks is None:
        return None

    if result.masks.data is None:
        return None

    masks = (

        result.masks.data
        .detach()
        .cpu()
        .numpy()
    )

    if len(masks) == 0:
        return None

    best_mask = None
    best_area = -1

    for mask in masks:

        mask = mask.astype(
            np.float32
        )

        area = int(

            np.count_nonzero(
                mask > 0.5
            )
        )

        if area > best_area:

            best_area = area
            best_mask = mask

    if best_mask is None:
        return None

    if (
        best_mask.shape[0]
        !=
        frame.shape[0]
        or
        best_mask.shape[1]
        !=
        frame.shape[1]
    ):

        best_mask = cv2.resize(

            best_mask,

            (
                frame.shape[1],
                frame.shape[0]
            ),

            interpolation=cv2.INTER_NEAREST
        )

    return best_mask


# ============================================================
# SAM mask 最底部接地點
# ============================================================

def mask_bottom_contact(
    mask
):

    mask_bool = (
        mask > 0.5
    )

    ys, xs = np.where(
        mask_bool
    )

    if len(xs) < MIN_MASK_PIXELS:
        return None

    y_max = int(
        np.max(ys)
    )

    band_min_y = max(
        0,
        y_max
        -
        BOTTOM_BAND_PX
        +
        1
    )

    use = (
        ys >= band_min_y
    )

    bottom_xs = xs[
        use
    ]

    bottom_ys = ys[
        use
    ]

    if len(
        bottom_xs
    ) < MIN_BOTTOM_PIXELS:

        return None

    u = float(
        np.median(
            bottom_xs
        )
    )

    v = float(
        np.max(
            bottom_ys
        )
    )

    return (
        u,
        v,
        len(bottom_xs)
    )


# ============================================================
# 完整 Vision Pipeline
# ============================================================

def vision_to_target(

    frame,

    yolo_model,

    sam_model,

    H,

    rear_cam_to_front_m,

    calibration_hull
):

    # --------------------------------------------------------
    # YOLO
    # --------------------------------------------------------

    yolo_results = yolo_model.predict(

        frame,

        imgsz=YOLO_IMGSZ,

        conf=YOLO_CONF,

        verbose=False
    )

    if not yolo_results:

        return (
            None,
            "NO_YOLO"
        )

    detection = select_best_box(

        yolo_results[0],

        yolo_model.names
    )

    if detection is None:

        return (
            None,
            "NO_YOLO"
        )

    bbox = detection[
        "bbox"
    ]

    (
        x1,
        y1,
        x2,
        y2
    ) = bbox


    # --------------------------------------------------------
    # SAM2
    # --------------------------------------------------------

    mask = run_sam2_with_bbox(

        sam_model,

        frame,

        bbox
    )

    if mask is None:

        return (
            {
                "det": detection,
                "mask": None
            },
            "SAM_FAILED"
        )


    # --------------------------------------------------------
    # SAM bottom point
    # --------------------------------------------------------

    contact = mask_bottom_contact(
        mask
    )

    if contact is None:

        return (
            {
                "det": detection,
                "mask": mask
            },
            "MASK_INVALID"
        )

    (
        u,
        v,
        bottom_count
    ) = contact


    # --------------------------------------------------------
    # SAM bottom 與 YOLO bottom consistency
    # --------------------------------------------------------

    if abs(
        v - y2
    ) > MAX_SAM_BOTTOM_TO_BBOX_BOTTOM_PX:

        return (
            {
                "det": detection,
                "mask": mask,
                "u": u,
                "v": v,
                "bottom_count": bottom_count
            },
            "BOTTOM_MISMATCH"
        )


    # --------------------------------------------------------
    # Homography calibration area
    # --------------------------------------------------------

    if not inside_calibration_hull(

        u,
        v,
        calibration_hull
    ):

        return (
            {
                "det": detection,
                "mask": mask,
                "u": u,
                "v": v,
                "bottom_count": bottom_count
            },
            "OUTSIDE_CALIBRATION"
        )


    # --------------------------------------------------------
    # Pixel → ground
    # --------------------------------------------------------

    ground = pixel_to_ground(

        u,
        v,
        H
    )

    if ground is None:

        return (
            {
                "det": detection,
                "mask": mask,
                "u": u,
                "v": v,
                "bottom_count": bottom_count
            },
            "H_INVALID"
        )

    (
        x_rear,
        y_left
    ) = ground


    # --------------------------------------------------------
    # rear camera → front/base reference
    # --------------------------------------------------------

    x_forward = (
        x_rear
        -
        rear_cam_to_front_m
    )


    # --------------------------------------------------------
    # Floor / geometry validity
    # --------------------------------------------------------

    if x_forward < MIN_FORWARD_M:

        on_floor = False
        reason = "TOO_CLOSE_OR_BEHIND"

    elif x_forward > MAX_FORWARD_M:

        on_floor = False
        reason = "TOO_FAR"

    elif abs(
        y_left
    ) > MAX_ABS_LEFT_M:

        on_floor = False
        reason = "LATERAL_OUT_OF_RANGE"

    else:

        on_floor = True
        reason = "VALID"


    distance = math.hypot(
        x_forward,
        y_left
    )

    bearing = math.atan2(
        y_left,
        x_forward
    )


    result = {

        "det": detection,

        "mask": mask,

        "u": u,

        "v": v,

        "bottom_count": bottom_count,

        "on_floor": on_floor,

        "x": float(
            x_forward
        ),

        "y": float(
            y_left
        ),

        "dist": float(
            distance
        ),

        "bearing": float(
            bearing
        ),

        "x_rear": float(
            x_rear
        )
    }

    return (
        result,
        reason
    )


def apply_latency_compensation_to_vision_result(
    vision_result,
    frame_receive_time,
    vision_start_time,
    vision_done_time,
    ros_bridge
):

    """
    Compensate the target geometry from an estimated camera capture
    time to the moment inference finishes.

    Important limitation:
    RTP/H.264 subprocess reader gives us the Windows decoded-frame receive time,
    not the original camera exposure timestamp. CAMERA_STREAM_DELAY_COMP_S is
    therefore a small adjustable estimate for encode/network/decode transport delay.
    """

    info = {
        "queue_s": max(0.0, vision_start_time - frame_receive_time),
        "infer_s": max(0.0, vision_done_time - vision_start_time),
        "stream_assumed_s": max(0.0, CAMERA_STREAM_DELAY_COMP_S),
        "total_est_s": 0.0,
        "odom_comp_s": 0.0,
        "full_coverage": False
    }

    measurement_time = (
        frame_receive_time
        -
        max(0.0, CAMERA_STREAM_DELAY_COMP_S)
    )

    info["total_est_s"] = max(
        0.0,
        vision_done_time - measurement_time
    )

    if vision_result is None:
        return (vision_result, info)

    if not vision_result.get(
        "on_floor",
        False
    ):
        return (vision_result, info)

    if ros_bridge is None:
        return (vision_result, info)

    raw_x = float(vision_result["x"])
    raw_y = float(vision_result["y"])

    (
        comp_x,
        comp_y,
        used_s,
        full_coverage
    ) = ros_bridge.compensate_relative_point(
        raw_x,
        raw_y,
        measurement_time,
        vision_done_time
    )

    vision_result["raw_x_before_latency_comp"] = raw_x
    vision_result["raw_y_before_latency_comp"] = raw_y
    vision_result["raw_dist_before_latency_comp"] = math.hypot(
        raw_x,
        raw_y
    )
    vision_result["raw_bearing_before_latency_comp"] = math.atan2(
        raw_y,
        raw_x
    )

    vision_result["x"] = float(comp_x)
    vision_result["y"] = float(comp_y)
    vision_result["dist"] = float(
        math.hypot(
            comp_x,
            comp_y
        )
    )
    vision_result["bearing"] = float(
        math.atan2(
            comp_y,
            comp_x
        )
    )

    info["odom_comp_s"] = float(used_s)
    info["full_coverage"] = bool(full_coverage)

    return (vision_result, info)



# ============================================================
# Asynchronous Vision Worker
# ============================================================
# Camera preview/control loop must never wait for YOLO + SAM2.
# The worker always grabs the newest decoded frame, performs inference in
# the background, and publishes only the newest completed result.
#
# This is intentionally separate from LatestFrameReader:
#   Camera thread  : ~30 FPS, always drains UDP/raw pipe
#   Vision worker  : ~5 Hz, YOLO + SAM2
#   Main/control   : 6 Hz PPO + ROS safety + live preview
#
# Therefore a 160-200 ms SAM2 inference no longer freezes the displayed
# camera or the main control loop.

class AsyncVisionWorker:

    def __init__(
        self,
        reader,
        yolo_model,
        sam_model,
        H,
        rear_cam_to_front_m,
        calibration_hull,
        ros_bridge=None
    ):
        self.reader = reader
        self.yolo_model = yolo_model
        self.sam_model = sam_model
        self.H = H
        self.rear_cam_to_front_m = rear_cam_to_front_m
        self.calibration_hull = calibration_hull
        self.ros_bridge = ros_bridge

        self.lock = threading.Lock()
        self.running = True

        self.sequence = 0
        self.result = None
        self.reason = "WAIT"
        self.done_time = 0.0
        self.frame_receive_time = 0.0
        self.vision_frame = None
        self.latency_info = {
            "queue_s": 0.0,
            "infer_s": 0.0,
            "stream_assumed_s": CAMERA_STREAM_DELAY_COMP_S,
            "total_est_s": 0.0,
            "odom_comp_s": 0.0,
            "full_coverage": False,
        }

        self.last_processed_frame_time = 0.0
        self.last_inference_start = 0.0

        self.thread = threading.Thread(
            target=self._loop,
            daemon=True,
        )
        self.thread.start()

    def _loop(self):
        while self.running:
            now = time.time()

            if now - self.last_inference_start < VISION_INTERVAL:
                time.sleep(0.002)
                continue

            ok, frame, frame_time = self.reader.read()

            if not ok or frame is None:
                time.sleep(0.005)
                continue

            # Never process the same decoded frame twice.
            if frame_time <= self.last_processed_frame_time + 1e-6:
                time.sleep(0.002)
                continue

            # Lock this frame object for the inference call. LatestFrameReader
            # replaces its reference for newer frames; it does not mutate this one.
            self.last_processed_frame_time = frame_time
            vision_start_time = time.time()
            self.last_inference_start = vision_start_time

            try:
                vision_result, vision_reason = vision_to_target(
                    frame,
                    self.yolo_model,
                    self.sam_model,
                    self.H,
                    self.rear_cam_to_front_m,
                    self.calibration_hull,
                )
            except Exception as exc:
                vision_result = None
                vision_reason = f"VISION_ERROR:{type(exc).__name__}"

            vision_done_time = time.time()

            try:
                vision_result, latency_info = apply_latency_compensation_to_vision_result(
                    vision_result,
                    frame_time,
                    vision_start_time,
                    vision_done_time,
                    self.ros_bridge,
                )
            except Exception:
                latency_info = {
                    "queue_s": max(0.0, vision_start_time - frame_time),
                    "infer_s": max(0.0, vision_done_time - vision_start_time),
                    "stream_assumed_s": max(0.0, CAMERA_STREAM_DELAY_COMP_S),
                    "total_est_s": max(
                        0.0,
                        vision_done_time - (frame_time - CAMERA_STREAM_DELAY_COMP_S),
                    ),
                    "odom_comp_s": 0.0,
                    "full_coverage": False,
                }

            with self.lock:
                self.sequence += 1
                self.result = vision_result
                self.reason = vision_reason
                self.done_time = vision_done_time
                self.frame_receive_time = frame_time
                # Keep the exact frame used for this inference result.
                # This lets the debug window draw bbox/mask on the matching
                # historical frame instead of on the newest live frame.
                self.vision_frame = frame
                self.latency_info = latency_info

    def snapshot(self):
        with self.lock:
            # result/mask are replaced atomically and only read by main thread.
            return (
                self.sequence,
                self.result,
                self.reason,
                self.done_time,
                self.frame_receive_time,
                self.vision_frame,
                dict(self.latency_info),
            )

    def release(self):
        self.running = False
        try:
            self.thread.join(timeout=1.0)
        except Exception:
            pass


# ============================================================
# ROS sensor states
# ============================================================

@dataclass
class ScanState:

    ranges: Optional[
        np.ndarray
    ] = None

    angle_min: float = 0.0

    angle_increment: float = 0.0

    range_min: float = 0.0

    range_max: float = 0.0

    stamp: float = 0.0


@dataclass
class OdomState:

    vx: float = 0.0

    wz: float = 0.0

    # odom pose yaw。Final centering 用這個直接量「已經轉了多少角度」，
    # 不再等 bbox / SAM 的下一次結果來決定何時停止。
    yaw: float = 0.0

    stamp: float = 0.0


# ============================================================
# ROSBridge
# ============================================================

class RosSensorBridge:

    def __init__(
        self,
        host,
        port
    ):

        self.lock = threading.Lock()

        self.scan = ScanState()

        self.odom = OdomState()

        # Windows local-time history of odom feedback.
        # 用來把 vision 在「較舊 frame」得到的 target 座標
        # propagate 到 inference 完成時的當下。
        self.odom_history = deque()

        self.client = roslibpy.Ros(

            host=host,

            port=port
        )

        self.scan_topic = None

        self.odom_topic = None


    def start(self):

        print(
            f"[ROS] Connecting "
            f"ws://{JETSON_IP}:{ROSBRIDGE_PORT}"
        )

        self.client.run()

        if not self.client.is_connected:

            raise RuntimeError(
                "rosbridge connection failed"
            )

        self.scan_topic = roslibpy.Topic(

            self.client,

            SCAN_TOPIC,

            "sensor_msgs/LaserScan"
        )

        self.odom_topic = roslibpy.Topic(

            self.client,

            ODOM_TOPIC,

            "nav_msgs/Odometry"
        )

        self.scan_topic.subscribe(
            self._scan_cb
        )

        self.odom_topic.subscribe(
            self._odom_cb
        )

        print(
            f"[ROS] subscribed: "
            f"{SCAN_TOPIC}, "
            f"{ODOM_TOPIC}"
        )


    def _scan_cb(
        self,
        msg
    ):

        try:

            ranges = np.asarray(

                msg.get(
                    "ranges",
                    []
                ),

                dtype=np.float32
            )

            angle_min = float(

                msg.get(
                    "angle_min",
                    0.0
                )
            )

            angle_increment = float(

                msg.get(
                    "angle_increment",
                    0.0
                )
            )

            range_min = float(

                msg.get(
                    "range_min",
                    0.0
                )
            )

            range_max = float(

                msg.get(
                    "range_max",
                    0.0
                )
            )

        except Exception:
            return

        if ranges.size == 0:
            return

        if angle_increment == 0.0:
            return

        with self.lock:

            self.scan = ScanState(

                ranges=ranges,

                angle_min=angle_min,

                angle_increment=angle_increment,

                range_min=range_min,

                range_max=range_max,

                stamp=time.time()
            )


    def _odom_cb(
        self,
        msg
    ):

        try:

            twist = msg[
                "twist"
            ][
                "twist"
            ]

            vx = float(
                twist[
                    "linear"
                ][
                    "x"
                ]
            )

            wz = float(
                twist[
                    "angular"
                ][
                    "z"
                ]
            )

            q = msg[
                "pose"
            ][
                "pose"
            ][
                "orientation"
            ]

            qx = float(q.get("x", 0.0))
            qy = float(q.get("y", 0.0))
            qz = float(q.get("z", 0.0))
            qw = float(q.get("w", 1.0))

            yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz)
            )

        except Exception:
            return

        stamp = time.time()

        with self.lock:

            self.odom = OdomState(

                vx=vx,

                wz=wz,

                yaw=yaw,

                stamp=stamp
            )

            self.odom_history.append(
                (
                    stamp,
                    float(vx),
                    float(wz)
                )
            )

            cutoff = stamp - ODOM_HISTORY_S

            while (
                self.odom_history
                and
                self.odom_history[0][0] < cutoff
            ):
                self.odom_history.popleft()


    @staticmethod
    def _propagate_relative_point_once(
        x,
        y,
        vx,
        wz,
        dt
    ):

        if dt <= 0.0:
            return (float(x), float(y))

        vx = float(vx)
        wz = float(wz)
        dt = float(dt)

        dtheta = wz * dt

        # Robot displacement expressed in the robot frame at the
        # beginning of this small interval.
        if abs(wz) < 1e-6:
            dx = vx * dt
            dy = 0.0
        else:
            radius = vx / wz
            dx = radius * math.sin(dtheta)
            dy = radius * (1.0 - math.cos(dtheta))

        qx = float(x) - dx
        qy = float(y) - dy

        c = math.cos(dtheta)
        s = math.sin(dtheta)

        # Target coordinates in the new robot frame = R(-dtheta) * q
        x_new = c * qx + s * qy
        y_new = -s * qx + c * qy

        return (
            float(x_new),
            float(y_new)
        )


    def compensate_relative_point(
        self,
        x,
        y,
        from_time,
        to_time
    ):

        """
        Use odom history to transform a target point measured in an
        older robot frame into the current robot frame.

        Returns:
            x_now, y_now, used_seconds, full_coverage
        """

        from_time = float(from_time)
        to_time = float(to_time)

        if to_time <= from_time:
            return (float(x), float(y), 0.0, True)

        # Avoid propagating an obviously stale / bogus frame for too long.
        if to_time - from_time > MAX_VISION_COMPENSATION_S:
            from_time = to_time - MAX_VISION_COMPENSATION_S
            full_coverage = False
        else:
            full_coverage = True

        with self.lock:
            history = list(self.odom_history)

        if not history:
            return (float(x), float(y), 0.0, False)

        oldest_t = history[0][0]
        newest_t = history[-1][0]

        start_t = from_time

        if start_t < oldest_t:
            start_t = oldest_t
            full_coverage = False

        if start_t >= to_time:
            return (float(x), float(y), 0.0, False)

        # Velocity state valid at start_t: last sample at/before start_t.
        current_vx = history[0][1]
        current_wz = history[0][2]

        for stamp, vx_i, wz_i in history:
            if stamp <= start_t:
                current_vx = vx_i
                current_wz = wz_i
            else:
                break

        px = float(x)
        py = float(y)
        t = start_t

        for stamp, vx_i, wz_i in history:
            if stamp <= start_t:
                continue

            if stamp > to_time:
                break

            dt = stamp - t

            if dt > 0.0:
                px, py = self._propagate_relative_point_once(
                    px,
                    py,
                    current_vx,
                    current_wz,
                    dt
                )

            t = stamp
            current_vx = vx_i
            current_wz = wz_i

        if t < to_time:
            px, py = self._propagate_relative_point_once(
                px,
                py,
                current_vx,
                current_wz,
                to_time - t
            )

        if newest_t < to_time - ODOM_STALE_TIMEOUT:
            full_coverage = False

        return (
            float(px),
            float(py),
            float(to_time - start_t),
            bool(full_coverage)
        )


    def snapshot(
        self
    ):

        with self.lock:

            scan = ScanState(

                ranges=(
                    None
                    if self.scan.ranges is None
                    else self.scan.ranges.copy()
                ),

                angle_min=self.scan.angle_min,

                angle_increment=self.scan.angle_increment,

                range_min=self.scan.range_min,

                range_max=self.scan.range_max,

                stamp=self.scan.stamp
            )

            odom = OdomState(

                vx=self.odom.vx,

                wz=self.odom.wz,

                yaw=self.odom.yaw,

                stamp=self.odom.stamp
            )

        return (
            scan,
            odom
        )


    def close(
        self
    ):

        try:

            if self.scan_topic is not None:

                self.scan_topic.unsubscribe()

        except Exception:
            pass

        try:

            if self.odom_topic is not None:

                self.odom_topic.unsubscribe()

        except Exception:
            pass

        try:

            self.client.terminate()

        except Exception:
            pass


# ============================================================
# TG30 angle -> scan index
# ============================================================

def angle_to_scan_index(

    target_raw_angle,

    scan
):

    n = len(
        scan.ranges
    )

    best_idx = None

    best_error = float(
        "inf"
    )

    # 支援：
    #
    # [-pi, pi]
    # [0, 2pi]
    # 或其他等價 angle range
    #
    for angle in (

        target_raw_angle,

        target_raw_angle
        +
        2.0
        *
        math.pi,

        target_raw_angle
        -
        2.0
        *
        math.pi
    ):

        idx_float = (

            angle
            -
            scan.angle_min

        ) / scan.angle_increment

        idx = int(
            round(
                idx_float
            )
        )

        if not (
            0 <= idx < n
        ):
            continue

        actual_angle = (

            scan.angle_min
            +
            idx
            *
            scan.angle_increment
        )

        error = abs(
            actual_angle
            -
            angle
        )

        if error < best_error:

            best_idx = idx
            best_error = error

    return best_idx


def sanitize_policy_range(
    value
):

    if not math.isfinite(
        value
    ):

        return LIDAR_MAX_M

    if value <= 0.0:

        return LIDAR_MAX_M

    return float(

        min(
            value,
            LIDAR_MAX_M
        )
    )


# ============================================================
# /scan -> 48 rays
# ============================================================

def scan_to_policy_rays(scan):
    if scan.ranges is None or len(scan.ranges) == 0:
        return None
    samples = robot_scan_samples(
        scan.ranges, scan.angle_min, scan.angle_increment, LIDAR_GEOMETRY)
    return np.asarray(nearest_policy_ranges(
        samples, POLICY_ANGLES_DEG, max_range_m=LIDAR_MAX_M,
        distance_scale=PPO_LIDAR_DISTANCE_SCALE), dtype=np.float32)


# ============================================================
# Dense raw /scan front metrics
# ============================================================

def raw_front_metrics(scan):
    """Dense robot-frame front ranges; only PPO ranges receive scaling."""
    if scan.ranges is None or len(scan.ranges) == 0:
        return float("inf"), float("inf"), 0
    half_angle = math.radians(FRONT_SAFETY_HALF_ANGLE_DEG)
    values = [sample.range_m for sample in robot_scan_samples(
        scan.ranges, scan.angle_min, scan.angle_increment, LIDAR_GEOMETRY)
        if sample.range_m is not None and abs(sample.angle_rad) <= half_angle]
    if not values:
        return float("inf"), float("inf"), 0
    arr = np.asarray(values, dtype=np.float32)
    strict_min = float(np.min(arr))
    robust_distance = (strict_min if arr.size < FRONT_ROBUST_MIN_POINTS
                       else float(np.percentile(arr, FRONT_ROBUST_PERCENTILE)))
    return robust_distance, strict_min, int(arr.size)


def raw_side_metrics(scan):
    """Original side percentiles/guards, with robot x forward and y left."""
    if scan.ranges is None or len(scan.ranges) == 0:
        return (float("inf"), float("inf"), 0,
                float("inf"), float("inf"), 0)
    side_max = math.radians(SIDE_SAFETY_MAX_ANGLE_DEG)
    front_exclude = math.radians(SIDE_SAFETY_FRONT_EXCLUDE_DEG)
    right_values, left_values = [], []
    for sample in robot_scan_samples(
            scan.ranges, scan.angle_min, scan.angle_increment, LIDAR_GEOMETRY):
        if sample.range_m is None:
            continue
        if -side_max <= sample.angle_rad <= -front_exclude:
            right_values.append(sample.range_m)
        elif front_exclude <= sample.angle_rad <= side_max:
            left_values.append(sample.range_m)

    def summarize(values):
        if not values:
            return float("inf"), float("inf"), 0
        arr = np.asarray(values, dtype=np.float32)
        strict_min = float(np.min(arr))
        robust = (strict_min if arr.size < SIDE_ROBUST_MIN_POINTS
                  else float(np.percentile(arr, SIDE_ROBUST_PERCENTILE)))
        return robust, strict_min, int(arr.size)

    return (*summarize(right_values), *summarize(left_values))



# ============================================================
# Anti-jitter command filter / safety state
# ============================================================

class AntiJitterSafetyFilter:

    """
    名稱為了少改主程式而保留；這個 class 本身不做 PPO temporal smoothing。

    PPO action 經 action_to_velocity() 後直接進入這個 safety class。

    這個 safety class 只做：
      1) hard stop hysteresis
      2) forward-block hysteresis
      3) 接近障礙物時縮小正向 vx
      4) 近障礙物時的轉向方向確認
      5) 側邊安全 guard / hard-stop reverse recovery

    這裡不會用 EMA，也不會保存上一幀 vx/wz 做 command smoothing。
    """

    def __init__(self):
        self.forward_blocked = False
        self.hard_stopped = False

        # 250-degree side-clearance guard hysteresis.
        self.right_side_guard = False
        self.left_side_guard = False

        # HARD_STOP simple reverse recovery state.
        self.hard_stop_since = None
        self.recovery_active = False
        self.recovery_started_at = 0.0
        self.recovery_last_time = 0.0
        self.recovery_travel_m = 0.0
        self.recovery_settle_until = 0.0
        # 同一次 HARD_STOP episode 只允許倒退一次，避免連續瘋狂後退。
        self.recovery_used_this_hard_stop = False

        # 只在近障礙物時保存「轉向方向」。
        # 不保存 wz 大小，因此不是 smoothing，也不會有舊轉速殘留。
        self.avoid_turn_sign = 0.0
        self.avoid_switch_sign = 0.0
        self.avoid_switch_count = 0


    def reset_motion(self):
        # PPO command 不做 temporal smoothing；只清除近障礙物方向鎖。
        self.avoid_turn_sign = 0.0
        self.avoid_switch_sign = 0.0
        self.avoid_switch_count = 0


    def reset_all(self):
        self.forward_blocked = False
        self.hard_stopped = False
        self.right_side_guard = False
        self.left_side_guard = False
        self.hard_stop_since = None
        self.recovery_active = False
        self.recovery_started_at = 0.0
        self.recovery_last_time = 0.0
        self.recovery_travel_m = 0.0
        self.recovery_settle_until = 0.0
        self.recovery_used_this_hard_stop = False
        self.reset_motion()


    def _update_side_latches(
        self,
        right_robust,
        left_robust
    ):

        if self.right_side_guard:
            if (
                (not math.isfinite(right_robust))
                or
                right_robust >= SIDE_TURN_GUARD_EXIT_M
            ):
                self.right_side_guard = False
        else:
            if (
                math.isfinite(right_robust)
                and
                right_robust < SIDE_TURN_GUARD_ENTER_M
            ):
                self.right_side_guard = True

        if self.left_side_guard:
            if (
                (not math.isfinite(left_robust))
                or
                left_robust >= SIDE_TURN_GUARD_EXIT_M
            ):
                self.left_side_guard = False
        else:
            if (
                math.isfinite(left_robust)
                and
                left_robust < SIDE_TURN_GUARD_ENTER_M
            ):
                self.left_side_guard = True


    def _update_distance_latches(
        self,
        front_robust,
        front_min
    ):

        # Emergency stop hysteresis
        if self.hard_stopped:
            if front_min >= FRONT_HARD_STOP_EXIT_M:
                self.hard_stopped = False
        else:
            if front_min < FRONT_HARD_STOP_ENTER_M:
                self.hard_stopped = True

        # Forward-block hysteresis
        if self.forward_blocked:
            if front_robust >= FRONT_BLOCK_EXIT_M:
                self.forward_blocked = False
        else:
            if front_robust < FRONT_BLOCK_ENTER_M:
                self.forward_blocked = True


    def apply_center_turn(
        self,
        desired_wz,
        front_robust,
        front_min
    ):

        # Final centering 只受真正的 hard stop 限制。
        self._update_distance_latches(
            front_robust,
            front_min
        )

        if self.hard_stopped:
            return (
                0.0,
                0.0,
                (
                    "HARD_STOP_HOLD "
                    f"min={front_min:.3f}m"
                )
            )

        wz = clamp(
            float(desired_wz),
            -VISUAL_CENTER_MAX_WZ,
            VISUAL_CENTER_MAX_WZ
        )

        return (
            0.0,
            float(wz),
            (
                "CENTER_DIRECT "
                f"front={front_robust:.3f}m"
            )
        )


    def apply_final_approach(
        self,
        desired_vx,
        front_robust,
        front_min
    ):

        self._update_distance_latches(
            front_robust,
            front_min
        )

        if self.hard_stopped:
            return (
                0.0,
                0.0,
                (
                    "HARD_STOP_HOLD "
                    f"min={front_min:.3f}m"
                )
            )

        if self.forward_blocked:
            return (
                0.0,
                0.0,
                (
                    "FWD_BLOCK_HOLD "
                    f"front={front_robust:.3f}m "
                    f"release>{FRONT_BLOCK_EXIT_M:.2f}"
                )
            )

        vx = clamp(
            float(desired_vx),
            0.0,
            TEST_MAX_FWD
        )

        return (
            float(vx),
            0.0,
            (
                "FINAL_DIRECT "
                f"front={front_robust:.3f}m"
            )
        )


    def _stabilize_turn_direction(
        self,
        wz,
        front_robust
    ):
        """
        近障礙物專用的方向 hysteresis。

        - 不平滑 wz 大小。
        - 不延遲同方向的 PPO 變化。
        - 只有 PPO 左右方向反覆翻轉時，要求新方向連續出現
          AVOID_TURN_SWITCH_CONFIRM_TICKS 次才切換。
        """

        wz = float(wz)

        near_obstacle = (
            math.isfinite(front_robust)
            and
            front_robust < AVOID_TURN_HOLD_ENTER_M
        )

        clear_again = (
            (not math.isfinite(front_robust))
            or
            front_robust >= AVOID_TURN_HOLD_EXIT_M
        )

        if clear_again:
            self.reset_motion()
            return (wz, "")

        if not near_obstacle and self.avoid_turn_sign == 0.0:
            return (wz, "")

        if abs(wz) < AVOID_TURN_MIN_ABS_WZ:
            # PPO 幾乎沒有要求轉向時，不自行創造轉向。
            return (wz, "")

        requested_sign = 1.0 if wz > 0.0 else -1.0

        if self.avoid_turn_sign == 0.0:
            self.avoid_turn_sign = requested_sign
            self.avoid_switch_sign = 0.0
            self.avoid_switch_count = 0

            return (
                self.avoid_turn_sign * abs(wz),
                "TURN_HOLD_START=L" if self.avoid_turn_sign > 0.0 else "TURN_HOLD_START=R"
            )

        if requested_sign == self.avoid_turn_sign:
            self.avoid_switch_sign = 0.0
            self.avoid_switch_count = 0

            return (
                self.avoid_turn_sign * abs(wz),
                "TURN_HOLD=L" if self.avoid_turn_sign > 0.0 else "TURN_HOLD=R"
            )

        # PPO 想換方向：必須連續多個 tick 都要求同一個新方向。
        if requested_sign != self.avoid_switch_sign:
            self.avoid_switch_sign = requested_sign
            self.avoid_switch_count = 1
        else:
            self.avoid_switch_count += 1

        if self.avoid_switch_count >= AVOID_TURN_SWITCH_CONFIRM_TICKS:
            self.avoid_turn_sign = requested_sign
            self.avoid_switch_sign = 0.0
            self.avoid_switch_count = 0

            return (
                self.avoid_turn_sign * abs(wz),
                "TURN_SWITCH=L" if self.avoid_turn_sign > 0.0 else "TURN_SWITCH=R"
            )

        # 尚未確認換向：只鎖 sign，wz magnitude 仍使用本幀 PPO 值。
        return (
            self.avoid_turn_sign * abs(wz),
            (
                "TURN_SWITCH_WAIT "
                f"{self.avoid_switch_count}/{AVOID_TURN_SWITCH_CONFIRM_TICKS}"
            )
        )


    def apply(
        self,
        desired_vx,
        desired_wz,
        front_robust,
        front_min,
        right_robust=float("inf"),
        left_robust=float("inf"),
        odom_vx=0.0,
        now=None
    ):

        """
        PPO command -> LiDAR safety -> motor.

        HARD_STOP recovery（簡化版）：
          1) HARD_STOP 立即停止
          2) 必須連續維持 HARD_RECOVERY_WAIT_S（0.30 s）
          3) 才倒退一小段（odom 實際量約 6 cm）
          4) 停一下後把控制權交回 PPO
          5) 同一次 HARD_STOP episode 只退一次

        注意：後方 LiDAR 被車體遮住，因此完全不使用 rear scan。
        """

        if now is None:
            now = time.monotonic()

        now = float(now)

        self._update_distance_latches(
            front_robust,
            front_min
        )

        self._update_side_latches(
            right_robust,
            left_robust
        )

        # ----------------------------------------------------
        # Active reverse recovery
        # ----------------------------------------------------
        if self.recovery_active:

            dt = max(
                0.0,
                min(
                    0.25,
                    now - self.recovery_last_time
                )
            )
            self.recovery_last_time = now

            # 用實際 odom 倒車速度積分，靜摩擦時不會把「有送命令」誤當成已退距離。
            if math.isfinite(float(odom_vx)):
                self.recovery_travel_m += (
                    max(0.0, -float(odom_vx)) * dt
                )

            elapsed = now - self.recovery_started_at

            enough_distance = (
                self.recovery_travel_m >= HARD_RECOVERY_TARGET_M
            )
            timed_out = (
                elapsed >= HARD_RECOVERY_MAX_S
            )

            if enough_distance or timed_out:
                travelled = self.recovery_travel_m
                self.recovery_active = False
                self.recovery_settle_until = now + HARD_RECOVERY_SETTLE_S
                self.hard_stop_since = None
                self.reset_motion()

                return (
                    0.0,
                    0.0,
                    (
                        "HARD_RECOVERY_DONE "
                        f"travel={travelled:.3f}m "
                        + ("timeout " if timed_out and not enough_distance else "")
                        + f"settle={HARD_RECOVERY_SETTLE_S:.2f}s"
                    ),
                    0.0
                )

            return (
                float(HARD_RECOVERY_REVERSE_VX),
                0.0,
                (
                    "HARD_RECOVERY_BACKUP "
                    f"travel={self.recovery_travel_m:.3f}/"
                    f"{HARD_RECOVERY_TARGET_M:.3f}m "
                    f"t={elapsed:.2f}s"
                ),
                0.0
            )

        # Recovery 完成後先完全停一下，避免一退完立刻又被 PPO 指令拉走。
        if now < self.recovery_settle_until:
            return (
                0.0,
                0.0,
                (
                    "HARD_RECOVERY_SETTLE "
                    f"remain={self.recovery_settle_until - now:.2f}s"
                ),
                0.0
            )

        # HARD_STOP 已真的解除，才算新的 episode；之後若再撞近才可再退一次。
        if not self.hard_stopped:
            self.hard_stop_since = None
            self.recovery_used_this_hard_stop = False

        # ----------------------------------------------------
        # Hard stop: 必須連續卡住 0.30 s，之後只倒一次
        # ----------------------------------------------------
        if self.hard_stopped:

            if self.hard_stop_since is None:
                self.hard_stop_since = now

            hard_elapsed = now - self.hard_stop_since

            if (
                hard_elapsed >= HARD_RECOVERY_WAIT_S
                and
                not self.recovery_used_this_hard_stop
            ):
                self.recovery_active = True
                self.recovery_started_at = now
                self.recovery_last_time = now
                self.recovery_travel_m = 0.0
                self.recovery_used_this_hard_stop = True
                self.reset_motion()

                return (
                    float(HARD_RECOVERY_REVERSE_VX),
                    0.0,
                    (
                        "HARD_RECOVERY_START "
                        f"waited={hard_elapsed:.2f}s "
                        f"target={HARD_RECOVERY_TARGET_M:.2f}m"
                    ),
                    0.0
                )

            if self.recovery_used_this_hard_stop:
                reason = (
                    "HARD_STOP_HOLD_AFTER_RECOVERY "
                    f"min={front_min:.3f}m"
                )
            else:
                reason = (
                    "HARD_STOP_HOLD "
                    f"min={front_min:.3f}m "
                    f"wait={hard_elapsed:.2f}/"
                    f"{HARD_RECOVERY_WAIT_S:.2f}s"
                )

            return (
                0.0,
                0.0,
                reason,
                0.0
            )

        # PPO 的 action[1] rate limit 已在進入這個 safety class 前完成。
        # 這裡不再額外做 EMA / vx-wz slew smoothing。
        vx = clamp(
            float(desired_vx),
            -TEST_MAX_REV,
            TEST_MAX_FWD
        )

        wz = clamp(
            float(desired_wz),
            -TEST_MAX_WZ,
            TEST_MAX_WZ
        )

        wz, turn_hold_reason = self._stabilize_turn_direction(
            wz,
            front_robust
        )

        side_reason_parts = []

        if self.right_side_guard and wz < 0.0:
            wz = 0.0
            side_reason_parts.append(
                f"SIDE_GUARD_R {right_robust:.3f}m"
            )

        if self.left_side_guard and wz > 0.0:
            wz = 0.0
            side_reason_parts.append(
                f"SIDE_GUARD_L {left_robust:.3f}m"
            )

        side_nearest = min(right_robust, left_robust)

        if (
            vx > 0.0
            and
            math.isfinite(side_nearest)
            and
            side_nearest < SIDE_VX_SLOW_ENTER_M
            and
            vx > SIDE_VX_CAP_MPS
        ):
            vx = SIDE_VX_CAP_MPS
            side_reason_parts.append(
                f"SIDE_VX_CAP {side_nearest:.3f}m"
            )

        slow_scale = 1.0

        if (
            vx > 0.0
            and
            math.isfinite(front_robust)
            and
            front_robust < FRONT_SLOW_START_M
        ):
            slow_scale = (
                front_robust - FRONT_BLOCK_ENTER_M
            ) / (
                FRONT_SLOW_START_M - FRONT_BLOCK_ENTER_M
            )

            slow_scale = clamp(
                slow_scale,
                0.0,
                1.0
            )

            vx *= slow_scale

        if self.forward_blocked and vx > 0.0:
            vx = 0.0

        if self.forward_blocked:
            reason = (
                "FWD_BLOCK_HOLD "
                f"front={front_robust:.3f}m "
                f"release>{FRONT_BLOCK_EXIT_M:.2f}"
            )
        elif slow_scale < 0.999:
            reason = (
                "SLOW "
                f"front={front_robust:.3f}m "
                f"scale={slow_scale:.2f}"
            )
        else:
            reason = (
                "OK "
                f"front={front_robust:.3f}m"
            )

        if turn_hold_reason:
            reason = reason + " | " + turn_hold_reason

        if side_reason_parts:
            reason = reason + " | " + " | ".join(side_reason_parts)

        return (
            float(vx),
            float(wz),
            reason,
            float(slow_scale)
        )


# ============================================================
# Bottom-of-frame FINAL VISUAL CENTERING controller
# ============================================================

class BottomCenterController:

    def __init__(self):
        self.active = False
        self.start_time = 0.0
        self.last_used_frame_time = 0.0
        self.last_bbox_u = None
        self.last_wz_cmd = 0.0
        self.center_confirm_count = 0
        self.final_approach_pending = False


    def reset(self):
        self.active = False
        self.start_time = 0.0
        self.last_used_frame_time = 0.0
        self.last_bbox_u = None
        self.last_wz_cmd = 0.0
        self.center_confirm_count = 0
        self.final_approach_pending = False


    def geometry_locked(self, now):
        # FINAL CENTER 持續接受最新 vision，不鎖 geometry。
        return False


    def high_rate_needed(self, now):
        return self.active or self.final_approach_pending


    @staticmethod
    def _extract_bbox_center(vision_result, vision_reason):
        if vision_reason != "VALID":
            return None
        if vision_result is None:
            return None
        if not vision_result.get("on_floor", False):
            return None

        det = vision_result.get("det")
        if det is None:
            return None
        bbox = det.get("bbox")
        if bbox is None or len(bbox) != 4:
            return None

        x1, y1, x2, y2 = [float(v) for v in bbox]
        u = 0.5 * (x1 + x2)
        v_bottom = y2
        return (u, v_bottom)


    @staticmethod
    def _pixel_error(u):
        # >0 = sugar box 在畫面右邊；<0 = 在左邊。
        return float(u) - (FRAME_W * 0.5)


    @staticmethod
    def _wz_from_error(error_px):
        # ROS wz: + 左轉、- 右轉。
        # 物體在右邊(error>0) -> 車要右轉 -> wz<0。
        if abs(error_px) <= VISUAL_CENTER_TOL_PX:
            return 0.0

        wz = -VISUAL_CENTER_KP * float(error_px)
        magnitude = abs(wz)

        magnitude = clamp(
            magnitude,
            VISUAL_CENTER_MIN_WZ,
            VISUAL_CENTER_MAX_WZ
        )

        return math.copysign(
            magnitude,
            wz
        )


    def requires_center(self, target):
        # update() 在 ARRIVED 判斷前就會執行；只要 active/pending 就禁止直接 ARRIVED。
        return self.active or self.final_approach_pending


    def update(
        self,
        vision_result,
        vision_reason,
        vision_frame_time,
        now
    ):
        """
        Continuous image-space P controller.

        - 不用 odom yaw。
        - 不做固定時間 pulse。
        - 每張 fresh vision 用 YOLO bbox center 更新 wz。
        - 兩張 vision 之間維持上一次 wz，所以動作連續。
        - bbox center 進入 tolerance 後立即 command 0，連續確認兩張再 FINAL_FORWARD。
        """

        pixel = self._extract_bbox_center(
            vision_result,
            vision_reason
        )

        # ----------------------------------------------------
        # IDLE -> bbox 已接近畫面底部才接管
        # ----------------------------------------------------
        if not self.active:
            if self.final_approach_pending:
                return (False, 0.0, "CENTER_PIX_READY", False)

            if pixel is None:
                return (False, 0.0, "", False)

            u, v_bottom = pixel

            if v_bottom < FRAME_H * BOTTOM_CENTER_TRIGGER_V_RATIO:
                return (False, 0.0, "", False)

            error_px = self._pixel_error(u)

            self.active = True
            self.start_time = now
            self.last_used_frame_time = float(vision_frame_time)
            self.last_bbox_u = float(u)
            self.center_confirm_count = 0
            self.last_wz_cmd = self._wz_from_error(error_px)

            print(
                "[CENTER_PIX] start continuous | "
                f"bbox_u={u:.1f}px | "
                f"err={error_px:+.1f}px | "
                f"wz={self.last_wz_cmd:+.3f}"
            )

            # 一開始就已在中心：先算第一張確認，不轉。
            if abs(error_px) <= VISUAL_CENTER_TOL_PX:
                self.center_confirm_count = 1
                self.last_wz_cmd = 0.0

            return (
                True,
                float(self.last_wz_cmd),
                (
                    "CENTER_PIX_TRACK "
                    f"err={error_px:+.1f}px "
                    f"wz={self.last_wz_cmd:+.3f}"
                ),
                True
            )

        # ----------------------------------------------------
        # Timeout
        # ----------------------------------------------------
        if now - self.start_time >= VISUAL_CENTER_TIMEOUT_S:
            self.active = False
            self.last_wz_cmd = 0.0
            self.final_approach_pending = False

            print("[CENTER_PIX] timeout -> STOP")

            return (
                True,
                0.0,
                "CENTER_PIX_TIMEOUT",
                False
            )

        # ----------------------------------------------------
        # 沒有有效 vision 時不要繼續拿舊方向亂轉。
        # ----------------------------------------------------
        if pixel is None:
            self.last_wz_cmd = 0.0
            return (
                True,
                0.0,
                "CENTER_PIX_WAIT_VISION",
                False
            )

        # 同一張 vision result：維持目前命令，不重算。
        if vision_frame_time <= self.last_used_frame_time + 1e-6:
            return (
                True,
                float(self.last_wz_cmd),
                (
                    "CENTER_PIX_HOLD "
                    f"wz={self.last_wz_cmd:+.3f}"
                ),
                False
            )

        u, v_bottom = pixel
        error_px = self._pixel_error(u)

        # fresh vision 已收到。
        self.last_used_frame_time = float(vision_frame_time)

        # 異常大跳點：這一張不拿來換向，先停等下一張。
        if (
            self.last_bbox_u is not None
            and
            abs(float(u) - float(self.last_bbox_u)) > VISUAL_CENTER_MAX_JUMP_PX
        ):
            print(
                "[CENTER_PIX] reject jump | "
                f"prev={self.last_bbox_u:.1f}px "
                f"new={u:.1f}px"
            )
            self.last_wz_cmd = 0.0
            self.center_confirm_count = 0
            return (
                True,
                0.0,
                "CENTER_PIX_REJECT_JUMP",
                False
            )

        self.last_bbox_u = float(u)

        # ----------------------------------------------------
        # 已進中心：立即停，再要求連續 fresh frames 確認。
        # ----------------------------------------------------
        if abs(error_px) <= VISUAL_CENTER_TOL_PX:
            self.last_wz_cmd = 0.0
            self.center_confirm_count += 1

            print(
                "[CENTER_PIX] centered frame | "
                f"bbox_u={u:.1f}px | "
                f"err={error_px:+.1f}px | "
                f"confirm={self.center_confirm_count}/{VISUAL_CENTER_CONFIRM_FRAMES}"
            )

            if self.center_confirm_count >= VISUAL_CENTER_CONFIRM_FRAMES:
                self.active = False
                self.final_approach_pending = True
                self.center_confirm_count = 0

                print("[CENTER_PIX] CENTERED -> FINAL_FORWARD")

                return (
                    False,
                    0.0,
                    (
                        "CENTER_PIX_CENTERED "
                        f"err={error_px:+.1f}px"
                    ),
                    False
                )

            return (
                True,
                0.0,
                (
                    "CENTER_PIX_CONFIRM "
                    f"{self.center_confirm_count}/{VISUAL_CENTER_CONFIRM_FRAMES} "
                    f"err={error_px:+.1f}px"
                ),
                False
            )

        # ----------------------------------------------------
        # 還沒中心：fresh frame 直接重算 proportional wz。
        # ----------------------------------------------------
        self.center_confirm_count = 0
        self.last_wz_cmd = self._wz_from_error(error_px)

        print(
            "[CENTER_PIX] track | "
            f"bbox_u={u:.1f}px | "
            f"err={error_px:+.1f}px | "
            f"wz={self.last_wz_cmd:+.3f}"
        )

        return (
            True,
            float(self.last_wz_cmd),
            (
                "CENTER_PIX_TRACK "
                f"err={error_px:+.1f}px "
                f"wz={self.last_wz_cmd:+.3f}"
            ),
            False
        )


    def consume_final_approach_ready(self, target, now):
        if not self.final_approach_pending:
            return False

        self.final_approach_pending = False
        return True


# ============================================================
# Final small forward pulse controller
# ============================================================

class FinalApproachController:

    def __init__(self):
        self.active = False
        self.start_time = 0.0
        self.last_update_time = 0.0
        self.travelled_m = 0.0


    def reset(self):
        self.active = False
        self.start_time = 0.0
        self.last_update_time = 0.0
        self.travelled_m = 0.0


    def start(self, target, now):
        if self.active:
            return False

        self.active = True
        self.start_time = now
        self.last_update_time = now
        self.travelled_m = 0.0

        print(
            "[FINAL] visual-centered -> forward 15cm | "
            f"vx={FINAL_APPROACH_VX:.3f}m/s | "
            f"travel={FINAL_APPROACH_TRAVEL_M:.2f}m | "
            f"timeout={FINAL_APPROACH_MAX_DURATION_S:.1f}s"
        )

        return True


    def command(self, target, odom_vx, now):
        if not self.active:
            return (False, 0.0, 0.0, "")

        dt = max(0.0, now - self.last_update_time)
        self.last_update_time = now

        # 用實際 odom forward velocity 積分走過的距離。
        # 小於 0 的速度不算進前進距離，避免倒滑把里程抵消。
        if dt <= 0.5:
            self.travelled_m += max(0.0, float(odom_vx)) * dt

        elapsed = now - self.start_time

        if self.travelled_m >= FINAL_APPROACH_TRAVEL_M:
            self.active = False
            return (
                True,
                0.0,
                0.0,
                (
                    "FINAL_DONE_TRAVEL "
                    f"travel={self.travelled_m:.3f}m"
                )
            )

        if elapsed >= FINAL_APPROACH_MAX_DURATION_S:
            self.active = False
            return (
                True,
                0.0,
                0.0,
                (
                    "FINAL_TIMEOUT_NOT_REACHED "
                    f"travel={self.travelled_m:.3f}m"
                )
            )

        return (
            True,
            float(FINAL_APPROACH_VX),
            0.0,
            (
                "FINAL_FORWARD "
                f"travel={self.travelled_m:.3f}/"
                f"{FINAL_APPROACH_TRAVEL_M:.3f}m "
                f"t={elapsed:.2f}s "
                f"vx={FINAL_APPROACH_VX:.3f}"
            )
        )


# ============================================================
# Dummy Env for VecNormalize
# ============================================================

class ObsOnlyEnv(
    gym.Env
):

    metadata = {}


    def __init__(
        self
    ):

        super().__init__()

        self.observation_space = spaces.Box(

            low=-np.inf,

            high=np.inf,

            shape=(
                OBS_DIM,
            ),

            dtype=np.float32
        )

        self.action_space = spaces.Box(

            low=-1.0,

            high=1.0,

            shape=(
                2,
            ),

            dtype=np.float32
        )


    def reset(
        self,
        *,
        seed=None,
        options=None
    ):

        super().reset(
            seed=seed
        )

        return (

            np.zeros(
                OBS_DIM,
                dtype=np.float32
            ),

            {}
        )


    def step(
        self,
        action
    ):

        observation = np.zeros(

            OBS_DIM,

            dtype=np.float32
        )

        return (

            observation,

            0.0,

            False,

            False,

            {}
        )


# ============================================================
# PPO / VecNormalize
# ============================================================

def load_policy_and_vecnormalize():

    print(
        "[RL] model:"
    )

    print(
        RL_MODEL_PATH
    )

    print(
        "[RL] VecNormalize:"
    )

    print(
        VECNORMALIZE_PATH
    )

    raw_vec_env = DummyVecEnv([

        lambda: ObsOnlyEnv()
    ])

    vecnormalize = VecNormalize.load(

        VECNORMALIZE_PATH,

        raw_vec_env
    )

    vecnormalize.training = False

    vecnormalize.norm_reward = False

    model = PPO.load(

        RL_MODEL_PATH,

        device="cpu",
        # Training schedules are unused at inference. Avoid old Python lambda
        # pickles; match the navigation runtime's cross-version fallback.
        custom_objects={"lr_schedule": lambda _: 0.0, "clip_range": lambda _: 0.0}
    )

    if (tuple(model.observation_space.shape) != (55,)
            or tuple(model.action_space.shape) != (2,)
            or tuple(vecnormalize.observation_space.shape) != (55,)):
        raise RuntimeError("avoidance contract must be obs=55D/action=2D")

    return (
        model,
        vecnormalize
    )


# ============================================================
# 55D Observation
# ============================================================

def build_raw_obs(

    distance,

    bearing,

    vx,

    wz,

    prev_action,

    lidar_48
):

    obs = np.zeros(

        OBS_DIM,

        dtype=np.float32
    )

    # Goal
    obs[0] = float(
        distance
    )

    obs[1] = math.sin(
        float(
            bearing
        )
    )

    obs[2] = math.cos(
        float(
            bearing
        )
    )


    # Feedback odom
    obs[3] = float(
        vx
    )

    obs[4] = float(
        wz
    )


    # Previous RAW policy action
    obs[5] = float(
        prev_action[0]
    )

    obs[6] = float(
        prev_action[1]
    )


    # 48 raw-meter LiDAR rays
    obs[7:55] = np.asarray(

        lidar_48,

        dtype=np.float32
    )

    return obs


def policy_predict(

    model,

    vecnormalize,

    raw_obs
):

    # 不自己除以 4
    # 不自己 normalize dist
    # 全部交給原本 VecNormalize
    #
    normalized_obs = vecnormalize.normalize_obs(

        raw_obs.reshape(
            1,
            -1
        ).copy()
    )

    action, _ = model.predict(

        normalized_obs,

        deterministic=True
    )

    action = np.asarray(

        action,

        dtype=np.float32
    ).reshape(
        -1
    )

    if action.size != 2:

        raise RuntimeError(

            f"PPO action shape wrong: "
            f"{action.shape}"
        )

    action = np.clip(

        action,

        -1.0,

        1.0
    )

    return action


# ============================================================
# PPO action -> vx / wz
# ============================================================

def action_to_velocity(
    action
):

    a0 = float(
        action[0]
    )

    a1 = float(
        action[1]
    )


    if a0 >= 0.0:

        vx = (
            a0
            *
            POLICY_MAX_FWD
        )

    else:

        vx = (
            a0
            *
            POLICY_MAX_REV
        )


    wz = (
        a1
        *
        POLICY_MAX_WZ
    )


    # --------------------------------------------------------
    # 第一次實車外層限速
    # --------------------------------------------------------

    vx = clamp(

        vx,

        -TEST_MAX_REV,

        TEST_MAX_FWD
    )

    wz = clamp(

        wz,

        -TEST_MAX_WZ,

        TEST_MAX_WZ
    )

    return (
        float(vx),
        float(wz)
    )


# ============================================================
# Motor TCP
# ============================================================

class MotorClient:

    def __init__(
        self,
        host,
        port
    ):

        self.host = host

        self.port = port

        self.sock = None

        self.last_send = 0.0


    def connect(
        self
    ):

        if self.sock is not None:
            return

        print(

            f"[MOTOR] connecting "
            f"{self.host}:{self.port}"
        )

        sock = socket.socket(

            socket.AF_INET,

            socket.SOCK_STREAM
        )

        sock.settimeout(
            1.0
        )

        sock.connect(

            (
                self.host,
                self.port
            )
        )

        self.sock = sock

        print(
            "[MOTOR] connected"
        )


    def _send_json(
        self,
        msg
    ):

        if self.sock is None:

            self.connect()

        data = (

            json.dumps(
                msg
            )
            +
            "\n"

        ).encode(
            "utf-8"
        )

        try:

            self.sock.sendall(
                data
            )

        except Exception:

            self.close()

            self.connect()

            self.sock.sendall(
                data
            )


    def send_velocity(

        self,

        vx,

        wz,

        force=False
    ):

        now = time.time()

        if (
            not force
            and
            now - self.last_send
            <
            MOTOR_SEND_INTERVAL
        ):

            return

        msg = {

            "action": "velocity",

            "vx": float(
                vx
            ),

            "wz": float(
                wz
            )
        }

        self._send_json(
            msg
        )

        self.last_send = now


    def stop(
        self,
        repeat=5
    ):

        for _ in range(
            repeat
        ):

            try:

                self.send_velocity(

                    0.0,

                    0.0,

                    force=True
                )

            except Exception:
                pass

            time.sleep(
                0.03
            )


    def close(
        self
    ):

        if self.sock is not None:

            try:

                self.sock.close()

            except Exception:
                pass

        self.sock = None


# ============================================================
# Target State
# ============================================================

@dataclass
class TargetState:

    x: float = 0.0

    y: float = 0.0

    dist: float = 0.0

    bearing: float = 0.0

    last_seen: float = 0.0

    # 最後一次真正更新 x/y/bearing 的 vision geometry 時間。
    # refresh_seen_only() 不會改這個值。CENTER 停下來後用它等待 fresh vision。
    last_geometry_stamp: float = 0.0

    last_motion_update: float = 0.0

    conf: float = 0.0

    class_name: str = ""

    u: float = 0.0

    v: float = 0.0

    on_floor: bool = False

    candidate_x: float = 0.0

    candidate_y: float = 0.0

    candidate_count: int = 0

    candidate_stamp: float = 0.0

    rejected_count: int = 0

    last_match_error: float = float("inf")


    def has_memory(
        self
    ):

        return (
            self.on_floor
            and
            self.last_geometry_stamp > 0.0
        )


    def visual_recent(
        self,
        now
    ):

        return (
            self.has_memory()
            and
            get_age(now, self.last_seen)
            <=
            TARGET_VISUAL_RECENT_S
        )


    def clear_candidate(
        self
    ):

        self.candidate_x = 0.0
        self.candidate_y = 0.0
        self.candidate_count = 0
        self.candidate_stamp = 0.0


    def clear(
        self
    ):

        self.x = 0.0
        self.y = 0.0
        self.dist = 0.0
        self.bearing = 0.0
        self.last_seen = 0.0
        self.last_geometry_stamp = 0.0
        self.last_motion_update = 0.0
        self.conf = 0.0
        self.class_name = ""
        self.u = 0.0
        self.v = 0.0
        self.on_floor = False
        self.rejected_count = 0
        self.last_match_error = float("inf")
        self.clear_candidate()


    def _update_candidate(
        self,
        x,
        y,
        now,
        max_error
    ):

        consistent = (
            self.candidate_count > 0
            and
            get_age(now, self.candidate_stamp)
            <=
            TARGET_CONFIRM_MAX_GAP_S
            and
            math.hypot(
                float(x) - self.candidate_x,
                float(y) - self.candidate_y
            )
            <=
            float(max_error)
        )

        if consistent:
            count = self.candidate_count + 1
            self.candidate_x += (
                float(x) - self.candidate_x
            ) / float(count)
            self.candidate_y += (
                float(y) - self.candidate_y
            ) / float(count)
            self.candidate_count = count
        else:
            self.candidate_x = float(x)
            self.candidate_y = float(y)
            self.candidate_count = 1

        self.candidate_stamp = float(now)


    def match_tolerance(
        self,
        measurement
    ):

        candidate_dist = math.hypot(
            float(measurement["x"]),
            float(measurement["y"])
        )

        return min(
            TARGET_REACQUIRE_MAX_ERROR_M,
            TARGET_REACQUIRE_BASE_ERROR_M
            +
            TARGET_REACQUIRE_RANGE_GAIN
            *
            max(self.dist, candidate_dist)
        )


    def consider_measurement(
        self,
        measurement,
        now,
        geometry_locked=False
    ):

        new_x = float(measurement["x"])
        new_y = float(measurement["y"])

        if not self.has_memory():
            self._update_candidate(
                new_x,
                new_y,
                now,
                TARGET_ACQUIRE_MAX_ERROR_M
            )

            if self.candidate_count < TARGET_ACQUIRE_CONFIRM_FRAMES:
                return (
                    False,
                    "TARGET_ACQUIRING {}/{}".format(
                        self.candidate_count,
                        TARGET_ACQUIRE_CONFIRM_FRAMES
                    ),
                    0.0,
                    TARGET_ACQUIRE_MAX_ERROR_M
                )

            accepted = dict(measurement)
            accepted["x"] = self.candidate_x
            accepted["y"] = self.candidate_y
            self.set_measurement(accepted, now)
            self.clear_candidate()
            return (
                True,
                "TARGET_LOCKED",
                0.0,
                TARGET_ACQUIRE_MAX_ERROR_M
            )

        tolerance = self.match_tolerance(measurement)
        error = math.hypot(
            new_x - self.x,
            new_y - self.y
        )
        self.last_match_error = error

        if error > tolerance:
            self.rejected_count += 1
            self.clear_candidate()
            return (
                False,
                "TARGET_MISMATCH {:.2f}>{:.2f}m".format(
                    error,
                    tolerance
                ),
                error,
                tolerance
            )

        if not self.visual_recent(now):
            self._update_candidate(
                new_x,
                new_y,
                now,
                tolerance
            )

            if self.candidate_count < TARGET_REACQUIRE_CONFIRM_FRAMES:
                return (
                    False,
                    "TARGET_RECONFIRMING {}/{}".format(
                        self.candidate_count,
                        TARGET_REACQUIRE_CONFIRM_FRAMES
                    ),
                    error,
                    tolerance
                )

            accepted = dict(measurement)
            accepted["x"] = self.candidate_x
            accepted["y"] = self.candidate_y
        else:
            accepted = measurement

        if geometry_locked:
            self.refresh_seen_only(accepted, now)
        else:
            self.set_measurement(accepted, now)

        self.clear_candidate()
        return (
            True,
            "TARGET_CONFIRMED",
            error,
            tolerance
        )


    def valid(
        self,
        now
    ):

        return self.has_memory()


    def set_measurement(

        self,

        measurement,

        now
    ):

        new_x = float(
            measurement[
                "x"
            ]
        )

        new_y = float(
            measurement[
                "y"
            ]
        )

        if (
            self.last_seen > 0.0
            and
            self.on_floor
        ):

            alpha = TARGET_EMA_ALPHA

            self.x = (

                (
                    1.0 - alpha
                )
                *
                self.x

                +
                alpha
                *
                new_x
            )

            self.y = (

                (
                    1.0 - alpha
                )
                *
                self.y

                +
                alpha
                *
                new_y
            )

        else:

            self.x = new_x
            self.y = new_y


        self.dist = math.hypot(

            self.x,

            self.y
        )

        self.bearing = math.atan2(

            self.y,

            self.x
        )

        self.last_seen = now

        self.last_geometry_stamp = now

        self.last_motion_update = now

        self.conf = float(

            measurement[
                "det"
            ][
                "conf"
            ]
        )

        self.class_name = str(

            measurement[
                "det"
            ][
                "class_name"
            ]
        )

        self.u = float(
            measurement[
                "u"
            ]
        )

        self.v = float(
            measurement[
                "v"
            ]
        )

        self.on_floor = True


    def refresh_seen_only(
        self,
        measurement,
        now
    ):

        # final centering 期間只更新「仍然看得到目標」的 freshness，
        # 不用延遲的 camera geometry 覆蓋 odom 推算中的 x/y/bearing。
        self.last_seen = now

        self.conf = float(
            measurement[
                "det"
            ][
                "conf"
            ]
        )

        self.class_name = str(
            measurement[
                "det"
            ][
                "class_name"
            ]
        )

        self.u = float(
            measurement[
                "u"
            ]
        )

        self.v = float(
            measurement[
                "v"
            ]
        )

        self.on_floor = True


    def propagate_with_odom(

        self,

        vx,

        wz,

        now
    ):

        if self.last_motion_update <= 0.0:

            self.last_motion_update = now

            return


        dt = (

            now
            -
            self.last_motion_update
        )

        if dt <= 0.0:

            return

        if dt > 0.5:

            self.last_motion_update = now

            return


        # ----------------------------------------------------
        # 機器人往前：
        # target 在 robot frame 中往後
        # ----------------------------------------------------

        x = (
            self.x
            -
            float(vx)
            *
            dt
        )

        y = self.y


        # ----------------------------------------------------
        # 機器人左轉：
        # target relative frame 做反方向旋轉
        # ----------------------------------------------------

        dtheta = (
            float(wz)
            *
            dt
        )

        c = math.cos(
            dtheta
        )

        s = math.sin(
            dtheta
        )

        x_new = (
            c * x
            +
            s * y
        )

        y_new = (
            -s * x
            +
            c * y
        )

        self.x = x_new

        self.y = y_new

        self.dist = math.hypot(

            self.x,

            self.y
        )

        self.bearing = math.atan2(

            self.y,

            self.x
        )

        self.last_motion_update = now


def run_target_memory_selftest():

    def measurement(x, y):
        return {
            "x": float(x),
            "y": float(y),
            "det": {
                "conf": 0.9,
                "class_name": "sugarbox"
            },
            "u": 320.0,
            "v": 360.0,
            "on_floor": True
        }

    target = TargetState()
    now = 100.0

    accepted, status, _, _ = target.consider_measurement(
        measurement(1.00, 0.10),
        now
    )
    assert not accepted and "1/3" in status

    accepted, status, _, _ = target.consider_measurement(
        measurement(1.03, 0.11),
        now + 0.2
    )
    assert not accepted and "2/3" in status

    accepted, status, _, _ = target.consider_measurement(
        measurement(1.01, 0.09),
        now + 0.4
    )
    assert accepted and status == "TARGET_LOCKED"
    assert target.has_memory()

    current = now + 0.4
    for _ in range(20):
        current += 0.1
        target.propagate_with_odom(0.10, 0.0, current)

    assert target.has_memory()
    assert not target.visual_recent(current)
    remembered_x = target.x
    remembered_y = target.y

    accepted, status, error, tolerance = target.consider_measurement(
        measurement(2.50, -1.00),
        current + 0.1
    )
    assert not accepted and status.startswith("TARGET_MISMATCH")
    assert error > tolerance
    assert target.x == remembered_x and target.y == remembered_y

    reconfirmed_at = current
    for frame_index in range(1, TARGET_REACQUIRE_CONFIRM_FRAMES + 1):
        reconfirmed_at = current + 0.2 * frame_index
        accepted, status, _, _ = target.consider_measurement(
            measurement(
                remembered_x + 0.04 - 0.01 * frame_index,
                remembered_y - 0.03 + 0.01 * frame_index
            ),
            reconfirmed_at
        )

        if frame_index < TARGET_REACQUIRE_CONFIRM_FRAMES:
            assert not accepted
            assert "{}/{}".format(
                frame_index,
                TARGET_REACQUIRE_CONFIRM_FRAMES
            ) in status
        else:
            assert accepted and status == "TARGET_CONFIRMED"

    assert target.visual_recent(reconfirmed_at)

    target.clear()
    assert not target.has_memory()
    print("[selftest] persistent target memory + vision gating OK")


# ============================================================
# Display
# ============================================================

def overlay_mask(
    frame,
    mask
):

    if mask is None:
        return frame

    if not SHOW_MASK:
        return frame

    mask_bool = (
        mask > 0.5
    )

    if not np.any(
        mask_bool
    ):
        return frame

    overlay = frame.copy()

    overlay[
        mask_bool
    ] = (
        0,
        255,
        0
    )

    output = cv2.addWeighted(

        overlay,

        MASK_ALPHA,

        frame,

        1.0
        -
        MASK_ALPHA,

        0.0
    )

    return output



def draw_vision_annotation_frame(
    frame,
    vision_result
):
    """
    Draw YOLO bbox + SAM mask + contact point on the exact frame that
    produced that vision result. This is for display only; control logic
    does not read this rendered image.
    """

    if frame is None:
        return None

    output = frame.copy()

    if vision_result is None:
        return output

    detection = vision_result.get("det")

    if detection is not None:
        x1, y1, x2, y2 = detection["bbox"]
        cv2.rectangle(
            output,
            (int(x1), int(y1)),
            (int(x2), int(y2)),
            (0, 255, 255),
            2
        )

        label = (
            f"{detection.get('class_name', 'target')} "
            f"{detection.get('conf', 0.0):.2f}"
        )

        cv2.putText(
            output,
            label,
            (int(x1), max(18, int(y1) - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (0, 255, 255),
            1
        )

    mask = vision_result.get("mask")

    if mask is not None:
        output = overlay_mask(
            output,
            mask
        )

    if (
        "u" in vision_result
        and
        "v" in vision_result
    ):
        cv2.circle(
            output,
            (
                int(round(vision_result["u"])),
                int(round(vision_result["v"]))
            ),
            7,
            (0, 0, 255),
            -1
        )

    return output


def draw_hud(

    frame,

    vision_result,

    vision_reason,

    target,

    mode,

    sensor_text,

    rl_text,

    safety_text,

    draw_vision_overlay=True
):

    output = frame.copy()


    # --------------------------------------------------------
    # bbox / SAM / contact
    # --------------------------------------------------------

    if draw_vision_overlay and vision_result is not None:

        detection = vision_result.get(
            "det"
        )

        if detection is not None:

            (
                x1,
                y1,
                x2,
                y2
            ) = detection[
                "bbox"
            ]

            cv2.rectangle(

                output,

                (
                    int(x1),
                    int(y1)
                ),

                (
                    int(x2),
                    int(y2)
                ),

                (
                    0,
                    255,
                    255
                ),

                2
            )


        mask = vision_result.get(
            "mask"
        )

        if mask is not None:

            output = overlay_mask(

                output,

                mask
            )


        if (
            "u" in vision_result
            and
            "v" in vision_result
        ):

            u = int(
                round(
                    vision_result[
                        "u"
                    ]
                )
            )

            v = int(
                round(
                    vision_result[
                        "v"
                    ]
                )
            )

            cv2.circle(

                output,

                (
                    u,
                    v
                ),

                7,

                (
                    0,
                    0,
                    255
                ),

                -1
            )


    # --------------------------------------------------------
    # HUD black background
    # --------------------------------------------------------

    cv2.rectangle(

        output,

        (
            0,
            0
        ),

        (
            FRAME_W,
            150
        ),

        (
            0,
            0,
            0
        ),

        -1
    )


    lines = [

        (
            f"MODE={mode} | "
            f"vision={vision_reason}"
        ),

        (
            f"target "
            f"x={target.x:.3f} "
            f"y={target.y:+.3f} "
            f"dist={target.dist:.3f} "
            f"bearing="
            f"{math.degrees(target.bearing):+.1f}deg"
        ),

        sensor_text,

        rl_text,

        safety_text,

        "ESC quit | SPACE E-STOP | R reset"
    ]


    y = 22

    for index, text in enumerate(
        lines
    ):

        color = (
            255,
            255,
            255
        )

        if index == 4:

            if (
                "STOP" in text
                or
                "BLOCK" in text
                or
                "STALE" in text
            ):

                color = (
                    0,
                    0,
                    255
                )

        cv2.putText(

            output,

            text,

            (
                10,
                y
            ),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.43,

            color,

            1
        )

        y += 24

    return output


# ============================================================
# File check
# ============================================================

def check_files():

    required = [

        YOLO_MODEL_PATH,

        SAM2_MODEL_PATH,

        HOMOGRAPHY_PATH,

        GSTREAMER_LAUNCH_EXE
    ]

    if RUN_MODE != "VISION":

        required += [

            RL_MODEL_PATH,

            VECNORMALIZE_PATH
        ]

    missing = [

        path

        for path in required

        if not os.path.exists(
            path
        )
    ]

    if not missing:

        return True

    print(
        "[ERROR] Missing files:"
    )

    for path in missing:

        print(
            "   ",
            path
        )

    return False


# ============================================================
# MAIN
# ============================================================

def main():

    if RUN_MODE not in {

        "VISION",

        "DRY_RUN",

        "DRIVE"
    }:

        raise ValueError(

            "RUN_MODE must be "
            "VISION / DRY_RUN / DRIVE"
        )


    print()
    print(
        "============================================================"
    )
    print(
        " SUGARBOX RL APPROACH - STANDALONE FINAL"
    )
    print(
        "============================================================"
    )

    print(
        f"[MODE] {RUN_MODE}"
    )

    print(
        f"[NET] Jetson = {JETSON_IP}"
    )

    print(
        f"[CAM] RTP/H264 UDP :{CAMERA_UDP_PORT}"
    )

    print(
        f"[CAM] GStreamer = {GSTREAMER_LAUNCH_EXE}"
    )

    print(
        f"[PERF] camera_pipe={CAMERA_PIPE_FPS} FPS (live async) | "
        f"vision={1.0 / VISION_INTERVAL:.1f} Hz max | "
        f"display={DISPLAY_HZ:.1f} Hz"
    )


    # --------------------------------------------------------
    # Files
    # --------------------------------------------------------

    if not check_files():
        return


    # --------------------------------------------------------
    # Homography
    # --------------------------------------------------------

    print()
    print(
        "[GROUND] Loading..."
    )

    (
        H,
        rear_cam_to_front_m,
        calibration_hull
    ) = load_ground_calibration(

        HOMOGRAPHY_PATH
    )

    print(
        "[GROUND] Loaded"
    )

    print(
        "[GROUND] rear_cam_to_front_m = "
        f"{rear_cam_to_front_m:.3f}"
    )

    print(
        "[GROUND] H ="
    )

    print(
        H
    )


    # --------------------------------------------------------
    # YOLO
    # --------------------------------------------------------

    print()
    print(
        "[MODEL] Loading YOLO..."
    )

    yolo_model = YOLO(
        YOLO_MODEL_PATH
    )

    print(
        "[MODEL] YOLO classes:"
    )

    print(
        yolo_model.names
    )


    # --------------------------------------------------------
    # SAM2
    # --------------------------------------------------------

    print()
    print(
        "[MODEL] Loading SAM2..."
    )

    sam_model = SAM(
        SAM2_MODEL_PATH
    )

    print(
        "[MODEL] SAM2 ready"
    )


    # --------------------------------------------------------
    # PPO / ROS
    # --------------------------------------------------------

    rl_model = None

    vecnormalize = None

    ros_bridge = None

    motor = None


    if RUN_MODE != "VISION":

        (
            rl_model,
            vecnormalize
        ) = load_policy_and_vecnormalize()

        ros_bridge = RosSensorBridge(

            JETSON_IP,

            ROSBRIDGE_PORT
        )

        ros_bridge.start()


    # --------------------------------------------------------
    # DRIVE only motor
    # --------------------------------------------------------

    if RUN_MODE == "DRIVE":

        motor = MotorClient(

            JETSON_IP,

            MOTOR_PORT
        )

        motor.connect()

        motor.stop(
            repeat=3
        )


    # --------------------------------------------------------
    # Camera
    # --------------------------------------------------------

    print()
    print(
        "[CAMERA] Opening rear camera..."
    )

    reader = LatestFrameReader(

        width=FRAME_W,

        height=FRAME_H,

        udp_port=CAMERA_UDP_PORT,

        gst_exe=GSTREAMER_LAUNCH_EXE
    )

    vision_worker = AsyncVisionWorker(
        reader=reader,
        yolo_model=yolo_model,
        sam_model=sam_model,
        H=H,
        rear_cam_to_front_m=rear_cam_to_front_m,
        calibration_hull=calibration_hull,
        ros_bridge=(ros_bridge if RUN_MODE != "VISION" else None),
    )


    # --------------------------------------------------------
    # Runtime states
    # --------------------------------------------------------

    target = TargetState()

    prev_raw_action = np.zeros(

        2,

        dtype=np.float32
    )


    # motor_delay_steps = 2
    action_delay = [

        np.zeros(
            2,
            dtype=np.float32
        )

        for _ in range(
            MOTOR_DELAY_STEPS
        )
    ]


    last_vision_sequence = 0
    last_latency_print = 0.0

    last_vision_latency = {
        "queue_s": 0.0,
        "infer_s": 0.0,
        "stream_assumed_s": CAMERA_STREAM_DELAY_COMP_S,
        "total_est_s": 0.0,
        "odom_comp_s": 0.0,
        "full_coverage": False
    }

    last_control_run = 0.0

    last_print = 0.0
    last_display_run = 0.0


    last_frame = np.zeros(

        (
            FRAME_H,
            FRAME_W,
            3
        ),

        dtype=np.uint8
    )


    last_vision_result = None

    # Exact camera frame that produced last_vision_result.
    # Used for the one-window VISION inset so bbox/mask always line up.
    last_vision_frame = None

    # One-window vision inset. It is rendered from the exact inference frame
    # so bbox/SAM always align, while the main background stays live.
    last_vision_panel = None

    last_vision_reason = "WAIT"


    last_raw_action = np.zeros(

        2,

        dtype=np.float32
    )


    last_desired_vx = 0.0

    last_desired_wz = 0.0

    last_cmd_vx = 0.0

    last_cmd_wz = 0.0

    last_front_robust = float(
        "inf"
    )

    last_front_min = float(
        "inf"
    )

    last_front_samples = 0

    last_safety_reason = "IDLE"

    anti_jitter = AntiJitterSafetyFilter()
    center_controller = BottomCenterController()
    final_approach = FinalApproachController()


    e_stop_latched = False

    arrived_latched = False

    target_search_start = 0.0


    print()
    print(
        "[READY] Running"
    )

    print(
        "[KEY] ESC   = exit"
    )

    print(
        "[KEY] SPACE = emergency stop"
    )

    print(
        "[KEY] R     = reset stop/arrival latch"
    )

    print(
        "[KEY] C     = clear remembered target"
    )

    print(
        "[SAFETY] front slow/block/hard = "
        f"{FRONT_SLOW_START_M:.2f} / "
        f"{FRONT_BLOCK_ENTER_M:.2f}->{FRONT_BLOCK_EXIT_M:.2f} / "
        f"{FRONT_HARD_STOP_ENTER_M:.2f}->{FRONT_HARD_STOP_EXIT_M:.2f} m"
    )

    print(
        "[FILTER] PPO temporal smoothing = OFF "
        "(no EMA / slew / deadzone)"
    )

    print(
        "[AVOID] near-obstacle turn sign hold = "
        f"{AVOID_TURN_HOLD_ENTER_M:.2f}->{AVOID_TURN_HOLD_EXIT_M:.2f}m, "
        f"switch_confirm={AVOID_TURN_SWITCH_CONFIRM_TICKS} ticks"
    )
    print(
        "[SIDE_SAFETY] external LiDAR coverage = "
        f"-{SIDE_SAFETY_MAX_ANGLE_DEG:.0f}..+{SIDE_SAFETY_MAX_ANGLE_DEG:.0f}deg "
        "(PPO stays -90..+90deg) | "
        f"turn_guard={SIDE_TURN_GUARD_ENTER_M:.2f}->{SIDE_TURN_GUARD_EXIT_M:.2f}m | "
        f"vx_cap<{SIDE_VX_SLOW_ENTER_M:.2f}m => {SIDE_VX_CAP_MPS:.2f}m/s"
    )

    print(
        "[PPO_LIDAR] 48-ray distance scale = "
        f"x{PPO_LIDAR_DISTANCE_SCALE:.2f} "
        "(external safety still uses raw meters)"
    )
    print(
        "[RECOVERY] hard-stop = STOP -> "
        f"wait {HARD_RECOVERY_WAIT_S:.2f}s -> "
        f"reverse {HARD_RECOVERY_TARGET_M:.2f}m "
        f"@ {abs(HARD_RECOVERY_REVERSE_VX):.2f}m/s -> "
        f"settle {HARD_RECOVERY_SETTLE_S:.2f}s | "
        "one backup per hard-stop episode | NO rear LiDAR"
    )

    print(
        "[CENTER_PIX] bbox-bottom>="
        f"{FRAME_H * BOTTOM_CENTER_TRIGGER_V_RATIO:.0f}px | "
        f"center_tol=±{VISUAL_CENTER_TOL_PX:.0f}px | "
        f"Kp={VISUAL_CENTER_KP:.4f} | "
        f"wz={VISUAL_CENTER_MIN_WZ:.2f}~{VISUAL_CENTER_MAX_WZ:.2f} | "
        f"jump_reject>{VISUAL_CENTER_MAX_JUMP_PX:.0f}px | "
        f"confirm={VISUAL_CENTER_CONFIRM_FRAMES} frames"
    )

    print(
        "[CENTER_PIX] mode = CONTINUOUS PIXEL P-CONTROL (YOLO bbox center)"
    )

    print(
        "[FINAL] centered image -> forward by odom | "
        f"vx={FINAL_APPROACH_VX:.2f}m/s | "
        f"travel={FINAL_APPROACH_TRAVEL_M:.2f}m | "
        f"timeout={FINAL_APPROACH_MAX_DURATION_S:.1f}s"
    )

    print(
        "[LATENCY] artificial UDP/H264 transport delay = "
        f"{CAMERA_STREAM_DELAY_COMP_S:.3f}s (disabled)"
    )

    print(
        "[TARGET_MEMORY] acquire={} frames | reacquire={} frames | "
        "match={:.2f}~{:.2f}m | search<={:.2f}m".format(
            TARGET_ACQUIRE_CONFIRM_FRAMES,
            TARGET_REACQUIRE_CONFIRM_FRAMES,
            TARGET_REACQUIRE_BASE_ERROR_M,
            TARGET_REACQUIRE_MAX_ERROR_M,
            TARGET_SEARCH_START_DIST_M
        )
    )

    print()


    try:

        while True:

            now = time.time()


            # ==================================================
            # Read newest camera frame
            # ==================================================

            (
                camera_ok,
                frame,
                frame_time
            ) = reader.read()

            if camera_ok:

                last_frame = frame


            # ==================================================
            # Consume newest ASYNC vision result
            # ==================================================
            # YOLO + SAM2 run in AsyncVisionWorker. The main loop never waits
            # for inference, so the live preview and ROS/PPO control remain live.

            (
                vision_sequence,
                vision_result,
                vision_reason,
                vision_done_time,
                vision_frame_time,
                vision_source_frame,
                latency_info,
            ) = vision_worker.snapshot()

            if vision_sequence != last_vision_sequence:
                last_vision_sequence = vision_sequence
                last_vision_latency = latency_info
                last_vision_result = vision_result
                last_vision_frame = vision_source_frame
                last_vision_reason = vision_reason

                if vision_source_frame is not None:
                    last_vision_panel = draw_vision_annotation_frame(
                        vision_source_frame,
                        vision_result
                    )

                if (
                    now - last_latency_print
                    >= VISION_LATENCY_PRINT_INTERVAL
                ):
                    last_latency_print = now

                    raw_bearing_text = "n/a"
                    comp_bearing_text = "n/a"

                    if (
                        vision_result is not None
                        and
                        "raw_bearing_before_latency_comp" in vision_result
                    ):
                        raw_bearing_text = (
                            f"{math.degrees(vision_result['raw_bearing_before_latency_comp']):+.1f}deg"
                        )
                        comp_bearing_text = (
                            f"{math.degrees(vision_result['bearing']):+.1f}deg"
                        )

                    print(
                        "[VISION_LAT] "
                        f"queue={latency_info['queue_s']*1000:.0f}ms "
                        f"infer={latency_info['infer_s']*1000:.0f}ms "
                        f"stream_assumed={latency_info['stream_assumed_s']*1000:.0f}ms "
                        f"total_est={latency_info['total_est_s']*1000:.0f}ms "
                        f"odom_comp={latency_info['odom_comp_s']*1000:.0f}ms "
                        f"coverage={'OK' if latency_info['full_coverage'] else 'PARTIAL'} "
                        f"bearing={raw_bearing_text}->{comp_bearing_text}"
                    )

                if (
                    vision_result is not None
                    and
                    vision_reason == "VALID"
                    and
                    vision_result.get("on_floor", False)
                ):
                    # The compensated geometry represents vision_done_time.
                    # Use that timestamp here; the normal odom propagation below
                    # will advance it from done_time to the current control time.
                    measurement_stamp = (
                        vision_done_time
                        if vision_done_time > 0.0
                        else now
                    )

                    (
                        measurement_accepted,
                        measurement_status,
                        measurement_error,
                        measurement_tolerance
                    ) = target.consider_measurement(
                        vision_result,
                        measurement_stamp,
                        geometry_locked=center_controller.geometry_locked(now)
                    )

                    if measurement_accepted:
                        last_vision_reason = "VALID"

                        if measurement_status == "TARGET_LOCKED":
                            print(
                                "[TARGET] memory locked at "
                                "({:.3f}, {:+.3f}) m".format(
                                    target.x,
                                    target.y
                                )
                            )
                    else:
                        last_vision_reason = measurement_status

                        if measurement_status.startswith("TARGET_MISMATCH"):
                            print(
                                "[TARGET] rejected detection: "
                                "error={:.3f}m tolerance={:.3f}m | "
                                "memory=({:.3f},{:+.3f}) "
                                "candidate=({:.3f},{:+.3f})".format(
                                    measurement_error,
                                    measurement_tolerance,
                                    target.x,
                                    target.y,
                                    float(vision_result["x"]),
                                    float(vision_result["y"])
                                )
                            )


            # ==================================================
            # ROS snapshot
            # ==================================================

            scan = None

            odom = OdomState()


            if RUN_MODE != "VISION":

                (
                    scan,
                    odom
                ) = ros_bridge.snapshot()


                # ------------------------------------------------
                # YOLO/SAM 掉一兩幀時，用 odom 暫時更新 target
                # ------------------------------------------------

                if (

                    target.has_memory()

                    and

                    get_age(
                        now,
                        odom.stamp
                    )
                    <=
                    ODOM_STALE_TIMEOUT
                ):

                    target.propagate_with_odom(

                        odom.vx,

                        odom.wz,

                        now
                    )


            # ==================================================
            # 6 Hz PPO control
            # ==================================================

            safety_reason = last_safety_reason


            active_control_period = (
                CENTER_CONTROL_PERIOD
                if center_controller.high_rate_needed(now)
                else CONTROL_PERIOD
            )

            if (

                RUN_MODE != "VISION"

                and

                now
                -
                last_control_run
                >=
                active_control_period
            ):

                last_control_run = now


                scan_age = get_age(
                    now,
                    scan.stamp
                )

                odom_age = get_age(
                    now,
                    odom.stamp
                )

                camera_age = get_age(
                    now,
                    frame_time
                )

                target_age = get_age(
                    now,
                    target.last_seen
                )


                should_stop = False


                # ------------------------------------------------
                # Safety freshness
                # ------------------------------------------------

                if e_stop_latched:

                    safety_reason = (
                        "E_STOP_LATCHED"
                    )

                    should_stop = True


                elif arrived_latched:

                    safety_reason = (
                        "ARRIVED_LATCHED"
                    )

                    should_stop = True


                elif (
                    camera_age
                    >
                    CAMERA_STALE_TIMEOUT
                ):

                    safety_reason = (

                        "CAMERA_STALE "
                        f"{camera_age:.2f}s"
                    )

                    should_stop = True


                elif (
                    not target.has_memory()
                ):

                    safety_reason = (
                        "TARGET_NOT_LOCKED | "
                        f"{last_vision_reason}"
                    )

                    should_stop = True


                elif (
                    scan_age
                    >
                    SCAN_STALE_TIMEOUT
                ):

                    safety_reason = (

                        "SCAN_STALE "
                        f"{scan_age:.2f}s"
                    )

                    should_stop = True


                elif (
                    odom_age
                    >
                    ODOM_STALE_TIMEOUT
                ):

                    safety_reason = (

                        "ODOM_STALE "
                        f"{odom_age:.2f}s"
                    )

                    should_stop = True


                # ------------------------------------------------
                # LiDAR adapter
                # ------------------------------------------------

                lidar_48 = None

                front_robust = float(
                    "inf"
                )

                front_min = float(
                    "inf"
                )

                front_samples = 0

                right_side_robust = float("inf")
                right_side_min = float("inf")
                right_side_samples = 0

                left_side_robust = float("inf")
                left_side_min = float("inf")
                left_side_samples = 0

                if not should_stop:

                    lidar_48 = scan_to_policy_rays(
                        scan
                    )

                    (
                        front_robust,
                        front_min,
                        front_samples
                    ) = raw_front_metrics(
                        scan
                    )

                    (
                        right_side_robust,
                        right_side_min,
                        right_side_samples,
                        left_side_robust,
                        left_side_min,
                        left_side_samples
                    ) = raw_side_metrics(
                        scan
                    )

                    if lidar_48 is None:

                        safety_reason = (
                            "LIDAR_ADAPTER_FAILED"
                        )

                        should_stop = True

                    else:

                        last_front_robust = front_robust
                        last_front_min = front_min
                        last_front_samples = front_samples


                # ------------------------------------------------
                # Odom-memory visual reacquisition near the target
                # ------------------------------------------------

                target_searching = (
                    not should_stop
                    and
                    target.has_memory()
                    and
                    not target.visual_recent(now)
                    and
                    target.dist <= TARGET_SEARCH_START_DIST_M
                )

                if target_searching:
                    if target_search_start <= 0.0:
                        target_search_start = now
                        print(
                            "[TARGET] near remembered position; "
                            "searching toward bearing {:+.1f}deg".format(
                                math.degrees(target.bearing)
                            )
                        )

                    elif (
                        now - target_search_start
                        >
                        TARGET_SEARCH_TIMEOUT_S
                    ):
                        should_stop = True
                        safety_reason = "TARGET_SEARCH_TIMEOUT"
                else:
                    if (
                        target_search_start > 0.0
                        and
                        target.visual_recent(now)
                    ):
                        print("[TARGET] visual position reconfirmed")

                    target_search_start = 0.0


                # ------------------------------------------------
                # Bottom-of-frame final centering
                # ------------------------------------------------

                center_controlling = False
                center_wz_cmd = 0.0
                center_reason = ""
                center_just_started = False

                if not should_stop and not target_searching:

                    # FINAL_FORWARD 一旦開始，禁止 CENTER 在途中重新啟動。
                    # 之前 log 會反覆出現 CENTERED -> FINAL start，導致 0.55s
                    # 計時一直被重設；這裡直接把狀態互斥。
                    if not final_approach.active:
                        (
                            center_controlling,
                            center_wz_cmd,
                            center_reason,
                            center_just_started
                        ) = center_controller.update(
                            last_vision_result,
                            last_vision_reason,
                            vision_frame_time,
                            now
                        )

                        if center_just_started:
                            anti_jitter.reset_motion()

                            prev_raw_action = np.zeros(
                                2,
                                dtype=np.float32
                            )

                            action_delay = [
                                np.zeros(
                                    2,
                                    dtype=np.float32
                                )
                                for _ in range(
                                    MOTOR_DELAY_STEPS
                                )
                            ]

                        # bbox 連續確認置中後，只啟動一次 FINAL_FORWARD。
                        if (
                            not center_controlling
                            and
                            center_controller.consume_final_approach_ready(target, now)
                        ):
                            anti_jitter.reset_motion()
                            final_approach.start(
                                target,
                                now
                            )
                    else:
                        center_controlling = False
                        center_wz_cmd = 0.0
                        center_reason = ""
                        center_just_started = False


                # ------------------------------------------------
                # Reached sugar box
                #
                # 如果畫面底部且仍需要置中，先完成原地旋轉，
                # 不要因為 dist <= STOP_DISTANCE_M 直接 ARRIVED。
                # ------------------------------------------------

                # 不再使用 target.dist <= STOP_DISTANCE_M 提前宣告 ARRIVED。
                # 必須先看到 sugar box 進入畫面底部 -> 完成視覺置中 ->
                # 再實際往前約 FINAL_APPROACH_TRAVEL_M，最後才鎖定 ARRIVED。


                # ------------------------------------------------
                # STOP
                # ------------------------------------------------

                if should_stop:

                    center_controller.reset()
                    final_approach.reset()

                    last_raw_action[:] = 0.0

                    last_desired_vx = 0.0

                    last_desired_wz = 0.0

                    last_cmd_vx = 0.0

                    last_cmd_wz = 0.0

                    # sensor/target/E-stop 等安全停止。
                    anti_jitter.reset_motion()

                    prev_raw_action = np.zeros(
                        2,
                        dtype=np.float32
                    )

                    action_delay = [
                        np.zeros(
                            2,
                            dtype=np.float32
                        )
                        for _ in range(
                            MOTOR_DELAY_STEPS
                        )
                    ]


                    if RUN_MODE == "DRIVE":

                        motor.send_velocity(
                            0.0,
                            0.0
                        )


                # ------------------------------------------------
                # Reacquire the remembered target near its odom position
                # ------------------------------------------------

                elif target_searching:

                    center_controller.reset()
                    final_approach.reset()
                    last_raw_action[:] = 0.0
                    prev_raw_action = np.zeros(
                        2,
                        dtype=np.float32
                    )
                    action_delay = [
                        np.zeros(
                            2,
                            dtype=np.float32
                        )
                        for _ in range(
                            MOTOR_DELAY_STEPS
                        )
                    ]

                    search_sign = (
                        1.0
                        if target.bearing >= 0.0
                        else -1.0
                    )
                    search_wz = search_sign * clamp(
                        abs(TARGET_SEARCH_KP * target.bearing),
                        TARGET_SEARCH_MIN_WZ,
                        TARGET_SEARCH_MAX_WZ
                    )

                    last_desired_vx = 0.0
                    last_desired_wz = float(search_wz)

                    (
                        vx_cmd,
                        wz_cmd,
                        search_safety_reason
                    ) = anti_jitter.apply_center_turn(
                        search_wz,
                        front_robust,
                        front_min
                    )

                    last_cmd_vx = float(vx_cmd)
                    last_cmd_wz = float(wz_cmd)
                    safety_reason = (
                        "TARGET_SEARCH | "
                        +
                        search_safety_reason
                    )

                    if RUN_MODE == "DRIVE":
                        motor.send_velocity(
                            last_cmd_vx,
                            last_cmd_wz
                        )


                # ------------------------------------------------
                # Final centering override
                # ------------------------------------------------

                elif center_controlling:

                    # 這一段刻意 bypass PPO。
                    # FINAL CENTER 完全使用 image pixel 做 stop-and-look：
                    # 短轉 -> 停 -> 等 fresh vision -> 再決定。
                    last_raw_action[:] = 0.0

                    prev_raw_action = np.zeros(
                        2,
                        dtype=np.float32
                    )

                    action_delay = [
                        np.zeros(
                            2,
                            dtype=np.float32
                        )
                        for _ in range(
                            MOTOR_DELAY_STEPS
                        )
                    ]

                    last_desired_vx = 0.0
                    last_desired_wz = float(center_wz_cmd)

                    # 每一個 visual turn pulse 結束都要立刻停，
                    # 等 fresh frame 時不可殘留任何旋轉命令。
                    center_immediate_stop = (
                        abs(center_wz_cmd) < 1e-9
                    )

                    if center_immediate_stop:

                        anti_jitter.reset_motion()
                        vx_cmd = 0.0
                        wz_cmd = 0.0
                        safety_reason = center_reason

                    else:

                        (
                            vx_cmd,
                            wz_cmd,
                            center_safety_reason
                        ) = anti_jitter.apply_center_turn(
                            center_wz_cmd,
                            front_robust,
                            front_min
                        )

                        # hard stop 永遠比置中優先；
                        # 其他情況 center-turn 直接送，不再經過 EMA/slew/deadzone。
                        if center_safety_reason.startswith(
                            "HARD_STOP"
                        ):
                            safety_reason = center_safety_reason
                        else:
                            safety_reason = (
                                center_reason
                                +
                                " | "
                                +
                                center_safety_reason
                            )

                    last_cmd_vx = float(vx_cmd)
                    last_cmd_wz = float(wz_cmd)

                    if RUN_MODE == "DRIVE":

                        motor.send_velocity(
                            last_cmd_vx,
                            last_cmd_wz,
                            force=center_immediate_stop
                        )


                # ------------------------------------------------
                # Final straight approach after centering
                # ------------------------------------------------

                elif final_approach.active:

                    # 畫面置中後只做固定時間的小段直走；
                    # 不再讀 bearing，也不再做額外角度修正。
                    last_raw_action[:] = 0.0

                    prev_raw_action = np.zeros(
                        2,
                        dtype=np.float32
                    )

                    action_delay = [
                        np.zeros(
                            2,
                            dtype=np.float32
                        )
                        for _ in range(
                            MOTOR_DELAY_STEPS
                        )
                    ]

                    (
                        _final_controlling,
                        final_vx_des,
                        final_wz_des,
                        final_reason
                    ) = final_approach.command(
                        target,
                        odom.vx,
                        now
                    )

                    last_desired_vx = float(final_vx_des)
                    last_desired_wz = float(final_wz_des)

                    if final_reason.startswith("FINAL_FORWARD"):

                        (
                            vx_cmd,
                            wz_cmd,
                            final_safety_reason
                        ) = anti_jitter.apply_final_approach(
                            final_vx_des,
                            front_robust,
                            front_min
                        )

                        if (
                            final_safety_reason.startswith("HARD_STOP")
                            or
                            final_safety_reason.startswith("FWD_BLOCK")
                        ):
                            safety_reason = final_safety_reason
                        else:
                            safety_reason = (
                                final_reason
                                +
                                " | "
                                +
                                final_safety_reason
                            )

                    else:
                        anti_jitter.reset_motion()
                        vx_cmd = 0.0
                        wz_cmd = 0.0
                        safety_reason = final_reason

                        if final_reason.startswith("FINAL_DONE_TRAVEL"):
                            arrived_latched = True
                            safety_reason = "ARRIVED_AFTER_15CM"

                    last_cmd_vx = float(vx_cmd)
                    last_cmd_wz = float(wz_cmd)

                    if RUN_MODE == "DRIVE":
                        motor.send_velocity(
                            last_cmd_vx,
                            last_cmd_wz,
                            force=final_reason.startswith("FINAL_DONE")
                        )
                        if EXIT_ON_ARRIVAL and arrived_latched:
                            motor.stop(repeat=5)
                            break


                # ------------------------------------------------
                # PPO
                # ------------------------------------------------

                else:

                    raw_obs = build_raw_obs(

                        target.dist,

                        target.bearing,

                        odom.vx,

                        odom.wz,

                        prev_raw_action,

                        lidar_48
                    )


                    raw_action = policy_predict(

                        rl_model,

                        vecnormalize,

                        raw_obs
                    )


                    last_raw_action = (
                        raw_action.copy()
                    )


                    # observation 下一幀要使用
                    # 上一幀 raw action
                    prev_raw_action = (
                        raw_action.copy()
                    )


                    # ------------------------------------------------
                    # 2-step motor delay
                    # ------------------------------------------------

                    action_delay.append(
                        raw_action.copy()
                    )

                    delayed_action = (
                        action_delay.pop(
                            0
                        )
                    )


                    # ------------------------------------------------
                    # action -> vx/wz
                    # ------------------------------------------------

                    (
                        vx_cmd,
                        wz_cmd
                    ) = action_to_velocity(
                        delayed_action
                    )


                    # ------------------------------------------------
                    # Anti-jitter safety layer
                    # ------------------------------------------------

                    # 保存 PPO + action-delay 後的原始真車 command。
                    # 這版 safety layer 不再做 temporal smoothing。
                    last_desired_vx = float(vx_cmd)
                    last_desired_wz = float(wz_cmd)

                    (
                        vx_cmd,
                        wz_cmd,
                        safety_reason,
                        _slow_scale
                    ) = anti_jitter.apply(
                        vx_cmd,
                        wz_cmd,
                        front_robust,
                        front_min,
                        right_side_robust,
                        left_side_robust,
                        odom.vx,
                        now
                    )

                    last_cmd_vx = vx_cmd
                    last_cmd_wz = wz_cmd


                    # ------------------------------------------------
                    # 真正送馬達
                    # ------------------------------------------------

                    if RUN_MODE == "DRIVE":

                        motor.send_velocity(
                            vx_cmd,
                            wz_cmd
                        )


                # HUD / terminal 在兩次 6 Hz control 之間也保留最後狀態。
                last_safety_reason = safety_reason


            # ==================================================
            # HUD
            # ==================================================

            if RUN_MODE == "VISION":

                sensor_text = (
                    "ROS / PPO disabled"
                )

                rl_text = (
                    "RL disabled"
                )

                safety_text = (
                    "VISION ONLY"
                )

            else:

                (
                    scan_display,
                    odom_display
                ) = ros_bridge.snapshot()


                sensor_text = (

                    f"scan_age="
                    f"{get_age(now, scan_display.stamp):.2f}s "

                    f"odom_age="
                    f"{get_age(now, odom_display.stamp):.2f}s "

                    f"vx="
                    f"{odom_display.vx:+.3f} "

                    f"wz="
                    f"{odom_display.wz:+.3f} "

                    f"front="
                    f"{last_front_robust:.3f} "
                    f"min="
                    f"{last_front_min:.3f} "
                    f"n="
                    f"{last_front_samples} "

                    f"lat="
                    f"{last_vision_latency['total_est_s']*1000:.0f}ms"
                )


                rl_text = (

                    f"a=["
                    f"{last_raw_action[0]:+.3f},"
                    f"{last_raw_action[1]:+.3f}] "

                    f"des="
                    f"({last_desired_vx:+.3f},"
                    f"{last_desired_wz:+.3f}) "

                    f"cmd="
                    f"({last_cmd_vx:+.3f},"
                    f"{last_cmd_wz:+.3f})"
                )


                safety_text = (
                    safety_reason
                )


            if (
                now - last_display_run
                >= DISPLAY_PERIOD
            ):

                last_display_run = now

                # 單一視窗：主畫面維持最新 live frame。
                # 視覺辨識結果放在右下角 inset，且 inset 使用「真正做 inference 的那張 frame」，
                # 因此 bbox / SAM 不會被錯疊到較新的 live frame 上。
                live_display = draw_hud(

                    last_frame,

                    last_vision_result,

                    last_vision_reason,

                    target,

                    RUN_MODE,

                    sensor_text,

                    rl_text,

                    safety_text,

                    draw_vision_overlay=False
                )

                # FINAL visual centering 的真正判定區：畫面中心 ± tolerance pixels。
                center_x = FRAME_W // 2
                tol_px = int(round(VISUAL_CENTER_TOL_PX))

                cv2.line(
                    live_display,
                    (center_x, 150),
                    (center_x, FRAME_H),
                    (255, 255, 255),
                    1
                )

                cv2.line(
                    live_display,
                    (center_x - tol_px, 150),
                    (center_x - tol_px, FRAME_H),
                    (180, 180, 180),
                    1
                )

                cv2.line(
                    live_display,
                    (center_x + tol_px, 150),
                    (center_x + tol_px, FRAME_H),
                    (180, 180, 180),
                    1
                )

                if last_vision_panel is not None:
                    inset_w = 256
                    inset_h = 192
                    inset = cv2.resize(
                        last_vision_panel,
                        (inset_w, inset_h),
                        interpolation=cv2.INTER_AREA
                    )

                    inset_center_x = inset_w // 2
                    inset_tol = int(round(
                        VISUAL_CENTER_TOL_PX
                        *
                        inset_w
                        /
                        FRAME_W
                    ))

                    cv2.line(
                        inset,
                        (inset_center_x, 0),
                        (inset_center_x, inset_h),
                        (255, 255, 255),
                        1
                    )

                    cv2.line(
                        inset,
                        (inset_center_x - inset_tol, 0),
                        (inset_center_x - inset_tol, inset_h),
                        (180, 180, 180),
                        1
                    )

                    cv2.line(
                        inset,
                        (inset_center_x + inset_tol, 0),
                        (inset_center_x + inset_tol, inset_h),
                        (180, 180, 180),
                        1
                    )

                    cv2.rectangle(
                        inset,
                        (0, 0),
                        (inset_w - 1, inset_h - 1),
                        (255, 255, 255),
                        2
                    )

                    cv2.rectangle(
                        inset,
                        (0, 0),
                        (115, 22),
                        (0, 0, 0),
                        -1
                    )

                    cv2.putText(
                        inset,
                        "VISION",
                        (7, 16),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.48,
                        (255, 255, 255),
                        1
                    )

                    x0 = FRAME_W - inset_w - 8
                    y0 = FRAME_H - inset_h - 8

                    live_display[
                        y0:y0 + inset_h,
                        x0:x0 + inset_w
                    ] = inset

                cv2.imshow(
                    "Sugarbox RL Approach",
                    live_display
                )


            # ==================================================
            # Terminal log
            # ==================================================

            if (
                now
                -
                last_print
                >=
                PRINT_INTERVAL
            ):

                last_print = now


                if RUN_MODE == "VISION":

                    print(

                        "[VISION] "

                        f"reason="
                        f"{last_vision_reason} | "

                        f"x="
                        f"{target.x:.3f} | "

                        f"y="
                        f"{target.y:+.3f} | "

                        f"dist="
                        f"{target.dist:.3f} | "

                        f"bearing="
                        f"{math.degrees(target.bearing):+.1f}deg | "

                        f"age="
                        f"{get_age(now, target.last_seen):.2f}s"
                    )


                else:

                    print(

                        "[RUN] "

                        f"mode="
                        f"{RUN_MODE} | "

                        f"vision="
                        f"{last_vision_reason} | "

                        f"target=("
                        f"{target.x:.3f},"
                        f"{target.y:+.3f}) | "

                        f"dist="
                        f"{target.dist:.3f} | "

                        f"bearing="
                        f"{math.degrees(target.bearing):+.1f}deg | "

                        f"odom "
                        f"vx="
                        f"{odom.vx:+.3f} "

                        f"wz="
                        f"{odom.wz:+.3f} | "

                        f"a=("
                        f"{last_raw_action[0]:+.3f},"
                        f"{last_raw_action[1]:+.3f}) | "

                        f"des=("
                        f"{last_desired_vx:+.3f},"
                        f"{last_desired_wz:+.3f}) | "

                        f"cmd=("
                        f"{last_cmd_vx:+.3f},"
                        f"{last_cmd_wz:+.3f}) | "

                        f"front="
                        f"{last_front_robust:.3f}/"
                        f"min{last_front_min:.3f} | "

                        f"lat="
                        f"{last_vision_latency['total_est_s']*1000:.0f}ms | "

                        f"{last_safety_reason}"
                    )


            # ==================================================
            # Keyboard
            # ==================================================

            key = (
                cv2.waitKey(1)
                &
                0xFF
            )


            # ESC
            if key == 27:

                break


            # SPACE = emergency stop
            elif key == 32:

                e_stop_latched = True

                print(
                    "[KEY] E-STOP LATCHED"
                )

                if RUN_MODE == "DRIVE":

                    motor.stop(
                        repeat=5
                    )


            # R = clear latch
            elif key in (

                ord("r"),

                ord("R")
            ):

                e_stop_latched = False

                arrived_latched = False

                prev_raw_action = np.zeros(

                    2,

                    dtype=np.float32
                )

                action_delay = [

                    np.zeros(
                        2,
                        dtype=np.float32
                    )

                    for _ in range(
                        MOTOR_DELAY_STEPS
                    )
                ]

                anti_jitter.reset_all()
                center_controller.reset()
                final_approach.reset()
                target_search_start = 0.0
                last_safety_reason = "RESET"

                print(
                    "[KEY] stop / arrival latch cleared"
                )


            # C = forget target memory and require a fresh multi-frame lock
            elif key in (
                ord("c"),
                ord("C")
            ):

                target.clear()
                target_search_start = 0.0
                center_controller.reset()
                final_approach.reset()
                anti_jitter.reset_all()
                last_safety_reason = "TARGET_MEMORY_CLEARED"

                if RUN_MODE == "DRIVE":
                    motor.stop(repeat=5)

                print(
                    "[KEY] target memory cleared; waiting for "
                    "{} consistent detections".format(
                        TARGET_ACQUIRE_CONFIRM_FRAMES
                    )
                )


            time.sleep(
                0.005
            )


    except KeyboardInterrupt:

        print(
            "\n[INFO] Ctrl+C"
        )


    finally:

        print(
            "[INFO] stopping safely..."
        )


        if motor is not None:

            try:

                motor.stop(
                    repeat=8
                )

            except Exception:
                pass

            motor.close()


        if ros_bridge is not None:

            ros_bridge.close()


        try:
            vision_worker.release()
        except Exception:
            pass

        reader.release()

        cv2.destroyAllWindows()

        print(
            "[INFO] ended"
        )

    if EXIT_ON_ARRIVAL:
        return 0 if arrived_latched and RUN_MODE == "DRIVE" else 3
    return 0


if __name__ == "__main__":

    if "--selftest-target-memory" in sys.argv:
        run_target_memory_selftest()
    else:
        raise SystemExit(main())
