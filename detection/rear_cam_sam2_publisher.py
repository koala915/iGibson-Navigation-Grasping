#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""後相機 YOLO + SAM2 目標發布器（在 Windows 開發機上執行，不在 Jetson 上）。

與 ``detection/arm_cam.py`` 是兩條不同的路線：

    arm_cam.py                本檔
    手臂相機（arm_link4）      後相機（HTTP 或 RTP/H.264 UDP）
    bbox 底邊中點              SAM2 遮罩最低點
    僅在 home 姿勢成立         底盤座標，與手臂姿態無關
    TCP 5555 送夾取端          rosbridge 發 /trash_target/detection

取遮罩最低點而非 bbox 底邊，是因為 bbox 底邊會被物體的透視外擴撐開，
遮罩的最低點才是真正的接地點。代價是多跑一次 SAM2，所以這支放在
開發機上跑，不佔 Jetson 的 RAM。

執行前確認以下檔案：

    detection/sam2.1_b.pt                SAM2 權重，Ultralytics 格式
    detection/rear_ground_homography.json  已隨 repo 提供的實機後相機地面校正

校正資料量測於 2026-08-06，畫面為 640×480；更換相機安裝位置或影像尺寸後，
請以 calibration/calibrate_rear_ground_homography.py 重新校正。

    ⚠ .gitignore 會擋掉 *.pt。放進來的 sam2 權重不會進版控，
      這是刻意的（權重不進 repo），但 git status 不會提醒你。

先安裝符合顯示卡 CUDA 的 PyTorch，再執行
``pip install -r detection/requirements_offboard.txt``。UDP 模式另需 GStreamer，
並以 ``--gst-exe`` 或 ``GSTREAMER_LAUNCH_EXE`` 指定 gst-launch-1.0。

輸出 topic ``/trash_target/detection``（std_msgs/String，內容是 JSON），
座標系為 ``base_footprint``、``x_forward_y_left``。連續 5 幀在門檻內才
標記 ``valid: true``，未穩定或幾何不合理時仍會發布但 ``valid: false``。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple


import cv2
import numpy as np
import roslibpy
import torch
from ultralytics import SAM, YOLO




# ============================================================
# 1. 網路設定
# ============================================================


# X3PLUS_JETSON_HOST 是本專案其他八支程式共用的變數（見 set_jetson_host.ps1），
# 這裡沿用同一個，額外接受 JETSON_IP 以相容本檔原本的用法。
JETSON_IP = os.environ.get(
    "X3PLUS_JETSON_HOST",
    os.environ.get(
        "JETSON_IP",
        "yahboom.local",
    ),
)


ROSBRIDGE_PORT = int(
    os.environ.get(
        "ROSBRIDGE_PORT",
        "9090",
    )
)


REAR_CAMERA_URL = os.environ.get(
    "REAR_CAMERA_URL",
    f"http://{JETSON_IP}:8080/stream?topic=/back_cam/image_raw",
)


PUBLISH_TOPIC = "/trash_target/detection"
PUBLISH_TYPE = "std_msgs/String"




# ============================================================
# 2. 檔案路徑
# ============================================================


PROJECT_DIR = Path(__file__).resolve().parent


# 專案內既有的 sugarbox 模型（nc=1, names=['sugarbox']）。
YOLO_MODEL_PATH = PROJECT_DIR / "models" / "best.pt"

# 以下兩個不在 repo 內，須自備放進 detection/。
SAM2_MODEL_PATH = PROJECT_DIR / "sam2.1_b.pt"
HOMOGRAPHY_JSON_PATH = PROJECT_DIR / "rear_ground_homography.json"




# ============================================================
# 3. 視覺參數
# ============================================================


FRAME_WIDTH = 640
FRAME_HEIGHT = 480


YOLO_CONFIDENCE = 0.45
YOLO_IOU = 0.65
YOLO_IMAGE_SIZE = 640
MAX_DETECTIONS = 10


TARGET_CLASS_NAMES = {
    "sugarbox",
}


SAM_MASK_THRESHOLD = 0.5
MIN_MASK_AREA_PX = 50
MAX_MASK_AREA_RATIO = 0.45
MAX_SAM_BOTTOM_TO_BBOX_BOTTOM_PX = 30
CALIBRATION_MARGIN_PX = 10.0


MIN_OBJECT_DISTANCE_M = 0.15
MAX_OBJECT_DISTANCE_M = 3.00
MAX_LATERAL_DISTANCE_M = 1.50


STABLE_REQUIRED_FRAMES = 5
MAX_STABLE_X_SPREAD_M = 0.15
MAX_STABLE_Y_SPREAD_M = 0.15
MAX_STABLE_U_SPREAD_PX = 24
MAX_STABLE_V_SPREAD_PX = 24


PUBLISH_INTERVAL_SEC = 0.20


WINDOW_NAME = "Windows YOLO + SAM2 Sugarbox Target"




# ============================================================
# 4. 工具函式
# ============================================================


def clamp(
    value: float,
    lower: float,
    upper: float,
) -> float:
    return max(lower, min(upper, value))




def normalize_class_name(name: Any) -> str:
    return str(name).strip().lower()




def get_model_class_name(
    names: Any,
    class_id: int,
) -> str:
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))


    if isinstance(names, (list, tuple)):
        if 0 <= class_id < len(names):
            return str(names[class_id])


    return str(class_id)




def resize_binary_mask(
    mask: np.ndarray,
    width: int,
    height: int,
) -> np.ndarray:
    if mask.shape[:2] != (height, width):
        mask = cv2.resize(
            mask.astype(np.uint8),
            (width, height),
            interpolation=cv2.INTER_NEAREST,
        )


    return mask.astype(bool)




def largest_component_near_point(
    mask: np.ndarray,
    point_xy: Tuple[int, int],
) -> np.ndarray:
    binary = mask.astype(np.uint8)


    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        binary,
        connectivity=8,
    )


    if count <= 1:
        return mask.astype(bool)


    point_x, point_y = point_xy
    chosen_label = 0


    if 0 <= point_y < labels.shape[0] and 0 <= point_x < labels.shape[1]:
        chosen_label = int(labels[point_y, point_x])


    if chosen_label == 0:
        best_score = float("inf")


        for label_id in range(1, count):
            area = int(stats[label_id, cv2.CC_STAT_AREA])


            if area < MIN_MASK_AREA_PX:
                continue


            center_x, center_y = centroids[label_id]


            distance = math.hypot(
                center_x - point_x,
                center_y - point_y,
            )


            score = distance - 0.002 * area


            if score < best_score:
                best_score = score
                chosen_label = label_id


    if chosen_label == 0:
        chosen_label = 1 + int(
            np.argmax(
                stats[1:, cv2.CC_STAT_AREA]
            )
        )


    return labels == chosen_label




def find_mask_bottom_point(
    mask: np.ndarray,
) -> Optional[Tuple[int, int]]:
    y_indices, x_indices = np.where(mask)


    if len(x_indices) == 0:
        return None


    bottom_y = int(np.max(y_indices))


    bottom_x_values = x_indices[
        y_indices == bottom_y
    ]


    if len(bottom_x_values) == 0:
        return None


    bottom_x = int(np.median(bottom_x_values))


    return bottom_x, bottom_y




def make_json_safe(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()


    if isinstance(value, np.ndarray):
        return value.tolist()


    if isinstance(value, dict):
        return {
            str(key): make_json_safe(item)
            for key, item in value.items()
        }


    if isinstance(value, (list, tuple)):
        return [
            make_json_safe(item)
            for item in value
        ]


    if isinstance(value, Path):
        return str(value)


    return value




# ============================================================
# 5. 最新影像讀取器
# ============================================================


class LatestFrameReader:
    def __init__(
        self,
        url: str,
        width: int,
        height: int,
    ) -> None:
        self.url = url
        self.width = width
        self.height = height


        self.lock = threading.Lock()
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_time = 0.0


        self.running = True
        self.capture: Optional[cv2.VideoCapture] = None


        self.thread = threading.Thread(
            target=self._loop,
            daemon=True,
        )
        self.thread.start()


    def _open_capture(self) -> None:
        if self.capture is not None:
            self.capture.release()


        print(f"[CAMERA] Opening: {self.url}")


        self.capture = cv2.VideoCapture(self.url)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)


    def _loop(self) -> None:
        while self.running:
            if self.capture is None or not self.capture.isOpened():
                self._open_capture()
                time.sleep(0.5)


                if self.capture is None or not self.capture.isOpened():
                    time.sleep(1.0)
                    continue


            success, frame = self.capture.read()


            if not success or frame is None:
                print("[CAMERA] Frame read failed. Reconnect...")
                self.capture.release()
                self.capture = None
                time.sleep(0.5)
                continue


            frame = cv2.resize(
                frame,
                (self.width, self.height),
            )


            with self.lock:
                self.latest_frame = frame
                self.latest_time = time.monotonic()


    def read(
        self,
    ) -> Tuple[Optional[np.ndarray], float]:
        with self.lock:
            if self.latest_frame is None:
                return None, 0.0


            return self.latest_frame.copy(), self.latest_time


    def close(self) -> None:
        self.running = False
        self.thread.join(timeout=1.0)


        if self.capture is not None:
            self.capture.release()


class GStreamerUdpFrameReader:
    """Low-latency RTP/H.264 reader using gst-launch stdout on Windows."""

    def __init__(self, gst_exe: str, udp_port: int, width: int, height: int) -> None:
        self.width = int(width)
        self.height = int(height)
        self.frame_bytes = self.width * self.height * 3
        if not Path(gst_exe).is_file():
            raise FileNotFoundError(
                f"GStreamer executable not found: {gst_exe}. "
                "Pass --gst-exe or set GSTREAMER_LAUNCH_EXE.")
        args = [
            gst_exe, "-q", "udpsrc", f"port={int(udp_port)}",
            "caps=application/x-rtp,media=video,encoding-name=H264,payload=96",
            "!", "rtph264depay", "!", "h264parse", "!", "avdec_h264",
            "!", "videoconvert", "!",
            f"video/x-raw,format=BGR,width={self.width},height={self.height}",
            "!", "queue", "max-size-buffers=1", "max-size-bytes=0",
            "max-size-time=0", "leaky=downstream", "!", "fdsink",
            "fd=1", "sync=false",
        ]
        print(f"[CAMERA] GStreamer RTP/H.264 UDP port={udp_port}")
        self.process = subprocess.Popen(args, stdout=subprocess.PIPE,
                                        stderr=None, bufsize=0)
        if self.process.stdout is None:
            raise RuntimeError("failed to create GStreamer stdout pipe")
        self.lock = threading.Lock()
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_time = 0.0
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _read_exact(self, size: int) -> Optional[bytes]:
        data = bytearray()
        while self.running and len(data) < size:
            chunk = self.process.stdout.read(size - len(data))
            if not chunk:
                return None
            data.extend(chunk)
        return bytes(data) if len(data) == size else None

    def _loop(self) -> None:
        while self.running:
            raw = self._read_exact(self.frame_bytes)
            if raw is None:
                if self.running:
                    time.sleep(0.01)
                continue
            frame = np.frombuffer(raw, dtype=np.uint8).reshape(
                self.height, self.width, 3)
            with self.lock:
                self.latest_frame = frame
                self.latest_time = time.monotonic()

    def read(self) -> Tuple[Optional[np.ndarray], float]:
        with self.lock:
            if self.latest_frame is None:
                return None, 0.0
            return self.latest_frame, self.latest_time

    def close(self) -> None:
        self.running = False
        try:
            self.process.stdout.close()
        except Exception:
            pass
        try:
            self.process.terminate()
            self.process.wait(timeout=1.0)
        except Exception:
            try:
                self.process.kill()
            except Exception:
                pass
        self.thread.join(timeout=0.5)




# ============================================================
# 6. Homography 投影
# ============================================================


class GroundProjector:
    def __init__(
        self,
        json_path: Path,
    ) -> None:
        self.json_path = json_path
        self.homography: Optional[np.ndarray] = None
        self.rear_cam_to_front_m = 0.20
        self.calibration_hull: Optional[np.ndarray] = None


        if not json_path.is_file():
            raise FileNotFoundError(f"Homography JSON 不存在：{json_path}")


        with json_path.open("r", encoding="utf-8") as file:
            data = json.load(file)


        matrix = data.get(
            "H_pixel_to_ground",
            data.get(
                "homography",
                data.get("homography_image_to_ground"),
            ),
        )


        if matrix is None:
            raise KeyError(
                "Homography JSON 找不到："
                "H_pixel_to_ground、homography "
                "或 homography_image_to_ground"
            )


        homography = np.asarray(matrix, dtype=np.float64)


        if homography.shape != (3, 3):
            raise ValueError(
                "Homography 必須是 3x3，"
                f"目前為 {homography.shape}"
            )


        coordinate = data.get("coordinate", {})


        self.rear_cam_to_front_m = float(
            data.get(
                "rear_cam_to_front_m",
                coordinate.get(
                    "rear_cam_to_front_m",
                    0.20,
                ),
            )
        )


        self.homography = homography

        samples = data.get("samples") or []
        pixels = []
        for sample in samples:
            if not isinstance(sample, dict) or sample.get("inlier") is False:
                continue
            try:
                u, v = float(sample["u"]), float(sample["v"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(u) and math.isfinite(v):
                pixels.append([u, v])
        if len(pixels) >= 3:
            points = np.asarray(pixels, dtype=np.float32).reshape(-1, 1, 2)
            self.calibration_hull = cv2.convexHull(points)


        print("[GROUND] Homography loaded")
        print(f"[GROUND] File: {json_path}")
        print(f"[GROUND] rear_cam_to_front_m={self.rear_cam_to_front_m:.3f}")

    def inside_calibration_hull(self, pixel_u: float, pixel_v: float,
                                margin_px: float = CALIBRATION_MARGIN_PX) -> bool:
        if self.calibration_hull is None:
            return False
        signed_distance = cv2.pointPolygonTest(
            self.calibration_hull, (float(pixel_u), float(pixel_v)), True)
        return bool(signed_distance >= -float(margin_px))


    def pixel_to_robot_xy(
        self,
        pixel_u: int,
        pixel_v: int,
    ) -> Optional[Dict[str, float]]:
        if self.homography is None:
            return None


        pixel = np.asarray(
            [float(pixel_u), float(pixel_v), 1.0],
            dtype=np.float64,
        )


        ground = self.homography @ pixel
        scale = float(ground[2])


        if abs(scale) < 1e-9:
            return None


        x_camera_ground_m = float(ground[0] / scale)
        y_left_m = float(ground[1] / scale)


        x_robot_m = x_camera_ground_m - self.rear_cam_to_front_m


        values = np.asarray(
            [
                x_camera_ground_m,
                x_robot_m,
                y_left_m,
            ],
            dtype=np.float64,
        )


        if not np.isfinite(values).all():
            return None


        distance_robot_m = math.hypot(x_robot_m, y_left_m)
        bearing_robot_rad = math.atan2(y_left_m, x_robot_m)


        return {
            "x_camera_ground_m": x_camera_ground_m,
            "object_x_base": x_robot_m,
            "object_y_base": y_left_m,
            "distance_m": distance_robot_m,
            "bearing_rad": bearing_robot_rad,
        }




# ============================================================
# 7. ROSBridge 發布器
# ============================================================


class TrashTargetPublisher:
    def __init__(
        self,
        host: str,
        port: int,
    ) -> None:
        print(f"[ROSBRIDGE] Connecting to ws://{host}:{port}")


        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run(timeout=10)


        if not self.ros.is_connected:
            raise ConnectionError("無法連線 Jetson rosbridge")


        self.publisher = roslibpy.Topic(
            self.ros,
            PUBLISH_TOPIC,
            PUBLISH_TYPE,
        )


        self.publisher.advertise()


        print("[ROSBRIDGE] Connected")
        print(f"[ROSBRIDGE] Topic={PUBLISH_TOPIC}")


    def publish(
        self,
        payload: Dict[str, Any],
    ) -> None:
        safe_payload = make_json_safe(payload)


        json_text = json.dumps(
            safe_payload,
            ensure_ascii=False,
            sort_keys=True,
        )


        self.publisher.publish(
            roslibpy.Message(
                {
                    "data": json_text,
                }
            )
        )


    def close(self) -> None:
        try:
            self.publisher.unadvertise()
        except Exception:
            pass


        try:
            self.ros.terminate()
        except Exception:
            pass




# ============================================================
# 8. 主視覺程式
# ============================================================


class SugarboxVisionPublisher:
    def __init__(self, *, jetson_host: str = JETSON_IP,
                 ros_port: int = ROSBRIDGE_PORT,
                 camera_url: str = REAR_CAMERA_URL,
                 camera_backend: str = "http",
                 gst_exe: Optional[str] = None,
                 udp_port: int = 5600,
                 yolo_model: Path = YOLO_MODEL_PATH,
                 sam_model: Path = SAM2_MODEL_PATH,
                 homography: Path = HOMOGRAPHY_JSON_PATH) -> None:
        self.yolo_model_path = Path(yolo_model)
        self.sam_model_path = Path(sam_model)
        self.homography_path = Path(homography)
        self._verify_files()


        self.device: Any = 0 if torch.cuda.is_available() else "cpu"


        print(f"[MODEL] Device={self.device}")
        print(f"[MODEL] Loading YOLO: {self.yolo_model_path}")


        self.yolo = YOLO(str(self.yolo_model_path))


        print(f"[MODEL] YOLO classes={self.yolo.names}")


        self.target_class_ids = self._resolve_target_class_ids()


        print(f"[MODEL] Loading SAM2: {self.sam_model_path}")
        self.sam = SAM(str(self.sam_model_path))


        self.projector = GroundProjector(self.homography_path)


        if camera_backend == "udp":
            executable = gst_exe or os.environ.get(
                "GSTREAMER_LAUNCH_EXE", r"D:\msvc_x86_64\bin\gst-launch-1.0.exe")
            self.frame_reader = GStreamerUdpFrameReader(
                executable, udp_port, FRAME_WIDTH, FRAME_HEIGHT)
        else:
            self.frame_reader = LatestFrameReader(
                camera_url, FRAME_WIDTH, FRAME_HEIGHT)
        self.camera_description = (f"udp://0.0.0.0:{udp_port}"
                                   if camera_backend == "udp" else camera_url)


        self.publisher = TrashTargetPublisher(
            jetson_host,
            ros_port,
        )


        self.history: Deque[Dict[str, float]] = deque(
            maxlen=STABLE_REQUIRED_FRAMES
        )


        self.last_publish_time = 0.0


    def _verify_files(self) -> None:
        required_files = [
            self.yolo_model_path,
            self.sam_model_path,
            self.homography_path,
        ]


        for file_path in required_files:
            if not file_path.is_file():
                raise FileNotFoundError(f"找不到檔案：{file_path}")


    def _resolve_target_class_ids(self) -> Optional[List[int]]:
        names = self.yolo.names
        selected_ids: List[int] = []


        if isinstance(names, dict):
            iterator = names.items()
        else:
            iterator = enumerate(names)


        for class_id, class_name in iterator:
            normalized_name = normalize_class_name(class_name)


            if normalized_name in TARGET_CLASS_NAMES:
                selected_ids.append(int(class_id))


        if selected_ids:
            print(f"[MODEL] Selected target class IDs={selected_ids}")
            return selected_ids


        number_of_classes = len(names)


        if number_of_classes == 1:
            print("[MODEL] Single-class model. Accepting class 0.")
            return [0]


        raise RuntimeError(f"模型中找不到 sugarbox 類別：{names}")


    def _choose_best_box(
        self,
        result: Any,
    ) -> Optional[Tuple[int, int, int, int, int, float]]:
        if result.boxes is None or len(result.boxes) == 0:
            return None


        best_box = None
        best_score = -1.0


        for box in result.boxes:
            class_id = int(box.cls[0].item())


            if self.target_class_ids is not None and class_id not in self.target_class_ids:
                continue


            confidence = float(box.conf[0].item())


            values = (
                box.xyxy[0]
                .detach()
                .cpu()
                .numpy()
                .tolist()
            )


            x1, y1, x2, y2 = values


            x1 = int(clamp(round(x1), 0, FRAME_WIDTH - 1))
            y1 = int(clamp(round(y1), 0, FRAME_HEIGHT - 1))
            x2 = int(clamp(round(x2), x1 + 1, FRAME_WIDTH))
            y2 = int(clamp(round(y2), y1 + 1, FRAME_HEIGHT))


            area = max(1, x2 - x1) * max(1, y2 - y1)
            score = confidence + 0.00002 * area


            if score > best_score:
                best_score = score
                best_box = (
                    x1,
                    y1,
                    x2,
                    y2,
                    class_id,
                    confidence,
                )


        return best_box


    def _create_sam_mask(
        self,
        frame: np.ndarray,
        bbox: Tuple[int, int, int, int],
    ) -> Optional[np.ndarray]:
        x1, y1, x2, y2 = bbox


        results = self.sam.predict(
            source=frame,
            bboxes=[x1, y1, x2, y2],
            device=self.device,
            verbose=False,
        )


        if not results or results[0].masks is None:
            return None


        masks = results[0].masks.data


        if masks is None or len(masks) == 0:
            return None


        raw_mask = (
            masks[0]
            .detach()
            .float()
            .cpu()
            .numpy()
            > SAM_MASK_THRESHOLD
        )


        raw_mask = resize_binary_mask(
            raw_mask,
            FRAME_WIDTH,
            FRAME_HEIGHT,
        )


        bbox_center = ((x1 + x2) // 2, (y1 + y2) // 2)


        mask = largest_component_near_point(
            raw_mask,
            bbox_center,
        )


        mask_area = int(mask.sum())


        if mask_area < MIN_MASK_AREA_PX:
            return None


        if mask_area > FRAME_WIDTH * FRAME_HEIGHT * MAX_MASK_AREA_RATIO:
            return None


        return mask


    def _check_stability(
        self,
        measurement: Dict[str, float],
    ) -> Tuple[bool, Dict[str, float]]:
        current_measurement = dict(measurement)


        current_x = float(current_measurement["object_x_base"])
        current_y = float(current_measurement["object_y_base"])


        current_measurement["distance_m"] = math.hypot(current_x, current_y)
        current_measurement["bearing_rad"] = math.atan2(current_y, current_x)


        self.history.append(current_measurement)


        if len(self.history) < STABLE_REQUIRED_FRAMES:
            return False, current_measurement


        x_values = np.asarray(
            [item["object_x_base"] for item in self.history],
            dtype=np.float64,
        )


        y_values = np.asarray(
            [item["object_y_base"] for item in self.history],
            dtype=np.float64,
        )


        u_values = np.asarray(
            [item["bottom_u"] for item in self.history],
            dtype=np.float64,
        )


        v_values = np.asarray(
            [item["bottom_v"] for item in self.history],
            dtype=np.float64,
        )


        stable = bool(
            (float(np.ptp(x_values)) <= MAX_STABLE_X_SPREAD_M)
            and (float(np.ptp(y_values)) <= MAX_STABLE_Y_SPREAD_M)
            and (float(np.ptp(u_values)) <= MAX_STABLE_U_SPREAD_PX)
            and (float(np.ptp(v_values)) <= MAX_STABLE_V_SPREAD_PX)
        )


        median_x = float(np.median(x_values))
        median_y = float(np.median(y_values))


        median_measurement = {
            "object_x_base": median_x,
            "object_y_base": median_y,
            "bottom_u": float(np.median(u_values)),
            "bottom_v": float(np.median(v_values)),
            "distance_m": math.hypot(median_x, median_y),
            "bearing_rad": math.atan2(median_y, median_x),
        }


        return stable, median_measurement


    def _make_invalid_payload(
        self,
        reason: str,
    ) -> Dict[str, Any]:
        return {
            "valid": False,
            "source": "windows_yolo_sam2",
            "class_name": "sugarbox",
            "frame_id": "base_footprint",
            "coordinate_convention": "x_forward_y_left",
            "reason": reason,
            "on_floor": None,
            "floor_check_enabled": False,
            "timestamp_unix": time.time(),
        }


    def _draw_display(
        self,
        frame: np.ndarray,
        bbox: Optional[Tuple[int, int, int, int]],
        mask: Optional[np.ndarray],
        bottom_point: Optional[Tuple[int, int]],
        lines: List[str],
        valid: bool,
    ) -> np.ndarray:
        display = frame.copy()


        # ========================================================
        # 顯示 SAM2 mask
        # 亮紫色半透明填滿，並畫綠色輪廓
        # ========================================================


        if mask is not None:
            mask_bool = mask.astype(bool)


            # 紫色遮罩，OpenCV 使用 BGR
            mask_color = np.zeros_like(display)
            mask_color[:, :] = (255, 0, 255)


            display[mask_bool] = cv2.addWeighted(
                display[mask_bool],
                0.45,
                mask_color[mask_bool],
                0.55,
                0.0,
            )


            # 畫出 SAM2 mask 外框
            contours, _ = cv2.findContours(
                mask.astype(np.uint8),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )


            cv2.drawContours(
                display,
                contours,
                -1,
                (0, 255, 0),
                2,
            )


        # ========================================================
        # 顯示 YOLO bbox
        # 綠色 = valid，橘色 = 尚未穩定或無效
        # ========================================================


        if bbox is not None:
            bbox_color = (
                (0, 255, 0)
                if valid
                else (0, 165, 255)
            )


            cv2.rectangle(
                display,
                (bbox[0], bbox[1]),
                (bbox[2], bbox[3]),
                bbox_color,
                2,
            )


        # ========================================================
        # 顯示 mask 最低點
        # ========================================================


        if bottom_point is not None:
            cv2.circle(
                display,
                bottom_point,
                7,
                (0, 0, 255),
                -1,
            )


            cv2.line(
                display,
                (
                    bottom_point[0] - 14,
                    bottom_point[1],
                ),
                (
                    bottom_point[0] + 14,
                    bottom_point[1],
                ),
                (0, 0, 255),
                2,
            )


            cv2.line(
                display,
                (
                    bottom_point[0],
                    bottom_point[1] - 14,
                ),
                (
                    bottom_point[0],
                    bottom_point[1] + 14,
                ),
                (0, 0, 255),
                2,
            )


        # ========================================================
        # 文字區域：改成黑色半透明背景
        # 不再重複畫黑字和白字
        # ========================================================


        line_height = 30
        text_start_y = 30
        text_panel_height = (
            len(lines) * line_height + 12
        )


        panel = display.copy()


        cv2.rectangle(
            panel,
            (0, 0),
            (FRAME_WIDTH, text_panel_height),
            (0, 0, 0),
            -1,
        )


        display = cv2.addWeighted(
            panel,
            0.55,
            display,
            0.45,
            0.0,
        )


        for index, line in enumerate(lines):
            text_y = text_start_y + index * line_height


            cv2.putText(
                display,
                line,
                (12, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )


        # ========================================================
        # 底部狀態文字
        # ========================================================


        cv2.rectangle(
            display,
            (0, FRAME_HEIGHT - 34),
            (FRAME_WIDTH, FRAME_HEIGHT),
            (0, 0, 0),
            -1,
        )


        cv2.putText(
            display,
            "SAM2: ENABLED | SegFormer floor check: NOT ENABLED",
            (12, FRAME_HEIGHT - 11),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )


        return display


    def run(self) -> None:
        print()
        print("=" * 65)
        print("Windows YOLO + SAM2 publisher started")
        print(f"Camera: {self.camera_description}")
        print(f"Topic: {PUBLISH_TOPIC}")
        print("Press Q to quit")
        print("Press R to reset stability history")
        print("=" * 65)


        try:
            while True:
                frame, frame_stamp = self.frame_reader.read()


                if frame is None:
                    print("[WAIT] Waiting for rear camera...")
                    time.sleep(0.2)
                    continue


                bbox = None
                mask = None
                bottom_point = None
                payload: Dict[str, Any]
                lines: List[str]


                try:
                    yolo_result = self.yolo.predict(
                        source=frame,
                        conf=YOLO_CONFIDENCE,
                        iou=YOLO_IOU,
                        imgsz=YOLO_IMAGE_SIZE,
                        max_det=MAX_DETECTIONS,
                        classes=self.target_class_ids,
                        device=self.device,
                        verbose=False,
                    )[0]


                    chosen_box = self._choose_best_box(yolo_result)


                    if chosen_box is None:
                        self.history.clear()


                        payload = self._make_invalid_payload("yolo_target_not_found")


                        lines = [
                            "YOLO: target not found",
                            "Valid: False",
                        ]


                    else:
                        x1, y1, x2, y2, class_id, confidence = chosen_box


                        bbox = (x1, y1, x2, y2)


                        class_name = get_model_class_name(
                            self.yolo.names,
                            class_id,
                        )


                        mask = self._create_sam_mask(frame, bbox)


                        if mask is None:
                            self.history.clear()


                            payload = self._make_invalid_payload("sam2_mask_invalid")


                            lines = [
                                f"YOLO: {class_name} {confidence:.2f}",
                                "SAM2 mask: invalid",
                                "Valid: False",
                            ]


                        else:
                            bottom_point = find_mask_bottom_point(mask)


                            if bottom_point is None:
                                self.history.clear()


                                payload = self._make_invalid_payload("mask_bottom_missing")


                                lines = [
                                    f"YOLO: {class_name} {confidence:.2f}",
                                    "Mask bottom: missing",
                                    "Valid: False",
                                ]


                            else:
                                projection = self.projector.pixel_to_robot_xy(
                                    bottom_point[0],
                                    bottom_point[1],
                                )


                                if projection is None:
                                    self.history.clear()


                                    payload = self._make_invalid_payload("homography_invalid")


                                    lines = [
                                        f"YOLO: {class_name} {confidence:.2f}",
                                        "Homography: invalid",
                                        "Valid: False",
                                    ]


                                else:
                                    object_x = float(projection["object_x_base"])
                                    object_y = float(projection["object_y_base"])
                                    distance = float(projection["distance_m"])

                                    bottom_consistent = bool(
                                        abs(float(bottom_point[1]) - float(y2))
                                        <= MAX_SAM_BOTTOM_TO_BBOX_BOTTOM_PX)
                                    on_floor = self.projector.inside_calibration_hull(
                                        bottom_point[0], bottom_point[1])


                                    geometry_valid = bool(
                                        object_x > 0.05
                                        and MIN_OBJECT_DISTANCE_M <= distance <= MAX_OBJECT_DISTANCE_M
                                        and abs(object_y) <= MAX_LATERAL_DISTANCE_M
                                        and bottom_consistent
                                        and on_floor
                                    )


                                    measurement = {
                                        "object_x_base": object_x,
                                        "object_y_base": object_y,
                                        "bottom_u": float(bottom_point[0]),
                                        "bottom_v": float(bottom_point[1]),
                                        "distance_m": distance,
                                        "bearing_rad": float(projection["bearing_rad"]),
                                    }


                                    stable, stable_value = self._check_stability(measurement)


                                    valid = bool(geometry_valid and stable)


                                    reason = (
                                        "accepted"
                                        if valid
                                        else (
                                            "geometry_invalid"
                                            if not geometry_valid
                                            else "waiting_for_stability"
                                        )
                                    )


                                    payload = {
                                        "valid": valid,
                                        "source": "windows_yolo_sam2",
                                        "class_id": class_id,
                                        "class_name": class_name,
                                        "confidence": confidence,
                                        "frame_id": "base_footprint",
                                        "coordinate_convention": "x_forward_y_left",
                                        "object_x_base": stable_value["object_x_base"],
                                        "object_y_base": stable_value["object_y_base"],
                                        "distance_m": stable_value["distance_m"],
                                        "bearing_rad": stable_value["bearing_rad"],
                                        "x_camera_ground_m": projection["x_camera_ground_m"],
                                        "rear_cam_to_front_m": self.projector.rear_cam_to_front_m,
                                        "bbox_xyxy": [x1, y1, x2, y2],
                                        "bbox_center_u": 0.5 * (x1 + x2),
                                        "bbox_bottom_v": y2,
                                        "mask_bottom_u": bottom_point[0],
                                        "mask_bottom_v": bottom_point[1],
                                        "mask_area_px": int(mask.sum()),
                                        "stable_frames": len(self.history),
                                        "on_floor": on_floor,
                                        "floor_check_enabled": True,
                                        "bottom_consistent": bottom_consistent,
                                        "observation_age_s": max(
                                            0.0, time.monotonic() - frame_stamp),
                                        "reason": reason,
                                        "timestamp_unix": time.time(),
                                    }


                                    lines = [
                                        f"YOLO: {class_name} {confidence:.2f}",
                                        f"Bottom: ({bottom_point[0]}, {bottom_point[1]})",
                                        f"x={stable_value['object_x_base']:.3f} m y={stable_value['object_y_base']:.3f} m",
                                        f"dist={stable_value['distance_m']:.3f} m stable={len(self.history)}/{STABLE_REQUIRED_FRAMES}",
                                        f"Valid: {valid} ({reason})",
                                    ]


                except Exception as exc:
                    self.history.clear()


                    payload = self._make_invalid_payload("vision_exception")
                    payload["error"] = f"{type(exc).__name__}: {exc}"


                    lines = [
                        "Vision exception:",
                        f"{type(exc).__name__}: {exc}",
                    ]


                    print(f"[VISION ERROR] {type(exc).__name__}: {exc}")


                current_time = time.monotonic()


                if current_time - self.last_publish_time >= PUBLISH_INTERVAL_SEC:
                    self.publisher.publish(payload)
                    self.last_publish_time = current_time


                display = self._draw_display(
                    frame=frame,
                    bbox=bbox,
                    mask=mask,
                    bottom_point=bottom_point,
                    lines=lines,
                    valid=bool(payload.get("valid", False)),
                )


                cv2.imshow(WINDOW_NAME, display)


                key = cv2.waitKey(1) & 0xFF


                if key in (ord("q"), ord("Q")):
                    print("[STOP] Q pressed")
                    break


                if key in (ord("r"), ord("R")):
                    self.history.clear()
                    print("[RESET] Stability history cleared")


        finally:
            invalid_payload = self._make_invalid_payload("publisher_shutdown")


            try:
                self.publisher.publish(invalid_payload)
                time.sleep(0.1)
            except Exception:
                pass


            self.frame_reader.close()
            self.publisher.close()
            cv2.destroyAllWindows()


            print("[INFO] Publisher stopped")




# ============================================================
# 9. 程式入口
# ============================================================


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--jetson-host", default=JETSON_IP)
    parser.add_argument("--ros-port", type=int, default=ROSBRIDGE_PORT)
    parser.add_argument("--camera-backend", choices=("http", "udp"),
                        default="http")
    parser.add_argument("--camera-url", default=REAR_CAMERA_URL)
    parser.add_argument("--udp-port", type=int, default=int(
        os.environ.get("CAMERA_UDP_PORT", "5600")))
    parser.add_argument("--gst-exe", default=os.environ.get(
        "GSTREAMER_LAUNCH_EXE"))
    parser.add_argument("--yolo-model", type=Path, default=YOLO_MODEL_PATH)
    parser.add_argument("--sam-model", type=Path, default=SAM2_MODEL_PATH)
    parser.add_argument("--homography", type=Path,
                        default=HOMOGRAPHY_JSON_PATH)
    args = parser.parse_args()
    application = SugarboxVisionPublisher(
        jetson_host=args.jetson_host,
        ros_port=args.ros_port,
        camera_url=args.camera_url,
        camera_backend=args.camera_backend,
        gst_exe=args.gst_exe,
        udp_port=args.udp_port,
        yolo_model=args.yolo_model,
        sam_model=args.sam_model,
        homography=args.homography,
    )
    application.run()




if __name__ == "__main__":
    try:
        main()


    except KeyboardInterrupt:
        print("\n[STOP] Keyboard interrupt")


    except Exception as exc:
        print()
        print("=" * 65)
        print("[FATAL ERROR]")
        print(f"{type(exc).__name__}: {exc}")
        print("=" * 65)
        raise
