import cv2
import math
import csv
import json
import os
import time
import numpy as np
from pathlib import Path
from ultralytics import YOLO


# ============================================================
# 1. YOLO 與 ARM camera 設定
# ============================================================

YOLO_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v8i.yolov11(0517)\runs\detect\train\weights\best.pt"

JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "yahboom.local")
TOPIC_ARM = "/arm_cam/image_raw"
URL_ARM = f"http://{JETSON_IP}:8080/stream?topic={TOPIC_ARM}"

YOLO_CONF = 0.3
IMG_SIZE = 640


# ============================================================
# 2. ARM camera 參數
# ============================================================

FRAME_W = 640
FRAME_H = 480

# 先沿用你之前的近似內參
# 如果之後有 ARM camera 自己的內參，再改這裡
FY_ARM = 956.0294
CY_ARM = 221.3649

FX_ARM = 957.6253
CX_ARM = 320.0

# ARM camera 離地高度，單位 m
# 這個很重要，請依照實際量測修改
# 如果 ARM camera 約 35 cm 高，就用 0.35
H_ARM = 0.35

# 你預期 ARM theta 至少 40 度
# 這只是顯示用，不是固定值
EXPECTED_THETA_MIN = 40.0


# ============================================================
# 3. 輸出檔案
# ============================================================

RAW_CSV_PATH = Path("arm_camera_calibration_points.csv")
RESULT_JSON_PATH = Path("arm_camera_calibration_result.json")


# ============================================================
# 4. 工具函式
# ============================================================

def select_largest_box(results):
    """
    校正時建議畫面只放一顆紙球。
    如果有多個框，取最大框。
    """
    if results.boxes is None or len(results.boxes) == 0:
        return None

    best_box = None
    best_area = -1

    for box in results.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        area = max(1.0, x2 - x1) * max(1.0, y2 - y1)

        if area > best_area:
            best_area = area
            best_box = box

    return best_box


def theta_from_point(real_dist_m, y2):
    """
    用一組已知距離 + YOLO box 底部 y2 反推 ARM camera theta。

    alpha = atan((y2 - CY) / FY)
    total_angle = atan(H / real_dist)
    theta = total_angle - alpha
    """
    alpha = math.atan((y2 - CY_ARM) / FY_ARM)
    total_angle = math.atan(H_ARM / real_dist_m)
    theta_rad = total_angle - alpha
    theta_deg = math.degrees(theta_rad)
    return theta_deg


def estimate_distance_from_y2(y2, theta_deg):
    """
    用校正後 theta 估算距離。
    """
    theta = math.radians(theta_deg)
    alpha = math.atan((y2 - CY_ARM) / FY_ARM)

    total_angle = theta + alpha

    if total_angle <= 0:
        return -1.0

    return H_ARM / math.tan(total_angle)


def estimate_offset_x(cx_box, dist_m, theta_deg):
    """
    offset_x > 0：目標在畫面右邊
    offset_x < 0：目標在畫面左邊
    """
    if dist_m <= 0:
        return 0.0

    theta = math.radians(theta_deg)
    optical_depth = dist_m * math.cos(theta) + H_ARM * math.sin(theta)
    return optical_depth * (cx_box - CX_ARM) / FX_ARM


def robust_average(values):
    """
    用 median + MAD 過濾太離群的 theta。
    """
    if len(values) == 0:
        return None, []

    arr = np.array(values, dtype=float)

    if len(arr) < 4:
        return float(np.mean(arr)), list(arr)

    med = np.median(arr)
    abs_dev = np.abs(arr - med)
    mad = np.median(abs_dev)

    if mad < 1e-9:
        kept = arr
    else:
        kept = arr[abs_dev <= 2.5 * mad]

    if len(kept) == 0:
        kept = arr

    return float(np.mean(kept)), list(kept)


def save_points_csv(points):
    with RAW_CSV_PATH.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "index",
                "real_dist_m",
                "x1",
                "y1",
                "x2",
                "y2",
                "cx_box",
                "cy_box",
                "box_h",
                "box_w",
                "theta_deg",
                "pred_dist_m",
                "error_m",
                "abs_error_m"
            ]
        )
        writer.writeheader()
        writer.writerows(points)


def save_result_json(result):
    with RESULT_JSON_PATH.open("w", encoding="utf-8") as f:
        json.dump(result, f, indent=4, ensure_ascii=False)


def draw_status_panel(frame, target_found, info_lines):
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (630, 210), (0, 0, 0), -1)
    frame[:] = cv2.addWeighted(overlay, 0.50, frame, 0.50, 0)

    cv2.putText(
        frame,
        "ARM CAMERA THETA CALIBRATION - NO ROS",
        (25, 38),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (0, 255, 255),
        2
    )

    target_text = "FOUND" if target_found else "LOST"
    color = (0, 255, 0) if target_found else (0, 0, 255)

    cv2.putText(
        frame,
        f"TARGET: {target_text}",
        (25, 68),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        color,
        2
    )

    y = 96
    for line in info_lines[:5]:
        cv2.putText(
            frame,
            line,
            (25, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            1
        )
        y += 24

    cv2.putText(
        frame,
        "C: capture point | F: finish | Q: quit",
        (25, 462),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (255, 255, 255),
        2
    )


# ============================================================
# 5. 主程式
# ============================================================

def main():
    print("[INFO] ARM camera theta calibration")
    print("[INFO] 不使用 ROS，只讀 ARM camera stream")
    print(f"[INFO] Camera URL: {URL_ARM}")
    print(f"[INFO] H_ARM = {H_ARM:.3f} m")
    print(f"[INFO] 你預期 theta 至少約 {EXPECTED_THETA_MIN:.1f} deg")
    print()
    print(f"[INFO] raw csv     = {RAW_CSV_PATH.resolve()}")
    print(f"[INFO] result json = {RESULT_JSON_PATH.resolve()}")
    print()

    print("[INFO] Loading YOLO model...")
    model = YOLO(YOLO_MODEL_PATH)

    print("[INFO] Opening ARM camera stream...")
    cap = cv2.VideoCapture(URL_ARM)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("[ERROR] 無法開啟 ARM camera 串流。")
        print("[CHECK] 確認 Jetson web_video_server 有開：")
        print(URL_ARM)
        return

    points = []
    last_box_info = None
    theta_preview = None

    try:
        while True:
            ret, frame = cap.read()

            if not ret:
                print("[WARN] 無法讀取 ARM frame，重試中...")
                time.sleep(0.05)
                continue

            frame = cv2.resize(frame, (FRAME_W, FRAME_H))

            results = model.predict(
                source=frame,
                conf=YOLO_CONF,
                imgsz=IMG_SIZE,
                verbose=False
            )[0]

            annotated = results.plot()
            box = select_largest_box(results)

            target_found = False
            info_lines = []

            if box is not None:
                target_found = True

                x1, y1, x2, y2 = box.xyxy[0].tolist()

                x1_i = int(x1)
                y1_i = int(y1)
                x2_i = int(x2)
                y2_i = int(y2)

                cx_box = int((x1 + x2) / 2)
                cy_box = int((y1 + y2) / 2)

                box_w = int(max(1, x2 - x1))
                box_h = int(max(1, y2 - y1))

                last_box_info = {
                    "x1": x1_i,
                    "y1": y1_i,
                    "x2": x2_i,
                    "y2": y2_i,
                    "cx_box": cx_box,
                    "cy_box": cy_box,
                    "box_w": box_w,
                    "box_h": box_h
                }

                # 畫底部點與中心線
                cv2.circle(annotated, (cx_box, y2_i), 6, (0, 255, 0), -1)
                cv2.line(annotated, (int(CX_ARM), 0), (int(CX_ARM), FRAME_H), (255, 255, 255), 1)
                cv2.line(annotated, (0, y2_i), (FRAME_W, y2_i), (0, 255, 0), 1)

                info_lines.append(f"y2 bottom pixel: {y2_i}")
                info_lines.append(f"cx: {cx_box} | box_h: {box_h}")

                if theta_preview is not None:
                    dist_preview = estimate_distance_from_y2(y2_i, theta_preview)

                    if dist_preview > 0:
                        offset_x = estimate_offset_x(cx_box, dist_preview, theta_preview)

                        info_lines.append(f"preview theta: {theta_preview:.2f} deg")
                        info_lines.append(f"preview dist: {dist_preview:.2f} m")
                        info_lines.append(f"preview offset_x: {offset_x:.2f} m")

                        cv2.putText(
                            annotated,
                            f"Dist: {dist_preview * 100:.1f} cm",
                            (x1_i, max(25, y1_i - 35)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.65,
                            (0, 255, 255),
                            2
                        )

                        cv2.putText(
                            annotated,
                            f"X: {offset_x * 100:.1f} cm",
                            (x1_i, max(50, y1_i - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.60,
                            (255, 100, 255),
                            2
                        )
                    else:
                        info_lines.append(f"preview theta: {theta_preview:.2f} deg")
                        info_lines.append("preview dist invalid")

                else:
                    info_lines.append("press C to capture calibration point")
                    info_lines.append("input real distance in meter")

            else:
                last_box_info = None
                info_lines.append("No YOLO target.")
                info_lines.append("Put one paper ball in ARM camera view.")

            draw_status_panel(annotated, target_found, info_lines)

            cv2.imshow("ARM Camera Theta Calibration", annotated)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                print("[INFO] Quit without finishing.")
                break

            elif key == ord("c"):
                if last_box_info is None:
                    print("[WARN] 目前沒有偵測到目標，不能記錄校正點。")
                    continue

                print("\n----------------------------------------")
                print("[CAPTURE] 新增 ARM camera 校正點")
                print(f"目前 YOLO box bottom y2 = {last_box_info['y2']}")
                print("請輸入紙球到 ARM camera 地面投影點/車體參考點的真實前方距離，單位：m")
                print("例如 0.35 m 就輸入 0.35")
                print("----------------------------------------")

                try:
                    real_dist_m = float(input("真實距離 m = ").strip())
                except Exception:
                    print("[WARN] 輸入錯誤，略過。")
                    continue

                if real_dist_m <= 0:
                    print("[WARN] 距離必須 > 0，略過。")
                    continue

                theta_deg = theta_from_point(real_dist_m, last_box_info["y2"])

                row = {
                    "index": len(points) + 1,
                    "real_dist_m": real_dist_m,
                    "x1": last_box_info["x1"],
                    "y1": last_box_info["y1"],
                    "x2": last_box_info["x2"],
                    "y2": last_box_info["y2"],
                    "cx_box": last_box_info["cx_box"],
                    "cy_box": last_box_info["cy_box"],
                    "box_h": last_box_info["box_h"],
                    "box_w": last_box_info["box_w"],
                    "theta_deg": theta_deg,
                    "pred_dist_m": "",
                    "error_m": "",
                    "abs_error_m": ""
                }

                points.append(row)

                theta_values = [p["theta_deg"] for p in points]
                theta_preview, kept = robust_average(theta_values)

                print(f"[OK] 已新增第 {len(points)} 點")
                print(f"     real_dist = {real_dist_m:.3f} m")
                print(f"     y2 = {last_box_info['y2']}")
                print(f"     theta from this point = {theta_deg:.3f} deg")
                print(f"     current theta avg = {theta_preview:.3f} deg")

                if theta_deg < EXPECTED_THETA_MIN:
                    print(f"[NOTE] 這筆 theta < {EXPECTED_THETA_MIN:.1f} deg，請確認 H_ARM 或距離量測是否正確。")
                print()

            elif key == ord("f"):
                if len(points) < 2:
                    print("[WARN] 至少記錄 2 個點，建議 4~6 個點。")
                    continue

                theta_values = [p["theta_deg"] for p in points]
                theta_avg, kept = robust_average(theta_values)

                errors = []

                for p in points:
                    pred_dist = estimate_distance_from_y2(p["y2"], theta_avg)
                    err = pred_dist - p["real_dist_m"]
                    errors.append(err)

                    p["pred_dist_m"] = pred_dist
                    p["error_m"] = err
                    p["abs_error_m"] = abs(err)

                mean_abs_error = float(np.mean([abs(e) for e in errors]))
                max_abs_error = float(np.max([abs(e) for e in errors]))

                result = {
                    "theta_arm_deg": theta_avg,
                    "camera_height_m": H_ARM,
                    "fx": FX_ARM,
                    "fy": FY_ARM,
                    "cx": CX_ARM,
                    "cy": CY_ARM,
                    "points_count": len(points),
                    "theta_values_deg": theta_values,
                    "theta_values_used_by_robust_avg": kept,
                    "mean_abs_error_m": mean_abs_error,
                    "max_abs_error_m": max_abs_error,
                    "points": points
                }

                save_points_csv(points)
                save_result_json(result)

                print("\n===================================================")
                print("ARM camera theta 校正完成")
                print("===================================================")
                print(f"THETA_ARM = {theta_avg:.3f} deg")
                print(f"mean abs error = {mean_abs_error * 100:.1f} cm")
                print(f"max abs error  = {max_abs_error * 100:.1f} cm")
                print()
                print("之後主程式請填：")
                print(f"THETA_ARM = {theta_avg:.3f}")
                print(f"H_ARM = {H_ARM:.3f}")
                print()
                print(f"raw csv     = {RAW_CSV_PATH.resolve()}")
                print(f"result json = {RESULT_JSON_PATH.resolve()}")
                print("===================================================")
                print()

                theta_preview = theta_avg

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected.")

    finally:
        cap.release()
        cv2.destroyAllWindows()

        if len(points) > 0:
            try:
                save_points_csv(points)
                print(f"[INFO] 已保存目前校正點到：{RAW_CSV_PATH.resolve()}")
            except Exception as e:
                print(f"[WARN] save csv failed: {e}")

        print("[INFO] 程式結束。")


if __name__ == "__main__":
    main()
