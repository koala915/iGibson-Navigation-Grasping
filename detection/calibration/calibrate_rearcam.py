import cv2
import time
import math
import csv
import os
from datetime import datetime
import numpy as np
from ultralytics import YOLO


# ============================================================
# 基本設定
# ============================================================

YOLO_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v8i.yolov11(0517)\runs\detect\train\weights\best.pt"

JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "yahboom.local")
URL_REAR = f"http://{JETSON_IP}:8080/stream?topic=/back_cam/image_raw"

FRAME_W = 640
FRAME_H = 480

IMG_SIZE = 640
YOLO_CONF = 0.10

# 後鏡頭高度，單位 m
H_REAR = 0.50

# 後鏡頭到車頭距離，單位 m
# 如果你輸入的是「球到車頭距離」，程式會自動 +0.20 變成球到後鏡頭距離
REAR_CAM_TO_FRONT_M = 0.20

# 後鏡頭內參
FY_REAR = 956.0294
CY_REAR = 221.3649
FX_REAR = 957.6253
CX_REAR = 320.0

SAVE_DIR = r"C:\Users\user\Downloads\rear_cam_calibration"


# ============================================================
# theta 計算
# ============================================================

def compute_theta_from_y_and_distance(y_pixel, actual_front_dist_m):
    """
    使用已知距離與 YOLO 框底部 y_pixel 反推 rear cam 俯角 theta。

    actual_front_dist_m:
        紙球到車頭的實際距離，單位 m

    程式內部會轉成：
        actual_rear_dist_m = actual_front_dist_m + REAR_CAM_TO_FRONT_M

    原本距離公式：
        dist = H / tan(theta + alpha)

    所以反推：
        theta = atan(H / dist) - alpha

    alpha = atan((y - cy) / fy)
    """

    actual_rear_dist_m = actual_front_dist_m + REAR_CAM_TO_FRONT_M

    if actual_rear_dist_m <= 0:
        return None

    alpha = math.atan((y_pixel - CY_REAR) / FY_REAR)
    total_angle = math.atan(H_REAR / actual_rear_dist_m)

    theta_rad = total_angle - alpha
    theta_deg = math.degrees(theta_rad)

    return theta_deg, actual_rear_dist_m, math.degrees(alpha), math.degrees(total_angle)


def estimate_distance_with_theta(y_pixel, theta_deg):
    """
    用目前 theta 估算距離，方便檢查校正結果。
    回傳：
        dist_rear_m, dist_front_m
    """

    theta = math.radians(theta_deg)
    alpha = math.atan((y_pixel - CY_REAR) / FY_REAR)
    total_angle = theta + alpha

    if total_angle <= 0.01:
        return None, None

    dist_rear_m = H_REAR / math.tan(total_angle)
    dist_front_m = max(0.0, dist_rear_m - REAR_CAM_TO_FRONT_M)

    return dist_rear_m, dist_front_m


# ============================================================
# YOLO 偵測
# ============================================================

def detect_best_ball(model, frame):
    """
    回傳最高信心值的偵測框。
    """

    results = model.predict(
        frame,
        imgsz=IMG_SIZE,
        conf=YOLO_CONF,
        verbose=False
    )

    if len(results) == 0 or results[0].boxes is None:
        return None

    boxes = results[0].boxes

    best = None
    best_conf = -1.0

    for box in boxes:
        xyxy = box.xyxy[0].cpu().numpy()
        conf = float(box.conf[0].cpu().numpy())
        cls_id = int(box.cls[0].cpu().numpy())

        if conf > best_conf:
            x1, y1, x2, y2 = xyxy
            best = {
                "bbox": (int(x1), int(y1), int(x2), int(y2)),
                "conf": conf,
                "cls_id": cls_id
            }
            best_conf = conf

    return best


# ============================================================
# CSV 儲存
# ============================================================

def save_records_to_csv(records):
    now = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(SAVE_DIR, f"rear_cam_theta_calibration_{now}.csv")

    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)

        writer.writerow([
            "index",
            "actual_front_dist_cm",
            "actual_rear_dist_cm",
            "y_pixel",
            "alpha_deg",
            "total_angle_deg",
            "theta_deg",
            "yolo_conf"
        ])

        for i, r in enumerate(records, start=1):
            writer.writerow([
                i,
                r["actual_front_dist_m"] * 100,
                r["actual_rear_dist_m"] * 100,
                r["y_pixel"],
                r["alpha_deg"],
                r["total_angle_deg"],
                r["theta_deg"],
                r["yolo_conf"]
            ])

    print(f"[SAVE] CSV saved: {csv_path}")


# ============================================================
# 畫面顯示
# ============================================================

def draw_info(frame, best_det, records):
    display = frame.copy()

    cv2.line(display, (0, int(CY_REAR)), (FRAME_W, int(CY_REAR)), (100, 100, 100), 1)
    cv2.putText(
        display,
        f"CY_REAR={CY_REAR:.1f}",
        (10, int(CY_REAR) - 5),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (150, 150, 150),
        1
    )

    if best_det is not None:
        x1, y1, x2, y2 = best_det["bbox"]
        conf = best_det["conf"]

        cx = int((x1 + x2) / 2)
        y_bottom = y2

        cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(display, (cx, y_bottom), 6, (0, 0, 255), -1)

        cv2.putText(
            display,
            f"conf={conf:.2f} y2={y_bottom}",
            (x1, max(25, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

    if len(records) > 0:
        theta_values = [r["theta_deg"] for r in records]
        mean_theta = float(np.mean(theta_values))
        median_theta = float(np.median(theta_values))
        std_theta = float(np.std(theta_values))

        cv2.putText(
            display,
            f"samples={len(records)} mean={mean_theta:.2f} deg median={median_theta:.2f} deg std={std_theta:.2f}",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2
        )

        cv2.putText(
            display,
            f"Recommended THETA_REAR = {median_theta:.2f}",
            (15, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 0, 255),
            2
        )
    else:
        cv2.putText(
            display,
            "No calibration sample yet.",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

    cv2.putText(
        display,
        "Press c: add sample | s: save CSV | r: reset | q: quit",
        (15, FRAME_H - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2
    )

    return display


# ============================================================
# 主程式
# ============================================================

def main():
    os.makedirs(SAVE_DIR, exist_ok=True)
    print("[INFO] Loading YOLO model...")
    model = YOLO(YOLO_MODEL_PATH)

    print("[INFO] Model classes:")
    print(model.names)

    print("[INFO] Opening rear camera stream...")
    cap = cv2.VideoCapture(URL_REAR)

    if not cap.isOpened():
        print("[ERROR] Cannot open rear camera stream.")
        print("[CHECK] Jetson IP / camera stream / network")
        return

    print("\n================ Rear Cam Theta Calibration ================")
    print("使用方式：")
    print("1. 把紙球放在車頭前方已知距離，例如 60cm、80cm、100cm")
    print("2. 確認 YOLO 有框到紙球")
    print("3. 按 c，輸入紙球到車頭的實際距離 cm")
    print("4. 多做幾個距離，最後看 Recommended THETA_REAR")
    print("5. 按 s 儲存 CSV")
    print("============================================================\n")

    records = []
    last_best_det = None

    while True:
        ok, frame = cap.read()

        if not ok or frame is None:
            print("[WARN] Cannot read frame.")
            time.sleep(0.05)
            continue

        frame = cv2.resize(frame, (FRAME_W, FRAME_H))

        best_det = detect_best_ball(model, frame)
        last_best_det = best_det

        display = draw_info(frame, best_det, records)

        cv2.imshow("Rear Cam Theta Calibration", display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            print("[INFO] Quit.")
            break

        elif key == ord("r"):
            records = []
            print("[INFO] Records reset.")

        elif key == ord("s"):
            if len(records) > 0:
                save_records_to_csv(records)
            else:
                print("[WARN] No records to save.")

        elif key == ord("c"):
            if last_best_det is None:
                print("[WARN] No YOLO detection. Cannot calibrate.")
                continue

            x1, y1, x2, y2 = last_best_det["bbox"]
            y_pixel = y2
            yolo_conf = last_best_det["conf"]

            print("\n[INPUT] 請輸入紙球到車頭的實際距離，單位 cm")
            print("例如：60、80、100")
            user_input = input("actual_front_dist_cm = ")

            try:
                actual_front_dist_cm = float(user_input)
            except ValueError:
                print("[ERROR] 輸入錯誤，請輸入數字。")
                continue

            actual_front_dist_m = actual_front_dist_cm / 100.0

            result = compute_theta_from_y_and_distance(
                y_pixel,
                actual_front_dist_m
            )

            if result is None:
                print("[ERROR] Cannot compute theta.")
                continue

            theta_deg, actual_rear_dist_m, alpha_deg, total_angle_deg = result

            record = {
                "actual_front_dist_m": actual_front_dist_m,
                "actual_rear_dist_m": actual_rear_dist_m,
                "y_pixel": y_pixel,
                "alpha_deg": alpha_deg,
                "total_angle_deg": total_angle_deg,
                "theta_deg": theta_deg,
                "yolo_conf": yolo_conf
            }

            records.append(record)

            theta_values = [r["theta_deg"] for r in records]
            mean_theta = float(np.mean(theta_values))
            median_theta = float(np.median(theta_values))
            std_theta = float(np.std(theta_values))

            print("\n[CALIBRATION SAMPLE]")
            print(f"actual_front_dist = {actual_front_dist_cm:.1f} cm")
            print(f"actual_rear_dist  = {actual_rear_dist_m * 100:.1f} cm")
            print(f"y_pixel / y2      = {y_pixel}")
            print(f"YOLO conf         = {yolo_conf:.2f}")
            print(f"alpha             = {alpha_deg:.2f} deg")
            print(f"total_angle       = {total_angle_deg:.2f} deg")
            print(f"theta             = {theta_deg:.2f} deg")

            print("\n[CURRENT RESULT]")
            print(f"samples           = {len(records)}")
            print(f"mean theta        = {mean_theta:.2f} deg")
            print(f"median theta      = {median_theta:.2f} deg")
            print(f"std theta         = {std_theta:.2f} deg")
            print(f"Recommended       = THETA_REAR = {median_theta:.2f}")
            print("------------------------------------------------------------")

    cap.release()
    cv2.destroyAllWindows()

    if len(records) > 0:
        theta_values = [r["theta_deg"] for r in records]
        mean_theta = float(np.mean(theta_values))
        median_theta = float(np.median(theta_values))
        std_theta = float(np.std(theta_values))

        print("\n================ Final Result ================")
        print(f"samples      = {len(records)}")
        print(f"mean theta   = {mean_theta:.2f} deg")
        print(f"median theta = {median_theta:.2f} deg")
        print(f"std theta    = {std_theta:.2f} deg")
        print("")
        print(f"請把主程式改成：")
        print(f"THETA_REAR = {median_theta:.2f}")
        print("==============================================")

        save_records_to_csv(records)


if __name__ == "__main__":
    main()
