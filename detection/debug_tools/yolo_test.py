import cv2
import os
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
YOLO_CONF = 0.10   # 先調低，確認到底有沒有偵測到


# ============================================================
# 主程式
# ============================================================

def main():
    print("[INFO] Loading YOLO model...")
    model = YOLO(YOLO_MODEL_PATH)

    print("[INFO] Model classes:")
    print(model.names)

    print("[INFO] Opening camera...")
    cap = cv2.VideoCapture(URL_REAR)

    if not cap.isOpened():
        print("[ERROR] Cannot open camera stream.")
        return

    print("[INFO] Start YOLO test.")
    print("[INFO] Press q to quit.")
    print("[INFO] Press + to increase conf.")
    print("[INFO] Press - to decrease conf.")

    conf_thres = YOLO_CONF

    while True:
        ok, frame = cap.read()

        if not ok or frame is None:
            print("[WARN] Cannot read frame.")
            continue

        frame = cv2.resize(frame, (FRAME_W, FRAME_H))

        results = model.predict(
            frame,
            imgsz=IMG_SIZE,
            conf=conf_thres,
            verbose=False
        )

        detect_count = 0

        if len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes

            for box in boxes:
                detect_count += 1

                xyxy = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())

                x1, y1, x2, y2 = xyxy
                x1 = int(x1)
                y1 = int(y1)
                x2 = int(x2)
                y2 = int(y2)

                class_name = model.names.get(cls_id, str(cls_id))

                # 畫框
                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

                # 中心點
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                cv2.circle(
                    frame,
                    (cx, cy),
                    5,
                    (0, 0, 255),
                    -1
                )

                label = f"{class_name} {conf:.2f}"

                cv2.putText(
                    frame,
                    label,
                    (x1, max(25, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

                # 終端機也印出來
                print(
                    f"[DETECT] class={class_name}, conf={conf:.2f}, "
                    f"box=({x1},{y1},{x2},{y2}), center=({cx},{cy})"
                )

        # 畫面資訊
        cv2.putText(
            frame,
            f"YOLO CONF = {conf_thres:.2f}",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"Detections = {detect_count}",
            (15, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2
        )

        cv2.imshow("YOLO Only Test", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        elif key == ord("+") or key == ord("="):
            conf_thres += 0.05
            if conf_thres > 0.95:
                conf_thres = 0.95
            print(f"[INFO] conf_thres = {conf_thres:.2f}")

        elif key == ord("-") or key == ord("_"):
            conf_thres -= 0.05
            if conf_thres < 0.01:
                conf_thres = 0.01
            print(f"[INFO] conf_thres = {conf_thres:.2f}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
