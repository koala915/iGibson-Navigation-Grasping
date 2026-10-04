"""手臂相機即時辨識檢視器（除錯用）。

幾何常數一律來自 integration/arm_cam_geometry.py，這裡不再自己留一份。
之前 bridge / pipeline / 這支各留一份 H 與 THETA，正是 2026-08-01 姿勢防護
只補到 pipeline、沒補到 bridge 的原因。

⚠ 預設用的是 **v17 nav home** 的外參（36.4°、0.332m），因為這支是拿來在 nav
home 眼睛看辨識框用的。v21 的 C3 起始姿勢相機幾乎垂直朝下（約 89.7°、
0.215m），在那個姿勢下這裡印出的距離沒有意義——要看 C3 請改 ARM_CAM_POSE。
"""
import os
import sys
import cv2
import math
from ultralytics import YOLO

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "integration"))
import arm_cam_geometry as acg

# ========= 1. 檔案與硬體參數 =========
# best.pt 位於同層的 models/ 子資料夾（detection/models/best.pt）
MODEL_PATH = os.path.join(os.path.dirname(__file__), "models", "best.pt")
JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "yahboom.local")
VIDEO_PATH = f"http://{JETSON_IP}:8080/stream?topic=/arm_cam/image_raw"
CONF_THRESHOLD = 0.3 
IMG_SIZE = 640

# ========= 2. 手臂相機專用物理參數 =========
# 全部取自 arm_cam_geometry 的姿勢登錄表，不在此複製數值。
ARM_CAM_POSE = acg.V17_NAV_HOME   # 這支檢視器預設在 nav home 使用
CX_CAM = acg.CX
CY = acg.CY
FX = acg.FX
FY = acg.FY
DIST = acg.DIST  # k1 k2 p1 p2 k3

H = ARM_CAM_POSE.h_m            # 相機離地高度
FIXED_THETA = ARM_CAM_POSE.theta_deg  # 光軸俯角（朝 base +X 量）

# ========= 3. 座標系轉換參數 =========
ARM_TO_REAR_OFFSET = 0.22  # 手臂鏡頭到後置鏡頭的物理距離 (22公分)
# =======================================

def undistort_pixel(u, v, iters=8):
    """把單一像素座標去畸變（plumb-bob，與 cv2.undistortPoints(..., P=K) 同法）。
    手臂相機在 bbox 底邊中點的畸變位移約 9px，距離模型前必須先過這步。"""
    return acg.undistort_pixel(u, v, iters=iters)


def estimate_distance(y_max):
    """物體到手臂鏡頭的地面距離（y_max 需為去畸變後的像素列）。

    回傳值是**有號**的：相機正下方為 0，更靠近機身側為負。相機接近垂直時
    這是正常且必要的（C3 姿勢下畫面約 64% 的列都是負值），所以呼叫端不可以
    用 `if d > 0` 過濾。射線根本打不到地面時才回傳 None。
    """
    try:
        return acg.ground_hit(CX_CAM, y_max, ARM_CAM_POSE).ground_dist_m
    except acg.GroundGeometryError:
        return None


def estimate_lateral_offset(cx_pixel, ground_dist_m):
    """右正左負；水平像素換算必須使用相機 optical depth。"""
    if ground_dist_m <= 0:
        return 0.0
    theta = math.radians(FIXED_THETA)
    optical_depth = ground_dist_m * math.cos(theta) + H * math.sin(theta)
    return optical_depth * (cx_pixel - CX_CAM) / FX


def main():
    print("[INFO] 載入 YOLO 模型中...")
    model = YOLO(MODEL_PATH)

    print(f"[INFO] 正在開啟手臂相機: {VIDEO_PATH}")
    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print("[ERROR] 無法連線到相機！")
        return

    print(">>> [系統就緒] 進入近戰視覺測試模式 (統一座標系) <<<")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model.predict(source=frame, conf=CONF_THRESHOLD, imgsz=IMG_SIZE, verbose=False)
        result = results[0]
        annotated_frame = result.plot()

        if result.boxes is not None and len(result.boxes) > 0:
            box = result.boxes[0] 
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx_box = int((x1 + x2) / 2)

            # 1. 一次算完：去畸變 → 地面交點（有號距離 + 光軸深度）
            try:
                hit = acg.ground_hit_from_raw(cx_box, y2, ARM_CAM_POSE)
            except acg.GroundGeometryError as e:
                hit = None
                print(f"[WARN] 幾何無解，略過此框：{e}")

            if hit is not None:
                arm_dist = hit.ground_dist_m
                # 2. 左右偏移的透視基準是**光軸深度**，不是地面距離。
                #    用地面距離會低估（nav home 低 20%），相機接近垂直時更會塌成 0。
                target_offset_x = hit.lateral_m

                # 3. 🚀 【座標統一】把距離轉換成「距離後置主相機」的數據
                unified_dist = arm_dist + ARM_TO_REAR_OFFSET
                
                # 視覺標示
                cv2.circle(annotated_frame, (cx_box, int(y2)), 8, (0, 0, 255), -1)
                side_text = "Right" if target_offset_x > 0 else "Left"
                
                # 畫在畫面上的文字 (顯示統一後的總距離)
                cv2.putText(annotated_frame, f"Total Dist: {unified_dist:.2f}m", (int(x1), int(y2) + 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(annotated_frame, f"Offset: {abs(target_offset_x):.2f}m ({side_text})", (int(x1), int(y2) + 45), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 255), 2)
                
                # 在終端機印出詳細數據供你檢查
                print(f"手臂距離: {arm_dist:.2f}m | 統一總距離: {unified_dist:.2f}m | 偏移: {target_offset_x:.2f}m | Y像素: {int(y2)}")

        # 畫面顯示與準星
        h, w, _ = annotated_frame.shape  
        center_x = int(CX_CAM) 
        cv2.line(annotated_frame, (center_x, 0), (center_x, h), (0, 255, 255), 2)
        cv2.putText(annotated_frame, "Center (X=0)", (center_x + 10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("Arm Camera - Unified Coordinate Test", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
