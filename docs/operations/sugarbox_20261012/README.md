# Sugarbox 接近＋E1 夾取：2026-10-12 現場原始資料

分析與結論在 `docs/planning/DUAL_CAMERA_APPROACH_GRASP_DISCUSSION_2026-10-05.md` §10.6–10.12，
這裡只放原始記錄，方便之後重播或對照。場地：四面圍牆、寬約 1 m、長約 2.2 m 的白色地板區。

## runs/：實車記錄

`runN_real.log` 是總指揮 `integration/sugarbox_approach_grasp.py` 的完整輸出。
`runN_obs.jsonl` 是 `--obs-log` 記錄的 PPO 每一步輸入（55 維觀測）與輸出，可以離線重播。

| 次 | 設定 | 結果 |
|----|------|------|
| 1 | enoch 模式、首次辨識即鎖定、舊後相機校正 | 停在中心距 0.432 m、+9.1°，盒子偏左約 11 cm（照片），E1 未夾 |
| 2 | 關掉首次鎖定、新校正、補側邊盲區、YOLO 1280 | 0.436 m、+18.3°，偏左約 14–20 cm，PPO 幾乎不轉 |
| 3 | 同上＋觀測記錄 | 0.431 m、+8.6°；重播證實兩側圍牆讓 PPO 當成走道而直走 |
| 4 | 射線 ×1.15、走道煞車、到達後直走 10 cm | PPO 左右交替轉向、平均約 0，偏左約 8 cm；直走後 0.336 m 太近 |
| 5 | PPO 開到 0.80 m 交棒，慢速轉向對準（增益 1.5） | 停在 x 0.309，但轉向太弱（馬達 12/20，幾乎沒轉），偏左 4–6 cm |
| 6 | 1.00 m 交棒、轉向 0.30–0.60 rad/s | 記憶 (0.307, −0.011)；E1 量到 x 0.259–0.275、y +0.0175，超出左界 1 mm |
| 7 | 對準目標補償兩相機中線差 2.8 cm、停車 0.30 m | 盒子在後相機校正範圍外，全程未鎖定、車子沒動；90 s 逾時。之後電腦當機 |

第 6 次之後的修正（對準目標 −0.044 m、停車 0.30 m）還沒有成功的實車驗證。

## calibration/：後相機 1.5 m 重新校正

- `rear_calib_samples.jsonl`：18 個擺放點（p01 被夾爪擋住失敗；v17/v18 是保留驗證點）。
  `x_wheel` 從兩前輪最前緣連線量起，`y` 左正；`u, v` 是執行時同一套 YOLO(1280)+SAM2
  遮罩最低點，8 張取中位數。結果已寫入 `detection/rear_ground_homography.json`
  （檔案座標 = `x_wheel + 0.215`）。
- `rear_calib_capture.py`：當天用的擷取工具。路徑寫死在這台 Windows 開發機，
  要從 UDP 5600 收後相機串流（`deploy/ros/start_rear_approach_stream.sh`）。

## lidar/：靜止掃描（`sensor_msgs/LaserScan` 轉 JSON，每檔數圈）

- `scan_board_left_wall_right.json`：左側貼一塊板子、右側約 35 cm 有牆，用來確認團隊的
  旋轉 180° 左右正確（鏡射會把牆放到左邊）。
- `scan_arena.json`：拿掉板子後的圍牆場地，也用來找車身回波範圍（±15 cm 方框）。
