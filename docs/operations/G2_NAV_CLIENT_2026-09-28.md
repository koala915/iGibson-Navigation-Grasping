# G2 導航專用控制端（2026-09-28）

`integration/g2_nav_client.py` 是導航端在 G2 常駐服務上做短直線驗收的第一個
TCP 7000 用戶端；**不是完整自主巡航，也不夾取、不轉彎**。服務仍獨占
`/dev/myserial`，此程式只讀 rosbridge 的 `/scan`、`/odom_setmotor`，向
`127.0.0.1:7000` 送 `velocity`／`stop`。舊的 `mission_pipeline.py --real` 仍被
homography 安全閘禁止，模式 C `nav_rl_grasp_pipeline.py` 仍直接持有序列埠，
不可與 `grasp-service` 同時運行。

## 已實作的安全閘

- 預設無移動：`--selftest` 只跑純邏輯；`--probe` 只讀服務狀態與 ROS。
  `--real` 要另外帶現場淨空及可斷電確認、Route B 光達方向證據。
- 實車僅允許在 Jetson 本機、手臂為已確認的 travel 姿態、服務靜止、
  odom 有效、控制板有回報，且 `fuser` 的唯一序列埠持有 PID 等於
  `grasp-service` 的 MainPID 時啟動。行駛中**不輪詢 `graspctl status`**。
- 只發直線 `vx=0.15 m/s`、`wz=0`（已尺量的馬達輸出 30）、20 Hz。
  目標距離限 10–25 cm；odom 停車預留 6 cm，硬上限 28 cm／2.5 s。
  這些數字只適用當前地面及巡航姿態，其他速度／轉彎須另外校正。
- LiDAR 用 `nav_rl.front_min_brake` 的連續回波規則，車體中心 x <0.42 m
  停車，近處單點仍會觸發緊急停車。`/scan` 到達時間 >0.35 s、訊息時間戳
  >0.40 s、odom 到達時間 >0.20 s、frame 不符、掃描空白、odom 跳躍／
  不前進或直線偏離，都會停車。失去 TCP 連線時服務本身也會停輪。
- `--real` 必須有既有四方向板子驗證檔，且前方 LiDAR 覆蓋率 ≥90%。
  不能把單次紙板測試當成左右方向都正確的證據。

## 唯讀操作與目前結果

```bash
cd /home/jetson/Documents/deploy_jetson2
/home/jetson/grasp_venv/bin/python3 integration/g2_nav_client.py --selftest
/home/jetson/grasp_venv/bin/python3 integration/g2_nav_client.py --probe
```

本次已在 Jetson 跑過上述兩項：自測通過，probe 讀到 `scan_frame=laser`、
車體中心前方紙板 x 約 0.322 m、odom 速度零，輸出
`brake=lidar_brake; no TCP motor command sent`。**尚未執行此新程式的
`--real`**，不能引用先前獨立測試程式的 3/3 作為它的實車驗收。

probe 曾誤報前方 180° 只有 50% 覆蓋：TG30 2020 束的 float32
`angle_increment` 算出 359.9999964°，略低於舊程式的 360°−1e-6
判定值。`nav_rl.describe_scan` 已改成容忍不大於一個 sample 的缺口，
Jetson 自測與再 probe 後不再報此假警告；**未放寬真實部分視野的檢查**。

待現場清空紙板、重新標記起點與確認路徑後，才可提出一次 `--real`
短直線測試。該次須記錄 odom 與尺量、停車原因、停後速度與最短障礙距離；
安全閘觸發屬於正確停車，但不算「到達距離目標」。完整 waypoint／AMCL
巡航及導航↔夾取狀態切換仍是下一階段，不能由本程式宣稱已完成。
