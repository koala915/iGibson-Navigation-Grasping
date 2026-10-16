# 固定 LiDAR 自回波濾波

使用者確認周邊回波為機構／線材並授權濾除後，將本次靜止採樣的26個原始角度格固定保存為
`deploy/ros/lidar_self_returns_2026-10-16.json`。不會在開機時自動採樣或學習場地。

## 資料流程

- TG30 driver → `/scan_raw`：完整原始掃描，保留排查來源。
- `g2_self_return_filter` → `/scan`：只將匹配固定角格及量距±2.5cm的點標成NaN。
- 現有導航、建圖濾波、定位等訂閱 `/scan` 的程式直接使用新結果。
- 時戳、frame、角度／量距metadata、陣列長度保留；ROS publisher會產生自己的header.seq。
- 不做後180°整片清除，亦未把0–75cm整段當成空白；仍保留不同角格或不匹配量距的點。

這是從使用者確認的本車回波建立的固定profile，不是通用物體分類器。
與profile重疊的外物可能同時被遮蔽；線材、雷射安裝或機構位置變更時需以原始掃描重新驗證。
啟動時profile缺失或不合法不發布濾波scan；錯誤frame／超過0.35秒的資料不轉發。
RAM可用低於768MB時濾波節點停止，原始driver與濾波節點在同一required launch內，不自動繞過濾波。
本次僅固定點濾除接入全域scan；3幀側向群聚確認仍僅屬監督式校正程式，未全域套用。

## 部署與開機

沿用已啟用的 `x3plus-g2-lidar.service`，不新增Serial使用者、不更動grasp-service。
`deploy/ros/package.xml` 提供 `x3plus_g2_lidar` ROS package；
`start-g2-ros.sh` 增加專案deploy/ros到ROS_PACKAGE_PATH。
遠端現有launch與本機HEAD原稿比對相同後才修改，保留組員其他設定。
備份：`/home/jetson/lidar_filter_backup_20261016/`。

重新部署時需一併複製filter腳本、profile、package.xml、CMakeLists.txt及launch，並執行：

```bash
chmod +x /home/jetson/Documents/deploy_jetson2/deploy/ros/g2_self_return_filter.py
sudo install -m 755 /home/jetson/Documents/deploy_jetson2/deploy/ros/start-g2-ros.sh /usr/local/libexec/x3plus/start-g2-ros.sh
sudo systemctl restart x3plus-g2-lidar.service
```

本次確認停止且maintenance上鎖後已重啟LiDAR服務；systemd為enabled/active。
尚未實際重開機，勿將本次服務重啟等同冷開機驗收。

## 驗證

- 本次新增與相關測試共29通過。
- 現場 `/scan_raw` 唯一發布者為driver，`/scan` 唯一發布者為filter。
- 12組配對的時戳、frame、掃描metadata一致，陣列長度保留。
- 每幀約55–65個有限回波標成NaN；現有 `laser_scan_to_points` 成功解析。
- RAM可用約1946MB；odom valid、輪命令全零、maintenance=true。
- 原始記錄：`docs/calibration/odom_new_floor_2026-10-16/persistent_filter_live.json`、
  `persistent_filter_metadata_health.json`。
- 既有controller138、servo37、floor641、UI50（skip1）、mission selftest通過。
  controller與floor使用僅測試程序的相對URDF路徑轉接，Windows中文絕對路徑問題未修復。

## 旋轉量測回覆

使用者實際左轉約14°，本輪odom停穩14.729°，相差約+0.73°。
實際角度為近似值，未更動角速度倍率；停止後超轉仍需另行調整，不能宣稱轉角控制驗收完成。
往後校正若需重新建立profile必須使用 `/scan_raw`，不要從已濾波的 `/scan` 再學一份遮罩。
