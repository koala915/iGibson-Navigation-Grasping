# G2 odom／TF／AMCL 上機驗收方法（2026-09-28）

這份是 **G2 第 3 步完成後**才執行的驗收程序。目前只建立方法，不表示已驗收。
目標是比較常駐 Python 3.8 服務經 rosbridge 發布 odom／TF 後，是否仍能供 AMCL 穩定使用。

## 0. 前置條件與失敗即停

- 有操作員在車旁，手可立即切斷電源；先架輪測，再落地低速測。
- `grasp-service` 是 `/dev/myserial` 唯一持有者；`x3plus-navigation`、P0 與其他底盤
  driver 必須停止。
- roscore、TG30、map server、AMCL、rosbridge 正常；在 RViz 設定緊的初始姿態。
- 測試前後都確認 `/odom_setmotor` twist 為零。任何 TF authority 改變、odom 中斷、
  AMCL 跳點或 covariance 超限時立即送 TCP 7000 `stop`，不繼續自主巡航。

```bash
systemctl is-active grasp-service
systemctl is-active x3plus-navigation       # 必須 inactive
sudo fuser -v /dev/myserial                 # 只能看到 grasp-service 的程序
rosnode list
rostopic list | grep -E '^/(odom_setmotor|tf|amcl_pose|scan)$'
```

將原始證據保留下來，至少錄 30 秒靜止、一次直行與一次轉向：

```bash
rosbag record -O /tmp/g2_odom_acceptance \
  /odom_setmotor /tf /amcl_pose /scan
```

## 1. `/odom_setmotor` 頻率與延遲

每項至少觀察 200 筆訊息：

```bash
rostopic hz -w 200 /odom_setmotor
rostopic delay -w 200 /odom_setmotor
```

通過條件：

- 平均頻率在 **18–22 Hz**，沒有超過 **0.15 秒**的發布空窗；
- `rostopic delay` 平均不超過 **50 ms**、最大不超過 **100 ms**；
- header stamp 單調遞增，odom 與同一次 `odom→base_footprint` TF 使用相同 stamp；
- 靜止時 twist 回到 0，移動前後 pose 不歸零、不倒退跳號。

延遲數字只在 ROS master 與發布服務使用同一台 Jetson 時成立；若跨機，必須先同步時鐘，
否則 `rostopic delay` 不能當證據。

## 2. `odom → base_footprint` 只有一個發布者

```bash
rosrun tf tf_monitor odom base_footprint
rosrun tf view_frames
rostopic info /tf
```

至少監看 30 秒。通過條件：

- `tf_monitor` 對 `odom → base_footprint` 只列出 **一個 authority**，頻率為 18–22 Hz；
- 不得同時看到 P0、舊 rospy odom publisher 或第二個 mission process 發同一條 transform；
- TF 樹保持單一路徑：`map → odom → base_footprint → base_link → laser`；
- 沒有 `TF_REPEATED_DATA`、時間倒退、future extrapolation 或多父節點警告。

`/tf` 本來就會有多個 publisher，不能只看 `rostopic info /tf` 的 publisher 數量；判定重點是
`tf_monitor` 中 **這一條 transform 的 authority**。

## 3. AMCL 靜止與移動驗收

先設好 2D Pose Estimate，靜止 30 秒，再依序做一段已量測的短直線及低速轉向。角度 odom
係數 `0.501` 尚未重新尺量前，不把轉向結果寫成正式校正通過。
（2026-09-28 註：機器上已是 2026-09-24 重校的 `0.986`，已收進 repo，見 `progress.md`
2026-09-28。第 1、2 節已在架輪狀態通過；第 3 節落地 3/3 通過，數據同見該段。
沒有 RViz 時用 Windows 的 Lichtblick，版面檔與量測工具在 `deploy/ros/`。）

```bash
rostopic hz -w 100 /amcl_pose
rostopic echo /amcl_pose/pose/covariance
rosrun tf tf_echo map base_footprint
```

同時從 bag 計算每筆：

- 位置 variance：`max(covariance[0], covariance[7])`；
- yaw variance：`covariance[35]`；
- 相鄰 AMCL pose 的平移與 yaw 跳量；
- 車停下後 AMCL pose 是否收斂，而非持續漂移。

通過條件沿用 `integration/ros_io.py` 的任務安全閘：

- position variance 全程 `<= 0.25^2 = 0.0625 m^2`；
- yaw variance 全程 `<= radians(20)^2 ≈ 0.1219 rad^2`；
- 移動時 pose 方向與 odom／實際車體一致，不能突然跳過 0.25 m 或 20°；
- 停車後 covariance 不持續增加，`map→odom` 不反覆大幅修正；
- 呼叫 `/request_nomotion_update` 後，靜止狀態仍能取得新鮮 pose，任務端不得報
  `AMCL pose stale`。

最終報告需同時保存尺量位移／角度、odom 結果、AMCL 結果與 bag 路徑；單次看起來正常只記
「跑過一次」，至少三次同條件重複且數值均通過才寫「已驗證」。

## 4. rosbridge 延遲不通過時的備案

若第 1 節失敗，先用同一段資料比較：

1. 常駐服務經 rosbridge 直接發 odom／TF；
2. 常駐服務把 feedback state 送給 Jetson 本機的小型 rospy relay，再由 relay 發
   `/odom_setmotor` 與 TF。

relay 必須是純轉發器，不得另算一份 odom，也不能與 rosbridge 版本同時發布。只有 relay
能恢復 18–22 Hz、延遲門檻與 AMCL 穩定度時才採用，並保留 A/B bag 作為決策證據。
