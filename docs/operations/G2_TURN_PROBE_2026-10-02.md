# G2 開闊區限角轉向與側向 LiDAR 初測（2026-10-02）

`integration/g2_turn_probe.py` 是 supervised 小角度轉向測試，不是自主側向避障。
它只向常駐 `grasp-service` 的 TCP 7000 發速度命令，不開啟第二個
`/dev/myserial` owner。`--selftest` 為純邏輯、`--probe` 僅訂閱 ROS；
`--real` 須在 Jetson 本機、操作員確認四周實體淨空並可立即斷電時使用。

實車左、右轉各有一次限角結果：目標在 odom 轉過約 6° 時送停，
最終左轉 +8.99°（平移 7 mm）、右轉 −7.28°（平移 3 mm）；
兩次停止後 `graspctl status` 均為四輪命令 0、odom 角速度 0，
操作員回報無碰撞、無電線拉扯。先前 0.30 秒的馬達 20、25
左轉脈衝分別得到約 +1.1°、+3.2°，也都停穩。

程式在每次實車測試前確認手臂 `travel`、底盤靜止、odom 有效、
`grasp-service` 是序列埠唯一 owner、四方向 LiDAR orientation evidence
與新鮮 `/scan`。行駛中以 20 Hz 檢查掃描／odom、方向、平移，
在 odom +6°、14° 硬界、1 秒上限、3 cm 平移或任何感測異常時停車；
退出時連送三次 `stop` 並斷開 TCP（服務另有 0.5 秒看門狗）。
這些限制只涵蓋本次固定 motor 25 的小角度測試，不能推廣成大角度轉彎。

側邊紙板的唯讀測試：放在使用者所稱的右側後，−70° 到 −90°
約 40–50 cm 車體中心側距出現數十個回波，連續 39 幀可見；
移走後這組大面積回波消失。轉向測試端新增側向群聚禁行，
紙板在位時會回報 `side_left+side_right` 並拒絕轉彎。移走紙板後，
左後約 42 cm 有 19 點群聚、右後約 57 cm 有 12 點群聚仍存在。
手推至另一處開闊區後，右側禁行回波消失，但左後約 40 cm、
角度約 104–110° 的回波仍維持相近的車體相對位置；這支持
「隨車線材／零件」的可能，尚未經手轉車身與紙板遮蔽實驗確認，
**不可直接當雜訊忽略，也不可放寬側向禁行閘**。
現場照片顯示藍桶、白色板材和牆角位於車身附近，也有線材垂在底盤旁；
它們是合理的候選來源，但不能憑單張照片逐點對應 LiDAR 回波。
需先對照現場照片及手轉車身重測，不在近障礙旁實車轉彎。

2026-10-02 收工前，`grasp-service` 約從 17:26 起記錄
`SerialException: device reports readiness to read but returned no data`，
後續連續回報控制板無資料、停止發布新 odom。當時沒有再發移動指令；
17:34 執行 `sudo shutdown -h now`，SSH 由 Jetson 主動關閉，稍後 IP 無回應。
下次開機須先檢查 `/dev/myserial`、USB／控制板連線、服務日誌及 odom
是否恢復，未恢復前不可做實車動作。此異常原因尚未確認。

目前側向閘只檢查車體中段 x 在 ±35 cm、側距 35–65 cm 的
連續三點群聚。小於 35 cm 的 TG30 自身／機身回波區未被當成可靠
外部障礙感測；大於 65 cm 的障礙、轉彎掃掠半徑、高度低於光束的物品
仍須靠現場淨空確認。正式側向避障需要量測完整車體／手臂掃掠包絡、
確認近回波來源，設計並重複驗證完整轉彎安全策略。

唯讀指令：

```bash
cd /home/jetson/Documents/deploy_jetson2
/home/jetson/grasp_venv/bin/python3 integration/g2_turn_probe.py --selftest
/home/jetson/grasp_venv/bin/python3 integration/g2_turn_probe.py --probe
```
