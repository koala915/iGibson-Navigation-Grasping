# X3Plus 開機確認（2026-10-04）

目前 demo 場地約 2 × 1 m，未來擴充至 2 × 2 m。尚未載入舊地圖、啟動 AMCL 或驗證新場地。

## 已確認

- SSH 可登入 `jetson@172.20.10.2`。未在此文件保存密碼。
- `/dev/myserial -> ttyUSB1`，唯一持有者為常駐 `grasp-service` PID 4537；導航 serial-owner 服務未啟動。
- 本次開機服務日誌未見 `SerialException`；板端接收 age 回報 0.00 s，內部 odom 回授 valid=true，速度為零。
- 多次 status 的四輪命令均為 `[0,0,0,0]`，沒有發送底盤或手臂動作指令。
- 現場調整後，六個伺服角度多次讀取為 `[90,96,81,95,90,29]` 度；與程式巡航姿態 `[90,140,0,0,90,30]` 度不符。調整前 servo_deg=null；不能據此判定故障原因。
- `status.chassis.arm_pose=unreadable` 保留啟動／最後動作設定的狀態，不隨 status 的角度讀取刷新，不能拿它代替當下角度量測。
- 已啟動臨時 systemd units `g2-roscore`、`g2-lidar`、`g2-rosbridge`，目前 active；使用既有 TG30 launch，未修改來源。這些 units 重開機後消失。
- `/scan` 在穩定後 6 秒取樣收到 60 筆（約 10 Hz），每筆 2020 點，最後一筆約 1677 個有效距離，時間戳 age 約 0.20 s。
- TF 只有已驗證的 `base_footprint -> base_link -> laser`；靜態 tf publisher 約超前 0.1 s。

## 尚未排除

- ROS master 尚未就緒便啟動兩個 roslaunch，曾造成多個 master 自動啟動及 run_id 衝突。已改為 master XML-RPC 確認可用後再啟動，感測服務恢復。
- 常駐 grasp-service 的 odom bridge 沒有在 rosbridge 恢復後連線：status 持續 rosbridge=false、published=0。以相同 grasp_venv 另開純 roslibpy 測試客戶端，可立即連線，故伺服器可用，需釐清常駐程序重連路徑。
- 沒有 `/odom_setmotor` 或 `odom -> base_footprint` 新資料；完整導航 TF 尚未成立。
- systemd 回報 grasp-service NeedDaemonReload=yes；尚未 daemon-reload 或重啟 grasp-service。磁碟設定所列 ExecStart／兩個環境變數與載入值相符，未證實存在語意差異。
- 左後 104–110° 回波尚未重新做車體方向／纸板對照；沒有新增自遮罩或放寬禁轉閘。

## 下一步

先釐清並恢復常駐 odom bridge 的重連，驗證新鮮 odom 與動態 TF。核對實際手臂巡航姿態；任何實車動作前須現場確認淨空、有人可立即斷電。不另開 `/dev/myserial`，不自行合併 owner 分支或覆寫組員改動。

相關 Jetson 檔案限定於：

- `/home/jetson/Documents/deploy_jetson2/grasp/v23/odom_bridge.py`
- `/home/jetson/Documents/deploy_jetson2/grasp/v23/grasp_service.py`
- `/home/jetson/Documents/deploy_jetson2/grasp/v23/chassis_server.py`
- `/home/jetson/Documents/deploy_jetson2/deploy/ros/start_g2_test_stack.sh`
- `/home/jetson/Documents/deploy_jetson2/deploy/ros/x3plus_tg30_navigation.launch`

初次開機檢查僅啟動感測與 ROS 通訊程序；未重啟 grasp-service、未修改機器人程式、未載入舊地圖。

## 後續巡航姿態調整

使用者確認夾爪無物品、掃掠區淨空及有人可立即斷電後，透過既有常駐服務執行 `home -> stow`。

- home 成功，11 次迭代，約 4.17 秒；角度為 `[90,74,8,9,90,30]` 度，arm_pose=e1。
- stow 成功，約 3.79 秒；連續三次角度確認為 `[90,140,0,0,90,30]` 度，arm_pose=travel。
- 前後四輪命令均為零；沒有發送非零底盤速度或馬達命令。
- 常駐 odom bridge 仍未連線，published=0，尚未解決導航回授阻礙。
- 尚未設定開機自動移臂。現有 stow 僅接受 E1 起點；開機姿態不可假設已在 E1，不得跳過回授／姿態／現場動作條件。

## 最終更新：odom 恢復與自動開機巡航姿態

本節取代前文的「尚未設定開機自動移臂」與「odom 尚未恢復」狀態。使用者已明確要求往後開機自動調整手臂。

- 開機設定已啟用 `GRASP_SERVICE_STARTUP_TRAVEL=1`。每次 grasp-service 啟動／重啟會讀實際角度；巡航位跳過動作，E1 經 S2=115° 中繼點到巡航位，其他姿態先受保護地回 home 再 stow。
- 無回授、板端接收逾時、底盤非停止、夾爪非張開或姿態移動／驗證失敗：停止自動流程、不啟動底盤 TCP，保持診斷 socket 可用。閉合夾爪可能持物，不能在服務失去持物狀態後自動張開。
- 開機會發生實際手臂動作；操作開機／重啟時應讓掃掠區淨空。不是自主避障或碰撞偵測功能。
- 持久化啟用 `x3plus-g2-roscore`、`x3plus-g2-lidar`、`x3plus-g2-rosbridge`，以 bounded readiness 檢查依序等待 master、rosbridge；grasp-service 排在其後。不載入舊地圖或 AMCL，不啟動第二個 serial owner。Wi-Fi 尚未取得位址時，機內 ROS 使用 loopback，不因 DHCP 尚未完成而拒絕啟動。
- 原來常駐 odom 未重連的精確原因未完全證實；未修改 roslibpy 或 odom_bridge.py。修正啟動順序並重啟常駐服務後，發布恢復。
- 實測 odom 約 19.82 Hz，時間戳 age 約 0.0089 s；`odom -> base_footprint` age 約 0.0086 s；scan 約 9.99 Hz，時間戳 age 約 0.1348 s。
- rosbridge 停止約 5 秒期間，published 固定在 701；重新啟動後不需重啟 grasp-service 就恢复連線並增加至 714。沒有下發底盤速度。
- 已實車驗證從 E1 重啟服務，自動 stow 回巡航位並確認 `[90,140,0,0,90,30]` 度；巡航位重啟時不移臂。四輪命令保持零，序列埠唯一持有者仍為常駐服務。grasp-vision 已恢復 active。
- 純邏輯測試：12 項新開機流程測試、28 項既有姿態檢查、22 項 odom 檢查、60 項底盤互鎖檢查通過；shell 語法與 systemd units 驗證通過。
- 尚未實際整台關機／開機；目前驗證範圍為服務啟動／重啟與 ROS 啟動順序。未驗證新場地 SLAM／AMCL、側向回波來源或自主導航。

部署來源與配置副本保存在本機 `robot_update_20261004/`。僅修改常駐服務 grasp_service.py，增加 ROS 啟動服務、readiness helper 與獨立 startup-travel.conf，未覆寫原有 unit、drop-ins 或控制器／姿態保護程式；部署前已確認原 grasp_service.py 雜湊未變更。

Jetson 原檔備份：`/home/jetson/robot_boot_backup_20261004_131019/`。

回復時先停 grasp-service／grasp-vision，在停止狀態下還原備份 grasp_service.py，移除本次新增的 startup-travel.conf，停用／停止本次新增的三個 x3plus-g2 ROS 服務，再 daemon-reload。這會回復手動啟動感測堆疊的流程，不是保留本次開機自動化的設定。回復前同樣需要現場確認手臂掃掠區淨空；備份原 unit 與原 g2-chassis.conf 僅供比對，不應覆寫後續組員修改。

## 2 × 1 m demo 場地建圖試驗

- 靜止掃描得到約 1584 個 3 m 內有效點；可見界線約 x=-0.288..1.782 m、y=-0.844..0.634 m。前方 p10 約 1.198 m；左右 p10 約 0.516／0.374 m，場地輪廓足以建圖。
- 左右側向安全閘均觸發（約 0.35 m）；未放寬自遮罩或繞過禁轉閘，因此本次不原地轉彎。
- 使用 GMapping，解析度 0.02 m，初始範圍 4 × 4 m；只載入現有 `/scan`、odom 與 TF，不載入舊地圖或 AMCL。
- 執行兩段各 0.25 m 的受保護直線移動：實際 odom 0.266 m、0.267 m。前方距離約 1.281 -> 1.031 -> 0.714 m；每段停止後 vx=0。
- 最終 odom 約 `[0.5335, 0.0031, 0.0129]`，GMapping 的 map -> base_footprint 約 `[0.466, -0.018, yaw 2.916°]`。
- 地圖共有 695 個 occupied、4186 個 free cells；牆面 occupied bbox 約 2.28 × 1.64 m。長方形主要牆面清楚，起點側有 LiDAR／車體遮蔽造成的缺口與短重影，適合確認「可建圖」，尚不足以視為最終導航地圖。
- 地圖已保存於 Jetson `/home/jetson/maps/demo_2x1_20261004/demo_2x1.{pgm,yaml}`，本機副本在 `maps/demo_2x1_20261004/`。
- 保存後已停止臨時 GMapping，避免之後與 map_server／AMCL 的 `map` TF 衝突。底盤四輪為零、手臂維持 travel、odom 與 rosbridge 正常。

## 360° 原地旋轉嘗試

- 使用者確認場地具有足夠安全距離後，建立獨立的分段旋轉工具 `integration/g2_mapping_spin.py`。工具每約 30° 停車驗證，檢查全周 LiDAR（包含後方）、odom／時間戳、平移漂移與最終停車；未修改既有 `g2_turn_probe.py` 的側向禁轉閘。
- 車體量測旋轉包絡半徑約 0.192 m；當下全周回波 p01 約 0.306–0.313 m，估計保留約 11 cm。硬閘設為 0.30 m，單點回波最小約 0.27–0.29 m。
- 第一次以 wz=0.60（馬達約 ±20）嘗試，2 秒內僅約 4° 轉角並產生約 5 cm 平移；因無足夠轉角進展而停止。
- 第二次、第三次使用既有小角度測試輸出 wz=1.00（馬達 ±25）。分別在 0.8 秒與延長後的 2.5 秒無有效進展，自動停止；未提高到更大馬達輸出。
- 最終 odom `[0.5836,-0.0260,0.0786]`，約 yaw 4.5°；GMapping `map -> base_footprint` 約 yaw 4.0°，LiDAR 場地朝向也改變。這證明車體只完成少量旋轉並伴隨明顯平移，不是 odom 單獨漏報。
- 360° 未完成。若依目前約 4°／5 cm 的運動型態繼續，會先超過 8 cm 平移限制並接近牆面，因此安全中止。未碰撞，四輪已歸零。
- 部分嘗試地圖保存於 Jetson `/home/jetson/maps/demo_2x1_20261004_spin_attempt/` 與本機 `maps/demo_2x1_20261004_spin_attempt/`，僅作診斷，不能作導航地圖；臨時 GMapping 已停止。
- 再試前應現場確認四輪是否實際同時反向轉動、是否有輪子頂住軟牆／地面接縫、車重分布與電池電壓。可先把車移至場地正中央，再做單次 10–15° 量測；未解決平移漂移前不做完整 360°。

### ±30 修正與完整旋轉

- 使用者確認場內無障礙、近距離回波只有牆面／車身／線材／雜訊，並指出原地轉向需馬達 30 才能避免打滑。
- 找到舊換算限制：`CMD_MAX_ABS_WZ=1.0` 與 `ANGULAR_MOTOR_PER_RAD_S=25` 使任何純轉向最多只能得到 `[-25,-25,25,25]`，把 wz 提高到 1.2 仍不會變成 30。
- 將 `integration/sugarbox_rl_motor_server.py` 的轉向比例改成環境變數 `G2_ANGULAR_MOTOR_PER_RAD_S`，預設仍為 25；只有 grasp-service 的新 drop-in `turn-motor30.conf` 設為 30。直線換算與維修用 x3plus-navigation 預設不變。原檔備份在 `/home/jetson/robot_boot_backup_20261004_131019/sugarbox_rl_motor_server.py.before_motor30`。
- 驗證 default 純轉向仍為 ±25，override 為 ±30；既有 60 項底盤互鎖測試全部通過。
- ±30 短測實際轉 20.05°、平移 0.021 m、停止後 wz=0，證實能克服靜摩擦。
- 完整圈分兩段完成：第一段 213.08°，在累積平移 0.081 m 時依限制停止；續段 154.30°，平移 0.026 m。合計 367.38°，所有 checkpoint 的全周 p01 距離為 0.311–0.355 m，最終 wz=0，未碰撞。
- 連同短測，服務重啟後的最終 odom 為 `[-0.0958,0.0417,0.4786]`；四輪為零、手臂 travel、odom／LiDAR／rosbridge／vision 均正常。
- 新地圖保存於 Jetson `/home/jetson/maps/demo_2x1_spin30_20261004/demo_2x1_spin30.{pgm,yaml}`，本機副本在 `maps/demo_2x1_spin30_20261004/`。
- Hough 長線段檢查：主要水平牆約 0.57°、1.74°；主要垂直牆約 86.57°–90°，四邊大致平行／垂直。一圈已足以確認牆面一致性，未再增加圈數。
- 地圖主輪廓閉合，但上緣仍有近距離車體／線材造成的波紋及斜向缺測射線；整張 grid 因離群點擴展到 736×192 cells，occupied bbox 約 3.40×1.64 m。這份圖適合 demo 場地幾何驗證，正式導航前仍應裁切／清理並做 AMCL 測試。
- 保存後停止臨時 GMapping，避免與後續 map_server／AMCL 的 map TF 衝突。
