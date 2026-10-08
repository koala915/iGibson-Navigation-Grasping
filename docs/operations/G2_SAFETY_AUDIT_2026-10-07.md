# 開機與共用序列安全檢查（2026-10-07）

## 實機檢查與修正

- Jetson `172.20.10.2`，/dev/myserial 指向 ttyUSB1，只有 grasp-service 持有。
- 初次啟動夾爪 165–166°，startup 判定 jaw_not_open_at_boot，TCP 7000 未開放。
  這是正確保護；不能自動張開可能持物的夾爪。
- 使用者明確確認空夾爪、掃掠區淨空及可立即斷電後，home 4.43 s、stow 3.80 s，
  最終 `[90,140,0,0,90,30]`；前後四輪命令為零，沒有發送非零底盤命令。
- 發現 lidar／rosbridge 雖 active，ROS master 註冊的 node URI 為 10.42.0.1，
  現場實際網路是 172.20.10.2；無法聯繫發布者，scan／odom 訂閱皆無資料。
  start-g2-ros.sh 改為 ROS_IP=127.0.0.1（所有機內 ROS 通訊，外部透過 rosbridge）。
  移除對外部 discover_ros_ip 的依賴。外部原生 ROS node 不能直接連 loopback，需使用 rosbridge。
- ROS 與 grasp-service 重啟後，startup travel_verified、TCP 7000 開放，
  6 秒收 scan 60、odom 118；scan age 約 0.249 s，odom 約 0.067 s。
  稽核亦檢查 odom→base_footprint TF 更新；不能只用 /tf 整體頻率判斷。
- 開機時系統校時約跳進 43 小時，舊 wall-clock uptime 因此失真。
  service uptime 改 monotonic；其他 arm timing 尚未全面改寫，開機校時穩定前不進行動作。
- 本次 service 多次重啟使依附的 grasp-vision 遇到 start-limit-hit；
  最後明確 reset-failed 並恢復一次。稽核納入 vision 狀態，但不自動清除其啟動限制。
- 新增 health 指令，僅取 chassis／odom cache 與 RSS，不請求 servo 編碼器。
  status 仍是實際 servo 讀取，行駛期間不得高頻輪詢。
- Python3 系統環境無 rospy；濾波器改用 ROS Melodic 的 Python2，launcher 保持 Python3。
  source ROS 環境後前視 filter＋GMapping 靜止 10 秒 smoke test 正常初始化、結束。
  這沒有完成新地圖或移動建圖品質驗收。

## 安全自動化腳本

`integration/g2_safety_audit.py` 已部署，預設唯讀，30 秒，最多 300 秒。
health／services／唯一 serial owner／board age／odom twist／scan／動態 TF／MemAvailable
必須同時通過才 ready=true。報告是當下快照，不授權自動移動或任務續跑。

```bash
cd /home/jetson/Documents/deploy_jetson2
/home/jetson/grasp_venv/bin/python3 integration/g2_safety_audit.py --seconds 30
```

加 `--repair-ros` 才允許有限修復：一次运行最多兩次，只可重啟
x3plus-g2-lidar／x3plus-g2-rosbridge，保留常駐 serial owner 與 odom 原點。
先確認零速、無導航 client、手臂 idle travel、board age ≤0.2 s、可用 RAM ≥1024 MB，
再向 service 要求原子 maintenance-lock。鎖內拒絕輪子及手臂命令。
ROS 資料全部恢復並重新通過控制閘後才 maintenance-unlock。
修復失敗／程序終止時鎖保留，不靠逾時自行解鎖。應人工排障後重新執行稽核；
必要時保持現場淨空，重啟服務重新做 startup 姿態驗證。

```bash
/home/jetson/grasp_venv/bin/python3 integration/g2_safety_audit.py \
  --seconds 60 --repair-ros
```

結果寫 `/tmp/g2_safety_audit.json`（單一 atomic replace，避免記憶體／日誌無限累積）。
離開時只停止自身監測，不恢復舊導航命令、不移臂、不重啟控制板／grasp-service。
serial／board 故障、手臂不明、低 RAM 等只回報失敗，不做盲目硬體恢復。
這個稽核是 idle maintenance 工具，不是完整行駛中監督器；有活躍導航 client 時拒絕修復。
RAM 低於 768 MB 判不通過；未對系統施加人工記憶體壓力，未保證任意程序不會 OOM。

## 已驗證與未解決

- 本機 24 項新監督／健康／maintenance 測試通過。
- 既有 Windows 回歸：chassis 58、travel 28、odom 22 通過（POSIX 限定項除外）。
- 停止 LiDAR 故障注入：偵測 scan stale，maintenance 鎖定、重啟一次，
  scan 恢復後解鎖，ready=true，四輪持續零，arm travel，可用 RAM 約 1978 MB。
- 停止 rosbridge 故障注入：偵測 scan／odom／動態 TF 中斷，重啟一次 rosbridge，
  常駐 odom 自行重連；不重啟 grasp-service，原點保留，維修鎖解除，四輪零。
  末次可用 RAM 約 1974 MB、service RSS 約 504 MB、odom failures=0。
- Linux 使用 fake plant 的 12 項 startup 回歸全部通過；沒有在測試中連接控制板。
- 全部修復需保持零速；機械卡輪／碰撞、真正拔 USB、行駛中故障、AMCL 新地圖、
  前視移動建圖、冷開機後固定 loopback 長期行為仍待驗收。
- kernel usb 2-1-port3 反覆 Cannot enable／config error；未證明對應哪個外接裝置，
  不等於目前 Rosmaster 失聯。control board age 正常、odom failures=0。
  應查 port3 線材、Hub 供電／裝置，不能用無限 restart 掩蓋。
- 核心執行程式部署前核对遠端與已提交基線 bytes 一致；未整庫 git pull 或合併 owner。
  Jetson deploy_jetson2 本身無 .git，部署採逐檔備份與複製。
- 回復副本 `/home/jetson/g2_safety_backup_20261007/`：grasp_service.py、graspctl.py、
  chassis_server.py、start-g2-ros.sh。回復前確認零速／掃掠區與唯一 owner；
  停 service 後還原，不覆蓋組員的後續修改。移除新增 auditor 不會自動移動機器人。

## 開機自動化（同日後續部署）

- 已啟用 `x3plus-boot-audit.service` 一次性檢查；grasp-service 的
  `boot-audit.conf` 同時要求啟動此服務。重啟 grasp-service 亦重新檢查。
- 原有 startup-travel 先讀取編碼器，經受控路徑歸位；巡航姿態已正確則不重做動作。
  夾爪閉合、讀值失敗或板回授異常仍保留原有禁止啟動條件，不自行張開未知持物。
- 歸位後、TCP 接受命令前先設 maintenance 鎖。檢查唯一序列 owner、板回授、
  零速 odom、動態 TF、scan、視覺與 ROS 服務、可用 RAM；连续兩次正常才解鎖。
- 檢查最多 90 秒，ROS 維修最多兩次；只在靜止且 RAM ≥1024 MB 時修復
  lidar／rosbridge，不重啟底盤控制服務、不發移動指令。一般通行 RAM 下限 768 MB。
  檢查失敗保持鎖定，systemd 不無限重試。
- 報告固定覆寫 `/run/x3plus/g2_boot_audit.json`，不累積 scan 或歷史資料。
- 實測修正開機診斷客戶端逾時造成 BrokenPipe 的問題：完成歸位才 listen，
  回覆時客戶端斷線也不退出控制服務。
- 服務啟動流程實測成功：巡航編碼器 `[90,140,0,0,90,30]`，四輪零命令，
  serial owner 唯一、odom／TF／scan 新鮮、全部服務 active，檢查結束解鎖。
  證據見 `g2_boot_audit_20261007.json`；今天沒有底盤行駛。
- 已設定下次開機自動執行；整機断電再開的冷開機驗收仍待下一次開機。
  本服務是開機一次性檢查，不代表已完成行駛中的全時故障恢復。
- 本次最終回歸：27 項 auditor／監督器測試、13 項 Linux fake-plant startup 測試
  全部通過；開機鎖在 TCP 開放前生效，診斷斷線不終止 service。
