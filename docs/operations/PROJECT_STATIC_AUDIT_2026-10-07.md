# 專案靜態檢查與待驗證項目（2026-10-07）

此為當日整合前的檢查基線。CI、新 v23 plan、monotonic、vision 心跳、RAM 起動閘與
UI 接入的後續修正，見 `docs/handoff/V23_FINAL_DEMO_2026-10-07.md`。

## 本次檢查結果

本次只做離線檢查，沒有發送機器人移動／手臂命令，也沒有修改控制程式。
範圍為 CI 所列測試、v21/v23/v24 靜態回歸、新增 G2 測試、Python/shell 語法及當前部署設計。
不包含模型重新訓練、模擬正式評估重播或全部實機行為。

- 136 個 Python 檔 AST 語法檢查通過；11 個 shell 檔 `bash -n` 通過。
- 27 組 Python 測試入口中，26 組在相容環境重跑後通過；1 組受 Windows 平台限制未通過。
- Git Bash 執行 systemd helper 測試通過；`git diff --check` 通過。
- v21 controller 138、v23 controller 163、兩套 floor guard 各 641、共用 safety guards 73 項通過。
- 新 auditor／demo supervisor 27 項通過；任務 FSM、pipeline、vision pipeline selftest 通過。
- v23 startup 13 項在本機 Windows 有 3 項錯誤：此 Python 的 socket 沒有 AF_UNIX。
  同日先前 Jetson fake-plant 13 項通過的證據在 G2_SAFETY_AUDIT 文件；本次 SSH 連線
  172.20.10.2 逾時，未能重新跑 Linux startup／POSIX SIGTERM 測試。
- chassis 測試雖通過 58 項，明確跳過 POSIX `systemctl stop while driving` 項。
  不把整體 exit 0 解讀成所有平台測試都已執行。

第一次 Windows 執行有 6 組錯誤，其中 5 組涉及 PyBullet 無法載入中文路徑的 URDF。
檔案確實存在；使用純英文路徑實體副本與 `PYTHONIOENCODING=utf-8` 重跑後通過。
途中 junction 與不完整副本造成的跨磁碟 relpath／缺少 import 檔案錯誤，均屬重跑環境問題，
不是新增控制邏輯失敗。原始結果及重跑紀錄保留於 `static_audit_20261007/`。

## 已確認的工程缺口

| 優先 | 項目 | 證據與影響 |
|---|---|---|
| 高 | 任務主流程尚未接入常駐 G2 控制架構 | `integration/mission_pipeline.py` 仍載入 v21，建立自己的 GraspController／序列持有者；要求沒有其他 7000 server。不能直接與現行 grasp-service 並行，也不能把 stub 任務測試通過當成 v23 常駐整合通過。 |
| 高 | CI 未涵蓋目前主要新控制流程 | `.github/workflows/tests.yml` 缺 v23 controller、chassis、odom、travel、startup 及新 auditor／demo supervisor。requirements-ci.txt 亦未列 pytest。直接 push v23-grasp-test 不觸發此 workflow；PR 或手動 dispatch 才會。 |
| 中 | 手臂部分逾時仍依賴 wall clock | grasp_service.wait_for_detection 使用 time.time；本次只把 uptime 改 monotonic。校時跳變仍可能改變等待時間，尚缺跳時回歸。 |
| 中 | 開機 ready 不能代表長期運行健康 | boot-audit 為 oneshot，完成後不再監控；vision 僅檢查 active，沒有相機影像／偵測新鮮度 gate。service health 的手臂姿態為快取，真正編碼器是在 startup／arm command 讀取。 |
| 中 | RAM 保護尚未覆蓋全部入口 | mapping 有 prlimit 與可用記憶體閘；grasp／vision unit 未設 MemoryHigh／MemoryMax。尚未做整合峰值與低 RAM 壓力驗證；不能保證任意程序不會 OOM。 |
| 中 | 車身／線材濾波仍未驗收 | mapping 預設只啟用前 180°；footprint、speckle 預設 false。前方自回波仍可能入圖，原始 /scan 的停車判定仍可能受雜訊影響。自遮罩來源與尺寸尚未量測。 |
| 中 | 手動修復鎖的恢復流程未完整自動化 | 一般 --repair-ros 只有在實際選定修復單元並恢復感測後解鎖；若上次稽核中止留下鎖、感測已自行恢復，單純重跑預設稽核不會解鎖。應明確使用 boot gate／經驗證解鎖流程，不盲目宣稱再次執行就修好。 |
| 低 | 文件／Git 狀態待同步 | 專案根目錄不存在 CODEX_HANDOFF.md；目前新開機與修復檔仍有未提交變更。舊 progress／handoff 不宜作唯一現況來源。 |

## 尚待實機驗收

1. 整機斷電後冷開機：ROS loopback、手臂歸位、boot gate、vision 及唯一 serial owner。
2. USB port3 的 Cannot enable／config error：目前僅有同日診斷紀錄，根因與裝置歸屬未確認。
3. demo supervisor：UDP/TCP 中斷、雜訊消失後有限恢復、停車距離、卡輪與低 RAM；目前是短直線候選，不是完整避障巡航。
4. 2×1 m 地圖：既有 spin30 主輪廓可作幾何參考，但 occupied bbox 約 3.40×1.64 m，仍有離群點／波紋。需用濾波新掃描做 A/B，再驗 AMCL／定位漂移／路徑；不能直接列為正式導航圖。
5. 轉向掃掠包絡、側向避障、長距離及不同角度；完整導航→接近→夾取→投放→巡航任務。
6. v23 manifest 的 LEFT/RIGHT motion／yaw mapping、E1 FOV、實際搆及範圍、8 mm guard margin 等 false gate。
7. v24 候選的新權重載入、deployment guard parity、dry-run、固定座標與視覺夾取；目前硬體 gate 仍全 false，不應切換正式部署。

靜態測試沒有發現尚未排除的控制邏輯斷言失敗；仍有平台限制、CI 覆蓋不足及上述整合／實機缺口。
本次沒有推送、合併、部署或啟動實機動作。
