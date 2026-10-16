# 獨立健康介面與相機有限恢復

## 完成範圍

在實機既有v23原稿加入獨立`/tmp/grasp_health.sock`與真實影像推論心跳，
並啟用`v23-control-vision-monitor.service`，隨開機啟動。
目前僅控制健康與相機恢復，不訂閱ROS、不修復LiDAR/rosbridge、不主動重啟Serial owner。
未驗證冷開機、負載夾取/投放全流程、Serial硬體故障恢復或RAM分級策略。

## 為什麼採相容補丁

實機控制程式包含組員較新的手臂/辨識功能，與本機完整transport版本不同。
未用本機整份檔案覆蓋實機；先取原稿與hash，精確替換健康快照、加入獨立socket生命週期，
以及在每次真實影像推論完成後寫心跳。AST確認grasp_service只有cmd_health與serve方法變更。
手臂/底盤寫入邏輯、模型、校正、速度映射及odom倍率未更改。

相關來源：

- `grasp/v23/health_server.py`：獨立診斷執行緒，僅接受health，不接受手臂命令。
- `grasp/v23/legacy_health_compat.py`：只讀快取，不讀servo；回報模型/holding/PID等必要metadata。
- `integration/frame_health.py`：原子寫入PID、單調時間、尺寸與實際推論耗時。
- `deploy/patches/add_independent_health_legacy.py`：拒絕未知或重複補丁，產生候選檔，不直接部署。
- 本機原稿/候選/diff/hash：`docs/calibration/odom_new_floor_2026-10-16/deployment_review/`。
- 遠端原稿備份/候選/manifest：`/home/jetson/health_compat_20261016/`。

舊transport沒有完整診斷時，健康回報保留serial=null、owner_restart=false，
不編造Serial健康資料；控制檢查仍驗證新鮮底盤回傳與有效odom。

## 載入與設定保留

部署前確認巡航位、空夾爪、零輪命令、唯一owner、RAM>=1GB及無未完成動作checkpoint。
備份並比對hash後只更換5個候選/新增模組；停掉原owner、確認無Serial持有者，再啟動新owner。
部署腳本有來源核對、候選編譯及失敗回復路徑；本次未觸發回復。

owner PID30006→2518，startup travel_verified、TCP ready、輪命令0，odom原點已重置。
原本左右90°與小角度JSON保留，但舊reference不得再拿來續算新odom。
此次是為部署而手動重啟一次，非監督器啟用owner自動重啟。

啟用新unit需daemon-reload。為避免讀回磁碟舊參數而丟失組員目前有效設定，
先從實際vision程序argv生成`grasp-vision.service.d/zz-preserve-deployed-20261016.conf`，
保留allow-top-clipped、frame-report、相機路徑、模型、homography與既有數值。
之後若修改vision啟動參數，須同步更新/移除此override，否則它會繼續覆蓋較早drop-in。
沒有改動grasp-service的ExecStart或校正參數；新unit啟用時owner PID未變。
daemon-reload會讀取原有磁碟設定，但不會自行重啟其他已執行服務。

## 常駐策略

- `integration/v23_recovery.py --control-vision-only`：不建立ROS訂閱，避免重演診斷流量干擾導航。
- 與`--restart-owner`同時使用會拒絕啟動。
- 手臂/底盤正在動作或有導航client時只回報task_active，不搶修服務。
- 每監督程序每服務最多2次修復、冷卻20秒；新進程/重新開機會建立新預算，不是永久累計。
- 故障時等待真實新PID心跳；一般待命需連續3次正常才ready，失敗超預算則degraded。
- 已有maintenance鎖時只容許相機修復，驗證唯一owner、巡航/零輪/新鮮回傳/RAM後操作；
  不解鎖。修復正常後仍顯示parked，不當成導航/完整展示ready。
- 未重播grasp/release，不自動home，不以服務active取代影像心跳。

此外修正了監督器以script執行時`__main__`與`v23_recovery`產生不同TransientFault類別的問題；
入口改呼叫同名模組main，確保backend拋出的短暫錯誤能被監督器捕捉。

實機使用獨立版本bundle：
`/home/jetson/health_compat_20261016/recovery/{v23_recovery,v23_demo,g2_safety_audit,mission_status}.py`。
unit檔在`deploy/systemd/v23-control-vision-monitor.service`。
此bundle只供上述監督模式，不拿來執行導航plan；不要與舊的全功能v23-demo-monitor同時啟用。
重新部署應一併複製四個模組並核對hash，而不是只替換單一檔案。

## 實測

| 測試 | 結果 |
|---|---|
| home/stow忙碌時獨立健康查詢 | 73次全成功，35/38次確實讀到arm_busy；最慢10.9ms |
| 向健康socket送home | 回ok=false，不執行手臂 |
| Recovery runner相機中斷 | 修復1次，12.78秒取得新PID新影像 |
| 維護鎖內常駐相機修復 | 修復1次，12.24秒恢復，鎖保留、owner不變 |
| 一般待命常駐相機修復 | 修復1次，15.35秒回ready，owner不變，測後重新上鎖 |
| 常駐監督器+導航感測8秒 | 107次檢查、81個scan，0過期；最大scan時間戳年齡0.366秒 |

最後狀態：monitor enabled/active，scope=control_vision、owner_restart_enabled=false、
state=parked、maintenance=true、owner PID2518；正式監督器本輪測試已用掉1次vision修復預算。
相機/owner服務正常，尚未重新開機驗證enabled行為。

相關105項測試通過、1略過。五組既有回歸亦通過：controller138、servo37、floor641、
UI50（略過1）、mission selftest。controller/floor仍為測試程序內的相對URDF路徑轉接，
未修復Windows中文絕對路徑問題。

## RAM記錄及下一步

### 使用者要求提前停止與關機

使用者要求依目前結果判斷後關機，已停止RAM transient unit並取回完整本輪檔案。
最終時長1398.713秒（23分19秒）、685筆，最低可用RAM1749.0MB，最高2295.1MB。
owner RSS首末507.37/505.34MB，vision首末301.55/298.52MB。
未觸及1024MB步驟門檻或768MB執行門檻，現有資料未顯示記憶體壓力，
因此本輪沒有依據因RAM而降低展示功能或直接放寬門檻。
期間曾重啟owner與vision，不使用跨PID首末值證明沒有洩漏；亦未完成滿30分鐘驗收。
summary complete=false代表使用者提前停止，不是記錄器故障。
完整log、summary及關機前狀態已取回本機，位於同日calibration資料夾。
關機前四輪0、maintenance=true、arm_pose=travel、holding=false、arm_busy=false。
已停止監督器並送出系統關機指令，後續172.20.10.3的SSH連線已離線；
關機過程未傳回完成JSON，不以網路離線單獨證明實體電源已切斷。記錄見poweroff_result.json。

下方為停止前的中途快照，最終結果以上段及baseline_ram.summary.json為準。

原RAM-only記錄仍繼續，最新約20.4分鐘、最低MemAvailable1749MB，尚未滿30分鐘。
期間有本次owner部署重啟與相機故障注入，RSS變化須按PID/事件分段，不能直接宣稱同程序無洩漏。
完成後讀`/home/jetson/demo_stability_20261016/baseline_ram.summary.json`的complete與時長，再取回完整log。

下一步：確認30分鐘結果、做一次正常冷開機驗證，再安排最小夾取/收臂/投放流程。
Serial transport完整隔離/自動owner重啟、RAM持續時間/降級策略仍需另行整合測試。
