# Codex 下一步

關機操作已送出，之後172.20.10.3:22無法連線（SSH已離線）；遠端關機前未傳回完成JSON，不以此聲稱實體電源已切斷。poweroff_result.json已保存。

10/16 本輪結束：使用者要求提前停止RAM記錄、依現有資料判斷後關機。記錄已停止/取回：1398.713秒（23分19秒）、685筆、最低MemAvailable1749.0MB，未觸及1024/768MB；first/last RSS為owner507.37→505.34MB、vision301.55→298.52MB，但期間有部署/相機重啟，不把跨PID數值當成無洩漏證明。summary complete=false為使用者提前停止，非滿30分鐘驗收。檔案baseline_ram.jsonl、baseline_ram.summary.json、shutdown_assessment.json已保存本機calibration資料夾。關機前輪命令0、maintenance=true、travel、holding=false；接著依使用者要求系統關機。下次開機先確認實際IP、獨立健康socket/影像心跳及新監督器enabled狀態，不沿用已重置前的odom reference。

10/16 最新：獨立健康介面與有限相機自動修復已正式啟用。以實機原稿最小補丁新增/tmp/grasp_health.sock、相容metadata及實際推論心跳/tmp/x3plus_vision_health.json；保留組員控制/校正/CLI。部署重啟owner一次，PID30006→2518、odom原點已重置，舊量測reference不可直接續用。home/stow忙碌期間73次健康查詢全成功、最慢10.9ms。受控相機故障恢復：測試runner12.8s、維護鎖內常駐模式12.2s、一般待命常駐模式15.3s，均只重啟vision1次、owner未變。v23-control-vision-monitor.service已enabled/active，scope=control_vision（無ROS訂閱/修復，不自動重啟owner）；目前parked維持maintenance鎖，仍可有限修復相機，不自動解鎖。每監督進程每服務最多2次、冷卻20秒，讀到新PID真實心跳才恢復。新服務使用/home/jetson/health_compat_20261016/recovery四模組bundle，勿與舊全功能monitor同時啟用。daemon-reload前已將vision有效argv保存為zz-preserve-deployed-20261016.conf，保留allow-top-clipped與frame-report；未改owner啟動參數或校正。冷開機尚未實測。相關105通過/skip1，五組回歸通過（controller/floor仍測試內相對路徑轉接）。RAM-only記錄最新約20.4分鐘、最低1749MB，尚未滿30分；後續先查baseline_ram.summary.json。詳見docs/operations/HEALTH_COMPAT_RECOVERY_2026-10-16.md。

10/16 最新工作改為下次開機穩定性清單：使用者無量角器，小角度左5°第1輪僅目測相近、精密量測/其餘小角度輪次暫緩（使用者要求下次再做）。相機原設定路徑已重新出現，連續5張640x480可讀；reset-failed/start grasp-vision恢復原已載入配置（含組員frame-report/allow-top-clipped），未改相機校正或daemon-reload。home→stow實車成功、Serial owner同PID30006、原生ROS odom/TF最大更新間隔約65ms，底盤停止上鎖。發現記錄器ROS訂閱會影響rosbridge導航：有ROS記錄8秒70/125次stale_stamp；停記錄0/104；改RAM-only並行0/109。記錄器integration/g2_baseline_recorder.py現預設只記RAM，--ros-topics僅隔離診斷勿與導航並行。30分RAM基線正在x3plus-baseline-ram-20261016.service執行，遠端/home/jetson/demo_stability_20261016/baseline_ram.jsonl；初期61秒min RAM1754.6MB，尚未滿30分。其他三個實驗記錄unit已停止，其summary complete=false。後續先查RAM記錄是否完成，再處理實機無/tmp/grasp_health.sock、新監督器未部署/版本不相容問題，保留組員校正；勿啟用自動owner重啟或宣稱修復部署完成。詳見docs/operations/BOOT_STABILITY_TEST_2026-10-16.md。

10/16 小角度測試已開始：使用者已擺好起點，左5°第1輪以6個短脈衝停穩，odom=5.74846°、odom平移約2.77mm，輪命令0、maintenance=true、odom有效、RSS507.4MB。已請使用者量本輪實際角度（或距轉動中心50cm處側向間距），等待回覆，勿搬車/再轉。報告small_left5_01.json與small_left5_01_reference.json；工具新增--target-deg 5/10/90與較緊角度/位移界限，38項相關測試通過。這是第1輪，還不能宣稱5°精度/重複性驗收；後續原計畫左右5°/10°各3輪，需逐輪外部量測，先依本輪實測決定脈衝調整，不改odom倍率。

10/16 右轉外部回覆已收到：使用者確認「基本上轉了90度」，對應本輪odom右轉89.23017°。左右各一次90°的近似外部對照均相符，保留原角速度倍率；不把目測/近似基準算成精密誤差值，重複性與連續轉動煞車仍未驗收。車保持停止、maintenance鎖定；未自行進行下一輪。

10/16 最新右轉已執行停止：以左轉完成位置為新reference，28個短脈衝累積右轉89.23017°，odom平移約2.69cm，輪命令全零、maintenance=true、odom有效、RSS507.4MB。報告right_turn_reference.json、right_turn_real.json；已請使用者確認是否基本回到最初直線及偏差角，等待量測，不再轉車。左89.93649°與右89.23017°的odom淨差約+0.706°，不是實際角誤差。角度倍率不變。校正工具已支援--direction right，37項相關測試通過，維持本次已授權的self-return監督模式，不代表自主避障驗收。

10/16 最新左轉90°實測已完成：沿最初filtered_turn_left2起點續算，odom累積89.93649°，使用者確認「基本上完成90度」。只代表本次左轉近似外部基準相符，不做精密誤差/比例校正；保留倍率。21個補轉脈衝（先至84.767°、最短脈衝未有效起轉而停止；核對同一reference後續轉），全程原始起點odom平移約3.24cm，最後輪命令0、maintenance=true、odom有效、RSS507.4MB。報告quarter_turn_real.json、quarter_turn_finish.json；新工具integration/g2_quarter_turn_calibration.py。使用者已明確確認近回波全是線材/機構，本次以--operator-confirmed-self-returns僅記錄包絡回波不擋下，並非自回波分類/避障通過；全域LiDAR濾波設定未再放寬。右轉、重複性、連續90°煞車精度尚待驗證；未啟動後續轉動。

10/16 常駐LiDAR濾波已依使用者要求部署並啟用：driver改發/scan_raw，固定profile節點g2_self_return_filter發/scan；沿用已enabled的x3plus-g2-lidar開機載入，未重新開機驗證。profile為deploy/ros/lidar_self_returns_2026-10-16.json，26個原始雷射角格，開機不學習新遮罩；原始scan保留，mask為NaN而非可清空射線。12組同時間stamp/laser frame/幾何配對通過，每幀約55–65點濾除、現有導航解析成功、RAM可用約1.9GB、底盤停止上鎖、odom正常。部署前備份/home/jetson/lidar_filter_backup_20261016；需保留ROS wrapper的ROS_PACKAGE_PATH與腳本可執行權限。使用者回覆實際左轉約14°，odom14.729°差約+0.73°（近似尺量，非正式校正），倍率不變。下一步處理停止延遲／超轉，勿重新執行會從已濾波/scan自動採樣的舊校正流程；若重做profile須取/scan_raw並在靜止且現場確認淨空時明確執行。

10/16 最新：使用者現場確認周邊近回波為線材／機構並明確授權濾除。新增 integration/lidar_self_return_filter.py、g2_filtered_turn_calibration.py，僅本次監督校正側向 profile（10 個不同靜止 scan、1°格、反覆出現的距離±2.5cm、原始/scan不變、行進中不學習）；遠距側向群聚需3個不同scan確認，<=26cm立即停止。23項相關測試通過。實車左轉一次：發5筆速度，odom到6.50°停車，停穩14.73°／平移0.73cm，超14°限制，結果為未通過；已輪命令零、maintenance鎖定、odom正常。等待使用者量「起始線到目前車身」實際左轉角，勿先搬車或再轉、勿改倍率；需先處理停車延遲／超轉，不能宣稱角度精度驗收完成。詳見 filtered_turn_left2.json 與當日 operations 記錄。

10/16 最新幾何量測：四輪中心到巡航外緣前20／後13／左12／右12 cm，使用者確認含線材；雷射中心前方約13 cm，現行 TF／客戶端為10 cm。已保存 measured_geometry_comparison.json 並以新偏移離線重算，近包絡外回波減少但未消除，未改正式設定／遮罩或發轉向命令。下一步用一致的實測座標與預定轉角掃掠範圍評估，再做馬達轉角校正。下列舊的「等待外形量測」已由本段取代。

決賽主線為 `v23-grasp-test`／v23 常駐服務，入口 `grasp/v23/demo.py`。

- **10/16 優先：新地板里程計校正**。已連上 `172.20.10.3`；依 [新地板校正](docs/operations/ODOM_NEW_FLOOR_2026-10-16.md)：S3 恢復可讀、home→stow 成功；三段 odom 合計 88.85 cm、尺量約 88–89.5 cm，倍率不變。微動三輪實際 1.0／0.6／0.7–0.8 cm，可觀察到 <5 cm 點動，非精準定距驗收。已停穩、maintenance 上鎖。轉向唯讀探測觸發左側 35–41 cm 回波（約 86–87°、102–109°），等待使用者量左側牆距及說明線材／支架；未轉車、不放寬側向閘。轉角與一般直線停距仍待校正，未改正式程式／係數。
- **方向 A/B 已完成**：左前紙板放置時左近側約 280 點、移除後 0–1 點；右固定牆約 520 點不變，左右方向正常。已修正 turn probe 35 cm 內側盲帶為既有車體半寬 12 cm，32 項相關測試／實際 scan 回放通過，單檔備份後部署並唯讀確認會報右牆；未改 driver／TF／服務、未轉車。右牆近距與左側未明回波仍阻擋轉向，不直接放寬閘，轉角校正待處理量測位置。
- **下一個現場回覆**：使用者手動左轉 90° 的掃描已保存，不能當成馬達轉角校正；已盡量收束線材，100–110° 回波點數／距離前後相近，不確認為線材、不加遮罩。固定 65 cm probe 側距仍阻擋；已請尺量「轉動中心到巡航外緣」前／後／左／右四個公分數（含手臂、支架、固定線材），等待幾何量測後推導掃掠包絡候選，不直接放寬閘。未發馬達轉向、maintenance 鎖定；勿重問安全。

- 下次開機優先依 [減少展示中斷清單](docs/operations/NEXT_BOOT_DEMO_STABILITY_2026-10-12.md) 執行：先回傳／RAM 基線，再部署與共用控制、短暫恢復及完整展示；RAM 分級／時間確認仍待實作與測試。

- 下次開機順序與 10/5–10/12 進度報告：[報告](docs/planning/PROGRESS_REPORT_2026-10-05_TO_2026-10-12.md)。

- 10/23 前交付、20 小時排程與最新 LiDAR 邏輯檢查：[交付計畫](docs/planning/V23_DELIVERY_20H_2026-10-11.md)。
- 最新組員 GitHub 成果及尚未整合項目：[同步核對](docs/handoff/GITHUB_SYNC_2026-10-12.md)。機器人目前未開機／連線。

- 決賽目的與優先順序：[決賽目標](docs/planning/V23_FINAL_DEMO_PRIORITIES.md)。
- 最新狀態與部署界線：[交接](docs/handoff/V23_FINAL_DEMO_2026-10-07.md)。
- 操作：[FINAL_DEMO.md](grasp/v23/FINAL_DEMO.md)。
- 有限修復／降級／升級重啟：[恢復說明](docs/operations/V23_RECOVERY_2026-10-07.md)。

10/16 已連上 `172.20.10.3`，本次底盤回傳／odom／scan 正常，手臂巡航姿態已實測；相機因 by-id 路徑不符仍失敗，整體展示尚未 ready。正式服務先前曾有回傳停止問題，本次短時正常不等於根因已排除。
新版 recovery／monitor 主體只在本機與 Jetson `/tmp` 假硬體驗證，尚未覆寫正式部署或啟用監督單元；10/16 僅 turn probe 近側漏報修正已單檔備份部署，勿混淆。
優先唯讀確認回傳／相機，再比對遠端檔案及組員校正、逐檔備份部署與驗證。
USB issue 暫緩；今天沒有場地。不得啟動第二個 Serial owner、覆寫組員校正、合併 owner，
或把靜態通過當成完整自主導航／AMCL／側向避障驗收。
