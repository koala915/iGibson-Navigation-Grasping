# Codebase audit 第一批修正交付

基準 branch：v23-grasp-test；commit：67d29af58ae68acbba020e427c748d1790bc3b1b。
修正期間未 commit/push；原有 CLAUDE/progress、vision bridge、UI、圖片與 deploy_v23 未提交工作保留。
第一輪原始 audit 與重現結果保留在 [audit folder](codebase_audit_2026-09-06/CODEBASE_AUDIT.md)；
以下是修正後狀態，不覆寫基準結果。

## Patch 說明

| Audit ID | Problem / Root Cause | Change | Risk / remaining limits | Validation |
|---|---|---|---|---|
| B02 | Mission 直接建構 controller，跳過 standalone release gate；real 可 skip hash。 | 在裝置開啟前重用 v21 release gate；real 禁止 skip-model-hash。 | 候選仍需 unlock；不接受同 shape 就代表相同語意。 | mission tests 驗 candidate 拒絕、禁止 skip hash、homography gate。 |
| B03 | Mission 繞過現有 orchestrator 的 homography 禁止 real 閘。 | final align/latch 未接校正前，mission --real 也拒絕。 | 這是封鎖不安全入口，不是完成 homography 整合；既有 versioned standalone 功能保留。 | 入口拒絕測試、既有 v21/v23 controller/launcher 回歸。 |
| B06 | stdin EOF/OSError 被視為同意啟動。 | 不設 started，記錄 fault。 | 明確的 start-immediately 仍是另外的 CLI 行為；所有 real mission 目前先被 B03 擋下。 | closed stdin test、mission FSM/selftests。 |
| B07 | Thread._stop 被 Event 蓋掉，join 失敗中斷 cleanup。 | 改名 _stop_event；各資源獨立 cleanup。 | timeout 會告警，不宣稱可以中斷任意卡死 driver。 | fake odom thread 可 join；所有 controller 回歸。 |
| B08 | 消費者相信保存的 fresh/stationary boolean。 | 讀取 OdomPublisher.state 時重算 age；過期不得 stationary。 | 尚用 wall clock；B01 底層 cached get_motion_data 新鮮度未解決。 | frozen snapshot ageing test。 |
| B09 | UI 對 A/B/C 產生同一組不相容 CLI。 | C dry 不帶不存在的 --dry-run；real 在 server 拒絕並说明缺少的模式證據；修正 UI 提示與文件。 | parser 可接受不等於 assets/ROS/相機已部署；B18 standalone placeholder 仍待處理。 | 三個真 parser contract test；UI 48 tests。 |
| B10 | 舊最小 scalar 24 把 near 速度與轉速抬高。 | ARM 最小 scalar 歸零，角速度量化向下取整。 | 只證明命令限速；低速死區與實際速度需要實機量測。 | near linear/turn cap tests。 |
| B12（部分） | 無可用影像也被當成物體消失。 | verify 回傳 True/False/None；不足有效觀測為 UNKNOWN，mission 只收 True。 | camera pose、獨立新影格、視野內負證據仍未完整驗證；不宣稱視覺驗收已可靠。 | unavailable-frame UNKNOWN test；mission end-to-end。 |
| B15 | AMCL wrong frame、zero quaternion、negative variance 可通過。 | 強制 map frame、finite/近單位 quaternion 與非負使用中 variance。 | frame 改名需正式 config adapter；未完成全 covariance PSD 或 ROS clock 支援。 | wrong-frame/zero-q/negative-cov tests；ros_io selftest。 |
| B16（部分） | 全 NaN scan 變成清空間。 | empty/全不可用 scan 拒絕；保留合法 +inf 無回波。 | 混合失效 coverage、sensor capture timestamp 仍需後续處理。 | NaN vs +inf tests；nav_rl selftest。 |
| B20 | controller 未釋放 serial；單項 close 失敗跳過後續。 | v21/v23 各自 close detection、servo device、FK；receiver 有限等待 thread 結束；UI subprocess pipes 關閉。 | driver 自身 RX thread 終止能力與板端停車仍須驗證。 | cleanup failure test；兩版 controller/servo/floor/launcher；UI 無先前 pipe ResourceWarning。 |
| B23 | legacy wrapper 只驗 shape，可能接 incremental 權重。 | model package 強制 arm_action_mode；legacy wrapper 僅接受 absolute。 | 舊 package 缺欄位會拒絕，必須依真正訓練契約補齊；不能猜欄位。 | model package 11 tests。 |
| B28（部分） | 文件仍顯示舊測試計數與可從 UI 開 real。 | 更新 README/CLAUDE/UI/MISSION/TEST_PLAN 與 CI 名稱。 | 原始歷史 audit 不重寫為修正後狀態。 | diff check、UI 回歸。 |

## Validation

先前 audit 的 offline baseline 已完成；本批 patch 後以下全過：

- v21 controller 135、servo 37、floor 641、launcher 29 checks。
- v23 controller 148、servo 37、floor 641、launcher 50、three-pose scan 89 checks。
- safety 68、mission end-to-end 28、model package 11、UI 48 tests。
- homography 9、camera capture recovery 7、vision bridge pose 20、trash target 18 tests。
- mission_fsm、mission_pipeline、vision_grasp_pipeline、ros_io、nav_rl selftests 全過。
- 涵蓋目前 .github/workflows/tests.yml 的所有測試指令；本機 Windows Python 3.12，
  不代表已在 Jetson Python 3.8 或 ROS Melodic 上執行。
- Controller 首跑在 CP950 輸出字元時中止；設 PYTHONIOENCODING=utf-8 後完整重跑通過。
- 沒有實體動作。未降低 floor guard、改 motor mapping、改 E1/C3 pose 或替換權重。

## 尚未完成的整合

B01 packet freshness、B04 independent/firmware watchdog、B05 serial ownership、
B11 calibrated base target、B13 bin yaw、B14 可通行 delivery route、B17 外部 ROS/TF bringup、
B18 standalone 模型解析、B19 部署 ports、B21 倒車安全、B22 clock adapter、
B24 Jetson environment、B25 legacy cmd_vel、B26 camera raw validity、B27 deployment identity
仍開放；B12/B16 僅完成上述明確子問題。

訓練端新提供的 v24 包已另作 [接收審查](v24_intake_2026-09-07/INTAKE_REVIEW.md)，
並封裝在 `grasp/v24/`：候選 launcher 鎖定 pair/contract/manifest，重用 v23 controller，
9 項靜態 package/release-gate/profile tests 全過。它不會自動解決上述硬體/ROS 整合缺口，
也未被切換成正式 runtime。
