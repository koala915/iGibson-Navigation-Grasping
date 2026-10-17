# PPO 重訓：GitHub 現行基線與 PR #10 審查對齊

日期：2026-10-17。這份文件優先於 PR #10 舊快照中的部署與重訓建議；不是新的控制器或訓練設定。

## 1. 現在以哪個版本為準

- 現行實機進度：[PR #9](https://github.com/koala915/iGibson-Navigation-Grasping/pull/9)，分支 `codex/approach-lidar-frame`。
- 本次實際 fetch、讀取與測試的版本：`eafba63f3ba32673738f2daa29aa8ec80686e7ab`。
- PR #9 和 PR #10 本次查詢均仍 open、未合併；`main` 和 `v23-grasp-test` 不能僅因是預設/合併目標，就視為最新實機程式。
- [PR #10 的 COMMENTED review](https://github.com/koala915/iGibson-Navigation-Grasping/pull/10#pullrequestreview-5558186386) 已讀取，不是 REQUEST_CHANGES。署名不是驗證依據，以下另外對照原始碼和測試。
- `integration/sugarbox_local/20261017_retraining_handoff/` 是歷史交接快照，**不得再當作最新部署來源或直接拿它覆寫 PR #9**。

後續工作請先 fetch 現行分支並記錄 SHA；若遠端再有新 commit，重新比較本表，保留組員的新修改。這次沒有合併 PR #9、改控制器、改機器人或啟動訓練。

## 2. 四項審查在現行程式中的情況

以下檔案均指上述 PR #9 commit，而不是 PR #10 的隔離快照。

| 項目 | 本次核對結果 | 重訓注意 |
| --- | --- | --- |
| 訓練契約模式跳過外掛保護 | 現行 `sugarbox_rl_approach_final2.py` 的契約分支會呼叫 `AntiJitterSafetyFilter.apply_contract()`；有 hard stop、forward block、side guard、倒車限幅，不套額外 turn hold | 不將歷史快照的 bypass 帶入現行部署。安全保護與學習所需的動作模型要分層，不等於所有啟發式過濾都必須加入訓練 |
| LiDAR 鏡射 | 現行 `sugarbox_lidar_geometry.py` 用 raw angle + yaw，預設 yaw=180°，前向平移 0.10 m，不是 pi-raw 鏡射；有板子/牆原始資料及方向測試 | 保留現行轉換；掛載距離仍有不同量測來源，不隨意改成歷史 0.09 m 或新提到的 0.13 m |
| 中心射線漏掉命中 | 現行 `sugarbox_training_contract.center_rays()` 在半格角度內取有效命中的最小距離；與通用 nearest-policy adapter 不同 | 以實際 enoch 契約呼叫路徑為準，不僅憑同 repo 某個函式名稱判定。點雲分箱仍是近似，不宣稱重建真正中心起始射線 |
| 感測器正側盲區 | 現行 center_rays 納入 sensor 後半圈可用回波，過濾中心 ±0.15 m 方框內的後側車身回波，policy 輸出仍為中心 ±90° | 這不是讓 policy 看整個後方。車身回波 mask 不是已量測的完整碰撞 footprint；要核對是否遮掉真障礙 |

兩個說法需要更精確：舊 PR #10 快照仍有 `CONTRACT_RAW_HARD_STOP`，問題是 bypass 了完整 `anti_jitter.apply()`，不是字面上完全沒有保護；舊訓練的 0.25/0.22 m 分支不會觸發，不代表其他近障調速、0.66 m proximity 成本或物理碰撞懲罰不存在。

## 3. 現行流程不全由 PPO 控制

現行 `sugarbox_approach_grasp.py` 預設 `--approach enoch`。PPO 在盒子離中心約 `--handoff-m=1.00` m 時，轉交 `PursuitController`，再以低速對準到 `--final-x-m=0.28` m 附近，後續另有 E1 停車量測/微調與夾取。

因此，對「繞過後提早轉回目標」的第 19 次案例，必須先檢查失敗當時：

1. 仍在 policy 分支，還是已經列印 HANDOFF/PURSUIT、由 pursuit 控制？
2. `a_raw`、提議 twist、保護後/實際指令、控制階段是否對得上？
3. 障礙是否仍出現在正確側的 policy rays，以及 raw front/side 資料？
4. 是否已啟用另一本機未提交版本的救援/切換行為？

若是 pursuit 切角，不可把這段當作純 PPO 訓練失敗的證據。可仍設計對應 PPO 課程，但需明確區分目前部署的階段責任。

## 4. GitHub 與第 19 次現場報告仍有缺口

- GitHub 目前原始紀錄到 `docs/operations/sugarbox_20261016/` 第 8–15 次；未在這個 commit 找到第 17–19 次對應的版本與完整日誌。
- 現行 `apply_contract()` hard stop 時只回傳零指令；沒有自動退 7 cm。`HARD_RECOVERY_TARGET_M=0.06` 存在於 legacy `apply()` 路徑，不可當成契約模式已用到。
- 所以 review 中的「PR #9 有 6 cm 救援」與使用者的「第 19 次退 7 cm」，都需要對應到實際 commit/分支/旗標與日誌，不能靜默拼成新基線。
- 掛載距離 0.09 / 0.10 / 約 0.13 m、車身 25x32 cm / 前20後13左右各12 cm、目標基準 +0.135 m，仍須以同一原點重新核對。這次保持 eafba63 的值，不宣告已統一。

請組員將第 17–19 次真正跑的變更、完整啟動命令、模型/VecNormalize 指紋和 raw/action/stage/odom 紀錄提交到 GitHub，再進一步做失敗重現；不用為此重新開實機。

## 5. 使用者最新決策：C 為主，不依靠救援替策略完成

這部分取代舊交接文件「先把所有選定過濾器與其狀態一起放入訓練」的優先順序。

- 第一版先模擬必需的真實 actuator 行為：速度上限、死區、控制週期、經量測的響應與延遲，以及正確車身/感測幾何。
- 不預設加入強制方向保持、換向確認、寫死繞行、臨時 waypoint 或自動退 7 cm 的救援。
- PPO 仍可自行倒退；不是禁止反向動作。
- 實機基本保護和急停保留。模擬中碰撞、保護介入、停滯不得冒充成功；不靠自動救援取得「一次繞過」成功。
- 動作變化/無效反轉獎罰採小權重，與完成率、碰撞、淨空、卡住時間一起比較，不用穩定卻不動的策略取代任務成功。
- 轉向保持只作後續 A/B 候選，不因 review 認為某個實作正確就直接設成訓練必備。若最後保留有狀態濾波，再處理 train/deploy 一致性與足夠狀態/歷史。
- 優先建立「目標在障礙後方、剛到側邊卻提早回切」的可行模擬課程及獨立測試。第 19 次需先確認控制階段和程式版本，不只加舊模型訓練步數。

導航期間維持使用者要求的巡航姿態，不恢復原本朝前/循線/向下的三段式接近視角。這不等於取消組員既有 E1 夾取姿態；重訓本身不操作手臂。

## 6. 本次驗證

- 乾淨 checkout 為 `eafba63`，未套用舊獨立快照，未更動此基線控制器。
- `python -X utf8 -m unittest discover -s tests -p 'test_sugarbox*.py'`：141 項通過。
- 使用既有 `rs_rgbd` Python 3.10.19 / NumPy 2.2.6 / OpenCV 4.12.0。沒有修改或升級 ig38。
- Python 3.8 初跑因測試使用 `ast.unparse` 而報錯；另一個既有 Python 3.10 環境缺 OpenCV。這些是環境相容性問題，未藉由改測試或裝套件掩蓋。
- 本次沒有啟動 ROS、相機、串列埠、馬達、手臂、訓練，也不宣稱新增實機驗收。

## 7. 後續入口

現行程式與原始資料：

- [現行 final2.py](https://github.com/koala915/iGibson-Navigation-Grasping/blob/eafba63f3ba32673738f2daa29aa8ec80686e7ab/integration/sugarbox_rl_approach_final2.py)
- [現行訓練契約](https://github.com/koala915/iGibson-Navigation-Grasping/blob/eafba63f3ba32673738f2daa29aa8ec80686e7ab/integration/sugarbox_training_contract.py)
- [現行 LiDAR 轉換](https://github.com/koala915/iGibson-Navigation-Grasping/blob/eafba63f3ba32673738f2daa29aa8ec80686e7ab/integration/sugarbox_lidar_geometry.py)
- [10/12 原始資料](https://github.com/koala915/iGibson-Navigation-Grasping/tree/eafba63f3ba32673738f2daa29aa8ec80686e7ab/docs/operations/sugarbox_20261012)
- [10/16 第 8–15 次紀錄](https://github.com/koala915/iGibson-Navigation-Grasping/tree/eafba63f3ba32673738f2daa29aa8ec80686e7ab/docs/operations/sugarbox_20261016)

本地已建立新的 GitHub 基線工作分支 `codex/ppo-training-followup-20261017`，起點為上述 commit；舊 `.codex-runs/cc7df8` 僅保留歷史，不再作後續現行基線。
