# Sugarbox PPO 重訓交接：固定巡航姿態

> **歷史參考，非部署版本。** 已依 [PR #10 review](https://github.com/koala915/iGibson-Navigation-Grasping/pull/10#pullrequestreview-5558186386) 核對現行來源：PR #9 的 `codex/approach-lidar-frame`，commit `eafba63f3ba32673738f2daa29aa8ec80686e7ab`。請先讀 [最新基線與修正決策](../../../docs/handoff/SUGARBOX_PPO_CURRENT_BASELINE_2026-10-17.md)（ZIP 使用者請到 PR #10 的 docs/handoff 讀此檔），不要將本包覆寫到現行程式。舊鏡射/射線取樣/側邊盲區和旗標限制均只保留作歷史證據，不是建議的訓練輸入。

> **最新方案優先：** 先訓練 PPO 自己繞障，模擬真實 actuator 與基本安全底線；不預設加入強制方向保持、換向確認或自動後退救援。舊文件的「共用所有選定過濾器」不是當前已定案要求。新 observation 維度也未定案，不先擴充。需要的話，後續另作 A/B。

更新：2026-10-17。使用者已取消接近過程的三段式手臂視角切換，全程固定巡航／循線／行駛姿態 `(90,140,0,0,90,30)`。本包取代同日較早的 21 檔版本。

## 為什麼不只有 final2.py

核心仍是 `integration/sugarbox_rl_approach_final2.py`，但目前它有兩個本機純邏輯依賴：

1. `integration/sugarbox_training_contract.py`：55D 觀測、車體中心換算、舊訓練動作處理。
2. `integration/sugarbox_navigation_handoff.py`：停車交接條件與巡航姿態常數，不含任何手臂動作。

**訓練端先看這三支即可，不需要搬入一整套手臂控制。** 原本手臂三段式控制、校正工具、夾取編排和它們的測試不再列入交接包。

`diagnostics/run_approach_camera_demo.py` 只是選讀的實機錄影入口；現在只初始化巡航姿態，交接時只切鏡頭，不改手臂角度、不自動夾取。它不再依賴多段手臂控制模組。不要在訓練機直接執行。

## 交接位置與閱讀順序

群組 GitHub（部署端）：<https://github.com/koala915/iGibson-Navigation-Grasping>。
實際訓練端：<https://github.com/koala915/igibson_x3_test>。

先讀 `PPO_RETRAINING_HANDOFF_2026-10-17.md`，再看 `contract_alignment.json` 和上面的三支導航程式。
`artifact_fingerprints.json` 是現用模型/VecNormalize 指紋；`manifest.json` 是包內檔案雜湊。
`reference/doorway_ft_training_config.json` 是舊訓練參數參考，不是已完成的新訓練設定。

提交時將本包放在獨立快照目錄，不覆寫部署主線的 `integration/`。團隊指定 PR base 是 `v23-grasp-test`，不直接合併 main。

## 必須對齊的重點

- 舊策略：55 維 float32 observation、2 維 action、48 束 ±90° LiDAR。新過濾器狀態的 observation 最終維度未定案，不可直接套舊 VecNormalize。
- 訓練與實機共用選定過濾器，同時計算原始動作抽動與不必要介入成本，不能只靠濾波後平順宣告成功。
- 現有 training-contract 模式因 OR 旗標仍跳過 `anti_jitter.apply()`；此交接沒有趁機修改 PPO 過濾或停車條件。
- 現用人工 delay=0，舊訓練是 2 step；真實馬達/網路延遲需量測。速度映射、控制頻率、LiDAR order/下限、座標基準也要一起核對。
- 車寬約 25 cm、長約 32 cm、LiDAR 中心前方約 9 cm；目標前緣到中心的 13.5 cm 與半車長 16 cm 不同，先核對參考點。
- 今日模型離線核對為 55D/2D、ZIP metadata `num_timesteps=400224`、SB3 2.2.1；與舊文件的 398304 checkpoint 敘述不同，保留差異待查。
- 固定姿態後，原本非巡航角度的影像校正不能沿用；最終可夾位置仍未校準。本包沒有訓練新權重，也沒有實機驗收。

## 請組員準備的資料

- 訓練 checkout 的 branch、commit、dirty diff；不要清掉現有修改。
- 實際 `navigation_env_room.py`、`navigation_train_new.py`、JSON config、wrapper、訓練啟動命令與評估程式。
- Python/iGibson/SB3/Gym/PyTorch/CUDA 版本、URDF/mesh、`Rs_int` 資產路徑及成功載入證據，不先升級環境。
- 舊模型與同一 checkpoint 的 VecNormalize 配對檔、SHA256、訓練/評估 seeds 和歷史結果。
- 完整部署重現時再補目前 Jetson 的 `feedback_odom.py`、校正版本、馬達環境覆寫和相機校正；本次未連線抓取，也不拿舊副本冒充。

## 內容範圍與安全檢查

本包沒有模型/VecNormalize 二進位檔、YOLO/SAM、照片、影片、原始 log、憑證或場景資產。部署 repo 的 `integration/nav_best_model/` 可先找舊模型，但需核對兩個雜湊，不只看檔名。

在已有 NumPy 的相容 Python 環境，只執行離線子集即可：

```text
python -X utf8 -m unittest discover -s tests -v
```

它不是全部本機回歸測試，更不代表實機安全。請先回報契約差異、過濾器方案、觀測布局、權重相容方式和 A/B 計畫；使用者另行確認後才開始訓練或操作機器人。
