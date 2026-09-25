# AGENTS.md

**規格的單一事實來源是 [`CLAUDE.md`](CLAUDE.md)。先讀那一份，再回來看這裡。**

**剛接手的話，在讀規格之前先看
[`docs/handoff/HANDOFF_2026-09-20.md`](docs/handoff/HANDOFF_2026-09-20.md)** ——
現況、可以跑的指令、卡在哪、下一步，以及「機器上的檔案跟 repo 不同步」這個最容易
踩到的坑。規格告訴你系統應該長什麼樣，交接文件告訴你它現在實際長什麼樣。

這個檔案以前是 `CLAUDE.md` 的第二份副本。副本必然會漂移，而它確實漂移了：
到 2026-08-06 為止它還說主部署腳本是根目錄的 `x3plus_real_grasp.py`、
`max_delta_deg=3.0`、Stage 1 的觸發條件是 `dist < 5cm` —— 這三項全都是 v17 的
說法，照著做會把 v17 的慣例套到 v21 的程式上。所以現在這裡不再複述規格，
只留指標與幾條給代理人的工作守則。

---

## 進來先看哪幾份

| 想知道什麼 | 看這裡 |
|-----------|--------|
| 專案在做什麼、怎麼跑起來 | [`README.md`](README.md) |
| 完整規格：檔案結構、部署指令、關節映射、觀測空間、三段式控制 | [`CLAUDE.md`](CLAUDE.md) |
| 要改的程式碼在哪一支 | [`INDEX.md`](INDEX.md) |
| 這個坑是不是有人踩過了 | [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md) |
| 之前做到哪、實機測過什麼 | [`progress.md`](progress.md) |

---

## 動手前必須知道的四件事

1. **現行夾取流程是 `grasp/v21/`，不是根目錄那支。**
   根目錄的 `x3plus_real_grasp.py` 是 v17 備援，無人匯入，只能手動執行。

2. **v21 與 v17 的權重絕對不可混用。**
   v21 是 incremental（`desired = current + action × 0.08 rad`），v17 是 absolute。
   兩者 shape 都是 28D/6D，**任何 shape 檢查都抓不到**，混用手臂會直接暴走。
   以 `--contract obs_28_incremental` 與 `grasp/v21/manifest.json` 的 sha256 為準。

3. **改完要跑回歸測試，而且不需要硬體。**
   ```bash
   python3 grasp/v21/test_deploy_controller.py    # 須 138 全過
   python3 grasp/v21/test_servo_read.py           # 須  37 全過
   python3 grasp/v21/test_deploy_floor_guard.py   # 須 641 全過
   python3 ui/test_server.py                      # 須  48 全過
   python3 integration/mission_fsm.py --selftest
   ```
   CI（`.github/workflows/tests.yml`）會跑完整清單。

4. **這是會自己移動的機器人。** 沒有 `--real` 一律只印指令不送伺服機；
   要拿掉任何一道安全閘之前，先去 `CLAUDE.md` 的「注意事項」確認那道閘擋的是什麼。

---

## 收到新的 `.py` 檔時的檢查順序

1. 確認用的是 `Rosmaster_Lib`，不是舊的 `Arm_Lib`
2. 確認觀測空間 28D、動作空間 6D
3. 確認 `VecNormalize` 載入時 `training=False`、`norm_reward=False`
4. 確認契約是 `obs_28_incremental`（v21）而不是 `obs_28_absolute`（v17）
5. 直接修改並回報差異

---

## 文件慣例

- 工作文件收在 `docs/` 底下（`calibration` / `operations` / `handoff` / `planning`），
  引用時要帶路徑。根目錄只留 README、CLAUDE、AGENTS、INDEX、TROUBLESHOOTING、progress。
- ⚠️ `grasp/v21/HANDOFF.md` 是 v21 自己的交接文件，**不是** `docs/handoff/HANDOFF.md`。
- 規格改動寫進 `CLAUDE.md`，**不要**再複製一份到這裡 —— 這份檔案就是那樣壞掉的。
