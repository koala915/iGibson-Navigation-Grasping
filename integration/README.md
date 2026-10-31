# X3Plus 視覺辨識 → 夾取 整合說明

把 YOLOv11 垃圾辨識接到 PPO 夾取部署。提供三種模式：

- **模式 A（推薦・自走全流程）** `integration/vision_grasp_pipeline.py`
  單一程式統一控制：辨識 → 底盤導航靠近 → 到範圍 → 手臂夾取 → 失敗重試（≤3）。
- **模式 B（分離式・除錯用）** `integration/vision_grasp_bridge.py`
  只做「辨識→TCP 5555 送座標+高度/寬度」，底盤導航需另外處理。

```
模式 A：後相機/手臂相機 ─YOLO─▶ 決策 ─set_car_motion─▶ 麥輪靠近
        ─到 handoff 距離─▶ 鎖定 {x,y,z,w} ─obj_provider─▶ PPO 手臂夾取 ─驗證/重試
模式 B：手臂相機 ─YOLO─▶ {x,y,z,w,height} ─TCP 5555─▶ x3plus_real_grasp.py(--socket)
```

> ### ★ 2026-08-01：手臂端已從 v17 換成 v21
> **模式 A** 與 **模式 C**（`nav_rl_grasp_pipeline.py`）都改接 `grasp/v21/`，
> 入口是 `vision_grasp_pipeline._load_grasp_module()`（模式 C 直接呼叫 `vgp._load_grasp_module()`
> 所以自動跟著換）。理由：v21 是唯一在實機上夾取成功過的版本（2026-07-31，3cm 物體）。
>
> 隨之而來的介面差異（v21 沒有的東西）：
> - `DeployConfig.grip_width_control` / `grip_max_object_width_m` / `grip_min_close_deg` → **已移除**。
>   v21 改用接觸偵測 + hold，不再靠寬度算閉合角。`DeployConfig` 是 dataclass，
>   設一個不存在的欄位**不會報錯**，只會產生一個沒人讀的死屬性 —— 所以移植必須逐欄位對齊。
> - `ServoController.move_to_home()` → 改用 `controller.move_home()`（過 FloorGuard、等編碼器確認到位、回傳 bool）。
> - `obj_provider` 的第二個元素：v17 是**寬度**，v21 是**高度（公尺）**。
>   兩支 pipeline 都沒有高度量測，所以一律傳 `None`，讓 v21 用它自己的對稱物體估計。
>   寬度仍保留，但只當「太寬夾不起來」的閘。
>
> **模式 B 同日也移到 v21**（橋接端送 superset payload，兩邊都能接）；
> 過程中補上了 v21 socket 路徑原本缺的 payload 驗證、過期偵測與 latch。

> ### ⚠️ 2026-08-01：相機外參綁在姿勢上，v21 換了姿勢
> `H_ARM = 0.332` / `THETA_ARM = 36.40` 是 2026-07-08 在 **v17 的 nav home**
> `(90, 140, 0, 0, 90, 30)` 量的。手臂相機裝在 `arm_link4` 會跟著手臂動，
> 這兩個是**安裝幾何**，換姿勢就失效（相機內參與畸變不受影響）。
>
> v21 的 `home_deg` 是 **C3 夾取姿態** `(90, 67.08, 9.79, 9.79, 90, 30)`——
> URDF FK 算出相機低了 **11.7cm**、俯角從 **40° 變 86.7°**。模式 A/C 改接 v21 之後，
> `move_home()` 會把手臂帶到 C3 才偵測，用的卻還是舊姿勢的距離模型。
> **距離模型在任何姿勢都會回傳一個看似合理的數字，下游沒有任何一關能發現。**
>
> 現在 `check_arm_cam_pose()` 會在 `--real` 時擋下來（除非 `--i-confirm-arm-cam-pose`）。
> 正解是**在 C3 重量這兩個外參**（CALIBRATION_PLAN Phase 1/2，內參不用重做）——
> C3 本來就是為了視野挑的（近垂直俯視、地面可視 x 0.196–0.330m，正好涵蓋可夾範圍；
> v17 的夾取姿態只看得到前方 0–7cm）。

專案分成三個資料夾：

| 資料夾 | 角色 | 說明 |
|--------|------|------|
| `grasp/` | 夾取 | PPO 夾取部署。★現行 = `grasp/v21/`（接觸式夾持、`obj_provider` 注入）；`grasp/` 根目錄那支 v17 保留備援 |
| `detection/` | 辨識 | YOLOv11 模型與相機辨識/校正/除錯腳本 |
| `integration/` | 整合 | 自走 pipeline（模式 A）＋ TCP 橋接（模式 B） |

---

## 0. 模式 A：自走全流程（vision_grasp_pipeline.py）

> 單一程式持有 Rosmaster，同時驅動麥輪與手臂。**執行前確認 Jetson 沒有跑 port 7000
> motor server 或 ROS 底盤 driver**，否則會搶序列埠。

```bash
# 純邏輯自測（不需相機/硬體/torch，可在開發機跑）
python3 integration/vision_grasp_pipeline.py --selftest

# 乾跑（不驅動硬體，印出狀態機與會送出的 set_car_motion；需相機串流）
python3 integration/vision_grasp_pipeline.py

# 正式自走夾取
python3 integration/vision_grasp_pipeline.py --real --show \
  --i-confirm-camera-frame
```

流程對應使用者 5 步：1 辨識 → 2 底盤前進 → 3 到 handoff 距離(預設 0.24m) →
4 手臂依鎖定座標+寬度夾取 → 5 失敗(原位置仍辨識到)則退回重試，最多 `--max-retries`(預設 3) 次。

主要參數：`--real`、`--show`、`--max-retries`、`--max-steps`、`--port`、`--handoff-dist`、`--selftest`。
座標/相機校正常數在檔案頂部（`CAM_TO_BASE_X/Y`、`SIGN_Y`、`OBJ_Z_FIXED`、`THETA_ARM/H_ARM`、`KX/KZ` 等）。

> ⚠️ 導航決策/幾何由 `detection/rear_nav/rear_to_arm_blind_handoff.py` 移植，但 actuator 由
> port 7000 motor server 改成 `set_car_motion`，速度/轉向門檻可能需**重新微調**；
> 且 `set_car_motion` 的 v_z 正負(左右轉)需實機確認。

---

## 模式 B：TCP 橋接（vision_grasp_bridge.py）

**這個模式在做什麼**：把「辨識」和「夾取」拆成兩個獨立行程，用 TCP 5555 串起來。
辨識端跑 YOLO、把 bbox 換算成 `{x, y, z, w, height}` 送出；夾取端只負責夾。
拆開的好處是可以單獨驗證任一端 —— 手動送一筆假座標測夾取、或先只看辨識算出來的
座標合不合理而不動手臂。**底盤導航不在模式 B 範圍內**，要自己處理（這是它跟模式 A 的差別）。

> **2026-08-01：模式 B 已支援 v21。**
> 橋接端送 superset payload，v17 讀 `w` 忽略 `height`、v21 讀 `height` 忽略 `w`，
> 同一支橋接程式兩邊都能接。同日補強了 v21 的 socket 路徑（原本缺 payload 驗證、
> 缺過期偵測、缺 latch，NaN 座標會直接進 policy）。

> **2026-08-02：手臂相機幾何全面改寫，模式 B 是主要受益者。**
> v21 從 **C3** 起始姿勢偵測，但所有相機常數描述的都是 v17 的 nav home。URDF FK 顯示
> C3 的相機低 11.7 cm、幾乎垂直朝下（以 base +X 量 93.34°，扣掉安裝誤差約 **89.74°**），
> nav home 則是 36.4° 且明顯朝前。由此爆出三個**會靜默夾錯地方**的錯誤：
>
> 1. `if dist > 0` 過濾掉 C3 畫面 64% 的列（地面距離在相機正下方之後是負值，這是正確的）。
>    在 `verify_grasp()` 裡更會反轉結論，把夾失敗判成成功。
> 2. 橫向偏移與寬度用了地面距離當投影深度，正確要用**光軸深度**。nav home 差 22–39%，
>    C3 直接塌成 0（2.3 cm 的 sugarbox 會量成 0.3 cm）。
> 3. `CAM_TO_BASE_X = 0.0`。C3 的地面距離只有 ±4 cm，`cam_x` 就是答案，補 0 等於把目標
>    放在手臂底座上。
>
> 根因是 bridge / pipeline / `detection/arm_cam.py` **各留一份 H 與 θ**——這正是
> 2026-08-01 的姿勢防護只補到 pipeline 的原因。現在統一到
> **`integration/arm_cam_geometry.py`**（內參 + 姿勢登錄表 + 正確的地面模型）。
>
> **跨行程姿勢驗證**：bridge 知道自己的幾何屬於哪個姿勢但讀不到伺服機；夾取端握有
> 序列埠卻不知道座標怎麼算出來的。任一端單獨都抓不到不一致。現在 payload 帶
> `cam_pose` 戳記，夾取端在 **latch 當下**拿它跟編碼器實際讀值對照——那是兩邊事實
> 唯一同時存在的時刻。不符在 `--real` 下直接中止。
>
> ⚠ **C3 的外參目前是 URDF 推算的預測值，不是實測。** bridge 會拒絕送出，除非跑完
> `docs/calibration/CALIBRATION_PLAN.md` Phase 2，或明示 `--i-accept-predicted-extrinsics`。
> 上機步驟見 **`docs/operations/MODE_B_TEST_PLAN.md`**。

## 1. 端到端開啟流程

> 三步都在 Jetson 上、`grasp_venv` 已啟用的前提下執行。

1. **啟動手臂相機串流**（Jetson，ROS web_video_server，預設 8080）
   - 確認 `http://127.0.0.1:8080/stream?topic=/arm_cam/image_raw` 可開。

2. **終端機 A — 夾取端**（監聽 TCP 5555）
   ```bash
   cd grasp/v21
   python3 x3plus_real_grasp.py --real --socket \
     --latch-obj --i-confirm-external-frame --unlock-candidate-real \
     --model models/candidate_v21_seed816_ckpt550000.zip \
     --vecnorm models/candidate_v21_seed816_ckpt550000_vec.pkl \
     --contract obs_28_incremental
   ```
   - `--socket`：從 5555 接收 `{x,y,z,height}`。
   - `--latch-obj`：**`--real --socket` 強制要求**。手臂相機（URDF `mono_link`）裝在 `arm_link4`，
     會隨手臂移動，`arm_cam` 的固定高度/俯仰角只在 home 姿勢成立。開啟後會在 home 姿勢
     **擷取一次** obj 座標+高度並凍結整輪。等候逾時（`latch_wait_sec`，預設 5 秒）時：
     dry-run 印警告後退回預設座標；`--real` 直接 raise，不會朝沒人確認過的座標移動。
   - `--i-confirm-external-frame`：**`--real --socket` 強制要求**。確認送來的 XYZ 已在
     PPO/URDF base_link 座標系。座標系錯了下游偵測不到，只會很有把握地夾錯地方。
   - 兩個旗標都是「確認」而非「開關」——不改任何幾何，只是不讓失敗變成無聲。
     兩者都在載入模型、開序列埠**之前**檢查，被擋下不會半動手臂。
   - 第一次務必先不加 `--real` 做 dry-run，目視確認角度合理。

   <details><summary>備援：v17 夾取端</summary>

   ```bash
   cd grasp
   python3 x3plus_real_grasp.py --real --socket --width-grip --latch-obj \
     --i-confirm-external-frame
   ```
   `--width-grip`：依寬度決定夾爪閉合角（v21 已無此概念，改用接觸偵測）。
   </details>

3. **終端機 B — 辨識+整合端**
   ```bash
   python3 integration/vision_grasp_bridge.py --host 127.0.0.1 --show      --class-height eraser-detect=0.03 --class-z eraser-detect=0.015
   ```
   - `--show`：顯示標註視窗（按 `q` 關閉）。
   - `--class-height NAME=公尺`：物體**全高**（頂到地）。v21 用它算 `wrist_z_offset`。
     **不能從 bbox 量出來**——相機斜著往下看，bbox 像素高度混了高度與深度——所以按類別填。
     不填就不送 `height`，v21 退回它自己的對稱物體估計（`2×(質心z−地面z)`）。v17 忽略此欄。
   - `--class-z NAME=公尺`：物體**質心** z。跟高度是兩個量，只有「對稱且貼地」時才有 `高度 = 2×質心z`。
   - 類別名打錯會印 WARN（列出模型實際有的類別）。沒有這個警告的話，查不到就靜靜落回預設值，
     整輪看起來正常卻夾在錯的高度。
   - 正式跑之前，先用 `--once` 送一筆、核對印出的 x/y/z 是否合理：
     ```bash
     python3 integration/vision_grasp_bridge.py --host 127.0.0.1 --once --show        --class-height eraser-detect=0.03 --class-z eraser-detect=0.015
     ```

---

## 2. 各檔案用途

### `integration/`（整合，本次新增）
| 檔案 | 用途 |
|------|------|
| `vision_grasp_pipeline.py` | ★模式 A 主程式。統一控制：雙相機導航(set_car_motion) → handoff → PPO 夾取(obj_provider) → 驗證/重試。含 `--selftest` 純邏輯自測。 |
| `vision_grasp_bridge.py` | 模式 B。讀 `detection/models/best.pt` + 手臂相機，將 bbox 換算成 `{x,y,z,w,height}` 並透過 TCP 5555 送給夾取端（每筆開新連線、送完即關，符合夾取端 `DetectionReceiver` 行為）。payload 是 superset：v17 讀 `w`、v21 讀 `height`。 |
| `arm_cam_geometry.py` | ★**手臂相機幾何單一來源**（2026-08-02）。內參、畸變、按姿勢登錄的外參（`V17_NAV_HOME` / `V21_C3_GRASP_HOME`）、正確處理近垂直的地面模型（有號距離 + 光軸深度）、`cam_pose` 戳記協定。有 `python integration/arm_cam_geometry.py` 自測。 |
| `smoke_mode_b.py` | ★**上機前必跑**。在桌上把整條模式 B 雙行程路徑跑一遍（dry-run）：正常 latch、偵測過期、完全沒偵測、姿勢戳記不符、超出可及範圍、缺 height 共 6 個情境。 |
| `capture_arm_cam_obs.py` | ★校正資料採集。擺物體→輸入 base 座標→按 Enter，自動取 20 幀中位數像素、即時存檔可續做、自動留保留點、涵蓋不足會提示。`--simulate` 可在桌上演練整套流程（不用機器）。 |
| `solve_arm_cam_extrinsics.py` | ★由 base-frame 實擺點一次解出 θ / cam_x / cam_y / sign_y（C3 專用；舊的兩階段解法在近垂直姿勢不成立）。有 `--selftest`。 |
| `README.md` | 本說明文件。 |

`vision_grasp_bridge.py` 主要參數：

| 參數 | 預設 | 說明 |
|------|------|------|
| `--host` / `--port` | `127.0.0.1` / `5555` | 夾取端位址 |
| `--stream` | arm_cam HTTP 串流 | 相機來源；給數字（如 `0`）則當 USB webcam 索引 |
| `--conf` | `0.3` | YOLO 信心閾值 |
| `--rate` | `0.3` | 每筆送出最小間隔（秒） |
| `--once` | 關 | 送出第一筆就結束（**僅限測試**）。夾取端要先走到 C3 才 latch，實機要幾秒，那筆早就過期了 |
| `--show` | 關 | 顯示標註視窗 |
| `--pose` | `v21_c3_grasp_home` | 用哪一組姿勢外參（見 `arm_cam_geometry.POSES`） |
| `--cam-theta` / `--cam-h` | 取自 `--pose` | 實測的光軸俯角（度，**朝 base +X 量**）與相機離地高。C3 約 **89.7°**，不是 36° |
| `--cam-x` / `--cam-y` | 取自 `--pose` | 相機地面投影點在 base 座標（**需校正**）。C3 預測 +0.2762 / −0.0056。⚠ 這是 **policy 座標**（URDF 載入時有 `URDF_TO_TRAINING_FRAME` 偏移），比 `verify_camera_grasp_frame.py` 印的原始 base_link 值多 19.9 mm |
| `--i-accept-predicted-extrinsics` | 關 | 外參還是 URDF 預測值時仍允許送出。**手臂會照這些數字動** |
| `--dry-run` | 關 | 只算不送，校正時讀數字用 |
| `--max-width` | `0.06` | 超過此寬度直接不送（夾爪張不開） |
| `--obj-z` | `0.02` | 物體**質心** z（公尺），不是高度 |
| `--class-z` | 無 | 按類別覆寫質心 z，`NAME=公尺`，可重複 |
| `--class-height` | 無 | 按類別指定物體**全高**（頂到地），`NAME=公尺`，可重複。v21 用來算 `wrist_z_offset`；v17 忽略。不給就不送 `height` |
| `--sign-y` | `1.0` | 相機左右偏移 → 夾取 +Y 的方向（`+1` 或 `-1`，**需校正**） |

### `detection/`（辨識）
| 檔案 | 主線? | 用途 |
|------|:----:|------|
| `arm_cam.py` | ★ | 手臂相機 + YOLO，把 bbox 換算成前向距離與左右偏移（無 ROS）。`vision_grasp_bridge.py` 的幾何即源自此檔。 |
| `arm_center_setmotor.py` |  | 手臂相機對中（馬達 socket 狀態機，使用 box 寬度）。 |
| `nav_arm_cam.py` |  | 手臂相機 + ROS `cmd_vel` 驅動底盤靠近。 |
| `models/best.pt` | ★ | **正式模型**。2026-08-01 起為單類別 `sugarbox`（藍色盒子，高 6.5cm）。前兩代備份在同目錄 `best_eraser_detect_train11.pt.bak` / `best_trash_identify_train5.pt.bak`。 |
| `models/yolo11n.pt` |  | YOLOv11 基底模型（備用）。 |
| `models/data.yaml` | ★ | 類別定義。 |
| `models/README.roboflow.txt` |  | Roboflow 資料集來源說明。 |
| `calibration/` |  | 相機/底盤校正腳本與量測資料（見下）。 |
| `debug_tools/` |  | 測試/擷取/雜項腳本（見下）。 |
| `rear_nav/` |  | 只剩 `rear_to_arm_blind_handoff.py`（模式 A 導航幾何的來源，保留供溯源）。其餘 8 支舊實驗已於 2026-08-01 刪除，需要時從 git 歷史取回。 |

`detection/calibration/`：
`calibrate_arm_camera_theta.py`（★校手臂相機俯仰角 `FIXED_THETA`）、`calibrate_rearcam.py`、
`calibrate_mecanum_setmotor.py`、`calibrate_vy_only.py`、`calibrate_wx_only.py`，
以及量測資料 `*_calibration_points.csv`、`action_speed_calibration_*`、`wz_90deg_calibration_result.json`、`Vx.csv`、`Wz.csv`。

`detection/debug_tools/`：
`yolo_test.py`（純 YOLO 檢視器，驗證模型與類別）、`detect_video.py`（對影片跑 YOLO 存檔）、
`multiple_ball_detect.py`、`armcam_photoshot.py`、`rear_cam_autophoto.py`、`snapshot_sim.py`、
`check_rl_model.py`、`train_yolo.py`、`test.py`。

### `grasp/`（夾取）
| 檔案 | 用途 |
|------|------|
| `v21/x3plus_real_grasp.py` | ★**現行**主部署腳本（模式 A/C 匯入的就是這支）。incremental action、`gripper_center` TCP、接觸偵測夾持、預防式 FloorGuard。 |
| `v21/models/` | ★現行 PPO 權重 + VecNormalize。**不可與 v17 混搭**（incremental vs absolute，shape 檢查抓不到）。 |
| `v21/manifest.json` | 權重 sha256、契約、硬體 gate、變更紀錄（單一事實來源）。 |
| `v21/jetson_verify.sh` | 上機前一鍵前置檢查（135/37/641、`wrist_z_offset=0.0564`、安全閘 exit 3）。 |
| `v21/bus_probe.py` / `pose_check.py` | 唯讀診斷：半雙工伺服匯流排、姿態/FK 核對。 |
| `x3plus_real_grasp.py`（根目錄） | v17 舊版，**保留備援**。有 `--width-grip` / `--latch-obj`。模式 B 自 2026-08-01 起兩邊都能接（橋接送 superset payload），文件中的模式 B 指令用的是 v21。 |
| `trained_6d_models_v17/` | v17 的 PPO 模型 `.zip` + VecNormalize `.pkl`。 |
| `x3plus/` | FK 用 URDF 與 meshes（v17/v21 共用）。 |
| `fk_test.py` / `servo_test.py` / `workspace_scan.py` / `joint_direction_calibration.py` | 測試/校正小工具。 |
| `Rosmaster_Lib_reference.py` | 硬體驅動 API 參考（實機需 `cp` 真正的 `Rosmaster_Lib`）。 |
| `requirements_jetson.txt` | 夾取端相依套件。 |

---

## 3. 模型說明
- 正式使用：`detection/models/best.pt`。**2026-07-31 起改為單類別 `eraser-detect`**
  （來源：另一個 Roboflow 專案 `eraser_detect.v1i.yolov11`，訓練跑 `train11`，150 epochs，
  precision 1.0 / recall 0.995 / mAP50 0.997）。原本的雙類別 `bottle-cap`/`paper-ball`
  （`train5`）備份在同目錄 `best_trash_identify_train5.pt.bak`，需要時把它改名回 `best.pt`
  即可換回。**類別從 2 個變成 1 個，任何用 `--class-z bottle-cap=...`/`paper-ball=...`
  指定物高的指令都要改成 `--class-z eraser-detect=<高度>`，否則會安靜落回預設高度。**
- 其他複製進來的腳本原本指向 `train6`/`train7` 或外部 `v8i(0517)` 模型（**未隨附**），
  若要跑那些腳本需自行修改其 `MODEL_PATH` 或補上對應權重。
- 已修正為相對路徑、可直接執行：`detection/arm_cam.py`、`detection/debug_tools/detect_video.py`。
- 驗證模型：`python -c "from ultralytics import YOLO; print(YOLO('detection/models/best.pt').names)"`
  → 應印出 `{0: 'eraser-detect'}`。

---

## 4. 校正清單（接實機前務必做）

1. **相機俯仰角 `FIXED_THETA` / 內參**：用 `detection/calibration/calibrate_arm_camera_theta.py`
   以已知距離反推，更新 `arm_cam.py` 與 `integration/vision_grasp_bridge.py` 頂部常數。
2. **相機↔手臂基座偏移 `--cam-x` / `--cam-y`**：Phase 3 已於 2026-07-16
   以正前、左、右、遠方四點解得 `0.1639 / 0.0331 m`。
3. **左右方向 `--sign-y`**：Phase 3 已確認為 `-1`；物理右側對應 base 負 Y。
4. **物體高度 `--obj-z`**：依實際物體擺放高度設定（地面物約 0.02）。
5. **寬度→夾爪角**：`grasp/x3plus_real_grasp.py` 的 `DeployConfig` 內
   `grip_max_object_width_m`（最寬可夾物寬度）與 `grip_min_close_deg`（對應的閉合角）依夾爪實測調整。
   公式：`close_deg = 180 − (w / max_w) × (180 − min_close)`（窄物趨近全閉 180°，寬物提早停）。

---

## 5. 安全提醒
- 首次接伺服機前，夾取端先跑 dry-run（不加 `--real`），目視確認角度合理。
- 橋接端先用 `--once` 確認座標/寬度數值正確，再連續送。
- 夾取端每步限動 `max_delta_deg`：**v21 是 8°**，根目錄的 v17 備援才是 3°。
  限速是從**上一次的指令值**走而非編碼器，所以夾爪被物體擋住時指令仍會前進 ——
  v21 由接觸偵測擋下，這也是它敢用 8° 的原因。
