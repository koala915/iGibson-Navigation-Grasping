# G2 運作架構：一個常駐程式持有序列埠，對外提供兩個介面（2026-09-28 定案）

**決定**：Jetson 上只留**一個**常駐程式持有 `/dev/myserial`。它同時提供
**TCP 7000 速度指令**（格式跟現在的馬達服務完全一樣）與 **`graspctl` 夾取指令**，
並在程式內部強制「輪子與手臂不同時動」。導航、接近、夾取都變成它的用戶端。

負責人：G2 本體 koala915；導航端 112303577ncu；接近端 enoch20050427。

---

## 為什麼這樣定

**1. TCP 7000 已經是接近端的介面。** enoch 的 `sugarbox_rl_approach_final2.py` 經 7000
送速度指令，對面是 `integration/sugarbox_rl_motor_server.py`（在 112303577ncu 的 PR 裡）。
協定是一行一個 JSON：`{"action":"velocity","vx":…,"wz":…}` 或 `{"action":"stop"}`；
**沒寫 `action` 等於 stop**，其他 action 一律停車並記錄；0.5 秒沒收到指令就停車。
**讓持有序列埠的程式講同一個協定，接近程式一行都不用改。**

⚠ 7000 上其實有**兩種格式**（2026-09-28 核對程式碼才發現，初版寫錯）。`detection/` 底下的
舊工具（`arm_center_setmotor.py`、`rear_nav/rear_to_arm_blind_handoff.py`、`calibration/`、
`debug_tools/`）送的是 `{"action":"forward"/"turn_left"/…,"speed":N}`，對面是
`x3plus-navigation` 跑的 `ai_motor_server_P0.py`。這支**只存在 Jetson 上、不在 repo**，
格式的完整定義讀不到。**決定：常駐服務只講速度格式**；舊格式工具照舊在維修模式
（`x3plus-navigation` + P0）下使用，與常駐服務互斥。等 P0 收進 repo、確認有工具還需要，
再決定要不要加。

另一類是 repo 內的 `mission_pipeline.py`、`nav_rl_grasp_pipeline.py`（模式 C）、
`vision_grasp_pipeline.py`（模式 A）：它們在程式內直接 `set_car_motion` 控車，並要求 7000
馬達服務**關閉**。它們在新架構裡的角色見「實作順序」第 6 步。

**2. 夾取常駐服務已經在機器上驗證過了。** `grasp/v23/grasp_service.py` 已經常駐持有
序列埠，所有安全閘在啟動時跑一次，實測夾取 7.2 秒、放下 4.6 秒、記憶體約 490 MB，
開機由 systemd 帶起。在它上面加底盤功能，比從頭寫一個新的主程式風險低。

**3. 跟原計畫的差異。** `GRASP_LATENCY_PLAN_2026-09-21.md` 的 G2 原本寫「把 v23 接進
`mission_pipeline.py`，不另外啟動 serial service」。那樣做的話，enoch 的接近程式和所有
7000 工具都得改寫成 `mission_pipeline` 內部的一段迴圈。**目標不變**（單一持有者、切換
不重啟、模型常駐），只是持有者改成常駐服務，任務流程改成用戶端。

---

## 架構

```mermaid
flowchart LR
  subgraph WIN["Windows 筆電"]
    APP["enoch：接近<br/>YOLO + SAM2 + PPO"]
  end
  subgraph JET["Jetson Nano"]
    D["常駐服務（唯一的序列埠持有者）<br/>v23 夾取控制器 + 底盤"]
    V["grasp-vision<br/>手臂相機 video0 → YOLO"]
    ROS["ROS：TG30 /scan、AMCL、rosbridge :9090"]
    CAM["後相機 video1 → NVENC 串流"]
    NAV["其他速度格式用戶端<br/>（導航測試工具）"]
  end
  BOARD["Rosmaster 控制板<br/>/dev/myserial"]

  APP -- "TCP 7000 速度" --> D
  NAV -- "TCP 7000 速度" --> D
  APP -. "rosbridge：/scan、odom" .-> ROS
  CAM -. "RTP/UDP" .-> APP
  V -- "TCP 5555 物體座標" --> D
  D -- "/odom_setmotor + TF（經 rosbridge）" --> ROS
  D == "唯一連線" ==> BOARD
```

`graspctl`（`/tmp/grasp_service.sock`）是任務流程或操作員下夾取／放下指令的入口。

---

## 介面

| 介面 | 內容 | 狀態 |
|------|------|------|
| TCP 7000 | 一行一個 JSON；`action: velocity` + `vx`／`wz`，或 `action: stop`；缺 action 視為 stop；0.5 s 看門狗；速度上限與死區沿用 `sugarbox_rl_motor_server.py` | **沿用，不改格式**；舊 `action/speed` 格式不收（見上） |
| `graspctl` socket | `grasp`／`release`／`home`／`status`／`quit` | 沿用；`status` 加上底盤狀態 |
| `/odom_setmotor` + `odom→base_footprint` TF | `FeedbackOdomReader`（2026-09-24 校正：linear 0.966、angular 0.986）算出，20 Hz | **已改由常駐服務發**（第 3 步） |
| TCP 5555 | 視覺服務送物體座標 | 沿用 |

odom 用 rosbridge 發（`integration/ros_io.py` 的做法）：常駐服務跑在 Python 3.8 venv，
不能 import rospy。`mission_pipeline.py` 已經用同一種方式發 odom 與 TF。

---

## 程式內強制的安全規則

**R1 手臂動作前，底盤必須已停止 ≥ 0.5 秒。** 否則 `grasp`／`release`／`home` 直接回
`{"ok": false, "reason": "chassis_moving"}`，手臂不動。「停止」指最後一次**非零**輪速寫入
已過 0.5 秒；用戶端持續送 `stop` 或 `vx=wz=0` 不算在動，連線可以一直開著。

**R2 手臂動作期間，底盤指令一律丟棄。** 開始動手臂前先送一次停車，整段手臂動作持有
硬體鎖；這段時間 7000 收到的速度指令不寫入控制板，只記錄並回報。

**R3 所有寫入控制板的動作共用一把硬體鎖。** 目前 v23 控制器沒有保護序列埠寫入的鎖，
輪子與手臂分在兩個執行緒時，封包可能交錯。odom 讀取來自 Rosmaster_Lib 的接收執行緒
快取，不寫入，不需要鎖。

**R4 看門狗與速度上限照舊**：0.5 秒沒指令停車；`RL_MOTOR_LIMIT = 30`。

**R5 啟動前的序列埠檢查照舊**：`deploy/systemd` 的 `check-serial-owner.sh`，佔用中就拒絕啟動。

**R6 急停仍是電源開關。** SIGINT 時先停車、手臂原地凍結，再釋放序列埠。

---

## 分工

**koala915（G2 本體）**
1. 在 `grasp/v23/grasp_service.py` 加上底盤功能（TCP 7000 伺服器、odom 發布、R1–R3）。
   啟動路徑與安全閘不動，沿用 launcher 的 `build_ctrl_cmd()`。
2. 速度換算直接 import `sugarbox_rl_motor_server.py` 的 `velocity_to_motor_values`，
   不再複製一份常數。
3. 開機預設的序列埠持有者就是這個服務；`x3plus-navigation.service` 保留給維修用，
   仍然互斥。

**112303577ncu（導航）**
1. 先把 fork 上 9/27 的 4 個 commit 開 PR 合進 `v23-grasp-test`（改到 `mission_pipeline.py`
   與 `sugarbox_rl_motor_server.py`，G2 要用）。
2. 把 Jetson 上的 `ai_motor_server_P0.py` 收進 repo，寫清楚它收哪些 action。它是
   `x3plus-navigation` 實際在跑的馬達服務，也是舊 `action/speed` 格式唯一的定義。
3. 程式內直接控車的 `nav_rl.py`／`nav_rl_grasp_pipeline.py`（模式 C）要改成 7000 用戶端，
   或保留為與常駐服務互斥的獨立模式，由導航端決定。
4. 驗證 odom 改由 rosbridge 發之後，AMCL 與 TF 是否正常。

**enoch20050427（接近）**
1. 把 `codex/sugarbox-approach-review` 重新接到 `v23-grasp-test` 上。
2. 接近程式照舊送 TCP 7000、從 rosbridge 讀資料、收後相機串流，不用改。
3. 確認 Jetson 端的後相機串流是由哪個程式啟動，之後一併做成 systemd 單元。

**所有人**：`git config user.name / user.email` 設成自己的，現在兩個 fork 的 commit 都掛在
koala915 名下。

---

## 實作順序

> **進度（2026-09-28）**：第 1、2、3 步完成。odom 驗收三節全過，第 3 節（AMCL）落地
> 3/3。第 4 步進行中：行駛姿態 ↔ E1 轉場已模擬並上機（`stow`），新增 R7（輪子看手臂姿態）、
> R8（控制板沒回報就停輪）與「夾取前等新畫面」；整段流程尚未跑完。數據見 `progress.md`
> 2026-09-28。「停車 → 開始夾取」因姿態轉換約需 6 s，下面 G3 的 1 s 標準要改。程式：
> `grasp/v23/chassis_server.py`、`odom_bridge.py`；開關在
> `deploy/systemd/grasp-service.service.d/g2-chassis.conf`。

1. 合併 112303577ncu 的 PR（前置條件）。
2. 常駐服務加上 TCP 7000 與 R1–R3。**先把車架起來、輪子離地**，用幾行 Python 送
   速度格式的 JSON（`detection/calibration/` 的工具是舊格式，不能拿來測），確認輪子會轉、
   看門狗會停、手臂動作期間速度指令被擋下。
3. 加上 odom 發布，驗證 AMCL 與 TF。
4. 落地測：enoch 的接近程式開到盒子前 → `graspctl grasp` → `graspctl release`，
   中間不重啟任何服務。
5. G3 驗收（下一節）。
6. 最後再決定任務流程（巡航 → 接近 → 夾取 → 送桶 → 續巡）由誰來串：把
   `mission_pipeline.py` 改成 7000 + `graspctl` 的用戶端，或寫一支新的薄層。
   在那之前，`mission_pipeline.py`、模式 A、模式 C 維持現狀（直接控車、接 v21），
   與常駐服務互斥。這一步不影響前面五步。

---

## 驗收（G3）

連續 10 輪「接近 → 夾取 → 送到桶子 → 放下 → 回到巡航」，每輪檢查：

- `sudo fuser -v /dev/myserial` 永遠只有一個持有者；
- 沒有任何控制器、YOLO、PyBullet 重新載入的訊息；
- 從底盤停止到開始夾取不超過 1 秒（今天的兩服務切換是 11.6 秒）；
- 夾取約 7.2 秒、放下約 4.6 秒維持不變；
- 常駐服務 RSS 不持續上升；
- odom 在夾取前後連續，沒有歸零或 TF 跳變；
- 手臂動作期間，7000 收到的速度指令全部被擋下（看服務日誌）。

---

## 還沒解決、要留意的

- **odom 經 rosbridge 的延遲**：馬達服務原本用 rospy 發。如果 AMCL 對延遲敏感，備案是讓
  常駐服務把 odom 送到本機一支很小的 rospy 節點轉發。
- **記憶體**：常駐服務約 490 MB、視覺約 300 MB、ROS 全套約 570 MB，另加後相機串流。
  4 GB 放得下，但要在 G3 期間持續看剩餘記憶體。
- **放下時夾爪卡在半開**（2026-09-25 發生過一次）：控制器會凍結不收回。任務流程要能處理
  `release` 回傳 `aborted`，不能當成成功。
- **搬運途中掉落偵測不到**：2026-09-25 決定接受此風險，任務流程不要把 `released` 當成
  「確定投進桶子」。
