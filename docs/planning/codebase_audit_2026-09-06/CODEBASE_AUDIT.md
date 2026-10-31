# Codebase Audit — 2026-09-06

## Overall assessment

本專案已經有可辨識的整合主線、相當多的離線回歸測試，以及有價值的實機校正與故障修正紀錄。真正的主線是 Python3 MissionRunner、PPO navigation、v21 grasp 與共享 Rosmaster handle，不是文件容易讓人聯想到的 move_base/cmd_vel 標準導航堆疊。既有 floor guard、servo half-duplex read、model pair、camera pose stamp、候選模型限制都應保留。主要弱點是安全條件散落在不同 CLI 與類別層，重用類別時可能跳過入口保護。回授新鮮度與 stationary 的定義亦未真正綁定硬體封包，因此會使原本合理的安全閘失去依據。現在有多條獨立控制入口，但沒有跨程序的 serial 排他與命令仲裁；主迴圈停止工作時的停止時間也沒有完整證據。外部 ROS launch、map、AMCL 與 motor server 缺席，無法據此 checkout 宣稱已建立可重現的全系統 bringup。結論是 **局部工程品質已有基礎，整機仍屬整合候選，尚不足以宣稱正式自主移動與投放流程可靠**。

## 交付文件

1. 本文件：範圍、baseline、模組、重複實作、環境、測試與總結。
2. [ACTUAL_RUNTIME_ARCHITECTURE.md](ACTUAL_RUNTIME_ARCHITECTURE.md)：實際 caller/data flow、Mermaid、ROS/TF contracts。
3. [BUG_AND_RISK_REGISTER.md](BUG_AND_RISK_REGISTER.md)：28項按P0–P3分級的問題與證據。
4. [CODEBASE_CLASSIFICATION.md](CODEBASE_CLASSIFICATION.md)：逐檔用途、active/legacy/test/experimental與dead判斷限制。
5. [CONTROL_OWNERSHIP_ANALYSIS.md](CONTROL_OWNERSHIP_ANALYSIS.md)：所有已發現的直接與間接控制來源。
6. [INTEGRATION_PLAN.md](INTEGRATION_PLAN.md)：canonical architecture、最小patch順序與驗收。
7. [MASTER_BRINGUP_PLAN.md](MASTER_BRINGUP_PLAN.md)：目前缺件、實際啟動依賴與統一profile方案。

## Baseline與修改範圍

| Item | Value |
|---|---|
| Workspace | C:\Users\user\Documents\repo\iGibson-Navigation-Grasping |
| Branch | v23-grasp-test |
| HEAD | 67d29af58ae68acbba020e427c748d1790bc3b1b |
| HEAD description | Give v21 the jaw protections that were only ever fixed in v23 |
| 執行平台 | Windows / Python 3.12.10；非ROS Melodic / Jetson環境 |
| 本輪修改 | 僅本audit目錄內Markdown、靜態盤點與隔離重現腳本、測試結果 |
| 未執行 | 實體motor/servo movement、ROS bringup、TCP motor commands、package安裝、legacy硬體script的全量import |

進場已存在的修改全部保留：`CLAUDE.md`、`docs/images/ui/` 下的五張PNG（03_mobile_connection、04_mobile_console、05_mobile_setup_top、06_mobile_setup_actions、07_mobile_map）、`integration/vision_grasp_bridge.py`、`progress.md`、`tests/test_vision_grasp_bridge_pose.py`、`ui/server.py`、`ui/static/app.js`、`ui/static/index.html`、`ui/static/style.css`。精確清單見 [git_baseline.txt](evidence/git_baseline.txt)；其中新增的 audit 目錄是本輪產物。

進場未追蹤：`SUMMER_MEETING_PPT.md`、`SUMMER_RESULTS_REPORT.md`、`docs/planning/PROJECT_EXECUTION_ROADMAP_2026-08-31.md`、`grasp/deploy_v23/`。未checkout、reset、commit或覆蓋它們。repo忽略training/document與hidden快照，本輪仍盤點其文字來源，沒有清除它們。

## 範圍、方法與證據強度

- 先依AGENTS讀CLAUDE/INDEX與交接文件；使用 codebase-memory graph 做architecture/search/trace。graph目前未覆蓋integration/UI/v23等現行路徑，索引回傳仍有舊root路徑；對漏索引的部分改用原始碼、AST、字串搜尋與依賴追蹤。沒有以零inbound edge認定DEAD。
- 主checkout盤點 **187文字檔、105 Python、86,714行文字**：44 Markdown、17 JSON、5 shell、3 URDF、2 YAML、1 CI YAML及UI/requirements等。另讀253份ignored `.claude`文字，主要是三份歷史source snapshot。合计258 Python皆AST parse成功，Python3.8語法parse亦未發現錯誤。
- [source_inventory.json](evidence/source_inventory.json) 保存path、tracked狀態、SHA256、line count、Python symbols/imports/calls；[inventory_summary.json](evidence/inventory_summary.json) 保存排除範圍。`.git` object、依賴環境、cache不作runtime source；Git內容另行讀取。
- 主線入口、motor/feedback、ROS、policy、camera/grasp、FSM、UI argv、model gates採函式級人工分析；其他utility、歷史副本與文書生成器以結構/用途/差異盤點。**本報告不是「全部歷史副本逐行人工審完」的宣稱，也不能證明沒有其他bug。**
- 模型權重不能當文字閱讀；本輪透過既有package tests、manifest hash resolver與實際dry model load驗證可用性；沒有把訓練統計或文檔實測宣稱當成本轮重新量測。
- 實際地圖圖片/route YAML/AMCL/nav launch不在checkout；文件中的外部Navigation路徑不是本輪已讀取的配置。此限制使ROS deployment層的完整audit仍有外部缺口。

## 主要模組與真正入口

| Module | 真正entry / runtime | Dependency / usage | 判斷 |
|---|---|---|---|
| Task / mission | integration/mission_pipeline.py | build_and_run組裝policy、v21、ROS、odom、camera、FSM | ACTIVE整合候選；沒有唯一ROS launch |
| FSM | mission_fsm.py | MissionRunner._sense/_act | ACTIVE；selftest不需要robot |
| Motor / servo | v21 ServoController → 外部Rosmaster_Lib | mission把同一device給Navigator與feedback reader | ACTIVE；不是自帶TCP motor server |
| Feedback odom | feedback_odom.py + mission OdomPublisher | get_motion_data→integrate→RosBridgeIO | ACTIVE；packet freshness有已重現缺陷 |
| ROS / TF | ros_io.py | /amcl_pose、/odom_setmotor、/tf、offboard/status | ACTIVE Python3 bridge；ROS native nodes外部 |
| LiDAR / navigation RL | nav_rl.py | RosLaserScanSource→55D PPO→plant shape | ACTIVE；direct rplidar為legacy option |
| Patrol / delivery | map_goal_provider.py | external route + AMCL→dist/bearing→PPO | ACTIVE；沒有move_base global planning |
| Near vision navigation | Navigator/RLNavigator繼承鏈 | mission直接使用class | ACTIVE library，standalone orchestrators另有gate/模型入口問題 |
| Grasp v21 | grasp/v21/jetson_one_command_grasp.py；x3plus_real_grasp.py | standalone launcher或mission dynamic import | ACTIVE candidate，manifest明確非hardware-approved |
| Grasp v23 | grasp/v23/jetson_one_command_grasp.py + three_pose_scan.py | E1/pose scan standalone | EXPERIMENTAL；有3/3局部實測記錄但尚非整機認證 |
| Camera / detection | stream_cam.py、vision_grasp_bridge.py；可選rear_cam_sam2_publisher | MJPEG、YOLO、TCP5555、offboard ROS target | ACTIVE可選路徑；需明確stream與calibration身份 |
| UI | launch_ui.sh→launcher.py→server.py→subprocess | A mission / B bridge / C RL | ACTIVE frontend；argv與ports有blocking bugs |
| Legacy nav / calibration | detection/arm_center_setmotor、rear_nav、nav_arm_cam與calibration/debug_tools | TCP7000或cmd_vel | 不應和mission同跑；部分TEST會動硬體 |
| Model tooling | model_tools/run_grasp_package.py | 固定subprocess跑grasp/x3plus_real_grasp.py | LEGACY v17/v18 contract workflow；不是v21通用entry |

## Duplicate / conflicting implementations

| Function | Files | Current usage | Difference | Recommended canonical version |
|---|---|---|---|---|
| Grasp controller | grasp/x3plus_real_grasp.py；v21/；v23/；v21/reference/；deploy_v23/；training/ | mission→v21；standalone可跑v21/v23；wrapper仍指v17 | absolute vs incremental；home/model/scan/contact/floor/manifest差異；舊副本含Arm_Lib | v21整合主線；v23隔離candidate，其他標legacy，不刪 |
| Serial ownership | v21/v23 ServoController、move_arm、servo_test、joint_direction_calibration | 各自可建立Rosmaster | 單次工具與mission完整owner不同；缺跨程序排他 | 共享主線owner + maintenance互斥模式 |
| Wheel commands | vgp.Navigator、nrgp.RLNavigator、MissionNavigator | PPO與fine-align/recovery都會寫device | SI與legacy speed scalar、gate涵蓋不同 | 一個MotorAdapter承接各source |
| TCP motor clients | arm_center_setmotor、rear_to_arm_blind_handoff、calibrate_*、部分debug tools | 手動legacy測試 | action/speed與custom motor payload；server未提供 | 主線不需此server；legacyclient隔離，取回server後再決定 |
| TCP servers | v17/v21/v23 DetectionReceiver | grasp observation接收 | 5555 object-pose server，不是motor server；同時bind也會衝突 | 選定版本一個receiver；mission in-process用obj_provider |
| Odometry | feedback_odom.py / OdomPublisher；文件Route A另有runtime | 此repo可確認一條主線；舊snapshot副本 | feedback-derived；未找到多個ACTIVE local odom實作 | 保留feedback_odom，關閉外部重複publisher前先取證 |
| TF | ros_io odomTF；3份同源URDF；外部TF文件 | mission動態TF與PyBullet幾何 | URDF copy不是TF publisher；外部staticTF是否重複未知 | 唯一odom authority + 唯一URDF/static authority |
| LiDAR | nav_rl RosLaserScanSource / RPLidarSource | ROS主線與legacy direct backend | TG30ROS vs Slamtec protocol、角度方向不同 | ROS /scan + 實測orientation evidence |
| Navigation runtime | nav_rl_grasp_pipeline / vision_grasp_pipeline / mission / detection legacy | mission類別鏈；其他standalone | PPO vs視覺heuristics vscmd_vel/TCP；realgate不同 | mission + nav_rl；不把全部main同時啟動 |
| Patrol / waypoint | MapGoalProvider/FSM；外部Route C文件；UI synthetic route fixture | mission route；test只fake | fixture117點不是正式map route；bin override非完整delivery path | MapGoalProvider + 版本化實際route/map |
| Safety gates | v21release/floor/servo；missionFSM；navbrake；bridgepose/homography；UI | 分散於main/class/stream layers | 有些跨入口未共享；timeout不等於獨立watchdog | 原有gate保留，統一entry validation與motor safety出口 |
| Vision geometry / readers | arm_cam_geometry、bridge、vgp、legacy detection、stream_cam | 主線與standalone並存 | C3/E1/travel pose；trig vs measured homography；freshness/resizing差異 | shared projector + explicit validity/pose contract，後續才抽reader |
| Incremental helpers / tests | v21/v23/deploy/training deploy_contract/action_execution | v21/v23各自匯入 | 部分內容相同、來源不同；controller仍分叉 | 先建立契約差異驗證，不直接合併整套controller |
| AMCL / navigation launch/config | 外部文件有引用；此repo無實檔 | 無法確認 | 無法列出現場重複node/param | 取回deployment bundle後選canonical；不捏造檔名 |
| URDF/model資產 | grasp/x3plus、deploy_v23/x3plus、training/x3plus及hiddensnapshots | v21/v23引用既有FK模型；其他交接副本 | source/meshes複製使搜尋易誤判，並非多個活動TF | 主線URDF hash作身份，保留交接provenance |

## Python / environment review

主線大量使用 `from __future__ import annotations`、dataclasses、f-string、SB3/Ultralytics，不能在ROS Melodic通常的Python2 runtime直接執行。專案已採Python3.8任務環境透過roslibpy連接Melodic；這個分界合理，應保留。唯一明顯的原生rospy導航腳本 `detection/nav_arm_cam.py` 同時使用Python3視覺依賴，需視為legacy環境整合風險，不能用「改shebang成python3」宣稱已解决Melodic binary modules。

所有primary Python通過3.8語法parse只證明語法，不證明Jetson wheels、CUDA、torch、OpenCV、rospy/tf binary相容。現有CI與本次測試都是3.12；本機缺roslibpy，ROS tests使用fake，沒有建立live bridge。實際dry controller load成功，但SB3提示舊pickle中的lr_schedule/clip_range不能完整deserialize；推論建構沒有失敗，不把此warning當實機部署已壞掉。

UI interpreter來源依序為X3PLUS_PYTHON、~/grasp_venv、repo .venv、system python3；fallback可能落入不相容環境。PyYAML為route loader所需，部署清單不應只依賴其他套件間接帶入；roslibpy等版本未完全固定。五個主checkout shell的Git mode為100644，fresh Linux clone應用`bash script.sh`或後續補executable。沒有在本輪將Melodic node全面升級Python。

## Git history / logs

[git_history.txt](evidence/git_history.txt) 保存所有本地可见commit與檔案變更；未fetch遠端，因此不是遠端完整歷史保證。關鍵演進包括：`a3a0709` v21整合、`ab115f5` mission layer、`02ca1f9` LiDAR gate、`da75c7c` nav/grasp home分離、`327855c` AMCL nomotion、`7d97747` UI、`ba0657f` 移除根目錄log、`585e173` offboard target、`fed3c4f` v23基於v21、`29865c9` 相機舊frame修正、`8e8bc0f` jaw protection、`a7143bc` homography路徑修正、HEAD `67d29af` 將jaw修正回移v21。

因此不能只看較早handoff就宣稱v21仍缺jaw保護；同理v23「只有home與模型不同」是初始移植描述，後續scan與保護已演進。先前Mode B bridge logs從歷史blob恢復到evidence，原檔編碼／換行造成部分恢復文字不完整，僅作歷史camera開啟與bridge偵測背景，**不用其數值當本輪定量驗證**。沒有取得目前真機完整motion/TF/serial trajectory log。

## Offline validation

[offline_test_results.json](evidence/offline_test_results.json)：**32個安全離線命令全部exit0**，每條命令、耗時與完整stdout/stderr各自保存。包括v21/v23 controller、servo、floor、one-command launcher、v23 three-pose；7個tests/ suites；ui/test_server；以及mission、FSM、vision、RL、odom、map、ros_io、status、homography、camera geometry/solver、pose explorer、UI launcher selftests。

目前實際計數：v21 controller **135 checks**（AGENTS/CI寫119已過時）、v23 controller **148 checks**、每版servo **37**、每版floor **641**、UI **47 tests**。不同suite的check/test意義不同，沒有把數字任意相加成獨立測例總數。

另外 [reproduce_findings.py](evidence/reproduce_findings.py) 用真實函式和fake device/message/input重現cached feedback、Thread.join、EOF start、invalid vision success、UI parser、bin yaw、AMCL validation、NaN scan問題；[reproduce_controller.py](evidence/reproduce_controller.py) 以dry controller排除height疑點並確認close漏掉servo。這些腳本記錄現況，**不是已套用的fix regression**，其exit0表示證據收集完成，不表示被測缺陷不存在。

## Top 5 / 最先處理

1. **B01+B08**：回授／快照新鮮度不可靠，stationary gate可被cached資料滿足。
2. **B02+B03**：mission類別重用繞過模型release與homography real-entry保護。
3. **B04+B05**：沒有可驗證停止期限與跨程序唯一serial/controller ownership。
4. **B06+B07**：stdin EOF自動開始，以及確定可重現的shutdown join崩潰。
5. **B10–B16**：near alignment速度/座標、視覺成功判定、bin heading與sensor有效性，影響完整任務閉環。

最值得先修的確定性bug是join、EOF、UI argv、消費端資料驗證與unknown verification；最值得先取實機／部署證據的是packet freshness來源、板端watchdog、serial所有權、TF／LiDAR方向與真實map/route。Canonical主線仍是mission + nav_rl + feedback_odom + ros_io + grasp/v21，v23維持隔離candidate，不做大rewrite。

收尾驗證見 [audit_validation.json](evidence/audit_validation.json)：七份報告皆存在、local links無缺件、inventory中的原始文字檔SHA256均未變、三份主checkout URDF的21links/20joints與mesh路徑可解析且內容hash相同、v21/v23模型及VecNormalize四份artifact全部吻合manifest。額外數值重現亦確認near speed由0.045提高為0.16788m/s、minimum turn為0.57216rad/s，以及凍結odom快照不會自行過期。

下一輪每個patch需記錄Problem、Root Cause、Change、Risk、Validation，先保存before反例，再驗修正與相關既有tests。需實機確認的項目包括motor mapping/units、回授scale、停止延遲、後向障礙、TF/clock/map identity、C3/E1校正與相機pose、bin heading/clearance；目前沒有把這些標成已驗證。
