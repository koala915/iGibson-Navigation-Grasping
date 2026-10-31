"""Build a review inventory, not a declaration that unreferenced files are dead."""
from pathlib import Path
import json, collections
OUT=Path(__file__).parent
rows=json.loads((OUT/'source_inventory.json').read_text(encoding='utf-8'))
active={
'grasp/x3plus/__init__.py':'ACTIVE FK/URDF asset package marker；不是hardware test',
'integration/mission_pipeline.py':'正式整合候選入口；build_and_run直接組裝所有adapter',
'integration/mission_fsm.py':'MissionRunner匯入；pure state transitions',
'integration/feedback_odom.py':'Mission OdomPublisher使用；board-motion feedback積分',
'integration/map_goal_provider.py':'Mission使用；external route與AMCL→goal',
'integration/ros_io.py':'Mission使用；rosbridge odom/TF/pose/status I/O',
'integration/mission_status.py':'Mission/UI間狀態與控制協定',
'integration/nav_rl.py':'Mission/RLNavigator使用；55D PPO與LaserScan adapter',
'integration/nav_rl_grasp_pipeline.py':'ACTIVE類別庫；standalone Mode C為EXPERIMENTAL且有B18',
'integration/vision_grasp_pipeline.py':'ACTIVE Navigator/base helpers；standalone legacy orchestration已限制real',
'integration/vision_grasp_bridge.py':'Mode B與one-command launcher使用；pose-stamped TCP sender',
'integration/arm_cam_geometry.py':'Bridge/nav/calibration共用幾何與pose身份',
'integration/grasp_home_homography.py':'Bridge/standalone校正投影；mission尚未完整接入',
'integration/trash_target.py':'Mission可選offboard target adapter',
'detection/rear_cam_sam2_publisher.py':'可選offboard視覺publisher；不直接持有motor',
'stream_cam.py':'被文件/操作流程使用的standalone camera服務；非ROS driver',
'ui/server.py':'Console entry與child管理；B09/B19尚需修',
'ui/launcher.py':'launch_ui.sh真正exec入口',
'ui/make_qr.py':'UI QR helper',
'grasp/v21/x3plus_real_grasp.py':'Mission動態載入 + v21 standalone controller',
'grasp/v21/deploy_contract.py':'v21 controller與tests匯入的模型契約',
'grasp/v21/action_execution_v21.py':'v21 controller使用的incremental safety action execution',
'grasp/v21/jetson_one_command_grasp.py':'v21正式獨立抓取候選launcher；不是完整mission',
}
def classify(p):
    if p in active:return 'ACTIVE',active[p]
    if '/reference/' in p or p.startswith('training/') or p.startswith('grasp/deploy_v23/'):
        return 'LEGACY','交接/舊來源保留；不在現行mission import鏈；部分Arm_Lib版本不得執行'
    if p=='grasp/x3plus_real_grasp.py':return 'LEGACY','v17 absolute備援；model_tools wrapper仍會subprocess呼叫，非DEAD'
    if p=='grasp/Rosmaster_Lib_reference.py':return 'LEGACY','driver參考源碼；runtime import的是外部Rosmaster_Lib，不是此檔'
    if p.startswith('grasp/trained_6d_models_v18/') or p.startswith('model_tools/'):
        return 'LEGACY','舊模型package流程；wrapper固定v17，不能當v21/v23通用launcher'
    if Path(p).name.startswith('test_') or p.startswith('tests/'):
        return 'TEST','離線suite候選；已執行清單以offline_test_results.json為準'
    if p.startswith('grasp/v23/'):
        return 'EXPERIMENTAL','v23 candidate獨立路徑；有實測記錄但mission loader仍選v21'
    if p.startswith('detection/calibration/') or p.startswith('detection/debug_tools/'):
        return 'TEST','校正/診斷utility；其中部分可送TCP motor command，非一律offline-safe'
    if p.startswith('detection/'):
        return 'LEGACY','較早vision/navigation standalone；TCP或cmd_vel旁路，未被mission匯入'
    if p.startswith('yolo/'):
        return 'EXPERIMENTAL','獨立模型demo/資料配置；mission實際載入detection/models/best.pt'
    if p.startswith('grasp/') and p.endswith('.py'):
        return 'TEST','FK/servo/bus/pose/workspace工具；real或硬體probe須獨立維護模式'
    if p.startswith('integration/') and p.endswith('.py'):
        return 'TEST','部署檢查、camera/geometry校正或smoke工具；按CLI區分offline與hardware'
    if p.startswith('document/') or p.startswith('SUMMER_'):
        return 'EXPERIMENTAL','本機報告/生成器，不是機器人runtime；保留使用者未提交工作'
    if p.endswith(('.md','.txt')):
        return 'ACTIVE','文件/依賴記錄；描述不等於runtime證據，歷史handoff需按日期解讀'
    if p.startswith('ui/') or p=='.github/workflows/tests.yml':
        return 'ACTIVE','UI資產/啟動環境工具或CI；不是獨立motor controller'
    if p.endswith('.urdf') or p.startswith('integration/nav_best_model/') or p.startswith('grasp/v21/'):
        return 'ACTIVE','模型/幾何/契約資產；需與實際consumer和版本配對'
    if p=='startup_device_check.py':return 'TEST','裝置preflight入口；非純離線unit test'
    return 'ACTIVE','配置/輔助資產；沒有足夠證據宣稱DEAD'
head='''# Codebase Classification

ACTIVE 表示現行 caller/操作入口或支援資產，**不表示 hardware-approved**。LEGACY 是保留的舊路徑；TEST 包括會動硬體的診斷程式，不能用檔名判定可安全執行；EXPERIMENTAL 是未接上正式整合的候選或本機工作。DEAD 需要證明沒有動態/手動使用，本輪不以 graph 零結果刪檔。

本表覆蓋主 checkout 的187個文字檔（105 Python）；另253個文字檔位於ignored `.claude`資料（主要是三套歷史source snapshots），完整路徑/sha256/AST symbols/imports/calls在 [source_inventory.json](evidence/source_inventory.json)。snapshot中的153 Python亦通過靜態parse，**不納入現行入口**。未追蹤檔不等於已合併。二進位模型/mesh不是Python程式，不以AST分類；模型身份由manifest和測試確認。

讀取層級：所有列入inventory的文字檔完整讀取並作機械化盤點；主線控制/安全/ROS/影像/入口有函式級追蹤與人工深讀。歷史快照與文書生成器以結構、差異與用途辨識為主，**不宣稱每份歷史副本都完成逐行獨立語意審查**。這個範圍限制不能被「258 Python parse passed」掩蓋。

## 個別檔案

| File | Classification | Usage / evidence basis | Git |
|---|---|---|---|
'''
lines=[head]
classified=[]
for row in rows:
    p=row['path']
    if p.startswith('.claude/'):continue
    status,why=classify(p)
    classified.append({'path':p,'classification':status,'basis':why,'tracked':row['tracked']})
    lines.append(f'| [{p}](../../../{p}) | {status} | {why} | {"tracked" if row["tracked"] else "untracked / ignored"} |\n')
lines.append('''
## DEAD候選與不能誤判的檔案

- `MapGoalProvider.in_forbidden()` 沒有在mission行進迴圈找到consumer：屬於尚未接上的能力／dead-call candidate，不代表整個map_goal_provider.py可刪。
- `grasp/x3plus_real_grasp.py` 有model_tools subprocess入口，因此不能照「沒有import」分類DEAD。
- v21/v23的測試、entrypoint、dynamic import、socket receiver與utility均可能沒有一般inbound graph edge。
- `.claude/worktrees/{grasp-v18-arm-c3,grasp-v21-candidate,settle-before-close}/x3plus` 是歷史source snapshots；git worktree list本輪只列根目錄。列LEGACY snapshot，保留，不當成另一套正在執行的deployment。
- `grasp/deploy_v23/` 為使用者未追蹤的handoff；其controller與正式 `grasp/v23/`不同且帶舊Arm_Lib，禁止以檔名替換。

## 文件與log的使用規則

CLAUDE為規格入口，INDEX為導覽；progress/manifest/handoff提供歷史實測宣稱，與目前code逐項比對。v21/HANDOFF不是docs/handoff/HANDOFF。Git曾保存的Mode B log只有bridge視覺傳輸，不能證明目前完整patrol→grasp→bin任務已成功。binary模型成對hash不等於模型行為已通過實機驗收。
''')
(OUT.parent/'CODEBASE_CLASSIFICATION.md').write_text(''.join(lines),encoding='utf-8')
(OUT/'classification.json').write_text(json.dumps(classified,ensure_ascii=False,indent=2),encoding='utf-8')
print(dict(collections.Counter(x['classification'] for x in classified)))
