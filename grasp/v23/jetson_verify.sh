#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════
# v23 Jetson 驗證 — 快速路徑步驟 2（E1 grasp home）
#
# 電腦端（PowerShell）先做：
#   .\set_jetson_host.ps1 <Jetson IP>
#   scp -r x3plus\grasp\v23 "jetson@${env:X3PLUS_JETSON_HOST}:~/Documents/deploy_jetson2/grasp/"
#
# Jetson 上：
#   cd ~/Documents/deploy_jetson2/grasp/v23 && bash jetson_verify.sh
#
# 這支只做驗證，不驅動任何伺服機。任何一項 [FAIL] 就停下來回報，不要往下走。
# ═══════════════════════════════════════════════════════════════════════════
set -u
cd "$(dirname "$0")"
fail=0
ok()   { echo "  [OK]   $*"; }
bad()  { echo "  [FAIL] $*"; fail=1; }

echo "=== 0. 虛擬環境 ==="
if [ -z "${VIRTUAL_ENV:-}" ]; then
    echo "  尚未啟用，執行 source ~/grasp_venv/bin/activate 後重跑"
    exit 1
fi
ok "venv = $VIRTUAL_ENV"
python3 -c "import sys; print('  python', sys.version.split()[0])"

echo
echo "=== 1. 資產（URDF + mesh 沿用上層 grasp/x3plus/，v23 不自帶）==="
URDF=../x3plus/yahboomcar.urdf
if [ -f "$URDF" ]; then
    # Jetson 是 Linux、沒有 autocrlf，工作檔雜湊應直接等於 git blob 雜湊。
    # 對不上代表 Jetson 上這份 URDF 是舊的 → FK 會錯，且錯得很安靜。
    want=0a43968783b178d2f270572dc2e01c684722864af1bf8a0c52f05a66fe9d2be2
    got=$(sha256sum "$URDF" | cut -d' ' -f1)
    [ "$got" = "$want" ] && ok "URDF 雜湊符合" \
        || bad "URDF 雜湊不符！got=$got  want=$want （Jetson 上是舊版，需重新 scp grasp/x3plus/）"
else
    bad "找不到 $URDF"
fi
n=$(find ../x3plus/meshes -type f 2>/dev/null | wc -l)
[ "$n" -ge 38 ] && ok "mesh $n 個（需 >= 38）" || bad "mesh 只有 $n 個，需 >= 38"

echo
echo "=== 2. Rosmaster 驅動（A1 移植後的新前置條件，最容易卡這）==="
if [ ! -d ../Rosmaster_Lib ]; then
    echo "  上層沒有 Rosmaster_Lib，嘗試從系統複製…"
    cp -r /usr/local/lib/python3.6/dist-packages/Rosmaster_Lib .. 2>/dev/null \
        && echo "  已複製到 ../Rosmaster_Lib" || bad "複製失敗，請手動確認出廠 SDK 路徑"
fi
python3 -c "
import sys; sys.path.append('..')
from Rosmaster_Lib import Rosmaster
print('  driver import OK')" || bad "Rosmaster_Lib 無法 import（--real 會被擋下）"

if [ -e /dev/myserial ]; then
    ok "/dev/myserial 存在"
else
    bad "/dev/myserial 不存在 → 實跑時要用 --port 指定實際裝置（ls /dev/ttyUSB*）"
fi

echo
echo "=== 3. 序列埠佔用者（出廠會自啟，會搶埠）==="
busy=$(ps aux | grep -E "rosmaster_main|motor_server|roslaunch" | grep -v grep)
if [ -n "$busy" ]; then
    bad "有程序佔著序列埠，實跑前要先 kill："
    echo "$busy" | sed 's/^/      /'
else
    ok "沒有已知的佔用程序"
fi

echo
echo "=== 4. floor guard（641 檢查，Nano 上慢）丟背景 ==="
nohup python3 test_deploy_floor_guard.py > /tmp/v23_guard.log 2>&1 &
echo "  PID $! → /tmp/v23_guard.log（趁這段時間去量木塊、清場地）"

echo
echo "=== 5. SB3 載入（訓練端 2.2.1 < Jetson 2.3.2，預期乾淨無警告）==="
python3 -c "
import stable_baselines3 as s
from stable_baselines3 import PPO
print('  sb3', s.__version__)
m = PPO.load('models/candidate_v23_seed23401_ckpt250000.zip', device='cpu')
print('  obs', m.observation_space, '| act', m.action_space)
assert m.observation_space.shape == (28,), m.observation_space
assert m.action_space.shape == (6,), m.action_space
print('  spaces OK')" || bad "PPO.load 失敗 — 這關過不了後面全部停擺"

echo
echo "=== 6. --real 安全閘 + 權重完整性（A2；不會驅動伺服機）==="
# 不帶 --unlock-candidate-real 的 --real 會在開序列埠、載模型之前就退出 3。
# 閘的順序是「sha256 → contract → status」，所以看到 [REFUSED] 就同時證明了：
# 權重與 vecnorm 的 sha256 對得上 manifest（＝scp 沒有傳壞這 20MB）、contract 正確。
# 傳壞的話這裡會變成 [FATAL] sha256 mismatch，在你站到機器人旁邊之前就抓到。
gate=$(python3 x3plus_real_grasp.py \
  --model models/candidate_v23_seed23401_ckpt250000.zip \
  --vecnorm models/candidate_v23_seed23401_ckpt250000_vec.pkl \
  --contract obs_28_incremental \
  --object-height 0.066 --obj-x 0.24 --obj-y 0.00 --obj-z 0.033 \
  --real 2>&1)
grc=$?
echo "$gate" | grep -E "REFUSED|FATAL|mismatch" | head -4
if [ "$grc" -eq 3 ] && echo "$gate" | grep -q "\[REFUSED\]"; then
    ok "安全閘擋下 candidate 的 --real（exit 3），且 sha256 / contract 皆符合"
else
    bad "安全閘行為不符（exit=$grc，預期 3 且含 [REFUSED]）——若是 sha256 mismatch，代表 scp 傳壞了，重傳 models/"
fi
echo "$gate" | grep -qi "serial Close" \
    && bad "安全閘竟然開了序列埠（必須在碰硬體前就擋下）" \
    || ok "拒絕發生在碰序列埠之前"

echo
echo "=== 7. controller 自測（需 all 163 checks passed）==="
# 用 grep 抓結果行而不是 tail：驅動的 "serial Close!" 之類訊息會在直譯器結束時
# 才印出來，把結果行擠出尾端，看起來就像測試沒跑完。
# 116 = 38 + 夾爪接觸處理（第 4 跑：物體夾住後指令仍走向 180，齒輪研磨；
# 現在接觸即停、指令停在接觸角+bias，Stage 2 全程維持同一 hold）
#      + 懸停後備（第 5 跑：策略自己夾住物體、在進場半徑外 1mm 懸停 270 步，
# 超時開爪把物體還回去；現在 xy/z/pads 全過、只差半徑 ≤5mm 且連續 3 秒就進 Stage 1）
#      + guard 死鎖偵測與手指誤差修正（第 6 跑：8mm 線落在 S2 相鄰兩個編碼器刻度之間，
# 策略往下、guard 往上，42 次 raise、z_now 完全不變、213 步無進展直到人為中斷）
#      + socket 路徑補強（2026-08-01：模式 B 移到 v21。v21 原本的 socket 沒有 payload 驗證、
# 沒有過期偵測、也沒有 latch —— NaN 座標會直接進 policy，且手臂一離開 home 相機距離模型就失效。
# 新增 26 項：_parse_payload 驗證、snapshot() 新鮮度、--latch-obj 凍結、--real --socket 雙重確認）。
ctl=$(python3 test_deploy_controller.py 2>&1)
echo "$ctl" | grep -E "checks passed|FAIL|Traceback|Error" || echo "$ctl" | tail -5
echo "$ctl" | grep -q "all 163 checks passed" \
    && ok "163 項全過" || bad "controller 自測未通過（完整輸出見上）"
# 測試不得碰硬體：若出現驅動的開/關埠訊息，代表有建構點漏掉 dry_run
echo "$ctl" | grep -qi "serial Close" \
    && bad "測試過程開了序列埠（不該發生，檢查 ServoController 建構點）" \
    || ok "測試未觸碰序列埠"

echo
echo "=== 7.5 伺服機讀取（半雙工匯流排；需 all 37 checks passed）==="
# 2026-07-31 三次實跑分別死在 S1（第 1 步）、S6（第 49 步）、S3+S4（啟動第一次讀取，
# 當時還沒送出任何指令）。最後一項排除了「與自己的寫入碰撞」。
# 真正的原因在驅動裡：get_uart_servo_value 回傳「第一個抵達的回應」而不檢查是不是
# 問的那一顆，get_uart_servo_angle 再自己比對 ID、對不上就回 -1——把好的讀值丟掉。
# 一個遲到封包會讓整條管線錯位，症狀就是相鄰數顆同時「讀不到」。
rd=$(python3 test_servo_read.py 2>&1)
echo "$rd" | grep -E "checks passed|FAIL|Traceback" || echo "$rd" | tail -5
echo "$rd" | grep -q "all 37 checks passed" \
    && ok "37 項全過" || bad "伺服機讀取測試未通過（完整輸出見上）"

echo
echo "=== 7.6 一鍵 launcher 接線（需 all 62 checks passed）==="
# 2026-08-14：launcher 還在傳 nav-home 外參、也沒傳 --homography，而 bridge 早就把 grasp
# home 改成只收實測 homography。兩邊各自都對，是介面對不上——而且要等手臂走到定位才會發現。
# v23 又多釘四條同類型的線：--pose 必須是 v23_e1_grasp_home（bridge 的預設還是 C3，漏傳
# 不會報錯，只會讓每一幀都因為 S2 差 7.1° 被丟掉）、policy band 要跟 controller 對得起來、
# 校正檔路徑不能是 v21 那支，權重 sha256 要對得上 manifest（v21/v23 契約相同，形狀檢查抓不到）。
lch=$(python3 test_one_command_launcher.py 2>&1)
echo "$lch" | grep -E "checks passed|FAIL|Traceback" || echo "$lch" | tail -5
echo "$lch" | grep -q "all 62 checks passed" \
    && ok "62 項全過" || bad "一鍵 launcher 接線測試未通過（完整輸出見上）"

echo
echo "=== 7.7 三姿態掃描交接（需 all 89 checks passed）==="
# 純邏輯：不開相機/序列埠、不載 PyBullet/YOLO/PPO。固定左中右三姿態、E1 單一
# homography + S1 剛體旋轉、
# 回 E1 後才釋出 target，以及 scanner/controller 不同時擁有硬體，都在這裡釘住。
scn=$(python3 test_three_pose_scan.py 2>&1)
echo "$scn" | grep -E "checks passed|FAIL|Traceback" || echo "$scn" | tail -5
echo "$scn" | grep -q "all 89 checks passed" \
    && ok "89 項全過" || bad "三姿態掃描測試未通過（完整輸出見上）"

echo
echo "=== 8. dry-run（不驅動伺服機；需 wrist_z_offset = 0.0564）==="
python3 x3plus_real_grasp.py \
  --model models/candidate_v23_seed23401_ckpt250000.zip \
  --vecnorm models/candidate_v23_seed23401_ckpt250000_vec.pkl \
  --contract obs_28_incremental \
  --object-height 0.066 --obj-x 0.24 --obj-y 0.00 --obj-z 0.033 \
  2>&1 | grep -E "wrist_z_offset|^\[Stage\]|^\[Retreat\]|^\[FK\]"
echo "  dry-run exit=${PIPESTATUS[0]}（需 0）"

echo
echo "=== 9. 背景那支的結果（需 all 641 checks passed, clamped 291x）==="
wait
tail -3 /tmp/v23_guard.log

echo
echo "═══════════════════════════════════════════════════════════════"
echo "Gate 判準："
echo "  · 163 / 37 / 641 / 62 / 89 一字不差"
echo "  · wrist_z_offset 必須 0.0564（純幾何，跨平台不該變；也不隨姿態變——hover 高度錨在地板不是手臂，所以 C3→E1 這個值不動）"
echo "  · Stage 序列 0→1 → jaw close → ABORT → retreat home，exit 0"
echo "  · 策略步數容許 ±3 浮點漂移；超過就停下來回報"
[ "$fail" -eq 0 ] && echo "前置檢查全過。" || echo "⚠ 上面有 [FAIL]，先解掉再上機。"
echo "═══════════════════════════════════════════════════════════════"
