# v23 Grasp Deployment Package

## 概要

v23 使用 E1 高姿態 (S2=-0.275 rad / 74.2 deg)，相比 v21/C3：
- 相機高度 22.7 → 24.3 cm
- 可用 FOV 面積 70 → 101 cm² (+44%)
- Spawn band 已替物體寬度內縮 2.5cm，YOLO 框不再被裁切

## 包含檔案

```
deploy_v23/
├── README.md                 ← 本文件
├── DEPLOY_V23_MANIFEST.md    ← 完整 checklist
├── model/
│   ├── candidate_v23_seed23401_ckpt250000.zip      ← policy 權重
│   └── candidate_v23_seed23401_ckpt250000_vec.pkl  ← VecNormalize
├── x3plus_real_grasp.py      ← 主部署腳本（已更新 home_deg + model path）
├── deploy_contract.py        ← obs/action 契約定義
├── action_execution_v21.py   ← stage 狀態機
└── x3plus/                   ← URDF + meshes（FK 用）
    ├── yahboomcar.urdf
    └── meshes/...
```

## 快速啟動

### 1. Dry-run（不送馬達命令，先確認流程）

```bash
python x3plus_real_grasp.py \
    --contract obs_28_incremental \
    --obj-x 0.24 --obj-y 0.0 --obj-z 0.02
```

### 2. 實機（送馬達命令）

```bash
python x3plus_real_grasp.py --real \
    --contract obs_28_incremental \
    --obj-x 0.24 --obj-y 0.0 --obj-z 0.02
```

### 3. Socket 模式（接 YOLO 偵測結果）

```bash
python x3plus_real_grasp.py --real --socket \
    --contract obs_28_incremental
```

### 4. 只跑 release（抓完後到 bin 上方開爪）

```bash
python x3plus_real_grasp.py --real --release-only \
    --bin-rim-height 0.05
```

## 重要參數

| 參數 | 值 | 說明 |
|------|----|----|
| `--contract` | `obs_28_incremental` | **必填**，v23 用增量式動作 |
| `--model` | (已內建) | 可省略，預設指向包內 model |
| `--vecnorm` | (已內建) | 可省略 |
| `--max-steps` | 300 (預設) | 一次夾取的最大步數 |
| `--hz` | 10.0 (預設) | 控制頻率 |

## Home Pose (v23 / E1)

| Joint | API (deg) | 說明 |
|-------|-----------|------|
| S1 | 90.0 | 底座旋轉 |
| S2 | **74.2** | 肩膀（C3 是 67.08，E1 抬高了） |
| S3 | 8.6 | 上臂 |
| S4 | 8.6 | 前臂 |
| S5 | 90.0 | 手腕 |
| S6 | 30.0 | 夾爪張開 |

## 與 v21 的差異

1. **Home pose 改了** — S2 從 67.08 → 74.2 度，S3/S4 從 9.79 → 8.6 度
2. **Nav 停止距離需加 +1.3cm** — gc_z 從 0.145 → 0.158m
3. **Spawn band 縮小了** — x(0.205,0.280) y(-0.070,0.065)，物體中心必須在此範圍內

## 上機前確認

- [ ] 把手臂移到 E1 pose (90, 74.2, 8.6, 8.6, 90, 30)，目視確認相機看得到的範圍
- [ ] 放尺量 FOV，確認 x 方向約 13cm、y 方向約 19cm
- [ ] 先 dry-run 再 --real，第一次 --real 時人在旁手扶急停
- [ ] 確認 YOLO→grasp 的座標是用 joint angle 反算還是 fixed homography
