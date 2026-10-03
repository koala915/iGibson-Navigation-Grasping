# 交付原文（未修改）

訓練端在 `grasp/deploy_v23/` 交付的兩份文件，原封收在這裡當出處。
**這兩份是訓練端的說法，不是本包的事實來源** —— 本包的事實來源是
`../manifest.json`，裡面每個幾何數字都是用本包自己的 `FKComputer` 算出來的。

| 檔案 | 原本位置 |
|---|---|
| `DEPLOY_V23_MANIFEST.md` | `grasp/deploy_v23/DEPLOY_V23_MANIFEST.md` |
| `README_as_delivered.md` | `grasp/deploy_v23/README.md` |

## 已知需要對照著讀的兩點

**1. 高度是不同座標系。** 交付文件寫相機 22.7 → 24.3 cm、gc_z 0.145 → 0.158 m，
那是 URDF 原點座標。本包一律用 training frame（低 0.907 cm，
`dc.URDF_TO_TRAINING_FRAME`），同樣兩個姿勢是 0.2183 → 0.2340 與 0.1439 → 0.1558。
兩邊差值都是 +1.6 cm，**都沒錯，但不能混在同一條算式裡**。

**2. checklist 講的那支腳本本包沒用。** 交付的 `x3plus_real_grasp.py` 是
2026-07-31 之前的 release 分支副本，缺夾爪接觸偵測等硬體防線；本包改用
`grasp/v21/` 那支硬體版，只換 `home_deg`、model 路徑與 target envelope。
理由與缺什麼，見 `../manifest.json` 的 `deployment_base`。

其餘 205 MB（meshes、URDF、model、腳本）沒有進版控：URDF 與 38 個 mesh 與
`grasp/x3plus/` 逐位元組相同，權重已經在 `../models/`，腳本已經在
`../../v21/reference/`。原始交付目錄 `grasp/deploy_v23/` 留在工作目錄未追蹤，
確認過就可以自行刪除。
