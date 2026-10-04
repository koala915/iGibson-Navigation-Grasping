# v23 Deployment Manifest

## Model

| Item | Value |
|------|-------|
| Model file | `trained_6d_models_v23/candidate_v23_seed23401_ckpt250000.zip` |
| VecNormalize | `trained_6d_models_v23/candidate_v23_seed23401_ckpt250000_vec.pkl` |
| Contract | `obs_28_incremental` (28D obs, incremental arm, absolute gripper) |
| Training seed | 23401 |
| Teacher | `runs/three_stage_20260818-024231/checkpoints/ppo_three_stage_801408_steps.zip` |

## Grasp Home Pose

| Coord | Sim (rad) | API (deg) |
|-------|-----------|-----------|
| S1 | 0.0 | 90.0 |
| S2 | -0.275 | 74.2 |
| S3 | -1.42 | 8.6 |
| S4 | -1.42 | 8.6 |
| S5 | 0.0 | 90.0 |
| S6 (gripper open) | -1.5 | 30.0 |

**x3plus_real_grasp.py `home_deg`**: change `(90.0, 67.08, 9.79, 9.79, 90.0, 30.0)` to `(90.0, 74.2, 8.6, 8.6, 90.0, 30.0)`

## Spawn Band (FOV-safe, object-width inset)

| Axis | Range (base frame, m) |
|------|-----------------------|
| x | 0.205 - 0.280 |
| y | -0.070 - +0.065 |
| Usable area | ~101 cm2 |

## Changes from v21 (previous deployed version)

| What | v21 (C3) | v23 (E1) | Delta |
|------|----------|----------|-------|
| S2 | -0.400 (67.08 deg) | -0.275 (74.2 deg) | +7.12 deg |
| Camera height | 22.7 cm | 24.3 cm | +1.6 cm |
| gc_z (grasp center) | 0.145 m | 0.158 m | +1.3 cm |
| Usable FOV area | 70 cm2 | 101 cm2 | +44% |

## Deployment Checklist

- [ ] Update `x3plus_real_grasp.py` `home_deg` to `(90.0, 74.2, 8.6, 8.6, 90.0, 30.0)`
- [ ] Update `x3plus_real_grasp.py` `model_path` to `trained_6d_models_v23/candidate_v23_seed23401_ckpt250000.zip`
- [ ] Update `x3plus_real_grasp.py` `vecnorm_path` to `trained_6d_models_v23/candidate_v23_seed23401_ckpt250000_vec.pkl`
- [ ] Confirm `contract_name` is `obs_28_incremental` (should already be correct)
- [ ] Nav stop distance: add +1.3 cm (gc_z increased from 0.145 to 0.158)
- [ ] Real-robot FOV ruler verification at E1 pose (expected: model window shifted -4.0cm x, -3.5cm y, depth x0.89, width x0.97)
- [ ] Confirm YOLO coordinate projection method (fixed homography vs joint-angle reprojection); if fixed homography, update for E1 pose

## Known Limitations

- Grid cell (0.28, -0.070) scored 73.3% (below 80% per-cell gate); hotspot added, awaiting RL re-run
- E1b (+10% FOV, S2=-0.227) blocked by oracle near-field degradation (worst cell 37.5%); requires oracle rewrite to proceed
- Pose repeatability (+-2 deg jitter) causes ~1.6 cm guaranteed clip-free depth variation; mitigated if YOLO uses joint-angle reprojection instead of fixed homography
