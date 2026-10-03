# Grasp v24 E1 replacement candidate

This directory stages the training-side answer to the v23 E1 weak cells. It is a
**candidate**, not the production grasp runtime. The delivered formal rows recount to
230/235 successes (97.87%), including 15/15 at v23's old `(0.280, -0.070)` hotspot.

`run_candidate.py` deliberately reuses `grasp/v23/x3plus_real_grasp.py`. V24 changes
the PPO/VecNormalize pair, while its 28D/6D incremental contract, E1 home, URDF and
workspace envelope remain the same. The wrapper locks all four identity inputs:

- model
- VecNormalize
- `obs_28_incremental`
- `grasp/v24/manifest.json`

It rejects command-line attempts to substitute any of them. This keeps one hardened
implementation of serial, encoder, jaw-contact, FloorGuard and shutdown behavior.
It also fixes the previously validated E1 controller profile (10 mm entry gate,
15 mm finger correction, 20 mm TCP correction, 0.5 jaw tracking, 15 degree lag
ceiling and two blocked samples). This carries forward measured geometry and control
settings so acceptance testing changes only the policy pair; it does not carry
forward v23's success result to the new hash.

## Safe checks

These commands do not open the serial port or move the robot:

```bash
python3 grasp/v24/test_candidate_package.py
python3 grasp/v24/run_candidate.py --help
```

Dry-run inference still loads the model and dependencies, so perform it on the Jetson
only after the static package test succeeds:

```bash
python3 grasp/v24/run_candidate.py \
  --obj-x 0.24 --obj-y 0.0 --obj-z 0.02 --max-steps 0
```

## Real hardware status

`--real` is blocked by candidate status unless the operator deliberately adds
`--unlock-candidate-real`. The override does not waive artifact hashes or contract
identity. Keep a person beside the robot and follow the v23 physical checklist before
using that override.

This exact hash has not yet passed Jetson load, dry-run, deployment-controller parity,
fixed-coordinate grasp, or visual E1 3/3. It cannot inherit v23's prior 3/3 result.
The formal evaluator also used simulator magnet/contact behavior, and 97/235 recorded
episodes went below the deployment guard's 8 mm clearance. Do not reduce that guard.

Original provenance and the full row-level evaluation are in `evidence/`. The intake
analysis is `docs/planning/v24_intake_2026-09-07/INTAKE_REVIEW.md`.
