# X3Plus grasp v24 E1 replacement candidate

## Release status

- Run: `v24_e1_corner_recovery_seed23404_r3`
- Training seed: `23404`
- Selected candidate: targeted DAgger BC update `10`
- Simulation selection gates: **PASS** on full dev seeds `6242` and `6343`
- Held-out formal gate: **PASS** on seed `90410`
- New-weight physical robot validation: **PENDING**
- May replace v21 in production: **after the packaged model/VecNormalize pair passes the deployment-side physical acceptance test**

The earlier v23 physical result cannot be inherited by this new weight hash. This package is therefore a
fully traceable, formally selected replacement candidate, not evidence that the new weights have already
completed the deployment-side 3/3 test.

## Locked artifacts

- Model SHA-256: `0ed9989154ed280971fd4499c8927c198539f37915f6d9191f9cd08eaf100d81`
- VecNormalize SHA-256: `6ebb5a64ef967a1fcfca522fe330f71e0eef3f95b452c78a8248b4edb74ecd26`
- Selection lock SHA-256: `6ccb27de7e38e2b7639dad0a67b7a0d2263e1601719de0e906322e40ab937f8e`
- Formal claim SHA-256: `8fb31509f5e17c58657a425baa57415804b4df92d88d88622da6cd064de59ed2`
- Formal evaluation SHA-256: `308628a15629eb1367d8b576eea4ff8ff8c141ff77e801a5181090e2c8feeaa1`

The formal claim was created before the first formal episode and identifies exactly the model and
VecNormalize hashes above. Do not mix either file with another release.

## Formal result

- Episodes: `230/235` (`97.87%`)
- Bottle cap: `25/25` (`100%`)
- Mixed height/shape: `75/75` (`100%`)
- Worst grid cell: `13/15` (`86.67%`, threshold `80%`)
- Floor penetration episodes: `0`
- Floor guard interventions: `0`
- Case plan valid: `true`
- Integrity unchanged: `true`

Grid rates:

| x | y | success |
|---:|---:|---:|
| 0.2050 | -0.0700 | 86.7% |
| 0.2050 | -0.0025 | 100.0% |
| 0.2050 | +0.0650 | 100.0% |
| 0.2425 | -0.0700 | 100.0% |
| 0.2425 | -0.0025 | 100.0% |
| 0.2425 | +0.0650 | 93.3% |
| 0.2800 | -0.0700 | 100.0% |
| 0.2800 | -0.0025 | 100.0% |
| 0.2800 | +0.0650 | 86.7% |

The original v23 hotspot `(0.280, -0.070)` was rerun under the new held-out protocol and achieved
`15/15` (`100%`).

## Why update 10

All six preregistered updates were screened. The fixed shortlist was updates `20`, `3`, `10`, and `1`.
Only update `10` passed every gate on both full development seeds, so it was the only eligible model;
the formal seed was not used for selection. The complete ranking, failed candidates, and per-episode rows
are retained in `evidence/candidate_registry.json` and `evidence/development/`.

## Reproducibility

Training ran from a dirty Git worktree. The commit hash alone is not sufficient to reproduce this run.
`evidence/training_metadata.json` records `git_head`, full `git_status_porcelain`, protocol and dependency
hashes, parent model/VecNormalize hashes, package versions, task configuration, and `source_sha256`.
`source_bundle/` contains byte-for-byte snapshots of all required training sources, including the six
canonical v21 manifest files. Verify `SHA256SUMS.txt` before deployment.
