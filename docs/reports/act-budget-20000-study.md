# Paired ACT: 20,000-step simulation qualification

Date: 2026-10-16. **The frozen paired-camera simulation recipe meets its
predeclared gates:** 18/20 diagnostic successes and **90/100** on the audited
unused final cohort. [Full metrics](act-budget-20000-metrics.json) retain all
cases, failures, configuration, lineage, source/image identity, error probes,
training records, seed audit and frozen inference-file hashes.

## Result

| Cohort | ACT | Expert | ACT grasp / drop | ACT collision-stop commands |
| --- | ---: | ---: | ---: | ---: |
| Diagnostic 21000–21019 | 18/20 | 20/20 | 18 / 0 | 461 |
| Final 23000–23099 | **90/100** | 100/100 | 90 / 0 | **1,073** |

Final ACT 95% Wilson interval: **82.56–94.48%**. The threshold was point success
>=90/100, not a confidence lower bound or a robot safety certification. All ten
failures were 15-second timeouts before grasp. All ninety successful cases had
zero safety rejections. Collision-stop counts are rejected commands, including
repeated holds, not observed physical impacts.

| Failed final seed | Collision-stop commands | Grasp |
| --- | ---: | ---: |
| 23003 | 0 | 0 |
| 23015 | 256 | 0 |
| 23017 | 286 | 0 |
| 23024 | 244 | 0 |
| 23029 | 0 | 0 |
| 23042 | 0 | 0 |
| 23043 | 287 | 0 |
| 23056 | 0 | 0 |
| 23066 | 0 | 0 |
| 23096 | 0 | 0 |

Mean successful completion: 11.15 s, median 11.02 s. These condition on success;
the expert's different success set is not used to claim a speed advantage.
Final decision wall time, including RGB rendering: p50 8.32 ms, p95 8.81 ms,
p99 14.03 ms, maximum **528.04 ms**, over 28,856 calls. This offline benchmark
is not real-time qualification; cold/tail latency and independent inhibition
remain necessary before any live control integration.

## Frozen recipe and provenance

- Forty original expert episodes, exact 30 train / 10 reserved split, same seeds
  and source hashes. No recollection, optics changes or privileged policy input.
- Converter v4: current state/RGB paired with next accepted requested intent;
  rejected/disabled targets retain explicit/reconstructed holds and accepted
  aperture. All 11,624 frames and 23,248 encoded images were checked.
- Dataset fingerprint:
  `49e3f6f6fa7f016bef373cf4050b88f8b6c81905106332cdde70813291a2be04`.
- ACT ResNet-18, VAE, seed 1000, batch 8, 100-action training chunks, unchanged
  AdamW recipe and learning rate. CUDA required; AMP disabled.
- Staged official resume: 0 -> 5,000 -> 10,000 with the original loader, then
  10,000 -> 20,000 with explicit numeric-action projection. All optimizer/RNG and
  data-order state restored. Per-attempt source sidecars preserve actual Git/image
  identity; the initial manifest Git field is not mislabeled as resume identity.
- Inference: wrist + fixed policy RGB, 25 Hz, actual horizon one, fresh prediction
  every call, explicit temporal coefficient 0.01. Histories clear on episode/input
  faults. Shared `SafeCartesianRobot` at 50 Hz, unchanged limits/IK/collisions and
  15-second task budget.
- Qualified model digest:
  `4525c1dc57eb840ffe2090317813a3998e9fdad0f2ccc8a569a7d3c386986d6d`.
  All eight inference files matched before and after final evaluation.

The 10,000 extra updates took about **18 min 25 s** of the training loop.
The [isolated three-pair A/B](action-loading-gpu.md) established 4.55x median
steady throughput and 3.30x whole short-trial wall improvement, with exact model,
optimizer and RNG tensors. The full-run observation is reported separately from
that controlled throughput comparison.

Sampled first-action XYZ error median was 3.81 mm train / 4.27 mm reserved;
p95 12.73 / 13.59 mm. Gripper median error was 0.0084 / 0.0106, with p95
0.640 / 0.620. The fixed five-frame sampling includes terminal holds and is
descriptive, not a separate independent policy-selection benchmark.

## Cohort discipline and retained failures

Diagnostics reused 21000–21019 explicitly for selection. The earlier final cohort
22000–22099 remained consumed and was never reused as fresh evidence. Before
final execution, the audit checked 3,324 retained JSON files, relevant trace paths,
tracked metadata and 434 readable domain-MCAP provenance records; no execution
use of 23000–23099 was found. Planning-only preflight references were identified.
Other transport files and nine corrupt/empty fault fixtures were excluded and
listed in the audit. This is an audit of retained evidence, not a claim to recover
deleted historical artifacts.

The checkpoint, configuration and target were frozen before all 100 final cases.
No partial-score tuning, early stop or threshold change occurred. **23000–23099
is now consumed** and must not serve as a fresh cohort for a later candidate.

Earlier results remain: corrected v2 0/20, interrupted v3 at update 789, requested
5,000-step None 0/20, 5,000-step ensemble 3/20 and 10,000-step ensemble 8/20.
The 20,000-step diagnostic retained all eight older successes and gained ten.
No discarded trial or safety failure is hidden by the final qualification.

The unused scheduled 15,000-step checkpoint was removed only after qualification,
with hashes/retention record, reclaiming 619,599,740 bytes. Full 5,000/10,000/20,000
checkpoints, data, traces and failed evidence remain. No raw data, models,
checkpoints or large pixel artifacts were committed or uploaded.

## Reproduce retained-artifact replay

This command needs the locally retained frozen checkpoint. It is a **replay** of
an already consumed cohort, not a new qualification run:

```bash
docker compose --profile gpu run --rm train-gpu act-lab sim evaluate \
  --checkpoint runs/requested-intent-study/paired/training/lerobot/checkpoints/020000 \
  --device cuda --policy-hz 25 --temporal-ensemble-coeff 0.01 \
  --config configs/sim/ur5e_2f85_d405_paired.toml \
  --seed-start 23000 --episodes 100 --no-video --trace-actions \
  --output runs/new-qualified-replay
```

Exact replication needs the frozen selection/data and staged resume evidence;
a fresh collection/retraining is a new experiment. No new Hub release was made.

## Validation and handoff

Dev build, Ruff, mypy (82 source files), doctor, 439 dev tests and 37 real CPU
learning/conversion/parser checks passed. Fourteen optional dev skips and one
existing multiprocessing warning are explicit; learning checks ran separately.
Actual CUDA work covered all training stages, three matched GPU A/B pairs,
complete diagnostics and the 100-case final cohort. All nine hosted checks passed
on the prior pushed implementation; final report-only push status is tracked on
PR #30.

Qualification applies to this frozen paired-image simulation distribution.
Wrist-only sufficiency, broader scene/robot generalization, live timing, ROS
learned motion, hardware, and a new curated Hub release remain separate work.
