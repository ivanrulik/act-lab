# ACT policy coverage experiment

This follow-up tests whether broader scripted demonstrations improve ACT's
closed-loop grasp reliability. It uses the existing simulation, safe action
path, MCAP validation, LeRobot conversion, training CLI, and paired evaluator.
It does not change the camera layout, controller, policy architecture, or ROS
boundary.

## Baseline and seed protocol

The clean PR 9 checkpoint is
`runs/training/act-gpu-clean-retry/lerobot/checkpoints/last`. On diagnostic
seeds 10000–10019 it succeeded 16/20 times after the gripper output clamp.
Those seeds have informed this experiment and are a regression set, not an
untouched final holdout. The original training demonstrations used seeds
200–219, with 19 successful validated episodes, 14 assigned to train.

Use seeds 300–359 for the new expert cohort. Keep both evaluation ranges
10000–10019 and 11000–11099 out of all training demonstrations. Fix the
fresh final comparison to 11000–11099 before viewing its results. Run both
checkpoints on exactly those 100 seeds with the same simulation and policy
execution settings. The comparison command rejects mismatched seeds,
simulation configuration, safety path, or execution frequency.

The simulator samples cube X from `[0.35, 0.52]` m and Y from
`[-0.22, -0.08]` m. The original 14 training starts occupied 7/16 cells of a
4×4 XY grid. Seeds 300–359 span all 16 cells before quality filtering. The
coverage report must be inspected again after selection; a generated seed is
not automatically a usable training demonstration.

## Collect, validate, and convert

Run every command from the repository root. Compose owns the runtime. Paths
under `data/` and `runs/` are ignored, and existing datasets/runs must not be
overwritten.

```bash
docker compose run --rm dev act-lab experiment coverage \
  --manifest data/selection-pr9-clean.json \
  --output runs/experiments/act-coverage/baseline-coverage.json

docker compose run --rm dev act-lab sim expert \
  --seed-start 300 --episodes 60 --record-dir data/raw --json \
  > runs/experiments/act-coverage/expert-300-359.json

docker compose run --rm dev act-lab recording manifest data/raw/*.mcap \
  --include-source expert \
  --include-seed-range 200:20 --include-seed-range 300:60 \
  --validation-fraction 0.2 --split-seed 0 \
  --output data/selection-act-coverage.json

docker compose run --rm dev act-lab experiment coverage \
  --manifest data/selection-act-coverage.json \
  --output runs/experiments/act-coverage/candidate-coverage.json

docker compose --profile data run --rm data act-lab recording convert \
  data/selection-act-coverage.json \
  --output data/lerobot/pick-place-act-coverage \
  --repo-id local/act-lab-pick-place-act-coverage --fps 25
```

Review the expert report for failures, the manifest for quality rejection and
train/validation assignment, and the candidate coverage grid before training.
The manifest retains unselected and rejected candidates with their reasons.
The coverage command verifies selected raw hashes and checks recorded seed,
source, and spawn bounds. Its grid is descriptive; it is not a success metric.

## Train and compare

This is a new full training run, not a resume of the existing checkpoint.
The current production configuration is 100,000 steps and previously took
about 17 hours on the local GPU. CUDA requests fail rather than fall back to
CPU.

```bash
docker compose --profile gpu run --rm train-gpu \
  act-lab train act --dataset data/lerobot/pick-place-act-coverage \
  --output runs/training/act-gpu-coverage --device cuda

docker compose --profile gpu run --rm train-gpu \
  act-lab sim evaluate \
  --checkpoint runs/training/act-gpu-clean-retry/lerobot/checkpoints/last \
  --device cuda --seed-start 11000 --episodes 100 \
  --output runs/evaluation/act-baseline-11000

docker compose --profile gpu run --rm train-gpu \
  act-lab sim evaluate \
  --checkpoint runs/training/act-gpu-coverage/lerobot/checkpoints/last \
  --device cuda --seed-start 11000 --episodes 100 \
  --output runs/evaluation/act-coverage-11000

docker compose run --rm dev act-lab experiment compare \
  --baseline runs/evaluation/act-baseline-11000/evaluation.json \
  --candidate runs/evaluation/act-coverage-11000/evaluation.json \
  --output runs/experiments/act-coverage/comparison-11000.json
```

The comparison reports each policy's Wilson 95% success interval, the paired
success-rate difference with a deterministic 95% bootstrap interval, per-seed
improvements and regressions, completion time among successes, and aggregate
grasp/drop/limit/rejection events. Review the evaluation videos and individual
failure records as well. A higher training success rate or lower training loss
alone does not establish improvement. Report an inconclusive or negative result
as such; do not add fresh-holdout seeds to the training set after seeing them.

Only after the fresh comparison, rerun seeds 10000–10019 for the regression
check. The PR 10 clean-clone release is a separate roadmap item.

## Local result, 2026-10-05

The seed 300–359 expert cohort produced 59 successes and one failure (316).
The frozen selection contains 78 quality-eligible successful expert episodes:
58 train and 20 validation. Its 25 Hz LeRobot dataset has 16,118 frames and
logical fingerprint
`e00ef486fa0957b8aa939e009c8c5e260223109ab591b5693ce08cda3d2e4a87`.
Training starts occupy 15/16 XY grid cells, versus 7/16 for the baseline.
The new CUDA run completed 100,000 steps in about 27 hours 15 minutes and has
a distinct checkpoint. The compared checkpoint SHA-256 values are
`30af9368390b5ea0729e23bbb8403e497d96dbb0967f8bb1a5ec8e72c7d8c7c3`
(baseline) and
`d54662b9488d57b398822da73f4b163bdfa4f7d05070b5404b3d058e67e00743`
(coverage).

On the untouched comparison seeds 11000–11099, both checkpoints used the same
25 Hz policy cadence, 50 Hz simulation rate, configuration, and safety path:

| Metric | Baseline ACT | Coverage ACT |
| --- | ---: | ---: |
| Successes | 67/100 | 96/100 |
| 95% Wilson success interval | 57.3–75.4% | 90.2–98.4% |
| Mean completion time among successes | 8.46 s | 8.66 s |
| Grasp events | 72 | 96 |
| Drop events | 3 | 0 |
| Safety rejections | 8 collision stops | 41 IK failures |

The paired success difference is **+29 percentage points**, with a
deterministic paired bootstrap 95% interval of **+20 to +38 points**. The
candidate succeeds on 29 seeds where the baseline fails and regresses on none
for task success. The scripted expert succeeds on 99/100; its seed 11058 fails
with an IK error. The completion-time means include different successful
subsets and are descriptive, not a paired speed comparison.

The candidate fails to grasp on seeds 11008, 11012, 11058, and 11062, all by
timeout. They start near the high-X, low-Y side of the cube spawn area. Their
action traces contain 9, 10, 1, and 16 rejected IK commands, respectively.
The remaining 5 IK rejections occur in successful candidate episodes. The
safety controller rejects these commands; they are not executed. The higher
IK rejection count is a real regression in target feasibility and needs a
separate diagnosis before treating this checkpoint as a default policy.

After the fresh comparison, the original diagnostic seeds 10000–10019 were
rerun: coverage ACT succeeds on 19/20 versus the baseline's earlier 16/20.
Seed 10007 still times out without a grasp; this regression-set run has zero
safety rejections.

Local, ignored artifacts are at:

- `data/selection-act-coverage.json` and
  `data/lerobot/pick-place-act-coverage/`;
- `runs/training/act-gpu-coverage/`;
- `runs/evaluation/act-baseline-11000/` and
  `runs/evaluation/act-coverage-11000/`, including videos and failure records;
- `runs/experiments/act-coverage/comparison-11000.json`;
- `runs/evaluation/act-coverage-regression-10000/` and
  `runs/evaluation/act-coverage-trace-{11008,11012,11058,11062}/`.

These recordings, datasets, checkpoints, and reports remain outside Git under
the repository's data policy. Their publication and clean-clone workflow
belong to PR 10.
