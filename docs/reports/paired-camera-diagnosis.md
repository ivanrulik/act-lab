# Paired ACT diagnosis — action-horizon loader defect and feedback stall

Date: 2026-10-16. Same frozen 5,000-update checkpoint and dataset as the
[pilot](paired-camera-pilot.md). All replay work used diagnostic seeds
21000–21019. The final cohort was not replayed or used for tuning.
[Metrics](paired-camera-diagnosis-metrics.json) retain both per-seed evaluations,
geometry, checkpoint digest and sampled first-action error summaries.

## Confirmed integration defect

The adapter called `ACTPolicy.from_pretrained(..., n_action_steps=1)` and the
evaluator reported one action per inference. Pinned LeRobot 0.6.1's config loader
actually consumes `cli_overrides`; ordinary keyword arguments do not override
serialized config fields. Direct inspection of the real pilot checkpoint
resolved **n_action_steps=100, chunk_size=100, action_queue.maxlen=100**.

LeRobot predicts a chunk only when its queue is empty. At 25 Hz the old adapter
therefore predicted once per 100 calls, potentially four seconds apart. Capturing
fresh images and retimestamping queued targets did not make those predictions
fresh. Numerical safety still bounded every command, but the declared feedback
semantics were incorrect. The original 33/100 result and latency measurements
are historical measurements of this unintended chunk execution, not evidence
for the declared one-action feedback protocol.

The adapter now loads an explicit config with device and action-horizon CLI
overrides, verifies the effective horizon, and reports the resolved value. A
real checkpoint regression test starts from saved horizon two, requests one,
and proves two calls invoke two fresh chunk predictions. Both wrist and paired
calibrated inference cases pass. This enforces the existing experiment contract;
it does not change safety limits, training weights, architecture or cameras.

## Controlled replay: fixing the bug does not qualify the policy

| Same 20 diagnostic seeds | Original queue 100 | Corrected queue 1 |
| --- | ---: | ---: |
| ACT successes | 6/20 | 0/20 |
| Expert successes | 20/20 | 20/20 |
| Grasps / drops | 9 / 0 | 0 / 0 |
| Safety rejections | 727 | 0 |
| Decision latency p95 | See metrics | 9.374 ms |

The original replay reproduced the same six successful seeds. Its rejection
count differed by one from the initial run (727 versus 726); exact floating-point
replay is not claimed. Initial requested positions match exactly between the
original and corrected traces on every seed, and checkpoint digests match.
The action-horizon intervention is the changed execution factor.

Corrected runs all timed out while open, with final TCP heights **0.588–0.614 m**;
nominal grasp TCP height is cube center 0.425 m plus the 0.030 m tool offset.
None entered the approximate grasp region (XY error <15 mm and TCP height less
than cube center +50 mm). Largest last-second TCP excursion across all twenty
runs was below **0.0001 mm**. Requested targets remained close to the measured
pose, so this is a steady pre-grasp feedback stall, not safety preventing
otherwise successful motion. The diagnostic region is an analysis heuristic,
not a replacement task-success definition.

Original chunk execution reached that approximate region on 14/20 seeds. It
could progress through future actions without repeatedly asking the weak first
prediction to advance the task. This explains why the accidental execution mode
can outperform the corrected feedback loop; it does not justify keeping the
loader defect or silently changing the freshness contract.

## First-action precision on actual dataset frames

A separate reproducible probe samples five uniform-index frames from each of
40 frozen episodes: 150 training samples and 50 reserved samples. It supplies
recorded state/RGB through the actual saved preprocessing, predicts with the
inference latent, reverses action normalization and compares the first predicted
action against the same row's recorded action. No task geometry is a model input.

| Error | Training median / p95 | Reserved median / p95 |
| --- | ---: | ---: |
| XYZ Euclidean | 12.42 / 27.69 mm | 18.96 / 27.18 mm |
| Normalized gripper absolute | 0.054 / 0.595 | 0.055 / 0.551 |

The probe reproduced exactly on rerun. These are descriptive samples, not a
full-dataset validation loss or an independent generalization benchmark. The
first action is already imprecise on training examples. Combined with the
closed-loop fixed point, this supports insufficient first-action/phase-transition
learning as a remaining problem. It does **not** identify a unique cause or prove
that increasing training budget alone will solve it. Image use, data alignment,
VAE behavior and training convergence remain candidates for a controlled study.

The corrected p95 decision time is below the 40 ms policy period in this offline
run; the dominant observed task failure is not an inference throughput limit.
Maximum latency and all quantiles remain in the JSON; this is not a real-time
qualification. No new training, camera movement or safety relaxation ran.

## Reproduce

Run the existing evaluator with `--trace-actions --no-video`, the frozen paired
config, CUDA, 25 Hz, seed start 21000 and 20 episodes. Select a new output path.
The corrected adapter enforces horizon one; historical baseline artifacts are
retained under ignored `runs/paired-camera-diagnosis/traced`.

```bash
docker compose --profile gpu run --rm train-gpu \
  python scripts/probe-act-first-action.py \
  --pilot runs/paired-camera-pilot-v2 \
  --output runs/paired-camera-diagnosis/new-probe.json
```

The probe deliberately uses the pilot's fixed artifact layout. It refuses an
existing output. Large traces, data and checkpoints remain ignored.

## Next bounded decision

Preserve one-step feedback and diagnose learning on the frozen data before a
new qualification campaign. Predeclare one intervention and its budget, inspect
only training/reserved data and diagnostic seeds, then use a new untouched final
cohort. Neither camera superiority nor a useful fresh-feedback policy has been
established. The saved legacy release also has horizon 100; historical replay
claims using this loader need an effective-horizon audit, not assumed one-step
semantics. No legacy model was retrained or requalified here.

## Validation

Dev image build, Ruff, mypy (81 source files), doctor, documentation links and
diff checks passed. Full dev suite: 417 passed, six expected optional skips.
The pinned CPU learning image passed all four real training/resume cases,
including the new fresh-prediction checks for wrist and paired observations.
Actual CUDA runs covered both complete twenty-seed diagnostic cohorts and the
200-frame dataset probe, which reproduced exactly. No final-cohort replay or
additional training ran. Hosted CI for this correction is tracked on PR #30.

## Continued diagnosis: causal label timing

The raw archive correctly stores a command's **resulting** state and synchronous
RGB. Reading/conversion v2 pairs that resulting observation with the command
that has already been applied. The requested command timestamp precedes the
stored observation by exactly 20 ms throughout all 40 pilot episodes. This is a
causal supervision mismatch for the declared observe-then-command feedback loop.
It is not a corruption of the raw recording or a camera coordinate change.

Across 23,189 enabled adjacent samples, 19,999 moved more than 0.1 mm. For those
moving samples, median target-to-post-state distance was **0.170 mm**; median
target-to-pre-state distance was **1.600 mm**. Median actual motion was 1.632 mm.

On the exact 200 earlier probe frames, a diagnostic identity baseline targeting
the recorded current end-effector pose gives:

| XYZ error median | Training samples | Reserved samples |
| --- | ---: | ---: |
| Identity baseline, original labels | 0.059 mm | 0.104 mm |
| Learned first prediction | 12.424 mm | 18.961 mm |
| Identity baseline, next raw command labels | 1.600 mm | 1.600 mm |

The identity baseline uses recorded end-effector pose for analysis. The policy
receives joint angles and gripper state, not an explicit end-effector XYZ input;
kinematics relate those features. These numbers demonstrate that the old labels
reward predicting an already-reached pose. They do not prove the network's
internal strategy, or prove that corrected labels alone will solve the task.
A very low same-sample regression error would not establish useful task motion.

### Phase handoffs

For diagnostic seeds 21000 and 21001, the expert first reached each of eight
phases, then handed over to the unchanged one-action policy for the remaining
original 750-step episode budget. The phases were open/raise, approach, descend,
close/dwell, lift, transit, lower, and open/dwell. **All 16 conditional rollouts
failed to finish.** These are deliberately selected diagnostic conditions, not
sixteen independent full-policy trials or a new generalization benchmark.

Stalling therefore persists when the expert provides intermediate task states;
it is not confined to the initial opening step. Phase labels/task geometry were
used to arrange the handoff, never provided as learned-policy inputs. Every
command retained the same application safety. Per-case geometry/rejections are
in the metrics file.

### Versioned correction and verification

[ADR 029](../adr/029-causal-derived-action-alignment.md) adds explicit
`--action-alignment next-command` (converter v3). Labels shift by **one raw
command before resampling**, preserving current state/RGB/timestamps and
per-episode boundaries. The final unlabeled observation is dropped. Legacy
same-sample conversion remains available for historical reproduction.

A real two-episode subset produced **572 frames**. Every numeric action matched
the next raw command after resampling; every current state and decoded RGB image
matched the original observation. Raw hashes were unchanged, and dataset
verification retained one train and one reserved episode. Its distinct v3
fingerprint is in the metrics. No pilot-scale checkpoint was trained on these new labels. Tiny CPU
training/resume tests establish mechanics only.

The next [controlled alignment study](../plans/causal-action-alignment-study.md)
changes labels alone at a frozen training budget. Learning remains unqualified
until that experiment establishes closed-loop improvement on corrected feedback.

### Reproduce these probes

```bash
docker compose run --rm dev python scripts/probe-raw-action-alignment.py \
  --pilot runs/paired-camera-pilot-v2 --output runs/new-alignment-audit.json

docker compose --profile training run --rm train-cpu \
  python scripts/probe-act-identity-baseline.py \
  --pilot runs/paired-camera-pilot-v2 \
  --probe runs/paired-camera-diagnosis/teacher-probe-reproduced.json \
  --output runs/new-identity-probe.json

docker compose --profile gpu run --rm train-gpu \
  python scripts/probe-act-phase-handoff.py \
  --pilot runs/paired-camera-pilot-v2 --output runs/new-phase-handoffs.json
```

Raw alignment and matched identity summaries reproduced exactly. Causal
conversion/learning checks passed 17 tests in the pinned CPU image. Full dev:
422 passed, eight expected optional skips; build, Ruff, mypy and doctor passed.
