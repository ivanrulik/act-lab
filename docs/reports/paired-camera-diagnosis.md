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
