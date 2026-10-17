# Intent labels and repeated safety filtering

Date: 2026-10-16. Extends the [paired diagnosis](paired-camera-diagnosis.md).
[Metrics](intent-label-diagnosis-metrics.json) preserve every diagnostic case and
the interrupted run record. These are simulation controller diagnostics, not a
new learned-policy evaluation or hardware qualification.

## Diagnostic question

Does an already-filtered action make a suitable target for an interface that
accepts Cartesian intent and filters it again? `SafeCartesianRobot._limit_pose`
uses the configured 100 ms pose-response horizon, in addition to motion limits.
The intermediate executed target is therefore different from the requested
intent. Correct temporal pairing does not make those quantities interchangeable.

## Oracle hold experiment

At each 25 Hz decision tick, the scripted expert submits its requested target
through shared safety. On the second 50 Hz safety tick, the experiment either
holds that request or substitutes the first tick's executed target. Both branches
use the same seed, physics, expert, episode budget, gripper and safety settings.
No learned model, rendering, safety relaxation or final-test seed is involved.
The applied branch still receives a normal requested command on the first tick:
this is a diagnostic intervention, not an exact oracle for learned v3 actions.

| Branch, diagnostic seeds 21000–21019 | Completed | Failure | Rejections |
| --- | ---: | --- | ---: |
| Hold requested intent | 16/20 | Four explicit IK failures during lowering | 4 |
| Hold first applied target | 0/20 | All 15-second timeouts during descent | 0 |

This identifies a representation risk independent of network regression error.
It does not prove a requested-label policy will succeed, establish a ceiling on
learned success, or qualify the 25 Hz scripted expert. The four IK failures stay
visible and require subsequent investigation rather than weaker safety limits.

## Interrupted training and revised protocol

The v3 alignment-only dataset converted successfully: 40 episodes, unchanged
30/10 split, 11,624 frames and verified distinct logical/storage fingerprints.
Its fresh 5,000-update CUDA run was explicitly interrupted at update **789** after
the oracle evidence. The manifest records `failed / KeyboardInterrupt`, with an
additional preemption reason. No final checkpoint exists. Automatic downstream
checkpoint stages failed their completion prerequisite and did not evaluate an
incomplete model. There is no learned v3 outcome to compare or hide.

[ADR 030](../adr/030-requested-intent-training-labels.md) adds explicit causal
requested-intent labels, preserving executed holds or reconstructing disabled
measured-pose holds from accepted aperture history for existing reports. The
[next bounded protocol](../plans/requested-intent-study.md) freezes source data,
model, seed, 5,000-update budget, rates and safety. Since the alignment-only run
is incomplete, this cannot separately estimate timing and representation effects.

## Reproduce

```bash
docker compose --profile gpu run --rm train-gpu \
  python scripts/probe-expert-target-hold.py \
  --output runs/new-oracle.json --seed-start 21000 --episodes 20
```

The probe refuses existing output, records all phases/failures and sends a
shared disabled hold on shutdown. Large training data/checkpoints stay ignored.

The requested-label implementation passed the dev build, Ruff, mypy (81 source
files), doctor, 433 dev tests and 23 real CPU learning/conversion tests. Ten
dev-profile skips are optional learning/display checks; learning checks were run
in the separate learning image. One existing multiprocessing warning remains.
Full verification covered all **11,624 frames** and **23,248 unchanged encoded
images**, with unchanged raw hashes and episode splits. Maximum action/state
float32 errors were 2.91e-8 / 2.38e-7. Forty raw holds were reconstructed using
accepted aperture history. New model outcomes remain pending.

```bash
docker compose --profile gpu run --rm train-gpu \
  python scripts/verify-requested-intent-study.py \
  --pilot runs/requested-intent-study --raw-pilot runs/paired-camera-pilot-v2 \
  --reference runs/causal-label-study/paired/dataset \
  --output runs/new-intent-verification.json
```

## Metric counter correction

During the requested trial, the original parser read `1K` as step `1`. The
pinned formatter also rounds 1200/1400 to `1K`; suffix expansion alone cannot
recover exact updates. This affects structured reporting, not optimizer steps.
The adapter now derives exact step/sample counts from pinned logging cadence and
resume offset, with regression coverage across that rounding boundary. A recovery
script preserves original logs/metrics and writes separate JSONL/provenance.
The interrupted run recovered exactly three records (200, 400, 600). The active
run still uses its original loaded code; recover it only after completion.

The reporting fix passed the dev build, Ruff, mypy, doctor, **435 dev tests** and
**12 CPU training/parser tests**, including all six tiny real train/resume cases.
Earlier full conversion checks remain valid. No model/config/safety change was
made during the requested trial.
