# Lean wrist-camera ACT study

Base: merged main `919b0cd`. Branch: `feature/wrist-camera-study`.

## Outcome

Decide whether the current articulated robot can use wrist RGB plus measured
joint/gripper state. Compare only wrist and wrist+fixed `policy` views. Overview
stays an optional debug view. No optics, control, safety, hardware or ROS changes.
The source review is summarized in ADR 028; UMI's fisheye results are not evidence
for the current 58-degree vertical pinhole view.

## Stages and success metrics

1. **Visibility preflight:** ten diagnostic seeds 19000–19009, original 320×240
   images and expert trajectories through `SafeCartesianRobot`. Sample at 5 Hz.
   Actual rendered geom-ID segmentation measures occlusion as well as framing.
   Visible means at least 16 target pixels. Cube visibility during approach and
   descent, tray visibility during transit and lowering, must each reach 95%
   in every seed. Every required phase must be sampled and the expert must
   finish successfully. Closing/held-object occlusion is reported separately;
   it is not an automatic failure. Report individual and union-of-two views.
   Segmentation and privileged geometry are audit-only, never policy inputs.
   A failed observation gate is a completed no-go assessment; do not train that
   variant, silently move the camera or relax thresholds after seeing results.
2. **Matched data:** collect expert seeds 20000–20039 once with wrist+policy,
   validate raw MCAP, freeze the manifest with 20% reserved episodes. Both
   conversions preserve IDs, split assignments, numeric samples, timestamps and
   selected camera bytes. Different camera selections have distinct dataset and
   calibrated checkpoint identities. Reject missing, duplicate or unknown views.
   Require at least 30 successful eligible demonstrations before the pilot.
3. **Bounded pilot:** explicit CUDA, seed 1000, batch 8, 5,000 updates per viable
   candidate, same ACT architecture and preprocessing. Save at update 5,000 only;
   report actual wall time and storage. No silent CPU fallback. Diagnostic policy
   seeds 21000–21019 never enter training or the final claim. No automatic
   extension to a 100,000-update run or parameter sweep.
4. **Frozen comparison:** after fixing both checkpoints/configs, evaluate each
   on seeds 22000–22099, 25 Hz policy / 50 Hz environment, same device and safety
   path. Require at least 90/100 successes for a candidate to qualify. Report
   Wilson intervals and deterministic paired bootstrap intervals, completion
   times, all failures, drops and rejected commands. Rejected commands are not
   executed safety violations. Wrist qualifies only if the lower 95% paired
   success-difference bound (wrist minus paired) is at least −0.05 and it has
   no additional drops or safety rejections. Inconclusive evidence must say so.
5. **Cost and publication:** report measured dataset bytes, RGB-render plus
   policy decision latency p50/p95/p99 and training wall time. The lean choice
   must reduce dataset size at least 25% and not increase decision p95. Latency
   is an offline measurement, not hardware real-time qualification. Publish only
   a qualified selected dataset/inference checkpoint through the existing Hub
   release workflow; keep diagnostic reports and representative failures.

## Implementation and tests

- Version camera presets and pilot training configuration.
- Add rendered visibility audit with bounded outputs, per-phase metrics and
  calibration/config provenance; reuse existing expert and safety composition.
- Add explicit camera projection during validated conversion, with original raw
  provenance and a distinct derived model/camera contract.
- Add a dedicated matched-camera comparator; preserve the stricter existing
  same-config comparison command. Reject differences beyond camera selection,
  mismatched episode splits, seed leakage and training/execution settings.
- Add end-to-end timing to the evaluator without changing action/control logic.
- Unit tests cover exact gate boundaries, missing phases, nonfinite metrics,
  mismatched trials and camera selection. Headless integration checks cover
  actual segmentation and unchanged simulator time/state across rendering.
- Run dev build, Ruff, mypy, pytest, doctor, and real conversion/model tests in
  the pinned learning image. Record actual gate outcomes; leave failed variants
  untrained. Keep large artifacts in ignored `runs/`, with an explicit 5 GiB
  acquisition/derived-data pilot budget and no duplicate full campaigns.

## Handoff

Deliver one draft PR with source-reviewed rationale, measured audit, executable
commands and exact pending gates. A visibility no-go does not establish that a
recurrent/actively observing policy could never work; it blocks this conservative
baseline. Any different optics, active search, policy architecture or relaxed
visibility requirement is a separate explicitly designed experiment.
