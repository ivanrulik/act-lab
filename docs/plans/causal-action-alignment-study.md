# Causal action-label experiment

Status: preempted at update 789 after a diagnostic oracle exposed an action-meaning mismatch. No completed checkpoint or learned-policy result.
Extends ADR 029 and the paired pilot.
The only experimental factor is action-label alignment. Preserve the corrected
one-action feedback loop, source episodes, camera pair, model, safety and gains.

1. Reuse the frozen 40-episode MCAP selection (20000–20039), with the same
   30 train / 10 reserved episode IDs. Convert at 25 Hz with wrist + policy and
   `--action-alignment next-command`. Store a separate dataset/run root; require
   a v3 namespace/fingerprint and exact numeric/image lineage checks. Raw data
   stays in its existing location. Terminal-frame trimming is part of this factor.
2. Freeze the existing ACT pilot config: seed 1000, batch 8, 5,000 updates,
   unchanged initialization/architecture, explicit CUDA. Train from scratch,
   rather than resuming a v2 checkpoint against different data. One final save,
   no automatic extension or sweep. Keep the 5 GiB per-experiment soft gate.
3. Report first-action error against causal labels, including the zero-motion
   identity baseline. These descriptive losses are not success gates by themselves.
4. Evaluate fixed diagnostic seeds 21000–21019 with verified horizon one and
   fresh prediction each policy tick. Compare with the frozen corrected v2
   baseline (0/20): full task success, first grasp, timeouts, drops, rejections,
   individual failures and latency. At least 18/20 successes advances to final
   qualification; otherwise finish the assessment and diagnose without consuming
   another final cohort. This is a proposed predeclared engineering gate.
5. Before executing the final stage, check evaluation manifests and reserve an
   unused 100-seed cohort (candidate 23000–23099). Final target remains >=90/100,
   with Wilson intervals, events and all failures. Consumed 22000–22099 is not
   a fresh test. Freeze the checkpoint/config before final evaluation.

The current checkpoint has not been improved by changing conversion alone.
A negative result would separate a necessary timing correction from remaining
learning/convergence/action-representation issues. Any later budget, loss or
representation change is a separate predeclared factor. Publication and hardware
use require their own existing gates.

## Iteration discipline

Continue with one predeclared factor per iteration. Preserve diagnostic and final
cohort separation, measured safety decisions, source/dataset/checkpoint identity
and a 5 GiB soft artifact gate per experiment. Reuse raw data and converted data
where unchanged. A failed diagnostic gate starts another hypothesis and bounded
protocol; it does not trigger a blind budget extension or final-seed reuse.
Stop the improvement loop when a frozen candidate qualifies or progress requires
external information. Training, data and model changes remain inspectable in
individual run manifests and the report.

## Active run record

- Reused the original selection and verified identical episode split manifests.
- v3 logical fingerprint:
  `0a9a44a9ab5e3d4cba261d1cac2bf630786dd4a0c7496182604b18458f40758a`.
- Verified storage fingerprint:
  `083c427ad1de71f2e781b3335abfb9d7ec3092a2bf4c8c2f8f32443e1331e559`.
- 11,624 total frames; 8,789 training frames; 662,515,256 dataset bytes.
- Fresh CUDA training started from revision `66b0f92`, using the frozen config.
  Its manifest records effective device, dependencies, dataset and source identity.
- The identity probe now supports explicit causal lineage and a separate raw
  source root. It reproduced the old 200-frame summaries exactly and verified
  200 causal labels against their next raw command. It rejects a raw selection
  that differs from the dataset's embedded selection.
- Training was interrupted explicitly (manifest: failed / KeyboardInterrupt).
  Automatic checkpoint-dependent stages stopped as required; none ran.
- The 20-seed oracle yielded 0/20 first-applied holds versus 16/20 requested
  holds, with four reported IK failures. See ADR 030 and the next
  [requested-intent experiment](requested-intent-study.md).
