# Proposed next experiment: causal action labels

Status: specified; training not started. Extends ADR 029 and the paired pilot.
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
