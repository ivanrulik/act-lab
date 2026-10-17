# Requested-intent ACT: bounded 10,000-update continuation

Status: completed; diagnostic gate failed (8/20).
Predeclared before resume.

The frozen 5,000-step model with ensemble coefficient 0.01 completed 3/20
(21000, 21005, 21007), versus 0/20 without ensembling. It recorded three grasps
and 827 collision stops; the 18/20 diagnostic gate failed. Original checkpoint,
all failures and None/0.01 results stay intact; final seeds remain untouched.

Training L1 continued decreasing across the final 1,000 updates (0.079 -> 0.068),
and sampled train/reserved XYZ errors were similar (18.46 / 18.54 mm). This
supports testing additional optimization as a hypothesis; it does not prove
underfitting or guarantee improvement. Loading remains the dominant wall-time
component; no data-loader or hardware configuration changes are bundled here.

1. Snapshot the completed 5,000-step manifest and all eight inference hashes.
2. Verify unchanged v4 dataset/storage fingerprint and exact episode split.
3. Resume the official pipeline from step 5,000 to **10,000**, preserving optimizer
   and RNG state, seed, batch, learning rate, architecture, cameras, dataset,
   normalizers, logging/save cadence and CUDA requirement. One bounded extension.
4. Record an explicit resume sidecar with current Git/container identity, source
   checkpoint/dataset fingerprints, resolved configuration and command. Preserve
   original manifest source identity and completed-manifest snapshot. The current
   manifest lacks automatic per-attempt source journaling; do not treat its initial
   Git identity as the resume-code identity.
5. Evaluate the new checkpoint on diagnostic seeds 21000–21019 with unchanged
   coefficient 0.01, horizon one, fresh predictions, 25/50 Hz policy/safety and
   15-second task budget. Compare with frozen 5,000-step ensemble result. Report
   all failures, grasp/drop/collision events, latency and sampled prediction error.
6. Require >=18/20 to advance to an audited fresh 100-seed final qualification
   (candidate 23000–23099, target >=90/100). Otherwise diagnose the remaining
   gap; no automatic extension, coefficient sweep or final-cohort reuse.

Reuse existing data. Combined checkpoints, traces and data must remain below
5 GiB for this run; do not copy raw episodes. No hardware or publication approval.

Outcome: 8/20 successes and grasps, zero drops, 1,469 collision-stop commands.
Median sampled XYZ error: 6.06 mm train / 8.09 mm reserved. No final seeds used.
See the [report](../reports/act-budget-10000-study.md) and conditional
[next bounded protocol](act-budget-20000-study.md).
