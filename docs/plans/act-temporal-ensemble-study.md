# Frozen-checkpoint temporal ensemble diagnostic

Status: predeclared before implementation and evaluation.

The v4 requested-intent checkpoint completed 5,000 CUDA updates. Horizon-one
inference completed 0/20 diagnostic episodes: no grasps/rejections, all timeouts,
with substantially deeper approach/descent than v2. Sampled first-action XYZ
median error is 18.46 mm train / 18.54 mm reserved; p95 decision time 8.78 ms.
The final cohort remains untouched.

The pinned ACT implementation includes temporal ensembling of overlapping chunks.
It predicts a fresh chunk every call and combines predictions for the current
time, rather than consuming a queued chunk without new observations. Its source
records 0.01 as the original ACT coefficient. This may improve continuity and
use of trained future predictions; it is a hypothesis, not established benefit.

- Freeze all eight inference files and checkpoint digest from the requested run.
- Change only inference temporal_ensemble_coeff: None -> 0.01.
- Preserve n_action_steps=1, fresh prediction every policy tick, 25 Hz decisions,
  50 Hz safety, images, input validation, task budget and seed cohort 21000–21019.
- Reset ensemble history at episode boundaries. Test real checkpoint loading,
  two calls/two predictions, resolved coefficient and history reset.
- One coefficient, one 20-seed diagnostic run, no retraining or sweep. Report all
  outcomes, events, errors and latency. Require >=18/20 before final qualification.
- Keep the original None result and all traces. Do not use consumed 22000–22099;
  only a passing frozen candidate may use an audited fresh 100-seed final cohort.
