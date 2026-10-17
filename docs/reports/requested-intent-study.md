# Causal requested-intent ACT study

Date: 2026-10-16. The model is **not qualified**. The predeclared diagnostic gate
failed, so no new final-test cohort was consumed. Full [metrics](requested-intent-study-metrics.json)
include all episodes, first-action probe rows, geometry, recovered counters,
frozen inference-file hashes, configuration and runtime identity.

## Frozen experiment

Reused the original 40 expert episodes and exact 30/10 split. Converter v4 uses
causal accepted requested intent and explicit/reconstructed disabled hold targets;
all 11,624 states/labels and 23,248 encoded input images were verified. Camera
pair, architecture, initialization seed 1000, batch 8, 25 Hz decisions, 50 Hz
application safety and 15-second task budget stayed fixed. No changed-data resume.

Fresh CUDA training completed all **5,000 updates** in about **44 minutes**.
Final displayed L1 was 0.068, KL 0.019. The eight inference files were frozen;
weights and normalizer are unchanged across subsequent inference experiments.
The manifest records source `6ab7c17`; the later logging correction does not
change this model's source identity or weights. Separate recovered metric records
retain exact steps 200–5000 while preserving original logs and compact counters.

## Diagnostic results, seeds 21000–21019

| Result | Expert | ACT, fresh horizon one |
| --- | ---: | ---: |
| Task success | 20/20 | **0/20** |
| Grasp events | Reported per case | **0** |
| ACT safety rejections | — | **0** |
| ACT limited commands | — | 15,000 |

All ACT failures were 15-second timeouts; the 95% Wilson interval is
0–16.11%. Each case performed 375 fresh predictions. Decision time including
RGB rendering: p50 8.26 ms, p95 8.78 ms, p99 12.23 ms, maximum 301.17 ms.
These offline measurements do not establish real-time guarantees.

Final tool heights were 0.485–0.521 m, compared with approximately 0.588–0.614 m
for the corrected v2 checkpoint. The new model approaches and descends farther,
but does not grasp. It is not motionless: the median final-second tool range is
13.3 mm, with vertical oscillation and no sustained completion of descent.

Five uniformly sampled frames per episode gave median first-action XYZ error
18.46 mm (train) / 18.54 mm (reserved); p95 44.16 / 55.77 mm. Corresponding
zero-motion identity errors are about 53.38 / 53.24 mm. Median gripper errors
were 0.072 / 0.079, with p95 near 0.71 / 0.70. These descriptive sampled errors
are not rollout success or an unbiased policy-selection benchmark.

## Interpretation and next controlled factor

Corrected intent semantics produce more useful motion, but 5,000 updates do not
establish a useful policy. The interrupted v3 run supplies no matched trained
comparison, so timing and action-source effects cannot be separated causally.
Source changes, camera count and hardware usefulness remain unqualified.

The next [frozen-checkpoint diagnostic](../plans/act-temporal-ensemble-study.md)
tests one inference-only coefficient (0.01), using fresh predictions and unchanged
safety/rates. No blind training extension, final-seed reuse or hardware approval
follows from the falling loss. Default ensembling remains disabled.
