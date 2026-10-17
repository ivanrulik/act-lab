# Frozen-checkpoint temporal ensemble diagnostic

Date: 2026-10-16. The diagnostic gate **failed**. Full
[metrics](act-temporal-ensemble-metrics.json) retain every episode and safety
outcome. The experiment changed only the inference ensemble coefficient to
0.01 on the frozen 5,000-step requested-intent checkpoint.

| Diagnostic seeds 21000–21019 | None | Coefficient 0.01 |
| --- | ---: | ---: |
| Success | 0/20 | **3/20** |
| Grasp events | 0 | 3 |
| Collision stops | 0 | **827** |
| Drops | 0 | 0 |
| p95 decision time | 8.78 ms | 8.75 ms |

Successful seeds: 21000, 21005, 21007. Ensemble Wilson 95% interval: 5.24–36.04%.
Mean successful completion was 10.02 seconds. All seventeen remaining cases
timed out. Disabled ensembling remains the retained baseline. The
matched seed observations suggest improvement but are not independent final
qualification or evidence of safe physical control. The collision guard remained
active; its 827 interventions are explicit failures requiring further assessment.

Effective horizon one, coefficient 0.01 and unchanged checkpoint digest
`36cca5c575c983de9b5485d5224039d8426abd397fa38978a49b8ee89e0061d9`
were recorded. p99 decision time was 15.38 ms, maximum 403.17 ms; these are
offline wall measurements, not real-time guarantees. All eight inference file
hashes remained unchanged. Fresh inference runs every tick; histories clear on
episode reset or camera rejection.

Implementation checks: dev build, Ruff, mypy, doctor, 438 dev tests and 15 real
CPU training/parser tests passed, including load/coefficient/fresh-prediction/reset
checks. Ten dev skips are optional learning/display tests, with learning tests
run separately; one existing multiprocessing warning remains.

The [next bounded experiment](../plans/act-budget-10000-study.md) changes training
budget only, keeping coefficient 0.01 and all safety/rate/data/model settings.
No final-test seeds, hardware use, model publication or merge were authorized by
this diagnostic outcome.
