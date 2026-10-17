# Numeric action loading: isolated GPU A/B

Date: 2026-10-16. Full [metrics](action-loading-gpu-metrics.json) include all six
trials, configurations, runtime/source identity, scalar windows, source hashes
and exact tensor-comparison results.

## Protocol

Three pairs resumed the **same preserved 5,000-step checkpoint** for 100 updates
per trial on the same dataset, sampler, optimizer/RNG state, batch, model and
CUDA device. First 20 updates were warmup; logged windows were 20 updates. Order
alternated by pair. The original 10,000-step experiment and its probes had exited;
no other ACT GPU container was active. Upstream source stayed unchanged.

| Pair | Stock steady updates/s | Projected steady updates/s | Stock trial wall | Projected trial wall |
| --- | ---: | ---: | ---: | ---: |
| 0 | 2.052 | 9.091 | 59.62 s | 18.61 s |
| 1 | 2.000 | 9.091 | 61.44 s | 18.36 s |
| 2 | 1.878 | 9.091 | 62.35 s | 19.38 s |

Median steady throughput improved **4.55x** (2.0 -> 9.09 updates/s). Median full
trial wall time improved **3.30x** (61.44 -> 18.61 s), including initialization,
worker startup and checkpoint saving. These are three local short-run
measurements with upstream scalar-display precision; sustained-run performance,
other machines and policy quality need separate measurement.

## Equivalence and retention

All three pairs produced **exactly equal model, optimizer and RNG tensors**.
Source-checkpoint hashes remained unchanged after every pair. Only successful
throughput-replica checkpoints were removed; complete logs/metrics/provenance
remain. These replicas are not task candidates. Original 5,000/10,000 models,
data and all failed policy evidence remain intact.

The first GPU harness attempt exposed a missing script main guard under spawned
workers. It was interrupted with zero completed trials; its failed report/logs
remain under the earlier output. Commit `ab842a3` fixed the guard before the
measured run. No timing or learning result is attributed to that failed attempt.

The explicit projection option is now eligible for the separately predeclared
[20,000-step continuation](../plans/act-budget-20000-study.md). It does not change
learning tensors, model, losses, rates or safety. It is not a claim that the
8/20 task-success model is qualified.
