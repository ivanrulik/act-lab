# Future-action loading: measured redundant image decoding

Date: 2026-10-16. [Metrics](action-loading-profile-metrics.json) retain all four
read cases. This is a loading diagnosis, not a model result or an end-to-end GPU
training benchmark.

## Observed behavior

Pinned LeRobot 0.6.1 queries `hf_dataset["action"][indices]`. With datasets 4.8.5,
formatting that future-action query also decodes the camera columns. A current
paired sample with 100 future action targets therefore decoded **202 images**;
only two current images enter the model. The action query consumed about 0.312 s
of a 0.331 s profiled read.

Selecting the numeric action column before formatting decoded **two images**.
Every returned field matched exactly in four cases: start, middle, another
source episode, and the padded terminal window. Indices were 0, 100, 292, 8788
in the selected 8,789-frame training view. Raw data and dataset fingerprints
were unchanged.

| Index | Stock read | Numeric projection | Image decodes |
| --- | ---: | ---: | --- |
| 0 | 368.73 ms | 7.10 ms | 202 -> 2 |
| 100 | 300.71 ms | 7.16 ms | 202 -> 2 |
| 292 | 291.15 ms | 7.59 ms | 202 -> 2 |
| 8788 | 248.33 ms | 6.92 ms | 202 -> 2 |

These are four single-thread CPU reads in the CPU learning image during ongoing
GPU training, always stock then projected. Cache state, order and competing work
limit timing interpretation. Decode counts and exact field comparisons establish
the redundant work; these ratios are not measured training acceleration.

## Optional adapter and gates

[ADR 032](../adr/032-opt-in-numeric-action-projection.md) adds default-false
`project_action_columns`. It preserves current RGB/state, target/mask order,
padding, episode mapping, dtype and other query paths. The pinned source remains
unchanged; an importable reader subclass and bounded process-local factory hook
support spawned workers and restore the factory on failure.

The real seeded tiny training comparison with two spawned workers produced
**exactly equal model, optimizer and RNG tensors** against stock. Order,
duplicate/padded indices, selected-episode mapping, unknown-reader rejection and
factory restoration have explicit checks. Dev build, Ruff, mypy (82 files),
doctor, 439 dev tests and 37 real CPU learning/conversion/parser checks passed.
Fourteen dev-profile skips and one existing multiprocessing warning remain;
learning checks ran separately.

The active 5,000 -> 10,000 continuation uses the original reader throughout.
The subsequent [isolated GPU A/B](action-loading-gpu.md) passed exact checkpoint
equality and measured local throughput separately. This read profile alone
establishes no GPU speedup, learning improvement or robot hardware validation.

## Reproduce

```bash
COMPOSE_FILE=compose.yaml:compose.ci-runtime.yaml \
  docker compose --profile training run --rm train-cpu \
  python scripts/profile-act-loading.py \
  --run runs/requested-intent-study/paired/training \
  --output runs/new-loading-profile.json
```
