# Lean wrist-camera study

The [implementation plan](plans/wrist-camera-study.md) freezes the seed protocol,
visibility, quality, cost and safety metrics. [ADR 028](adr/028-explicit-camera-study.md)
records the source comparison and explicit camera-projection contract.

## Run the inexpensive gate first

```bash
docker compose run --rm dev act-lab experiment camera-audit \
  --config configs/sim/ur5e_2f85_d405_paired.toml \
  --output runs/camera-audit
```

Ten seeded expert rollouts sample actual rendered cube/tray geom pixels at 5 Hz
and on phase entry. Wrist and union-of-wrist+fixed are assessed on identical
paused physics snapshots. The union requires at least one complete view to
meet the pixel threshold; it does not add subthreshold pixels across cameras.
Every critical phase in every seed must have samples and reach 95% visibility
with at least 16 target pixels. Missing phases or expert failure cannot pass.
First wrist failures save small paired RGB evidence images. Rendering is a
read-only audit and its labels never enter training or generic observations.

A completed no-go returns success with explicit `passed: false` decisions.
Broken execution returns an error. Read decisions, not just the process code.

## Run the bounded staged experiment

```bash
docker compose --profile gpu run --rm train-gpu \
  python scripts/run-camera-study.py --output runs/camera-study
```

The runner always audits first. A failed visibility gate records
`completed_observation_no_go` and starts neither collection nor a pilot. It
does not silently change optics or thresholds. On a pass, CUDA is checked before
collection. The recipe records seeds 20000–20039 once, validates MCAP, freezes
episode splits, projects both datasets, and trains two fixed 5,000-update ACT
pilots. Both checkpoints finish before any final evaluation starts. Diagnostic
seeds are 21000–21019; final seeds are 22000–22099. The final report distinguishes
quality, noninferiority, safety events, measured bytes, training wall time and
decision latency. A 5 GiB artifact check after each command stops the next stage
if the budget has been exceeded; it is not a hard filesystem quota.

No full-size training sweep or upload starts automatically. Missing GPU never
falls back to CPU. The existing pinned learning images supply dependencies;
no ROS or new Python dependency is needed. Tiny CPU train/resume tests validate
mechanics separately and do not count as pilot policy quality.

## Explicit camera projection

Use one validated raw selection for both variants:

```bash
docker compose --profile training run --rm train-cpu act-lab recording convert \
  runs/camera-study/selection.json --fps 25 --cameras wrist \
  --output runs/camera-study/wrist/dataset --repo-id local/camera-wrist
docker compose --profile training run --rm train-cpu act-lab recording convert \
  runs/camera-study/selection.json --fps 25 --cameras wrist policy \
  --output runs/camera-study/paired/dataset --repo-id local/camera-paired
```

Raw files are unchanged. Original hashes and quality are checked before
projection. The converter retains source contracts and raw provenance, and
emits a calibrated selected-camera contract. Images, numeric frames, time grid,
episode order and splits are shared; selected camera sets and logical
fingerprints differ. Articulated policies must retain wrist. Missing/duplicate
views fail. Use the matching `ur5e_2f85_d405_wrist.toml` or `_paired.toml` for
evaluation; an incompatible saved contract still fails before policy loading.

`experiment camera-compare` requires both evaluation reports, verified datasets,
training runs and inference checkpoints. It permits only the declared camera
difference and checks common raw selection, frozen splits, training budgets and
execution conditions. The existing `experiment compare` remains strict about
every simulation setting. See `--help` and the runner for the full invocation.

## Current result

[Measured evidence](reports/wrist-camera-study.md): wrist-only failed the Mesa
software-rendered transfer audit but passed the NVIDIA run of the same gate.
Paired input passes both. The paired preset is the authorized next learning
candidate; these audits do not establish camera superiority. Visibility success does not establish
learned-policy success. Wider optics, active observation or a policy designed
to remember unseen goals require a separately specified experiment.

## Paired candidate continuation

The [paired-camera pilot](plans/paired-camera-pilot.md) runs the viable candidate
alone with `scripts/run-camera-study.py --variant paired`. The original default
comparison still stops when either visibility gate fails. A paired-only result
qualifies one candidate and makes no camera superiority claim.

The [completed paired pilot](reports/paired-camera-pilot.md) achieved 33/100 final
successes and did not qualify at its frozen 5,000-update budget. Every final
outcome and measured cost is retained; the final cohort is now consumed.
