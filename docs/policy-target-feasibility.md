# PR 9.2: high-X/low-Y target feasibility

The PR 9.1 coverage checkpoint succeeded on 96/100 held-out seeds but issued
41 rejected IK commands. Its four timeouts (11008, 11012, 11058, and 11062)
began near the high-X/low-Y side of the cube spawn area. This follow-up uses
the existing checkpoint and evaluation seeds; it does not retrain ACT or change
the action, safety, collision, or task-success contracts.

## Diagnosis

Replay of the saved action traces reproduced the first rejected command in
each failed episode. The targets were close to the measured end-effector pose.
With the original 50-iteration, 0.1 mm IK settings, final position residuals
were 0.120–0.155 mm; orientation residuals were about 0.02–0.03 mrad, far below
the configured 1 mrad limit. Additional iterations alone converged for two
of the four commands, but the others plateaued around 0.145–0.153 mm. These
were numerical fit failures near the sampled reach boundary, not grossly
unreachable policy poses.

The MuJoCo driver now allows 200 iterations and a 0.2 mm position residual.
That tolerance remains one tenth of the separately tested 2 mm measured pose
convergence and well below the expert's 15 mm grasp tolerance. Joint margins,
contact checks, Cartesian rate limits, the watchdog, and
`SafeCartesianRobot.command(Action)` are unchanged. A candidate still fails
closed if IK cannot meet the revised tolerance or contact checks fail.

Integration tests reconstruct two recorded near-boundary measured joint states
and one later seed-11012 state. They show the original settings reject those
targets while the revised settings yield collision-free candidates. The
existing unreachable-pose test still requires an IK rejection and safe hold.

## Evaluation protocol

Use the original coverage checkpoint and seeds 11000–11099 at 25 Hz policy and
50 Hz simulation rates. Keep the existing PR 9.1 report immutable. Run a new
report with the revised simulation configuration:

```bash
docker compose --profile gpu run --rm train-gpu \
  act-lab sim evaluate \
  --checkpoint runs/training/act-gpu-coverage/lerobot/checkpoints/last \
  --device cuda --seed-start 11000 --episodes 100 \
  --output runs/evaluation/pr92-iter200-coverage-11000 --no-video
```

The old and new reports use different simulation configurations by design.
Compare their per-seed outcomes and safety counts as a controlled controller
configuration experiment; the standard `act-lab experiment compare` correctly
rejects this configuration mismatch. Do not present this as a comparison of two
ACT checkpoints. These 100 seeds have already informed PR 9.1 and this
follow-up; their results are regression evidence, not a fresh unbiased holdout.

The focused reruns over seeds 11008–11012 and 11058–11062 produced 9/10 ACT
successes. Seeds 11008, 11012, and 11062 now succeed. Seed 11058 still times
out without an IK rejection; the expert succeeds under the revised settings,
although it failed on that seed in the original PR 9.1 report. That remaining
grasp failure is visible in its action trace: the tool closes roughly 2–3 cm
from the cube center, then transports an empty grasp toward the tray. It
requires separate policy or demonstration analysis rather than another IK
tolerance adjustment.

## Full 100-seed regression, 2026-10-07

Both runs use the same coverage checkpoint digest
`d54662b9488d57b398822da73f4b163bdfa4f7d05070b5404b3d058e67e00743`
and seeds 11000–11099. Only the two IK settings above differ.

| Metric | Original IK | Revised IK |
| --- | ---: | ---: |
| ACT successes | 96/100 | 97/100 |
| ACT Wilson 95% interval | 90.2–98.4% | 91.5–99.0% |
| ACT IK rejections | 41 | 2 |
| ACT collision stops | 0 | 0 |
| ACT grasp events | 96 | 98 |
| Expert successes | 99/100 | 100/100 |

ACT improves on seeds 11008, 11012, and 11062 and regresses on 11005 and
11092. The paired success difference is +1 percentage point; a deterministic
paired bootstrap 95% interval is −3 to +5 points. This is inconclusive for task
success. The large reduction in IK rejections is the clear observed change.
The revised policy still fails on 11005, 11058, and 11092. Seed 11005 lifts
the cube but reaches the 500-step limit before placement; 11058 and 11092
never register a grasp. Their tool centers are roughly 2–3 cm from the cube
center while closing. None of the three failure episodes contains an IK
rejection. Under the revised configuration the scripted expert succeeds on
seed 11058, so its former failure remains a historical simulator/controller
regression case rather than a current expert failure.

Ignored local artifacts include the full report at
`runs/evaluation/pr92-iter200-coverage-11000/evaluation.json`, focused action
traces under `runs/evaluation/pr92-iter200-11008-11012/` and
`runs/evaluation/pr92-iter200-11058-11062/`, and trace/video pairs for the
three current failures under `runs/evaluation/pr92-failure-{11005,11058,11092}/`.
These are local evidence, not Git-tracked release artifacts; publication is
part of PR 10.

## Control timing follow-up

The 200-iteration cap can consume the 20 ms control period when an intent is
infeasible. Burst timing alone understates this tail: a synthetic source paced
at 50 Hz exposed slower solves between sleeps. The timing fix retains the
200-iteration cap, 0.2 mm tolerance, damping, and safety path.

The driver reuses Jacobian, linear-system, and rollback buffers, hoists constant
iteration work, and projects all six joint values together. IK iterations now
run `mj_kinematics` followed by `mj_comPos`, the minimal pipeline required for a
site Jacobian. Predicted contacts and simulated motion still run the full
pipeline. The damped 6x6 system is positive definite; MuJoCo's Cholesky routines
solve it without general NumPy/LAPACK dispatch. An unsuccessful factorization
uses the original NumPy solve. See the official [Jacobian pipeline documentation](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-jac)
and [Cholesky routines](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mju-cholfactor).
These are adapter implementation changes within ADR 004 and ADR 009.

`last_ik_diagnostics` reports the last solve's iteration count, convergence,
and position/orientation residuals. Passing `profile=True` also records wall
time for IK, collision checks, and driver stepping; profiling is off by default.
Step timing includes nested collision checks, so those totals must not be added
again. Neither timing nor scheduling changes convergence decisions. Reset clears
diagnostics and accumulated timings. The driver and its reusable buffers remain
owned by the single simulation/control thread.

The regression tests compare kinematics-only updates exactly against full
forward updates, and Cholesky solutions against NumPy within 1e-12 rad on home,
recorded boundary, infeasible, and seeded nearby targets. They also check
identical command outcomes, close measured states, capped infeasible solves,
and profiling invariance. Existing collision-stop, watchdog, grasp-preserving
hold, joint-bound, and sustained-motion tests remain required. CI does not
assert machine-dependent milliseconds.

### Reproduce the timing run

```bash
docker compose build dev
docker compose run --rm dev python scripts/benchmark-control.py \
  --output runs/benchmarks/control-timing.json \
  --image-id "$(docker image inspect act-lab-dev --format '{{.Id}}')" \
  --variants current --samples 300 --warmup 20 --repeats 3 --paced-cycles 1500
```

The script rejects an existing output. It records source hashes, Git state,
image identity, library/BLAS versions, CPU architecture/affinity, thread settings,
resolved control configuration, distributions, individual samples, solver
residuals, outcomes, and maximum consecutive work overruns. `original` and
`cap100` select alternative **configuration** settings on the implementation
being run; they do not restore the former implementation.

The recorded fixture is the measured joint state before seed 11012's first
rejection at 50 iterations with a 0.2 mm tolerance. It requires 99 iterations
under the revised solver. Benchmarks include direct IK, continuous scripted
motion, sustained infeasible intent, predicted collision stops, stale/disabled
holds, and a paced source alternating ten enabled requests with one disabled
request. Preparation/reset work is outside each burst sample. Command timing
includes action creation/polling, the full safe command, and observation;
paced work also includes the initial observation. Paced reports separate work
from actual start-to-start intervals, which include sleep/scheduler overhead.
Disabled-to-hold measures command dispatch through returned measured state,
not physical input capture latency.

### Local timing results, 2026-10-07

Both measurements use an Intel Core i7-13620H, the same dev image
`sha256:0989d1ce751eb3bd9ae28b2ea5acce496717b9095737f8cd920ba613965dd08b`,
NumPy 2.2.6, MuJoCo 3.12.0, and unchanged 200-iteration / 0.2 mm settings.
The reference loads the driver from commit `9abfda6` with a compatibility shim
for the benchmark API. It has no phase timers or solver diagnostics; optimized
measurements include profiling overhead. Both use 20 warmups, three batches of
300 samples per case, and 1,500 paced cycles. Timing runs were sequential.

| Measurement | Reference | Optimized |
| --- | ---: | ---: |
| Direct boundary IK p99 | 16.50 ms | 3.74 ms |
| Direct infeasible IK p99 | 33.68 ms | 7.26 ms |
| Sustained infeasible safe command p99 | 24.73 ms | 7.97 ms |
| Paced cycle work p99 | 44.64 ms | 13.78 ms |
| Paced work over 20 ms | 1,078/1,500 | 2/1,500 |
| Longest work-overrun sequence | 10 cycles | 1 cycle |
| Paced maximum work | 60.34 ms | 23.97 ms |
| Disabled dispatch-to-hold maximum | 4.37 ms | 2.41 ms |

Command outcome sequences match in every benchmark case and in the paced stream,
including IK failures, predicted collision stops, and disabled/stale holds.
All optimized burst command samples stayed below 20 ms. The paced p99 leaves
about 6.2 ms of measured headroom, but the two outliers show that this host is
not a hard real-time guarantee. Sleep also adds scheduler overhead: optimized
start-to-start intervals have a 20.22 ms p99 even when work fits the period.
No machine thread environment or global training configuration was changed.

Ignored reports are `runs/benchmarks/pr92-timing-reference-matched.json` and
`runs/benchmarks/pr92-timing-optimized.json`; the reference source and shim are
saved beside them. Reports include exact source hashes because measurements
ran from an uncommitted worktree. Earlier pilot reports are retained separately.
These measurements exclude viewer synchronization, physical-camera capture and
inference, ROS transport, and hardware. They establish simulator command-path
performance on this host; interactive-camera acceptance remains separate.

### Regression after timing changes

A new CUDA run at `runs/evaluation/pr92-timing-coverage-11000/evaluation.json`
uses the same checkpoint digest, resolved simulation configuration, seeds,
policy stride/device, and safety path as the earlier revised-IK report. The
standard comparison accepts these matched inputs; its derived report is
`runs/experiments/pr92-timing-comparison.json`.

| Metric | Revised IK before optimization | Optimized |
| --- | ---: | ---: |
| ACT successes | 97/100 | 97/100 |
| ACT IK rejections | 2 | 2 |
| ACT collision stops / drops | 0 / 0 | 0 / 0 |
| ACT grasp events | 98 | 98 |
| ACT mean successful completion | 8.6864 s | 8.6858 s |
| Expert successes | 100/100 | 100/100 |
| Expert mean successful completion | 8.5166 s | 8.5166 s |

All per-seed success/failure assignments and grasp/drop counts match. The same
three ACT failures remain: 11005, 11058, and 11092. The two IK rejections remain
on successful seeds 11019 and 11062. Ten successful ACT episodes differ by
1–3 simulation steps in completion time, and the maximum final cube position
difference is 4.36 mm. Expert timing/event outcomes match exactly, with final
cube differences below 3e-14 m. Cholesky changes floating-point rounding; the
ACT trajectory variations are consistent with those small differences being
amplified by camera feedback. They introduce no observed task or safety
regression in this inspected cohort. The repeated runs establish regression
evidence; a fresh generalization claim requires a new holdout.

Validation ran through Docker Compose: dev image build, Ruff, mypy, pytest
(163 passed, 3 skipped), and `act-lab doctor`. The skips cover LeRobot training
and dataset tests absent from the dev image, plus explicit X11 display
validation. The separate 100-seed evaluation did run the real ACT checkpoint
on CUDA. No physical camera, interactive X11 session, ROS, or hardware run is
claimed by this follow-up.
