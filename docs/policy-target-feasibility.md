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
