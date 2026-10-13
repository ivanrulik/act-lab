---
license: apache-2.0
library_name: lerobot
tags:
- robotics
- act
- mujoco
datasets:
- ivanrulik/act-lab-ur5e-educational-pick-place-v1
---

# ACT Lab educational UR5e ACT checkpoint

One evaluated 100,000-update ACT inference checkpoint with ResNet-18,
ImageNet initialization, VAE, 100-action chunks and batch size 8.
Training uses 58 of 78 frozen expert episodes at 25 Hz; 20 episodes are reserved.
LeRobot 0.6.1, Python 3.12, PyTorch 2.11.0. This release includes inference
weights, normalization processors and resolved training configuration.
Optimizer/RNG state and intermediate checkpoints are intentionally excluded:
this package supports inference and new training, not exact optimizer resume.

## Execution

Load the downloaded `payload` directory as the checkpoint. Use ACT Lab's
two-camera educational preset, `--policy-hz 25`, one action per inference,
50 Hz environment steps and `SafeCartesianRobot.command(Action)`.
Frozen training configuration retains historical `/workspace` paths; callers
must supply their own dataset/checkpoint locations. No automatic upload or
physical command is performed by loading the checkpoint.

## Measured results

Original untouched seeds 11000–11099: 96/100 successes, Wilson 95% interval
90.2–98.4%, 41 rejected IK intents and zero drops. Baseline ACT was 67/100
on the same seeds and safety path. Individual failures remain in the evidence.

Later solver/timing regression on those already inspected seeds: 97/100,
Wilson 95% interval 91.5–99.0%, two IK rejections and zero drops. This is not
a new holdout. The +1 percentage-point change has an inconclusive paired
interval of −3 to +5 points. Preserve both reports and their resolved settings.

## Restrictions and provenance

Use only with the legacy educational two-camera simulator. This checkpoint
does not qualify the newer articulated gripper/wrist-camera preset, physical
hardware, other tasks, altered camera calibration or learned generalization.
Training recorded a dirty source checkout; its revision is partial provenance.
The release verifies original dataset and checkpoint hashes. Clean-download
smoke evidence is reported separately; no new full evaluation is implied.

Weights and publication metadata are Apache-2.0 under ACT Lab's declaration.
LeRobot/PyTorch and pretrained ResNet provenance remain recorded in the resolved
configuration; simulated asset source notices are retained in `ASSET-NOTICE.txt`.
See [ACT Lab](https://github.com/ivanrulik/act-lab) for source and dependencies.
