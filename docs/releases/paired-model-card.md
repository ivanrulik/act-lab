---
license: apache-2.0
library_name: lerobot
tags:
- robotics
- act
- mujoco
datasets:
- ivanrulik/act-lab-ur5e-paired-pick-place-v1
---

# ACT Lab paired-camera UR5e ACT — simulation-qualified recipe

One frozen 20,000-update inference checkpoint for articulated UR5e pick/place:
wrist + fixed policy RGB, ResNet-18 ImageNet V1 initialization, VAE, batch 8,
100-action training chunks, AdamW 1e-5, seed 1000, no AMP. LeRobot 0.6.1,
Python 3.12. All inference weights, model/camera contract and normalization
processors are preserved byte-for-byte. Optimizer/RNG state, intermediate
checkpoints and raw acquisition logs are excluded.

## Required execution settings

**Use ACT Lab's adapter**, horizon **one**, fresh predictions at **25 Hz** and
explicit temporal ensemble coefficient **0.01**, with shared application safety
at **50 Hz**. See `inference.json` and the included reproduction instructions.
The serialized training config retains horizon 100 and ensemble None; loading
it with framework defaults does **not** reproduce this qualified recipe.
The adapter verifies overrides and clears ensemble history at reset/input fault.

Only `ur5e_2f85_d405_paired.toml` with its unchanged assets, control limits and
camera calibrations is qualified. The loader enforces the model/camera contract.
Original `/workspace` training paths remain in frozen payload files; use explicit
downloaded paths. `reproduction/check-paired-release.py` verifies compatibility,
loads only the 30 frozen training episodes and executes the downloaded model.

## Measured evidence

18/20 diagnostics on seeds 21000–21019; then **90/100 fresh final tasks** on
23000–23099. Wilson 95% interval **82.56–94.48%**; expert 100/100. The gate was
90% point success, not a 90% lower confidence bound. Ninety grasps, zero drops.
All ten failures timed out before grasp. Four failures generated **1,073
collision-stop commands**, including repeated rejected commands/holds; these
are not measured physical impacts. All successful cases had zero safety rejects.

Failed seeds: 23003, 23015, 23017, 23024, 23029, 23042, 23043, 23056, 23066,
23096. Mean successful completion 11.15 simulated seconds. Decision wall-time
including RGB rendering: median 8.32 ms, p99 14.03 ms, maximum 528.04 ms.
Offline results do not establish live or hardware timing guarantees.

Checkpoint aggregate digest:
`4525c1dc57eb840ffe2090317813a3998e9fdad0f2ccc8a569a7d3c386986d6d`.
The training manifest retains its initial dirty source revision; staged resumes
and their source identities are separately recorded. 10k→20k used the explicit
numeric-action loading projection; matched tests found identical tensors.
Bit-identical retraining and independent newcomer acceptance are not claimed.

## Replay examples and limitations

`examples/act-seed-23000.mcap`: successful 10.16 s demonstration rerun.
`examples/act-seed-23003.mcap`: 15 s timeout demonstration rerun. Both contain
original wrist/fixed RGB, commands and state, plus Foxglove RawImage derivatives.
These are new recordings of previously consumed seeds, **not new held-out
evaluation** or byte-identical original trajectories. Recording comparisons
retained the same safety outcome and step count, with small pose differences.
The replay report also describes seed 23001, which is omitted to keep this lean.
Do not use these policy reruns as expert training demonstrations.

Simulation only. No hardware, learned ROS motion, other objects/tasks, changed
optics or broader generalization qualification; wrist-only remains unresolved.
Weights/metadata are Apache-2.0 under ACT Lab's declaration; dependency and
pretrained-backbone provenance and simulated-asset notices remain separate.
Source: [ACT Lab](https://github.com/ivanrulik/act-lab).
