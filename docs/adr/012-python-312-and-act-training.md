# ADR 012: Python 3.12 and pinned LeRobot ACT training

- Status: accepted
- Date: 2026-09-20
- Supersedes: ADR 011's Python 3.11, LeRobot 0.4.4, and conversion-only image decisions

## Context

PR 8 adds the missing validated LeRobotDataset-to-ACT-checkpoint stage. LeRobot
0.6.1 requires Python 3.12 and provides sample-exact single-process checkpoint
resume, while ADR 011 deliberately stopped at LeRobot 0.4.4 conversion on Python
3.11. Explicit CPU/GPU selection and local lineage are necessary because a
requested but unavailable accelerator must never become an unnoticed CPU run.

## Decision

Move every project image, tool target, and package declaration to Python 3.12.
Pin LeRobot 0.6.1, NumPy 2.2.6, PyTorch 2.11.0, and torchvision 0.26.0, with
separate CPU and CUDA 12.8 constraint sets. Conversion and training share this
framework version; converter version 2 adds a byte-level storage fingerprint,
so version-1 derived datasets must be regenerated from their immutable MCAP
selection manifests.

ACT Lab owns framework-neutral request, resolved configuration, dataset identity,
and run-manifest models. Its LeRobot adapter selects only frozen training episode
indices, disables held-out loading and Hub publication, and calls LeRobot's
official trainer and checkpoint/resume path. The CLI requires `cpu` or `cuda`
and checks CUDA before creating a run. CPU and NVIDIA Compose services expose
their capabilities explicitly. W&B is opt-in and only mirrors local metrics.

The production configuration retains upstream ACT defaults: ResNet-18 with
ImageNet initialization, VAE, 100-step chunks and action horizon, batch size 8,
100,000 updates, and the ACT AdamW preset. Seed 1000, deterministic cuDNN, no
AMP, and checkpoints every 20,000 steps are resolved and recorded. PyTorch does
not guarantee identical results across releases, platforms, or CPU/GPU.

## Consequences

Training remains local, single-node, and single-GPU. A run is append-only and
contains local metrics, logs, environment identity, lineage, and full LeRobot
state. Resume rejects device, batch, configuration, or dataset drift and may
only extend a step target. Offline loss demonstrates optimization, not task
success; closed-loop held-out evaluation remains PR 9.
