# ADR 011: Deterministic episode selection and LeRobot conversion

- Status: accepted
- Date: 2026-09-13

## Context

Raw acquisition evidence contains successful, failed, discarded, interrupted,
and legacy attempts. Training needs uniformly sampled LeRobotDataset records
without deleting evidence, leaking frames across splits, or coupling domain
contracts to MCAP or LeRobot types.

## Decision

Translate MCAP at the adapter boundary into dependency-free recorded-episode
models. Quality validation is an application policy. A versioned JSON selection
manifest retains every candidate, its raw SHA-256, quality decision, and a
deterministic train/validation assignment computed from the complete episode ID.
Only complete, valid, successful episodes are selected. Missing late PR 6 webcam
acquisition identity is a visible warning rather than a retroactive rejection.

Resample on an exact integer-nanosecond grid. Interpolate numeric state/action
features linearly, normalize interpolated action quaternions, and select discrete
values and RGB images by nearest timestamp with earlier ties. Conversion rechecks
raw hashes and quality, then uses the public LeRobotDataset v3 writer. Publish the
derived directory only after finalization and record a canonical logical
fingerprint plus the full selection lineage.

Keep LeRobot and CPU-only PyTorch in an isolated, fully constrained Compose data
image. Pin LeRobot 0.4.4, the last v3 release compatible with the project's Python
3.11 baseline. Do not add ACT training in this image or PR.

## Consequences

Rejected raw episodes remain inspectable and explainable. Episode-level hashing
prevents frame leakage and conversion is repeatable from immutable inputs. Image
features consume more files than encoded video but avoid codec nondeterminism in
this educational baseline. The isolated data image is larger than the simulation
image; changing LeRobot or Python requires an explicit compatibility review and
lock refresh. Training, normalization choices for ACT, and held-out evaluation
remain later roadmap work.
