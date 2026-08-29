# Pull-request roadmap

The PR number is an ordering aid, not a promise that unrelated work must wait.
Do not combine adjacent PRs merely because they are listed together.

## PR 1 — Repository and container foundation

Status: scaffolded in the initial repository.

Deliver the package layout, domain ports, multi-stage Docker build, Compose
services, devcontainer, CI, lint/type/test checks, agent contract, ADRs, and
documentation map.

Acceptance: a clean clone builds the `dev` target and passes `act-lab doctor`,
linting, typing, and tests entirely inside the container.

## PR 2 — Deterministic UR5e simulation

Add pinned/licensed MuJoCo Menagerie UR5e assets, simple parallel gripper,
table/cube/tray scene, fixed-step environment, headless rendering, and task
success contract.

Acceptance: seeded resets reproduce state; headless rollout and rendered-frame
tests pass; reachable randomization bounds are documented.

## PR 3 — Cartesian controller and safety envelope

Add damped-least-squares IK, pose/gripper targets, joint/workspace limits,
velocity/acceleration bounds, collision checks where applicable, and watchdog.

Acceptance: reachable pose tolerances pass; unreachable or stale commands stop
safely; long randomized tests produce no NaNs or limit violations.

## PR 4 — Keyboard teleop and scripted expert

Provide a stable keyboard input adapter and deterministic finite-state-machine
expert before introducing camera perception.

Acceptance: both use the generic action contract; a user can finish the task;
the scripted expert meets a documented success threshold on seeded trials.

## PR 5 — Webcam hand teleoperation

Add MediaPipe capture/tracking, neutral-pose calibration, clutch, dead zones,
filtering, confidence gating, pinch gripper mapping, diagnostics, and recorded
video input for tests.

Acceptance: tracking loss stops motion within the watchdog deadline; physical
camera access requires neither root nor a privileged container.

## PR 6 — Versioned MCAP episode recording

Add Protobuf schemas, shared monotonic timestamps, episode lifecycle, atomic
finalization, provenance metadata, channel inspection, and recovery tests.

Acceptance: expected streams/rates exist; interrupted files are recoverable;
failed and discarded demonstrations remain traceable.

## PR 7 — Validation, replay, and LeRobot conversion

Add quality rules, replay UI/CLI, deterministic resampling, episode selection
manifest, episode-level data splits, LeRobotDataset conversion, and small test
fixtures.

Acceptance: conversion is deterministic; sampled records round-trip; invalid
episodes fail with actionable reports; no frame-level data leakage occurs.

## PR 8 — ACT training CLI and teaching notebook

Integrate LeRobot ACT behind an adapter, add resolved training configuration,
checkpoint/resume, dataset fingerprints, a tiny overfit test, and a notebook
that calls production APIs.

Acceptance: a small dataset intentionally overfits; run metadata is complete;
explicit GPU requests fail rather than silently fall back to CPU.

## PR 9 — Closed-loop evaluation

Evaluate scripted and learned policies through the same safety/action path over
held-out seeded initial conditions.

Acceptance: reports include success, completion time, grasp/drop/limit events,
confidence intervals, configuration, videos, and individual failure cases.

## PR 10 — Reproducible learning release

Add the guided curriculum, data-quality checklist, experiment template, failure
taxonomy, model/dataset cards, privacy guidance, and immutable published images.

Acceptance: a second person completes the entire workflow from a clean clone.

## PRs 11–14 — Optional ROS 2 and hardware track

1. Define ROS topics, QoS, frames, clock behavior, and domain conversions.
2. Expose MuJoCo through that contract and test local/ROS equivalence.
3. Record ROS topics through rosbag2 MCAP and reuse the existing converter.
4. Add official UR hardware integration only after a dedicated safety review.

ROS must remain optional for simulation, conversion, training, and evaluation.

