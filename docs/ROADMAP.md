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

Status: implemented in the current working tree; acceptance checks are listed
in the implementing change.

Add pinned/licensed MuJoCo Menagerie UR5e assets, simple parallel gripper,
table/cube/tray scene, fixed-step environment, headless rendering, and task
success contract.

Acceptance: seeded resets reproduce state; headless rollout and rendered-frame
tests pass; reachable randomization bounds are documented.

## PR 3 — Cartesian controller and safety envelope

Status: implemented in the current working tree; acceptance checks are listed
in the implementing change.

Add damped-least-squares IK, pose/gripper targets, joint/workspace limits,
velocity/acceleration bounds, collision checks where applicable, and watchdog.

Acceptance: reachable pose tolerances pass; unreachable or stale commands stop
safely; long randomized tests produce no NaNs or limit violations.

## PR 4 — Keyboard teleop and scripted expert

Status: complete. The deterministic 20-seed acceptance benchmark passes at the
documented 0.90 threshold and produces identical repeated reports. Manual X11
validation confirmed keyboard jogging and completion of the pick-and-place
task.

Provide a stable keyboard input adapter and deterministic finite-state-machine
expert before introducing camera perception.

Acceptance: both use the generic action contract; a user can finish the task;
the scripted expert meets a documented success threshold on seeded trials.

## PR 5 — Webcam hand teleoperation

Status: complete. Automated tests cover calibrated mapping, gesture clutching,
tracking loss, the 100 ms watchdog boundary, and recorded MediaPipe input.
Live camera and X11 behavior remain environment-specific validation and must be
reported separately when run.

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

## Optional ROS 2, CRISP, and hardware track

[CRISP Controllers](https://github.com/learnsyslab/crisp_controllers) is a
candidate low-level controller for this track. Its real-time Cartesian
impedance and operational-space controllers could turn sparse pose targets from
ACT or teleoperation into smooth, compliant torque commands. CRISP is not a
replacement for ACT, the application-owned safety envelope, or the validated
MCAP-to-LeRobot data lifecycle.

### PR 11 — ROS 2 contracts

Define ROS topics, QoS, frames, clock behavior, watchdog behavior, and
conversions at the domain boundary. ROS messages and clocks remain adapter
types and must not enter `act_lab.domain`.

Acceptance: the contract defines stale and lost-command behavior; frame and
timestamp conversions have automated tests; local simulation, conversion,
training, and evaluation still run without ROS 2.

### PR 12 — CRISP feasibility spike and decision

Evaluate a pinned CRISP revision in an isolated Docker Compose profile. Record
its license and dependency rationale, and test its action semantics, control
frequency, gravity and friction conventions, watchdog behavior, and
compatibility with the official UR5e `ros2_control` effort interface. Do not
adopt CRISP merely because its demos run on another manipulator.

Acceptance: a reproducible report compares CRISP requirements with ACT Lab's
control and safety contracts, exercises command loss and stale targets without
hardware, and records a go/no-go decision in an ADR. A no-go result preserves
the ROS contract and selects or defers an alternative controller without
changing the learning pipeline.

Reference: the official Universal Robots driver documents its
[joint-torque interface](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_robot_driver/ur_robot_driver/doc/usage/force_torque_control.html),
including PolyScope requirements and the safety responsibilities of direct
torque control.

### PR 13 — CRISP simulation adapter

Proceed only after a CRISP go decision. Expose CRISP behind the existing robot
and feasibility ports, connect it to an effort-controlled MuJoCo/`ros2_control`
test environment, and keep the current local MuJoCo adapter as the
deterministic baseline.

Acceptance: identical input traces produce comparable local and ROS/CRISP
trajectories and inspectable command outcomes; workspace, joint, torque, and
rate limits hold; disabled, stale, invalid, and lost commands stop safely; the
default headless workflow does not require ROS 2 or CRISP.

### PR 14 — ROS MCAP recording and conversion equivalence

Record synchronized ROS topics through rosbag2 MCAP and translate them into
the existing versioned raw-recording contract. Reuse the validator, episode
selection manifest, deterministic converter, and LeRobotDataset representation.
CRISP Gym's direct LeRobot recording may inform tests but is not an
authoritative acquisition path.

Acceptance: equivalent local and ROS recordings pass the same quality rules
and produce schema-compatible derived datasets; invalid and interrupted
episodes remain traceable; training never consumes unvalidated raw logs.

### PR 15 — UR5e hardware readiness and safety review

Before commanding a physical robot, document controller ownership, torque and
torque-rate limits, payload and gravity configuration, protective stops,
network-loss behavior, startup/shutdown ordering, recovery procedures, and
operator supervision. Define a staged, low-energy commissioning protocol and
the evidence required to advance each stage.

Acceptance: a dedicated ADR and safety checklist are reviewed; all feasible
fault-injection and dry-run tests pass in simulation; documentation clearly
distinguishes simulated evidence from hardware validation.

### PR 16 — Supervised UR5e hardware integration

Integrate the official Universal Robots ROS 2 driver and the controller chosen
by PR 12. Run the commissioning protocol under trained supervision, then prove
that teleoperation and learned policies use the same domain action, application
safety, recording, and reporting paths as simulation.

Acceptance: hardware, PolyScope, driver, controller, configuration, payload,
and calibration versions are recorded; watchdog and protective-stop tests
pass; evaluation reports include every intervention and failure. Do not claim
CRISP hardware support if PR 12 selected another controller or hardware tests
did not run.

ROS 2 and CRISP remain optional adapters. Their packages must not become
dependencies of the domain, data conversion, training, evaluation, or default
local simulation workflows.
