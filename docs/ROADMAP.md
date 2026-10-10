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

PR 5.1 replaces the default position mapper with velocity-style webcam control,
adaptive filtering, hysteretic gestures, explicit command-speed limits, and
latency/drop diagnostics. The original mapper remains available for comparison.

Add MediaPipe capture/tracking, neutral-pose calibration, clutch, dead zones,
filtering, confidence gating, pinch gripper mapping, diagnostics, and recorded
video input for tests.

Acceptance: tracking loss stops motion within the watchdog deadline; physical
camera access requires neither root nor a privileged container.

## PR 6 — Versioned MCAP episode recording

Status: implemented on the PR 6 feature branch; validation is reported in the
implementing change. See `recording.md` and ADR 010. Multiple attempts within one
viewer and asynchronous recording performance tuning are deferred follow-ups;
current interactive commands record one attempt per run with function-key
stop/outcome/discard controls.

Controller convergence, lifted-cube pause/resume tests, and the 20-seed scripted
benchmark pass. The operator reports keyboard seeds 0–4 successful and successful
live webcam pick/place plus clutch pause/resume. The operator closed this testing
round with intermittent webcam frustration remaining; quantitative webcam timing
and latency acceptance are not claimed. Follow-up: measured webcam usability
benchmark and ergonomic tuning, informed by recorded failed/discarded attempts.
See `controller-readiness.md` for the evidence and limitations.

Add Protobuf schemas, shared monotonic timestamps, episode lifecycle, atomic
finalization, provenance metadata, channel inspection, and recovery tests.

Acceptance: expected streams/rates exist; interrupted files are recoverable;
failed and discarded demonstrations remain traceable.

## PR 7 — Validation, replay, and LeRobot conversion

Status: implemented on the PR 7 feature branch. Quality reports preserve legacy
metadata warnings, selection manifests retain rejected attempts, and the
LeRobotDataset v3 writer is isolated in the pinned CPU-only data image. See
`data-pipeline.md` and ADR 011.

Add quality rules, replay UI/CLI, deterministic resampling, episode selection
manifest, episode-level data splits, LeRobotDataset conversion, and small test
fixtures.

Acceptance: conversion is deterministic; sampled records round-trip; invalid
episodes fail with actionable reports; no frame-level data leakage occurs.

## PR 8 — ACT training CLI and teaching notebook

Status: merged as GitHub PR #12. Python 3.12 and LeRobot 0.6.1
supersede ADR 011's compatibility pins; existing derived datasets require
regeneration. See ADR 012 and `docs/training.md`.

Integrate LeRobot ACT behind an adapter, add resolved training configuration,
checkpoint/resume, dataset fingerprints, a tiny overfit test, and a notebook
that calls production APIs.

Acceptance: a small dataset intentionally overfits; run metadata is complete;
explicit GPU requests fail rather than silently fall back to CPU.

## PR 9 — Closed-loop evaluation

Status: merged as GitHub PR #13. `act-lab sim evaluate` runs the
scripted expert and a LeRobot ACT checkpoint on the same seed list, routes both
through `SafeCartesianRobot`, and writes a durable report plus per-episode MP4s
and failure records. The ACT device is explicit and CUDA requests fail closed.
The evaluation seed range defaults to 10000 onward and must be disjoint from
training demonstration seeds.

Acceptance: reports include success, completion time, grasp/drop/limit events,
confidence intervals, configuration, videos, and individual failure cases.

## PR 9.1 — ACT demonstration coverage experiment

Status: merged as GitHub PR #14.
Preserve the PR 9 checkpoint and diagnostic report, add
inspectable training-start coverage and paired evaluation comparison reports,
collect a separate seeded scripted-expert cohort, and train a new ACT run from
a validated derived dataset. See `policy-coverage-experiment.md`.

Acceptance: selection is frozen with raw hashes and episode-level splits;
training and evaluation seeds are disjoint; baseline and candidate are compared
on the same fresh held-out seeds with confidence intervals, safety events, and
individual outcomes. Report negative or inconclusive results honestly. No ROS
or hardware changes.

The fresh 100-seed comparison improved ACT success from 67% to 96%, while IK
rejections increased from 0 to 41 (the baseline had 8 collision stops).
Follow-up PR 9.2: diagnose high-X/low-Y grasp failures and reduce unreachable
policy targets without weakening `SafeCartesianRobot`; retain seed 11058 as an
expert/simulator failure case. See `policy-coverage-experiment.md`.

## PR 9.2 — High-X/low-Y target feasibility

Status: implemented and locally validated on the PR 9.2 feature branch. Replay
of the failed action traces found near-tolerance IK misses. The MuJoCo solver
now allows a 0.2 mm position residual and 200 iterations, with the same
application safety and collision checks. The timing follow-up reuses buffers,
updates only kinematics within IK, and uses a small Cholesky solve. A matched
1,500-cycle synthetic benchmark reduced paced work p99 from 44.64 to 13.78 ms;
two optimized cycles still exceeded 20 ms, so this is not a hard real-time
guarantee or interactive-camera validation. The repeated CUDA regression
preserves all 100 per-seed success/failure assignments, 97 ACT successes, and
two IK rejections. See `policy-target-feasibility.md`.

On the previously inspected seeds 11000–11099, the unchanged coverage ACT
checkpoint succeeds on 97/100 instead of 96/100 and produces 2 instead of 41
IK rejections. Three previous failures recover, two previous successes regress,
and seed 11058 still misses its grasp. The +1 percentage-point paired success
difference has a 95% bootstrap interval of −3 to +5 points, so task-success
improvement is inconclusive. The expert now succeeds on seed 11058 under the
revised solver settings. Preserve its earlier failure in the PR 9.1 report as
historical evidence. Follow-up: investigate the three remaining policy failures
without adding these inspected seeds to training.

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

Implemented on `feature/pr11-ros2-contracts`: generated v1 command/state/report
interfaces, topic QoS, primitive codecs, episode/sequence checks, simulation
clock offset, steady receipt/progress guards, and shared safe holds. An isolated
Jazzy Compose profile runs serialization and two-process DDS fault tests against
a deterministic fake driver. See [the executable contract and safety review](contracts/ros2.md)
and ADR 013. ROS messages/clocks remain adapter types outside `act_lab.domain`.

Acceptance: required ROS-free checks plus the separate ROS job pass; local
simulation and learning images build/run without ROS. No MuJoCo motion through
ROS or hardware is claimed. Source timestamps survive repeated polling; pause,
reset, replay, loss, future/stale intent, and recovery have executable evidence.

Deferred: moving ROS adapters/external clock-state authority binding in PR 13,
ROS recording in PR 14, hardware in PR 15–16. System-clock synchronization and
multi-producer arbitration require later scoped contract extensions.

### PR 12 — CRISP feasibility spike and decision

Implemented on `feature/pr12-crisp-feasibility`: the pinned real controller
plugin runs both Cartesian impedance and operational-space modes through lifecycle,
separate-process DDS and stationary UR5e mock effort interfaces. The isolated
profile records native safety gaps, test-only independent gate assertions,
compensation comparisons, timing and source/model/runtime provenance.

Original decision: **no-go for the filter=1 configurations**; retained in
[ADR 014](adr/014-crisp-feasibility-and-controller-decision.md) and
[the original report](reports/crisp-feasibility.md).

Completed follow-up: explicit target/state/output filter semantics, nine stationary
local-scene/UR-tool frame checks, independent pose/feedback effort calculations,
65 guarded assertions per mode and three local 500 Hz benchmark runs per mode.
**Conditional go for Cartesian impedance**, following the predefined tie rule.
See [ADR 015](adr/015-crisp-configuration-requalification.md),
[requalification evidence](reports/crisp-requalification.md) and
[the contract/safety matrix](contracts/crisp-feasibility.md).

Production watchdog/stop/hold, moving dynamics, calibrated tool/payload semantics,
hardware and recording remain later roadmap work.

### PR 13 — CRISP simulation adapter

Implemented on the PR 13 branch with Cartesian impedance: generated v1 DDS
commands pass through the shared safety path, actual controller manager and
six effort interfaces into MuJoCo. An independent physics owner applies bounded
dynamic hold and requires fresh intent/reactivation after faults. The existing
local driver remains the deterministic baseline. ADR 016 records the moving
frame correction and hold design; see the
[local qualification report](reports/pr13-ros2-simulation.md).

Hardware, system clocks and operational-space moving qualification remain deferred.
Near-singular moving trajectories require a separate expansion of qualification
fixtures; stationary bench results do not supply that evidence.

Acceptance: identical input traces produce comparable local and ROS/CRISP
trajectories and inspectable command outcomes; workspace, joint, torque, and
rate limits hold; disabled, stale, invalid, and lost commands stop safely; the
default headless workflow does not require ROS 2 or CRISP.

### Follow-up — ROS observability

Implementation plan: [ROS observability](plans/ros2-observability.md), based on
merged main `2ef41b9`. [Implemented evidence](reports/ros2-observability.md) is
under draft review; Foxglove interactive acceptance passed. PR 14 remains recording.

Add optional read-only Foxglove views of the MuJoCo/CRISP runtime.
Publish standard joint states, TF, target and measured pose markers, plus
inspectable effort, command age, watchdog faults and recovery telemetry. Ship
a saved Foxglove layout. Preserve the shared application
safety path, v1 contracts and ROS-free local learning workflows. The optional
observer and viewers do not own control.

Connect the telemetry to PR 14's MCAP recording and replay work. Foxglove
deployment and licensing requirements must be checked when selecting the viewer.
Gazebo is a separate possible physics adapter, requiring new model, contact,
controller and safety qualification; it is not part of this visualization scope.

### Intermediate upgrade — Articulated gripper and wrist RGB

Implementation branch: `feature/gripper-wrist-camera`, based on `25c6e32`.
[Plan](plans/gripper-wrist-camera.md), [contract](contracts/tooling.md) and
[ADR 019](adr/019-articulated-tool-and-wrist-rgb.md) govern this upgrade.
[Local qualification](reports/gripper-wrist-camera.md) records results and limits.
Add the explicit UR5e/2F-85/D405-inspired RGB preset, assembled URDF/MJCF,
contact grasp and loaded holds, isolated ROS camera projection and Foxglove
layout. Qualify wrist recording/conversion/ACT and reject incompatible model or
camera checkpoints. Local workflows remain ROS-free. Depth/stereo, physical
drivers and mounting, ROS recording, hardware and Gazebo remain deferred.

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
