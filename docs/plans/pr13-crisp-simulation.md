# PR 13 implementation plan: ROS/CRISP simulation motion

Status: implemented on the PR 13 branch; see the qualification report and ADR 016.
The home/nearby moving fixtures and near-singular safe rejection are qualified.
Motion through singularities remains outside the qualified fixtures.
Base: merged main `7207adc` (PR #18), verified against origin on 2026-10-09.

## Goal and scope

Run domain Cartesian commands through v1 ROS messages, the shared application
safety path and unmodified pinned CRISP Cartesian impedance, then integrate its
six efforts into the existing UR5e MuJoCo scene. Produce deterministic trajectory
and fault evidence alongside the existing local Cartesian driver.

ADR 015 permits this work conditionally. Hardware, ROS recording, system-clock
synchronization, operational-space qualification for moving control, camera/UI
integration and policy training through ROS remain later work. Preserve public
`Robot.command(Action)` and domain types; simulation/learning remain ROS-free.

## 1. Record the moving adapter design before implementation

Add proposed ADR 016 and `docs/contracts/ros2-simulation.md` covering:

- Separate command gateway, controller-manager process and physics-owner process.
  The physics owner alone touches live MuJoCo state and publishes authoritative
  state/clock, with a fresh episode UUID on reset.
- The gateway owns `CommandInbox` and `SafeCartesianRobot`. ROS callbacks store
  complete command envelopes; the application thread owns safety execution.
- A C++ `ros2_control` SystemInterface exports the six canonical position/velocity
  states and effort commands to the real CRISP plugin. Internal communications
  with physics use bounded, versioned local messages and immutable snapshots.
- Approved execution contains the limited pose, original source timestamp,
  episode/sequence, original receipt time and accepted gripper aperture. Explicit
  hold is distinguishable from an enabled zero-displacement target. If necessary,
  extend the internal CartesianDriver execution seam with a typed approved
  intent/hold descriptor; update local drivers/tests without changing behavior.
  Do not infer authorization from joint movement or revalidate by moving a
  second simulator before executing the real command.
- An actuator-owner watchdog operates independently of approval delivery and
  controller updates. Controller-manager failure cannot stop its fault checks.
  The physics process must continue its hold path after gateway/controller death.
- One command producer and clock authority per episode. Private controller target
  topics do not form the public command interface. This is not authentication or
  a hardware safety controller.

Validate the installed Jazzy controller-manager APIs in the container before
finalizing scheduling details; do not assume that a ROS-clock-driven update loop
will enforce a steady-clock timeout while simulation time is paused.

## 2. Implement and test explicit actuation modes first

Add an optional MuJoCo effort backend, keeping the default MJCF and driver intact.
Use the same geometry, seeds, gripper, contacts and 2 ms physics step.

- ENABLED: six unit-gear torque actuators; arm position/velocity servos disabled.
- HOLD: CRISP effort contribution removed, arm position/velocity hold servos own
  the joints with a latched measured pose and zero desired velocity. Keep the
  last accepted gripper aperture under its existing force-limited position servo.
- Never run arm position/velocity and CRISP effort control concurrently. Record
  ownership transitions and actual per-actuator contributions each physics tick.
- Preserve the existing per-body MuJoCo gravity compensation initially. Keep
  CRISP gravity/friction/Coriolis compensation off. Audit gripper/payload gravity
  and inertia separately; do not add `qfrc_bias` blindly or double compensate.
- Start with the previously reviewed +/-5 Nm task-effort and 100 Nm/s limits.
  Hold-servo contributions are separate, bounded by model actuator limits and
  included in reported total joint effort. Any necessary task-limit change needs
  explicit config/evidence; mock-bench limits are not physical UR limits.
- No direct qpos/qvel freezing, teleporting, or disabling gravity to claim a
  successful dynamic hold. Advance physics under the hold controller and measure
  displacement, settling, contact and gripper retention.

First acceptance gate: single-joint effort sign/units, no conflicting actuators,
stationary hold, moving-to-hold transition, and loaded gripper retention pass
without ROS. Do not proceed to ROS motion if this backend is unstable.

## 3. Connect real ros2_control ownership and CRISP

Add `ros2/act_lab_mujoco_system` and runtime/controller configuration.

- Use the same unmodified CRISP and official UR description revisions as PR 12.
- Keep ADR 015 target/state/output filter settings and Cartesian impedance gains
  explicit. Do not assume torque/gain settings alone prove dynamic stability.
- Enforce canonical joint ordering, exclusive effort ownership and lifecycle
  activation/deactivation/error transitions through actual controller manager.
- Preserve the approved scene-tip to UR-world/tool0 mapping from ADR 015. Check
  the mapping and tool lever arm on moving states; report the model residual.
- Keep IK/contact feasibility checks on scratch MuJoCo state using current measured
  snapshots. All commands, including rejected holds, use shared application safety.
- Publish generated v1 RobotState/CommandReport and `/clock`, using the existing
  ROS = domain + 1 second mapping and QoS. Bind clock/state to the authority's
  episode; reject inconsistent snapshots and stale controller output.
- Gripper commands travel atomically with approved pose intent and retain the
  application aperture on all fault paths.

Acceptance gate: real generated command -> shared safety -> loaded CRISP -> actual
MuJoCo torque integration -> measured state/report, with visible nonzero motion.

## 4. Implement independent watchdog, limits and recovery

Use an actuator-side state machine: STARTUP_HOLD, ENABLED, FAULT_HOLD, RECOVERING,
SHUTDOWN_HOLD. Faults latch until the required fresh authorization and controller
reactivation handshake complete. Output generations must match both episode and
recovery generation; late outputs cannot reactivate a faulted actuator.

- Source age, original receipt age and clock nonprogress expire at >=100 ms.
  Preserve timestamps on repeated polls/heartbeat/output updates.
- Add an independent controller-output progress lease. Duplicate output ticks
  cannot refresh it. A fresh approval alone cannot excuse a hung controller.
- Future, invalid numeric/frame/quaternion, replay, wrong episode, invalid state,
  nonfinite torque and interface/lifecycle failure enter hold immediately.
- Pause continues steady watchdog checks without advancing simulation time. On
  resume, hold is installed before the first physics step; require fresh sequence.
- Backward reset requires a new authority UUID and clears old intent/output.
- Check actual measured workspace, joint limits, Cartesian/joint velocity and
  acceleration, contacts, total effort and task effort slew each physics tick.
  A safe endpoint from IK is not proof that an effort-driven path is safe.
- Fault inhibition bypasses enabled task-effort slew and transfers to the tested
  hold controller. Report the resulting transient; do not call zero torque a hold.
- Graceful shutdown enters hold before relinquishing ownership. SIGKILL tests
  leave the independently running physics owner able to apply hold.

Deterministic watchdog tests inject steady/simulation clocks and exercise exact
boundaries. Process-loss tests use bounded waits to observe the installed hold,
without shared-runner wall-time performance certification.

## 5. Deterministic timing and motion evidence

Maintain 500 Hz physics/controller ticks and 50 Hz application/report samples
(ten physics steps per public environment step). Define startup sequencing to
avoid a circular dependency between publishing `/clock` and computing effort.

Provide a deterministic stepped test mode with explicit tick barriers and injected
steady time. Use the same actuator state machine in the paced runtime. Never
block the physics owner's safety loop indefinitely waiting for a gateway or
controller; missing/mismatched output resolves to hold.

A comparison runner feeds identical source intent traces and seeds to local and
ROS/CRISP backends. Compare timestamp-aligned measured trajectories, command
outcomes, settling, gripper aperture and safety events; do not require identical
joint trajectories from different controllers.

Fixed acceptance cases:

1. Startup hold and no-command hold.
2. Reachable +/-10 mm translation in each world axis and a 0.01 rad orientation
   target, starting at home and two nearby safe configurations.
3. Sustained reachable translation, reverse direction, stop and resume.
4. Gripper close/open and grasp/lift/pause/resume under a held aperture.
5. Producer loss, gateway SIGKILL, controller SIGKILL/stall, output replay,
   clock pause/reset/jump, rejected transport/numeric/frame and shutdown.
6. Joint/workspace/contact boundary cases and near-singular safe rejection.
7. Same-seed/trace repeated execution and snapshot consistency.

For reachable settled targets retain existing 2 mm / 0.02 rad tolerances; every
measured trajectory must respect the current documented motion/joint/workspace
limits. Hold tests run at least 0.5 s after switching and require <=2 mm tip drift,
<=0.02 rad orientation drift and <=0.01 rad/s final arm velocity. Gripper checks
use the existing loaded-hold regression tolerance. Tighten or revise only with an
explicit ADR/config decision and recorded evidence; do not silently relax limits.

Save versioned report, source/limited/measured trajectories, raw/gated/total effort,
actuation modes, events, seeds, configs, model hashes, ROS/RMW/build provenance and
individual failures under ignored `runs/ros2-simulation/`. No MCAP recording change.

## 6. Isolated runtime and CLI

Add an opt-in `ros2-simulation` Compose profile and separate Dockerfile using the
existing pinned Jazzy/Noble base and Fast DDS/localhost discovery. Build generated
interfaces, CRISP and the new ros2_control package. Pin MuJoCo to the repository
version; retain ROS-compatible packaged numerical dependencies and audit the
Python environment rather than installing training dependencies into ROS.

No host networking, devices, privileged mode, display or GPU. Default dev and
learning images remain independent of ROS. Add a lazy CLI such as:

```bash
# Planned commands; exercise before declaring them supported.
docker compose --profile ros2-simulation build ros2-simulation
docker compose --profile ros2-simulation run --rm ros2-simulation \
  act-lab ros2 simulation-smoke --output runs/ros2-simulation --json
```

Missing generated/controller/simulation dependencies must fail with a clear
Compose instruction, not silently fall back or skip required tests.

## 7. Delivery gates

Add required ROS-simulation CI while retaining dev, CPU training, ROS-contract and
CRISP-feasibility jobs. Upload reports/traces even on failure. Shared-runner tests
assert behavior and deterministic boundaries, not hardware realtime latency.

Run dev build, Ruff, mypy, pytest and doctor; optional simulation image build and
required moving/fault tests; existing ROS and CRISP tests; CPU training checks and
teaching notebook; default headless simulation and import isolation.

Complete ADR 016 and dedicated startup/loss/pause/reset/replay/recovery/shutdown
safety matrix with actual evidence. Update architecture, roadmap, README and
dependency rationale. Commit and open one draft PR with validation evidence.
A failed moving stability/safety gate blocks acceptance; preserve failed traces
and the local baseline. Hardware remains a separate roadmap decision.

## Primary references

These explain framework behavior; installed dependency source/version checks
remain authoritative for implementation.

- [Jazzy controller manager and resource manager](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
- [Jazzy controller-manager clocks](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/clocks.html)
- [Jazzy asynchronous controller scheduling](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/running_controllers_asynchronously.html)
- [MuJoCo dynamics and actuation](https://mujoco.readthedocs.io/en/stable/computation/index.html)
- Repository ADR 013, ADR 015, control/simulation contracts and PR 12 evidence.
