# PR 12: CRISP feasibility assessment

Date: 2026-10-08. Outcome: **no-go for both assessed configurations; selection deferred**.

## Reproduction and provenance

This historical configuration is retained at merged commit `36be655`. The current
bench uses the explicit [requalification profile](crisp-requalification.md).
To reproduce this original assessment, check out `36be655` and run its bench
contract commands. Full evidence
is generated under ignored `runs/crisp-feasibility/`; CI uploads its own evidence.
This report summarizes a completed local run, not hardware or motion validation.

- CRISP: `0279dc8f1196dab6a2b2cd5af853dd3d4dc3a969` (unmodified).
- UR description: `6b639c2efd9f4f12da858a72cf1a9e40da365ed8`.
- UR driver audit: `f6997bd4a0eee8d58198b696caf034009ab4de1e` (source only).
- Generated UR5e model SHA-256: `512fee40a7a538ce146e3c41aed1b205f6fdf791446d060a6378cfdd9c9dbea3`.
- Jazzy/Noble pinned base from `Dockerfile.crisp`, GCC 13.3.0, Release, native
  optimizations off, upstream precompiled headers on, single compiler job.
- Local CPU: Intel Core i7-13620H; Fast DDS; Python 3.12.3; Pinocchio 4.1.0.
- `provenance.json` records package versions, compiler/CMake flags, source/runner
  fingerprints and machine information. APT repositories are not snapshot pinned;
  future rebuild package versions must be compared with the recorded manifest.

## Controller and safety findings

| Check | Impedance | Operational-space |
|---|---|---|
| Lifecycle / canonical loaned interfaces | Pass | Pass |
| Finite baseline at home, nearby, near-singular wrist | Pass | Pass |
| Planned filter=1 translation / rotation response | Fail / fail | Fail / fail |
| Independent guarded assertions | 63 passed | 63 passed |
| Local 500 Hz p99 below 2 ms in all three runs | Pass | Pass |
| Qualification | No-go | No-go |

The numeric pose-response failure is decisive: CRISP EMA computes
`(1-alpha)*current + alpha*previous`. Target/state filters at 1.0 freeze history;
orientation uses `target.slerp(alpha, previous)` and also freezes. Output torque
passes its operands in the opposite order, so its 1.0 setting does pass new torque.
The assessment retains the requested baseline instead of silently changing it.

A diagnostic target filter of 0.1 produces nonzero effort for a 5 mm translation.
In that probe, stale/future stamps and an unsupported frame are ignored, killing
the separate DDS publisher leaves effort active, and deactivation leaves the
mock command buffer unchanged. These are measured native safety gaps.
Nonfinite position payloads produce nonfinite native effort (JSON nulls in the
trace); doubled nonunit quaternions are not rejected in the frozen baseline.

The pinned update ignores `joint_limit_repulsion.enabled`; the baseline also
sets maximum repulsion torque to zero. Task gains retain pinned defaults of
500 translation / 30 rotation with automatic damping. Operational-space inserts
regularized task inertia, so equal gain values do not imply equal torque.

Gravity output matches the model within 3e-14 Nm. Friction and Coriolis deltas
match their independent model/formula predictions within 1e-14 Nm in both modes.
Mock velocities are set before activation to avoid freezing a zero initial value.
Gravity-enabled raw shoulder effort exceeds the mock test ceiling; compensation
tests record raw output only and do not authorize that configuration for hardware.

The guard independently asserts source/receipt/clock expiry at exactly 100 ms,
pause/reset/fresh-sequence recovery, replay, wrong episodes, future stamps,
invalid frames/numerics/shapes/sequences, finite output, +/-5 Nm and 100 Nm/s.
Fault output is immediately zero outside the slew limit. Every recovery records
actual plugin reconfiguration/reactivation. Python approvals reuse the inbox and
shared safety application; invalid/stale application cases retain accepted gripper
aperture. Synthetic effort injections are separate from raw CRISP output.

Official-model home FK is approximately (-0.4494, 0.1492, 0.7591) m in its world
frame. Its X lies outside the local configured workspace; the shared application
limits the approved intent. The frames/workspace need explicit reconciliation
before moving simulation. No coordinate remapping or workspace relaxation was made.

## Local update timing

Each cell gives the range over three independent home-state runs; times are us.
Each run has 1,000 warmup and 10,000 measured updates. Maximum is worst observed.
Torque delta is scaled to 100 Nm/s at 250/500/1,000 Hz. This is throughput on a
general-purpose host; no realtime scheduling, moving dynamics or hardware deadlines
were validated. Full per-run statistics remain in the JSON evidence.

| Mode | Hz | Median | p95 | p99 | Max | Budget overruns |
|---|---:|---:|---:|---:|---:|---:|
| impedance | 250 | 10.79-10.87 | 10.92-11.00 | 13.04-14.14 | 90.18 | 0 |
| impedance | 500 | 10.72-10.78 | 10.86-10.94 | 12.74-14.01 | 29.55 | 0 |
| impedance | 1000 | 10.73-10.78 | 10.87-10.94 | 12.82-13.94 | 34.06 | 0 |
| operational_space | 250 | 15.22-15.24 | 15.40-15.46 | 17.57-19.90 | 95.64 | 0 |
| operational_space | 500 | 15.23-15.28 | 15.48-16.57 | 19.87-20.95 | 42.59 | 0 |
| operational_space | 1000 | 15.20-15.25 | 15.41-16.02 | 18.76-20.69 | 68.88 | 0 |

## Pinned UR driver audit versus rolling documentation

The pinned source checks below are retained as numbered excerpts in provenance.
They are implementation evidence, not verification on a connected robot.

| Topic | Pinned Jazzy implementation | Consequence |
|---|---|---|
| Effort availability | `hardware_interface.cpp` lines 181 and 377: effort command is optional and exported only when described | Confirm actual deployed URDF exports all six interfaces |
| Feedback units | Lines 582 and 912-915 select motor current or `actual_current_as_torque` | Never infer Nm from the interface name; verify hardware parameters and RTDE recipe |
| Torque command | Lines 1023-1024 forward raw values in `MODE_TORQUE` | No host-side gravity compensation is added here |
| Ownership | Mode compatibility and equal modes across joints, lines 1270-1410 | Effort must own all joints exclusively; validate switching |
| Torque mode gate | Lines 1390-1400: 5.23 / 10.10 | Does not certify the current supported physical software version |
| Actual torque state gate | Lines 798-808: 5.23 / 10.11 | Different from the torque command gate |
| Stop / activation | Lines 1465-1470 and 1520-1528 clear torque buffers | Buffer zeroing is not proof of a physical hold |

[Pinned driver source](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/f6997bd4a0eee8d58198b696caf034009ab4de1e/ur_robot_driver/src/hardware_interface.cpp)
supports these implementation checks. Its `forward_effort_controller` configuration
uses `effort_controllers/JointGroupEffortController`.

The [official rolling torque documentation](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_robot_driver/ur_robot_driver/doc/usage/force_torque_control.html)
inspected on 2026-10-08 instead requires PolyScope 5.25.1 / 10.12.1 and describes
`forward_command_controller/ForwardCommandController`. It says the robot adds
gravity compensation and that supplied torque must exclude it. It also describes
robot-internal friction compensation. Keep these rolling claims separate from
the pinned Jazzy implementation; neither is hardware evidence. Resolve versions,
units, compensation and ownership against the deployed driver/client library
before any hardware work.

## Notices, validation and handoff

The upstream `LICENSE.md` is MIT, `package.xml` declares Apache 2.0, and the
`visibility_control.h` source notice is retained separately. This metadata is
not presented as consistent. The image retains original upstream source/notices;
provenance includes the license file, package declaration and source notice.
Dependency rationale and the dedicated safety matrix are in the bench contract.

- Dev image build, Ruff, mypy and doctor: passed; pytest: 220 passed, 3 skipped.
  Skips are the opt-in display test and two learning tests covered by the CPU image.
- Required CRISP container tests: 5 passed; completed local no-go assessment exits 0.
- Existing ROS contract container tests: 3 passed.
- CPU training/dataset integration checks: 9 passed; teaching notebook executed.
- Default headless simulation smoke: passed, 500 physics ticks, status `ok`.
- Default dev and CPU learning images build without ROS dependencies.
- No hardware, GPU, webcam or moving ROS/MuJoCo validation claimed.

[ADR 014](../adr/014-crisp-feasibility-and-controller-decision.md) defers controller
selection. PR 13 remains blocked until filter configuration and frame semantics
are reviewed and requalified. Production guard, independent watchdog, physical
stop/hold, dynamics/compensation and later hardware commissioning remain required.
