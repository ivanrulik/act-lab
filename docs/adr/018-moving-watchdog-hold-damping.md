# ADR 018: Moving watchdog hold damping

Status: Accepted for the locally qualified legacy simulation; hosted CI pending.
Date: 2026-10-10
Partially supersedes: ADR 016's hold damping values.

## Context

Main CI run 38058439723 failed the ROS motion smoke check. At a gateway wall
timeout, elbow velocity changed from -0.0350275 to -0.0245992 rad/s in one
2 ms tick: 5.21413 rad/s², exceeding the unchanged 4 rad/s² bound. The ten
preceding motion tests passed. Reconstructing the saved six-joint state in the
ROS-free effort plant reproduced the spike without scheduling or DDS.

The fault hold's 50/10 Nm s/rad damping brakes this moving configuration too
abruptly. Torque ceilings alone do not bound measured acceleration.

## Decision

Retain the position stiffness and exclusive bounded PD hold, with damping
30 Nm s/rad for the three proximal joints and 6 Nm s/rad for wrists. The
reconstructed takeover peaks at 3.53158 rad/s² with these values. Preserve the
4 rad/s² acceleration bound, task inhibition, original velocity, continuing
contact dynamics and accepted gripper target. No state freezing or teleportation
occurs during takeover. Record the actual damping in generated provenance.

Add the reconstructed transition to headless integration tests. Require every
tick of its one-second hold to satisfy joint, translation and angular
acceleration bounds and 2 mm tip drift. Re-run existing loaded-gripper hold and
full moving ROS qualification before accepting this change.

## Consequences

The regression distinguishes watchdog scheduling, which selects a takeover
state, from deterministic braking dynamics at that state. It covers the observed
failure rather than proving all possible stopping states safe. New tools,
payloads and broader moving configurations require separate qualification;
these gains supply no physical stop mechanism or hardware safety evidence.

## Local qualification

The required dev image build, Ruff, mypy and doctor pass. Ordinary pytest reports
283 passed and three optional skips. All ten ROS motion tests pass. The separate
generated-DDS motion smoke passes 39 cases, including grasp/lift/fault hold;
measured maximum joint acceleration is 2.48900 rad/s² and maximum tip
acceleration 0.618324 m/s². The reconstructed failing transition peaks at
3.53158 rad/s². Raw evidence remains under ignored
`runs/gripper-camera-baseline/`; hosted CI must confirm the branch before merge.
