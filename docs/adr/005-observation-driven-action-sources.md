# ADR 005: Observation-driven action sources and privileged scripted state

- Status: accepted
- Date: 2026-09-01

## Context

Keyboard input and deterministic teaching baselines produce the same Cartesian
`Action`, but have different information needs. Timestamps must use simulation
time, focus loss must stop through the watchdog, and a scripted baseline needs
exact task geometry that must never become a learned-policy input.

The initial educational gripper also placed its pads on the palm's upward side,
which prevented an above-table grasp and destabilized tracking under contact.

## Decision

`Teleoperator.poll` receives the current dependency-free `Observation`. Device
events retain the observation timestamp at which they were consumed; polling
does not refresh it. Keyboard input therefore relies on the existing 100 ms
`SafeCartesianRobot` watchdog and MuJoCo's supported viewer callback. That
callback exposes discrete press events but not held-key or release state, so PR
4 keyboard control is explicitly tap-to-jog. The viewer overlay reports
accepted events and both requested and observed poses so safety intervention is
visible rather than mistaken for dropped input.

Exact cube pose, desired resting pose, and terminal result live in a separate
`PickPlaceTaskStateSource` port. Only the deterministic application-layer expert
uses it. Generic observations, recordings, learned policies, and training data
do not receive privileged task state.

The expert keeps reset orientation and uses configured Cartesian waypoints,
tolerances, gripper targets, and dwell counts. Every action executes through
`SafeCartesianRobot`. The MuJoCo gripper pads are mirrored to the working side
of the palm, narrow internal model-artifact contacts are excluded, the table
edge leaves arm clearance, and adapter target backoff preserves measured
translation limits.

## Consequences

Keyboard input loss stops without a parallel timeout model. Tap-to-jog is
deliberately conservative and does not provide fluid held-key motion; GPU
rendering cannot change its event cadence. The expert is a reproducible
teaching/evaluation baseline, not deployable perception. Simulator geometry
cannot leak through the generic policy contract. Orientation teleop, webcam
input, recording, learned policies, continuous keyboard teleop, and path
planning remain deferred.
