# ADR 008: Velocity-mapped webcam control with adaptive filtering

- Status: accepted
- Date: 2026-09-03
- Supersedes: the position-mapping and fixed-filter portions of ADR 006

## Context

PR 5 proved that a single unprivileged webcam can safely produce Cartesian
intent, but live use exposed poor task ergonomics. Relative position mapping
made the operator wait for the robot after the hand reached its comfortable
range. Fixed exponential smoothing reduced jitter by adding noticeable lag,
and binary gesture thresholds caused unnecessary clutch transitions.

## Decision

Make velocity mapping the default while retaining the original position mode
as a configuration-controlled comparison. Calibrated palm displacement acts as
a three-axis analog input: a dead zone suppresses rest noise, a nonlinear
transfer function provides precision near neutral and faster travel farther
away, and integration produces short-horizon world-frame pose targets. The
stable domain `Action` and `SafeCartesianRobot` path do not change.

Use timestamp-aware 1 Euro filters independently for image-plane and depth
signals. Use hysteretic clutch and pinch thresholds. A clearly released clutch,
tracking loss, stale or malformed input, worker failure, and handedness changes
still disable motion. Capture and inference use a one-frame mailbox so excess
camera frames are dropped instead of queued.

Separate the application command-speed limit from the adapter's measured-speed
ceiling. Both remain explicit configuration and the command limit may not
exceed the measured ceiling.

## Consequences

The hand behaves like a low-cost analog controller rather than a metric 3D
tracker. This is less literal than pose mirroring but supports continuous coarse
travel and fine acquisition with a monocular camera. Recorded input remains
deterministic, live overload favors freshness over processing every frame, and
all safety, framework, and dependency boundaries remain intact.
