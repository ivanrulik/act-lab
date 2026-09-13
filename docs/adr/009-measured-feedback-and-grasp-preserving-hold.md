# ADR 009: Measured feedback and grasp-preserving simulation hold

- Status: accepted
- Date: 2026-09-12
- Supersedes: ADR 004 position-history and gripper-hold semantics

## Context

Advancing controller history to requested positions hides actuator lag. The
velocity-servo implementation also regressed the scripted baseline. Holding
measured finger aperture under contact deflection removes the position error
that produces squeeze force; releasing the deadman can therefore release a
grasp without an open command.

## Decision

Compute Cartesian and joint corrections from measured state and refresh position
history after every driver step. Keep commanded velocity history separately for
acceleration limiting. Correct pose error over a configurable 100 ms response
horizon rather than attempting to eliminate it in a single 20 ms cycle. Keyboard
velocity intent uses that horizon relative to measured pose, without accumulating
an unreachable target. Commanded translation remains limited to 0.125 m/s and
the adapter's measured control-cycle displacement ceiling remains 0.25 m/s.

Use explicit arm velocity servos with gain 600, alongside position servos, instead
of the unstable gain 10000. Increase the educational gripper position gain from
200 to 800 while retaining its 20 N actuator force limit, geometry, mass, and
friction. This is simulated actuator tuning, not a hardware model.

Disabled, stale, invalid, infeasible, and collision-rejected commands discard arm
targets, command measured joints with zero velocity, and reset arm rate history.
They retain only the last accepted, rate-limited gripper aperture, not a new
rejected request or the final aperture of an unfinished ramp. Reset initializes
that aperture from measurement. Webcam clutch re-engagement retains pinch intent
while re-anchoring the arm to measured pose; calibration initializes the gripper.

## Consequences

An input-loss hold maintains squeeze force and does not intentionally open an
object. Fingers may settle toward the retained aperture under load; this is not
an emergency power-off or a guarantee of zero finger motion. Explicit valid open
commands remain available. Physical hardware requires its own fail-hold versus
fail-release risk assessment before adopting these semantics.

Regression coverage includes lag, pose convergence, sustained translation,
lift/pause/resume under disabled and stale input, and the seeded expert benchmark.
Scripted success does not establish manual demonstration readiness: keyboard and
physical-camera pick/place acceptance must be reported separately.
