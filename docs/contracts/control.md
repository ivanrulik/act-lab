# Control and safety contract

## Coordinate convention

All poses must identify their reference frame. Domain positions use metres,
joint angles use radians, velocities use SI units, and quaternions use WXYZ
ordering. Adapters perform conversion at their boundary.

`Pose.frame_id` is mandatory. The local simulation currently reports
end-effector poses in the `world` frame defined by the simulation contract.

## Action semantics

An `Action` is operator or policy intent, not permission to bypass safety. It
contains a monotonic timestamp, Cartesian target, normalized gripper target,
and enable state.

Before execution, an action must pass:

1. freshness/watchdog check;
2. enable/clutch check;
3. finite-value and range validation;
4. workspace and joint-limit filtering;
5. velocity and acceleration limiting;
6. controller feasibility checks.

Any ambiguous state defaults to no motion.

The application implementation validates in this exact order: timestamp
freshness, enable state, finite values, `world` frame, unit quaternion,
normalized gripper range, workspace clipping, Cartesian/angular/gripper rate
limits, IK, joint bounds/rate limits, and predicted contacts. A timestamp at or
beyond the 100 ms watchdog boundary is stale; a future timestamp is invalid.

Valid poses are clamped to `x=[-0.20, 0.80]`, `y=[-0.35, 0.60]`, and
`z=[0.44, 1.00]` metres. Translation is bounded by 0.25 m/s and 1.0 m/s²,
orientation by 1.0 rad/s and 4.0 rad/s², every arm joint by 1.0 rad/s and
4.0 rad/s² with a 0.02 rad bound margin, and the normalized gripper by 2.0
units/s. The simulator controller retains conservative tracking headroom below
the translation ceiling so measured discrete-time motion also respects the
hard limit.

`Robot.command(Action) -> RobotState` is unchanged. After every command,
`SafeCartesianRobot.last_report` exposes a `CommandReport` with the requested
intent, executed limited intent when applicable, resulting state, detail, and
one outcome: `applied`, `limited`, `disabled`, `stale`, `invalid`,
`ik_failure`, or `collision_stop`. Disabled, stale, invalid, non-convergent,
and collision-causing commands hold the observed joint and gripper positions.
These expected safety interventions do not raise exceptions.

## Teleoperation

Loss of hand tracking, camera frames, focus, or input process must stop motion
within the configured watchdog deadline. Translation and gripper control land
before optional wrist-orientation mapping. Calibration is explicit and stored
with episode provenance.

`Teleoperator.poll(Observation) -> Action` uses the observation's monotonic
timestamp. Keyboard events add 0.01 m world-frame translation or 0.1 normalized
gripper nudges. `W/S` map to ±X, `A/D` to ±Y, `R/F` to ±Z, `O/C` open and
close, Space emits disabled hold, and `Q` closes the session. Orientation stays
fixed. Polling without an event retains the previous input timestamp, so the
normal watchdog stops pursuit at 100 ms. Letter matching is case-insensitive.
The supported passive-viewer callback reports discrete presses rather than
held-key state, so this PR intentionally provides tap-to-jog rather than
continuous motion. Safety and actuator dynamics may stop before the observed
pose reaches the accumulated target; another press adds another nudge and
refreshes the input timestamp.

All keyboard and scripted-expert actions execute through `SafeCartesianRobot`.
The simulator driver may deterministically scale an actuator target from a
restored pre-step state when servo tracking would exceed the measured Cartesian
velocity ceiling.

## Scripted baseline

The deterministic expert runs open/raise, approach, descend, close/dwell, lift,
transit, lower, open/dwell, retreat, and disabled-hold phases. It advances only
inside configured observed pose/gripper tolerances and dwell counts. Timestamps
refresh each step. Terminal task state, timeout, IK failure, invalid/stale
intent, or prohibited contact ends in disabled hold.

Exact cube and goal poses arrive through the privileged task-state port. They
must not be copied into policy observations, recordings used as policy inputs,
derived training data, or learned-policy evaluation inputs.

## Physical hardware

Application safety is defense in depth and never replaces manufacturer safety
functions, risk assessment, protective stops, or trained operator supervision.
Physical integration requires a separate ADR and safety review.
