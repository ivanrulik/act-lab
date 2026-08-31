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

## Teleoperation

Loss of hand tracking, camera frames, focus, or input process must stop motion
within the configured watchdog deadline. Translation and gripper control land
before optional wrist-orientation mapping. Calibration is explicit and stored
with episode provenance.

## Physical hardware

Application safety is defense in depth and never replaces manufacturer safety
functions, risk assessment, protective stops, or trained operator supervision.
Physical integration requires a separate ADR and safety review.
