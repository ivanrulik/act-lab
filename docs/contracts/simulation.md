# Deterministic simulation contract

## Scene and frames

The PR 2 scene is a fixed-base UR5e with a coupled parallel-jaw gripper, table,
5 cm cube, and fixed tray. MuJoCo remains inside `act_lab.adapters.mujoco`;
domain observations contain only Python values in SI units.

`world` is the MuJoCo world frame and the fixed robot base origin. End-effector
poses are reported in `world`, positions are metres, joint angles are radians,
and quaternions use WXYZ ordering. `overview` and `policy` are stable RGB camera
keys. Camera images are 320 by 240 pixels.

## Clock and reset

Physics runs at 500 Hz. One environment step advances exactly ten physics steps,
so observations run at 50 Hz. Timestamps are integer multiples of 2,000,000 ns
and begin at zero after reset; wall-clock time never orders simulation state.

`reset(seed)` restores all MuJoCo data, controls, velocities, task counters, and
the environment RNG before sampling the cube. Equal seeds and equal actuator
targets must produce equal state sequences on the supported container image.

## Randomization

The tray is fixed at `(0.58, 0.23)` in the world XY plane. Cube centres sample
uniformly from `x=[0.35, 0.52]`, `y=[-0.22, -0.08]`, at `z=0.425`. These bounds
are on the table, do not overlap the tray or robot base, and lie within the
UR5e's geometric reach. Controller-level reachability and collision feasibility
are enforced by the safe Cartesian path.

All authoritative values live in `configs/sim/ur5e_pick_place.toml`; this page
documents rather than overrides them.

## Task success

Success requires every condition below for ten consecutive 50 Hz steps:

1. The complete cube footprint is inside the tray's 0.24 m by 0.20 m interior.
2. The cube bottom is within 0.008 m of the tray floor top.
3. Linear speed is at most 0.02 m/s and angular speed at most 0.10 rad/s.

The adapter reports each predicate, the settling count, terminal state, and
reason. An episode times out after 500 environment steps. A reset clears prior
success and timeout state.

The separate dependency-free `PickPlaceTaskState` reports cube pose, desired
resting pose at tray center, and terminal result for privileged scripted
teaching/evaluation. It is not part of `Observation` or a dataset feature.

## Control boundary

`ActuatorTargets` is an adapter-local seam for tests and neutral rollouts. It is
not operator or policy intent and does not implement `Robot.command(Action)`.
The application `SafeCartesianRobot` translates domain Cartesian `Action`
values through safety filtering before the MuJoCo driver emits these targets.
The driver uses the end-effector site Jacobian, shortest-path quaternion error,
and damped least squares with damping 0.05, at most 50 iterations, 2 mm
position tolerance, and 0.02 rad orientation tolerance.

Candidate targets are clipped inside model joint ranges with a 0.02 rad margin
and checked with `mj_forward` before execution. Robot self-contact and robot
contact with the floor, table, or tray stop the command. Finger-pad/cube contact
is allowed for grasping; other robot/cube contact stops. During a confirmed pad
grasp, the checker also classifies the wrist capsule's known coarse overlap with
that same cube as part of the pad interaction. The scene explicitly excludes
only persistent coarse-collision overlaps between the wrist capsules and the
elongated educational fingers. Reset has no unclassified contacts and clears
safety, watchdog, and controller-rate history. Per-link gravity compensation
lets the position actuators meet the documented Cartesian tolerance without
changing the cube's task dynamics.

The educational gripper pads extend from the working side of the palm for an
above-table grasp. Narrow wrist/finger exclusions represent internal coarse
geometry artifacts only; external contacts remain checked. The table begins at
world X=0.30 m, supporting all cube spawns and the tray while clearing the
fixed-base arm. Servo damping and deterministic target backoff preserve the
measured Cartesian ceiling.

`act-lab sim control-smoke --seed SEED --steps STEPS --json` exercises this path
deterministically. It is also the Compose `sim` service command. Direct
environment stepping remains an adapter-local test seam.

`act-lab sim expert --seed-start 0 --episodes 20 --min-success-rate 0.90
--json` runs the headless baseline. JSON includes aggregate success rate and
per-seed steps, result, terminal reason, final FSM phase, and safety counts.
Exit codes are 0 for a passing threshold, 1 for failure, and 2 for invalid
configuration or runtime input.

Unknown cameras, invalid configuration, non-finite controls, out-of-range
gripper values, closed environments, and non-finite simulator state fail
explicitly.

## Interactive viewer

`act-lab sim view` advances the same fixed-step environment at real-time 50 Hz
while a passive MuJoCo viewer is open. Closing the window ends the process. The
Compose `sim-ui` service forwards only the host's read-only X11/XWayland socket
and uses Mesa software rendering; the headless `sim` service remains the CI and
automation interface.

`act-lab sim keyboard-teleop --seed 0` uses the passive viewer key callback and
overlays controls, accepted input count, target and actual poses, latest safety
outcome with detail, and task state. Viewer and control threads synchronize
access to shared MuJoCo state through the viewer lock. The callback exposes
discrete presses, not held-key state; keyboard teleoperation is therefore
tap-to-jog. Its opt-in Compose service has the same narrow X11 mount as
`sim-ui`.
