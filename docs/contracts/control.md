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
and collision-causing commands hold observed arm joint positions with zero
commanded velocity and cleared arm rate history. They retain the last accepted,
rate-limited gripper aperture to preserve squeeze under contact deflection;
rejected payloads cannot change it. Fingers may settle toward that aperture.
These expected safety interventions do not raise exceptions.

Cartesian and joint steps use measured feedback, not the previous requested
position. Commanded velocities are retained separately for acceleration limits.
Pose error is corrected over `pose_response_time_s` (0.10 s); commanded
translation is capped at 0.125 m/s. See ADR 009 for servo and hold semantics.

## Teleoperation

Loss of hand tracking, camera frames, focus, or input process must stop motion
within the configured watchdog deadline. Translation and gripper control land
before optional wrist-orientation mapping. Calibration is explicit and stored
with episode provenance.

Webcam control requires `Enter` to collect 15 stable hand samples. Calibration
records the median palm location and scale, detected handedness, current robot
pose, and a stable identifier. Middle, ring, and little fingers confidently
extended for two consecutive frames engage motion. A separate release threshold
prevents marginal landmark jitter from chattering the clutch; a clear release
disengages immediately. Re-engagement anchors the current hand signal to the
current robot pose so clutching never jumps to an old target.

Velocity mapping is the default. Calibrated hand displacement passes through a
dead zone, saturation, nonlinear response curve, and timestamp-aware adaptive
filter, then integrates into a short-horizon Cartesian target. Distance from
neutral controls speed, allowing fine motion near the center and faster coarse
travel near saturation. The original relative position mapper remains an
explicit configuration option.

Mirrored screen-right maps to world `-Y`, screen-up to world `+X`, and a larger
palm scale maps to world `-Z`. The depth proxy uses the logarithm of a projected
multi-span palm-size ratio, excluding MediaPipe's pose-relative landmark Z
values, so equal motion toward and away from the camera is symmetric. It has a
dedicated dead zone and depth-only filtering. Orientation remains the calibrated
robot orientation. Filtering applies before the normal safety envelope.
Thumb/index distance normalized by palm width uses hysteretic closed/open
thresholds so the gripper does not chatter.

Camera capture and inference run independently from the 50 Hz control loop and
share a one-frame latest-value mailbox. Frames that cannot be processed in time
are counted and dropped rather than queued.
Only a new valid hand sample refreshes the action timestamp using the current
observation clock. Missing, older-than-80-ms, non-finite, low-confidence, or
changed-handedness samples disable immediately; retained timestamps guarantee
the normal watchdog subsequently reports stale intent. Camera EOF, worker
failure, uncalibrated state, and clutch release all fail closed.

The interactive webcam viewer remains open after pick/place success or the
configured episode-step timeout so operators can inspect the terminal status;
only `Q`, closing the viewer, or an explicit CLI step limit ends the session.
Its camera overlay displays the clutch anchor, image-plane dead zone, current
hand displacement vector, raw palm-scale depth ratio, signed anchor-relative
XYZ velocity command, clutch score, frame latency, and state. The
3D view displays the requested end-effector target and an actual-to-target
arrow. Target colors distinguish accepted, safety-limited, and rejected
commands; the text HUD separately reports requested pose, actual pose, pose
error, gripper command, tracking health, and the safety decision detail.

Webcam usability validation uses two unscored practice runs followed by seeds
0 through 4. Acceptance requires at least four successful pick/place attempts,
a median successful completion time no greater than 30 seconds, every successful
attempt below 45 seconds, p95 reported capture-to-action latency below 100 ms,
and no unintended gripper transition or safety-bound violation. The report
contains aggregate session metrics and resolved configuration, never camera
frames. This manual result must not be claimed unless a physical camera and X11
viewer were actually used.

`Teleoperator.poll(Observation) -> Action` uses the observation's monotonic
timestamp. With the viewer focused, either Shift key is a held deadman; `W/S`
map to ±X, `A/D` to ±Y, `R/F` to ±Z, and `O/C` open and close continuously.
`Q` closes the session. Orientation stays fixed and diagonal translation is
normalized. Releasing Shift, releasing all motion/gripper keys, or losing focus
emits a disabled measured-pose hold on the next control cycle. Letter matching
is case-insensitive. The optional X11 adapter captures press/release state on a
background thread; simulator access remains on the control thread.

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
