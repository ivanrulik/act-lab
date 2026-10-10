# ADR 019: Articulated 2F-85 and synthetic wrist RGB

- Status: Accepted for this bounded simulation preset.
- Scope: explicit `ur5e_2f85_d405_v1` simulation preset.
- Partially supersedes: ADR 009 educational gripper binding and ADR 016 fixed
  tool translation, for this preset only.
- Also supersedes: ADR 015's default translational gains for this preset only.

## Decision

Use the pinned Menagerie articulated Robotiq 2F-85, retaining its original
assets, closed linkage, contact parameters, actuator and notices. Compose it at
`tool0` with a grasp TCP 0.1558 m along flange Z. The existing educational model
remains the default. Domain gripper intent remains 0 closed, 1 open; feedback is
measured inner-pad separation divided by 0.085 m. Empty-stroke calibration maps
intent to the original 0–255 actuator input. Blocked closure retains actual
joint deflection; collision prediction does not invent an unloaded closed pose.

The visualization URDF is an articulated tree with all eight measured tool
joints. Closed-loop equality constraints remain in MuJoCo. No exact mimic claim
is made. The six-interface CRISP controller uses a separately generated rigid
payload model frozen at a nominal 50 mm aperture. This preserves the six arm
DOFs and includes tool inertia, but approximates changing articulated dynamics.
Gravity compensation remains in the simulator bodies; CRISP gravity, friction
and Coriolis compensation remain disabled. Hardware payload/dynamics validation
is deferred. Existing torque, acceleration and freshness ceilings apply.

The new controller configuration uses 1000 N/m translational stiffness instead
of the pinned default 500 N/m. At the 0.08 m/s application cap, the original
short-horizon target generated nearly only the 80 g cube's supporting force,
stalling lift. The qualified loaded source closes at 0.1 normalized aperture/s
and confirms opening separately (0.05 normalized tolerance). Faster inputs
remain subject to the independent physical contact/acceleration guard. Neither
the source cadence nor the gain setting changes the actuator force ceilings.

Use original primitive geometry for a D405-inspired camera and bracket. Camera
optical axes are +Z forward, +X right, +Y down. Its optical frame is translated
(0, 0.060322528, 0.035505774) m from the gripper mount and tilted 24 degrees
about mount X toward the grasp region. A parallel lateral camera clipped the
held cube; the angled mount keeps the cube within the loaded image bounds.
Synthetic RGB8 uses 320×240, vertical FOV
58 degrees, a calculated pinhole K, and zero distortion. Calibration, mounting
and model content hashes identify every acquisition. This is not a physical
D405 camera-mode or optical-fidelity claim; depth/stereo/SDK are deferred.

Local recording renders the synchronized observed state. The optional ROS
viewer has a separate scratch-model process, fed through a separate bounded
64 KiB state channel. Image bytes never enter the physics-owner telemetry slot.
The renderer calls forward kinematics without stepping physics and drops
expired or cross-reset results. Image staleness expires at 100 ms using original
steady capture time. Rendering cannot authorize motion. Standard Image and
CameraInfo are projections of an additive typed CameraSample with episode,
sequence, tick and hashes; existing v1 command/state/report messages stay intact.

New datasets/checkpoints carry the model and calibrated camera contract. Mixed
model/calibration conversion and incompatible inference fail explicitly. A new
policy refuses missing/mismatched/stale wrist frames with disabled intent,
which executes the existing gripper-preserving safety hold. Legacy checkpoints
remain usable with the legacy setup. Training/evaluation remain ROS-free.

## Consequences and validation

The new preset needs its own reachability, loaded contact and controller
qualification. It has a lower command translation speed (0.08 m/s) and a longer
15 second episode window to avoid near-singular tracking overshoot. These are
preset changes, not relaxed safety ceilings. Stationary URDF FK has been checked
against MuJoCo for all tool links across the stroke using independent Pinocchio.
Contact-loaded, fault, RGB, data and viewer evidence is recorded in the
[branch qualification report](../reports/gripper-wrist-camera.md).

Software rendering may deliver less than its 25 Hz projection target. Dropped
frames and source age are observable; there is no claim of 50 Hz wall-clock
acquisition, real-time hardware behavior, or safe physical hold.
