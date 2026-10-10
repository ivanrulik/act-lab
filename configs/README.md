# Configuration

Configuration is versioned and reviewed like code. Each run receives a resolved
configuration snapshot. Secrets and machine-specific absolute paths do not
belong here.

Subdirectories are introduced with their owning roadmap PR: `robot/`, `task/`,
`teleop/`, `recording/`, and `training/`.

`sim/ur5e_pick_place.toml` is the authoritative PR 2 scene/runtime
configuration. It records fixed clock rates, reset randomization bounds, task
success thresholds, robot home state, and stable camera names. MJCF geometry
and this configuration must agree; the adapter rejects a timestep mismatch.

The same file owns PR 3 controller settings and PR 4 keyboard/expert settings.
`[control].pose_response_time_s` sets the measured-error correction horizon
(0.10 s); speed and acceleration caps still apply. `[keyboard]` contains held
translation/gripper speeds and legacy nudge sizes. `[expert]` contains waypoint clearances,
tool offset, general/grasp tolerances, dwell counts, and gripper targets. These
values are benchmark inputs and belong in result provenance.

`teleop/webcam.toml` owns webcam capture requests, confidence and frame-age
gates, hysteretic gesture thresholds, calibration stability, position/velocity
mapping, response curves, adaptive filtering, and pinch normalization. Camera
device paths and model filesystem
paths are runtime arguments rather than versioned machine-specific values.

`ros2/crisp-feasibility.json` owns the reviewed stationary qualification profile:
filter retention settings and local scene/UR tool binding. The bench rejects
unreviewed changes. State feedback and target/output filters have different
operand conventions in pinned CRISP; see ADR 015 and the bench contract. Frame
tolerances reuse existing simulation convergence limits and do not relax safety.

`ros2/crisp-simulation.yaml` resolves the moving Cartesian impedance configuration
with the ADR 015 filter conventions, six canonical joints, 0.2 Nm/update torque
change at 500 Hz, and compensation/noise/external wrench/nullspace disabled.
The owner independently caps task effort at 5 Nm and 100 Nm/s. Hold damping and
force ceilings are specified in ADR 016; resolved YAML and model hashes are saved
with runtime evidence. MuJoCo 3.12.0 is the same pinned simulator dependency as
local execution. The optional image adds controller-manager/ros2-control to load
and arbitrate actual interfaces, retaining ROS-compatible apt numerical packages.
Its installed apt/Python manifests and inherited upstream notices are preserved.
