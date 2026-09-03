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
`[keyboard]` contains nudge sizes. `[expert]` contains waypoint clearances,
tool offset, general/grasp tolerances, dwell counts, and gripper targets. These
values are benchmark inputs and belong in result provenance.

`teleop/webcam.toml` owns PR 5 capture requests, confidence and frame-age
gates, gesture-clutch thresholds, calibration stability, Cartesian mapping,
filtering, and pinch normalization. Camera device paths and model filesystem
paths are runtime arguments rather than versioned machine-specific values.
