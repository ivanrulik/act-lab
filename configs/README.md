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
