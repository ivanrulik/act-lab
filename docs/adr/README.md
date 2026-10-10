# Architecture decision records

ADRs capture decisions that future contributors should not rediscover or change
implicitly. Accepted ADRs remain in the repository even when superseded.

Use `000-template.md`, assign the next number, and state context, decision,
consequences, and status.

- [ADR 011](011-deterministic-episode-selection-and-conversion.md): deterministic
  quality selection, episode splits, resampling, and LeRobot conversion
- [ADR 012](012-python-312-and-act-training.md): Python 3.12 migration and pinned
  LeRobot ACT training/checkpoint integration

- [ADR 013](013-ros2-executable-contracts.md): optional Jazzy executable contracts,
  simulation epoch mapping, replay guards, and independent steady watchdogs

- [ADR 014](014-crisp-feasibility-and-controller-decision.md): pinned stationary
  CRISP assessment, test-only effort guard, no-go for the assessed configuration

- [ADR 015](015-crisp-configuration-requalification.md): corrected filter and frame
  qualification; conditional go for Cartesian impedance in moving simulation

- [ADR 016](016-crisp-moving-simulation.md): independent physics ownership,
  bounded dynamic hold and measured frame residual correction for moving CRISP

- [ADR 017](017-read-only-ros-observability.md): optional bounded observational
  handoff, namespaced model and read-only RViz/Foxglove projections

- [ADR 018](018-moving-watchdog-hold-damping.md): moving watchdog takeover
  regression and revised bounded hold damping

- [ADR 019](019-articulated-tool-and-wrist-rgb.md): explicit articulated tool,
  synthetic wrist RGB, process isolation and learning compatibility
