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
- [ADR 020](020-gpu-viewer-and-compressed-preview.md): explicit GPU viewing,
  JPEG transport and an independently identified software preview profile.

- [ADR 021](021-ros-mcap-acquisition-equivalence.md): acknowledged stepped ROS
  acquisition, rosbag2 MCAP, canonical import and ROS-free dataset equivalence.

- [ADR 022](022-stepped-acquisition-hold.md): explicit authorization revocation
  before offline CRISP rendering and recorder ACK waits.

- [ADR 023](023-compose-ci-build-cache.md): Compose-derived cached CI builds,
  bounded optional cache exports and superseded PR cancellation.

- [ADR 024](024-paced-acquisition-worker.md): bounded asynchronous paced
  acquisition, capture overload/loss holds and explicit rendering identities.

- [ADR 025](025-crisp-task-qualification.md): explicit full-task source cadence,
  coherent owner task state and bounded acquisition transport qualification.

- [ADR 026](026-loaded-watchdog-hold-damping.md): model-specific loaded watchdog
  hold stiffness/damping and deterministic acceleration regression.

- [ADR 027](027-explicit-curated-hub-releases.md): opt-in curated publication,
  preserved artifact identity and verified Hub commit downloads.

- [ADR 028](028-explicit-camera-study.md): calibrated camera projection and
  bounded visibility/learning comparison with explicit no-go gates.

- [ADR 029](029-causal-derived-action-alignment.md): opt-in v3 causal command
  labels before resampling, preserving v2 reproduction and raw acquisition.

- [ADR 030](030-requested-intent-training-labels.md): opt-in v4 requested-intent
  labels with executed fault holds and unchanged application safety.

- [ADR 031](031-explicit-inference-temporal-ensemble.md): explicit optional ACT
  inference ensemble with fresh prediction and reset/fault history clearing.
