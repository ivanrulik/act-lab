# ROS observability implementation evidence

Branch: `feature/ros2-observability`, based on merged main `2ef41b9`.
Scope: optional observational follow-up after PR 13. PR 14 ROS recording,
Gazebo, viewer-driven commands and hardware remain deferred.

## Delivered

- Additive schema-v1 execution telemetry/events with source identity, raw response
  identity, independent injected/wall lease ages and distinct task/hold/total Nm.
- Bounded, nonblocking primitive handoff outside ROS publication; observer
  heartbeat detects stale authority at exactly 100 ms and removes intent markers.
- Measured JointState, authoritative scene-tip TF, namespaced official UR model,
  requested/application-approved/measured pose arrows and retained gripper label.
- Isolated optional headless bridge and separate RViz image/profile, loopback
  forwarding, localhost Fast DDS, saved RViz/Foxglove configurations and lazy CLI.
- Actual generated serialization and WebSocket schema delivery, denied command
  advertisement, parameter read/write, binary service call and forbidden asset;
  allowed UR5e mesh retrieval.
- Dedicated observer safety matrix, dependency rationale, ADR 017 and CI job.

## Local validation

| Gate | Result |
|---|---|
| Compose dev build | passed |
| Ruff / mypy | passed; 62 source files |
| Ordinary pytest | 277 passed, 3 expected optional skips |
| Doctor | Python 3.12, status ok |
| Default headless control-smoke | status ok, 50 steps / 500 ticks |
| Rebuilt CPU training image / ROS isolation | passed / no rclpy installed |
| CPU dataset/training tests | 9 passed; actual ACT test included |
| Teaching notebook | executed successfully |
| Existing ROS v1 contracts | 3 passed |
| Pinned stationary CRISP bench | 5 passed in its dedicated image |
| Existing moving ROS tests | 10 passed |
| Observability optional tests | 4 passed |
| Deterministic observational-read agreement | identical per-tick effort/state/hold traces; included in ordinary tests |
| Headless RViz startup | saved config opened; software OpenGL 4.5; corrected TF QoS, no remaining reliability warning |

The ordinary suite skips LeRobot tests in dev (run separately in training) and
one explicit X11 interactive replay test. The CRISP tests were first attempted
in the moving image, whose PATH intentionally lacks `crisp_bench`; they failed
explicitly, then all five passed in the correct dedicated bench image.

Ignored `runs/observability-*` directories contain reports, physics traces,
observer counters, resolved package/config/model hashes, native process logs,
WebSocket results and screenshots. CI uploads its corresponding evidence even
on failure. Timing from these runs does not certify hardware realtime behavior.

## Viewer validation and limits

The signed-in Foxglove web application (dashboard announced 3.3) connected to
`ws://localhost:8765`. The repository layout imported successfully with user
assistance because browser automation did not open its file chooser. Actual
live model geometry, effort/age plots, generated telemetry, command reports,
FAULT_HOLD, fresh recovery, new episodes and shutdown were inspected. The final
3D and effort panel settings were applied from the repository JSON through the
application's import/export settings dialog.

Validation found and fixed four concrete issues: entrypoint PYTHONPATH overriding
ROS setup, current SDK subprotocol selection, ROS byte diagnostic encoding, and
RViz reliable TF compatibility. The bridge now uses simulation time, avoiding
wall/source epoch disagreement in plots. Meshes/TF can remain cached after a
bridge disconnect, while the connection warning and STALE marker identify lost
live data. Age plots can show a generic int64-to-float warning; these bounded
nanosecond ages remain below the exact-integer range of float64.

RViz was built and launched with the saved configuration under Xvfb; interactive
desktop motion/hold/reset inspection is **pending** because this session has no
native desktop control surface. It is not claimed as passed. Headless startup
is bounded by a timeout; its termination can log an X11 connection shutdown.

No hardware, GPU, camera, Gazebo, ROS recording or cloud recording was validated.
Foxglove application licensing remains distinct from its MIT-declared ROS bridge;
no account upgrade or agreement was accepted by this implementation.
