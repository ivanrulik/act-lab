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
- Isolated optional headless bridge, loopback forwarding, localhost Fast DDS,
  saved Foxglove configuration and lazy CLI.
- Actual generated serialization and WebSocket schema delivery, denied command
  advertisement, parameter read/write, binary service call and forbidden asset;
  allowed UR5e mesh retrieval.
- Dedicated observer safety matrix, dependency rationale, ADR 017 and CI job.

## Local validation

| Gate | Result |
|---|---|
| Compose dev build | passed |
| Ruff / mypy | passed; 62 source files |
| Ordinary pytest | 282 passed, 3 expected optional skips |
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

The user selected Foxglove as the sole bundled viewer. The RViz image, profile
and configuration were removed; desktop RViz acceptance is no longer in scope.
Standard ROS JointState, TF and model projections remain available for future
independent viewer debugging. Interactive Foxglove acceptance passed.

No hardware, GPU, camera, Gazebo, ROS recording or cloud recording was validated.
Foxglove application licensing remains distinct from its MIT-declared ROS bridge;
no account upgrade or agreement was accepted by this implementation.

## PR review follow-up

Initial CI completed five of six jobs successfully. The moving ROS job completed
its 39 motion/fault cases, then failed shutdown: the owner acknowledged before
writing its physics trace, and the parent could terminate it after five seconds.
Shutdown now acknowledges only after atomic evidence publication and resource
cleanup, with bounded finalization waits and explicit failure propagation.
Compact streamed JSON avoids creating a second large formatted JSON string.
Regression coverage includes delayed finalization, failed owner exit, invalid
trace publication, and terminal shutdown-hold evidence in the moving DDS test.
The 100 ms motion watchdogs are unchanged.

Follow-up validation: dev and Foxglove runtime builds passed; Ruff and mypy
passed; ordinary pytest passed 280 tests with the same three optional skips;
doctor returned ok. The moving ROS suite passed all 10 tests in 154.81 seconds,
including finalization of 66,113 physics rows (110,751,447 bytes) with terminal
shutdown hold and no partial artifact. The four observability tests passed in
9.84 seconds. Hosted CI is rerun for the updated commit before merge.

The next hosted run passed all four observability tests but its separate evidence
command hit a busy shared slot on its one-shot final read. Its observer artifact
recorded 265 samples, confirming telemetry was present. The report reader now
retries for a bounded two seconds without restamping data or making owner offers
blocking. Regression tests cover transient contention and persistent missing
data at an injected deadline.

The final local ordinary suite passed 282 tests with the same three optional
skips. Ruff and mypy passed. The actual `observability-smoke --json` command
completed with delivered telemetry and bridge-denial evidence; its provenance
contains only the two retained Foxglove configurations.

Hosted moving ROS validation confirmed the original full motion scenario now
passes and publishes complete evidence. Its SIGKILL fault test also verified
independent hold, but cleanup correctly reported the deliberately killed gateway
exit (-9). The test now explicitly expects that diagnostic while retaining all
owner hold, effort, gripper and elapsed-simulation-time assertions.

The corrected deliberate-gateway-death test passed locally in 1.34 seconds;
Ruff passed after the assertion update. Superseded CI was cancelled and a final
run requested for the combined fixes.
