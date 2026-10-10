# ROS observability — implementation plan

Status: Implemented for draft review; interactive RViz acceptance remains pending.
Evidence: [validation report](../reports/ros2-observability.md).
Base: merged main `2ef41b9497c046dbd4e4b4c91a3d7fe2d3729911`.
Branch: `feature/ros2-observability`.
Roadmap: focused follow-up after PR 13, before PR 14 ROS recording.

## 1. Outcome and boundary

Make the existing ROS/CRISP/MuJoCo demonstrator inspectable in RViz2 and Foxglove:
a moving robot, requested/approved/measured poses, six joint efforts, command age,
clock progress, actuator ownership, faults and fresh-command recovery.

Viewers consume telemetry. Keep the existing command producer, physics owner,
shared `CommandInbox` / `SafeCartesianRobot`, 100 ms watchdogs and dynamic hold.
Preserve generated v1 command/state/report contracts and ROS-free learning.
Gazebo, ROS MCAP recording/replay, hardware, teleoperation from viewers, camera
streams and operational-space motion remain separate roadmap work.

Acceptance includes a useful live view and verifiable failure behavior. Screenshots
are presentation evidence; headless ROS tests verify the telemetry and isolation.

## 2. Design records and dependencies

During implementation, add proposed ADR 017 and
`docs/contracts/ros2-observability.md`, then complete them with actual evidence.
ADR 017 extends ADR 016 with optional observational projections; it does not
supersede actuation, clock or frame-correction decisions.

Use the existing pinned Jazzy/Noble runtime, Fast DDS and official UR description
revision. Add only justified viewer dependencies: robot_state_publisher, tf2_ros,
visualization_msgs, diagnostic_msgs, RViz2, resource retrieval and Foxglove bridge.
Build RViz separately from the headless observability runtime. Keep the default
simulation, dev and learning images free of viewer packages.

Inspect the available Jazzy bridge version and its installed launch/configuration
API before choosing it. Pin the tested bridge source revision or package version;
record its license, retained notices and resolved package versions. Review the
Foxglove application separately from the bridge: document current account/license
requirements and supported local connection/layout workflow. Do not assume a free,
redistributable or self-hostable application, or enroll the user in a paid plan.
RViz is the repository's independently usable viewer; Foxglove artifacts and
protocol tests must remain usable without a paid CI account.

## 3. Observational data contract

Use pure Python primitive telemetry models and codecs, with generated ROS types
loaded lazily in the optional adapter. Publish coherent authority snapshots at
50 Hz simulation time, retaining the original source timestamp and sequence.

Proposed topics (finalize and document QoS during implementation):

| Topic | Type | Purpose / QoS |
|---|---|---|
| `/joint_states` | sensor_msgs/JointState | Six canonical UR5e measured positions/velocities and total actuator effort; best effort, volatile, depth 1 |
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | Namespaced model FK and authoritative scene-tip transform; standard tf2 profiles |
| `/act_lab/view/robot_description` | std_msgs/String | Visualization model; reliable, transient local, depth 1 |
| `/act_lab/view/poses` | visualization_msgs/MarkerArray | Requested, approved and measured axes/arrows; best effort, volatile, depth 1 |
| `/act_lab/view/telemetry` | new ExecutionTelemetry message | Typed owner execution/effort/lease snapshot; best effort, volatile, depth 1 |
| `/act_lab/view/events` | new ExecutionEvent message | Ownership/fault/reset/recovery changes with episode, sequence and generation; reliable, volatile, bounded depth |
| `/act_lab/view/health` | diagnostic_msgs/DiagnosticArray | Observer health, dropped events and receipt freshness; low-rate steady heartbeat |

New observational messages are additive to `act_lab_interfaces`; existing v1
message definitions and their QoS remain unchanged. Pin the telemetry schema
version and document optional fields using explicit validity/presence flags.

Telemetry carries domain/ROS stamp, episode UUID, physics tick, generation,
original command identity, mode/reason and accepted gripper aperture. Include six
raw CRISP, gated task, hold-servo and total actuator efforts in Nm. Total actuator
effort excludes body compensation/contact forces. Raw output has a validity flag;
unavailable/nonfinite raw output has explicit diagnostic text, never an apparently
valid fabricated torque sample.

Show simulation source age and injected lease ages separately from real steady
receipt age, clock nonprogress, output nonprogress and observer receipt age.
Mark unavailable values explicitly. Viewer reception never renews motion authority.
Keep application `CommandReport` outcomes distinct from owner execution decisions:
approved intent can still be inhibited by an independent owner watchdog.

All requested/approved intent enters the view through observational copies from
the gateway's real safety path; measured state/effort/mode originates only from
the physics owner. Join by episode/sequence/tick, not arrival order. Never infer
approved intent from measured movement or an application outcome alone.

## 4. Keep observation independent of control

Use a bounded, nonblocking snapshot/event handoff to a separate telemetry process.
The owner copies primitives after an actual tick or ownership transition; it does
not wait for viewers, bridge acknowledgments, event delivery or robot model loading.
Use a latest-value slot for samples and a bounded event queue with visible overflow
counts. Capture rejected command identity and fault metadata before guard cache
clearing, solely as observational data; it cannot be reused as authorization.

Put ROS conversion and potentially reliable publishing in the observer process.
A slow/dead observer can lose samples/events but cannot change control decisions.
This stream is diagnostic, not the durable acquisition record planned for PR 14.

Centralize this capture across normal ticks, controller-wait hold, preparation,
idle wall-watchdog hold, reset, process loss and shutdown. The recent main CI
failures make these paths required fixtures, including wall-first expiry and late
native output. Publish a final held sample while the runtime is still alive.

Use an injected steady clock in the observer for freshness logic. If authority
input stops for 100 ms, diagnostics report stale/unavailable and the marker layer
removes or greys active intent. A heartbeat can diagnose pause with frozen ROS
time. If the observer itself dies, document each viewer's actual connection-loss
behavior; never imply that a cached robot rendering proves live state.

## 5. Frames and robot appearance

Fixed viewer frame is domain `world`. Every stamp follows ROS = domain + 1 second.
Require use_sim_time for ROS viewer/model processes. Publish authoritative
`act_lab_scene_tip` from the domain measured pose in world coordinates.

Use the existing pinned official UR description for the arm visuals, installed
meshes and six measured joints. Give its root/links a visualization namespace,
with an explicit scene-world to model-world transform derived from ADR 016's
Rz(pi) mapping. Run a namespaced visualization robot_state_publisher; it must not
replace CRISP's existing robot_description parameter source.

Verify the tool0 offset, rotation and FK at home and both nearby PR 13 fixtures.
Display model-tool and actual scene-tip frames separately and report the reviewed
residual. Do not force the official model FK to equal the MuJoCo tip or introduce
a second parent for a TF child. Retain the 2 mm / 0.02 rad residual checks.

Show the gripper aperture as a labeled marker initially; do not invent URDF joints
or copy unsupported mesh geometry. Optional table/cube/tray markers use actual
scene geometry and owner snapshots; if included, label privileged scene data as
visual diagnostics and keep it outside policy observations.

On episode reset, clear pose history/events for the prior episode, delete old
markers and clear/restart visualization TF buffers/model publication as needed.
Test backward ROS time explicitly; do not blend old and new episode trajectories.

## 6. Compose runtime and operator workflow

Add opt-in profiles `ros2-observability` and `ros2-rviz`, with a headless telemetry /
bridge image and a separate RViz image. Supervise demo, observer, model publisher
and bridge as separate processes, with explicit startup and shutdown ordering.

Keep localhost DDS discovery by sharing the runtime's network namespace with any
separate viewer container (`network_mode: service:<runtime>`). Do not assume
localhost discovery works across ordinary isolated Compose containers. No host
networking, privileged mode, device/GPU access or default service changes.

Publish only the bridge WebSocket port to `127.0.0.1:8765` on the host. Bind inside
the container as required for port forwarding; distinguish that from host exposure.
RViz uses the repository's existing narrow X11 socket/software-rendering pattern,
only in its explicit UI profile. Interactive desktop validation is separate from
headless CI and must be reported honestly.

Bridge configuration must allow only observational topics/assets. Omit client
publication, service and parameter-write capabilities; deny client topic/service
access as a second configuration check. Whitelist installed visualization model
assets only. Test actual WebSocket attempts to publish a command, change a
parameter or switch controllers. Configuration text alone is insufficient evidence.
This controls the viewer connection, not arbitrary ROS participants on the host.

Add a lazy CLI `act-lab ros2 observability-demo` with seed, bounded duration, paced
mode, output directory and JSON report. Default to a paced safe scripted motion
and an explicit producer-loss/hold/fresh-recovery segment, so viewers have time
to connect and inspect transitions. Reuse MotionSession and existing safety rules;
reconnect/viewer startup must never create an independent command producer.
Provide a separate `observability-smoke --json` for bounded headless CI.

Document tested Compose launch/connect/stop commands and missing-dependency errors.
Write reports, captures and screenshots under ignored `runs/ros2-observability/`.

## 7. Saved views

Commit `configs/ros2/rviz/act-lab.rviz` with world frame, RobotModel, TF, pose markers
and execution/health diagnostics. Distinguish target, approved and measured poses
by label and color; use mode text so fault interpretation does not depend on color.

Commit `configs/ros2/foxglove/act-lab.json` with a 3D view, six effort plots, separate
source/wall-age plots, command report/raw-message inspector, mode/fault fields and
event timeline. Verify import/export against the tested application version.
Document local WebSocket connection; no automatic cloud uploads or recording.
Both views expose retained gripper aperture, epoch changes and loss/recovery.

## 8. Verification and safety review

ROS-free tests: codecs, joint ordering, pose/quaternion preservation, optional raw
output, mode/report distinction, event identity, monotonic observer freshness,
exact 100 ms stale boundary, reset/history clearing and import isolation.

Required optional-container tests (missing packages fail, never skip):

- Generated telemetry/event construction and serialization.
- Actual JointState + TF + description delivery, resolvable meshes and frame/FK
  residual at the three qualified fixtures.
- Correct source/sequence preservation and raw/task/hold/total effort separation.
- Producer loss in both clock orders; paused/backward clock; stale telemetry;
  shutdown; late native response followed by fresh lifecycle recovery.
- Slow subscriber/event overflow, bridge disconnect and observer SIGSTOP/SIGKILL:
  owner continues its existing guard/hold behavior and needs no viewer to recover.
- Foxglove protocol/schema delivery and denied publication/service/parameter writes.
- Saved layout syntax/topic references; bounded discovery waits and injected-clock
  assertions, without shared-runner latency certification.

Run existing dev build/Ruff/mypy/pytest/doctor, default headless simulation, CPU
training/notebook, ROS contract, stationary CRISP and moving ROS checks. Add an
isolated observability CI job uploading reports on failure as well as success.
Keep existing jobs. Check control trace agreement with observation off/on using
fixed intents and injected clocks; separately record differing real wall faults.

Manual acceptance: open the paced demo in RViz2 and Foxglove, inspect motion,
target error, effort and age, then observe hold, stale indication, reset and fresh
recovery. Save representative screenshots and tested client/config versions.
If a display or Foxglove entitlement is unavailable, headless tests may complete
but interactive acceptance remains explicitly pending; do not claim it passed.

Complete a dedicated viewer safety matrix for startup, slow/lost telemetry,
viewer/bridge loss, clock pause/reset, invalid frames, command-write attempts,
recovery and shutdown. Update README, architecture, roadmap and ADR index. Open
one draft PR with validation evidence; merge only after required gates pass.

## Primary references

Reviewed 2026-10-09; rolling references describe features, not a pinned build.
Installed Jazzy source/configuration and recorded versions are authoritative.

- [robot_state_publisher](https://github.com/ros/robot_state_publisher): measured
  JointState input, model transforms and robot description.
- [Foxglove bridge configuration](https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md): explicit topic/assets allowlists and capabilities.
- [Foxglove layouts](https://docs.foxglove.dev/docs/visualization/layouts): shared
  view artifacts and import/export workflow.
- [Foxglove pricing](https://foxglove.dev/pricing) and
  [standalone licensing](https://docs.foxglove.dev/docs/standalone-license): verify
  current application access separately from the ROS bridge.
- Repository ADR 013/015/016, ROS simulation contract and PR 13 qualification.
