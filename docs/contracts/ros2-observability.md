# ROS 2 observability contract

The optional observer projects the moving CRISP/MuJoCo authority into
Foxglove. ADR 017 extends ADR 016. These views are diagnostic and do not authorize
motion or replace PR 14's future durable ROS recording.

## Run and connect

```bash
docker compose --profile ros2-observability build ros2-observability
docker compose --profile ros2-observability up ros2-observability
# Stop after inspection:
docker compose --profile ros2-observability down
```

The bounded demo defaults to 120 seconds. Its four phases exercise small motion,
producer loss, fresh-command recovery and a new episode after backward reset.
Duration is the nominal sum of four phase dwell times; bounded controller
preparation/recovery can extend wall runtime and cannot skip a fault phase.
Reports include actual elapsed time. For headless evidence:

```bash
docker compose --profile ros2-observability run --rm ros2-observability \
  act-lab ros2 observability-smoke --output runs/ros2-observability --json
```

Foxglove: use an existing account/client, open a Foxglove WebSocket connection to
`ws://localhost:8765`, then import `configs/ros2/foxglove/act-lab.json`. The
application's account/license is separate from the open source ROS bridge. The
repository does not provision accounts, accept agreements, or initiate recording
or cloud uploads. UI import validation is tracked in the implementation report.

DDS uses Fast DDS and localhost discovery. Only the WebSocket port is forwarded
to host **127.0.0.1**;
its internal bind to 0.0.0.0 is necessary for forwarding. No host networking,
devices, display, GPU or privileged access is requested.

## Topics and schema

| Topic | Type | QoS |
|---|---|---|
| `/joint_states` | sensor_msgs/JointState | best effort, volatile, depth 1 |
| `/tf` | tf2_msgs/TFMessage | observer reliable, volatile, depth 100; model publisher's standard TF profile |
| `/tf_static` | tf2_msgs/TFMessage | reliable, transient local, depth 1 from observer |
| `/act_lab/view/robot_description` | std_msgs/String | model publisher reliable, transient local |
| `/act_lab/view/poses` | visualization_msgs/MarkerArray | best effort, volatile, depth 1 |
| `/act_lab/view/telemetry` | act_lab_interfaces/ExecutionTelemetry | best effort, volatile, depth 1 |
| `/act_lab/view/events` | act_lab_interfaces/ExecutionEvent | reliable, volatile, depth 64 |
| `/act_lab/view/health` | diagnostic_msgs/DiagnosticArray | best effort, volatile, depth 1; 20 Hz steady heartbeat |

Telemetry/events have schema_version 1. Existing v1 command, state and report
messages are unchanged. Domain time maps to ROS time with +1 second; observer,
model publisher and bridge use simulation time. Heartbeat freshness uses an
independent monotonic clock even when ROS time pauses.

Canonical joint order is shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2,
wrist_3 (each with `_joint`). Efforts are Nm: raw CRISP response, gated task,
dynamic hold servo and measured total actuator force. Total excludes body
compensation and contact forces. `JointState.effort` contains measured total.
`raw_effort_valid=false` means zero placeholders are unavailable; diagnostic
text preserves nonfinite/unavailable status. Raw response has separate originating
episode, generation and tick, so delayed native output cannot be mistaken for
current authorized effort. It never renews a lease.

`source_valid` marks diagnostic command metadata; it can remain true after a
fault while `authorization_valid` is false. The source episode, sequence and
original timestamp are retained. Requested/approved markers come from copies
of the actual application safety path; approved is the application-executed
Cartesian intent, before the owner's scratch IK/rate-limit target. An application
report may approve intent that an independent owner watchdog subsequently inhibits.

Source age uses domain time. Injected receipt/clock/output ages use the guard's
steady timeline. Wall receipt/clock/output ages use real monotonic time and are
separate from observer receipt age. Source/receipt values are absent when
source_valid is false; injected receipt is absent without authorization. Output
age starts at owner startup until the first valid output. Negative source age
exposes future intent rather than making it fresh.

## Independence, freshness and reset

One owner offers primitive snapshots every ten 2 ms ticks and on transitions.
A 64 KiB shared latest-value slot uses nonblocking lock acquisition. Busy/dead
readers and oversized samples increment handoff_drops. No queue feeder, ROS
serialization or reliable publication runs in the owner. Capture exceptions
also count as dropped samples and cannot alter actuation. Up to 64 transition
events accompany snapshots; overflow and observer sequence gaps are explicit.
Reliable DDS does not make this diagnostic handoff lossless.

Observer duplicate samples never renew freshness. At exactly 100 ms without a
new authority sample, health is STALE and pose markers DELETEALL active intent;
a steady heartbeat preserves the warning during frozen ROS time. If the observer
or bridge dies, cached robot meshes/TF may remain displayed. Connection state and
health must be checked; a rendered robot is not proof of live state.

Each episode reset clears copied intent and event history, deletes pose markers
and republishes the static mapping. Model TF and scene TF retain separate child
names. Viewers must process the backward clock discontinuity; cached plots/TF
history can remain client-specific until reset/reconnection. That behavior must
be checked during interactive acceptance.

The official pinned UR model is namespaced `ur_model/`. `world` to
`ur_model/world` is Rz(pi), following ADR 016. Actual domain pose is published as
`world` to `act_lab_scene_tip`. UR `tool0` and the scene tip remain separate:
ADR 016 includes a 0.11 m tool offset and measured residual correction. Never
interpret the UR visual model as exact MuJoCo geometry. Owner residual fields
retain the qualified 2 mm / 0.02 rad comparison bounds. Gripper aperture is a
label, not an invented URDF joint.

## Bridge restrictions and dependencies

Pinned Ubuntu package: ros-jazzy-foxglove-bridge
`3.6.0-1noble.20260930.003057` (bridge 3.6.0, SDK 0.28.0).
The installed package.xml and Debian copyright metadata declare MIT.
The protocol client uses `foxglove.sdk.v1`, verified against the installed build.
Capabilities are only connectionGraph and assets. Topic allowlists include view,
state/report, joint_states, TF and clock, excluding commands. Client publication,
services and parameters have both disabled capabilities and empty access lists.
Assets are limited to installed UR5e visual/collision meshes. Actual protocol
tests attempt command advertisement, parameter reads/writes, a binary service
call, permitted mesh retrieval and denied `file:///etc/passwd` retrieval.
These restrictions cover the viewer connection, not arbitrary ROS participants.
The demo supervisor stops its producer and performs existing shutdown if viewer
infrastructure exits; the independent owner itself never waits for viewers.

Dependencies: robot_state_publisher computes model FK from measured joints;
tf2_ros supplies frame inspection; visualization_msgs/diagnostic_msgs provide
standard projections; Foxglove bridge supplies local WebSocket schemas/assets;
python3-websocket is the bounded protocol test client.
No viewer packages enter default simulation or learning images. Upstream apt
packages retain their copyright/license files under `/usr/share/doc` and ROS
package shares. Generated reports record resolved package versions, model and
configuration hashes, ROS/RMW and machine information under ignored `runs/`.

## Dedicated viewer safety review

| Case | Expected behavior / evidence |
|---|---|
| Startup without authority | observer health STALE; no intent markers or motion authorization |
| Lost/slow telemetry | exact 100 ms observer stale; overwrite/drop counters visible |
| Dead reader holding lock | owner offers fail immediately; existing watchdog and dynamic hold continue |
| Viewer/bridge disconnect | owner does not depend on connection or acknowledgments; cached rendering can persist |
| Producer loss | existing independent 100 ms leases inhibit task effort and retain accepted gripper |
| Late native reply | raw identity recorded separately; expired intent cannot enable motion |
| Clock pause | wall clock-age grows; motion guard requires fresh authorization |
| Backward reset | new UUID, intent/event clearing, DELETEALL, static mapping republished |
| Invalid frame/numeric intent | existing application rejection; unsupported markers omitted |
| Viewer write attempts | actual publication, parameter and service requests rejected |
| Fresh recovery | existing reactivation and new sequence; viewer reconnect grants nothing |
| Shutdown | dynamic hold integrates before termination; viewer children terminate together |

Required optional tests fail on missing generated packages. Headless evidence and
interactive viewer acceptance are reported separately. Gazebo, hardware, ROS
MCAP recording/replay and viewer-driven control remain deferred.

The evidence supervisor retries nonblocking snapshot reads for at most two
seconds when producing its final report. A busy slot is not evidence of absent
telemetry. This reporting wait retains the original snapshot timestamps and
metadata; owner offers and observer freshness remain unchanged. Persistent
missing data fails execution rather than silently producing incomplete evidence.
