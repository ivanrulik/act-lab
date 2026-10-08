# ROS 2 executable contracts (v1)

Roadmap PR 11 supplies generated interfaces and an executable DDS demonstration.
It establishes the boundary for later motion adapters. The domain, local MuJoCo,
MCAP conversion, training, and evaluation remain ROS-independent. ADR 013 records
this decision. No hardware or MuJoCo motion through ROS has been validated.

## Execution and dependencies

```bash
docker compose --profile ros2-contracts build ros2-contracts
docker compose --profile ros2-contracts run --rm ros2-contracts
# The ordinary suite excludes this dedicated, required ROS suite:
docker compose --profile ros2-contracts run --rm ros2-contracts \
  python -m pytest -o 'addopts=-ra -p no:cacheprovider' tests/ros2
```

The image builds `ros2/act_lab_interfaces` using colcon/ament and rosidl. It runs
Jazzy on Ubuntu 24.04/Python 3.12, pinned to the inspected official base digest
`ros:jazzy-ros-base-noble@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca`.
ROS distribution packages own rclpy, generated type support, numerical libraries,
and Fast DDS. A system-site-packages venv adds pytest 8.4.2 only. Repository code
is exposed through `PYTHONPATH`; installing the ACT dependencies here would mix
NumPy/ROS ABI environments and is deliberately avoided. Build tools generate the
interfaces; std/geometry messages carry standard timestamps and poses;
rosgraph messages carry `/clock`; `rmw_fastrtps_cpp` selects Fast DDS explicitly.
These packages are isolated from the default Dockerfile and Python requirements.
The image records its installed Debian package versions for provenance.

Discovery uses `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, domain ID 42, with both
processes inside one container. No host networking, ports, display, devices,
GPU, or privileged access is configured. The `ros2-contracts` profile is opt-in.
The lazy CLI is `act-lab ros2 contract-smoke --json` in an installed core
workflow and the source-only ROS image (a small wrapper invokes the same module). Missing ROS/generated packages produce the Compose instruction;
the ROS CI job fails instead of skipping missing packages.

## Wire interface

| Topic | Generated message | QoS |
|---|---|---|
| `/act_lab/v1/command` | `act_lab_interfaces/CartesianCommand` | Reliable, volatile, keep-last 1; deadline/lifespan 100 ms |
| `/act_lab/v1/state` | `act_lab_interfaces/RobotState` | Best effort, volatile, keep-last 1 |
| `/act_lab/v1/command_report` | `act_lab_interfaces/CommandReport` | Reliable, volatile, keep-last 10 |
| `/clock` | `rosgraph_msgs/Clock` | Best effort, volatile, keep-last 1 |

DDS deadline events and lifespan expiration supplement the application guards;
they do not authorize execution or replace `SafeCartesianRobot` freshness checks.
Message schemas live in `ros2/act_lab_interfaces/msg`. Incompatible changes
require a new topic version.

Commands bundle header, canonical episode UUID, strictly increasing uint64
sequence, Cartesian pose, normalized gripper intent, and enabled flag. State
contains a header, episode UUID, six UR5e joint names/positions/velocities,
end-effector pose, and gripper position. Canonical joint order is:

1. `shoulder_pan_joint`
2. `shoulder_lift_joint`
3. `elbow_joint`
4. `wrist_1_joint`
5. `wrist_2_joint`
6. `wrist_3_joint`

Decoders reorder a complete named state into this order and reject missing,
unknown, or repeated names. Reports preserve outcome, requested intent, optional
executed intent, resulting state, detail, commanded/measured Cartesian speed,
tracking error, and command age. `has_executed_command=false` makes the generated
placeholder executed message irrelevant. Nested metadata must agree with the
outer report. Rejected transport packets are never represented as accepted
commands: the report audits the resulting disabled hold and transport rejection
is exposed separately through `CommandInbox.last_rejection_reason` and smoke
case evidence. Report sequence is correlation metadata, not an executable intent.

Wire quaternions are XYZW; domain quaternions are WXYZ. Conversion reorders only,
with no normalization or coordinate transform. `world` is the supported command
frame. Other frames and nonfinite numeric payloads survive the codec and reach
application validation, producing INVALID and a shared safe hold. Structurally
malformed transport encodings are rejected before application execution.

## Clocks, ownership, and inbox

Require `use_sim_time=true`. System-clock synchronization is unsupported and
`CommandInbox(use_sim_time=False)` raises explicitly. The fixed mapping is:

```text
ROS timestamp_ns = domain simulation timestamp_ns + 1_000_000_000
```

Domain zero becomes ROS one second, avoiding ROS's uninitialized time zero.
The trusted simulation/state authority generates a new UUID on startup/reset,
publishes `/clock` and state, and calls `update_clock(ros_ns, episode, steady_ns)`.
`/clock` itself does not carry an episode UUID; a future external adapter must
bind it to the trusted state authority, never infer a reset from command traffic.
Only one producer and one clock authority are supported per episode. Arbitration
and authentication are deferred; an episode UUID is replay protection, not an
access control credential.

`CommandEnvelope` is a typed, ROS-independent domain intent wrapper. Codecs use
primitive dictionaries and runtime helpers lazily construct generated objects.
Callbacks decode and call `offer(envelope, receipt_steady_ns)` or `reject(reason)`.
The inbox lock protects cached intent. Callbacks never access the robot. Only the
consuming control thread calls `poll(observation, steady_now_ns)` and then
`SafeCartesianRobot.command(action)`. Numeric, workspace, Cartesian/joint rate,
IK, collision, and gripper checks remain in that shared application path.

- The command must match the authority's episode and advance the observed
  sequence. Invalid, duplicate, out-of-order, and wrong-episode packets clear
  cached intent. Observed valid sequence numbers are consumed even for a
  pre-episode timestamp, so retries cannot resurrect rejected intent.
- Polling preserves the original source timestamp; receipt or reuse never
  refreshes enabled intent. At domain age **100 ms or more**, Safety reports
  STALE. A future timestamp reaches Safety once for INVALID, then is discarded.
- Missing commands and transport faults return a disabled measured-pose action.
  Safety executes the hold, preserving its last accepted gripper aperture.
- Steady time is injected and must be monotonic. Receipt age and time since
  clock advancement each expire at **100 ms**, independent of ROS time.
  Repeated identical clock packets do not count as progress. Live command
  packets cannot bypass a paused clock. Observation and authoritative clock
  must agree exactly before intent can be enabled.
- A detected pause clears cached intent, including when the next clock update
  resumes after a pause that was not polled. Resume requires a newly offered
  command with a higher sequence. The observation clock must progress too.
- Backward ROS jumps latch a fault until a new UUID arrives. Commands cannot
  change that UUID. Forward jumps age existing intent normally. A new episode
  clears intent and sequence history; old-epoch packets cannot replay.
- Shutdown clears the inbox and executes a disabled hold before relinquishing
  driver ownership. A future moving adapter must implement that ordering and
  its driver-level stop semantics; abrupt process death cannot execute Python
  cleanup and requires an independent driver watchdog.

## Dedicated ROS safety review matrix

This is the contract-layer review and its executable evidence. It does not
approve physical motion; hardware commissioning requires roadmap PR 15.

| Event | Required response | Evidence |
|---|---|---|
| Startup/no clock/no command | Disabled measured hold; no enabled motion | Unit initialization/zero-time tests; DDS `startup_missing` |
| Producer loss | Steady receipt timeout at 100 ms; keep accepted aperture | Unit exact receipt boundary; DDS `publisher_loss` terminates producer |
| Clock loss/pause with live packets | Independent steady progress timeout at 100 ms | Unit boundary/live-packet tests; DDS `paused_clock` |
| Resume | Old intent stays cleared; higher fresh sequence required | Unit unpolled pause test; DDS `resume_without_fresh`, `resume_fresh` |
| Backward/reset | Clear intent; same UUID cannot recover; new UUID required | Unit reset tests; DDS `backward_jump`, `same_epoch_after_reset`, `new_epoch_recovery` |
| Forward jump/stale | Preserve stamp; shared STALE hold at age >=100 ms | Unit age boundary; DDS `forward_jump`, `exact_stale_boundary` |
| Replay/wrong episode | Clear active intent, disabled hold | Unit sequence/UUID tests; DDS `duplicate`, `out_of_order`, `old_epoch_replay` |
| Malformed/unsupported/nonfinite | Transport hold or shared INVALID as appropriate | Unit malformed wire tests; DDS `malformed_envelope`, `unsupported_frame`, `invalid_numeric` |
| Future command | Shared INVALID once, then clear cached intent | Unit future recovery; DDS `future` |
| Shutdown | Clear intent and shared disabled hold before cleanup | DDS `shutdown`; runner finally closes producer and ROS nodes |

The demonstrator runs separate spawned producer and consumer processes over DDS,
with generated command/state/report messages and an explicit clock publisher.
Its deterministic stationary fake driver exercises shared safety, not dynamics.
Assertions inject steady time for exact watchdog boundaries; discovery/delivery
use bounded 15-second waits, not wall-time latency acceptance assertions. JSON
records decisions, original timestamps, held aperture, configured QoS, process
IDs, ROS/Python/RMW identity, and the Debian manifest hash. Serialization tests
cover all generated messages. Ordinary tests verify imports do not load ROS or
learning/simulation frameworks. CI uploads the smoke JSON as validation evidence.

## Deferred work and references

PR 12: controller feasibility and decision. PR 13: ROS/CRISP MuJoCo motion,
external clock/state ownership wiring and motion-equivalence tests. PR 14: ROS
MCAP equivalence. PR 15–16: hardware safety and supervised integration.
System-clock synchronization and multi-producer arbitration need separate
contracts and roadmap decisions before enabling them.

- [ROS QoS documentation source](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)
- [ROS clock design](https://design.ros2.org/articles/clock_and_time.html)
- [REP 2000 platform support](https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst)
