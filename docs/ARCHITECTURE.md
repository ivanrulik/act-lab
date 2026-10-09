# Architecture

## Objective

ACT Lab makes the complete imitation-learning lifecycle understandable and
reproducible while allowing simulation components to be replaced by physical
hardware later.

## Dependency rule

Dependencies point inward:

```text
external frameworks and devices
            |
         adapters
            |
      application use cases
            |
    dependency-free domain
```

The domain defines observations, actions, robot state, and ports. Adapters
translate MuJoCo, MediaPipe, MCAP, LeRobot, and optional ROS 2 types at the
boundary. Framework objects must not leak into domain contracts.

## Data flow

```text
keyboard/scripted expert/webcam -> action source -> safety filter
                                               |
                                               v
                                       robot controller
                                               |
                                               v
                                      MuJoCo environment
                                       |      |      |
                                     state  images  events
                                       \      |      /
                                        synchronized
                                             sample
                                               |
                                               v
                                          raw MCAP
                                               |
                                  validate + resample + split
                                               |
                                               v
                                      LeRobotDataset
                                          |       |
                                          v       v
                                      ACT train  replay/QA
                                          |
                                          v
                                  held-out evaluation
```

## Clock and control model

- Physics advances on an explicit fixed timestep.
- Control runs at a configured integer divisor of physics frequency.
- Sensor acquisition timestamps use a monotonic clock.
- Wall-clock time is metadata, never the basis for ordering samples.
- Actions represent Cartesian intent plus gripper intent.
- A safety layer bounds the intent before controller or driver execution.
- Stale or disabled intent commands motion to stop.

The MuJoCo adapter advances 500 Hz physics in explicit groups of ten steps for
a 50 Hz environment interface. `SafeCartesianRobot` owns application safety,
validation order, limiting, hold behavior, and command reports. The MuJoCo
Cartesian driver owns Jacobian-based IK and predicted-contact feasibility. Its
low-level `ActuatorTargets` remains private to the adapter as a deterministic
test seam. Action-source polling receives the current domain observation so
timestamps use the same simulation clock. MuJoCo exposes exact pick-place
geometry through a separate privileged task-state port used only by the
scripted baseline; it is never added to generic observations.

If actuator dynamics would overshoot the measured translation ceiling, the
MuJoCo driver deterministically retries a scaled target from the same pre-step
state. This adapter guard complements application-owned intent and joint limits.

Webcam capture and MediaPipe inference run in an adapter-owned background
thread with a one-sample latest-value boundary. The control loop never waits on
camera I/O. A framework-neutral RGB `CameraFrame` crosses the camera port;
MediaPipe and OpenCV values do not. Calibration, relative velocity mapping,
adaptive filtering,
and gesture clutching produce the same domain `Action` used by keyboard and the
scripted expert.

## Storage model

The application RecordingRobot records the existing safe-control path through
EpisodeSink. The MCAP adapter owns Protobuf encoding, durable finalization and
prefix recovery. Scene images share measured-state timestamps; diagnostic and
lifecycle channels remain separate from policy observations. See ADR 010.

Raw MCAP logs are immutable acquisition evidence. Conversion produces a
derived, reproducible LeRobotDataset. A manifest connects raw episode IDs,
validation decisions, converter revision, resolved configuration, and derived
dataset fingerprint. Domain recorded-episode and quality types remain free of
MCAP and LeRobot values; their adapters decode and serialize at the outer edge.
Selection and train/validation assignment operate on whole episode IDs. See ADR
011.

## Deployment model

Docker Compose is the supported local orchestrator. CPU/headless execution is
the baseline. Webcam, display, and GPU access are opt-in profiles with narrow
host permissions. The host owns source and artifacts; images own dependencies.

## ROS 2 boundary

PR 11 supplies optional generated ROS interfaces and executable contracts in
`ros2/act_lab_interfaces` and `adapters/ros2`. Codecs/inbox require only domain
models; generated messages and rclpy load lazily in the isolated Jazzy image.
Callbacks store complete intent, while the control thread polls the inbox and
executes through `SafeCartesianRobot`. The demonstrator uses a stationary fake
driver and two DDS processes; ROS-driven MuJoCo motion remains later work.

Simulation time maps to ROS time with a one-second offset. The clock/state
authority owns the episode UUID; source timestamp/sequence checks and independent
steady receipt/progress guards prevent replay and enabled intent during pause.
The [ROS contract](contracts/ros2.md) includes QoS and a dedicated fault/safety
matrix. ADR 013 extends ADR 003 without changing domain ports. Default simulation,
conversion, training, and evaluation remain ROS-independent.


## Optional CRISP feasibility bench

PR 12 adds `ros2/act_lab_crisp_bench` and the isolated `crisp-feasibility`
Compose profile. It loads unmodified pinned controller code against stationary
UR5e mock interfaces. Python authorizations reuse the inbox and shared safety
application; an independent C++ test gate records raw and inhibited effort.
It is not a production robot adapter or physical stop/hold implementation.
The original filter=1 no-go is preserved in ADR 014. Explicit filter and frame
requalification now conditionally selects Cartesian impedance for PR 13. The bench
validates stationary local scene/UR tool binding without integrating motion; a
production guard, independent watchdog and stop/hold remain PR 13 requirements.
See [ADR 015](adr/015-crisp-configuration-requalification.md)
and the [bench contract, dependency rationale and safety matrix](contracts/crisp-feasibility.md).
