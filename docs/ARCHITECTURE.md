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
translate MuJoCo, MediaPipe, MCAP, LeRobot, and eventually ROS 2 types at the
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
MediaPipe and OpenCV values do not. Calibration, relative mapping, filtering,
and gesture clutching produce the same domain `Action` used by keyboard and the
scripted expert.

## Storage model

Raw MCAP logs are immutable acquisition evidence. Conversion produces a
derived, reproducible LeRobotDataset. A manifest connects raw episode IDs,
validation decisions, converter revision, resolved configuration, and derived
dataset fingerprint.

## Deployment model

Docker Compose is the supported local orchestrator. CPU/headless execution is
the baseline. Webcam, display, and GPU access are opt-in profiles with narrow
host permissions. The host owns source and artifacts; images own dependencies.

## ROS 2 boundary

ROS 2 is planned but not required by the core. It becomes valuable for
distributed processes, standard visualization, and physical UR integration.
ROS messages will be translated by adapters into the same domain contracts used
by local MuJoCo. Training and dataset inspection remain ROS-independent.
