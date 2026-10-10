# ROS/CRISP moving simulation contract

## Runtime and ownership

Run the isolated optional image:

```bash
docker compose --profile ros2-simulation build ros2-simulation
docker compose --profile ros2-simulation run --rm ros2-simulation \
  act-lab ros2 simulation-smoke --output runs/ros2-simulation --json
docker compose --profile ros2-simulation run --rm ros2-simulation \
  python -m pytest -o 'addopts=-ra -p no:cacheprovider' tests/ros2_simulation
```

The demonstrator delivers generated v1 commands over Fast DDS to a separate
application gateway. Callbacks offer messages to `CommandInbox`; its consuming
thread calls `SafeCartesianRobot`. A scratch driver checks limited IK/contact
candidates and emits explicitly approved intent. One physics process alone owns
live state, checks authorization, obtains actual CRISP effort via a C++ controller
manager/SystemInterface, and advances 2 ms ticks. Ten ticks form an application
step. Generated state, reports and clock are public DDS streams with PR 11 QoS.
The gateway consumes a coherent internal authority snapshot; independently
received best-effort clock/state messages are telemetry, not competing authorities.

Internal JSON packets are bounded to 256 KiB, with bounded response/discovery
waits. Source timestamps, original receipt time, sequence, episode and recovery
generation accompany approved intent. ROS time is domain time plus one second.
Use simulation time only. Single producer/clock authority; this is not a security
boundary. Diagnostic JSON represents rejected nonfinite values as text, never
as executable authorization.

## Safety review matrix

| Event | Required behavior | Evidence |
|---|---|---|
| Startup / missing input | Position hold, task effort zero | Real 250-tick startup integration test |
| Disabled / invalid frame/numeric/quaternion | Shared application rejection, dynamic hold | DDS cases and ROS-free application tests |
| Stale / future command | Preserve source age, >=100 ms hold | Exact guard boundaries and DDS cases |
| Producer loss | First expired source or wall lease inhibits effort; no refreshed source/receipt | Advance-only DDS case and forced wall-first regression |
| Gateway loss | Owner wall/EOF watchdog, hold continues | Process-loss test; no cleanup assumption |
| Controller loss/stall | Independent owner lease; discard late generation | Controller SIGKILL/restart and lease tests |
| Pause | Steady progress expiration; fresh sequence on resume | Injected clock case / exact boundary tests |
| Backward reset | New UUID required; old intent/output rejected | Reset/reversal cases and generation tests |
| Replay / wrong episode | Clear active intent | Inbox and owner guard tests |
| Near-singular/unreachable IK | Reject infeasible target without changing live state, then dynamic hold | Rank-deficient wrist regression |
| Unsafe effort-driven path | Predict limits/contact; replace candidate with dynamic hold | Effort plant boundary regression |
| Loaded fault hold | Preserve accepted aperture, cube contact, <=2 mm drift | Loaded local and DDS/CRISP grasp/lift regressions |
| Shutdown | Install hold, integrate 0.5 s before releasing owner | Runner physics trace |

Task effort is +/-5 Nm, slew 100 Nm/s. Fault transfer overrides task slew.
Hold actuator ceilings are +/-150/28 Nm, separate from body gravity compensation.
The ROS container also drives the scripted grasp and lift through DDS/CRISP,
then verifies loaded hold and fresh-command resume. The ROS-free fixture prepares
a grasp with the unchanged local driver before testing effort-to-hold transfer. Every motion claim must identify
its fixtures and traces. Hardware stop/hold remains a separate review.

## Evidence

`report.json` records individual cases, requested/measured poses, command outcomes,
process identities, DDS telemetry and configuration/model hashes. `physics.json`
records every integrated tick, ownership, source/receipt/sequence/generation,
raw/task/hold effort and frame residual. `controller.log` retains lifecycle output.
Failures preserve `failure.json` and physics traces. Generated artifacts stay in
ignored `runs/`. The same fixed source intents are executed by the local baseline, with measured
pose, joint, aperture and outcome samples stored alongside the ROS trajectory.
Application reports describe shared safety approval; owner mode/reason describe
actual gated execution and must accompany them. No wall timing is hardware realtime
evidence. ADR 016 explains residual correction,
hold tuning, numerical dependencies and gravity/payload assumptions.

The default runner uses deterministic tick barriers. Wall watchdog events remain
external inputs and can insert hold ticks when a process is delayed. Same-seed
trajectory comparison uses the existing 2 mm pose tolerance and separately
reports bitwise equality and held commands; differing wall events are not
identical clock input traces. `--paced` targets a 2 ms
steady schedule and keeps the owner ticking without gateway events, through the
same guard and dynamic hold path. Budget overruns are recorded per tick; no
hardware realtime qualification is inferred. Nominal motion uses a bounded
15-second recovery window: every attempt creates a fresh source timestamp and
sequence, and every intervening watchdog hold remains in the trace. Numeric,
frame, workspace and other application rejections fail immediately. Explicit
fault scenarios never use this recovery helper.

If reactivation consumes that command's lease, the runner prepares the controller
under dynamic hold, grants no authorization, and submits a separate fresh command.
Prepared readiness is cleared by invalid intent, clock discontinuity or reset.

The DDS gateway waits at most 100 ms for an expected publication, then clears
the inbox and returns a disabled hold with `delivery_timeout` as the separate
transport rejection. Nominal recovery records this loss and creates new intent;
missing ROS dependencies, sustained delivery loss and undelivered explicit fault
fixtures still fail assessment. Both schedulers use the 2 mm
physical comparison bound; bitwise equality additionally requires identical
clock and fault inputs. Precise
watchdog boundaries always use injected clocks, independently of wall scheduling.

The producer-loss assessment advances 250 ticks without publishing new intent.
Simulation source age and wall time can expire in either order on a shared runner.
It requires fault hold, unchanged sequence, source age of at least 100 ms, and a
source/gateway/controller lease rejection; the full trace must contain zero task
effort during hold. The actual first rejection is recorded. Exact individual
lease boundaries remain covered with injected clocks; runtime checks do not
require source expiry to beat an independent wall watchdog.

Ordinary native-controller responses have a bounded two-second drain deadline,
covering the native one-second DDS acknowledgment wait. This transport wait is
separate from motion authority: the owner continues checking its 100 ms wall
watchdog and integrating dynamic hold while waiting. A late response cannot
restore expired authorization. Recovery requires reactivation and a fresh
sequence; a missing response at the drain deadline remains a controller failure.
