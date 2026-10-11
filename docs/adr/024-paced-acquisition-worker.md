# ADR 024: Bounded paced acquisition worker

- Status: implementation under qualification
- Date: 2026-10-11
- Extends ADR 021; supersedes its rejection of paced acquisition for this mode.
- ADR 022 remains authoritative for stepped recording.

## Decision

Opt-in paced CRISP capture uses a separate spawned process for scratch rendering,
generated-message serialization, recorder ACKs and a canonical mirror. Control
offers immutable post-command state, full simulator coordinates and reports.
Rendering never steps physics. The worker's original RGB retains the measured
simulation timestamp; command metadata retains producer sequence and source time.
The existing owner and shared application remain the only motion authorities.

Pending sample capacity is four and the steady capture lease is 100 ms, including
queue, rendering and storage acceptance. Expiration at the exact boundary, queue
overload, wrong episode, nonincreasing source time, completion replay and worker
loss are terminal acquisition faults. They cannot extend enabled intent. The
shared failure path requests the existing gripper-preserving hold; the independent
owner watchdog still covers producer/gateway disappearance. There is no silent
frame replacement, timestamp refresh, lossy preview acquisition or partial success.

Start/warmup occurs under hold before acquisition. Stop first requests disabled
motion, then drains bounded work and finalizes. Reset during an active attempt
remains rejected; finish it and begin a new UUID/worker. Failed or partial attempts
stay excluded by existing import and training quality rules.

An explicit `ros2-recording-gpu` profile grants one NVIDIA GPU only for rendering.
The current EGL vendor is verified; unavailable requested acceleration fails.
Software remains the explicit default. Full acquisition quality remains default.
The explicitly selected `reduced-effects` profile disables shadows/reflections
and multisampling. Its camera calibration fingerprint includes the rendering
settings, rejecting silent mixing with full-quality datasets or checkpoints.
It is a qualification option, not an automatic performance fallback.
Python multiprocessing supplies process isolation; no new package is introduced.

## Safety review matrix

| Fault/stage | Acquisition | Motion |
| --- | --- | --- |
| Startup/absent recorder | Bounded failure before samples | Existing startup hold |
| Slow rendering/storage | At 100 ms interrupt, preserve partial evidence | Shared disabled hold; independent watchdog unchanged |
| Queue overload | Fail without overwriting queued samples | Shared disabled hold |
| Worker/recorder loss | Interrupt; no successful terminal label | Existing gripper-preserving hold/cleanup |
| Producer/gateway loss | Incomplete attempt remains evidence | Independent owner watchdog |
| Source pause/backward reset | No repeated/restamped image; reject active reset | Fresh command/new episode rules unchanged |
| Forward jump | Preserve real source gap | Application freshness and quality gates unchanged |
| Stop/shutdown | Disable then bounded drain; interrupted shutdown excluded | Existing owner hold |
| Recovery | New attempt and worker | Fresh sequence through existing guard recovery |

## Qualification

The plan lists injected-clock, worker and moving tests. Throughput and hardware
claims require measured evidence. Slow hosts may fail the lossless capture budget;
that failure must remain visible. GPU rendering does not prove hardware safety or
real-time controller scheduling.
