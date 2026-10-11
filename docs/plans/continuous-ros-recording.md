# Continuous ROS recording

Branch: `feature/continuous-ros-recording`, based on merged main `9ed1268`.

## Delivery

1. Keep the stepped recorder unchanged. Add explicit `--paced --controller crisp`
   acquisition through the existing independent physics owner and shared safety.
2. Move original RGB rendering, ROS serialization, ACK waits and canonical mirror
   writes into a separate worker. Offer immutable full simulator snapshots and
   complete command reports without waiting for rendering or storage.
3. Bound pending captures to four; expire pending work at 100 ms on an injected
   steady clock. Never overwrite snapshots, restamp images or renew intent to
   hide slow acquisition. An overloaded/lost worker interrupts the attempt and
   propagates through the existing gripper-preserving hold/cleanup path.
4. Verify renderer identity and provide an explicit optional NVIDIA profile.
   Acquisition uses the original model rendering settings and uncompressed RGB.
   Performance failure must be reported; no GPU-to-CPU fallback is permitted.
   A separately selected reduced-effects profile carries a distinct camera
   identity; CPU CI uses a small wrist-only fixture with that explicit profile.
5. Retain exact source identity, lifecycle, canonical import and quality gates.
   Close acquisition before resets; recovery starts a new recorded attempt.

## Testing stages

- ROS-free: exact watchdog boundary, queue overload, completion order, replay,
  source pause/reset, steady reset, recovery and import isolation.
- Worker integration: immutable snapshot/image timestamp and camera calibration
  agreement; separate PID; no data loss; recorder disappearance and worker loss.
- Moving runtime: paced physics continues while acquisition is busy; verify
  enabled motion, actual holds, retained gripper and existing independent faults.
- Evidence: source/receipt/render/ACK age, sample gaps, throughput, renderer and
  runtime provenance. Record failures without claiming hardware real-time safety.
- Acceptance: dev build, Ruff, mypy, pytest, doctor, required recording and moving
  suites; add paced cases to the existing recording CI job. Keep full stepped
  equivalence and ROS-free conversion checks.

## Deferred

Physical sensors, hardware commissioning, clock synchronization, crash repair,
multiple producers, depth/stereo and the Foxglove assembly appearance review.
Further quality changes require a separate identified acquisition profile and review.
