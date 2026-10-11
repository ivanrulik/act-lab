# Continuous ROS recording qualification

Tested working tree based on main `9ed1268`, 2026-10-11. See ADR 024 and the
continuous-recording plan. This is simulated bounded movement, not a successful
pick/place demonstration or hardware real-time qualification.

## Functional evidence

- Required recording container suite: 18 passed, including five new paced cases.
  Tests cover a real separate renderer/acquisition process, generated DDS/MCAP,
  exact canonical equality, slow rendering, worker/recorder loss, retained gripper
  under independent owner hold, partial evidence exclusion and new-episode recovery.
- Existing moving-owner regression suite: 10 passed, including gateway SIGKILL,
  controller stall, paced owner expiry, producer loss and late native replies.
- ROS-free suite: 345 passed, four expected optional skips, one existing warning.
  New injected-clock tests cover exact 100 ms expiry, bounded queue capacity,
  completion replay, delayed reply processing, pause/reset and fresh session recovery.
- Required dev build, Ruff, strict mypy (72 files), doctor and Compose validation
  passed. Existing recording CI automatically collects the added paced test file.

## Full-quality GPU capture

Command: `recording-demo --controller crisp --paced --steps 100`, original
`ur5e_2f85_d405.toml`, explicit `ros2-recording-gpu` profile.

- EGL vendor/renderer: NVIDIA Corporation, GeForce RTX 4060 Laptop GPU.
- 101 original RGB samples across 2.608 seconds of source simulation time.
- Receipt-to-render/storage-ACK age: median 14.57 ms, maximum 29.90 ms.
- Original overview, policy and wrist cameras; unchanged rendering effects.
- Canonical mirror and ROS import matched exactly, including pixels, quality and
  resampled data. Repeat import remained deterministic in the smoke qualification.
- Quality valid; outcome failure (`bounded_ros_demo`), excluded from training.
  The final disabled hold is retained as a rejected/disabled report rather than
  hidden. Long-duration task performance is not established by this movement.

## Failures and limits retained

Full-quality software capture exceeded the 100 ms lease. A 100-command GPU run
while the moving regression suite was active also exceeded it. Both interrupted
acquisition and retained partial evidence; the latter did not produce a successful
report. The standalone GPU run above passed. This shows bounded operation on the
tested workload, not a scheduling guarantee or proof of the overload's root cause.

The CPU qualification fixture is explicitly wrist-only, 64×48, with reduced
effects. Its camera fingerprint includes rendering settings and remains
incompatible with full-quality data/checkpoints. The implementation warms the
selected profile before motion; initial reduced-effects qualification exposed
and fixed a first-frame initialization stall. No deadline was relaxed.

Interactive Foxglove acceptance, long-duration throughput, successful CRISP task
recording, physical sensors, hardware synchronization and commissioning remain
separate work. The existing bag layout supports replay; this branch does not
replace the independent live preview pipeline. Generated bags, traces and timing
artifacts remain under ignored `runs/`.
