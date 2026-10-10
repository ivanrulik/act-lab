# Wrist preview rendering implementation

Local qualification: 2026-10-10, feature/gripper-wrist-camera, draft PR 22.
Simulation evidence only; physical camera, gripper and hardware control were not tested.

## Implementation

- Explicit `tooling-gpu` Compose profile reserves one NVIDIA GPU, uses headless
  EGL and graphics/utility capabilities, and verifies the actual GL vendor.
  Requested GPU failure is explicit; software fallback is prohibited.
- CPU `tooling` remains available without GPU access. Its preview disables
  shadows, reflections and MSAA; acquisition rendering stays unchanged.
- A reused scratch context consumes newest primitive state offered on a 50 Hz
  simulation grid, at a 25 Hz steady rendering cadence. Physics never renders
  or waits for the camera. The latest-state channel has no image backlog.
- Quality-90 JPEG uses standard CompressedImage with paired CameraInfo and
  original capture headers. Raw RGB diagnostics publish when subscribed.
  Both pre/post-encoding freshness checks retain the exact 100 ms boundary.
- Local MCAP/ACT acquisition uses the existing RGB path. Preview settings have
  their own hash and do not redefine acquisition/model calibration.
- Ubuntu Pillow is lazy-loaded only in the optional viewer image. Runtime
  evidence records the codec version, render settings and actual GL identity.

## Measured results

RTX 4060 Laptop GPU, NVIDIA driver 610.57.04; Jazzy/Noble, Fast DDS. CPU
renderer: llvmpipe LLVM 20.1.2, Mesa 25.2.8. Resolution 320 x 240, JPEG quality 90.

| Local run | Active publication FPS | Capture age p95 / max | Mean JPEG size | Frames / discards |
|---|---:|---:|---:|---:|
| GPU four-phase smoke, 20 s | 24.92 | 30.0 / 46.7 ms | 12.6 KB | 509 / 1 |
| GPU loaded grasp + fault phases | 24.92 | 40.4 / 50.4 ms | 12.8 KB | 1858 / 2 |
| Reduced-effects CPU four-phase smoke, 20 s | 24.81 | 55.6 / 71.6 ms | 13.1 KB | 500 / 6 |

Active FPS excludes startup/shutdown; these are publisher measurements, not a
browser playback benchmark. Raw RGB is 230,400 bytes per frame. The loaded
run averaged 7.42 ms rendering and 0.52 ms encoding. Timing is machine/load
dependent; CPU and GPU preview quality differ. Earlier software-only 2.5 FPS
was dominated by rendering configuration/scheduling, not JPEG encoding or a
proven hardware throughput limit. Direct rendering probes are in the
[research report](wrist-rendering-research.md).

The GPU loaded run lifted the cube 78.25 mm, preserved the accepted closed
gripper through a two-second command-loss hold (0.143 mm drift), and completed
loss, recovery and reset phases. The separate updated motion smoke completed
all 39 cases. These fixtures do not establish general tracking or hardware safety.

Foxglove live acceptance displayed the compressed wrist image, calibrated
original timestamps, articulated robot and live telemetry in the user layout.
The layout's camera topic was switched to JPEG without saving its existing
cloud changes. Screenshot: ignored `runs/render-foxglove-compressed.png`.

## Required checks and CI repair

- `docker compose --profile tooling-gpu build tooling-gpu`: passed.
- `docker compose build dev`: passed.
- Dev Ruff, mypy (68 source files), doctor: passed.
- Dev pytest: 306 passed, four existing optional LeRobot/display skips.
- Required CPU ROS observability tests: six passed, including generated JPEG
  serialization, decoding, header pairing and independent renderer-loss health.
- Camera/rendering unit subset: 14 passed.
- Full articulated controller motion smoke: 39 completed cases.

Prior hosted CI failed on slow software DDS delivery and ordinary MCAP modules
being collected inside the isolated ROS image. The CPU preview optimization
addresses the former; tooling CI now collects its ROS-specific test modules,
while the existing dev/training jobs own ordinary acquisition/learning tests.
Required ROS dependencies/assertions still fail rather than skip. Hosted CI
for the new commit must pass before merge.

Ignored artifacts: `runs/render-gpu-smoke/`, `runs/render-gpu-loaded/`,
`runs/render-cpu-smoke/`, `runs/render-motion-qualification/`,
`runs/render-cpu-dds-tests.log`, and `runs/render-{ruff,mypy-final,pytest,doctor}.log`.

## Remaining review

The user's flange mounting concern is still open; this rendering change does
not verify an adapter mating face or physical bolt pattern. See the
[review notes](gripper-wrist-camera-review.md). The user deferred Foxglove assembly appearance to a later follow-up; it is
not a merge gate for PR 22. This does not establish physical mounting validity. NVENC/H.264 remains deferred: JPEG already lowers
preview bandwidth roughly eighteen-fold at this resolution, without a video
encoder/session dependency or inter-frame recovery semantics.
