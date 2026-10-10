# PR 22 review notes — mounting and wrist frame rate

2026-10-10. User feedback: gripper appearance is promising, but flange mounting
looks incorrect (interpreting the user's likely missing “not”). User considers
2.5 FPS a major issue. Treat useful live delivery and mounting inspection as
merge blockers, rather than accepting the documented low rate as sufficient.

## Initial renderer diagnosis (before GPU/JPEG implementation)

A read-only diagnostic benchmark ran in the existing Compose software renderer,
with the same final model and 320×240 wrist camera, 30 measured frames per case
following warmup. It did not change production code or calibration.

- OpenGL renderer: llvmpipe, LLVM 20.1.2, 256 bits: CPU software rendering.
- Existing quality: 4096-pixel shadow map and four offscreen samples.
- Median normal scene update: 0.052 ms; rasterization: 64.907 ms;
  RGB array copy: 0.019 ms.
- With shadows and reflections disabled diagnostically: rasterization 13.519 ms
  on the same hardware; scene and copy costs remain negligible.
- Evidence: ignored `runs/profile-wrist-render.py`, `runs/2f85-render-profile.log`.

This establishes a large avoidable rendering-settings cost. It does not prove
end-to-end 25 FPS or justify changing the 100 ms freshness requirement. The
production source arrives at 25 Hz, so a captured state can already be tens of
milliseconds old before a roughly 65–85 ms render. Expiration then discards many
frames, making delivery much lower than raw rendering throughput. Concurrent
render-heavy jobs worsen the result. Hardware contributes through the chosen
CPU-only backend; no evidence establishes that this laptop cannot meet the
requested rate after optimization. No GPU benchmark ran.

Next qualification: isolate scene/render/serialization/transport timings and
capture-age/drop distributions; choose and document a reproducible low-cost
render profile; measure sustained live delivery and motion together. Preserve
source timestamps, reset isolation and the independent 100 ms watchdog. Validate
recording/image consistency and checkpoint identities for any profile change.
GPU rendering would require an explicit optional runtime and separate validation.

## Flange mounting inspection

The generated URDF attaches `gripper_mount` to `tool0` with identity translation
and rotation. The upstream gripper contributes its existing base mount and
7 mm local body-origin offset. That origin offset alone does not imply a 7 mm
mesh gap. In the compiled MuJoCo model at home, mesh bounds projected into the
mount frame show the wrist shell ending approximately 1.065 mm before the mount
plane and the gripper base-mount mesh starting at the plane. These are bounding
extents, not mating-surface or bolt-pattern verification.

Existing independent FK agreement validates link transforms, not flange contact,
visual mesh placement or physical mounting. A verified UR5e-to-2F-85 adapter,
bolt pattern and mating-face alignment have not been established. Next inspect
orthographic side/front views in both MuJoCo and the assembled official-UR URDF,
with flange/mount axes and measured surfaces. Resolve visual mismatch and model
an explicit adapter if required; regenerate identities and requalify payload,
TCP, contact and camera placement for any geometry or transform change.

## Hosted CI follow-up

At inspection, five jobs passed; two failed in run 38082786524:

- `ros2-observability`: bounded camera DDS capture wait expired.
- `tooling`: ordinary integration/unit collection imported MCAP tests into the
  isolated ROS image, which has no `mcap` dependency.

Investigate and fix both before merge. Do not skip required camera delivery or
add acquisition dependencies to the ROS runtime without a dependency rationale.
PR 22 remains draft. This note records findings and next work; it does not claim
production performance, mounting or CI fixes were implemented.

## Subsequent rendering research

The later [GPU/compression investigation](wrist-rendering-research.md) verifies
NVIDIA EGL rendering at 0.584 ms/frame and a bounded live publishing probe at
20.18 FPS overall. JPEG quality 90 reduced a test frame from 230,400 to 12,561
bytes at 0.126 ms encoding. These supersede the earlier absence of GPU evidence;
production integration and sustained browser-rate qualification remain pending.

## Rendering implementation follow-up

ADR 020 implements the researched GPU/JPEG path. GPU loaded viewing measured
24.92 FPS; the reduced-effects CPU viewer measured 24.81 FPS. Freshness remained
below 100 ms for accepted frames and camera-loss diagnostics remain independent.
See [implementation evidence](wrist-rendering-implementation.md). Flange geometry
review remains open; rendering acceptance does not resolve physical mounting.
