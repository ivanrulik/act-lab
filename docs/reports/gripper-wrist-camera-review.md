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

## Static flange mesh comparison — 2026-10-10

Compared mount-local orthographic side/front/flange views from compiled MuJoCo
visual triangles with the official pinned UR wrist DAE (including its embedded
node transform) and the generated tool URDF. Original tool STL vertices were
independently transformed through Pinocchio and URDF visual origins. Maximum
tool visual bounding-extents discrepancy was 9.17e-8 m (under 0.1 micrometre).
No tool mesh translation, scaling or rotation export error was found at home.
The tool stays centered on the flange in both static mesh reconstructions.

The Menagerie wrist visual ends 1.065 mm before the mount plane; the official
UR wrist mesh reaches 0.035 mm beyond it. This is a small visual-seam difference,
not evidence of a large displaced or flipped flange. The nominal arm tool0
registration differs by 0.719 mm at home after the established Rz(pi) world
mapping, consistent with the distinct nominal arm descriptions; the shared
tool geometry was compared in mount coordinates rather than hiding this offset.

A fresh Foxglove run was inspected with and without transform overlays. Dense
axes and dark tool materials obscure the connection; their contribution to the
appearance is an inference, not a verified Foxglove loader defect. Its current
view still deserves clearer lighting/close-up acceptance. No geometry was
changed to conceal the visual seam, and physical mating faces/adapter/bolt
patterns remain unverified.

Ignored evidence: `runs/check-flange.py`, `runs/flange-comparison/bounds.json`,
MuJoCo and official-URDF side/front/flange PNGs, and `foxglove-live.png`.
The official-URDF diagnostic PNGs use reconstructed mesh geometry rendered by
MuJoCo; they are not screenshots of Foxglove's own mesh loader.

## Live assembly follow-up

User reports that Foxglove still appears incorrectly assembled. Static mesh
reconstruction does not resolve this report. An isolated live DDS probe matched
eight JointState/TF timestamps, each carrying fourteen measured joints. All
thirty prefixed URDF link transforms agreed with independent Pinocchio FK to
1e-8 m/rad; all eight installed tool STL assets matched source bytes exactly.
Evidence: ignored `runs/check-viewer-transforms.py`,
`runs/flange-dds-verification.json`, and `runs/flange-dds-verification.log`.

Browser URL policy blocked the attempted live-panel inspection. Requested a
close-up screenshot; client control mode, stored frame/joint overrides, mesh
loading and reset-time behavior remain to be investigated. Do not describe
Foxglove assembly as accepted or change simulator geometry to hide this issue.

## User-directed deferral and CI fix

The user explicitly deferred the Foxglove assembly appearance issue to a later
follow-up. Preserve it as an open viewer acceptance item; do not claim that the
appearance or physical mounting was validated. It is no longer a PR 22 merge
gate. The separate hosted tooling failure imports MCAP while recording is
disabled. Load the storage sink only after recording is requested, keeping the
shared safety wrapper and optional ROS image dependency boundary intact.

CI fix validation: the exact isolated-tooling expert command completed 20/20
episodes with no MCAP package installed. Dev build, Ruff, mypy (68 files),
doctor and pytest (307 passed; four existing optional skips) passed. Recording
integration tests remain in the full suite, and a subprocess regression blocks
MCAP/Protobuf imports while running a non-recording expert session. Evidence:
ignored `runs/tooling-ci-expert-fixed.json` and `runs/mcap-lazy-*.log`.
