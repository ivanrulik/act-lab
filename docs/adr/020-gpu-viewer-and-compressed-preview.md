# ADR 020: Explicit GPU viewer and JPEG preview

- Status: Accepted for optional simulated viewing; integration evidence below.
- Partially supersedes: ADR 019's software-only viewer runtime.
- Scope: simulated wrist preview, separate from acquisition and control.

## Decision

Add `tooling-gpu` as an explicit opt-in Compose profile. Reserve one NVIDIA GPU,
expose only graphics/utility driver capabilities, use EGL without a display,
and verify the active OpenGL vendor/renderer before publishing. An unavailable
requested GPU fails; never substitute software. Keep `tooling` and CI explicitly
software-only, with a lower-cost viewer profile (no shadows/reflections or MSAA).
GPU preview retains original visual effects. Both reuse one scratch model and
context and never integrate physics or authorize robot commands.

The owner offers fresh primitive state on its 50 Hz acquisition grid through
the existing bounded independent channel. The worker targets a 25 Hz steady
cadence using newest state; no backlog, repeated source restamping or physics
clock modification is allowed. A paused clock cannot renew image health.

Publish quality-90 JPEG on `/act_lab/view/wrist/image_raw/compressed`, using
standard CompressedImage and paired CameraInfo. Preserve the original capture
header and reject at 100 ms both before and after encoding. Continue the raw
Image and CameraSample diagnostic projections when subscribed; acquire original
RGB for MCAP/ACT with the existing local adapter. Snapshot subscriber counts
before constructing a raw sample so discovery changes cannot publish empty RGB.

Pillow is supplied by Ubuntu's `python3-pil` package in the optional viewer image.
It encodes RGB without OpenCV's additional runtime and BGR conversion. Load it
lazily; domain and ordinary application imports remain ROS/codec independent.
Record its package version with runtime dependencies. JPEG is independently
decodable; NVENC/inter-frame video is deferred pending bandwidth evidence.

Record actual OpenGL identity, viewer settings and settings hash, encode/render
cost, capture age, bytes, delivery rate and discard count in camera evidence.
Changing viewer settings does not change acquisition calibration/model identities;
its separate render-profile hash identifies the preview's visual approximation.
This profile is not a source for policy training or camera fidelity claims.

## Safety review

The ADR 019 matrix still applies. Requested-GPU startup failure stops the bounded
demo; callbacks cannot enable motion. Encoding failure stops the worker and the
independent observer expires camera health. Slow encodes, lost/reset source and
paused clocks retain the 100 ms bound. Original gripper-preserving motion loss
holds remain independent of preview scheduling, JPEG and GPU resources.

## Validation and consequences

Required DDS tests decode actual JPEG, verify RGB ordering, dimensions, quality,
serialized round trip and original header pairing with raw capture; an independent
observer reports renderer loss. Injected clocks cover pause/reset/expiry. A unit
check rejects a software OpenGL context even when GPU was requested.

CPU CI runs required camera construction/delivery without a GPU. ROS-free assembly,
MCAP/model compatibility and learning tests run in their designated dev/training
images; ordinary pytest modules are not imported into the isolated ROS image.

See the [research](../reports/wrist-rendering-research.md) and
[implementation evidence](../reports/wrist-rendering-implementation.md) for measured
limits. Sustained FPS, capture age and full loaded motion must be checked together;
short rendering benchmarks do not establish hardware control real-time behavior.
