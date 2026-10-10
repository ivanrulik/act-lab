# Wrist rendering and transport research

2026-10-10. Research and isolated diagnostic probes for draft PR 22.
This report records the pre-implementation probes. The subsequent
[implementation report](wrist-rendering-implementation.md) covers the delivered
GPU/JPEG changes. Mounting remains a separate review blocker.

## Current practice from primary sources

[MuJoCo visualization](https://mujoco.readthedocs.io/en/stable/programming/visualization.html)
documents native OpenGL offscreen rendering and EGL for headless hardware
acceleration. Reuse one model-specific rendering context. Shadow/reflection
passes add cost and may be disabled when needed. The existing worker already
reuses its context; our cost is predominantly rasterization.

[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html)
requires the `graphics` driver capability for OpenGL/EGL; ordinary CUDA access
is insufficient. Use a selected GPU and `graphics,utility` for an EGL renderer.
`video` is additionally needed for NVIDIA hardware video encoding. A display or
host network is unnecessary for this headless route. Our production image
currently sets `LIBGL_ALWAYS_SOFTWARE=1`; EGL alone therefore did not establish
GPU rendering. Record and check actual OpenGL vendor/renderer, not only nvidia-smi.

[ROS image transport plugins](https://github.com/ros-perception/image_transport_plugins)
provide established compressed transports, including Jazzy support.
[Foxglove Image](https://docs.foxglove.dev/docs/visualization/panels/image)
supports ROS `sensor_msgs/msg/CompressedImage` and compressed video. JPEG gives
independently decodable preview frames; it is a suitable first live projection.
Keep original RGB for acquisition/learning, preserve capture stamp/frame and
paired CameraInfo, and recheck age after encoding.

For larger streams,
[ffmpeg_image_transport](https://github.com/ros-misc-utilities/ffmpeg_image_transport)
provides H.264/H.265 with software and NVENC/VAAPI options. Its FFMPEGPacket type
is not itself a Foxglove video schema. The dedicated
[Foxglove compressed-video transport](https://github.com/ros-misc-utilities/foxglove_compressed_video_transport)
outputs the supported schema; its documented configuration currently requires
GOP 1. Distinguish that plugin's restriction from Foxglove's schema support.
[CompressedVideo requirements](https://docs.foxglove.dev/docs/sdk/schemas/compressed-video)
include Annex B for H.264, complete per-frame packets and keyframe metadata;
B frames are unsupported. Inter-frame streams also need keyframe recovery after
drop/disconnect. The added encoder/decoder and recovery complexity should be
justified by measured bandwidth needs.

## Measurements on the available machine

Host reports RTX 4060 Laptop GPU, 8 GB, driver 610.57.04. GPU access was supplied
only to an ignored Compose diagnostic override with `graphics,utility`, EGL and
software forcing disabled. Actual OpenGL renderer identifies the NVIDIA GPU.
No driver installation or system reconfiguration occurred.

Same final model and 320×240 RGB camera, 30 measured frames after two warmups:

| Backend / effects | Median render/readback | Scene update | Array copy |
|---|---:|---:|---:|
| CPU llvmpipe, original effects | 64.907 ms | 0.052 ms | 0.019 ms |
| CPU llvmpipe, no shadows/reflections | 13.519 ms | 0.048 ms | 0.019 ms |
| NVIDIA EGL, original effects | 0.584 ms | 0.007 ms | 0.006 ms |
| NVIDIA EGL, no shadows/reflections | 0.148 ms | 0.006 ms | 0.006 ms |

The existing quality uses a 4096 shadow map and four offscreen samples.
GPU rasterization with original effects is about 111 times faster in this small
fixture. This is a local microbenchmark, not a general renderer-performance claim.

A 20-second bounded GPU observability smoke completed all four motion/loss/
recovery/reset cases and the DDS/WebSocket restriction probe. It published 457
frames and discarded one over 22.647 seconds including startup/teardown:
20.180 FPS overall. Mean successful render/publish 7.126 ms; maximum 11.495 ms.
The 100 ms expiry and source timestamps were unchanged. This verifies a substantial
publishing improvement, not sustained 25 FPS in the browser or the full loaded
90-second demo. Source scheduling and subscriber/browser latency still need
measurement.

For one actual home-view frame, OpenCV CPU JPEG encoding (30 measured iterations):

| JPEG quality | Original RGB bytes | JPEG bytes | Median encoding | p95 encoding |
|---|---:|---:|---:|---:|
| 80 | 230,400 | 8,845 | 0.112 ms | 0.166 ms |
| 90 | 230,400 | 12,561 | 0.126 ms | 0.155 ms |

Quality 90 is an 18.3-fold payload reduction for this image. Frame content changes
compression ratio; this is not a stream bandwidth or perceptual-quality guarantee.
The encoding timings exclude RGB/BGR conversion and DDS/browser decoding.
At 25 FPS one raw RGB projection is 5.76 MB/s before overhead. The current typed
CameraSample also embeds RGB; subscribing to both raw projections duplicates
pixel traffic. Publish/subscribe only requested projections and benchmark actual
bridge bytes and frame age rather than inferring network saturation from size.

Local evidence, all ignored:

- `runs/profile-wrist-render.py`, `runs/2f85-render-profile.log`.
- `runs/tooling-gpu-render-probe.yaml`, `runs/2f85-gpu-render-profile.log`.
- `runs/2f85-gpu-viewer-probe/` (report, camera, observer and controller evidence).
- `runs/profile-wrist-jpeg.py`, `runs/2f85-jpeg-probe.log`.

## Recommended implementation order

1. Add an explicit optional GPU viewer profile using native EGL. Fail if the
   requested GPU renderer is absent; retain a separately named, optimized CPU
   runtime for CI and machines without a GPU. Physics/control remain on their
   existing independent path. Document the GPU runtime boundary in an ADR.
2. Add quality-90 JPEG `CompressedImage` as the Foxglove preview projection.
   Preserve original timestamps, frame, identity/calibration pairing and expiry;
   raw acquisition and ACT inputs remain unchanged. Pin/document the chosen
   codec dependency and benchmark actual image quality and browser latency.
3. Measure after warmup: sustained delivered FPS, capture-age p95/p99, render,
   encode, DDS/WebSocket and browser time, byte rate, dropped frames and reason.
   Aim for sustained 25 FPS, with no frame authorized past 100 ms. Repeat with
   loaded grasp, command loss, renderer kill, pause/reset and competing workload.
4. Optimize source scheduling if its 25 Hz target still produces fewer fresh
   frames. Keep newest-state queues bounded; no backlog or timestamp refresh.
5. Consider NVENC H.264 only if JPEG bandwidth or higher-resolution/multiple
   streams justify it. Validate no B frames, keyframe recovery and actual
   Foxglove decoding. A batch renderer/physics-engine migration is not required
   by the evidence for this single wrist stream.

GPU and JPEG results support this direction; integration, complete browser
latency testing, mounting inspection and the two existing CI failures remain
pending. No production fix or merge is claimed by this research report.
