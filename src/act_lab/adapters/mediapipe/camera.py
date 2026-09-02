"""OpenCV implementation of the dependency-free camera port."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

from act_lab.domain import CameraFrame


class OpenCVCamera:
    """Capture mirrored RGB frames from a V4L2 device or a video file."""

    def __init__(
        self,
        source: str | Path,
        *,
        width: int,
        height: int,
        fps: int,
        recorded: bool = False,
    ) -> None:
        import cv2

        self._cv2 = cv2
        self._recorded = recorded
        self._fps = fps
        self._index = 0
        self._next_recorded_frame_ns = time.monotonic_ns()
        # Select V4L2 explicitly for live Linux devices. Letting OpenCV choose a
        # backend can route `/dev/video*` through FFmpeg, which is less reliable
        # for long-running UVC capture and can abort on VIDIOC_DQBUF failures.
        self._capture: Any = (
            cv2.VideoCapture(str(source), cv2.CAP_V4L2)
            if not recorded
            else cv2.VideoCapture(str(source))
        )
        if not self._capture.isOpened():
            raise OSError(f"unable to open camera or video source: {source}")
        if not recorded:
            # Prefer uncompressed YUYV so live capture does not depend on
            # OpenCV's bundled libavcodec MJPEG decoder. Set it before
            # dimensions/FPS so V4L2 negotiates one coherent mode.
            self._capture.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*"YUYV"),  # type: ignore[attr-defined]
            )
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self._capture.set(cv2.CAP_PROP_FPS, fps)
        else:
            source_fps = float(self._capture.get(cv2.CAP_PROP_FPS))
            if source_fps > 0.0:
                self._fps = max(1, round(source_fps))
        self._closed = False

    def capture(self) -> CameraFrame | None:
        if self._closed:
            raise RuntimeError("camera is closed")
        if self._recorded and self._index > 0:
            self._next_recorded_frame_ns += 1_000_000_000 // self._fps
            remaining_ns = self._next_recorded_frame_ns - time.monotonic_ns()
            if remaining_ns > 0:
                time.sleep(remaining_ns / 1_000_000_000)
        success, bgr = self._capture.read()
        if not success or bgr is None:
            return None
        # Mirror first so diagnostics and control share the user's screen frame.
        rgb = self._cv2.cvtColor(self._cv2.flip(bgr, 1), self._cv2.COLOR_BGR2RGB)
        height, width, channels = rgb.shape
        if channels != 3:
            raise RuntimeError("camera frame must contain three color channels")
        timestamp_ns = (
            self._index * (1_000_000_000 // self._fps)
            if self._recorded
            else time.monotonic_ns()
        )
        self._index += 1
        return CameraFrame(timestamp_ns, width, height, rgb.tobytes())

    def close(self) -> None:
        if not self._closed:
            self._capture.release()
            self._closed = True

    def __enter__(self) -> OpenCVCamera:
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()
