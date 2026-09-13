"""Latest-frame capture and tracking worker for low-latency hand input."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable

from act_lab.adapters.mediapipe.hand_tracker import HandSignal, MediaPipeHandTracker
from act_lab.domain import Camera, CameraFrame


class HandTrackingWorker:
    """Capture continuously and discard frames inference cannot keep up with."""

    def __init__(
        self,
        camera: Camera,
        tracker: MediaPipeHandTracker,
        callback: Callable[[HandSignal | None, int, str], None],
    ) -> None:
        self._camera = camera
        self._tracker = tracker
        self._callback = callback
        self._stop = threading.Event()
        self._condition = threading.Condition()
        self._capture_thread: threading.Thread | None = None
        self._tracking_thread: threading.Thread | None = None
        self._latest: tuple[CameraFrame, int] | None = None
        self._capture_ended = False
        self._failure: Exception | None = None
        self._captured_frames = 0
        self._processed_frames = 0
        self._dropped_frames = 0

    @property
    def failure(self) -> Exception | None:
        return self._failure

    @property
    def captured_frames(self) -> int:
        return self._captured_frames

    @property
    def processed_frames(self) -> int:
        return self._processed_frames

    @property
    def dropped_frames(self) -> int:
        return self._dropped_frames

    @property
    def capture_metadata(self) -> dict[str, float]:
        metadata = getattr(self._camera, "negotiated_capture", {})
        return dict(metadata) if isinstance(metadata, dict) else {}

    def start(self) -> None:
        if self._capture_thread is not None:
            raise RuntimeError("tracking worker already started")
        if not getattr(self._camera, "drop_obsolete_frames", True):
            self._capture_thread = threading.Thread(
                target=self._sequential_loop,
                name="act-lab-recorded-hand-tracking",
                daemon=True,
            )
            self._capture_thread.start()
            return
        self._capture_thread = threading.Thread(
            target=self._capture_loop, name="act-lab-camera-capture", daemon=True
        )
        self._tracking_thread = threading.Thread(
            target=self._tracking_loop, name="act-lab-hand-tracking", daemon=True
        )
        self._capture_thread.start()
        self._tracking_thread.start()

    def _sequential_loop(self) -> None:
        try:
            while not self._stop.is_set():
                frame = self._camera.capture()
                captured_ns = time.monotonic_ns()
                if frame is None:
                    self._callback(None, captured_ns, "end_of_stream")
                    return
                self._captured_frames += 1
                signal = self._tracker.track(frame)
                self._processed_frames += 1
                self._callback(
                    signal,
                    captured_ns,
                    "tracked" if signal is not None else "tracking_lost",
                )
        except Exception as error:
            self._record_failure(error)

    def _capture_loop(self) -> None:
        try:
            while not self._stop.is_set():
                frame = self._camera.capture()
                captured_ns = time.monotonic_ns()
                with self._condition:
                    if frame is None:
                        self._capture_ended = True
                        self._condition.notify_all()
                        return
                    self._captured_frames += 1
                    if self._latest is not None:
                        self._dropped_frames += 1
                    self._latest = (frame, captured_ns)
                    self._condition.notify_all()
        except Exception as error:
            self._record_failure(error)

    def _tracking_loop(self) -> None:
        try:
            while not self._stop.is_set():
                with self._condition:
                    self._condition.wait_for(
                        lambda: (
                            self._latest is not None
                            or self._capture_ended
                            or self._failure is not None
                            or self._stop.is_set()
                        )
                    )
                    if self._failure is not None or self._stop.is_set():
                        return
                    item = self._latest
                    self._latest = None
                    ended = self._capture_ended
                if item is None:
                    if ended:
                        self._callback(None, time.monotonic_ns(), "end_of_stream")
                        return
                    continue
                frame, captured_ns = item
                signal = self._tracker.track(frame)
                self._processed_frames += 1
                self._callback(
                    signal,
                    captured_ns,
                    "tracked" if signal is not None else "tracking_lost",
                )
        except Exception as error:
            self._record_failure(error)

    def _record_failure(self, error: Exception) -> None:
        if self._stop.is_set():
            return
        with self._condition:
            if self._failure is not None:
                return
            self._failure = error
            self._condition.notify_all()
        self._callback(None, time.monotonic_ns(), f"worker_failure: {error}")

    def stop(self) -> None:
        self._stop.set()
        self._camera.close()
        with self._condition:
            self._condition.notify_all()
        for thread in (self._capture_thread, self._tracking_thread):
            if thread is not None:
                thread.join(timeout=2.0)
                if thread.is_alive():
                    raise RuntimeError(f"{thread.name} did not stop")

    def __enter__(self) -> HandTrackingWorker:
        self.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self.stop()
