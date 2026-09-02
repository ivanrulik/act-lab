"""Non-blocking latest-sample worker for camera capture and hand tracking."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable

from act_lab.adapters.mediapipe.hand_tracker import HandSignal, MediaPipeHandTracker
from act_lab.domain import Camera


class HandTrackingWorker:
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
        self._thread: threading.Thread | None = None
        self._failure: Exception | None = None

    @property
    def failure(self) -> Exception | None:
        return self._failure

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("tracking worker already started")
        self._thread = threading.Thread(
            target=self._run, name="act-lab-hand-tracking", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                frame = self._camera.capture()
                received_ns = time.monotonic_ns()
                if frame is None:
                    self._callback(None, received_ns, "end_of_stream")
                    return
                signal = self._tracker.track(frame)
                self._callback(
                    signal,
                    received_ns,
                    "tracked" if signal is not None else "tracking_lost",
                )
        except Exception as error:
            if self._stop.is_set():
                return
            self._failure = error
            self._callback(None, time.monotonic_ns(), f"worker_failure: {error}")

    def stop(self) -> None:
        self._stop.set()
        self._camera.close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                raise RuntimeError("tracking worker did not stop")

    def __enter__(self) -> HandTrackingWorker:
        self.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self.stop()
