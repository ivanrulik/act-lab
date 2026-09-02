from __future__ import annotations

import threading

from act_lab.adapters.mediapipe import HandTrackingWorker


class FakeCamera:
    def __init__(self, error: Exception | None = None) -> None:
        self.error = error
        self.closed = False

    def capture(self) -> None:
        if self.error is not None:
            raise self.error
        return None

    def close(self) -> None:
        self.closed = True


class FakeTracker:
    def track(self, _frame: object) -> None:
        return None


def run_worker(
    camera: FakeCamera,
) -> tuple[HandTrackingWorker, list[str]]:
    completed = threading.Event()
    statuses: list[str] = []

    def callback(_signal: object, _received_ns: int, status: str) -> None:
        statuses.append(status)
        completed.set()

    worker = HandTrackingWorker(camera, FakeTracker(), callback)  # type: ignore[arg-type]
    worker.start()
    assert completed.wait(1.0)
    worker.stop()
    return worker, statuses


def test_worker_reports_end_of_stream_and_closes_camera() -> None:
    camera = FakeCamera()
    worker, statuses = run_worker(camera)
    assert statuses == ["end_of_stream"]
    assert worker.failure is None
    assert camera.closed


def test_worker_surfaces_capture_failure() -> None:
    camera = FakeCamera(OSError("camera disconnected"))
    worker, statuses = run_worker(camera)
    assert statuses == ["worker_failure: camera disconnected"]
    assert isinstance(worker.failure, OSError)
