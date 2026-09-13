from __future__ import annotations

import threading
import time

from act_lab.adapters.mediapipe import HandTrackingWorker
from act_lab.domain import CameraFrame


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


def test_worker_drops_obsolete_frames_instead_of_queueing_them() -> None:
    frame = CameraFrame(0, 1, 1, b"\0\0\0")

    class BurstCamera(FakeCamera):
        def __init__(self) -> None:
            super().__init__()
            self.remaining = 5

        def capture(self) -> CameraFrame | None:
            if self.remaining == 0:
                return None
            self.remaining -= 1
            return frame

    class SlowTracker:
        def track(self, value: CameraFrame) -> CameraFrame:
            time.sleep(0.02)
            return value

    ended = threading.Event()
    worker = HandTrackingWorker(
        BurstCamera(),  # type: ignore[arg-type]
        SlowTracker(),  # type: ignore[arg-type]
        lambda _signal, _received, status: (
            ended.set() if status == "end_of_stream" else None
        ),
    )
    worker.start()
    assert ended.wait(1.0)
    worker.stop()

    assert worker.captured_frames == 5
    assert worker.processed_frames < worker.captured_frames
    assert worker.dropped_frames > 0
