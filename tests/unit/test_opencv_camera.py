from __future__ import annotations

import sys
from types import SimpleNamespace

from act_lab.adapters.mediapipe.camera import OpenCVCamera


class FakeCapture:
    def __init__(self) -> None:
        self.settings: list[tuple[int, float]] = []

    def isOpened(self) -> bool:  # noqa: N802 - mirrors OpenCV
        return True

    def set(self, prop: int, value: float) -> bool:
        self.settings.append((prop, value))
        return True

    def release(self) -> None:
        pass


def test_live_camera_selects_v4l2_and_negotiates_yuyv_first(monkeypatch) -> None:
    capture = FakeCapture()
    calls: list[tuple[object, ...]] = []

    def video_capture(*args: object) -> FakeCapture:
        calls.append(args)
        return capture

    fake_cv2 = SimpleNamespace(
        CAP_V4L2=200,
        CAP_PROP_FOURCC=6,
        CAP_PROP_FRAME_WIDTH=3,
        CAP_PROP_FRAME_HEIGHT=4,
        CAP_PROP_FPS=5,
        VideoCapture=video_capture,
        VideoWriter_fourcc=lambda *_chars: 0x56595559,
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)

    with OpenCVCamera("/dev/video0", width=640, height=480, fps=30):
        pass

    assert calls == [("/dev/video0", fake_cv2.CAP_V4L2)]
    assert capture.settings == [
        (fake_cv2.CAP_PROP_FOURCC, 0x56595559),
        (fake_cv2.CAP_PROP_FRAME_WIDTH, 640),
        (fake_cv2.CAP_PROP_FRAME_HEIGHT, 480),
        (fake_cv2.CAP_PROP_FPS, 30),
    ]
