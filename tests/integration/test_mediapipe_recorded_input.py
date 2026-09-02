from __future__ import annotations

import base64
import hashlib
import os
from pathlib import Path

from act_lab.adapters.mediapipe import (
    MediaPipeHandTracker,
    OpenCVCamera,
    WebcamConfig,
)

FIXTURE = Path("tests/fixtures/webcam/open_hand_loss.mp4.b64")
MODEL = Path(
    os.environ.get(
        "ACT_LAB_HAND_MODEL", "/opt/act-lab/models/hand_landmarker.task"
    )
)


def test_recorded_video_tracks_open_hand_then_reports_loss(tmp_path: Path) -> None:
    video_bytes = base64.b64decode(FIXTURE.read_text())
    assert hashlib.sha256(video_bytes).hexdigest() == (
        "59fc6063feaef2d444ec01d88f702c9bd2c1b18b960781bb89dc56ae863a0b63"
    )
    video = tmp_path / "open_hand_loss.mp4"
    video.write_bytes(video_bytes)
    config = WebcamConfig.load(Path("configs/teleop/webcam.toml"))
    results = []

    with (
        OpenCVCamera(
            video,
            width=config.width,
            height=config.height,
            fps=config.fps,
            recorded=True,
        ) as camera,
        MediaPipeHandTracker(MODEL, config) as tracker,
    ):
        while (frame := camera.capture()) is not None:
            results.append(tracker.track(frame))

    tracked = [result for result in results[:30] if result is not None]
    assert len(results) == 36
    assert len(tracked) >= 25
    assert all(result.clutch for result in tracked)
    assert all(result is None for result in results[-3:])
