"""MediaPipe types terminate here; callers receive plain Python values."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from act_lab.adapters.mediapipe.config import WebcamConfig
from act_lab.domain import CameraFrame

_PALM = (0, 5, 9, 17)
_CLUTCH_FINGERS = ((10, 12), (14, 16), (18, 20))
_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),
)


@dataclass(frozen=True, slots=True)
class HandSignal:
    source_timestamp_ns: int
    handedness: str
    confidence: float
    palm_x: float
    palm_y: float
    palm_scale: float
    pinch_ratio: float
    clutch: bool
    preview: CameraFrame


class MediaPipeHandTracker:
    """Synchronous VIDEO-mode tracker intended for a dedicated worker thread."""

    def __init__(self, model_path: Path, config: WebcamConfig) -> None:
        import mediapipe as mp  # type: ignore[import-untyped]

        if not model_path.is_file():
            raise OSError(f"MediaPipe hand model not found: {model_path}")
        self._mp = mp
        self._config = config
        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(model_path)),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=config.min_detection_confidence,
            min_hand_presence_confidence=config.min_presence_confidence,
            min_tracking_confidence=config.min_tracking_confidence,
        )
        self._landmarker: Any = mp.tasks.vision.HandLandmarker.create_from_options(
            options
        )
        self._last_timestamp_ms = -1

    def track(self, frame: CameraFrame) -> HandSignal | None:
        pixels = np.frombuffer(frame.rgb_bytes, dtype=np.uint8).reshape(
            frame.height, frame.width, 3
        )
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=pixels)
        timestamp_ms = max(frame.timestamp_ns // 1_000_000, self._last_timestamp_ms + 1)
        self._last_timestamp_ms = timestamp_ms
        result = self._landmarker.detect_for_video(image, timestamp_ms)
        if not result.hand_landmarks or not result.handedness:
            return None
        category = result.handedness[0][0]
        confidence = float(category.score)
        if confidence < self._config.min_handedness_confidence:
            return None
        landmarks = tuple(
            (float(item.x), float(item.y), float(item.z))
            for item in result.hand_landmarks[0]
        )
        if len(landmarks) != 21 or not all(
            math.isfinite(value) for point in landmarks for value in point
        ):
            return None
        palm_x = sum(landmarks[index][0] for index in _PALM) / len(_PALM)
        palm_y = sum(landmarks[index][1] for index in _PALM) / len(_PALM)
        palm_scale = _distance(landmarks[5], landmarks[17])
        if palm_scale <= 1e-6:
            return None
        pinch_ratio = _distance(landmarks[4], landmarks[8]) / palm_scale
        wrist = landmarks[0]
        clutch = all(
            _distance(landmarks[tip], wrist)
            > _distance(landmarks[pip], wrist)
            + self._config.clutch_extension_margin * palm_scale
            for pip, tip in _CLUTCH_FINGERS
        )
        preview = _annotate(frame, landmarks, clutch)
        return HandSignal(
            source_timestamp_ns=frame.timestamp_ns,
            handedness=str(category.category_name),
            confidence=confidence,
            palm_x=palm_x,
            palm_y=palm_y,
            palm_scale=palm_scale,
            pinch_ratio=pinch_ratio,
            clutch=clutch,
            preview=preview,
        )

    def close(self) -> None:
        self._landmarker.close()

    def __enter__(self) -> MediaPipeHandTracker:
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()


def _distance(
    left: tuple[float, float, float], right: tuple[float, float, float]
) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right, strict=True)))


def _annotate(
    frame: CameraFrame,
    landmarks: tuple[tuple[float, float, float], ...],
    clutch: bool,
) -> CameraFrame:
    import cv2

    pixels = np.frombuffer(frame.rgb_bytes, dtype=np.uint8).reshape(
        frame.height, frame.width, 3
    ).copy()
    points = tuple(
        (round(point[0] * frame.width), round(point[1] * frame.height))
        for point in landmarks
    )
    color = (40, 220, 80) if clutch else (240, 180, 30)
    for start, end in _CONNECTIONS:
        cv2.line(pixels, points[start], points[end], color, 2)
    for point in points:
        cv2.circle(pixels, point, 3, (255, 255, 255), -1)
    return CameraFrame(
        frame.timestamp_ns, frame.width, frame.height, pixels.tobytes()
    )
