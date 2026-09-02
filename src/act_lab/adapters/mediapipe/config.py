"""Validated configuration for webcam hand teleoperation."""

from __future__ import annotations

import math
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class WebcamConfig:
    width: int
    height: int
    fps: int
    max_frame_age_ms: int
    min_detection_confidence: float
    min_presence_confidence: float
    min_tracking_confidence: float
    min_handedness_confidence: float
    clutch_extension_margin: float
    clutch_engage_frames: int
    calibration_sample_frames: int
    calibration_max_anchor_spread: float
    calibration_max_scale_spread: float
    world_x_gain_m: float
    world_y_gain_m: float
    world_z_gain_m: float
    image_xy_dead_zone: float
    depth_dead_zone: float
    ema_alpha: float
    depth_ema_alpha: float
    pinch_closed_ratio: float
    pinch_open_ratio: float

    @classmethod
    def load(cls, path: Path) -> WebcamConfig:
        with path.open("rb") as config_file:
            raw: dict[str, Any] = tomllib.load(config_file)
        capture = _table(raw, "capture")
        tracking = _table(raw, "tracking")
        calibration = _table(raw, "calibration")
        mapping = _table(raw, "mapping")
        config = cls(
            width=_positive_int(capture, "width"),
            height=_positive_int(capture, "height"),
            fps=_positive_int(capture, "fps"),
            max_frame_age_ms=_positive_int(capture, "max_frame_age_ms"),
            min_detection_confidence=_unit(tracking, "min_detection_confidence"),
            min_presence_confidence=_unit(tracking, "min_presence_confidence"),
            min_tracking_confidence=_unit(tracking, "min_tracking_confidence"),
            min_handedness_confidence=_unit(
                tracking, "min_handedness_confidence"
            ),
            clutch_extension_margin=_nonnegative(
                tracking, "clutch_extension_margin"
            ),
            clutch_engage_frames=_positive_int(tracking, "clutch_engage_frames"),
            calibration_sample_frames=_positive_int(calibration, "sample_frames"),
            calibration_max_anchor_spread=_positive(
                calibration, "max_anchor_spread"
            ),
            calibration_max_scale_spread=_positive(
                calibration, "max_scale_spread"
            ),
            world_x_gain_m=_finite(mapping, "world_x_gain_m"),
            world_y_gain_m=_finite(mapping, "world_y_gain_m"),
            world_z_gain_m=_finite(mapping, "world_z_gain_m"),
            image_xy_dead_zone=_nonnegative(mapping, "image_xy_dead_zone"),
            depth_dead_zone=_nonnegative(mapping, "depth_dead_zone"),
            ema_alpha=_unit(mapping, "ema_alpha"),
            depth_ema_alpha=_unit(mapping, "depth_ema_alpha"),
            pinch_closed_ratio=_nonnegative(mapping, "pinch_closed_ratio"),
            pinch_open_ratio=_positive(mapping, "pinch_open_ratio"),
        )
        config._validate()
        return config

    def _validate(self) -> None:
        if self.max_frame_age_ms >= 100:
            raise ValueError("max_frame_age_ms must be below the 100 ms watchdog")
        if self.ema_alpha == 0.0 or self.depth_ema_alpha == 0.0:
            raise ValueError("mapping EMA alpha values must be greater than zero")
        if self.pinch_closed_ratio >= self.pinch_open_ratio:
            raise ValueError("pinch_closed_ratio must be below pinch_open_ratio")
        if self.image_xy_dead_zone >= 1.0 or self.depth_dead_zone >= 1.0:
            raise ValueError("mapping dead zones must be below one")
        if not any(
            abs(value) > 0.0
            for value in (
                self.world_x_gain_m,
                self.world_y_gain_m,
                self.world_z_gain_m,
            )
        ):
            raise ValueError("at least one mapping gain must be non-zero")


def _table(raw: dict[str, Any], name: str) -> dict[str, Any]:
    value = raw.get(name)
    if not isinstance(value, dict):
        raise ValueError(f"missing [{name}] table")
    return value


def _finite(table: dict[str, Any], name: str) -> float:
    value = table.get(name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _positive(table: dict[str, Any], name: str) -> float:
    result = _finite(table, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def _nonnegative(table: dict[str, Any], name: str) -> float:
    result = _finite(table, name)
    if result < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return result


def _unit(table: dict[str, Any], name: str) -> float:
    result = _finite(table, name)
    if not 0.0 <= result <= 1.0:
        raise ValueError(f"{name} must be in [0, 1]")
    return result


def _positive_int(table: dict[str, Any], name: str) -> int:
    value = table.get(name)
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value
