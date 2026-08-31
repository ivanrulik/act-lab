"""Validated, framework-neutral configuration for the MuJoCo adapter."""

from __future__ import annotations

import math
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class SimulationConfig:
    physics_hz: int
    environment_hz: int
    home_joint_positions_rad: tuple[float, float, float, float, float, float]
    episode_steps: int
    cube_spawn_x_m: tuple[float, float]
    cube_spawn_y_m: tuple[float, float]
    cube_center_z_m: float
    cube_half_extent_m: float
    tray_center_xy_m: tuple[float, float]
    tray_inner_half_extents_xy_m: tuple[float, float]
    tray_floor_top_z_m: float
    support_tolerance_m: float
    linear_speed_threshold_m_s: float
    angular_speed_threshold_rad_s: float
    settling_steps: int
    render_width: int
    render_height: int
    cameras: tuple[str, ...]

    @property
    def substeps(self) -> int:
        return self.physics_hz // self.environment_hz

    @property
    def physics_step_ns(self) -> int:
        return 1_000_000_000 // self.physics_hz

    @classmethod
    def load(cls, path: Path) -> SimulationConfig:
        with path.open("rb") as config_file:
            raw: dict[str, Any] = tomllib.load(config_file)

        clock = _table(raw, "clock")
        robot = _table(raw, "robot")
        task = _table(raw, "task")
        render = _table(raw, "render")
        config = cls(
            physics_hz=_positive_int(clock, "physics_hz"),
            environment_hz=_positive_int(clock, "environment_hz"),
            home_joint_positions_rad=_six_float_tuple(
                robot, "home_joint_positions_rad"
            ),
            episode_steps=_positive_int(task, "episode_steps"),
            cube_spawn_x_m=_float_pair(task, "cube_spawn_x_m"),
            cube_spawn_y_m=_float_pair(task, "cube_spawn_y_m"),
            cube_center_z_m=_finite_float(task, "cube_center_z_m"),
            cube_half_extent_m=_positive_float(task, "cube_half_extent_m"),
            tray_center_xy_m=_float_pair(task, "tray_center_xy_m"),
            tray_inner_half_extents_xy_m=_float_pair(
                task, "tray_inner_half_extents_xy_m"
            ),
            tray_floor_top_z_m=_finite_float(task, "tray_floor_top_z_m"),
            support_tolerance_m=_positive_float(task, "support_tolerance_m"),
            linear_speed_threshold_m_s=_positive_float(
                task, "linear_speed_threshold_m_s"
            ),
            angular_speed_threshold_rad_s=_positive_float(
                task, "angular_speed_threshold_rad_s"
            ),
            settling_steps=_positive_int(task, "settling_steps"),
            render_width=_positive_int(render, "width"),
            render_height=_positive_int(render, "height"),
            cameras=_string_tuple(render, "cameras"),
        )
        config._validate()
        return config

    def _validate(self) -> None:
        if self.physics_hz % self.environment_hz != 0:
            raise ValueError(
                "physics_hz must be an integer multiple of environment_hz"
            )
        if 1_000_000_000 % self.physics_hz != 0:
            raise ValueError(
                "physics_hz must divide one billion for integer timestamps"
            )
        for name, bounds in (
            ("cube_spawn_x_m", self.cube_spawn_x_m),
            ("cube_spawn_y_m", self.cube_spawn_y_m),
        ):
            if bounds[0] >= bounds[1]:
                raise ValueError(f"{name} lower bound must be less than upper bound")
        if any(value <= 0 for value in self.tray_inner_half_extents_xy_m):
            raise ValueError("tray inner half extents must be positive")
        if len(set(self.cameras)) != len(self.cameras):
            raise ValueError("camera names must be unique")


def _table(raw: dict[str, Any], name: str) -> dict[str, Any]:
    value = raw.get(name)
    if not isinstance(value, dict):
        raise ValueError(f"missing [{name}] table")
    return value


def _positive_int(table: dict[str, Any], name: str) -> int:
    value = table.get(name)
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _finite_float(table: dict[str, Any], name: str) -> float:
    value = table.get(name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _positive_float(table: dict[str, Any], name: str) -> float:
    value = _finite_float(table, name)
    if value <= 0:
        raise ValueError(f"{name} must be positive")
    return value


def _float_tuple(
    table: dict[str, Any], name: str, length: int
) -> tuple[float, ...]:
    value = table.get(name)
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{name} must contain exactly {length} values")
    parsed: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError(f"{name} values must be numeric")
        converted = float(item)
        if not math.isfinite(converted):
            raise ValueError(f"{name} values must be finite")
        parsed.append(converted)
    return tuple(parsed)


def _float_pair(table: dict[str, Any], name: str) -> tuple[float, float]:
    values = _float_tuple(table, name, 2)
    return values[0], values[1]


def _six_float_tuple(
    table: dict[str, Any], name: str
) -> tuple[float, float, float, float, float, float]:
    values = _float_tuple(table, name, 6)
    return values[0], values[1], values[2], values[3], values[4], values[5]


def _string_tuple(table: dict[str, Any], name: str) -> tuple[str, ...]:
    value = table.get(name)
    if not isinstance(value, list) or not value:
        raise ValueError(f"{name} must be a non-empty list")
    if not all(isinstance(item, str) and item for item in value):
        raise ValueError(f"{name} values must be non-empty strings")
    return tuple(value)
