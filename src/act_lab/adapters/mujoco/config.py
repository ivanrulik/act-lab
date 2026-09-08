"""Validated, framework-neutral configuration for the MuJoCo adapter."""

from __future__ import annotations

import math
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class CartesianControlConfig:
    watchdog_timeout_ms: int
    workspace_x_m: tuple[float, float]
    workspace_y_m: tuple[float, float]
    workspace_z_m: tuple[float, float]
    max_translation_velocity_m_s: float
    max_translation_acceleration_m_s2: float
    max_orientation_velocity_rad_s: float
    max_orientation_acceleration_rad_s2: float
    max_joint_velocity_rad_s: float
    max_joint_acceleration_rad_s2: float
    joint_bound_margin_rad: float
    max_gripper_velocity_s: float
    dls_damping: float
    ik_max_iterations: int
    ik_position_tolerance_m: float
    ik_orientation_tolerance_rad: float

    @property
    def watchdog_timeout_ns(self) -> int:
        return self.watchdog_timeout_ms * 1_000_000


@dataclass(frozen=True, slots=True)
class KeyboardConfig:
    translation_nudge_m: float
    gripper_nudge: float
    translation_speed_m_s: float
    gripper_speed_s: float


@dataclass(frozen=True, slots=True)
class ExpertConfig:
    approach_clearance_m: float
    transit_clearance_m: float
    retreat_clearance_m: float
    tool_to_cube_offset_m: float
    position_tolerance_m: float
    grasp_position_tolerance_m: float
    gripper_tolerance: float
    close_dwell_steps: int
    open_dwell_steps: int
    open_gripper: float
    closed_gripper: float


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
    control: CartesianControlConfig
    keyboard: KeyboardConfig
    expert: ExpertConfig

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
        control = _table(raw, "control")
        keyboard = _table(raw, "keyboard")
        expert = _table(raw, "expert")
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
            control=CartesianControlConfig(
                watchdog_timeout_ms=_positive_int(control, "watchdog_timeout_ms"),
                workspace_x_m=_float_pair(control, "workspace_x_m"),
                workspace_y_m=_float_pair(control, "workspace_y_m"),
                workspace_z_m=_float_pair(control, "workspace_z_m"),
                max_translation_velocity_m_s=_positive_float(
                    control, "max_translation_velocity_m_s"
                ),
                max_translation_acceleration_m_s2=_positive_float(
                    control, "max_translation_acceleration_m_s2"
                ),
                max_orientation_velocity_rad_s=_positive_float(
                    control, "max_orientation_velocity_rad_s"
                ),
                max_orientation_acceleration_rad_s2=_positive_float(
                    control, "max_orientation_acceleration_rad_s2"
                ),
                max_joint_velocity_rad_s=_positive_float(
                    control, "max_joint_velocity_rad_s"
                ),
                max_joint_acceleration_rad_s2=_positive_float(
                    control, "max_joint_acceleration_rad_s2"
                ),
                joint_bound_margin_rad=_positive_float(
                    control, "joint_bound_margin_rad"
                ),
                max_gripper_velocity_s=_positive_float(
                    control, "max_gripper_velocity_s"
                ),
                dls_damping=_positive_float(control, "dls_damping"),
                ik_max_iterations=_positive_int(control, "ik_max_iterations"),
                ik_position_tolerance_m=_positive_float(
                    control, "ik_position_tolerance_m"
                ),
                ik_orientation_tolerance_rad=_positive_float(
                    control, "ik_orientation_tolerance_rad"
                ),
            ),
            keyboard=KeyboardConfig(
                translation_nudge_m=_positive_float(
                    keyboard, "translation_nudge_m"
                ),
                gripper_nudge=_positive_float(keyboard, "gripper_nudge"),
                translation_speed_m_s=_positive_float(
                    keyboard, "translation_speed_m_s"
                ),
                gripper_speed_s=_positive_float(keyboard, "gripper_speed_s"),
            ),
            expert=ExpertConfig(
                approach_clearance_m=_positive_float(
                    expert, "approach_clearance_m"
                ),
                transit_clearance_m=_nonnegative_float(
                    expert, "transit_clearance_m"
                ),
                retreat_clearance_m=_positive_float(
                    expert, "retreat_clearance_m"
                ),
                tool_to_cube_offset_m=_nonnegative_float(
                    expert, "tool_to_cube_offset_m"
                ),
                position_tolerance_m=_positive_float(
                    expert, "position_tolerance_m"
                ),
                grasp_position_tolerance_m=_positive_float(
                    expert, "grasp_position_tolerance_m"
                ),
                gripper_tolerance=_positive_float(expert, "gripper_tolerance"),
                close_dwell_steps=_positive_int(expert, "close_dwell_steps"),
                open_dwell_steps=_positive_int(expert, "open_dwell_steps"),
                open_gripper=_unit_float(expert, "open_gripper"),
                closed_gripper=_unit_float(expert, "closed_gripper"),
            ),
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
        if self.keyboard.gripper_nudge > 1.0:
            raise ValueError("gripper_nudge must be at most 1")
        if self.expert.gripper_tolerance > 1.0:
            raise ValueError("gripper_tolerance must be at most 1")
        if self.expert.closed_gripper >= self.expert.open_gripper:
            raise ValueError("closed_gripper must be less than open_gripper")
        if any(value <= 0 for value in self.tray_inner_half_extents_xy_m):
            raise ValueError("tray inner half extents must be positive")
        if len(set(self.cameras)) != len(self.cameras):
            raise ValueError("camera names must be unique")
        for name, bounds in (
            ("workspace_x_m", self.control.workspace_x_m),
            ("workspace_y_m", self.control.workspace_y_m),
            ("workspace_z_m", self.control.workspace_z_m),
        ):
            if bounds[0] >= bounds[1]:
                raise ValueError(f"{name} lower bound must be less than upper bound")


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


def _nonnegative_float(table: dict[str, Any], name: str) -> float:
    value = _finite_float(table, name)
    if value < 0:
        raise ValueError(f"{name} must be non-negative")
    return value


def _unit_float(table: dict[str, Any], name: str) -> float:
    value = _finite_float(table, name)
    if not 0.0 <= value <= 1.0:
        raise ValueError(f"{name} must be in [0, 1]")
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
