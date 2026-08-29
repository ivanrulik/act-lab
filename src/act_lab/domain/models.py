"""Transport- and framework-neutral domain values."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Pose:
    position_xyz_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class Action:
    """Cartesian intent at a monotonic timestamp, not a driver command."""

    timestamp_ns: int
    target_pose: Pose
    gripper_position: float
    enabled: bool


@dataclass(frozen=True, slots=True)
class RobotState:
    timestamp_ns: int
    joint_positions_rad: tuple[float, ...]
    joint_velocities_rad_s: tuple[float, ...]
    end_effector_pose: Pose
    gripper_position: float


@dataclass(frozen=True, slots=True)
class Observation:
    timestamp_ns: int
    robot: RobotState
    image_keys: tuple[str, ...]

