"""Transport- and framework-neutral domain values."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum


@dataclass(frozen=True, slots=True)
class Pose:
    frame_id: str
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


@dataclass(frozen=True, slots=True)
class CameraFrame:
    """Framework-neutral RGB camera sample ordered by a monotonic clock."""

    timestamp_ns: int
    width: int
    height: int
    rgb_bytes: bytes


@dataclass(frozen=True, slots=True)
class PickPlaceTaskState:
    """Privileged task state, deliberately separate from policy observations."""

    cube_pose: Pose
    desired_cube_pose: Pose
    success: bool
    terminal: bool
    reason: str | None


class CommandOutcome(StrEnum):
    """Inspectable result of applying Cartesian intent through safety."""

    APPLIED = "applied"
    LIMITED = "limited"
    DISABLED = "disabled"
    STALE = "stale"
    INVALID = "invalid"
    IK_FAILURE = "ik_failure"
    COLLISION_STOP = "collision_stop"


@dataclass(frozen=True, slots=True)
class CommandReport:
    """Safety decision associated with the most recent robot command."""

    outcome: CommandOutcome
    requested_action: Action
    executed_action: Action | None
    resulting_state: RobotState
    detail: str

    @property
    def requested_intent(self) -> Action:
        return self.requested_action

    @property
    def executed_intent(self) -> Action | None:
        return self.executed_action
