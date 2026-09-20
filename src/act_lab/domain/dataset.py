"""Framework-independent records used by validation and dataset conversion."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum


class IssueSeverity(StrEnum):
    WARNING = "warning"
    ERROR = "error"


@dataclass(frozen=True, slots=True)
class QualityIssue:
    code: str
    severity: IssueSeverity
    message: str


@dataclass(frozen=True, slots=True)
class RecordedImage:
    camera_id: str
    width: int
    height: int
    encoding: str
    rgb_bytes: bytes


@dataclass(frozen=True, slots=True)
class RecordedSample:
    timestamp_ns: int
    joint_positions_rad: tuple[float, ...]
    joint_velocities_rad_s: tuple[float, ...]
    end_effector_position_xyz_m: tuple[float, ...]
    end_effector_quaternion_wxyz: tuple[float, ...]
    gripper_position: float
    action_position_xyz_m: tuple[float, ...]
    action_quaternion_wxyz: tuple[float, ...]
    action_gripper_position: float
    action_enabled: bool
    command_outcome: str
    images: tuple[RecordedImage, ...]


@dataclass(frozen=True, slots=True)
class RecordedEpisode:
    episode_id: str
    outcome: str
    last_task_outcome: str
    complete: bool
    provenance: dict[str, object]
    samples: tuple[RecordedSample, ...]
    stream_counts: tuple[tuple[str, int], ...]
    tracking_states: tuple[tuple[int, str], ...]


@dataclass(frozen=True, slots=True)
class QualityReport:
    episode_id: str
    outcome: str
    valid: bool
    training_eligible: bool
    sample_count: int
    duration_ns: int
    issues: tuple[QualityIssue, ...]
