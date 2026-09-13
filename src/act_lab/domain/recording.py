"""Framework-independent acquisition records and episode labels."""

from dataclasses import dataclass
from enum import StrEnum

from act_lab.domain.models import CameraFrame, CommandReport, Observation


class EpisodeOutcome(StrEnum):
    UNKNOWN = "unknown"
    SUCCESS = "success"
    FAILURE = "failure"
    DISCARDED = "discarded"
    INTERRUPTED = "interrupted"


@dataclass(frozen=True, slots=True)
class EpisodeProvenance:
    application_version: str
    git_revision: str
    git_dirty: str
    seed: int
    source: str
    robot_id: str
    task_id: str
    camera_ids: tuple[str, ...]
    operator: str
    container_identity: str
    clock_id: str
    host_monotonic_start_ns: int
    wall_start_utc: str
    resolved_config_json: str
    rates_json: str
    calibration_json: str


@dataclass(frozen=True, slots=True)
class EpisodeSample:
    """Post-command observation and images at one simulation instant."""

    observation: Observation
    command: CommandReport
    images: tuple[tuple[str, CameraFrame], ...] = ()
