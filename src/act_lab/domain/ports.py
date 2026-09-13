"""Ports implemented by simulation, hardware, device, and storage adapters."""

from __future__ import annotations

from typing import Protocol

from act_lab.domain.models import (
    Action,
    CameraFrame,
    Observation,
    PickPlaceTaskState,
    RobotState,
)
from act_lab.domain.recording import EpisodeOutcome, EpisodeProvenance, EpisodeSample


class Robot(Protocol):
    def reset(self, seed: int) -> Observation: ...

    def observe(self) -> Observation: ...

    def command(self, action: Action) -> RobotState: ...


class Camera(Protocol):
    def capture(self) -> CameraFrame | None: ...

    def close(self) -> None: ...


class Teleoperator(Protocol):
    def poll(self, observation: Observation) -> Action: ...


class PickPlaceTaskStateSource(Protocol):
    """Privileged task-state port for scripted teaching/evaluation baselines."""

    def task_state(self) -> PickPlaceTaskState: ...


class EpisodeSink(Protocol):
    def start(self, provenance: EpisodeProvenance, timestamp_ns: int) -> str: ...

    def append(self, sample: EpisodeSample) -> None: ...

    def event(
        self, timestamp_ns: int, kind: str, outcome: EpisodeOutcome, reason: str
    ) -> None: ...

    def diagnostics(
        self,
        timestamp_ns: int,
        host_monotonic_ns: int,
        values_json: str,
        calibration_json: str,
    ) -> None: ...

    def stop(self, outcome: EpisodeOutcome, reason: str) -> None: ...

    def discard(self, reason: str) -> None: ...

    def interrupt(self) -> None: ...
