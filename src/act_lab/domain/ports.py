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
    def append(self, observation: Observation, action: Action) -> None: ...

    def close(self, *, success: bool) -> None: ...
