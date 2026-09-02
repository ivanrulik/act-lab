"""Deterministic privileged finite-state expert for simulated pick-and-place."""

from __future__ import annotations

import math
from enum import StrEnum
from typing import Protocol

from act_lab.domain import (
    Action,
    CommandOutcome,
    CommandReport,
    Observation,
    PickPlaceTaskState,
    PickPlaceTaskStateSource,
    Pose,
)


class ExpertSettings(Protocol):
    @property
    def approach_clearance_m(self) -> float: ...

    @property
    def transit_clearance_m(self) -> float: ...

    @property
    def retreat_clearance_m(self) -> float: ...

    @property
    def tool_to_cube_offset_m(self) -> float: ...

    @property
    def position_tolerance_m(self) -> float: ...

    @property
    def grasp_position_tolerance_m(self) -> float: ...

    @property
    def gripper_tolerance(self) -> float: ...

    @property
    def close_dwell_steps(self) -> int: ...

    @property
    def open_dwell_steps(self) -> int: ...

    @property
    def open_gripper(self) -> float: ...

    @property
    def closed_gripper(self) -> float: ...


class ExpertPhase(StrEnum):
    OPEN_RAISE = "open_raise"
    APPROACH_CUBE = "approach_cube"
    DESCEND = "descend"
    CLOSE_DWELL = "close_dwell"
    LIFT = "lift"
    TRANSIT = "transit"
    LOWER = "lower"
    OPEN_DWELL = "open_dwell"
    RETREAT = "retreat"
    HOLD = "hold"


_INFEASIBLE_OUTCOMES = {
    CommandOutcome.INVALID,
    CommandOutcome.STALE,
    CommandOutcome.IK_FAILURE,
    CommandOutcome.COLLISION_STOP,
}


class ScriptedPickPlaceExpert:
    """Observation-paced baseline that alone may read privileged task geometry."""

    def __init__(
        self,
        task_state_source: PickPlaceTaskStateSource,
        settings: ExpertSettings,
    ) -> None:
        self._source = task_state_source
        self._settings = settings
        self._phase = ExpertPhase.HOLD
        self._orientation = (1.0, 0.0, 0.0, 0.0)
        self._initial_xy = (0.0, 0.0)
        self._cube_xyz = (0.0, 0.0, 0.0)
        self._goal_xyz = (0.0, 0.0, 0.0)
        self._safe_z = 0.0
        self._transit_z = 0.0
        self._dwell_steps = 0
        self._failure_reason: str | None = None

    @property
    def phase(self) -> ExpertPhase:
        return self._phase

    @property
    def failure_reason(self) -> str | None:
        return self._failure_reason

    def reset(self, observation: Observation) -> None:
        task = self._source.task_state()
        pose = observation.robot.end_effector_pose
        self._orientation = pose.quaternion_wxyz
        self._initial_xy = pose.position_xyz_m[:2]
        self._cube_xyz = task.cube_pose.position_xyz_m
        self._goal_xyz = task.desired_cube_pose.position_xyz_m
        grasp_z = self._cube_xyz[2] + self._settings.tool_to_cube_offset_m
        self._safe_z = max(
            pose.position_xyz_m[2], grasp_z + self._settings.approach_clearance_m
        )
        self._transit_z = (
            max(self._cube_xyz[2], self._goal_xyz[2])
            + self._settings.tool_to_cube_offset_m
            + self._settings.transit_clearance_m
        )
        self._phase = ExpertPhase.OPEN_RAISE
        self._dwell_steps = 0
        self._failure_reason = None

    def record_command_report(self, report: CommandReport) -> None:
        if report.outcome in _INFEASIBLE_OUTCOMES:
            self._failure_reason = report.outcome.value
            self._phase = ExpertPhase.HOLD

    def poll(self, observation: Observation) -> Action:
        task = self._source.task_state()
        if task.terminal or self._failure_reason is not None:
            self._phase = ExpertPhase.HOLD
        else:
            self._advance_if_satisfied(observation, task)

        if self._phase is ExpertPhase.HOLD:
            return self._action(
                observation,
                observation.robot.end_effector_pose,
                observation.robot.gripper_position,
                enabled=False,
            )
        target, gripper = self._target_for_phase()
        return self._action(observation, target, gripper, enabled=True)

    def _advance_if_satisfied(
        self, observation: Observation, task: PickPlaceTaskState
    ) -> None:
        del task
        target, gripper = self._target_for_phase()
        precise_phases = {
            ExpertPhase.APPROACH_CUBE,
            ExpertPhase.DESCEND,
            ExpertPhase.CLOSE_DWELL,
            ExpertPhase.LOWER,
            ExpertPhase.OPEN_DWELL,
        }
        position_tolerance = (
            self._settings.grasp_position_tolerance_m
            if self._phase in precise_phases
            else self._settings.position_tolerance_m
        )
        pose_reached = (
            math.dist(
                observation.robot.end_effector_pose.position_xyz_m,
                target.position_xyz_m,
            )
            <= position_tolerance
        )
        gripper_reached = (
            abs(observation.robot.gripper_position - gripper)
            <= self._settings.gripper_tolerance
        )
        if self._phase is ExpertPhase.CLOSE_DWELL:
            self._dwell_or_advance(
                pose_reached and gripper_reached,
                self._settings.close_dwell_steps,
                ExpertPhase.LIFT,
            )
            return
        if self._phase is ExpertPhase.OPEN_DWELL:
            self._dwell_or_advance(
                pose_reached and gripper_reached,
                self._settings.open_dwell_steps,
                ExpertPhase.RETREAT,
            )
            return
        if not pose_reached or not gripper_reached:
            return
        transitions = {
            ExpertPhase.OPEN_RAISE: ExpertPhase.APPROACH_CUBE,
            ExpertPhase.APPROACH_CUBE: ExpertPhase.DESCEND,
            ExpertPhase.DESCEND: ExpertPhase.CLOSE_DWELL,
            ExpertPhase.LIFT: ExpertPhase.TRANSIT,
            ExpertPhase.TRANSIT: ExpertPhase.LOWER,
            ExpertPhase.LOWER: ExpertPhase.OPEN_DWELL,
            ExpertPhase.RETREAT: ExpertPhase.HOLD,
        }
        next_phase = transitions.get(self._phase)
        if next_phase is not None:
            self._phase = next_phase
            self._dwell_steps = 0

    def _dwell_or_advance(
        self, satisfied: bool, required: int, next_phase: ExpertPhase
    ) -> None:
        self._dwell_steps = self._dwell_steps + 1 if satisfied else 0
        if self._dwell_steps >= required:
            self._phase = next_phase
            self._dwell_steps = 0

    def _target_for_phase(self) -> tuple[Pose, float]:
        offset = self._settings.tool_to_cube_offset_m
        grasp_z = self._cube_xyz[2] + offset
        release_z = self._goal_xyz[2] + offset
        if self._phase is ExpertPhase.OPEN_RAISE:
            position = (*self._initial_xy, self._safe_z)
            gripper = self._settings.open_gripper
        elif self._phase is ExpertPhase.APPROACH_CUBE:
            position = (self._cube_xyz[0], self._cube_xyz[1], self._safe_z)
            gripper = self._settings.open_gripper
        elif self._phase in {ExpertPhase.DESCEND, ExpertPhase.CLOSE_DWELL}:
            position = (self._cube_xyz[0], self._cube_xyz[1], grasp_z)
            gripper = (
                self._settings.closed_gripper
                if self._phase is ExpertPhase.CLOSE_DWELL
                else self._settings.open_gripper
            )
        elif self._phase is ExpertPhase.LIFT:
            position = (self._cube_xyz[0], self._cube_xyz[1], self._transit_z)
            gripper = self._settings.closed_gripper
        elif self._phase is ExpertPhase.TRANSIT:
            position = (self._goal_xyz[0], self._goal_xyz[1], self._transit_z)
            gripper = self._settings.closed_gripper
        elif self._phase in {ExpertPhase.LOWER, ExpertPhase.OPEN_DWELL}:
            position = (self._goal_xyz[0], self._goal_xyz[1], release_z)
            gripper = (
                self._settings.open_gripper
                if self._phase is ExpertPhase.OPEN_DWELL
                else self._settings.closed_gripper
            )
        elif self._phase is ExpertPhase.RETREAT:
            position = (
                self._goal_xyz[0],
                self._goal_xyz[1],
                release_z + self._settings.retreat_clearance_m,
            )
            gripper = self._settings.open_gripper
        else:
            raise RuntimeError("hold phase has no enabled target")
        return Pose("world", position, self._orientation), gripper

    @staticmethod
    def _action(
        observation: Observation, target: Pose, gripper: float, *, enabled: bool
    ) -> Action:
        return Action(
            timestamp_ns=observation.timestamp_ns,
            target_pose=target,
            gripper_position=gripper,
            enabled=enabled,
        )
