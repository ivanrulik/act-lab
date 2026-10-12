from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from act_lab.adapters.mujoco import SimulationConfig
from act_lab.application import ExpertPhase, ScriptedPickPlaceExpert
from act_lab.domain import (
    Action,
    CommandOutcome,
    CommandReport,
    Observation,
    PickPlaceTaskState,
    Pose,
    RobotState,
)

CONFIG = SimulationConfig.load(Path("configs/sim/ur5e_pick_place.toml"))
ORIENTATION = (1.0, 0.0, 0.0, 0.0)
HOME = Pose("world", (0.45, -0.15, 0.65), ORIENTATION)
CUBE = Pose("world", (0.4, -0.1, 0.425), ORIENTATION)
GOAL = Pose("world", (0.58, 0.23, 0.445), ORIENTATION)


class FakeTaskStateSource:
    def __init__(self) -> None:
        self.state = PickPlaceTaskState(CUBE, GOAL, False, False, None)

    def task_state(self) -> PickPlaceTaskState:
        return self.state


def observation(timestamp_ns: int, pose: Pose, gripper: float) -> Observation:
    robot = RobotState(timestamp_ns, (0.0,) * 6, (0.0,) * 6, pose, gripper)
    return Observation(timestamp_ns, robot, ())


def satisfy_action(action: Action, timestamp_ns: int) -> Observation:
    return observation(timestamp_ns, action.target_pose, action.gripper_position)


def test_explicit_cube_approach_height_preserves_default_initial_raise() -> None:
    current = observation(17, HOME, CONFIG.expert.open_gripper)
    for clearance in (None, 0.08):
        settings = replace(CONFIG.expert, approach_cube_clearance_m=clearance)
        expert = ScriptedPickPlaceExpert(FakeTaskStateSource(), settings)
        expert.reset(current)
        action = expert.poll(current)
        assert expert.phase is ExpertPhase.APPROACH_CUBE
        assert action.timestamp_ns == 17
        expected = (
            HOME.position_xyz_m[2]
            if clearance is None
            else CUBE.position_xyz_m[2] + settings.tool_to_cube_offset_m + clearance
        )
        assert action.target_pose.position_xyz_m[2] == pytest.approx(expected)


def test_expert_runs_every_phase_with_fresh_timestamps_and_dwell() -> None:
    source = FakeTaskStateSource()
    expert = ScriptedPickPlaceExpert(source, CONFIG.expert)
    current = observation(0, HOME, 0.0)
    expert.reset(current)
    visited: list[ExpertPhase] = []

    for step in range(100):
        action = expert.poll(current)
        assert action.timestamp_ns == current.timestamp_ns
        if not visited or visited[-1] is not expert.phase:
            visited.append(expert.phase)
        if expert.phase is ExpertPhase.HOLD:
            assert not action.enabled
            break
        current = satisfy_action(action, (step + 1) * 20_000_000)

    assert visited == list(ExpertPhase)


def test_dwell_resets_when_tolerance_is_lost() -> None:
    source = FakeTaskStateSource()
    expert = ScriptedPickPlaceExpert(source, CONFIG.expert)
    current = observation(0, HOME, CONFIG.expert.open_gripper)
    expert.reset(current)
    while expert.phase is not ExpertPhase.CLOSE_DWELL:
        action = expert.poll(current)
        current = satisfy_action(action, current.timestamp_ns + 20_000_000)

    for _ in range(CONFIG.expert.close_dwell_steps - 1):
        action = expert.poll(current)
        current = satisfy_action(action, current.timestamp_ns + 20_000_000)
    current = observation(current.timestamp_ns, HOME, 1.0)
    expert.poll(current)
    assert expert.phase is ExpertPhase.CLOSE_DWELL


def test_terminal_failure_and_reset_are_disabled_and_deterministic() -> None:
    source = FakeTaskStateSource()
    expert = ScriptedPickPlaceExpert(source, CONFIG.expert)
    initial = observation(0, HOME, 0.0)
    expert.reset(initial)
    first = expert.poll(initial)
    expert.reset(initial)
    assert expert.poll(initial) == first

    state = initial.robot
    report = CommandReport(
        CommandOutcome.IK_FAILURE,
        first,
        None,
        state,
        "infeasible",
    )
    expert.record_command_report(report)
    assert not expert.poll(initial).enabled
    assert expert.failure_reason == "ik_failure"

    expert.reset(initial)
    source.state = PickPlaceTaskState(CUBE, GOAL, True, True, "success")
    assert not expert.poll(initial).enabled
    assert expert.phase is ExpertPhase.HOLD


@pytest.mark.parametrize("release_tolerance", [None, 0.002])
def test_placement_precision_is_independent_of_grasp_precision(release_tolerance):
    from dataclasses import replace

    settings = replace(
        CONFIG.expert,
        grasp_position_tolerance_m=0.004,
        release_position_tolerance_m=release_tolerance,
    )
    expert = ScriptedPickPlaceExpert(FakeTaskStateSource(), settings)
    current = observation(0, HOME, settings.open_gripper)
    expert.reset(current)
    while expert.phase is not ExpertPhase.DESCEND:
        action = expert.poll(current)
        current = satisfy_action(action, current.timestamp_ns + 20_000_000)
    above = replace(
        current.robot.end_effector_pose,
        position_xyz_m=(
            *current.robot.end_effector_pose.position_xyz_m[:2],
            current.robot.end_effector_pose.position_xyz_m[2] + 0.010,
        ),
    )
    expert.poll(observation(current.timestamp_ns, above, settings.open_gripper))
    assert expert.phase is ExpertPhase.DESCEND
    while expert.phase is not ExpertPhase.LOWER:
        action = expert.poll(current)
        current = satisfy_action(action, current.timestamp_ns + 20_000_000)
    above = replace(
        current.robot.end_effector_pose,
        position_xyz_m=(
            *current.robot.end_effector_pose.position_xyz_m[:2],
            current.robot.end_effector_pose.position_xyz_m[2] + 0.010,
        ),
    )
    action = expert.poll(
        observation(current.timestamp_ns, above, settings.closed_gripper)
    )
    if release_tolerance is None:
        assert expert.phase is ExpertPhase.OPEN_DWELL
        assert action.gripper_position == settings.open_gripper
    else:
        assert expert.phase is ExpertPhase.LOWER
        assert action.gripper_position == settings.closed_gripper


def test_configured_transfer_route_reaches_each_waypoint_then_destination():
    from dataclasses import replace

    waypoints = ((0.50, 0.10, 0.575), (0.54, 0.19, 0.575))
    settings = replace(
        CONFIG.expert,
        transfer_waypoints_xyz_m=waypoints,
        destination_transit_clearance_m=0.065,
    )
    expert = ScriptedPickPlaceExpert(FakeTaskStateSource(), settings)
    current = observation(0, HOME, 0.0)
    expert.reset(current)
    visited = []
    for _ in range(100):
        action = expert.poll(current)
        if expert.phase is ExpertPhase.TRANSIT:
            point = action.target_pose.position_xyz_m
            if not visited or point != visited[-1]:
                visited.append(point)
        if expert.phase is ExpertPhase.LOWER:
            break
        current = satisfy_action(action, current.timestamp_ns + 20_000_000)
    assert visited == [
        *waypoints,
        (
            GOAL.position_xyz_m[0],
            GOAL.position_xyz_m[1],
            GOAL.position_xyz_m[2] + settings.tool_to_cube_offset_m + 0.065,
        ),
    ]
    expert.reset(observation(0, HOME, 0.0))
    assert expert.phase is ExpertPhase.OPEN_RAISE
