from __future__ import annotations

from pathlib import Path

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
