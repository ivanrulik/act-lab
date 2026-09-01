from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import pytest

from act_lab.adapters.mujoco.config import SimulationConfig
from act_lab.application import SafeCartesianRobot
from act_lab.application.cartesian_control import JointVector
from act_lab.domain import (
    Action,
    CommandOutcome,
    Observation,
    Pose,
    RobotState,
)

CONFIG = SimulationConfig.load(Path("configs/sim/ur5e_pick_place.toml"))
HOME_POSE = Pose("world", (0.2, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0))
HOME_JOINTS: JointVector = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class FakeCartesianDriver:
    control_period_s = 0.02
    joint_position_bounds_rad = ((-2.0, 2.0),) * 6

    def __init__(self) -> None:
        self.ik_result: JointVector | None = HOME_JOINTS
        self.is_collision_free = True
        self.pose_from_ik = HOME_POSE
        self.state = self._state(0, HOME_JOINTS, HOME_POSE, 0.0)

    def reset(self, seed: int) -> Observation:
        del seed
        self.state = self._state(0, HOME_JOINTS, HOME_POSE, 0.0)
        return self.observe()

    def observe(self) -> Observation:
        return Observation(self.state.timestamp_ns, self.state, ())

    def solve_ik(self, target_pose: Pose) -> JointVector | None:
        self.pose_from_ik = target_pose
        return self.ik_result

    def collision_free(self, joints: JointVector, gripper: float) -> bool:
        del joints, gripper
        return self.is_collision_free

    def step(self, joints: JointVector, gripper: float) -> RobotState:
        self.state = self._state(
            self.state.timestamp_ns + 20_000_000,
            joints,
            self.pose_from_ik,
            gripper,
        )
        return self.state

    @staticmethod
    def _state(
        timestamp_ns: int,
        joints: JointVector,
        pose: Pose,
        gripper: float,
    ) -> RobotState:
        return RobotState(timestamp_ns, joints, (0.0,) * 6, pose, gripper)


def make_robot() -> tuple[SafeCartesianRobot, FakeCartesianDriver]:
    driver = FakeCartesianDriver()
    robot = SafeCartesianRobot(driver, CONFIG.control)
    robot.reset(0)
    return robot, driver


def action(
    *,
    timestamp_ns: int = 0,
    pose: Pose = HOME_POSE,
    gripper: float = 0.0,
    enabled: bool = True,
) -> Action:
    return Action(timestamp_ns, pose, gripper, enabled)


def outcome(robot: SafeCartesianRobot) -> CommandOutcome:
    assert robot.last_command_report is not None
    return robot.last_command_report.outcome


def test_applied_command_reports_requested_executed_and_resulting_state() -> None:
    robot, _ = make_robot()
    requested = action()

    resulting_state = robot.command(requested)

    assert outcome(robot) is CommandOutcome.APPLIED
    assert robot.last_command_report is not None
    assert robot.last_command_report.requested_action is requested
    assert robot.last_command_report.executed_action == requested
    assert robot.last_command_report.resulting_state is resulting_state


def test_disabled_command_holds_without_validating_bad_payload() -> None:
    robot, _ = make_robot()
    bad_pose = Pose("bad", (math.nan, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))

    robot.command(action(pose=bad_pose, gripper=2.0, enabled=False))

    assert outcome(robot) is CommandOutcome.DISABLED
    assert robot.last_command_report is not None
    assert robot.last_command_report.executed_action is None


def test_watchdog_rejects_boundary_and_future_timestamp() -> None:
    robot, _ = make_robot()

    robot.command(action(timestamp_ns=-CONFIG.control.watchdog_timeout_ns))
    assert outcome(robot) is CommandOutcome.STALE

    robot.reset(0)
    robot.command(action(timestamp_ns=1))
    assert outcome(robot) is CommandOutcome.INVALID


def test_repeated_command_stops_at_watchdog_deadline() -> None:
    robot, _ = make_robot()
    repeated = action(timestamp_ns=0)

    results = []
    for _ in range(6):
        robot.command(repeated)
        results.append(outcome(robot))

    assert results[:5] == [CommandOutcome.APPLIED] * 5
    assert results[5] is CommandOutcome.STALE


@pytest.mark.parametrize(
    ("pose", "gripper"),
    [
        (Pose("world", (math.nan, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0)), 0.0),
        (Pose("tool", (0.2, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0)), 0.0),
        (Pose("world", (0.2, 0.0, 0.6), (0.0, 0.0, 0.0, 0.0)), 0.0),
        (HOME_POSE, -0.01),
        (HOME_POSE, 1.01),
    ],
)
def test_invalid_values_frames_quaternions_and_gripper_hold(
    pose: Pose, gripper: float
) -> None:
    robot, _ = make_robot()

    robot.command(action(pose=pose, gripper=gripper))

    assert outcome(robot) is CommandOutcome.INVALID


def test_workspace_pose_gripper_and_acceleration_limits_are_reported() -> None:
    robot, _ = make_robot()
    far_pose = Pose("world", (2.0, 0.0, 0.6), (0.0, 0.0, 0.0, 1.0))

    robot.command(action(pose=far_pose, gripper=1.0))

    assert outcome(robot) is CommandOutcome.LIMITED
    assert robot.last_command_report is not None
    executed = robot.last_command_report.executed_action
    assert executed is not None
    assert executed.target_pose.position_xyz_m[0] == pytest.approx(0.2004)
    rotation = executed.target_pose.quaternion_wxyz
    assert 2.0 * math.acos(rotation[0]) == pytest.approx(0.0016)
    assert executed.gripper_position == pytest.approx(0.04)


def test_workspace_clipping_uses_configured_boundary() -> None:
    driver = FakeCartesianDriver()
    limits = replace(
        CONFIG.control,
        max_translation_velocity_m_s=1_000.0,
        max_translation_acceleration_m_s2=1_000_000.0,
    )
    robot = SafeCartesianRobot(driver, limits)
    robot.reset(0)

    robot.command(
        action(pose=Pose("world", (2.0, 0.0, 0.6), HOME_POSE.quaternion_wxyz))
    )

    assert robot.last_report is not None
    assert robot.last_report.executed_intent is not None
    assert robot.last_report.executed_intent.target_pose.position_xyz_m[0] == 0.8


def test_joint_velocity_acceleration_and_bounds_are_limited() -> None:
    robot, driver = make_robot()
    driver.ik_result = (3.0, -3.0, 1.0, -1.0, 0.5, -0.5)

    state = robot.command(action())

    assert outcome(robot) is CommandOutcome.LIMITED
    assert state.joint_positions_rad == pytest.approx(
        (0.0016, -0.0016, 0.0016, -0.0016, 0.0016, -0.0016)
    )


def test_ik_failure_and_collision_candidate_hold_safely() -> None:
    robot, driver = make_robot()
    driver.ik_result = None
    state = robot.command(action())
    assert outcome(robot) is CommandOutcome.IK_FAILURE
    assert state.joint_positions_rad == HOME_JOINTS

    robot.reset(0)
    driver.ik_result = HOME_JOINTS
    driver.is_collision_free = False
    state = robot.command(action())
    assert outcome(robot) is CommandOutcome.COLLISION_STOP
    assert state.joint_positions_rad == HOME_JOINTS


def test_reset_clears_rate_and_watchdog_history() -> None:
    robot, _ = make_robot()
    target = Pose("world", (0.5, 0.0, 0.6), HOME_POSE.quaternion_wxyz)
    robot.command(action(pose=target))
    first_delta = 0.0004

    robot.reset(123)
    robot.command(action(pose=target))

    assert robot.last_command_report is not None
    executed = robot.last_command_report.executed_action
    assert executed is not None
    assert executed.target_pose.position_xyz_m[0] == pytest.approx(
        HOME_POSE.position_xyz_m[0] + first_delta
    )
