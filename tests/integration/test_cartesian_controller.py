from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np
import pytest

from act_lab.adapters.mujoco import MujocoCartesianDriver, MujocoUR5eEnvironment
from act_lab.adapters.mujoco.config import SimulationConfig
from act_lab.application import SafeCartesianRobot
from act_lab.domain import Action, CommandOutcome, Pose, RobotState

CONFIG_PATH = Path("configs/sim/ur5e_pick_place.toml")


def make_driver(config: SimulationConfig | None = None) -> MujocoCartesianDriver:
    resolved = config or SimulationConfig.load(CONFIG_PATH)
    return MujocoCartesianDriver(MujocoUR5eEnvironment(resolved), resolved)


def quaternion_distance(left: tuple[float, ...], right: tuple[float, ...]) -> float:
    dot = min(abs(sum(a * b for a, b in zip(left, right, strict=True))), 1.0)
    return 2.0 * math.acos(dot)


def yaw_pose(pose: Pose, angle: float, position: tuple[float, float, float]) -> Pose:
    half = angle / 2.0
    delta = (math.cos(half), 0.0, 0.0, math.sin(half))
    w1, x1, y1, z1 = delta
    w2, x2, y2, z2 = pose.quaternion_wxyz
    quaternion = (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )
    return Pose("world", position, quaternion)


def test_control_configuration_has_documented_defaults() -> None:
    control = SimulationConfig.load(CONFIG_PATH).control

    assert control.watchdog_timeout_ns == 100_000_000
    assert control.workspace_x_m == (-0.2, 0.8)
    assert control.workspace_y_m == (-0.35, 0.6)
    assert control.workspace_z_m == (0.44, 1.0)
    assert control.max_translation_velocity_m_s == 0.25
    assert control.max_translation_acceleration_m_s2 == 1.0
    assert control.max_orientation_velocity_rad_s == 1.0
    assert control.max_orientation_acceleration_rad_s2 == 4.0
    assert control.max_joint_velocity_rad_s == 1.0
    assert control.max_joint_acceleration_rad_s2 == 4.0
    assert control.joint_bound_margin_rad == 0.02
    assert control.max_gripper_velocity_s == 2.0
    assert control.dls_damping == 0.05
    assert control.ik_max_iterations == 50
    assert control.ik_position_tolerance_m == 0.002
    assert control.ik_orientation_tolerance_rad == 0.02


def test_full_pose_converges_from_home_within_controller_tolerances() -> None:
    with make_driver() as driver:
        robot = SafeCartesianRobot(driver, driver.limits)
        state = robot.reset(11).robot
        home = state.end_effector_pose
        target = yaw_pose(
            home,
            0.08,
            (
                home.position_xyz_m[0] + 0.03,
                home.position_xyz_m[1] - 0.03,
                home.position_xyz_m[2] + 0.03,
            ),
        )
        for _ in range(250):
            state = robot.command(
                Action(state.timestamp_ns, target, 0.2, enabled=True)
            )

    assert math.dist(
        state.end_effector_pose.position_xyz_m, target.position_xyz_m
    ) <= 0.002
    assert (
        quaternion_distance(
            state.end_effector_pose.quaternion_wxyz, target.quaternion_wxyz
        )
        <= 0.02
    )


@pytest.mark.parametrize(
    ("joints", "approach_xy"),
    [
        (
            (
                -0.035311,
                -1.491761,
                -1.514101,
                -1.706474,
                -4.712415,
                -1.606097,
            ),
            (0.45, -0.15),
        ),
        (
            (
                -2.980660,
                -1.290932,
                1.244473,
                -1.524414,
                -1.570795,
                -1.409871,
            ),
            (0.58, 0.23),
        ),
    ],
)
def test_full_pose_converges_from_cube_and_tray_approach_configurations(
    joints: tuple[float, float, float, float, float, float],
    approach_xy: tuple[float, float],
) -> None:
    base = SimulationConfig.load(CONFIG_PATH)
    config = replace(base, home_joint_positions_rad=joints)
    with make_driver(config) as driver:
        robot = SafeCartesianRobot(driver, driver.limits)
        state = robot.reset(0).robot
        initial = state.end_effector_pose
        assert initial.position_xyz_m[:2] == pytest.approx(approach_xy, abs=2e-3)
        target = yaw_pose(
            initial,
            0.03,
            (
                initial.position_xyz_m[0],
                initial.position_xyz_m[1],
                initial.position_xyz_m[2] + 0.01,
            ),
        )
        for _ in range(150):
            state = robot.command(
                Action(state.timestamp_ns, target, 0.1, enabled=True)
            )

    assert math.dist(
        state.end_effector_pose.position_xyz_m, target.position_xyz_m
    ) <= 0.002
    assert (
        quaternion_distance(
            state.end_effector_pose.quaternion_wxyz, target.quaternion_wxyz
        )
        <= 0.02
    )


def test_unreachable_nonconvergent_and_collision_candidates_hold() -> None:
    base = SimulationConfig.load(CONFIG_PATH)
    permissive = replace(
        base.control,
        max_translation_velocity_m_s=100.0,
        max_translation_acceleration_m_s2=10_000.0,
        max_orientation_velocity_rad_s=100.0,
        max_orientation_acceleration_rad_s2=10_000.0,
        max_joint_velocity_rad_s=100.0,
        max_joint_acceleration_rad_s2=10_000.0,
    )
    config = replace(base, control=permissive)
    with make_driver(config) as driver:
        robot = SafeCartesianRobot(driver, driver.limits)
        initial = robot.reset(0).robot
        unreachable = Pose("world", (0.8, 0.6, 1.0), (1.0, 0.0, 0.0, 0.0))
        state = robot.command(
            Action(initial.timestamp_ns, unreachable, 0.0, enabled=True)
        )
        assert robot.last_command_report is not None
        assert robot.last_command_report.outcome is CommandOutcome.IK_FAILURE
        assert state.joint_positions_rad == pytest.approx(
            initial.joint_positions_rad, abs=2e-3
        )

        initial = robot.reset(0).robot
        cube = driver.environment.cube_position_xyz_m
        colliding = Pose(
            "world",
            (cube[0], cube[1], 0.58),
            initial.end_effector_pose.quaternion_wxyz,
        )
        robot.command(Action(initial.timestamp_ns, colliding, 0.0, enabled=True))
        assert robot.last_command_report is not None
        assert robot.last_command_report.outcome is CommandOutcome.COLLISION_STOP

    one_iteration = replace(permissive, ik_max_iterations=1)
    one_iteration_config = replace(base, control=one_iteration)
    with make_driver(one_iteration_config) as driver:
        robot = SafeCartesianRobot(driver, driver.limits)
        initial = robot.reset(0).robot
        pose = initial.end_effector_pose
        target = Pose(
            "world",
            (pose.position_xyz_m[0] + 0.1, *pose.position_xyz_m[1:]),
            yaw_pose(pose, 0.3, pose.position_xyz_m).quaternion_wxyz,
        )
        robot.command(Action(initial.timestamp_ns, target, 0.0, enabled=True))
        assert robot.last_command_report is not None
        assert robot.last_command_report.outcome is CommandOutcome.IK_FAILURE


def test_reset_contacts_are_classified_and_pad_cube_contact_is_allowed() -> None:
    with make_driver() as driver:
        observation = driver.reset(0)
        assert driver._data.ncon == 0  # noqa: SLF001
        model = driver._model  # noqa: SLF001
        data = driver._data  # noqa: SLF001
        left = driver._geom_id("left_finger_pad")  # noqa: SLF001
        right = driver._geom_id("right_finger_pad")  # noqa: SLF001
        midpoint = (data.geom_xpos[left] + data.geom_xpos[right]) / 2.0
        cube_address = model.jnt_qposadr[driver.environment._cube_joint_id]  # noqa: SLF001
        data.qpos[cube_address : cube_address + 7] = (*midpoint, 1.0, 0.0, 0.0, 0.0)
        mujoco.mj_forward(model, data)

        assert driver.collision_free(
            tuple(observation.robot.joint_positions_rad),  # type: ignore[arg-type]
            0.0,
        )


def test_fixed_seed_randomized_5000_step_rollout_is_safe_and_finite() -> None:
    config = SimulationConfig.load(CONFIG_PATH)
    rng = np.random.default_rng(20260831)
    with make_driver(config) as driver:
        robot = SafeCartesianRobot(driver, driver.limits)
        state = robot.reset(19).robot
        home = state.end_effector_pose
        target = home
        previous = state
        for step in range(5_000):
            if step % 20 == 0:
                position = tuple(
                    home.position_xyz_m[index] + float(rng.uniform(-0.015, 0.015))
                    for index in range(3)
                )
                target = yaw_pose(home, float(rng.uniform(-0.03, 0.03)), position)
                gripper = float(rng.uniform(0.0, 1.0))
            state = robot.command(
                Action(state.timestamp_ns, target, gripper, enabled=True)
            )
            report = robot.last_command_report
            assert report is not None
            assert report.outcome in {CommandOutcome.APPLIED, CommandOutcome.LIMITED}
            values = (
                *state.joint_positions_rad,
                *state.joint_velocities_rad_s,
                *state.end_effector_pose.position_xyz_m,
                *state.end_effector_pose.quaternion_wxyz,
                state.gripper_position,
            )
            assert all(math.isfinite(value) for value in values)
            for value, bounds in zip(
                state.joint_positions_rad,
                driver.joint_position_bounds_rad,
                strict=True,
            ):
                assert bounds[0] <= value <= bounds[1]
            assert all(
                abs(value) <= config.control.max_joint_velocity_rad_s + 0.05
                for value in state.joint_velocities_rad_s
            )
            speed = math.dist(
                state.end_effector_pose.position_xyz_m,
                previous.end_effector_pose.position_xyz_m,
            ) / driver.control_period_s
            assert speed <= config.control.max_translation_velocity_m_s + 0.02
            assert driver.collision_free(
                tuple(state.joint_positions_rad),  # type: ignore[arg-type]
                state.gripper_position,
            )
            previous = state


def deterministic_sequence(seed: int) -> list[RobotState]:
    with make_driver() as driver:
        robot = SafeCartesianRobot(driver, driver.limits)
        state = robot.reset(seed).robot
        pose = state.end_effector_pose
        target = Pose(
            "world",
            (pose.position_xyz_m[0] + 0.01, *pose.position_xyz_m[1:]),
            pose.quaternion_wxyz,
        )
        result = []
        for _ in range(100):
            state = robot.command(
                Action(state.timestamp_ns, target, 0.2, enabled=True)
            )
            result.append(state)
        return result


def test_identical_seed_and_actions_are_deterministic() -> None:
    assert deterministic_sequence(23) == deterministic_sequence(23)
