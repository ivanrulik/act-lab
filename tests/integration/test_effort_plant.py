"""Real torque integration and dynamic holds, without ROS or state freezing."""

import math
from pathlib import Path

import numpy as np
import pytest

from act_lab.adapters.mujoco.config import SimulationConfig
from act_lab.adapters.mujoco.effort import MujocoEffortPlant


@pytest.fixture
def plant():
    value = MujocoEffortPlant(
        SimulationConfig.load(Path("configs/sim/ur5e_pick_place.toml"))
    )
    value.reset(0)
    yield value
    value.close()


def test_startup_hold_advances_physics_without_drift(plant) -> None:
    initial = plant.observe().robot
    for _ in range(250):
        sample = plant.tick()
    assert sample.state.timestamp_ns == 500_000_000
    assert (
        math.dist(
            initial.end_effector_pose.position_xyz_m,
            sample.state.end_effector_pose.position_xyz_m,
        )
        < 0.002
    )
    assert max(abs(v) for v in sample.state.joint_velocities_rad_s) < 0.01
    assert sample.task_effort_nm == (0.0,) * 6


def test_torque_moves_and_fault_hold_retains_gripper(plant) -> None:
    initial = plant.observe().robot
    plant.enable(0.4)
    for _ in range(25):
        sample = plant.tick([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert sample.state.joint_positions_rad[0] > initial.joint_positions_rad[0]
    assert sample.servo_effort_nm == (0.0,) * 6
    assert abs(sample.task_effort_nm[0] - 1.0) < 1e-12
    before = sample.state
    plant.hold("gateway_loss")
    for _ in range(250):
        sample = plant.tick()
    assert sample.mode == "FAULT_HOLD"
    assert sample.task_effort_nm == (0.0,) * 6
    assert sample.accepted_gripper == 0.4
    assert (
        math.dist(
            before.end_effector_pose.position_xyz_m,
            sample.state.end_effector_pose.position_xyz_m,
        )
        < 0.002
    )
    assert max(abs(v) for v in sample.state.joint_velocities_rad_s) < 0.01


def test_nonfinite_output_inhibits_and_actuators_are_exclusive(plant) -> None:
    plant.enable(0.2)
    assert np.count_nonzero(plant._model.actuator_biasprm[:12]) == 0
    sample = plant.tick([float("nan")] * 6)
    assert sample.reason == "invalid_effort"
    assert sample.task_effort_nm == (0.0,) * 6
    assert sample.accepted_gripper == 0.2


def test_moving_watchdog_takeover_respects_measured_acceleration(plant) -> None:
    """Reproduce main CI's pre-timeout state, independently of wall scheduling."""
    import mujoco

    positions = (
        -0.03605151722179273,
        -1.735067956065026,
        -0.6191591523264554,
        -2.3590075411456244,
        1.5708396400579854,
        4.67633042035226,
    )
    velocities = (
        0.0004736242628733745,
        0.020494280893616757,
        -0.03502750624707234,
        0.01641820344541701,
        -9.111772623747803e-05,
        0.00047091500638239697,
    )
    plant.enable(0.0)
    for joint, position, velocity in zip(
        plant.environment._arm_joint_ids, positions, velocities, strict=True
    ):
        plant._data.qpos[plant._model.jnt_qposadr[joint]] = position
        plant._data.qvel[plant._model.jnt_dofadr[joint]] = velocity
    mujoco.mj_forward(plant._model, plant._data)
    before = plant.observe().robot.end_effector_pose
    plant.hold("gateway_wall_timeout")
    for _ in range(500):
        sample = plant.tick()
        assert sample.maximum_joint_acceleration_rad_s2 <= 4.0
        assert sample.cartesian_acceleration_m_s2 <= 1.0
        assert sample.angular_acceleration_rad_s2 <= 4.0
        assert not any(sample.task_effort_nm)
        assert (
            math.dist(
                before.position_xyz_m, sample.state.end_effector_pose.position_xyz_m
            )
            < 0.002
        )
    assert max(abs(v) for v in sample.state.joint_velocities_rad_s) < 0.01


def test_loaded_gripper_hold_after_effort_transition(plant) -> None:
    from act_lab.application import SafeCartesianRobot
    from act_lab.application.scripted_expert import ExpertPhase, ScriptedPickPlaceExpert

    robot = SafeCartesianRobot(plant, plant.limits)
    observation = robot.reset(0)
    # Prepare the fixture with the unchanged local driver servo configuration.
    plant._restore_servos()
    expert = ScriptedPickPlaceExpert(plant.environment, plant.simulation_config.expert)
    expert.reset(observation)
    for _ in range(400):
        robot.command(expert.poll(observation))
        observation = robot.observe()
        if expert.phase is ExpertPhase.TRANSIT:
            break
    assert expert.phase is ExpertPhase.TRANSIT
    assert robot.last_report is not None
    assert robot.last_report.executed_action is not None
    aperture = robot.last_report.executed_action.gripper_position
    cube_z = plant.environment.task_state().cube_pose.position_xyz_m[2]
    assert cube_z > plant.simulation_config.cube_center_z_m + 0.04
    plant.enable(aperture)
    plant.tick([0.0] * 6)
    plant.hold("producer_loss")
    held = plant.observe().robot.end_effector_pose
    for _ in range(250):
        sample = plant.tick()
        assert (
            math.dist(
                held.position_xyz_m, sample.state.end_effector_pose.position_xyz_m
            )
            < 0.002
        )
        assert (
            plant.environment.task_state().cube_pose.position_xyz_m[2] > cube_z - 0.01
        )
    assert sample.accepted_gripper == aperture
    assert max(abs(v) for v in sample.state.joint_velocities_rad_s) < 0.01


def test_predicted_unsafe_torque_transfers_to_bounded_hold(plant) -> None:
    initial = plant.observe().robot
    plant.enable(0.3)
    for _ in range(50):
        sample = plant.tick([0.0, 0.0, 0.0, 1000.0, 0.0, 0.0])
        if sample.mode != "ENABLED":
            break
    assert sample.mode == "FAULT_HOLD"
    assert sample.reason in {"joint_acceleration", "angular_acceleration"}
    assert sample.task_effort_nm == (0.0,) * 6
    assert sample.accepted_gripper == 0.3
    assert sample.state.timestamp_ns > initial.timestamp_ns
    assert all(
        abs(v) <= bound
        for v, bound in zip(
            sample.total_effort_nm, (150, 150, 150, 28, 28, 28), strict=True
        )
    )


def test_near_singular_fixture_rejects_unreachable_ik_without_motion() -> None:
    from dataclasses import replace

    import mujoco

    from act_lab.domain import Pose

    config = SimulationConfig.load(Path("configs/sim/ur5e_pick_place.toml"))
    joints = list(config.home_joint_positions_rad)
    joints[4] = 0.0
    singular = MujocoEffortPlant(
        replace(config, home_joint_positions_rad=tuple(joints))
    )
    try:
        initial = singular.reset(0).robot
        jp = np.zeros((3, singular._model.nv))
        jr = np.zeros_like(jp)
        mujoco.mj_jacSite(singular._model, singular._data, jp, jr, singular._site_id)
        assert (
            np.linalg.matrix_rank(
                np.vstack((jp, jr))[:, singular._dof_addresses], tol=1e-6
            )
            < 6
        )
        rejected = singular.solve_ik(
            Pose("world", (0.8, 0.6, 1.0), initial.end_effector_pose.quaternion_wxyz)
        )
        assert rejected is None
        assert (
            singular.observe().robot.joint_positions_rad == initial.joint_positions_rad
        )
        singular.hold("ik_failure")
        for _ in range(250):
            sample = singular.tick()
            assert sample.mode == "FAULT_HOLD"
            assert not any(sample.task_effort_nm)
        assert (
            math.dist(
                initial.end_effector_pose.position_xyz_m,
                sample.state.end_effector_pose.position_xyz_m,
            )
            < 0.002
        )
    finally:
        singular.close()


def test_predicted_joint_margin_crossing_is_replaced_by_dynamic_hold(plant):
    from dataclasses import replace

    config = plant.simulation_config
    joints = list(config.home_joint_positions_rad)
    joints[5] = (
        plant.joint_position_bounds_rad[5][0]
        + config.control.joint_bound_margin_rad
        + 1e-7
    )
    bounded = MujocoEffortPlant(replace(config, home_joint_positions_rad=tuple(joints)))
    try:
        bounded.reset(0)
        bounded.enable(0.2)
        sample = bounded.tick([0.0, 0.0, 0.0, 0.0, 0.0, -5.0])
        assert sample.mode == "FAULT_HOLD"
        assert sample.reason == "joint_boundary"
        assert not any(sample.task_effort_nm)
        assert (
            sample.state.joint_positions_rad[5]
            >= bounded.joint_position_bounds_rad[5][0]
            + config.control.joint_bound_margin_rad
        )
        assert sample.state.timestamp_ns == 2_000_000
    finally:
        bounded.close()


def test_observational_reads_preserve_the_same_effort_and_hold_path(plant):
    other = MujocoEffortPlant(
        SimulationConfig.load(Path("configs/sim/ur5e_pick_place.toml"))
    )
    other.reset(0)
    try:
        plant.enable(0.4)
        other.enable(0.4)
        for tick in range(150):
            if tick == 25:
                plant.hold("producer_loss")
                other.hold("producer_loss")
            effort = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0] if tick < 25 else None
            expected = plant.tick(effort)
            actual = other.tick(effort)
            snapshot = other.observation_sample()
            assert actual == expected
            assert snapshot.state == actual.state
            assert snapshot.total_effort_nm == actual.total_effort_nm
            assert snapshot.accepted_gripper == 0.4
    finally:
        other.close()
