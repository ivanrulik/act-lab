from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np
import pytest

from act_lab.adapters.mujoco import (
    KeyboardTeleoperator,
    MujocoCartesianDriver,
    MujocoUR5eEnvironment,
)
from act_lab.adapters.mujoco.config import SimulationConfig
from act_lab.application import SafeCartesianRobot
from act_lab.application.scripted_expert import ExpertPhase, ScriptedPickPlaceExpert
from act_lab.domain import Action, CommandOutcome, Pose, RobotState

CONFIG_PATH = Path("configs/sim/ur5e_pick_place.toml")


def test_optional_driver_profiling_preserves_state_and_reports_bounded_ik() -> None:
    config = SimulationConfig.load(CONFIG_PATH)
    sequences = []
    for profile in (False, True):
        with MujocoCartesianDriver(
            MujocoUR5eEnvironment(config), config, profile=profile
        ) as driver:
            robot = SafeCartesianRobot(driver, config.control)
            initial = robot.reset(11012)
            pose = initial.robot.end_effector_pose
            target = Pose(
                "world", (pose.position_xyz_m[0] - 0.02, *pose.position_xyz_m[1:]),
                pose.quaternion_wxyz,
            )
            states = []
            for _ in range(20):
                observation = robot.observe()
                states.append(
                    robot.command(Action(observation.timestamp_ns, target, 1.0, True))
                )
                diagnostics = driver.last_ik_diagnostics
                assert diagnostics is not None and diagnostics.converged
                assert 0 <= diagnostics.iterations <= config.control.ik_max_iterations
                assert (
                    diagnostics.position_residual_m
                    <= config.control.ik_position_tolerance_m
                )
                assert (
                    diagnostics.orientation_residual_rad
                    <= config.control.ik_orientation_tolerance_rad
                )
                assert (diagnostics.duration_ns is not None) == profile
            sequences.append(states)
            if profile:
                assert all(value > 0 for value in driver.profile_totals_ns.values())
            robot.reset(11012)
            assert driver.last_ik_diagnostics is None
            assert not any(driver.profile_totals_ns.values())
    assert sequences[0] == sequences[1]


@pytest.mark.parametrize("numpy_reference", [False, True])
def test_optimized_ik_matches_reference(numpy_reference: bool) -> None:
    class FullForwardDriver(MujocoCartesianDriver):
        def _update_ik_kinematics(self) -> None:
            mujoco.mj_forward(self._model, self._scratch)

        def _solve_damped_system(self, system, error):
            if numpy_reference:
                return np.linalg.solve(system, error)
            return super()._solve_damped_system(system, error)

    config = SimulationConfig.load(CONFIG_PATH)
    fixture = json.loads(Path("tests/fixtures/control/ik_boundary.json").read_text())
    results = []
    for driver_type in (MujocoCartesianDriver, FullForwardDriver):
        with driver_type(MujocoUR5eEnvironment(config), config) as driver:
            driver.reset(fixture["seed"])
            for address, value in zip(
                driver._qpos_addresses, fixture["joints"], strict=True
            ):
                driver._data.qpos[address] = value
            mujoco.mj_forward(driver._model, driver._data)
            rng = np.random.default_rng(23)
            targets = [
                driver.observe().robot.end_effector_pose,
                Pose("world", tuple(fixture["position"]), tuple(fixture["quaternion"])),
                Pose("world", (0.8, 0.6, 1.0), (1.0, 0.0, 0.0, 0.0)),
            ]
            home = targets[0]
            for _ in range(20):
                targets.append(Pose(
                    "world",
                    tuple(
                        np.asarray(home.position_xyz_m) + rng.uniform(-0.02, 0.02, 3)
                    ),
                    home.quaternion_wxyz,
                ))
            answers = []
            for target in targets:
                joints = driver.solve_ik(target)
                answers.append((joints, driver.last_ik_diagnostics))
            robot = SafeCartesianRobot(driver, config.control)
            states = []
            for target in targets:
                observation = robot.observe()
                states.append((
                    robot.command(Action(observation.timestamp_ns, target, 1.0, True)),
                    robot.last_report,
                ))
            results.append((answers, states))
    if not numpy_reference:
        assert results[0] == results[1]
    else:
        for (actual, diagnostic), (expected, reference_diagnostic) in zip(
            results[0][0], results[1][0], strict=True
        ):
            assert diagnostic is not None and reference_diagnostic is not None
            assert diagnostic.iterations == reference_diagnostic.iterations
            assert diagnostic.converged == reference_diagnostic.converged
            assert diagnostic.position_residual_m == pytest.approx(
                reference_diagnostic.position_residual_m, abs=1e-12
            )
            assert diagnostic.orientation_residual_rad == pytest.approx(
                reference_diagnostic.orientation_residual_rad, abs=1e-12
            )
            if expected is None:
                assert actual is None
            else:
                assert actual == pytest.approx(expected, abs=1e-12)
        for (actual, report), (expected, reference_report) in zip(
            results[0][1], results[1][1], strict=True
        ):
            assert report is not None and reference_report is not None
            assert report.outcome == reference_report.outcome
            assert actual.joint_positions_rad == pytest.approx(
                expected.joint_positions_rad, abs=1e-10
            )
            assert actual.end_effector_pose.position_xyz_m == pytest.approx(
                expected.end_effector_pose.position_xyz_m, abs=1e-10
            )
    assert results[0][0][1][0] is not None  # recorded boundary converges
    assert results[0][0][2][0] is None  # infeasible pose still fails closed
    failed = results[0][0][2][1]
    assert failed is not None and not failed.converged
    assert failed.iterations == config.control.ik_max_iterations


def test_held_keyboard_speed_and_release_with_real_servo() -> None:
    config = SimulationConfig.load(CONFIG_PATH)
    keyboard = KeyboardTeleoperator(config.keyboard, config.control)
    keyboard.update_focus(True)
    keyboard.update_key("shift", True)
    keyboard.update_key("w", True)
    with make_driver(config) as driver:
        robot = SafeCartesianRobot(driver, config.control)
        observation = robot.reset(0)
        start = observation.robot.end_effector_pose.position_xyz_m
        for _ in range(50):
            robot.command(keyboard.poll(observation))
            observation = robot.observe()
        actual = observation.robot.end_effector_pose.position_xyz_m
        assert 0.07 < actual[0] - start[0] < 0.12
        keyboard.update_key("shift", False)
        robot.command(keyboard.poll(observation))
        assert robot.last_report is not None
        assert robot.last_report.outcome is CommandOutcome.DISABLED
        assert robot.last_report.commanded_cartesian_speed_m_s == 0.0
        for _ in range(25):
            robot.command(keyboard.poll(robot.observe()))
        assert math.dist(
            actual, robot.observe().robot.end_effector_pose.position_xyz_m
        ) < 0.005


@pytest.mark.parametrize("stale", [False, True])
def test_lifted_cube_survives_input_loss_and_resumed_transit(stale: bool) -> None:
    config = SimulationConfig.load(CONFIG_PATH)
    with make_driver(config) as driver:
        robot = SafeCartesianRobot(driver, config.control)
        observation = robot.reset(0)
        expert = ScriptedPickPlaceExpert(driver.environment, config.expert)
        expert.reset(observation)
        for _ in range(400):
            robot.command(expert.poll(observation))
            observation = robot.observe()
            if expert.phase is ExpertPhase.TRANSIT:
                break
        assert expert.phase is ExpertPhase.TRANSIT
        cube_z = driver.environment.task_state().cube_pose.position_xyz_m[2]
        assert cube_z > config.cube_center_z_m + 0.04
        held_pose = observation.robot.end_effector_pose
        # An untrusted open command must not remove the accepted squeeze.
        for _ in range(50):
            timestamp = observation.timestamp_ns
            if stale:
                timestamp -= config.control.watchdog_timeout_ns
            robot.command(Action(timestamp, held_pose, 1.0, enabled=stale))
            observation = robot.observe()
            assert robot.last_report is not None
            assert robot.last_report.outcome is (
                CommandOutcome.STALE if stale else CommandOutcome.DISABLED
            )
            assert math.dist(
                held_pose.position_xyz_m,
                observation.robot.end_effector_pose.position_xyz_m,
            ) < 0.005
            actual_z = driver.environment.task_state().cube_pose.position_xyz_m[2]
            assert actual_z > cube_z - 0.01
        # Resume arm motion while retaining the same closed-gripper intent.
        for _ in range(25):
            robot.command(expert.poll(observation))
            observation = robot.observe()
        actual_z = driver.environment.task_state().cube_pose.position_xyz_m[2]
        assert actual_z > cube_z - 0.01


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
    assert control.max_command_translation_velocity_m_s == 0.125
    assert control.max_translation_acceleration_m_s2 == 1.0
    assert control.max_orientation_velocity_rad_s == 1.0
    assert control.max_orientation_acceleration_rad_s2 == 4.0
    assert control.max_joint_velocity_rad_s == 1.0
    assert control.max_joint_acceleration_rad_s2 == 4.0
    assert control.joint_bound_margin_rad == 0.02
    assert control.max_gripper_velocity_s == 2.0
    assert control.dls_damping == 0.05
    assert control.ik_max_iterations == 200
    assert control.ik_position_tolerance_m == 0.0002
    assert control.ik_orientation_tolerance_rad == 0.001


def test_full_pose_converges_from_home_within_controller_tolerances() -> None:
    with make_driver() as driver:
        robot = SafeCartesianRobot(driver, driver.limits)
        state = robot.reset(11).robot
        home = state.end_effector_pose
        target = yaw_pose(
            home,
            0.05,
            (
                home.position_xyz_m[0] + 0.01,
                home.position_xyz_m[1] - 0.01,
                home.position_xyz_m[2] + 0.01,
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


def test_full_pose_converges_from_cube_approach_configuration() -> None:
    with make_driver() as driver:
        robot = SafeCartesianRobot(driver, driver.limits)
        state = robot.reset(0).robot
        initial = state.end_effector_pose
        assert initial.position_xyz_m[:2] == pytest.approx((0.45, -0.15), abs=2e-3)
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
        pose_response_time_s=0.02,
        max_translation_velocity_m_s=100.0,
        max_command_translation_velocity_m_s=100.0,
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
        colliding = Pose(
            "world",
            (0.58, 0.23, 0.44),
            initial.end_effector_pose.quaternion_wxyz,
        )
        robot.command(Action(initial.timestamp_ns, colliding, 1.0, enabled=True))
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


@pytest.mark.parametrize(
    ("seed", "joints", "target"),
    [
        (
            11012,
            (
                -0.07369870431418968, -2.0807724703288084,
                -0.008207701692829472, -2.565880797056834,
                1.5708500612021756, 4.637284856162966,
            ),
            Pose(
                "world",
                (0.5017479578803741, -0.17026410252981017, 0.6696524630226813),
                (0.0018384615222529774, 0.9996815821017917,
                 0.000517301363505509, 0.025161217478279155),
            ),
        ),
        (
            11062,
            (
                -0.06800706816545633, -2.0844288260283075,
                -0.0032443562893293395, -2.5665682788272344,
                1.57080205830434, 4.642866983807686,
            ),
            Pose(
                "world",
                (0.5035224252738114, -0.167588007918977, 0.6692764021907852),
                (0.001735264682359282, 0.9996760775537944,
                 0.0005671863487500999, 0.025385175256199214),
            ),
        ),
        (
            11012,
            (
                -0.0680864428716752, -2.0754632973328806,
                -0.0039667299758757, -2.590235198531238,
                1.5706164356295342, 4.643663109289246,
            ),
            Pose(
                "world",
                (0.49487097877431546, -0.16847114153586146, 0.6695548207747647),
                (0.0012749800659896478, 0.9998459139091115,
                 0.00025962382135293977, 0.017505869317356048),
            ),
        ),
    ],
)
def test_near_boundary_policy_targets_converge_with_revised_ik_config(
    seed: int, joints: tuple[float, ...], target: Pose
) -> None:
    """Previously rejected policy targets admit collision-free IK solutions."""
    config = SimulationConfig.load(CONFIG_PATH)
    old_control = replace(
        config.control, ik_position_tolerance_m=0.0001, ik_max_iterations=50
    )
    with make_driver(config) as driver:
        driver.reset(seed)
        for address, value in zip(driver._qpos_addresses, joints, strict=True):  # noqa: SLF001
            driver._data.qpos[address] = value  # noqa: SLF001
        mujoco.mj_forward(driver._model, driver._data)  # noqa: SLF001
        driver._config = replace(config, control=old_control)  # noqa: SLF001
        assert driver.solve_ik(target) is None
        driver._config = config  # noqa: SLF001
        candidate = driver.solve_ik(target)
        assert candidate is not None
        assert driver.collision_free(candidate, 1.0)


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
        cube_address = driver._model.jnt_qposadr[  # noqa: SLF001
            driver.environment._cube_joint_id  # noqa: SLF001
        ]
        driver._data.qpos[cube_address : cube_address + 3] = (  # noqa: SLF001
            0.8,
            0.35,
            config.cube_center_z_m,
        )
        mujoco.mj_forward(driver._model, driver._data)  # noqa: SLF001
        state = robot.observe().robot
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


def test_sustained_100_mm_target_meets_speed_and_progress_acceptance() -> None:
    with make_driver() as driver:
        robot = SafeCartesianRobot(driver, driver.limits)
        state = robot.reset(0).robot
        start = state.end_effector_pose.position_xyz_m
        target = Pose(
            "world",
            (start[0] - 0.1, start[1], start[2]),
            state.end_effector_pose.quaternion_wxyz,
        )
        maximum_commanded = 0.0
        maximum_measured = 0.0
        for _ in range(60):
            state = robot.command(Action(state.timestamp_ns, target, 0.0, True))
            report = robot.last_report
            assert report is not None
            maximum_commanded = max(
                maximum_commanded, report.commanded_cartesian_speed_m_s
            )
            maximum_measured = max(
                maximum_measured, report.measured_cartesian_speed_m_s
            )

    progress = start[0] - state.end_effector_pose.position_xyz_m[0]
    assert progress >= 0.09
    assert math.dist(
        state.end_effector_pose.position_xyz_m, target.position_xyz_m
    ) < 0.01
    assert maximum_commanded <= 0.125 + 1e-12
    assert maximum_measured <= 0.25 + 1e-9
