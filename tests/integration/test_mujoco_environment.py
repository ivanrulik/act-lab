from __future__ import annotations

import hashlib
import math
from importlib.resources import files
from pathlib import Path

import mujoco
import numpy as np
import pytest

from act_lab.adapters.mujoco import ActuatorTargets, MujocoUR5eEnvironment
from act_lab.adapters.mujoco.config import SimulationConfig

CONFIG_PATH = Path("configs/sim/ur5e_pick_place.toml")


def make_environment() -> MujocoUR5eEnvironment:
    return MujocoUR5eEnvironment.from_config_file(CONFIG_PATH)


def test_seeded_reset_and_rollout_are_deterministic() -> None:
    with make_environment() as environment:
        first = environment.reset(123)
        first_cube = environment.cube_position_xyz_m
        first_rollout = [environment.step().robot for _ in range(10)]

        second = environment.reset(123)
        second_cube = environment.cube_position_xyz_m
        second_rollout = [environment.step().robot for _ in range(10)]

    assert first == second
    assert first_cube == second_cube
    assert first_rollout == second_rollout


def test_different_seeds_stay_in_documented_spawn_bounds() -> None:
    config = SimulationConfig.load(CONFIG_PATH)
    with make_environment() as environment:
        environment.reset(1)
        first = environment.cube_position_xyz_m
        environment.reset(2)
        second = environment.cube_position_xyz_m

    assert first != second
    for position in (first, second):
        assert config.cube_spawn_x_m[0] <= position[0] <= config.cube_spawn_x_m[1]
        assert config.cube_spawn_y_m[0] <= position[1] <= config.cube_spawn_y_m[1]
        assert position[2] == config.cube_center_z_m


def test_fixed_step_clock_and_observation_contract() -> None:
    config = SimulationConfig.load(CONFIG_PATH)
    with make_environment() as environment:
        initial = environment.reset(7)
        observation = environment.step()

    assert initial.timestamp_ns == 0
    assert environment.physics_ticks == config.substeps
    assert observation.timestamp_ns == 20_000_000
    assert len(observation.robot.joint_positions_rad) == 6
    assert len(observation.robot.joint_velocities_rad_s) == 6
    assert observation.robot.end_effector_pose.frame_id == "world"
    assert math.isclose(
        sum(value**2 for value in observation.robot.end_effector_pose.quaternion_wxyz),
        1.0,
        abs_tol=1e-9,
    )
    assert observation.image_keys == ("overview", "policy")


def test_reset_clears_prior_episode_state() -> None:
    with make_environment() as environment:
        expected = environment.reset(9)
        for _ in range(20):
            environment.step(ActuatorTargets((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0))
        actual = environment.reset(9)

    assert actual == expected
    assert environment.physics_ticks == 0


def test_invalid_targets_and_camera_fail_actionably() -> None:
    with make_environment() as environment:
        environment.reset(0)
        with pytest.raises(ValueError, match="finite"):
            environment.step(
                ActuatorTargets((math.nan, 0.0, 0.0, 0.0, 0.0, 0.0), 0.0)
            )
        with pytest.raises(ValueError, match="unknown camera"):
            environment.render("missing")


def test_cube_must_settle_in_tray_before_success() -> None:
    config = SimulationConfig.load(CONFIG_PATH)
    with make_environment() as environment:
        environment.reset(0)
        assert not environment.task_status().success
        environment._data.joint("cube_free_joint").qpos[:3] = (  # noqa: SLF001
            config.tray_center_xy_m[0],
            config.tray_center_xy_m[1],
            config.tray_floor_top_z_m + config.cube_half_extent_m,
        )
        environment._data.joint("cube_free_joint").qvel[:] = 0.0  # noqa: SLF001
        mujoco.mj_forward(environment._model, environment._data)  # noqa: SLF001
        for _ in range(config.settling_steps - 1):
            environment.step()
        assert not environment.task_status().success
        environment.step()
        status = environment.task_status()

    assert status.success
    assert status.terminal
    assert status.reason == "success"


def test_privileged_task_state_stays_separate_from_observation() -> None:
    config = SimulationConfig.load(CONFIG_PATH)
    with make_environment() as environment:
        observation = environment.reset(5)
        task = environment.task_state()

    assert not hasattr(observation, "cube_pose")
    assert task.cube_pose.position_xyz_m == environment.cube_position_xyz_m
    assert task.desired_cube_pose.position_xyz_m == (
        config.tray_center_xy_m[0],
        config.tray_center_xy_m[1],
        config.tray_floor_top_z_m + config.cube_half_extent_m,
    )
    assert not task.terminal


def test_long_headless_rollout_stays_finite() -> None:
    with make_environment() as environment:
        environment.reset(42)
        for _ in range(250):
            observation = environment.step()

    state = (
        *observation.robot.joint_positions_rad,
        *observation.robot.joint_velocities_rad_s,
    )
    assert all(math.isfinite(value) for value in state)


def test_headless_cameras_render_rgb_frames() -> None:
    with make_environment() as environment:
        environment.reset(4)
        first_overview = environment.render("overview")
        for camera in ("overview", "policy"):
            frame = environment.render(camera)
            assert frame.shape == (240, 320, 3)
            assert frame.dtype == np.uint8
            assert float(frame.std()) > 1.0
        environment.reset(4)
        assert np.array_equal(first_overview, environment.render("overview"))


def test_configuration_rejects_non_integral_clock_ratio(tmp_path: Path) -> None:
    invalid = tmp_path / "invalid.toml"
    contents = CONFIG_PATH.read_text().replace(
        "environment_hz = 50", "environment_hz = 60"
    )
    invalid.write_text(contents)

    with pytest.raises(ValueError, match="integer multiple"):
        SimulationConfig.load(invalid)


def test_vendored_meshes_match_manifest() -> None:
    root = files("act_lab.adapters.mujoco").joinpath("assets", "ur5e")
    manifest = root.joinpath("ASSET_MANIFEST.sha256").read_text().splitlines()
    for entry in manifest:
        expected, relative_path = entry.split(maxsplit=1)
        contents = root.joinpath(relative_path).read_bytes()
        assert hashlib.sha256(contents).hexdigest() == expected
