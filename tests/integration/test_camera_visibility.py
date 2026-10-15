"""Rendered segmentation stays a read-only diagnostic on the fixed-step scene."""

from pathlib import Path

import numpy as np

from act_lab.adapters.mujoco import MujocoUR5eEnvironment, SimulationConfig


def test_segmentation_counts_real_geoms_without_mutating_state() -> None:
    config = SimulationConfig.load(Path("configs/sim/ur5e_2f85_d405_paired.toml"))
    with MujocoUR5eEnvironment(config) as env:
        env.reset(19000)
        before_state = env._data.qpos.copy()
        before_velocity = env._data.qvel.copy()
        before_time = env.observe().timestamp_ns
        before_image = env.render("policy")
        pixels = env.visible_task_pixels("policy")
        assert pixels["cube"] >= 16 and pixels["tray"] >= 16
        np.testing.assert_array_equal(before_image, env.render("policy"))
        np.testing.assert_array_equal(before_state, env._data.qpos)
        np.testing.assert_array_equal(before_velocity, env._data.qvel)
        assert env.observe().timestamp_ns == before_time
        assert env.visible_task_pixels("policy") == pixels
