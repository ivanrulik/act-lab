"""Reproduce the cube spawn coordinates without constructing a MuJoCo scene."""

from __future__ import annotations

import numpy as np

from act_lab.adapters.mujoco.config import SimulationConfig


def cube_spawn_xy(seed: int, config: SimulationConfig) -> tuple[float, float]:
    """Use the same first two random draws as environment.reset."""
    if isinstance(seed, bool):
        raise ValueError("seed must be an integer")
    rng = np.random.default_rng(seed)
    return (
        float(rng.uniform(*config.cube_spawn_x_m)),
        float(rng.uniform(*config.cube_spawn_y_m)),
    )
