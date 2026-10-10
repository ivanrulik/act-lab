"""Model changes cannot silently cross the recording/checkpoint boundary."""

import json
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import pytest

from act_lab.adapters.lerobot.policy import LeRobotACTPolicy
from act_lab.adapters.mujoco.cartesian_driver import MujocoCartesianDriver
from act_lab.application.model_compatibility import model_contract, require_compatible

CONFIG = Path("configs/sim/ur5e_2f85_d405.toml")


def test_real_calibration_identity_rejects_changes_and_legacy_checkpoint(tmp_path):
    with MujocoCartesianDriver.from_config_file(CONFIG) as driver:
        env = driver.environment
        env.reset(0)
        resolved = dict(
            simulation=asdict(driver.simulation_config),
            model_identity=env.model_identity,
            scene_cameras={
                c: env.camera_calibration(c) for c in driver.simulation_config.cameras
            },
        )
        contract = model_contract(resolved)
        assert contract is not None and "wrist" in contract["cameras"]
        require_compatible(contract, json.loads(json.dumps(contract)))
        with pytest.raises(ValueError, match="incompatible"):
            LeRobotACTPolicy(tmp_path, "cpu", ("wrist",), model_contract=contract)
        altered = json.loads(json.dumps(resolved))
        altered["scene_cameras"]["wrist"]["k"][0] += 1
        with pytest.raises(ValueError, match="fingerprint"):
            model_contract(altered)
        policy = LeRobotACTPolicy.__new__(LeRobotACTPolicy)
        policy._model_contract = contract
        obs = env.observe()
        images = {
            c: np.zeros(p["shape"], dtype=np.uint8)
            for c, p in contract["cameras"].items()
        }
        # These failures return disabled intent before touching any ACT framework.
        cases = (
            (images, None, "missing_camera_timestamp"),
            ({}, obs.timestamp_ns, "missing_or_mismatched_camera"),
            (images, obs.timestamp_ns + 1, "stale_or_future_camera"),
        )
        for frames, stamp, reason in cases:
            action = policy.act(obs, frames, capture_timestamp_ns=stamp)
            assert (
                not action.enabled and action.target_pose == obs.robot.end_effector_pose
            )
            assert policy.input_rejection == reason
        later = replace(obs, timestamp_ns=obs.timestamp_ns + 100_000_000)
        assert not policy.act(
            later, images, capture_timestamp_ns=obs.timestamp_ns
        ).enabled
