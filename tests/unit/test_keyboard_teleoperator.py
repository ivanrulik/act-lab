from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from act_lab.adapters.mujoco import KeyboardTeleoperator, SimulationConfig
from act_lab.domain import Observation, Pose, RobotState

CONFIG = SimulationConfig.load(Path("configs/sim/ur5e_pick_place.toml"))
POSE = Pose("world", (0.4, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0))


def observation(
    timestamp_ns: int = 0,
    pose: Pose = POSE,
    gripper: float = 0.5,
) -> Observation:
    state = RobotState(timestamp_ns, (0.0,) * 6, (0.0,) * 6, pose, gripper)
    return Observation(timestamp_ns, state, ())


@pytest.mark.parametrize(
    ("key", "delta"),
    [
        ("W", (0.01, 0.0, 0.0)),
        ("S", (-0.01, 0.0, 0.0)),
        ("A", (0.0, 0.01, 0.0)),
        ("D", (0.0, -0.01, 0.0)),
        ("R", (0.0, 0.0, 0.01)),
        ("F", (0.0, 0.0, -0.01)),
    ],
)
def test_every_translation_mapping_preserves_orientation(
    key: str, delta: tuple[float, float, float]
) -> None:
    teleoperator = KeyboardTeleoperator(CONFIG.keyboard, CONFIG.control)
    teleoperator.on_key(ord(key))

    action = teleoperator.poll(observation(20_000_000))

    assert action.target_pose.position_xyz_m == pytest.approx(
        tuple(a + b for a, b in zip(POSE.position_xyz_m, delta, strict=True))
    )
    assert action.target_pose.quaternion_wxyz == POSE.quaternion_wxyz
    assert action.timestamp_ns == 20_000_000
    assert action.enabled


def test_nudges_accumulate_and_gripper_keys_clip() -> None:
    teleoperator = KeyboardTeleoperator(CONFIG.keyboard, CONFIG.control)
    for key in "WWAOOC":
        teleoperator.on_key(ord(key.lower()))

    action = teleoperator.poll(observation(gripper=0.95))

    assert action.target_pose.position_xyz_m == pytest.approx((0.42, 0.01, 0.6))
    assert action.gripper_position == pytest.approx(0.9)
    assert teleoperator.accepted_event_count == 6
    assert teleoperator.latest_input == "C"


def test_workspace_clipping_unknown_key_stop_and_quit() -> None:
    limits = replace(CONFIG.control, workspace_x_m=(0.39, 0.405))
    teleoperator = KeyboardTeleoperator(CONFIG.keyboard, limits)
    teleoperator.on_key(ord("?"))
    unknown = teleoperator.poll(observation(10))
    assert not unknown.enabled
    assert teleoperator.accepted_event_count == 0
    assert teleoperator.latest_input == "none"

    teleoperator.on_key(ord("W"))
    teleoperator.on_key(ord("W"))
    assert teleoperator.poll(observation(20)).target_pose.position_xyz_m[0] == 0.405

    stopped_pose = Pose("world", (0.3, 0.2, 0.7), POSE.quaternion_wxyz)
    teleoperator.on_key(ord(" "))
    stopped = teleoperator.poll(observation(30, stopped_pose, 0.25))
    assert not stopped.enabled
    assert stopped.target_pose == stopped_pose
    assert stopped.gripper_position == 0.25
    assert teleoperator.latest_input == "Space (stop)"

    teleoperator.on_key(ord("Q"))
    quit_action = teleoperator.poll(observation(40, stopped_pose, 0.25))
    assert teleoperator.quit_requested
    assert not quit_action.enabled
    assert teleoperator.latest_input == "Q (quit)"


def test_uppercase_and_lowercase_events_have_identical_results() -> None:
    lowercase = KeyboardTeleoperator(CONFIG.keyboard, CONFIG.control)
    uppercase = KeyboardTeleoperator(CONFIG.keyboard, CONFIG.control)
    lowercase.on_key(ord("a"))
    uppercase.on_key(ord("A"))

    assert lowercase.poll(observation()) == uppercase.poll(observation())


def test_poll_without_new_event_retains_input_timestamp_for_watchdog() -> None:
    teleoperator = KeyboardTeleoperator(CONFIG.keyboard, CONFIG.control)
    teleoperator.on_key(ord("W"))
    first = teleoperator.poll(observation(20_000_000))
    repeated = teleoperator.poll(observation(100_000_000))

    assert repeated == first
