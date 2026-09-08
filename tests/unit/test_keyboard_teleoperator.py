from __future__ import annotations

import math
from pathlib import Path

import pytest

from act_lab.adapters.mujoco import KeyboardTeleoperator, SimulationConfig
from act_lab.domain import Observation, Pose, RobotState

CONFIG = SimulationConfig.load(Path("configs/sim/ur5e_pick_place.toml"))
POSE = Pose("world", (0.4, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0))


def observation(
    timestamp_ns: int = 0, pose: Pose = POSE, gripper: float = 0.5
) -> Observation:
    state = RobotState(timestamp_ns, (0.0,) * 6, (0.0,) * 6, pose, gripper)
    return Observation(timestamp_ns, state, ())


def active(*keys: str) -> KeyboardTeleoperator:
    keyboard = KeyboardTeleoperator(CONFIG.keyboard, CONFIG.control)
    keyboard.update_focus(True)
    keyboard.update_key("shift", True)
    for key in keys:
        keyboard.update_key(key, True)
    return keyboard


@pytest.mark.parametrize(
    ("key", "axis", "sign"),
    [
        ("w", 0, 1),
        ("s", 0, -1),
        ("a", 1, 1),
        ("d", 1, -1),
        ("r", 2, 1),
        ("f", 2, -1),
    ],
)
def test_held_translation_is_continuous(
    key: str, axis: int, sign: int
) -> None:
    keyboard = active(key)
    first = keyboard.poll(observation(20_000_000))
    second = keyboard.poll(observation(40_000_000))
    expected = sign * CONFIG.keyboard.translation_speed_m_s / 50.0
    assert first.enabled and second.enabled
    assert first.target_pose.position_xyz_m[axis] == pytest.approx(
        POSE.position_xyz_m[axis] + expected
    )
    assert first.target_pose.quaternion_wxyz == POSE.quaternion_wxyz
    assert second.timestamp_ns == 40_000_000


def test_diagonal_input_is_normalized() -> None:
    action = active("w", "a", "r").poll(observation())
    displacement = math.dist(action.target_pose.position_xyz_m, POSE.position_xyz_m)
    assert displacement == pytest.approx(CONFIG.keyboard.translation_speed_m_s / 50)


def test_deadman_motion_release_and_focus_loss_disable_next_poll() -> None:
    keyboard = active("w")
    assert keyboard.poll(observation()).enabled
    keyboard.update_key("shift", False)
    released = keyboard.poll(observation(20_000_000))
    assert not released.enabled
    assert released.target_pose == POSE
    keyboard.update_key("shift", True)
    keyboard.update_key("w", True)
    keyboard.update_focus(False)
    unfocused = keyboard.poll(observation(40_000_000))
    assert not unfocused.enabled
    assert "focus lost" in keyboard.latest_input


def test_gripper_and_quit_controls() -> None:
    keyboard = active("o")
    assert keyboard.poll(observation()).gripper_position > 0.5
    keyboard.update_key("q", True)
    assert keyboard.quit_requested


def test_closed_gripper_target_is_latched_across_contact_deflection() -> None:
    keyboard = active("c")
    closed = keyboard.poll(observation()).gripper_position
    keyboard.update_key("c", False)
    keyboard.update_key("shift", False)
    deflected_state = observation(20_000_000, gripper=closed + 0.1)
    assert not keyboard.poll(deflected_state).enabled
    keyboard.update_key("shift", True)
    keyboard.update_key("w", True)
    resumed = keyboard.poll(deflected_state)
    assert resumed.enabled
    assert resumed.gripper_position == pytest.approx(closed)
