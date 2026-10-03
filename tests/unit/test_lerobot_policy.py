"""Framework adapter behavior that does not require LeRobot or a GPU."""

import pytest

from act_lab.adapters.lerobot.policy import _bounded_gripper_position


@pytest.mark.parametrize(
    ("value", "expected"),
    [(-0.02, 0.0), (0.0, 0.0), (0.4, 0.4), (1.0, 1.0), (1.01, 1.0)],
)
def test_gripper_prediction_is_bounded_to_actuator_range(
    value: float, expected: float
) -> None:
    assert _bounded_gripper_position(value) == expected
