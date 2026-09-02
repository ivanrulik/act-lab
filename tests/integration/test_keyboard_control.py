from pathlib import Path

from act_lab.adapters.mujoco import (
    KeyboardTeleoperator,
    MujocoCartesianDriver,
    SimulationConfig,
)
from act_lab.application import SafeCartesianRobot
from act_lab.domain import CommandOutcome

CONFIG_PATH = Path("configs/sim/ur5e_pick_place.toml")


def test_injected_key_uses_safe_robot_and_stops_at_watchdog_boundary() -> None:
    config = SimulationConfig.load(CONFIG_PATH)
    with MujocoCartesianDriver.from_config_file(CONFIG_PATH) as driver:
        robot = SafeCartesianRobot(driver, driver.limits)
        observation = robot.reset(0)
        keyboard = KeyboardTeleoperator(config.keyboard, config.control)
        initial_x = observation.robot.end_effector_pose.position_xyz_m[0]
        keyboard.on_key(ord("W"))
        action = keyboard.poll(observation)

        outcomes = []
        positions = []
        for _ in range(6):
            state = robot.command(action)
            assert robot.last_report is not None
            outcomes.append(robot.last_report.outcome)
            positions.append(state.end_effector_pose.position_xyz_m[0])

    assert outcomes[:5] == [CommandOutcome.LIMITED] * 5
    assert outcomes[5] is CommandOutcome.STALE
    assert max(positions) > initial_x
    assert positions[-1] < action.target_pose.position_xyz_m[0]
