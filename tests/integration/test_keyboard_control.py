from pathlib import Path

from act_lab.adapters.mujoco import KeyboardTeleoperator, MujocoCartesianDriver
from act_lab.application import SafeCartesianRobot
from act_lab.domain import CommandOutcome

CONFIG_PATH = Path("configs/sim/ur5e_pick_place.toml")


def test_deadman_release_holds_on_next_control_cycle() -> None:
    with MujocoCartesianDriver.from_config_file(CONFIG_PATH) as driver:
        robot = SafeCartesianRobot(driver, driver.limits)
        observation = robot.reset(0)
        keyboard = KeyboardTeleoperator(
            driver.simulation_config.keyboard, driver.limits
        )
        keyboard.update_focus(True)
        keyboard.update_key("shift", True)
        keyboard.update_key("w", True)
        for _ in range(8):
            robot.command(keyboard.poll(observation))
            observation = robot.observe()
        keyboard.update_key("shift", False)
        measured = observation.robot.end_effector_pose.position_xyz_m
        robot.command(keyboard.poll(observation))

        assert robot.last_report is not None
        assert robot.last_report.outcome is CommandOutcome.DISABLED
        assert robot.last_report.executed_action is None
        assert robot.last_report.requested_action.target_pose.position_xyz_m == measured
