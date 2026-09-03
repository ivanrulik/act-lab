from __future__ import annotations

import time
from dataclasses import replace
from pathlib import Path

from act_lab.adapters.mediapipe import HandSignal, WebcamConfig, WebcamTeleoperator
from act_lab.adapters.mujoco import MujocoCartesianDriver
from act_lab.application import SafeCartesianRobot
from act_lab.domain import CameraFrame, CommandOutcome

SIM_CONFIG = Path("configs/sim/ur5e_pick_place.toml")
FRAME = CameraFrame(0, 1, 1, b"\0\0\0")


def test_tracking_loss_reaches_watchdog_boundary_within_five_steps() -> None:
    webcam_config = replace(
        WebcamConfig.load(Path("configs/teleop/webcam.toml")),
        calibration_sample_frames=1,
        clutch_engage_frames=1,
    )
    signal = HandSignal(
        source_timestamp_ns=0,
        handedness="Right",
        confidence=0.99,
        palm_x=0.5,
        palm_y=0.5,
        palm_scale=0.2,
        apparent_scale=0.2,
        pinch_ratio=0.5,
        clutch=True,
        preview=FRAME,
    )
    with MujocoCartesianDriver.from_config_file(SIM_CONFIG) as driver:
        robot = SafeCartesianRobot(driver, driver.limits)
        observation = robot.reset(0)
        teleoperator = WebcamTeleoperator(webcam_config)
        teleoperator.request_calibration()
        teleoperator.update(signal, time.monotonic_ns(), "tracked")
        robot.command(teleoperator.poll(observation))
        observation = robot.observe()
        teleoperator.update(signal, time.monotonic_ns(), "tracked")
        active = teleoperator.poll(observation)
        robot.command(active)
        observation = robot.observe()

        teleoperator.update(None, time.monotonic_ns(), "tracking_lost")
        lost = teleoperator.poll(observation)
        outcomes = []
        for _ in range(6):
            robot.command(lost)
            assert robot.last_report is not None
            outcomes.append(robot.last_report.outcome)

    assert outcomes[0] is CommandOutcome.DISABLED
    assert CommandOutcome.STALE in outcomes
    assert outcomes.index(CommandOutcome.STALE) <= 5
