from __future__ import annotations

import math
import time
from dataclasses import replace
from pathlib import Path

import pytest

from act_lab.adapters.mediapipe import HandSignal, WebcamConfig, WebcamTeleoperator
from act_lab.domain import CameraFrame, Observation, Pose, RobotState

CONFIG = WebcamConfig.load(Path("configs/teleop/webcam.toml"))
TEST_CONFIG = replace(
    CONFIG,
    calibration_sample_frames=2,
    clutch_engage_frames=2,
    image_xy_dead_zone=0.0,
    depth_dead_zone=0.0,
    ema_alpha=1.0,
    depth_ema_alpha=1.0,
)
POSE = Pose("world", (0.4, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0))
FRAME = CameraFrame(0, 1, 1, b"\0\0\0")


def observation(timestamp_ns: int, pose: Pose = POSE) -> Observation:
    state = RobotState(timestamp_ns, (0.0,) * 6, (0.0,) * 6, pose, 0.5)
    return Observation(timestamp_ns, state, ())


def signal(
    *,
    x: float = 0.5,
    y: float = 0.5,
    scale: float = 0.2,
    pinch: float = 0.5,
    clutch: bool = True,
    handedness: str = "Right",
) -> HandSignal:
    return HandSignal(0, handedness, 0.95, x, y, scale, pinch, clutch, FRAME)


def submit(
    teleoperator: WebcamTeleoperator,
    sample: HandSignal | None,
    timestamp_ns: int,
) -> object:
    teleoperator.update(
        sample,
        time.monotonic_ns(),
        "tracked" if sample is not None else "tracking_lost",
    )
    return teleoperator.poll(observation(timestamp_ns))


def calibrate(teleoperator: WebcamTeleoperator) -> None:
    teleoperator.request_calibration()
    assert not submit(teleoperator, signal(x=0.499), 0).enabled
    assert not submit(teleoperator, signal(x=0.501), 20_000_000).enabled
    assert teleoperator.calibration_snapshot is not None


def engage(teleoperator: WebcamTeleoperator) -> None:
    assert not submit(teleoperator, signal(), 40_000_000).enabled
    assert submit(teleoperator, signal(), 60_000_000).enabled


def test_calibration_clutch_mapping_and_pinch() -> None:
    teleoperator = WebcamTeleoperator(TEST_CONFIG)
    calibrate(teleoperator)
    engage(teleoperator)

    action = submit(
        teleoperator,
        signal(x=0.6, y=0.4, scale=0.22, pinch=0.25),
        80_000_000,
    )

    assert action.enabled
    assert action.target_pose.position_xyz_m == pytest.approx(
        (0.5, -0.075, 0.6 - 0.15 * math.log(1.1))
    )
    assert action.target_pose.quaternion_wxyz == POSE.quaternion_wxyz
    assert action.gripper_position == 0.0
    assert action.timestamp_ns == 80_000_000


def test_depth_ratios_are_symmetric_and_filtered_independently() -> None:
    toward = WebcamTeleoperator(replace(TEST_CONFIG, depth_ema_alpha=0.1))
    calibrate(toward)
    engage(toward)
    toward_action = submit(toward, signal(x=0.6, scale=0.4), 80_000_000)

    away = WebcamTeleoperator(replace(TEST_CONFIG, depth_ema_alpha=0.1))
    calibrate(away)
    engage(away)
    away_action = submit(away, signal(x=0.6, scale=0.1), 80_000_000)

    toward_z = toward_action.target_pose.position_xyz_m[2] - POSE.position_xyz_m[2]
    away_z = away_action.target_pose.position_xyz_m[2] - POSE.position_xyz_m[2]
    assert toward_z == pytest.approx(-away_z)
    # Depth uses its own alpha while screen-horizontal motion remains immediate.
    assert toward_z == pytest.approx(0.1 * CONFIG.world_z_gain_m * math.log(2.0))
    assert toward_action.target_pose.position_xyz_m[1] == pytest.approx(-0.075)


def test_clutch_release_disables_and_reengagement_reanchors_without_jump() -> None:
    teleoperator = WebcamTeleoperator(TEST_CONFIG)
    calibrate(teleoperator)
    engage(teleoperator)
    submit(teleoperator, signal(x=0.6), 80_000_000)

    released = submit(teleoperator, signal(x=0.7, clutch=False), 100_000_000)
    assert not released.enabled
    moved_pose = Pose("world", (0.42, -0.04, 0.6), POSE.quaternion_wxyz)
    teleoperator.update(signal(x=0.7), time.monotonic_ns(), "tracked")
    assert not teleoperator.poll(observation(120_000_000, moved_pose)).enabled
    teleoperator.update(signal(x=0.7), time.monotonic_ns(), "tracked")
    reengaged = teleoperator.poll(observation(140_000_000, moved_pose))
    assert reengaged.enabled
    assert reengaged.target_pose == moved_pose


def test_tracking_loss_disables_and_retains_last_valid_timestamp() -> None:
    teleoperator = WebcamTeleoperator(TEST_CONFIG)
    calibrate(teleoperator)
    engage(teleoperator)
    active = submit(teleoperator, signal(), 80_000_000)

    lost = submit(teleoperator, None, 100_000_000)

    assert not lost.enabled
    assert lost.timestamp_ns == active.timestamp_ns
    assert teleoperator.diagnostics.state == "tracking_lost"
    assert teleoperator.diagnostics.tracking_losses == 1


def test_wrong_hand_and_nonfinite_signal_fail_closed() -> None:
    teleoperator = WebcamTeleoperator(TEST_CONFIG)
    calibrate(teleoperator)
    wrong = submit(teleoperator, signal(handedness="Left"), 40_000_000)
    invalid = submit(teleoperator, signal(scale=float("nan")), 60_000_000)
    low_confidence = replace(signal(), confidence=0.2)
    low = submit(teleoperator, low_confidence, 80_000_000)
    assert not wrong.enabled
    assert not invalid.enabled
    assert not low.enabled
    assert teleoperator.diagnostics.state == "low_confidence"


def test_old_camera_frame_fails_closed() -> None:
    teleoperator = WebcamTeleoperator(TEST_CONFIG)
    calibrate(teleoperator)
    teleoperator.update(
        signal(),
        time.monotonic_ns() - (TEST_CONFIG.max_frame_age_ms + 1) * 1_000_000,
        "tracked",
    )
    action = teleoperator.poll(observation(40_000_000))
    assert not action.enabled
    assert teleoperator.diagnostics.state == "stale_camera_frame"


def test_poll_without_new_frame_does_not_refresh_action_timestamp() -> None:
    teleoperator = WebcamTeleoperator(TEST_CONFIG)
    calibrate(teleoperator)
    engage(teleoperator)
    active = submit(teleoperator, signal(), 80_000_000)
    assert teleoperator.poll(observation(160_000_000)) == active


def test_enter_calibrates_without_using_mujoco_reserved_c_key() -> None:
    teleoperator = WebcamTeleoperator(TEST_CONFIG)

    teleoperator.on_key(ord("C"))
    assert teleoperator.diagnostics.state == "uncalibrated"

    teleoperator.on_key(257)  # GLFW_KEY_ENTER
    assert teleoperator.diagnostics.state == "calibrating"
