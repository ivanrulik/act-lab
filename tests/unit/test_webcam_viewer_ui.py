from __future__ import annotations

from pathlib import Path

import numpy as np

from act_lab.adapters.mediapipe import WebcamConfig
from act_lab.adapters.mediapipe.teleoperator import TeleopDiagnostics
from act_lab.adapters.mediapipe.viewer_ui import build_viewer_hud, draw_camera_hud
from act_lab.domain import (
    Action,
    CameraFrame,
    CommandOutcome,
    CommandReport,
    Observation,
    Pose,
    RobotState,
)


def diagnostics(**changes: object) -> TeleopDiagnostics:
    values: dict[str, object] = {
        "state": "active",
        "source_status": "tracked",
        "frames_seen": 20,
        "tracking_losses": 0,
        "clutch_transitions": 1,
        "calibration_samples": 0,
        "handedness": "Right",
        "confidence": 0.95,
        "frame_age_ms": 12.0,
        "clutch_detected": True,
        "clutch_active": True,
        "calibration_id": "calibration",
        "palm_xy": (0.6, 0.4),
        "clutch_anchor_xy": (0.5, 0.5),
        "clutch_anchor_scale": 0.2,
        "command_offset_xyz_m": (0.042, -0.008, 0.003),
        "commanded_gripper_position": 0.46,
    }
    values.update(changes)
    return TeleopDiagnostics(**values)  # type: ignore[arg-type]


def observation() -> Observation:
    pose = Pose("world", (0.4, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0))
    state = RobotState(20, (0.0,) * 6, (0.0,) * 6, pose, 0.4)
    return Observation(20, state, ())


def test_hud_exposes_generated_command_and_safety_result() -> None:
    observed = observation()
    target = Pose("world", (0.442, -0.008, 0.603), (1.0, 0.0, 0.0, 0.0))
    requested = Action(20, target, 0.46, True)
    report = CommandReport(
        CommandOutcome.LIMITED,
        requested,
        requested,
        observed.robot,
        "workspace clamp",
    )

    hud = build_viewer_hud(diagnostics(), report, observed, None)

    assert hud.banner == "ACTIVE | LIMITED"
    assert hud.show_target
    assert hud.target_xyz_m == target.position_xyz_m
    assert any("X +042mm" in line for line in hud.lines)
    assert any("GRIPPER   46%" in line for line in hud.lines)
    assert any("workspace clamp" in line for line in hud.lines)


def test_camera_hud_draws_anchor_dead_zone_and_command_text() -> None:
    frame = CameraFrame(1, 320, 240, bytes(320 * 240 * 3))

    image = draw_camera_hud(
        frame,
        diagnostics(),
        WebcamConfig.load(Path("configs/teleop/webcam.toml")),
    )

    assert image.shape == (240, 320, 3)
    assert image.dtype == np.uint8
    assert np.count_nonzero(image) > 0
