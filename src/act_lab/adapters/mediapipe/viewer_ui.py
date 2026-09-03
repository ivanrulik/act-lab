"""Testable presentation state for webcam teleoperation diagnostics."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from act_lab.adapters.mediapipe.config import WebcamConfig
from act_lab.adapters.mediapipe.teleoperator import TeleopDiagnostics
from act_lab.domain import CameraFrame, CommandOutcome, CommandReport, Observation

_ACCEPTED_RGBA = (0.10, 0.85, 1.00, 0.85)
_LIMITED_RGBA = (1.00, 0.65, 0.05, 0.90)
_REJECTED_RGBA = (1.00, 0.15, 0.15, 0.90)
_INACTIVE_RGBA = (0.55, 0.60, 0.65, 0.55)


@dataclass(frozen=True, slots=True)
class ViewerHud:
    banner: str
    lines: tuple[str, ...]
    target_xyz_m: tuple[float, float, float]
    actual_xyz_m: tuple[float, float, float]
    executed_target_xyz_m: tuple[float, float, float] | None
    target_rgba: tuple[float, float, float, float]
    show_target: bool


def build_viewer_hud(
    diagnostics: TeleopDiagnostics,
    report: CommandReport | None,
    observation: Observation,
    task_reason: str | None,
) -> ViewerHud:
    """Build display-only state from domain and teleoperator diagnostics."""
    actual = observation.robot.end_effector_pose.position_xyz_m
    if report is None:
        target = actual
        executed = None
        outcome = None
        detail = "waiting for first control step"
        gripper = diagnostics.commanded_gripper_position
        show_target = False
    else:
        requested = report.requested_action
        target = requested.target_pose.position_xyz_m
        executed = (
            report.executed_action.target_pose.position_xyz_m
            if report.executed_action is not None
            else None
        )
        outcome = report.outcome
        detail = report.detail
        gripper = requested.gripper_position
        show_target = requested.enabled

    banner = diagnostics.state.replace("_", " ").upper()
    if outcome is not None:
        banner = f"{banner} | {outcome.value.upper()}"
    offset_mm = tuple(value * 1000.0 for value in diagnostics.command_offset_xyz_m)
    pose_error_mm = tuple(
        (goal - measured) * 1000.0
        for goal, measured in zip(target, actual, strict=True)
    )
    lines = (
        "Enter: calibrate   Q: quit",
        f"COMMAND  X {_signed_mm(offset_mm[0])}  {_centered_bar(offset_mm[0])}",
        f"         Y {_signed_mm(offset_mm[1])}  {_centered_bar(offset_mm[1])}",
        f"         Z {_signed_mm(offset_mm[2])}  {_centered_bar(offset_mm[2])}",
        f"GRIPPER  {round(100.0 * (gripper or 0.0)):3d}%  {_level_bar(gripper or 0.0)}",
        "TARGET   " + _pose_text(target),
        "ACTUAL   " + _pose_text(actual),
        "ERROR    " + " ".join(_signed_mm(value) for value in pose_error_mm),
        f"CLUTCH   detected={diagnostics.clutch_detected} "
        f"active={diagnostics.clutch_active}",
        f"TRACKING {diagnostics.source_status} "
        f"{diagnostics.confidence or 0.0:.2f} "
        f"{diagnostics.frame_age_ms or 0.0:.0f}ms",
        f"DEPTH    {_depth_text(diagnostics.depth_ratio, config_dead_zone=None)}",
        f"SAFETY   {detail}",
        f"TASK     {task_reason or 'running'}",
    )
    return ViewerHud(
        banner=banner,
        lines=lines,
        target_xyz_m=target,
        actual_xyz_m=actual,
        executed_target_xyz_m=executed,
        target_rgba=_outcome_color(outcome),
        show_target=show_target,
    )


def draw_camera_hud(
    frame: CameraFrame,
    diagnostics: TeleopDiagnostics,
    config: WebcamConfig,
) -> NDArray[np.uint8]:
    """Draw the input anchor, dead zone, command vector, and state on RGB data."""
    import cv2

    image = (
        np.frombuffer(frame.rgb_bytes, dtype=np.uint8)
        .reshape(frame.height, frame.width, 3)
        .copy()
    )
    anchor = diagnostics.clutch_anchor_xy
    palm = diagnostics.palm_xy
    scale = diagnostics.clutch_anchor_scale
    if anchor is not None and scale is not None:
        anchor_px = _pixel(anchor, frame)
        half_width = max(3, round(config.image_xy_dead_zone * scale * frame.width))
        half_height = max(3, round(config.image_xy_dead_zone * scale * frame.height))
        cv2.rectangle(
            image,
            (anchor_px[0] - half_width, anchor_px[1] - half_height),
            (anchor_px[0] + half_width, anchor_px[1] + half_height),
            (255, 210, 40),
            2,
        )
        cv2.drawMarker(
            image,
            anchor_px,
            (255, 210, 40),
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=2,
        )
        if palm is not None:
            cv2.arrowedLine(
                image,
                anchor_px,
                _pixel(palm, frame),
                (30, 220, 255),
                3,
                tipLength=0.18,
            )
    x_mm, y_mm, z_mm = (value * 1000.0 for value in diagnostics.command_offset_xyz_m)
    cv2.rectangle(image, (0, 0), (frame.width, 74), (15, 18, 24), -1)
    cv2.putText(
        image,
        diagnostics.state.replace("_", " ").upper(),
        (10, 21),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (30, 220, 255) if diagnostics.clutch_active else (200, 205, 210),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        f"X {x_mm:+.0f}  Y {y_mm:+.0f}  Z {z_mm:+.0f} mm",
        (10, 44),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (235, 235, 235),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        "depth " + _depth_text(diagnostics.depth_ratio, config.depth_dead_zone),
        (10, 65),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (235, 235, 235),
        1,
        cv2.LINE_AA,
    )
    return image


def _outcome_color(
    outcome: CommandOutcome | None,
) -> tuple[float, float, float, float]:
    if outcome is CommandOutcome.APPLIED:
        return _ACCEPTED_RGBA
    if outcome is CommandOutcome.LIMITED:
        return _LIMITED_RGBA
    if outcome in {
        CommandOutcome.STALE,
        CommandOutcome.INVALID,
        CommandOutcome.IK_FAILURE,
        CommandOutcome.COLLISION_STOP,
    }:
        return _REJECTED_RGBA
    return _INACTIVE_RGBA


def _centered_bar(value_mm: float, limit_mm: float = 150.0) -> str:
    slots = 7
    position = min(slots, round(abs(value_mm) / limit_mm * slots))
    left = ["."] * slots
    right = ["."] * slots
    if value_mm < 0.0:
        for index in range(position):
            left[slots - index - 1] = "<"
    elif value_mm > 0.0:
        for index in range(position):
            right[index] = ">"
    return "[" + "".join(left) + "|" + "".join(right) + "]"


def _level_bar(value: float) -> str:
    filled = min(10, max(0, round(value * 10)))
    return "[" + "#" * filled + "." * (10 - filled) + "]"


def _signed_mm(value: float) -> str:
    return f"{value:+04.0f}mm"


def _depth_text(ratio: float | None, config_dead_zone: float | None) -> str:
    if ratio is None:
        return "no clutch anchor"
    change = (ratio - 1.0) * 100.0
    suffix = (
        f"  dead zone +/-{config_dead_zone * 100.0:.0f}%"
        if config_dead_zone is not None
        else ""
    )
    return f"{ratio:.3f}x ({change:+.1f}%){suffix}"


def _pose_text(position: tuple[float, float, float]) -> str:
    return " ".join(
        f"{axis}={value:+.3f}" for axis, value in zip("XYZ", position, strict=True)
    )


def _pixel(point: tuple[float, float], frame: CameraFrame) -> tuple[int, int]:
    return (
        min(frame.width - 1, max(0, round(point[0] * frame.width))),
        min(frame.height - 1, max(0, round(point[1] * frame.height))),
    )
