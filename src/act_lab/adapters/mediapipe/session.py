"""Run webcam teleoperation through the MuJoCo safe-control path."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

import numpy as np

from act_lab.adapters.mediapipe.teleoperator import WebcamTeleoperator
from act_lab.adapters.mediapipe.worker import HandTrackingWorker
from act_lab.domain import CommandOutcome, Observation

if TYPE_CHECKING:
    from act_lab.adapters.mujoco import MujocoCartesianDriver
    from act_lab.application import SafeCartesianRobot


def run_webcam_session(
    driver: MujocoCartesianDriver,
    robot: SafeCartesianRobot,
    teleoperator: WebcamTeleoperator,
    worker: HandTrackingWorker,
    seed: int,
    *,
    headless: bool,
    auto_calibrate: bool,
    max_steps: int | None,
) -> dict[str, object]:
    if max_steps is not None and max_steps <= 0:
        raise ValueError("max_steps must be positive when provided")
    if auto_calibrate:
        teleoperator.request_calibration()
    observation = robot.reset(seed)
    counts = {outcome.value: 0 for outcome in CommandOutcome}
    completed_steps = 0
    termination = "step_limit"
    worker.start()
    try:
        if headless:
            limit = (
                max_steps
                if max_steps is not None
                else driver.simulation_config.episode_steps
            )
            while completed_steps < limit:
                started_at = time.monotonic()
                observation = _control_step(
                    robot, teleoperator, observation, counts
                )
                completed_steps += 1
                diagnostics = teleoperator.diagnostics
                if diagnostics.source_status.startswith("worker_failure"):
                    termination = "worker_failure"
                    break
                if diagnostics.source_status == "end_of_stream":
                    termination = "end_of_stream"
                    # Execute through the watchdog boundary before exiting.
                    if (
                        robot.last_report is not None
                        and robot.last_report.outcome.value == "stale"
                    ):
                        break
                _pace(started_at, driver.control_period_s)
        else:
            completed_steps, termination = _run_viewer(
                driver,
                robot,
                teleoperator,
                observation,
                counts,
                max_steps,
            )
    finally:
        worker.stop()
    diagnostics = teleoperator.diagnostics
    report: dict[str, object] = {
        "calibration": teleoperator.calibration_snapshot,
        "environment_steps": completed_steps,
        "frames_seen": diagnostics.frames_seen,
        "tracking_losses": diagnostics.tracking_losses,
        "clutch_transitions": diagnostics.clutch_transitions,
        "safety_outcomes": counts,
        "source_status": diagnostics.source_status,
        "teleop_state": diagnostics.state,
        "termination_reason": termination,
    }
    if worker.failure is not None:
        raise RuntimeError(f"hand tracking worker failed: {worker.failure}")
    return report


def _control_step(
    robot: SafeCartesianRobot,
    teleoperator: WebcamTeleoperator,
    observation: Observation,
    counts: dict[str, int],
) -> Observation:
    action = teleoperator.poll(observation)
    robot.command(action)
    report = robot.last_report
    if report is None:
        raise RuntimeError("safe controller did not produce a report")
    counts[report.outcome.value] += 1
    return robot.observe()


def _run_viewer(
    driver: MujocoCartesianDriver,
    robot: SafeCartesianRobot,
    teleoperator: WebcamTeleoperator,
    observation: Observation,
    counts: dict[str, int],
    max_steps: int | None,
) -> tuple[int, str]:
    import cv2
    from mujoco import MjrRect, mjtGridPos, viewer  # type: ignore[import-untyped]

    environment = driver.environment
    completed = 0
    termination = "viewer_closed"
    with viewer.launch_passive(
        environment._model,  # noqa: SLF001
        environment._data,  # noqa: SLF001
        key_callback=teleoperator.on_key,
    ) as handle:
        while handle.is_running() and (max_steps is None or completed < max_steps):
            started_at = time.monotonic()
            with handle.lock():
                observation = _control_step(robot, teleoperator, observation, counts)
                task = environment.task_state()
            completed += 1
            diagnostics = teleoperator.diagnostics
            report = robot.last_report
            age = (
                f"{diagnostics.frame_age_ms:.0f} ms"
                if diagnostics.frame_age_ms is not None
                else "none"
            )
            handle.set_texts(
                (
                    None,
                    mjtGridPos.mjGRID_TOPLEFT,
                    "ACT Lab webcam teleoperation",
                    "\n".join(
                        (
                            "Enter: calibrate  Q: quit",
                            f"state: {diagnostics.state}",
                            f"source: {diagnostics.source_status}; age: {age}",
                            "hand: "
                            f"{diagnostics.handedness or 'none'} "
                            f"({diagnostics.confidence or 0.0:.2f})",
                            "clutch: "
                            f"detected={diagnostics.clutch_detected} "
                            f"active={diagnostics.clutch_active}",
                            f"calibration: {diagnostics.calibration_id or 'none'}",
                            "safety: "
                            f"{report.outcome.value if report else 'none'} "
                            f"({report.detail if report else 'no command'})",
                            f"task: {task.reason or 'running'}",
                        )
                    ),
                )
            )
            preview = teleoperator.preview
            viewport = handle.viewport
            if preview is not None and viewport is not None:
                image = np.frombuffer(preview.rgb_bytes, dtype=np.uint8).reshape(
                    preview.height, preview.width, 3
                )
                width = min(320, max(1, viewport.width // 3))
                height = max(1, round(width * preview.height / preview.width))
                resized = cv2.resize(image, (width, height))
                handle.set_images(
                    (MjrRect(max(0, viewport.width - width), 0, width, height), resized)
                )
            handle.sync()
            if teleoperator.quit_requested:
                termination = "quit_requested"
                handle.close()
                break
            _pace(started_at, driver.control_period_s)
    if max_steps is not None and completed >= max_steps:
        termination = "step_limit"
    return completed, termination


def _pace(started_at: float, period_s: float) -> None:
    remaining = period_s - (time.monotonic() - started_at)
    if remaining > 0.0:
        time.sleep(remaining)
