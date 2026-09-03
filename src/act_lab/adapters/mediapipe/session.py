"""Run webcam teleoperation through the MuJoCo safe-control path."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

import numpy as np

from act_lab.adapters.mediapipe.teleoperator import WebcamTeleoperator
from act_lab.adapters.mediapipe.viewer_ui import build_viewer_hud, draw_camera_hud
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
                observation = _control_step(robot, teleoperator, observation, counts)
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
    import mujoco  # type: ignore[import-untyped]
    from mujoco import MjrRect, mjtGridPos, viewer

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
            hud = build_viewer_hud(diagnostics, report, observation, task.reason)
            handle.set_texts(
                (
                    None,
                    mjtGridPos.mjGRID_TOPLEFT,
                    hud.banner,
                    "\n".join(hud.lines),
                )
            )
            _update_command_scene(mujoco, handle, hud)
            preview = teleoperator.preview
            viewport = handle.viewport
            if preview is not None and viewport is not None:
                image = draw_camera_hud(preview, diagnostics, teleoperator.config)
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


def _update_command_scene(mujoco: object, handle: object, hud: object) -> None:
    from act_lab.adapters.mediapipe.viewer_ui import ViewerHud

    assert isinstance(hud, ViewerHud)
    scene = handle.user_scn  # type: ignore[attr-defined]
    if scene is None:
        return
    scene.ngeom = 0
    if not hud.show_target:
        return
    identity = np.eye(3).ravel()
    target = np.asarray(hud.target_xyz_m)
    actual = np.asarray(hud.actual_xyz_m)
    mujoco.mjv_initGeom(  # type: ignore[attr-defined]
        scene.geoms[0],
        type=mujoco.mjtGeom.mjGEOM_SPHERE,  # type: ignore[attr-defined]
        size=[0.025, 0.0, 0.0],
        pos=target,
        mat=identity,
        rgba=np.asarray(hud.target_rgba),
    )
    scene.ngeom = 1
    if float(np.linalg.norm(target - actual)) > 1e-5:
        mujoco.mjv_connector(  # type: ignore[attr-defined]
            scene.geoms[1],
            mujoco.mjtGeom.mjGEOM_ARROW,  # type: ignore[attr-defined]
            0.008,
            actual,
            target,
        )
        scene.geoms[1].rgba = np.asarray(hud.target_rgba)
        scene.ngeom = 2
    executed = hud.executed_target_xyz_m
    if executed is not None and not np.allclose(executed, target, atol=1e-5):
        mujoco.mjv_initGeom(  # type: ignore[attr-defined]
            scene.geoms[scene.ngeom],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,  # type: ignore[attr-defined]
            size=[0.018, 0.0, 0.0],
            pos=np.asarray(executed),
            mat=identity,
            rgba=np.asarray((1.0, 0.65, 0.05, 0.9)),
        )
        scene.ngeom += 1


def _pace(started_at: float, period_s: float) -> None:
    remaining = period_s - (time.monotonic() - started_at)
    if remaining > 0.0:
        time.sleep(remaining)
