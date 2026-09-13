"""CLI composition for opt-in simulation recording, without webcam pixels."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import asdict
from datetime import UTC, datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any

from act_lab import __version__
from act_lab.adapters.mcap.recording import McapEpisodeSink
from act_lab.application import SafeCartesianRobot
from act_lab.application.recording import RecordingRobot
from act_lab.domain.models import CameraFrame, Observation
from act_lab.domain.recording import EpisodeOutcome, EpisodeProvenance

if TYPE_CHECKING:
    from act_lab.adapters.mediapipe import WebcamTeleoperator
    from act_lab.adapters.mujoco import MujocoCartesianDriver


def _git(*args: str) -> str | None:
    try:
        return subprocess.run(
            ["git", *args],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _sha256(path: Path) -> str:
    with path.open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def webcam_acquisition_provenance(args: argparse.Namespace) -> dict[str, Any]:
    """Identify acquisition inputs by content, without storing video or file paths."""
    return {
        "tracking_model": {"sha256": _sha256(args.model)},
        "input": (
            {"kind": "recorded_video", "sha256": _sha256(args.video)}
            if args.video is not None
            else {"kind": "live_camera", "camera_id": args.camera}
        ),
        "headless": args.headless,
        "auto_calibrate": args.auto_calibrate,
        "max_steps": args.max_steps,
    }


@contextmanager
def recording_robot(
    driver: MujocoCartesianDriver,
    args: argparse.Namespace,
    source: str,
    seed: int,
    teleoperator: WebcamTeleoperator | None = None,
) -> Iterator[SafeCartesianRobot]:
    if args.record_dir is None:
        yield SafeCartesianRobot(driver, driver.limits)
        return
    if args.outcome != "auto" and not args.reason:
        raise ValueError("an explicit --outcome requires --reason")
    config = driver.simulation_config
    status = _git("status", "--porcelain")
    resolved: dict[str, Any] = {"simulation": asdict(config)}
    if teleoperator is not None:
        resolved["teleoperation"] = asdict(teleoperator.config)
        resolved["acquisition"] = webcam_acquisition_provenance(args)
    provenance = EpisodeProvenance(
        application_version=__version__,
        git_revision=_git("rev-parse", "HEAD")
        or os.environ.get("ACT_LAB_GIT_REVISION", "unknown"),
        git_dirty="unknown" if status is None else str(bool(status)).lower(),
        seed=seed,
        source=source,
        robot_id="ur5e-parallel-gripper-v1",
        task_id="pick-place-v1",
        camera_ids=config.cameras,
        operator=args.operator or "",
        container_identity=os.environ.get("ACT_LAB_CONTAINER_IDENTITY", "unknown"),
        clock_id="simulation-fixed-step-ns",
        host_monotonic_start_ns=time.monotonic_ns(),
        wall_start_utc=datetime.now(UTC).isoformat(),
        resolved_config_json=json.dumps(resolved, sort_keys=True),
        rates_json=json.dumps(
            {
                "physics_hz": config.physics_hz,
                "control_hz": config.environment_hz,
                "scene_camera_hz": config.environment_hz,
                "webcam_requested_hz": (
                    teleoperator.config.fps if teleoperator else None
                ),
            },
            sort_keys=True,
        ),
        calibration_json=json.dumps(
            teleoperator.calibration_snapshot
            if teleoperator
            else {"status": "not_applicable"},
            sort_keys=True,
        ),
    )
    sink = McapEpisodeSink(Path(args.record_dir))

    def images(observation: Observation) -> tuple[tuple[str, CameraFrame], ...]:
        frames = []
        for camera in config.cameras:
            pixels = driver.environment.render(camera)
            frames.append(
                (
                    camera,
                    CameraFrame(
                        observation.timestamp_ns,
                        config.render_width,
                        config.render_height,
                        pixels.tobytes(),
                    ),
                )
            )
        return tuple(frames)

    previous_task: tuple[bool, bool, str | None] | None = None

    def after_sample(timestamp_ns: int) -> None:
        nonlocal previous_task
        # Labels only: privileged cube/goal geometry never enters recordings.
        task = driver.environment.task_state()
        current = (task.success, task.terminal, task.reason)
        if current != previous_task:
            sink.event(
                timestamp_ns,
                "task",
                (
                    EpisodeOutcome.SUCCESS
                    if task.success
                    else EpisodeOutcome.FAILURE
                    if task.terminal
                    else EpisodeOutcome.UNKNOWN
                ),
                task.reason or "running",
            )
            previous_task = current
        if teleoperator is not None:
            sink.diagnostics(
                timestamp_ns,
                time.monotonic_ns(),
                json.dumps(asdict(teleoperator.diagnostics), sort_keys=True),
                json.dumps(teleoperator.calibration_snapshot, sort_keys=True),
            )

    robot = RecordingRobot(
        driver, driver.limits, sink, provenance, images, after_sample
    )
    try:
        yield robot
        task = driver.environment.task_state()
        outcome = (
            (EpisodeOutcome.SUCCESS if task.success else EpisodeOutcome.FAILURE)
            if args.outcome == "auto"
            else EpisodeOutcome(args.outcome)
        )
        robot.finish(outcome, args.reason or task.reason or "session_ended")
        print(f"recorded episode: {sink.final_path}", file=sys.stderr)
    except BaseException:
        sink.interrupt()
        if sink.partial_path.exists():
            print(f"interrupted recording: {sink.partial_path}", file=sys.stderr)
        raise


def recording_key(robot: SafeCartesianRobot, keycode: int) -> None:
    """GLFW F6/F7/F8 request success/failure/discard for the current attempt."""
    if not isinstance(robot, RecordingRobot):
        return
    outcomes = {
        295: EpisodeOutcome.SUCCESS,
        296: EpisodeOutcome.FAILURE,
        297: EpisodeOutcome.DISCARDED,
    }
    if keycode in outcomes:
        robot.request_finish(outcomes[keycode], "operator_viewer_label")
        print(
            f"recording outcome requested: {outcomes[keycode].value}", file=sys.stderr
        )


def recording_status(robot: SafeCartesianRobot) -> str:
    if isinstance(robot, RecordingRobot):
        if robot.recording_finished:
            return (
                f"EPISODE SAVED: {robot.recording_status.upper()}\n"
                "Recording stopped - further movement is NOT recorded.\n"
                "Q: close viewer. Start a new session for another attempt."
            )
        return (
            "RECORDING ACTIVE\nF6: save success   F7: save failure   F8: discard"
        )
    return "RECORDING OFF"
