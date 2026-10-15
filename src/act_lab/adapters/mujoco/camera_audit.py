"""Bounded headless visibility audit; privileged labels stay outside learning."""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

from act_lab.adapters.mujoco import MujocoCartesianDriver, SimulationConfig
from act_lab.application import SafeCartesianRobot, ScriptedPickPlaceExpert
from act_lab.application.camera_study import REQUIRED_PHASES, visibility_summary
from act_lab.application.training import git_identity
from act_lab.domain import Action


def audit_cameras(config_path: Path, output: Path, seeds: range) -> dict[str, Any]:
    config = SimulationConfig.load(config_path)
    if set(config.cameras) != {"wrist", "policy"}:
        raise ValueError("visibility audit requires wrist+policy, without overview")
    if not seeds or len(seeds) > 10:
        raise ValueError("visibility audit requires 1–10 explicit seeds")
    if output.exists():
        raise ValueError("audit output already exists")
    output.mkdir(parents=True)
    episodes = []
    provenance = {}
    for seed in seeds:
        with MujocoCartesianDriver.from_config_file(config_path) as driver:
            env = driver.environment
            robot = SafeCartesianRobot(driver, driver.limits)
            observation = robot.reset(seed)
            expert = ScriptedPickPlaceExpert(env, config.expert)
            expert.reset(observation)
            samples = []
            sampled_phases: set[str] = set()
            illustrated_phases: set[str] = set()
            task = env.task_state()
            for step in range(config.episode_steps):
                action = expert.poll(observation)
                phase = expert.phase.value
                if step % 10 == 0 or phase not in sampled_phases:
                    sampled_phases.add(phase)
                    row: dict[str, Any] = {
                        "timestamp_ns": observation.timestamp_ns,
                        "phase": phase,
                        "cameras": {
                            camera: env.visible_task_pixels(camera)
                            for camera in config.cameras
                        },
                    }
                    samples.append(row)
                    target = REQUIRED_PHASES.get(phase)
                    if (
                        target
                        and phase not in illustrated_phases
                        and (row["cameras"]["wrist"][target] < 16)
                    ):
                        import cv2

                        illustrated_phases.add(phase)
                        for camera in config.cameras:
                            image = env.render(camera)
                            if not cv2.imwrite(
                                str(output / f"seed-{seed}-{phase}-{camera}.png"),
                                cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
                            ):
                                raise RuntimeError(
                                    "could not save audit evidence image"
                                )
                robot.command(action)
                if robot.last_report is None:
                    raise RuntimeError("missing safe command report")
                expert.record_command_report(robot.last_report)
                observation = robot.observe()
                task = env.task_state()
                if task.terminal or expert.failure_reason:
                    break
            robot.command(
                Action(
                    observation.timestamp_ns,
                    observation.robot.end_effector_pose,
                    observation.robot.gripper_position,
                    False,
                )
            )
            episodes.append(
                {
                    "seed": seed,
                    "expert_success": task.success,
                    "failure_reason": expert.failure_reason or task.reason,
                    "samples": samples,
                }
            )
            provenance = {
                "simulation": asdict(config),
                "model_identity": env.model_identity,
                "scene_cameras": {c: env.camera_calibration(c) for c in config.cameras},
            }
    report = {
        **visibility_summary(episodes),
        "episodes": episodes,
        "provenance": provenance,
        "safety_path": "SafeCartesianRobot.command(Action)",
        "policy_inputs": "RGB plus measured joints/gripper only",
        "source": {
            "git": git_identity(Path.cwd()),
            "files_sha256": {
                path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                for path in (
                    Path(__file__),
                    Path(__file__).with_name("environment.py"),
                    Path(__file__).parents[2] / "application/camera_study.py",
                    config_path,
                )
            },
        },
    }
    partial = output / "visibility.json.partial"
    partial.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    partial.replace(output / "visibility.json")
    return report
