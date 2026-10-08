"""Stable command-line entry point for all ACT Lab workflows."""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from act_lab import __version__


def _doctor(as_json: bool) -> int:
    report = {
        "act_lab_version": __version__,
        "python": platform.python_version(),
        "python_supported": sys.version_info[:2] == (3, 12),
        "status": "ok",
    }
    if not report["python_supported"]:
        report["status"] = "error"

    if as_json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        for key, value in report.items():
            print(f"{key}: {value}")
    return 0 if report["status"] == "ok" else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="act-lab")
    parser.add_argument("--version", action="version", version=__version__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    doctor = subparsers.add_parser("doctor", help="check the runtime environment")
    doctor.add_argument(
        "--json", action="store_true", help="emit machine-readable JSON"
    )

    sim = subparsers.add_parser("sim", help="run deterministic simulation workflows")
    sim_commands = sim.add_subparsers(dest="sim_command", required=True)
    rollout = sim_commands.add_parser("rollout", help="run a headless neutral rollout")
    rollout.add_argument("--seed", type=int, default=0)
    rollout.add_argument("--steps", type=int, default=50)
    rollout.add_argument(
        "--config",
        type=Path,
        default=Path("configs/sim/ur5e_pick_place.toml"),
    )
    rollout.add_argument("--render-dir", type=Path)
    rollout.add_argument("--json", action="store_true")
    control_smoke = sim_commands.add_parser(
        "control-smoke", help="run the safe Cartesian controller headlessly"
    )
    control_smoke.add_argument("--seed", type=int, default=0)
    control_smoke.add_argument("--steps", type=int, default=50)
    control_smoke.add_argument(
        "--config",
        type=Path,
        default=Path("configs/sim/ur5e_pick_place.toml"),
    )
    control_smoke.add_argument("--json", action="store_true")
    control_diagnostic = sim_commands.add_parser(
        "control-diagnostic",
        help="compare a sustained Cartesian target with measured motion",
    )
    control_diagnostic.add_argument("--seed", type=int, default=0)
    control_diagnostic.add_argument(
        "--distance-m",
        type=float,
        default=-0.1,
        help="signed world-X displacement; default -0.1 is reachable from home",
    )
    control_diagnostic.add_argument("--duration-s", type=float, default=1.2)
    control_diagnostic.add_argument(
        "--config",
        type=Path,
        default=Path("configs/sim/ur5e_pick_place.toml"),
    )
    control_diagnostic.add_argument("--json", action="store_true")
    view = sim_commands.add_parser("view", help="open the interactive MuJoCo viewer")
    view.add_argument("--seed", type=int, default=0)
    view.add_argument(
        "--config",
        type=Path,
        default=Path("configs/sim/ur5e_pick_place.toml"),
    )
    view.add_argument(
        "--max-steps",
        type=int,
        help="close after this many environment steps; default is to run until closed",
    )
    keyboard = sim_commands.add_parser(
        "keyboard-teleop", help="teleoperate safely with the MuJoCo viewer"
    )
    keyboard.add_argument("--seed", type=int, default=0)
    keyboard.add_argument(
        "--config",
        type=Path,
        default=Path("configs/sim/ur5e_pick_place.toml"),
    )
    keyboard.add_argument(
        "--max-steps",
        type=int,
        help="close after this many control steps; default is to run until closed",
    )
    webcam = sim_commands.add_parser(
        "webcam-teleop", help="teleoperate safely with MediaPipe hand tracking"
    )
    webcam.add_argument("--seed", type=int, default=0)
    webcam.add_argument(
        "--config",
        type=Path,
        default=Path("configs/sim/ur5e_pick_place.toml"),
        help="simulation and safety configuration",
    )
    webcam.add_argument(
        "--teleop-config",
        type=Path,
        default=Path("configs/teleop/webcam.toml"),
    )
    source = webcam.add_mutually_exclusive_group()
    source.add_argument("--camera", default="/dev/video0")
    source.add_argument("--video", type=Path)
    webcam.add_argument(
        "--model",
        type=Path,
        default=Path(
            os.environ.get(
                "ACT_LAB_HAND_MODEL",
                "/opt/act-lab/models/hand_landmarker.task",
            )
        ),
    )
    webcam.add_argument("--headless", action="store_true")
    webcam.add_argument("--auto-calibrate", action="store_true")
    webcam.add_argument("--max-steps", type=int)
    webcam.add_argument("--json", action="store_true")
    expert = sim_commands.add_parser(
        "expert", help="benchmark the deterministic scripted expert"
    )
    expert.add_argument("--seed-start", type=int, default=0)
    expert.add_argument("--episodes", type=int, default=20)
    expert.add_argument("--min-success-rate", type=float, default=0.90)
    expert.add_argument(
        "--config",
        type=Path,
        default=Path("configs/sim/ur5e_pick_place.toml"),
    )
    expert.add_argument("--json", action="store_true")
    evaluate = sim_commands.add_parser(
        "evaluate", help="compare expert and learned ACT in closed-loop simulation"
    )
    evaluate.add_argument("--checkpoint", type=Path, required=True)
    evaluate.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    evaluate.add_argument("--seed-start", type=int, default=10000)
    evaluate.add_argument("--episodes", type=int, default=20)
    evaluate.add_argument(
        "--policy-hz",
        type=int,
        help=(
            "ACT observation/action rate "
            "(default: infer from training dataset metadata)"
        ),
    )
    evaluate.add_argument(
        "--config", type=Path, default=Path("configs/sim/ur5e_pick_place.toml")
    )
    evaluate.add_argument("--output", type=Path, required=True)
    evaluate.add_argument(
        "--video", action=argparse.BooleanOptionalAction, default=True
    )
    evaluate.add_argument(
        "--trace-actions",
        action="store_true",
        help="write requested/executed actions and safety decisions per step",
    )
    evaluate.add_argument("--json", action="store_true")
    for command in (expert, keyboard, webcam):
        command.add_argument(
            "--record-dir", type=Path, help="opt in to MCAP scene/state recording"
        )
        command.add_argument("--operator", help="operator pseudonym")
        command.add_argument(
            "--outcome",
            default="auto",
            choices=("auto", "success", "failure", "discarded"),
        )
        command.add_argument("--reason", help="outcome or discard reason")
    recording = subparsers.add_parser("recording", help="inspect or recover raw MCAP")
    recording_commands = recording.add_subparsers(
        dest="recording_command", required=True
    )
    for name in ("inspect", "recover"):
        command = recording_commands.add_parser(name)
        command.add_argument("path", type=Path)
    validate = recording_commands.add_parser(
        "validate", help="apply training-quality rules to raw episodes"
    )
    validate.add_argument("paths", type=Path, nargs="+")
    manifest = recording_commands.add_parser(
        "manifest", help="validate, select, and split raw episodes"
    )
    manifest.add_argument("paths", type=Path, nargs="+")
    manifest.add_argument("--output", type=Path, required=True)
    manifest.add_argument("--validation-fraction", type=float, default=0.2)
    manifest.add_argument("--split-seed", type=int, default=0)
    manifest.add_argument(
        "--include-episode",
        action="append",
        default=[],
        help="select only these eligible episode IDs; repeat as needed",
    )
    manifest.add_argument(
        "--include-seed-range",
        action="append",
        default=[],
        metavar="START:COUNT",
        help="select eligible episodes by acquisition seed; repeat as needed",
    )
    manifest.add_argument(
        "--include-source", help="require this acquisition source for selection"
    )
    replay = recording_commands.add_parser(
        "replay", help="inspect or export synchronized scene frames"
    )
    replay.add_argument("path", type=Path)
    replay.add_argument("--camera")
    replay.add_argument("--render-dir", type=Path)
    replay.add_argument("--max-frames", type=int)
    replay.add_argument("--display", action="store_true")
    foxglove = recording_commands.add_parser(
        "foxglove", help="create a Foxglove-viewable MCAP derivative"
    )
    foxglove.add_argument("path", type=Path)
    foxglove.add_argument("--output", type=Path, required=True)
    convert = recording_commands.add_parser(
        "convert", help="convert a frozen selection manifest to LeRobotDataset"
    )
    convert.add_argument("manifest", type=Path)
    convert.add_argument("--output", type=Path, required=True)
    convert.add_argument("--repo-id", required=True)
    convert.add_argument("--fps", type=int, default=25)
    training = subparsers.add_parser("train", help="train or resume an ACT policy")
    training_commands = training.add_subparsers(dest="train_command", required=True)
    act = training_commands.add_parser("act", help="train ACT on a validated dataset")
    act.add_argument("--dataset", type=Path, required=True)
    act.add_argument("--output", type=Path, required=True)
    act.add_argument("--device", choices=("cpu", "cuda"), required=True)
    act.add_argument("--config", type=Path, default=Path("configs/training/act.toml"))
    act.add_argument(
        "--wandb-mode",
        choices=("disabled", "offline", "online"),
        default="disabled",
    )
    resume = training_commands.add_parser("resume", help="resume an ACT Lab run")
    resume.add_argument("--run", type=Path, required=True)
    resume.add_argument("--device", choices=("cpu", "cuda"), required=True)
    resume.add_argument("--steps", type=int)
    experiment = subparsers.add_parser(
        "experiment", help="inspect demonstration coverage and compare evaluations"
    )
    experiment_commands = experiment.add_subparsers(
        dest="experiment_command", required=True
    )
    coverage = experiment_commands.add_parser(
        "coverage", help="report selected cube spawn coverage by dataset split"
    )
    coverage.add_argument("--manifest", type=Path, required=True)
    coverage.add_argument(
        "--config", type=Path, default=Path("configs/sim/ur5e_pick_place.toml")
    )
    coverage.add_argument("--bins", type=int, default=4)
    coverage.add_argument("--output", type=Path, required=True)
    comparison = experiment_commands.add_parser(
        "compare", help="compare two ACT evaluations on the same seeds"
    )
    comparison.add_argument("--baseline", type=Path, required=True)
    comparison.add_argument("--candidate", type=Path, required=True)
    comparison.add_argument("--output", type=Path, required=True)
    ros2 = subparsers.add_parser("ros2", help="Optional ROS 2 contracts")
    ros_commands = ros2.add_subparsers(dest="ros2_command", required=True)
    contract = ros_commands.add_parser(
        "contract-smoke", help="Two-process DDS contracts"
    )
    contract.add_argument("--json", action="store_true")
    return parser


def _experiment(args: argparse.Namespace) -> int:
    from act_lab.application.experiment import compare_evaluations, coverage_report
    from act_lab.application.training import atomic_json

    try:
        if args.output.exists():
            raise FileExistsError(f"report already exists: {args.output}")
        if args.experiment_command == "coverage":
            from act_lab.adapters.mcap.inspection import inspect_episode
            from act_lab.adapters.mujoco.config import SimulationConfig
            from act_lab.adapters.mujoco.seed_geometry import cube_spawn_xy
            from act_lab.application.dataset import file_sha256

            manifest = json.loads(args.manifest.read_text())
            if manifest.get("manifest_version") != 1:
                raise ValueError("unsupported selection manifest version")
            config = SimulationConfig.load(args.config)
            episodes = []
            for entry in manifest["episodes"]:
                if not entry["selected"]:
                    continue
                path = (args.manifest.parent / entry["path"]).resolve()
                if file_sha256(path) != entry["sha256"]:
                    raise ValueError(f"raw episode changed since selection: {path}")
                provenance = inspect_episode(path)["provenance"]
                if (
                    provenance is None
                    or provenance["episode_id"] != entry["episode_id"]
                ):
                    raise ValueError(f"episode provenance mismatch: {path}")
                if provenance["source"] != "expert":
                    raise ValueError(f"selected episode is not scripted expert: {path}")
                recorded_simulation = json.loads(provenance["resolved_config_json"])[
                    "simulation"
                ]
                if recorded_simulation["cube_spawn_x_m"] != list(
                    config.cube_spawn_x_m
                ) or recorded_simulation["cube_spawn_y_m"] != list(
                    config.cube_spawn_y_m
                ):
                    raise ValueError(f"recorded spawn bounds differ: {path}")
                seed = provenance["seed"]
                episodes.append(
                    {
                        "episode_id": entry["episode_id"],
                        "seed": seed,
                        "split": entry["split"],
                        "cube_xy_m": cube_spawn_xy(seed, config),
                    }
                )
            result = coverage_report(
                episodes, config.cube_spawn_x_m, config.cube_spawn_y_m, bins=args.bins
            )
            result["selection_manifest"] = str(args.manifest)
            result["simulation_config"] = str(args.config)
        else:
            baseline = json.loads(args.baseline.read_text())
            candidate = json.loads(args.candidate.read_text())
            result = compare_evaluations(baseline, candidate)
            result["baseline_report"] = str(args.baseline)
            result["candidate_report"] = str(args.candidate)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        atomic_json(args.output, result)
    except (KeyError, OSError, TypeError, ValueError) as error:
        print(f"experiment error: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


def _write_ppm(path: Path, frame: object) -> None:
    import numpy as np

    pixels = np.asarray(frame, dtype=np.uint8)
    height, width, channels = pixels.shape
    if channels != 3:
        raise ValueError("rendered frame must have three RGB channels")
    path.write_bytes(f"P6\n{width} {height}\n255\n".encode() + pixels.tobytes())


def _sim_rollout(args: argparse.Namespace) -> int:
    from act_lab.adapters.mujoco import MujocoUR5eEnvironment

    if args.steps <= 0:
        print("steps must be positive", file=sys.stderr)
        return 2
    try:
        with MujocoUR5eEnvironment.from_config_file(args.config) as environment:
            initial = environment.reset(args.seed)
            if args.render_dir is not None:
                args.render_dir.mkdir(parents=True, exist_ok=True)
                for camera in initial.image_keys:
                    _write_ppm(
                        args.render_dir / f"{camera}.ppm", environment.render(camera)
                    )
            observation = initial
            for _ in range(args.steps):
                observation = environment.step()
            status = environment.task_status()
            report = {
                "cube_position_xyz_m": environment.cube_position_xyz_m,
                "environment_steps": args.steps,
                "physics_ticks": environment.physics_ticks,
                "seed": args.seed,
                "status": "ok",
                "task_success": status.success,
                "timestamp_ns": observation.timestamp_ns,
            }
    except (OSError, RuntimeError, ValueError) as error:
        print(f"simulation error: {error}", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        for key, value in report.items():
            print(f"{key}: {value}")
    return 0


def _sim_view(args: argparse.Namespace) -> int:
    from act_lab.adapters.mujoco import MujocoUR5eEnvironment

    try:
        with MujocoUR5eEnvironment.from_config_file(args.config) as environment:
            environment.launch_viewer(args.seed, max_steps=args.max_steps)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"simulation viewer error: {error}", file=sys.stderr)
        return 2
    return 0


def _sim_control_smoke(args: argparse.Namespace) -> int:
    from act_lab.adapters.mujoco import MujocoCartesianDriver
    from act_lab.application import SafeCartesianRobot
    from act_lab.domain import Action, CommandOutcome, Pose

    if args.steps <= 0:
        print("steps must be positive", file=sys.stderr)
        return 2
    try:
        with MujocoCartesianDriver.from_config_file(args.config) as driver:
            robot = SafeCartesianRobot(driver, driver.limits)
            observation = robot.reset(args.seed)
            initial_pose = observation.robot.end_effector_pose
            target = Pose(
                frame_id="world",
                position_xyz_m=(
                    initial_pose.position_xyz_m[0] + 0.03,
                    initial_pose.position_xyz_m[1],
                    initial_pose.position_xyz_m[2] + 0.02,
                ),
                quaternion_wxyz=initial_pose.quaternion_wxyz,
            )
            counts = {outcome.value: 0 for outcome in CommandOutcome}
            state = observation.robot
            for _ in range(args.steps):
                state = robot.command(
                    Action(state.timestamp_ns, target, 0.25, enabled=True)
                )
                command_report = robot.last_command_report
                if command_report is None:
                    raise RuntimeError("safe controller did not produce a report")
                counts[command_report.outcome.value] += 1
            report = {
                "command_outcomes": counts,
                "end_effector_position_xyz_m": state.end_effector_pose.position_xyz_m,
                "environment_steps": args.steps,
                "physics_ticks": driver.environment.physics_ticks,
                "seed": args.seed,
                "status": "ok",
                "timestamp_ns": state.timestamp_ns,
            }
    except (OSError, RuntimeError, ValueError) as error:
        print(f"controller smoke error: {error}", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        for key, value in report.items():
            print(f"{key}: {value}")
    return 0


def _sim_control_diagnostic(args: argparse.Namespace) -> int:
    from act_lab.adapters.mujoco import MujocoCartesianDriver
    from act_lab.application import SafeCartesianRobot
    from act_lab.domain import Action, Pose

    if not 0.0 < abs(args.distance_m) <= 0.25:
        print("absolute distance-m must be in (0, 0.25]", file=sys.stderr)
        return 2
    if not math.isfinite(args.duration_s) or args.duration_s <= 0.0:
        print("duration-s must be finite and positive", file=sys.stderr)
        return 2
    try:
        with MujocoCartesianDriver.from_config_file(args.config) as driver:
            robot = SafeCartesianRobot(driver, driver.limits)
            state = robot.reset(args.seed).robot
            initial = state.end_effector_pose
            requested = Pose(
                "world",
                (
                    initial.position_xyz_m[0] + args.distance_m,
                    initial.position_xyz_m[1],
                    initial.position_xyz_m[2],
                ),
                initial.quaternion_wxyz,
            )
            if driver.solve_ik(requested) is None:
                raise ValueError(
                    "diagnostic target is not IK-reachable from reset pose"
                )
            steps = max(1, round(args.duration_s / driver.control_period_s))
            trajectory: list[dict[str, object]] = []
            max_commanded_speed = 0.0
            max_measured_speed = 0.0
            for index in range(steps):
                state = robot.command(
                    Action(state.timestamp_ns, requested, state.gripper_position, True)
                )
                command_report = robot.last_report
                if command_report is None:
                    raise RuntimeError("safe controller did not produce a report")
                executed = command_report.executed_action
                trajectory.append(
                    {
                        "step": index + 1,
                        "sim_time_s": state.timestamp_ns / 1_000_000_000.0,
                        "safety_outcome": command_report.outcome.value,
                        "safety_limited_xyz_m": (
                            executed.target_pose.position_xyz_m
                            if executed is not None
                            else None
                        ),
                        "measured_xyz_m": state.end_effector_pose.position_xyz_m,
                        "commanded_speed_m_s": (
                            command_report.commanded_cartesian_speed_m_s
                        ),
                        "measured_speed_m_s": (
                            command_report.measured_cartesian_speed_m_s
                        ),
                        "tracking_error_m": command_report.tracking_error_m,
                    }
                )
                max_commanded_speed = max(
                    max_commanded_speed,
                    command_report.commanded_cartesian_speed_m_s,
                )
                max_measured_speed = max(
                    max_measured_speed,
                    command_report.measured_cartesian_speed_m_s,
                )
            displacement = math.dist(
                initial.position_xyz_m, state.end_effector_pose.position_xyz_m
            )
            report = {
                "seed": args.seed,
                "requested_target_xyz_m": requested.position_xyz_m,
                "final_measured_xyz_m": state.end_effector_pose.position_xyz_m,
                "requested_distance_m": args.distance_m,
                "measured_displacement_m": displacement,
                "duration_s": steps * driver.control_period_s,
                "max_commanded_speed_m_s": max_commanded_speed,
                "max_measured_speed_m_s": max_measured_speed,
                "final_tracking_error_m": math.dist(
                    requested.position_xyz_m,
                    state.end_effector_pose.position_xyz_m,
                ),
                "trajectory": trajectory,
            }
    except (OSError, RuntimeError, ValueError) as error:
        print(f"controller diagnostic error: {error}", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        for key, value in report.items():
            if key != "trajectory":
                print(f"{key}: {value}")
    return 0


def _sim_keyboard_teleop(args: argparse.Namespace) -> int:
    from act_lab.adapters.mcap.session import recording_robot
    from act_lab.adapters.mujoco import (
        KeyboardTeleoperator,
        MujocoCartesianDriver,
        run_keyboard_session,
    )

    try:
        with (
            MujocoCartesianDriver.from_config_file(args.config) as driver,
            recording_robot(driver, args, "keyboard", args.seed) as robot,
        ):
            teleoperator = KeyboardTeleoperator(
                driver.simulation_config.keyboard,
                driver.limits,
            )
            run_keyboard_session(
                driver,
                robot,
                teleoperator,
                args.seed,
                max_steps=args.max_steps,
            )
    except (OSError, RuntimeError, ValueError) as error:
        print(f"keyboard teleoperation error: {error}", file=sys.stderr)
        return 2
    return 0


def _sim_webcam_teleop(args: argparse.Namespace) -> int:
    from act_lab.adapters.mcap.session import recording_robot
    from act_lab.adapters.mediapipe import (
        HandTrackingWorker,
        MediaPipeHandTracker,
        OpenCVCamera,
        WebcamConfig,
        WebcamTeleoperator,
        run_webcam_session,
    )
    from act_lab.adapters.mujoco import MujocoCartesianDriver

    if args.max_steps is not None and args.max_steps <= 0:
        print("max-steps must be positive", file=sys.stderr)
        return 2
    if args.auto_calibrate and (args.video is None or not args.headless):
        print(
            "auto-calibrate requires --video and --headless",
            file=sys.stderr,
        )
        return 2
    try:
        teleop_config = WebcamConfig.load(args.teleop_config)
        teleoperator = WebcamTeleoperator(teleop_config)
        source = args.video if args.video is not None else args.camera
        with (
            OpenCVCamera(
                source,
                width=teleop_config.width,
                height=teleop_config.height,
                fps=teleop_config.fps,
                recorded=args.video is not None,
            ) as camera,
            MediaPipeHandTracker(args.model, teleop_config) as tracker,
            MujocoCartesianDriver.from_config_file(args.config) as driver,
            recording_robot(driver, args, "webcam", args.seed, teleoperator) as robot,
        ):
            worker = HandTrackingWorker(camera, tracker, teleoperator.update)
            report = run_webcam_session(
                driver,
                robot,
                teleoperator,
                worker,
                args.seed,
                headless=args.headless,
                auto_calibrate=args.auto_calibrate,
                max_steps=args.max_steps,
            )
    except (OSError, RuntimeError, ValueError) as error:
        print(f"webcam teleoperation error: {error}", file=sys.stderr)
        return 2
    if args.json or args.headless:
        print(json.dumps(report, indent=2, sort_keys=True))
    return 0


def _sim_expert(args: argparse.Namespace) -> int:
    from act_lab.adapters.mcap.session import recording_robot
    from act_lab.adapters.mujoco import MujocoCartesianDriver, SimulationConfig
    from act_lab.application import ScriptedPickPlaceExpert
    from act_lab.domain import CommandOutcome

    if args.episodes <= 0:
        print("episodes must be positive", file=sys.stderr)
        return 2
    if not 0.0 <= args.min_success_rate <= 1.0:
        print("min-success-rate must be in [0, 1]", file=sys.stderr)
        return 2
    try:
        config = SimulationConfig.load(args.config)
        episodes: list[dict[str, object]] = []
        for seed in range(args.seed_start, args.seed_start + args.episodes):
            with (
                MujocoCartesianDriver.from_config_file(args.config) as driver,
                recording_robot(driver, args, "expert", seed) as robot,
            ):
                observation = robot.reset(seed)
                expert = ScriptedPickPlaceExpert(driver.environment, config.expert)
                expert.reset(observation)
                counts = {outcome.value: 0 for outcome in CommandOutcome}
                completed_steps = 0
                final_task = None
                for _ in range(config.episode_steps):
                    completed_steps += 1
                    action = expert.poll(observation)
                    robot.command(action)
                    command_report = robot.last_report
                    if command_report is None:
                        raise RuntimeError("safe controller did not produce a report")
                    counts[command_report.outcome.value] += 1
                    expert.record_command_report(command_report)
                    observation = robot.observe()
                    task = driver.environment.task_state()
                    if task.terminal or expert.failure_reason is not None:
                        final_task = task
                        hold_action = expert.poll(observation)
                        robot.command(hold_action)
                        hold_report = robot.last_report
                        if hold_report is None:
                            raise RuntimeError(
                                "safe controller did not produce a hold report"
                            )
                        counts[hold_report.outcome.value] += 1
                        break
                task = final_task or driver.environment.task_state()
                terminal_reason = task.reason or expert.failure_reason or "step_limit"
                episodes.append(
                    {
                        "seed": seed,
                        "steps": completed_steps,
                        "result": "success" if task.success else "failure",
                        "terminal_reason": terminal_reason,
                        "final_phase": expert.phase.value,
                        "safety_outcomes": counts,
                    }
                )
        successes = sum(item["result"] == "success" for item in episodes)
        success_rate = successes / args.episodes
        benchmark_report = {
            "episodes": episodes,
            "episodes_requested": args.episodes,
            "min_success_rate": args.min_success_rate,
            "seed_start": args.seed_start,
            "successes": successes,
            "success_rate": success_rate,
            "threshold_passed": success_rate >= args.min_success_rate,
        }
    except (OSError, RuntimeError, ValueError) as error:
        print(f"expert benchmark error: {error}", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(benchmark_report, indent=2, sort_keys=True))
    else:
        print(
            f"expert success: {successes}/{args.episodes} "
            f"({success_rate:.1%}); required {args.min_success_rate:.1%}"
        )
        for item in episodes:
            print(
                f"seed {item['seed']}: {item['result']} in {item['steps']} steps "
                f"({item['terminal_reason']})"
            )
    return 0 if benchmark_report["threshold_passed"] else 1


def _sim_evaluate(args: argparse.Namespace) -> int:
    import hashlib
    from dataclasses import asdict
    from datetime import UTC, datetime

    import cv2

    from act_lab.adapters.lerobot.policy import LeRobotACTPolicy
    from act_lab.adapters.mujoco import MujocoCartesianDriver, SimulationConfig
    from act_lab.application import SafeCartesianRobot, ScriptedPickPlaceExpert
    from act_lab.application.evaluation import summarize_episodes
    from act_lab.domain import CommandOutcome

    if args.episodes <= 0 or args.seed_start < 0:
        print("episodes must be positive and seed-start nonnegative", file=sys.stderr)
        return 2
    try:
        config = SimulationConfig.load(args.config)
        if not args.checkpoint.is_dir():
            raise ValueError(f"checkpoint directory does not exist: {args.checkpoint}")
        checkpoint_root = args.checkpoint.resolve()
        pretrained_root = checkpoint_root / "pretrained_model"
        if pretrained_root.is_dir():
            checkpoint_root = pretrained_root
        training_config_path = checkpoint_root / "train_config.json"
        if not training_config_path.is_file():
            raise ValueError("checkpoint is missing pretrained_model/train_config.json")
        checkpoint_digest = hashlib.sha256()
        checkpoint_files = sorted(
            path for path in checkpoint_root.rglob("*") if path.is_file()
        )
        for path in checkpoint_files:
            checkpoint_digest.update(
                path.relative_to(checkpoint_root).as_posix().encode()
            )
            with path.open("rb") as source:
                while chunk := source.read(1024 * 1024):
                    checkpoint_digest.update(chunk)
        checkpoint_hash = checkpoint_digest.hexdigest()
        training_config = json.loads(training_config_path.read_text())
        dataset_root = Path(training_config["dataset"]["root"])
        dataset_info_path = dataset_root / "meta" / "info.json"
        dataset_fps = None
        if dataset_info_path.is_file():
            dataset_fps = int(json.loads(dataset_info_path.read_text())["fps"])
        policy_hz = args.policy_hz or dataset_fps
        if policy_hz is None:
            raise ValueError(
                "cannot infer training dataset FPS from checkpoint; provide --policy-hz"
            )
        if policy_hz <= 0 or config.environment_hz % policy_hz != 0:
            raise ValueError(
                f"policy-hz ({policy_hz}) must be a positive divisor of "
                f"simulation environment_hz ({config.environment_hz})"
            )
        policy_stride = config.environment_hz // policy_hz
        learned = LeRobotACTPolicy(
            args.checkpoint,
            args.device,
            config.cameras,
            n_action_steps=1,
        )
        args.output.mkdir(parents=True, exist_ok=True)
        all_results: dict[str, Any] = {}
        for policy_name in ("expert", "act"):
            episode_results: list[dict[str, object]] = []
            for seed in range(args.seed_start, args.seed_start + args.episodes):
                video_path = args.output / "videos" / policy_name / f"seed-{seed}.mp4"
                writer = None
                trace_stream = None
                trace_path = (
                    args.output / "traces" / policy_name / f"seed-{seed}.jsonl"
                )
                events = {
                    "grasp": 0,
                    "drop": 0,
                    "workspace_or_rate_limit": 0,
                    "safety_rejection": 0,
                }
                lifted = False
                observation = None
                failure = None
                completed_steps = 0
                held_action = None
                try:
                    with MujocoCartesianDriver.from_config_file(args.config) as driver:
                        robot = SafeCartesianRobot(driver, driver.limits)
                        observation = robot.reset(seed)
                        if policy_name == "act":
                            learned.reset()
                        expert = ScriptedPickPlaceExpert(
                            driver.environment, config.expert
                        )
                        expert.reset(observation)
                        if args.video:
                            video_path.parent.mkdir(parents=True, exist_ok=True)
                            first = driver.environment.render(config.cameras[0])
                            height, width, _ = first.shape
                            fourcc = cv2.VideoWriter.fourcc(*"mp4v")
                            writer = cv2.VideoWriter(
                                str(video_path),
                                fourcc,
                                config.environment_hz,
                                (width, height),
                            )
                            if not writer.isOpened():
                                raise RuntimeError(
                                    f"cannot create evaluation video: {video_path}"
                                )
                        if args.trace_actions:
                            trace_path.parent.mkdir(parents=True, exist_ok=True)
                            trace_stream = trace_path.open("w", encoding="utf-8")
                        command_outcomes = {
                            outcome.value: 0 for outcome in CommandOutcome
                        }
                        for step_index in range(config.episode_steps):
                            completed_steps += 1
                            if policy_name == "expert":
                                action = expert.poll(observation)
                            elif step_index % policy_stride == 0:
                                held_action = learned.act(
                                    observation,
                                    {
                                        camera: driver.environment.render(camera)
                                        for camera in config.cameras
                                    },
                                )
                                action = held_action
                            elif held_action is not None:
                                # Hold the policy-rate target while issuing a
                                # fresh command at the simulation safety rate.
                                action = type(held_action)(
                                    timestamp_ns=observation.timestamp_ns,
                                    target_pose=held_action.target_pose,
                                    gripper_position=held_action.gripper_position,
                                    enabled=held_action.enabled,
                                )
                            else:
                                raise RuntimeError("ACT has no action to hold")
                            robot.command(action)
                            command_report = robot.last_report
                            if command_report is None:
                                raise RuntimeError(
                                    "safe controller produced no command report"
                                )
                            command_outcomes[command_report.outcome.value] += 1
                            if command_report.outcome is CommandOutcome.LIMITED:
                                events["workspace_or_rate_limit"] += 1
                            elif command_report.outcome in {
                                CommandOutcome.INVALID,
                                CommandOutcome.STALE,
                                CommandOutcome.IK_FAILURE,
                                CommandOutcome.COLLISION_STOP,
                            }:
                                events["safety_rejection"] += 1
                            if policy_name == "expert":
                                expert.record_command_report(command_report)
                            observation = robot.observe()
                            if writer is not None:
                                frame = driver.environment.render(config.cameras[0])
                                writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                            task = driver.environment.task_state()
                            if trace_stream is not None:
                                trace_stream.write(
                                    json.dumps(
                                        {
                                            "seed": seed,
                                            "policy": policy_name,
                                            "step": completed_steps,
                                            "policy_inference": (
                                                policy_name == "act"
                                                and step_index % policy_stride == 0
                                            ),
                                            "outcome": command_report.outcome.value,
                                            "detail": command_report.detail,
                                            "requested_action": asdict(
                                                command_report.requested_action
                                            ),
                                            "executed_action": (
                                                asdict(command_report.executed_action)
                                                if command_report.executed_action
                                                is not None
                                                else None
                                            ),
                                            "resulting_end_effector_pose": asdict(
                                                command_report.resulting_state.end_effector_pose
                                            ),
                                            "cube_pose": asdict(task.cube_pose),
                                            "commanded_speed_m_s": (
                                                command_report.commanded_cartesian_speed_m_s
                                            ),
                                            "measured_speed_m_s": (
                                                command_report.measured_cartesian_speed_m_s
                                            ),
                                            "tracking_error_m": (
                                                command_report.tracking_error_m
                                            ),
                                        },
                                        sort_keys=True,
                                    )
                                    + "\n"
                                )
                            cube_z = task.cube_pose.position_xyz_m[2]
                            if not lifted and cube_z > config.cube_center_z_m + 0.025:
                                lifted = True
                                events["grasp"] += 1
                            task_status = driver.environment.task_status()
                            if (
                                lifted
                                and cube_z < config.cube_center_z_m + 0.025
                                and task_status.cube_supported
                                and not task_status.cube_in_tray
                            ):
                                events["drop"] += 1
                                lifted = False
                            if task.terminal or (
                                policy_name == "expert" and expert.failure_reason
                            ):
                                break
                        task = driver.environment.task_state()
                        failure = (
                            None
                            if task.success
                            else (
                                task.reason
                                or (
                                    expert.failure_reason
                                    if policy_name == "expert"
                                    else None
                                )
                                or "step_limit"
                            )
                        )
                        final_phase = (
                            expert.phase.value if policy_name == "expert" else None
                        )
                        elapsed = observation.timestamp_ns / 1_000_000_000.0
                        episode_results.append(
                            {
                                "seed": seed,
                                "success": task.success,
                                "completion_time_s": elapsed if task.success else None,
                                "elapsed_time_s": elapsed,
                                "steps": completed_steps,
                                "failure_reason": failure,
                                "final_phase": final_phase,
                                "events": events,
                                "command_outcomes": command_outcomes,
                                "action_trace": (
                                    str(trace_path.relative_to(args.output))
                                    if args.trace_actions
                                    else None
                                ),
                                "video": str(video_path.relative_to(args.output))
                                if args.video
                                else None,
                                "final_cube_position_xyz_m": (
                                    task.cube_pose.position_xyz_m
                                ),
                            }
                        )
                except (RuntimeError, ValueError, OSError) as error:
                    episode_results.append(
                        {
                            "seed": seed,
                            "success": False,
                            "completion_time_s": None,
                            "elapsed_time_s": (
                                completed_steps / config.environment_hz
                            ),
                            "steps": completed_steps,
                            "failure_reason": f"evaluation_error: {error}",
                            "events": events,
                            "action_trace": (
                                str(trace_path.relative_to(args.output))
                                if args.trace_actions
                                else None
                            ),
                            "video": str(video_path.relative_to(args.output))
                            if args.video
                            else None,
                        }
                    )
                finally:
                    if writer is not None:
                        writer.release()
                    if trace_stream is not None:
                        trace_stream.close()
            all_results[policy_name] = {
                "summary": summarize_episodes(episode_results),
                "episodes": episode_results,
            }
        report = {
            "schema_version": 1,
            "created_at_utc": datetime.now(UTC).isoformat(),
            "seed_start": args.seed_start,
            "seed_count": args.episodes,
            "seeds": list(range(args.seed_start, args.seed_start + args.episodes)),
            "simulation_config": asdict(config),
            "policy": {
                "checkpoint": str(args.checkpoint.resolve()),
                "checkpoint_sha256": checkpoint_hash,
                "device": args.device,
                "policy_hz": policy_hz,
                "environment_hz": config.environment_hz,
                "policy_inference_stride": policy_stride,
                "act_n_action_steps": 1,
                "training_dataset_fps": dataset_fps,
                "training_configuration": training_config,
            },
            "safety_path": "SafeCartesianRobot.command(Action)",
            "confidence_interval": "95% Wilson score",
            "results": all_results,
        }
        report_path = args.output / "evaluation.json"
        partial = report_path.with_suffix(".json.partial")
        partial.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        partial.replace(report_path)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"evaluation error: {error}", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(f"evaluation report: {report_path}")
        for name, values in all_results.items():
            summary = values["summary"]
            print(
                f"{name}: {summary['successes']}/{summary['episodes']} "
                f"({summary['success_rate']:.1%}), 95% CI "
                f"{summary['success_rate_ci_95']}"
            )
    return 0


def _recording_validate(paths: list[Path]) -> tuple[list[dict[str, object]], bool]:
    from dataclasses import asdict

    from act_lab.adapters.mcap.reading import read_episode
    from act_lab.application.dataset import validate_episode

    reports = [asdict(validate_episode(read_episode(path))) for path in paths]
    return reports, all(bool(report["valid"]) for report in reports)


def _recording_manifest(args: argparse.Namespace) -> dict[str, object]:
    from dataclasses import asdict

    from act_lab.adapters.mcap.reading import read_episode
    from act_lab.application.dataset import (
        CONVERTER_VERSION,
        file_sha256,
        split_for_episode,
        validate_episode,
    )
    from act_lab.application.experiment import episode_requested, parse_seed_ranges

    if args.output.exists():
        raise FileExistsError(f"manifest already exists: {args.output}")
    entries: list[dict[str, object]] = []
    episode_ids: set[str] = set()
    seed_ranges = parse_seed_ranges(args.include_seed_range)
    for path in sorted(args.paths, key=lambda item: str(item)):
        episode = read_episode(path)
        report = validate_episode(episode)
        if episode.episode_id and episode.episode_id in episode_ids:
            raise ValueError(f"duplicate episode ID: {episode.episode_id}")
        episode_ids.add(episode.episode_id)
        seed_value = episode.provenance.get("seed")
        seed = seed_value if type(seed_value) is int else -1
        requested = episode_requested(
            episode.episode_id,
            seed,
            str(episode.provenance.get("source", "")),
            args.include_episode,
            seed_ranges,
            args.include_source,
        )
        selected = report.training_eligible and requested
        entries.append(
            {
                "episode_id": episode.episode_id,
                "path": os.path.relpath(path.resolve(), args.output.parent.resolve()),
                "sha256": file_sha256(path),
                "selected": selected,
                "selection_reason": (
                    "selected"
                    if selected
                    else "quality_or_outcome_rejected"
                    if not report.training_eligible
                    else "not_requested"
                ),
                "split": (
                    split_for_episode(
                        episode.episode_id,
                        args.split_seed,
                        args.validation_fraction,
                    )
                    if selected
                    else None
                ),
                "quality": asdict(report),
            }
        )
    unknown_ids = set(args.include_episode) - episode_ids
    if unknown_ids:
        raise ValueError(
            "requested episode IDs were not found: " + ", ".join(sorted(unknown_ids))
        )
    manifest: dict[str, object] = {
        "manifest_version": 1,
        "converter_version": CONVERTER_VERSION,
        "split": {
            "method": "sha256(seed:episode_id)",
            "seed": args.split_seed,
            "validation_fraction": args.validation_fraction,
            "unit": "episode",
        },
        "selection": {
            "episode_ids": args.include_episode,
            "seed_ranges_start_count": args.include_seed_range,
            "source": args.include_source,
        },
        "episodes": entries,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    partial = args.output.with_name(args.output.name + ".partial")
    partial.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    os.replace(partial, args.output)
    return {
        "manifest": str(args.output),
        "candidates": len(entries),
        "selected": sum(bool(entry["selected"]) for entry in entries),
        "rejected": sum(not bool(entry["selected"]) for entry in entries),
    }


def _recording_replay(args: argparse.Namespace) -> dict[str, object]:
    from act_lab.adapters.mcap.reading import read_episode

    episode = read_episode(args.path)
    if args.max_frames is not None and args.max_frames <= 0:
        raise ValueError("max-frames must be positive")
    camera_ids = sorted(
        {image.camera_id for sample in episode.samples for image in sample.images}
    )
    camera = args.camera or (camera_ids[0] if camera_ids else None)
    if camera is not None and camera not in camera_ids:
        raise ValueError(f"camera {camera!r} not found; available: {camera_ids}")
    frames = []
    for sample in episode.samples:
        image = next((item for item in sample.images if item.camera_id == camera), None)
        if image is not None:
            frames.append((sample.timestamp_ns, image))
    if args.max_frames is not None:
        frames = frames[: args.max_frames]
    if args.render_dir is not None:
        import numpy as np

        args.render_dir.mkdir(parents=True, exist_ok=True)
        for index, (_, image) in enumerate(frames):
            pixels = np.frombuffer(image.rgb_bytes, dtype=np.uint8).reshape(
                image.height, image.width, 3
            )
            _write_ppm(args.render_dir / f"{index:06d}.ppm", pixels)
    if args.display:
        import cv2
        import numpy as np

        for index, (timestamp_ns, image) in enumerate(frames):
            pixels = np.frombuffer(image.rgb_bytes, dtype=np.uint8).reshape(
                image.height, image.width, 3
            )
            cv2.imshow(
                f"ACT Lab replay: {camera}", cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
            )
            next_timestamp_ns = (
                frames[index + 1][0] if index + 1 < len(frames) else timestamp_ns
            )
            delay_ms = max(
                1, min(1000, round((next_timestamp_ns - timestamp_ns) / 1e6))
            )
            if cv2.waitKey(delay_ms) & 0xFF in (ord("q"), 27):
                break
        cv2.destroyAllWindows()
    return {
        "episode_id": episode.episode_id,
        "outcome": episode.outcome,
        "samples": len(episode.samples),
        "camera": camera,
        "camera_frames": len(frames),
        "first_timestamp_ns": episode.samples[0].timestamp_ns
        if episode.samples
        else None,
        "last_timestamp_ns": episode.samples[-1].timestamp_ns
        if episode.samples
        else None,
    }


def _recording_convert(args: argparse.Namespace) -> dict[str, object]:
    from act_lab.adapters.lerobot import convert_episodes
    from act_lab.adapters.mcap.reading import read_episode
    from act_lab.application.dataset import file_sha256, validate_episode

    manifest = json.loads(args.manifest.read_text())
    if manifest.get("manifest_version") != 1:
        raise ValueError("unsupported selection manifest version")
    episodes = []
    selected_entries = [entry for entry in manifest["episodes"] if entry["selected"]]
    for entry in selected_entries:
        path = (args.manifest.parent / entry["path"]).resolve()
        if file_sha256(path) != entry["sha256"]:
            raise ValueError(f"raw episode changed since selection: {path}")
        episode = read_episode(path)
        report = validate_episode(episode)
        if episode.episode_id != entry["episode_id"] or not report.training_eligible:
            raise ValueError(f"episode no longer passes selection: {path}")
        episodes.append(episode)
    return convert_episodes(
        episodes,
        args.output,
        args.repo_id,
        args.fps,
        {"selection_manifest": manifest},
    )


def _train_act(args: argparse.Namespace) -> int:
    from act_lab.adapters.lerobot import train_act
    from act_lab.application.training import (
        RUN_MANIFEST,
        atomic_json,
        environment_identity,
        git_identity,
        load_act_config,
        preflight_device,
        utc_now,
        verify_dataset,
    )

    try:
        preflight_device(args.device)
        config = load_act_config(args.config)
        dataset = verify_dataset(args.dataset)
        output = args.output.resolve()
        if output.exists():
            raise FileExistsError(f"run output already exists: {output}")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.mkdir()
        manifest: dict[str, object] = {
            "manifest_version": 1,
            "policy": "act",
            "status": "running",
            "created_at": utc_now(),
            "updated_at": utc_now(),
            "completed_at": None,
            "failure_reason": None,
            "device": args.device,
            "wandb": {"mode": args.wandb_mode, "run_id": None},
            "configuration": config.to_dict(),
            "config_source": str(args.config.resolve()),
            "dataset": dataset.to_dict(),
            "git": git_identity(Path.cwd()),
            "environment": environment_identity(),
            "reproducibility_note": (
                "Fixed seeds and deterministic cuDNN reduce variance, but PyTorch does "
                "not guarantee identical results across releases, platforms, or "
                "CPU/GPU."
            ),
        }
        atomic_json(output / RUN_MANIFEST, manifest)
        try:
            train_act(output, dataset, config, args.device, args.wandb_mode)
        except BaseException as error:
            manifest.update(
                status="failed",
                updated_at=utc_now(),
                failure_reason=f"{type(error).__name__}: {error}",
            )
            atomic_json(output / RUN_MANIFEST, manifest)
            raise
        manifest.update(
            status="completed", updated_at=utc_now(), completed_at=utc_now()
        )
        atomic_json(output / RUN_MANIFEST, manifest)
        print(
            json.dumps(
                {"run": str(output), "status": "completed"}, indent=2, sort_keys=True
            )
        )
        return 0
    except (OSError, RuntimeError, TypeError, ValueError) as error:
        print(f"training error: {error}", file=sys.stderr)
        return 2


def _train_resume(args: argparse.Namespace) -> int:
    from act_lab.adapters.lerobot import checkpoint_step, latest_checkpoint, train_act
    from act_lab.application.training import (
        RUN_MANIFEST,
        atomic_json,
        preflight_device,
        resume_config,
        utc_now,
        verify_dataset,
    )
    from act_lab.domain.training import ActTrainingConfig

    try:
        preflight_device(args.device)
        run = args.run.resolve()
        manifest_path = run / RUN_MANIFEST
        if not manifest_path.is_file():
            raise ValueError(f"not an ACT Lab training run: {run}")
        manifest = json.loads(manifest_path.read_text())
        if manifest.get("manifest_version") != 1 or manifest.get("policy") != "act":
            raise ValueError("unsupported or non-ACT run manifest")
        if manifest.get("device") != args.device:
            raise ValueError("resume device type must match the original run")
        original = ActTrainingConfig(**manifest["configuration"])
        config = resume_config(original, args.steps)
        dataset = verify_dataset(Path(manifest["dataset"]["path"]))
        if dataset.to_dict() != manifest["dataset"]:
            raise ValueError("dataset identity changed since the original run")
        checkpoint = latest_checkpoint(run)
        step = checkpoint_step(checkpoint)
        if config.steps <= step:
            raise ValueError(
                f"target steps {config.steps} must exceed checkpoint step {step}"
            )
        wandb = manifest.get("wandb", {})
        wandb_mode = str(wandb.get("mode", "disabled"))
        manifest.update(
            status="running",
            updated_at=utc_now(),
            completed_at=None,
            failure_reason=None,
            configuration=config.to_dict(),
            resumed_from_step=step,
        )
        atomic_json(manifest_path, manifest)
        try:
            train_act(
                run, dataset, config, args.device, wandb_mode, checkpoint=checkpoint
            )
        except BaseException as error:
            manifest.update(
                status="failed",
                updated_at=utc_now(),
                failure_reason=f"{type(error).__name__}: {error}",
            )
            atomic_json(manifest_path, manifest)
            raise
        manifest.update(
            status="completed", updated_at=utc_now(), completed_at=utc_now()
        )
        atomic_json(manifest_path, manifest)
        print(
            json.dumps(
                {"run": str(run), "status": "completed"}, indent=2, sort_keys=True
            )
        )
        return 0
    except (KeyError, OSError, RuntimeError, TypeError, ValueError) as error:
        print(f"training resume error: {error}", file=sys.stderr)
        return 2


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "recording":
        from mcap.exceptions import McapError

        from act_lab.adapters.mcap.inspection import inspect_episode, recover_episode

        exit_code = 0
        try:
            if args.recording_command == "inspect":
                result = inspect_episode(args.path)
            elif args.recording_command == "recover":
                result = {"recovered_path": str(recover_episode(args.path))}
            elif args.recording_command == "validate":
                reports, valid = _recording_validate(args.paths)
                result = {"valid": valid, "reports": reports}
                exit_code = 0 if valid else 1
            elif args.recording_command == "manifest":
                result = _recording_manifest(args)
            elif args.recording_command == "replay":
                result = _recording_replay(args)
            elif args.recording_command == "foxglove":
                from act_lab.adapters.mcap.foxglove import export_foxglove

                result = export_foxglove(args.path, args.output)
            else:
                result = _recording_convert(args)
            print(json.dumps(result, indent=2, sort_keys=True))
        except (
            KeyError,
            OSError,
            RuntimeError,
            TypeError,
            ValueError,
            McapError,
        ) as error:
            print(f"recording error: {error}", file=sys.stderr)
            return 2
        return exit_code
    if args.command == "ros2":
        from act_lab.adapters.ros2.smoke import run_smoke

        try:
            result = run_smoke()
        except (RuntimeError, ValueError, ImportError) as error:
            print(str(error), file=sys.stderr)
            return 1
        print(
            json.dumps(result, indent=2)
            if args.json
            else f"ROS 2 contracts: {result['status']} ({len(result['cases'])} cases)"
        )
        return 0
    if args.command == "doctor":
        return _doctor(as_json=args.json)
    if args.command == "experiment":
        return _experiment(args)
    if args.command == "train" and args.train_command == "act":
        return _train_act(args)
    if args.command == "train" and args.train_command == "resume":
        return _train_resume(args)
    if args.command == "sim" and args.sim_command == "rollout":
        return _sim_rollout(args)
    if args.command == "sim" and args.sim_command == "control-smoke":
        return _sim_control_smoke(args)
    if args.command == "sim" and args.sim_command == "control-diagnostic":
        return _sim_control_diagnostic(args)
    if args.command == "sim" and args.sim_command == "view":
        return _sim_view(args)
    if args.command == "sim" and args.sim_command == "keyboard-teleop":
        return _sim_keyboard_teleop(args)
    if args.command == "sim" and args.sim_command == "webcam-teleop":
        return _sim_webcam_teleop(args)
    if args.command == "sim" and args.sim_command == "expert":
        return _sim_expert(args)
    if args.command == "sim" and args.sim_command == "evaluate":
        return _sim_evaluate(args)
    raise AssertionError(f"unhandled command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
