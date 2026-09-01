"""Stable command-line entry point for all ACT Lab workflows."""

from __future__ import annotations

import argparse
import json
import platform
import sys
from collections.abc import Sequence
from pathlib import Path

from act_lab import __version__


def _doctor(as_json: bool) -> int:
    report = {
        "act_lab_version": __version__,
        "python": platform.python_version(),
        "python_supported": sys.version_info[:2] in {(3, 11), (3, 12)},
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
    return parser


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


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "doctor":
        return _doctor(as_json=args.json)
    if args.command == "sim" and args.sim_command == "rollout":
        return _sim_rollout(args)
    if args.command == "sim" and args.sim_command == "control-smoke":
        return _sim_control_smoke(args)
    if args.command == "sim" and args.sim_command == "view":
        return _sim_view(args)
    raise AssertionError(f"unhandled command: {args.command}")
