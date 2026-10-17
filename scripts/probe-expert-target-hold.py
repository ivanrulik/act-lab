"""Diagnostic oracle: hold requested versus first applied target at 25 Hz."""

import argparse
import json
from dataclasses import replace
from pathlib import Path

from act_lab.adapters.mujoco import MujocoCartesianDriver, SimulationConfig
from act_lab.application import SafeCartesianRobot, ScriptedPickPlaceExpert
from act_lab.domain import Action

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--seed-start", type=int, default=21000)
parser.add_argument("--episodes", type=int, default=20)
args = parser.parse_args()
if args.output.exists():
    parser.error("output already exists")
if args.episodes <= 0:
    parser.error("episodes must be positive")
path = Path("configs/sim/ur5e_2f85_d405_paired.toml")
config = SimulationConfig.load(path)
rows = []
for seed in range(args.seed_start, args.seed_start + args.episodes):
    for representation in ["requested", "first-applied"]:
        with MujocoCartesianDriver.from_config_file(path) as driver:
            env = driver.environment
            robot = SafeCartesianRobot(driver, driver.limits)
            obs = robot.reset(seed)
            expert = ScriptedPickPlaceExpert(env, config.expert)
            expert.reset(obs)
            rejections = 0
            phases = []
            for step in range(config.episode_steps):
                if step % 2 == 0:
                    held = expert.poll(obs)
                    if not phases or phases[-1]["phase"] != expert.phase.value:
                        phases.append(
                            {
                                "phase": expert.phase.value,
                                "time_s": obs.timestamp_ns / 1e9,
                            }
                        )
                robot.command(replace(held, timestamp_ns=obs.timestamp_ns))
                report = robot.last_report
                expert.record_command_report(report)
                if step % 2 == 0 and representation == "first-applied":
                    assert report.executed_action is not None, report
                    held = report.executed_action
                rejections += report.outcome.value in [
                    "collision_stop",
                    "ik_failure",
                    "invalid",
                    "stale",
                ]
                obs = robot.observe()
                task = env.task_state()
                if task.terminal or expert.failure_reason:
                    break
            row = {
                "seed": seed,
                "representation": representation,
                "success": task.success,
                "reason": task.reason,
                "failure_reason": expert.failure_reason,
                "steps": step + 1,
                "phase": expert.phase.value,
                "tcp_xyz": list(obs.robot.end_effector_pose.position_xyz_m),
                "safety_rejections": rejections,
                "phases": phases,
            }
            rows.append(row)
            print(json.dumps(row), flush=True)
            robot.command(
                Action(
                    obs.timestamp_ns,
                    obs.robot.end_effector_pose,
                    obs.robot.gripper_position,
                    False,
                )
            )
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(
    json.dumps(
        {
            "schema_version": 1,
            "control_hz": 50,
            "decision_hz": 25,
            "description": (
                "Oracle expert requests vs first applied target held across "
                "two safety ticks; no learned policy or rendering"
            ),
            "episodes": rows,
        },
        indent=2,
    )
)
