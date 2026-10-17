"""Conditional expert-to-policy handoffs on two fixed diagnostic seeds."""

import argparse
import json
from dataclasses import replace
from pathlib import Path

from act_lab.adapters.lerobot.policy import LeRobotACTPolicy
from act_lab.adapters.mujoco import MujocoCartesianDriver, SimulationConfig
from act_lab.application import SafeCartesianRobot, ScriptedPickPlaceExpert
from act_lab.domain import Action

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--pilot", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    parser.error("output already exists")
args.output.parent.mkdir(parents=True, exist_ok=True)
config_path = Path("configs/sim/ur5e_2f85_d405_paired.toml")
config = SimulationConfig.load(config_path)
checkpoint = args.pilot / "paired/training/lerobot/checkpoints/005000/pretrained_model"
contract = json.loads((checkpoint / "act_lab_model.json").read_text())
policy = LeRobotACTPolicy(checkpoint, "cuda", config.cameras, model_contract=contract)
rows = []
for seed in [21000, 21001]:
    for phase in [
        "open_raise",
        "approach_cube",
        "descend",
        "close_dwell",
        "lift",
        "transit",
        "lower",
        "open_dwell",
    ]:
        with MujocoCartesianDriver.from_config_file(config_path) as driver:
            env = driver.environment
            robot = SafeCartesianRobot(driver, driver.limits)
            obs = robot.reset(seed)
            expert = ScriptedPickPlaceExpert(env, config.expert)
            expert.reset(obs)
            for _prefix in range(config.episode_steps):
                action = expert.poll(obs)
                if expert.phase.value == phase:
                    break
                robot.command(action)
                expert.record_command_report(robot.last_report)
                obs = robot.observe()
                if env.task_state().terminal or expert.failure_reason:
                    raise RuntimeError("expert prefix failed")
            else:
                raise RuntimeError("phase not reached")
            policy.reset()
            before = obs.robot.end_effector_pose.position_xyz_m
            rejections = 0
            minimum_z = before[2]
            first = None
            for step in range(config.episode_steps - _prefix):
                if step % 2 == 0:
                    held = policy.act(
                        obs,
                        {c: env.render(c) for c in config.cameras},
                        capture_timestamp_ns=obs.timestamp_ns,
                    )
                    if first is None:
                        first = {
                            "target": list(held.target_pose.position_xyz_m),
                            "gripper": held.gripper_position,
                        }
                robot.command(replace(held, timestamp_ns=obs.timestamp_ns))
                obs = robot.observe()
                task = env.task_state()
                minimum_z = min(
                    minimum_z, obs.robot.end_effector_pose.position_xyz_m[2]
                )
                rejections += robot.last_report.outcome.value in [
                    "collision_stop",
                    "ik_failure",
                    "invalid",
                    "stale",
                ]
                if task.terminal:
                    break
            row = {
                "seed": seed,
                "phase": phase,
                "prefix_steps": _prefix,
                "conditional_success": task.success,
                "reason": task.reason,
                "minimum_tcp_z_m": minimum_z,
                "start_tcp_xyz": list(before),
                "final_tcp_xyz": list(obs.robot.end_effector_pose.position_xyz_m),
                "first_prediction": first,
                "safety_rejections": rejections,
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
        args.output.write_text(json.dumps(rows, indent=2))
