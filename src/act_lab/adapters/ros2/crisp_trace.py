"""ROS-free, test-only authorization traces; never a production effort adapter."""

from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path
from typing import Any
from uuid import UUID

from act_lab.adapters.ros2.contracts import CLOCK_OFFSET_NS, CommandEnvelope
from act_lab.adapters.ros2.demonstrator import (
    ZERO,
    DemonstratorLimits,
    DeterministicDriver,
)
from act_lab.adapters.ros2.inbox import CommandInbox
from act_lab.application import SafeCartesianRobot
from act_lab.domain import Action, CommandOutcome, Pose, RobotState

SCHEMA_VERSION = 1
HOME_JOINTS = (-0.035074, -1.756617, -0.595185, -2.360379, 1.570771, 4.677315)


class StationaryDriver(DeterministicDriver):
    """No integration: real-model FK pose supplied by the C++ bench."""

    joint_position_bounds_rad = ((-2 * math.pi, 2 * math.pi),) * 6

    def __init__(self, pose: Pose) -> None:
        super().__init__()
        self.pose = pose
        self.ik_result = HOME_JOINTS
        self.reset(0)

    def reset(self, seed: int) -> Any:
        del seed
        self.state = RobotState(0, HOME_JOINTS, ZERO, self.pose, 0.3)
        return self.observe()

    def step(self, joints: Any, joint_velocity: Any, gripper: float) -> RobotState:
        del joints, joint_velocity
        self.state = replace(self.state, gripper_position=gripper)
        return self.state


def generate_trace(pose: Pose) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Generate approved intent through the existing shared safety path.

    Adversarial authorizations are explicitly marked, to test the independent
    gate against a compromised/stopped approval process. They are not approvals.
    """
    events: list[dict[str, Any]] = []
    approvals: list[dict[str, Any]] = []
    steady = 0
    episode_index = 0
    episode = ""
    domain = 0
    sequence = 0
    driver = StationaryDriver(pose)
    robot = SafeCartesianRobot(driver, DemonstratorLimits())
    inbox = CommandInbox()
    robot.reset(0)

    def clock(value: int, advance: int = 0, *, reset: bool = False) -> None:
        nonlocal domain, steady, episode, episode_index, sequence
        steady += advance
        domain = value
        if reset:
            episode_index += 1
            episode = str(UUID(int=episode_index))
            sequence = 0
        events.append(
            dict(kind="clock", steady_ns=steady, domain_ns=domain, episode_id=episode)
        )
        inbox.update_clock(domain + CLOCK_OFFSET_NS, episode, steady)
        driver.state = replace(driver.state, timestamp_ns=domain)

    def command(**changes: Any) -> dict[str, Any]:
        nonlocal sequence
        sequence += 1
        target = replace(
            pose,
            position_xyz_m=(pose.position_xyz_m[0] + 0.001, *pose.position_xyz_m[1:]),
        )
        intent = Action(domain, target, 0.4, True)
        envelope = CommandEnvelope(episode, sequence, intent)
        inbox.offer(envelope, steady)
        robot.command(inbox.poll(driver.observe(), steady))
        report = robot.last_report
        assert report is not None and report.executed_action is not None
        executed = report.executed_action
        assert report.outcome in (CommandOutcome.APPLIED, CommandOutcome.LIMITED)
        result: dict[str, Any] = dict(
            episode_id=episode,
            sequence=sequence,
            enabled=executed.enabled,
            source_timestamp_ns=executed.timestamp_ns,
            receipt_steady_ns=steady,
            frame_id=executed.target_pose.frame_id,
            position=list(executed.target_pose.position_xyz_m),
            quaternion_wxyz=list(executed.target_pose.quaternion_wxyz),
            gripper_position=executed.gripper_position,
        )
        approvals.append(
            dict(
                outcome=report.outcome.value,
                command=result.copy(),
                measured_gripper=report.resulting_state.gripper_position,
            )
        )
        result.update(changes)
        events.append(
            dict(
                kind="authorize",
                steady_ns=steady,
                command=result,
                adversarial=bool(changes),
            )
        )
        return result

    def step(case: str, enabled: bool, **extra: Any) -> None:
        events.append(
            dict(
                kind="step",
                steady_ns=steady,
                domain_ns=domain,
                case=case,
                expected_enabled=enabled,
                **extra,
            )
        )

    def fresh() -> None:
        clock(0, 200_000_000, reset=True)
        command()

    clock(0, reset=True)
    step("startup_missing", False)
    command()
    step("approved", True)
    clock(99_999_999, 1)
    step("source_before_boundary", True)
    clock(100_000_000, 1)
    step("source_exact_boundary", False)

    fresh()
    clock(1, 99_999_999)
    step("receipt_before_boundary", True)
    clock(2, 1)
    step("receipt_exact_boundary_gateway_loss", False)

    fresh()
    # Frequent upstream approval cannot mask a paused independent clock.
    clock(0, 99_999_999)
    command()
    step("pause_before_boundary", True)
    clock(0, 1)
    step("pause_exact_boundary", False)
    clock(1, 1)
    step("resume_requires_fresh", False)
    command()
    step("pause_recovery_reactivated", True)

    fresh()
    clock(2, 1)
    clock(1, 1)
    step("backward_reset", False)
    rejected = events[
        next(
            i
            for i in range(len(events) - 1, -1, -1)
            if events[i]["kind"] == "authorize"
        )
    ]["command"].copy()
    rejected["sequence"] += 1
    events.append(
        dict(kind="authorize", steady_ns=steady, command=rejected, adversarial=True)
    )
    step("same_episode_reset_rejected", False)
    fresh()
    step("new_episode_recovery", True)
    clock(500_000_000, 1)
    step("forward_jump_stale", False)

    for case, changes in (
        ("replay", dict(sequence=1)),
        ("wrong_episode", dict(episode_id=str(UUID(int=999)))),
        ("future_timestamp", dict(source_timestamp_ns=1)),
        ("invalid_frame", dict(frame_id="camera")),
        ("invalid_numeric", dict(position=[None, 0.0, 0.6])),
        ("non_unit_quaternion", dict(quaternion_wxyz=[2.0, 0.0, 0.0, 0.0])),
        ("malformed_shape", dict(position=[0.0, 0.6])),
        ("invalid_sequence", dict(sequence=-1)),
        ("disabled", dict(enabled=False)),
    ):
        fresh()
        command(**changes)
        step(case, False)
        command()
        step(case + "_recovery", True)

    fresh()
    step("nonfinite_raw_effort", False, inject_nonfinite=True)
    command()
    step("numeric_recovery", True)
    fresh()
    for i in range(30):
        clock(i * 2_000_000, 2_000_000)
        step(f"ceiling_slew_{i}", True, inject_large=True)
    events.append(dict(kind="fault", steady_ns=steady, reason="shutdown"))
    step("shutdown_immediate_zero", False)

    # Disabled holds preserve the last accepted aperture through shared safety.
    accepted = driver.state.gripper_position
    clock(200_000_000, 1)
    for case, invalid in (
        (
            "application_frame",
            Action(domain, replace(pose, frame_id="camera"), 0.0, True),
        ),
        (
            "application_numeric",
            Action(
                domain, replace(pose, position_xyz_m=(math.nan, 0.0, 0.6)), 0.0, True
            ),
        ),
        ("application_future", Action(domain + 1, pose, 0.0, True)),
        ("application_stale", Action(domain - 100_000_000, pose, 0.0, True)),
    ):
        sequence += 1
        inbox.offer(CommandEnvelope(episode, sequence, invalid), steady)
        robot.command(inbox.poll(driver.observe(), steady))
        report = robot.last_report
        assert report is not None and report.outcome in (
            CommandOutcome.INVALID,
            CommandOutcome.STALE,
        )
        assert driver.state.gripper_position == accepted
        approvals.append(
            dict(
                case=case,
                outcome=report.outcome.value,
                measured_gripper=driver.state.gripper_position,
                source_timestamp_ns=invalid.timestamp_ns,
            )
        )
    robot.command(Action(domain, pose, 0.0, False))
    assert driver.state.gripper_position == accepted
    approvals.append(dict(outcome="disabled", retained_gripper=accepted))
    validate_trace(events)
    return events, approvals


def validate_trace(events: list[dict[str, Any]]) -> None:
    """Validate schema/shape, permitting marked adversarial payloads as evidence."""
    previous = 0
    cases: set[str] = set()
    for event in events:
        steady = event["steady_ns"]
        if type(steady) is not int or not previous <= steady < 2**63:
            raise ValueError("trace steady clock must be monotonic int64")
        previous = steady
        kind = event["kind"]
        if kind in ("clock", "step"):
            domain = event["domain_ns"]
            if type(domain) is not int or not 0 <= domain < 2**63 - CLOCK_OFFSET_NS:
                raise ValueError("invalid domain timestamp")
        if kind == "clock":
            if str(UUID(event["episode_id"])) != event["episode_id"]:
                raise ValueError("invalid episode")
        elif kind == "authorize":
            cmd = event["command"]
            if str(UUID(cmd["episode_id"])) != cmd["episode_id"]:
                raise ValueError("invalid command episode")
            adversarial = event.get("adversarial", False)
            if not adversarial and (
                type(cmd["sequence"]) is not int or not 0 <= cmd["sequence"] < 2**64
            ):
                raise ValueError("invalid sequence")
            if type(cmd["enabled"]) is not bool or not isinstance(cmd["frame_id"], str):
                raise ValueError("invalid command flags")
            if (
                type(cmd["receipt_steady_ns"]) is not int
                or not 0 <= cmd["receipt_steady_ns"] <= steady
            ):
                raise ValueError("invalid receipt timestamp")
            if type(cmd["source_timestamp_ns"]) is not int:
                raise ValueError("invalid source timestamp")
            if not adversarial and (
                len(cmd["position"]) != 3 or len(cmd["quaternion_wxyz"]) != 4
            ):
                raise ValueError("invalid pose shape")
            if not event.get("adversarial", False):
                values = [
                    *cmd["position"],
                    *cmd["quaternion_wxyz"],
                    cmd["gripper_position"],
                ]
                if not all(
                    type(v) in (int, float) and math.isfinite(v) for v in values
                ):
                    raise ValueError("invalid approved numeric payload")
        elif kind == "step":
            if type(event["expected_enabled"]) is not bool or event["case"] in cases:
                raise ValueError("invalid/duplicate assertion")
            cases.add(event["case"])
        elif kind == "fault":
            if not isinstance(event["reason"], str):
                raise ValueError("invalid rejection reason")
        else:
            raise ValueError("unknown trace event")
    if not cases:
        raise ValueError("trace has no assertions")


def write_trace(path: Path, events: list[dict[str, Any]]) -> None:
    validate_trace(events)
    path.write_text("".join(json.dumps(e, allow_nan=False) + "\n" for e in events))
