from __future__ import annotations

import copy
import json
import subprocess
import sys
from dataclasses import replace

import pytest

from act_lab.adapters.ros2.contracts import (
    CLOCK_OFFSET_NS,
    JOINT_NAMES,
    CommandEnvelope,
    decode_command,
    decode_pose,
    decode_report,
    decode_stamp,
    decode_state,
    encode_command,
    encode_pose,
    encode_report,
    encode_stamp,
    encode_state,
)
from act_lab.adapters.ros2.demonstrator import HOME, make_robot
from act_lab.adapters.ros2.inbox import CommandInbox
from act_lab.domain import (
    Action,
    CommandOutcome,
    CommandReport,
    Observation,
    RobotState,
)

EPISODE = "c3318069-22c5-4af3-9df0-56b43ca3df1e"
OTHER = "270ff1c8-2e2e-49d3-a3f0-cb703cd8f66b"


def observation(timestamp: int = 0) -> Observation:
    state = RobotState(timestamp, (0.0, 1.0, 2.0, 3.0, 4.0, 5.0), (0.0,) * 6, HOME, 0.3)
    return Observation(timestamp, state, ())


def envelope(
    sequence: int = 1, timestamp: int = 0, episode: str = EPISODE
) -> CommandEnvelope:
    return CommandEnvelope(episode, sequence, Action(timestamp, HOME, 0.6, True))


def inbox() -> CommandInbox:
    result = CommandInbox()
    result.update_clock(CLOCK_OFFSET_NS, EPISODE, 0)
    return result


@pytest.mark.parametrize(
    "timestamp", [-1, 0, 1, 999_999_999, 1_000_000_000, 123_456_789_123]
)
def test_stamp_roundtrip(timestamp: int) -> None:
    assert decode_stamp(encode_stamp(timestamp)) == timestamp
    assert encode_stamp(0) == {"sec": 1, "nanosec": 0}


def test_pose_and_intent_roundtrip_does_not_normalize_or_transform() -> None:
    pose = replace(HOME, frame_id="camera", quaternion_wxyz=(2.0, 3.0, 4.0, 5.0))
    wire = encode_pose(pose)
    assert wire["orientation"] == {"x": 3.0, "y": 4.0, "z": 5.0, "w": 2.0}
    assert decode_pose(wire, "camera") == pose
    original = replace(envelope(), action=Action(123, pose, float("inf"), False))
    assert decode_command(encode_command(original)) == original


def test_state_joint_ordering_and_roundtrip() -> None:
    state = observation().robot
    fields = encode_state(state, EPISODE)
    for key in ("joint_names", "joint_positions_rad", "joint_velocities_rad_s"):
        fields[key].reverse()
    assert decode_state(fields) == (state, EPISODE)
    for bad in ([JOINT_NAMES[0]] * 6, list(JOINT_NAMES[:5]), ["unknown"] * 6):
        fields["joint_names"] = bad
        with pytest.raises(ValueError):
            decode_state(fields)


@pytest.mark.parametrize("executed", [None, Action(20, HOME, 0.4, True)])
@pytest.mark.parametrize("outcome", list(CommandOutcome))
def test_report_preserves_every_field(
    executed: Action | None, outcome: CommandOutcome
) -> None:
    report = CommandReport(
        outcome,
        envelope().action,
        executed,
        observation(40).robot,
        "specific detail",
        0.1,
        0.2,
        0.3,
        0.4,
    )
    assert decode_report(encode_report(report, EPISODE, 2**64 - 1)) == (
        report,
        EPISODE,
        2**64 - 1,
    )


@pytest.mark.parametrize(
    "mutate",
    [
        lambda f: f.update(sequence=-1),
        lambda f: f.update(sequence=2**64),
        lambda f: f.update(sequence=True),
        lambda f: f.update(episode_id="bad"),
        lambda f: f.update(enabled=1),
        lambda f: f["header"]["stamp"].update(nanosec=1_000_000_000),
        lambda f: f["target_pose"]["position"].update(x="bad"),
    ],
)
def test_malformed_wire_command(mutate: object) -> None:
    fields = encode_command(envelope())
    mutate(fields)  # type: ignore[operator]
    with pytest.raises(ValueError):
        decode_command(fields)


def test_report_rejects_inconsistent_metadata() -> None:
    robot, _ = make_robot()
    robot.command(envelope().action)
    assert robot.last_report is not None
    original = encode_report(robot.last_report, EPISODE, 1)
    for key, value in (
        ("episode_id", OTHER),
        ("command_sequence", 2),
        ("has_executed_command", 1),
    ):
        fields = copy.deepcopy(original)
        fields[key] = value
        with pytest.raises(ValueError):
            decode_report(fields)


def test_missing_and_uninitialized_clock_hold_then_initialize() -> None:
    box = CommandInbox()
    assert not box.poll(observation(), 0).enabled
    box.update_clock(0, EPISODE, 0)
    assert not box.offer(envelope(), 0)
    box.update_clock(CLOCK_OFFSET_NS, EPISODE, 1)
    assert box.offer(envelope(), 1)
    assert box.poll(observation(), 1).enabled
    with pytest.raises(ValueError, match="use_sim_time"):
        CommandInbox(use_sim_time=False)


def test_repeated_polls_keep_source_stamp_and_exact_stale_boundary() -> None:
    box = inbox()
    assert box.offer(envelope(), 0)
    for now in (1, 99_999_999, 100_000_000):
        box.update_clock(CLOCK_OFFSET_NS + now, EPISODE, now)
        action = box.poll(observation(now), now)
        assert action.timestamp_ns == 0
        robot, driver = make_robot()
        driver.state = replace(driver.state, timestamp_ns=now)
        robot.command(action)
        assert robot.last_report is not None
        assert robot.last_report.outcome == (
            CommandOutcome.STALE if now == 100_000_000 else CommandOutcome.LIMITED
        )


@pytest.mark.parametrize(
    "bad, reason",
    [
        (envelope(1), "out_of_order"),
        (envelope(0), "out_of_order"),
        (envelope(2, episode=OTHER), "wrong_episode"),
        (envelope(-1), "malformed_envelope"),
        (envelope(2, timestamp=-1), "pre_episode_timestamp"),
    ],
)
def test_transport_rejections_clear_intent(bad: CommandEnvelope, reason: str) -> None:
    box = inbox()
    box.offer(envelope(), 0)
    assert not box.offer(bad, 1)
    hold = box.poll(observation(), 1)
    assert not hold.enabled and hold.target_pose == HOME
    assert box.last_rejection_reason == reason


def test_future_once_then_cleared_and_no_timestamp_refresh() -> None:
    box = inbox()
    box.offer(envelope(timestamp=1), 0)
    assert box.poll(observation(), 0).timestamp_ns == 1
    assert box.last_rejection_reason == "future_timestamp"
    box.update_clock(CLOCK_OFFSET_NS + 1, EPISODE, 1)
    assert not box.poll(observation(1), 1).enabled


def test_receipt_watchdog_independent_of_ros_age() -> None:
    box = inbox()
    box.offer(envelope(), 0)
    box.update_clock(CLOCK_OFFSET_NS + 20_000_000, EPISODE, 99_000_000)
    assert box.poll(observation(20_000_000), 99_999_999).enabled
    assert not box.poll(observation(20_000_000), 100_000_000).enabled
    assert box.last_rejection_reason == "receipt_timeout"


def test_clock_pause_with_live_packets_and_resume_requires_fresh_sequence() -> None:
    box = inbox()
    box.offer(envelope(), 0)
    box.offer(envelope(2), 99_999_999)
    assert box.poll(observation(), 99_999_999).enabled
    assert not box.poll(observation(), 100_000_000).enabled
    assert box.last_rejection_reason == "clock_paused"
    box.update_clock(CLOCK_OFFSET_NS + 1, EPISODE, 100_000_001)
    assert not box.poll(observation(1), 100_000_001).enabled
    assert not box.offer(envelope(2, 1), 100_000_001)
    assert box.offer(envelope(3, 1), 100_000_001)
    assert box.poll(observation(1), 100_000_001).enabled


def test_pause_detected_on_resume_without_poll_during_pause() -> None:
    box = inbox()
    box.offer(envelope(), 0)
    box.offer(envelope(2), 100_000_000)
    box.update_clock(CLOCK_OFFSET_NS + 1, EPISODE, 100_000_001)
    assert not box.poll(observation(1), 100_000_001).enabled
    assert box.offer(envelope(3, 1), 100_000_001)
    assert box.poll(observation(1), 100_000_001).enabled


def test_backward_clock_requires_new_epoch_and_forward_jump_ages_command() -> None:
    box = inbox()
    box.update_clock(CLOCK_OFFSET_NS + 20, EPISODE, 1)
    box.offer(envelope(timestamp=20), 1)
    box.update_clock(CLOCK_OFFSET_NS, EPISODE, 2)
    assert not box.poll(observation(), 2).enabled
    assert box.last_rejection_reason == "clock_reversed"
    box.update_clock(CLOCK_OFFSET_NS + 21, EPISODE, 3)
    assert not box.offer(envelope(2, 21), 3)
    box.update_clock(CLOCK_OFFSET_NS, OTHER, 4)
    assert box.offer(envelope(0, episode=OTHER), 4)
    box.update_clock(CLOCK_OFFSET_NS + 100_000_000, OTHER, 5)
    assert box.poll(observation(100_000_000), 5).timestamp_ns == 0
    assert box.last_rejection_reason == "stale_command"


def test_fault_hold_keeps_last_accepted_gripper_and_uses_shared_validation() -> None:
    box = inbox()
    robot, driver = make_robot()
    box.offer(envelope(), 0)
    robot.command(box.poll(robot.observe(), 0))
    accepted = robot.observe().robot.gripper_position
    # Simulate aperture feedback lag; holds preserve accepted intent rather than
    # silently switching to a measured, partially closed aperture.
    driver.state = replace(driver.state, gripper_position=0.1)
    box.reject("malformed_envelope")
    box.update_clock(CLOCK_OFFSET_NS + 20_000_000, EPISODE, 1)
    robot.command(box.poll(robot.observe(), 1))
    assert robot.observe().robot.gripper_position == accepted
    for action in (
        Action(40_000_000, replace(HOME, frame_id="bad"), 0.5, True),
        Action(60_000_000, HOME, float("nan"), True),
    ):
        robot.command(action)
        assert robot.last_report is not None
        assert robot.last_report.outcome == CommandOutcome.INVALID


def test_import_isolation_and_missing_ros_cli_hint() -> None:
    script = """
import json, sys
import act_lab.domain, act_lab.application, act_lab.cli
import act_lab.adapters.ros2.contracts, act_lab.adapters.ros2.inbox
print(json.dumps([name for name in sys.modules if name.split('.')[0] in
['rclpy', 'act_lab_interfaces', 'mujoco', 'torch', 'lerobot', 'mediapipe']]))
"""
    result = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, check=True
    )
    assert json.loads(result.stdout) == []
    command = subprocess.run(
        [sys.executable, "-m", "act_lab.cli", "ros2", "contract-smoke"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert command.returncode == 1
    assert "docker compose --profile ros2-contracts" in command.stderr


def test_clock_state_disagreement_clears_cached_intent() -> None:
    box = inbox()
    box.offer(envelope(), 0)
    assert not box.poll(observation(1), 1).enabled
    assert box.last_rejection_reason == "clock_state_mismatch"
    box.update_clock(CLOCK_OFFSET_NS + 1, EPISODE, 2)
    assert not box.poll(observation(1), 2).enabled


def test_malformed_domain_envelope_and_steady_reversal_fail_closed() -> None:
    box = inbox()
    box.offer(envelope(), 0)
    malformed = replace(envelope(2), action=replace(envelope().action, enabled=1))
    assert not box.offer(malformed, 1)
    assert not box.poll(observation(), 1).enabled
    assert box.last_rejection_reason == "malformed_envelope"
    box.offer(envelope(3), 2)
    with pytest.raises(ValueError, match="steady clock"):
        box.poll(observation(), 1)
    assert not box.poll(observation(), 2).enabled


def test_report_without_executed_intent_ignores_generated_placeholder() -> None:
    report = CommandReport(
        CommandOutcome.INVALID, envelope().action, None, observation().robot, "fault"
    )
    fields = encode_report(report, EPISODE, 1)
    fields["executed_command"] = {"episode_id": "", "sequence": 0}
    assert decode_report(fields)[0] == report
