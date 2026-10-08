"""Required ROS job tests: intentionally fail on missing generated packages."""

from __future__ import annotations

import importlib

import pytest

from act_lab.adapters.ros2.contracts import (
    CommandEnvelope,
    decode_command,
    decode_report,
    decode_state,
    encode_command,
    encode_report,
    encode_state,
)
from act_lab.adapters.ros2.demonstrator import HOME, make_robot
from act_lab.adapters.ros2.runtime import make_message, message_fields, qos_profiles
from act_lab.adapters.ros2.smoke import run_smoke
from act_lab.domain import Action

EPISODE = "c3318069-22c5-4af3-9df0-56b43ca3df1e"


@pytest.mark.parametrize("enabled", [True, False])
def test_generated_serialization(enabled: bool) -> None:
    serialization = importlib.import_module("rclpy.serialization")
    envelope = CommandEnvelope(EPISODE, 1, Action(0, HOME, 0.3, enabled))
    robot, _ = make_robot()
    robot.command(envelope.action)
    report = robot.last_command_report
    assert report is not None
    for kind, fields, decoder, expected in (
        ("CartesianCommand", encode_command(envelope), decode_command, envelope),
        (
            "RobotState",
            encode_state(robot.observe().robot, EPISODE),
            decode_state,
            (robot.observe().robot, EPISODE),
        ),
        (
            "CommandReport",
            encode_report(report, EPISODE, 1),
            decode_report,
            (report, EPISODE, 1),
        ),
    ):
        message = make_message(kind, fields)
        wire = serialization.serialize_message(message)
        restored = serialization.deserialize_message(wire, type(message))
        assert decoder(message_fields(restored)) == expected


def test_qos_and_two_process_faults() -> None:
    profiles = qos_profiles()
    assert profiles["command"].deadline.nanoseconds == 100_000_000
    assert profiles["command"].lifespan.nanoseconds == 100_000_000
    assert profiles["state"].reliability.name == "BEST_EFFORT"
    assert profiles["report"].depth == 10
    assert profiles["clock"].depth == 1
    result = run_smoke()
    assert result["status"] == "ok"
    assert result["provenance"]["consumer_pid"] != result["provenance"]["producer_pid"]
    assert result["provenance"]["rmw"] == "rmw_fastrtps_cpp"
    assert {case["case"] for case in result["cases"]} >= {
        "startup_missing",
        "future",
        "exact_stale_boundary",
        "publisher_loss",
        "paused_clock",
        "backward_jump",
        "new_epoch_recovery",
        "duplicate",
        "wrong_episode",
        "unsupported_frame",
        "invalid_numeric",
    }
