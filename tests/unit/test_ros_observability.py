import subprocess
import sys
from types import SimpleNamespace
from uuid import UUID

import pytest

from act_lab.adapters.ros2.observability import (
    ObservationCapture,
    ObservationChannel,
    ObserverFreshness,
)
from act_lab.adapters.ros2.observation_view import joint_fields, marker_fields
from act_lab.adapters.ros2.simulation_guard import SimulationGuard
from act_lab.domain import Pose, RobotState


def packet_fixture():
    pose = Pose("world", (0.4, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0))
    state = RobotState(0, (0.0,) * 6, (0.0,) * 6, pose, 0.4)
    guard = SimulationGuard(UUID(int=1))
    sample = SimpleNamespace(
        task_effort_nm=(0.0,) * 6,
        servo_effort_nm=(0.0,) * 6,
        total_effort_nm=(0.0,) * 6,
        accepted_gripper=0.4,
    )
    channel = ObservationChannel()
    capture = ObservationCapture(channel)
    capture.capture(
        state,
        str(guard.episode),
        0,
        "STARTUP_HOLD",
        "startup",
        0,
        0,
        guard,
        sample,
        None,
        0,
        0,
    )
    return channel.read(), channel, capture, state, guard, sample


def test_busy_observer_cannot_block_or_corrupt_owner():
    channel = ObservationChannel()
    assert channel.offer(dict(schema_version=1, value=1))
    channel.lock.acquire()
    try:
        assert not channel.offer(dict(schema_version=1, value=2))
        assert channel.read() is None
        assert channel.dropped.value == 1
    finally:
        channel.lock.release()
    assert channel.read()["value"] == 1
    assert not channel.offer(dict(payload="x" * 65536))


def test_duplicate_snapshot_does_not_refresh_observer_receipt():
    packet, *_ = packet_fixture()
    freshness = ObserverFreshness()
    assert freshness.receive(packet, 0)
    assert not freshness.stale(99_999_999)
    assert not freshness.receive(packet, 99_999_999)
    assert freshness.stale(100_000_000)
    packet["telemetry"]["episode_id"] = str(UUID(int=2))
    assert freshness.receive(packet, 100_000_001)
    assert not freshness.stale(100_000_001)


def test_raw_invalid_is_explicit_and_joint_effort_is_total():
    packet, channel, capture, state, guard, sample = packet_fixture()
    capture.capture(
        state,
        str(guard.episode),
        10,
        "FAULT_HOLD",
        "invalid_effort",
        1,
        0,
        guard,
        sample,
        [float("nan")] * 6,
        0,
        0,
    )
    fields = channel.read()["telemetry"]
    assert fields["raw_effort_valid"] is False
    assert "nan" in fields["raw_effort_detail"]
    assert packet["telemetry"]["source_valid"] is False
    assert joint_fields(packet)["effort"] == [0.0] * 6
    packet["state"]["joint_names"].reverse()
    with pytest.raises(ValueError, match="canonical"):
        joint_fields(packet)


def test_stale_markers_delete_intent_and_label_unavailable():
    packet, *_ = packet_fixture()
    packet["requested"] = packet["measured_pose"]
    packet["requested_frame"] = "world"
    live = marker_fields(packet)
    assert any(m["id"] == 1 for m in live["markers"])
    stale = marker_fields(packet, stale=True)
    assert stale["markers"][0]["action"] == 3
    assert all(m["id"] not in (1, 2, 3) for m in stale["markers"])
    assert "STALE" in stale["markers"][-1]["text"]
    packet["requested_frame"] = "camera"
    assert not any(m["id"] == 1 for m in marker_fields(packet)["markers"])


def test_source_identity_retained_for_diagnosis_then_cleared_on_reset():
    _, channel, capture, state, guard, sample = packet_fixture()
    source = dict(
        sequence=7, source_ns=0, wall_receipt_ns=1, source_episode_id=str(guard.episode)
    )
    capture.capture(
        state,
        str(guard.episode),
        10,
        "ENABLED",
        "authorized",
        1,
        0,
        guard,
        sample,
        [1.0] * 6,
        2,
        2,
        source,
    )
    capture.capture(
        state,
        str(guard.episode),
        20,
        "FAULT_HOLD",
        "controller_wall_timeout",
        2,
        0,
        guard,
        sample,
        None,
        100_000_001,
        2,
    )
    assert channel.read()["telemetry"]["command_sequence"] == 7
    assert channel.read()["telemetry"]["source_timestamp_ns"] == 0
    capture.capture(
        state,
        str(UUID(int=2)),
        0,
        "FAULT_HOLD",
        "episode_reset",
        3,
        0,
        guard,
        sample,
        None,
        100_000_002,
        2,
    )
    assert channel.read()["telemetry"]["source_valid"] is False


def test_event_overflow_is_visible():
    _, channel, capture, state, guard, sample = packet_fixture()
    for i in range(70):
        capture.capture(
            state,
            str(guard.episode),
            i,
            "FAULT_HOLD",
            str(i),
            i,
            0,
            guard,
            sample,
            None,
            0,
            0,
        )
    packet = channel.read()
    assert len(packet["events"]) == 64
    assert packet["telemetry"]["event_drops"] > 0


def test_observability_imports_do_not_load_ros_or_simulator():
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys;"
            "from act_lab.adapters.ros2 import observability,observation_view;"
            "assert not any(k.startswith(('rclpy','mujoco','act_lab_interfaces')) "
            "for k in sys.modules)",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0


def test_cli_missing_observer_runtime_has_compose_instruction(capsys):
    from act_lab.cli import main

    assert main(["ros2", "observability-smoke", "--duration", "2"]) == 1
    assert "docker compose --profile ros2-observability" in capsys.readouterr().err


def test_raw_response_identity_is_distinct_from_current_epoch():
    _, channel, capture, state, guard, sample = packet_fixture()
    capture.capture(
        state,
        str(guard.episode),
        10,
        "FAULT_HOLD",
        "late_output",
        5,
        0,
        guard,
        sample,
        dict(values=[1.0] * 6, episode="old-episode", generation=4, tick=9),
        1,
        1,
    )
    telemetry = channel.read()["telemetry"]
    assert telemetry["raw_output_episode_id"] == "old-episode"
    assert telemetry["raw_output_generation"] == 4
    assert telemetry["raw_output_tick"] == 9
    assert not telemetry["authorization_valid"]


def test_report_reader_retries_busy_slot_without_restamping():
    from act_lab.adapters.ros2.observability import read_report_snapshot

    packet, channel, *_ = packet_fixture()
    channel.lock.acquire()
    waits = []

    def release_reader(duration):
        waits.append(duration)
        channel.lock.release()

    assert read_report_snapshot(channel, wait=release_reader) == packet
    assert waits == [0.005]


def test_report_reader_missing_input_fails_at_bounded_deadline():
    from act_lab.adapters.ros2.observability import read_report_snapshot

    channel = ObservationChannel()
    elapsed = [0.0]

    def advance(duration):
        elapsed[0] += duration

    with pytest.raises(RuntimeError, match="report timeout"):
        read_report_snapshot(
            channel, timeout_s=0.01, steady_now=lambda: elapsed[0], wait=advance
        )
    assert elapsed[0] == 0.01
