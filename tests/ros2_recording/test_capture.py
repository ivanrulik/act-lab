"""Required generated CDR, DDS, rosbag2 storage and fail-closed import tests."""

import time
from pathlib import Path
from uuid import uuid4

import pytest
import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message

from act_lab.adapters.ros2.recording import RosEpisodeSink, import_bag
from act_lab.adapters.ros2.recording_contracts import (
    PACKET_TOPIC,
    PACKET_TYPE,
    encode_sample,
    packet,
    provenance_json,
)
from act_lab.adapters.ros2.recording_demo import Recorder, run_recording
from act_lab.adapters.ros2.runtime import make_message
from act_lab.domain import (
    Action,
    CommandOutcome,
    CommandReport,
    Observation,
    Pose,
    RobotState,
)
from act_lab.domain.models import CameraFrame
from act_lab.domain.recording import EpisodeOutcome, EpisodeProvenance, EpisodeSample


def provenance():
    return EpisodeProvenance(
        "0.1.0",
        "test",
        "false",
        0,
        "expert",
        "ur5e",
        "pick-place-v1",
        ("wrist",),
        "",
        "test",
        "simulation-fixed-step-ns",
        1,
        "2026-10-10T00:00:00Z",
        "{}",
        '{"control_hz":50}',
        "{}",
    )


def sample(stamp=20_000_000):
    pose = Pose("world", (0.4, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0))
    state = RobotState(stamp, (0.0,) * 6, (0.0,) * 6, pose, 0.6)
    action = Action(stamp - 20_000_000, pose, 0.6, True)
    return EpisodeSample(
        Observation(stamp, state, ("wrist",)),
        CommandReport(CommandOutcome.APPLIED, action, action, state, "accepted"),
        (("wrist", CameraFrame(stamp, 2, 1, bytes(range(6)))),),
    )


def test_generated_cdr_full_sample():
    episode = str(uuid4())
    value = packet(
        episode, 2, 20_000_000, "sample", sample=encode_sample(sample(), episode, 2)
    )
    message = make_message("RecordingPacket", value)
    decoded = deserialize_message(serialize_message(message), type(message))
    assert bytes(decoded.sample.images[0].data) == bytes(range(6))
    assert decoded.sample.report.requested_command.header.stamp.sec == 1
    assert decoded.sample.report.resulting_state.gripper_position == 0.6


def test_live_local_equivalence(tmp_path):
    result = run_recording(tmp_path / "local", smoke=True, steps=3)
    assert result["import_report"]["samples"] == 3
    assert result["storage"]["packets"] == 6
    assert result["storage"]["pid"] != result["acquisition"]["producer_pid"]
    assert (
        "/act_lab/recording/v1/camera/wrist/camera_info" in result["storage"]["topics"]
    )


def test_real_crisp_synchronized_acquisition(tmp_path):
    result = run_recording(tmp_path / "crisp", controller="crisp", steps=3, smoke=True)
    assert result["import_report"]["samples"] >= 4
    assert result["acquisition"]["physics_pid"] != result["acquisition"]["producer_pid"]
    assert all(
        case["quality"]["valid"] and not case["quality"]["training_eligible"]
        for case in result["import_report"]["cases"]
    )


def test_offline_acquisition_revokes_motion_before_slow_capture(tmp_path):
    from act_lab.adapters.ros2.simulation import snapshot_state
    from act_lab.adapters.ros2.simulation_smoke import MotionSession

    recorder = Recorder(tmp_path / "rosbag")
    session = None
    try:
        session = MotionSession(tmp_path / "motion")
        session.rpc(dict(kind="record_start", seed=0))
        target = snapshot_state(session.value).end_effector_pose
        first = session.motion_command(target, gripper=0.4)
        assert first["mode"] == "FAULT_HOLD"
        assert first["reason"] == "acquisition_pause"
        assert first["acquisition_execution_mode"] == "ENABLED"
        # Deliberately exceed the wall watchdog while offline I/O is idle.
        time.sleep(0.25)
        idle = session.rpc(dict(kind="snapshot"))
        assert idle["state"] == first["state"]
        assert idle["reason"] == "acquisition_pause"
        assert idle["accepted_gripper"] == first["accepted_gripper"]
        second = session.motion_command(target, gripper=0.4)
        assert second["acquisition_execution_mode"] == "ENABLED"
        assert (
            snapshot_state(second).timestamp_ns - snapshot_state(first).timestamp_ns
            == 20_000_000
        )
        session.rpc(dict(kind="record_stop"))
        recorder.close()
        imported = import_bag(tmp_path / "rosbag" / "bag", tmp_path / "imported")
        assert imported["samples"] == 3
        assert all(case["quality"]["valid"] for case in imported["cases"])
        assert all(
            not case["quality"]["training_eligible"] for case in imported["cases"]
        )
    finally:
        if session is not None:
            session.close()
        if recorder.process.is_alive():
            recorder.close(False)


@pytest.mark.parametrize(
    "fault", ["gap", "replay", "clock_reset", "missing_stop", "wrong_episode"]
)
def test_corrupt_bag_keeps_evidence_and_never_publishes(tmp_path, fault):
    bag = tmp_path / "bag"
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=0, name=PACKET_TOPIC, type=PACKET_TYPE, serialization_format="cdr"
        )
    )
    episode = str(uuid4())
    values = [
        packet(episode, 1, 0, "start", metadata_json=provenance_json(provenance())),
        packet(
            episode, 2, 20_000_000, "sample", sample=encode_sample(sample(), episode, 2)
        ),
    ]
    if fault == "gap":
        values[1]["sequence"] = 3
    elif fault == "replay":
        values[1]["sequence"] = 1
    elif fault == "clock_reset":
        values[1]["header"]["stamp"]["sec"] = 0
    elif fault == "wrong_episode":
        values[1]["episode_id"] = str(uuid4())
    if fault != "missing_stop":
        values.append(
            packet(episode, 3, 20_000_000, "stop", outcome="success", reason="fixture")
        )
    for index, value in enumerate(values):
        writer.write(
            PACKET_TOPIC,
            serialize_message(make_message("RecordingPacket", value)),
            index + 1,
        )
    writer.close()
    output = tmp_path / "imported"
    with pytest.raises(ValueError):
        import_bag(bag, output)
    assert bag.exists() and not output.exists()
    assert output.with_name("imported.partial").exists()
    assert (output.with_name("imported.partial") / "failure.json").exists()


def test_publisher_loss_retains_unfinished_bag(tmp_path):
    recorder = Recorder(tmp_path / "rosbag")
    rclpy.init()
    node = rclpy.create_node("loss_producer")
    sink = RosEpisodeSink(node)
    try:
        sink.start(provenance(), 0)
        sink.append(sample())
        sink.interrupt()
        result = recorder.close()
        assert result["packets"] == 2
        with pytest.raises(ValueError, match="interrupted"):
            import_bag(tmp_path / "rosbag" / "bag", tmp_path / "imported")
    finally:
        sink.interrupt()
        node.destroy_node()
        rclpy.try_shutdown()


def test_new_episode_after_reset_and_discard(tmp_path):
    recorder = Recorder(tmp_path / "rosbag")
    rclpy.init()
    node = rclpy.create_node("reset_producer")
    ids = []
    sequence = 0
    try:
        for outcome in (EpisodeOutcome.FAILURE, EpisodeOutcome.DISCARDED):
            sink = RosEpisodeSink(node, sequence_start=sequence)
            ids.append(sink.start(provenance(), 0))
            sink.append(sample())
            sink.stop(outcome, "bounded fixture")
            sequence = sink.sequence
        recorder.close()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    imported = import_bag(tmp_path / "rosbag" / "bag", tmp_path / "imported")
    assert set(imported["episodes"]) == set(ids)
    assert {item["quality"]["outcome"] for item in imported["cases"]} == {
        "failure",
        "discarded",
    }
    assert all(not item["quality"]["training_eligible"] for item in imported["cases"])


def test_recorder_loss_is_bounded_and_retains_partial(tmp_path):
    recorder = Recorder(tmp_path / "rosbag")
    rclpy.init()
    node = rclpy.create_node("recorder_loss_producer")
    sink = RosEpisodeSink(node)
    try:
        sink.start(provenance(), 0)
        recorder.process.terminate()
        recorder.process.join(5)
        sink.timeout_s = 0.2
        with pytest.raises(RuntimeError, match="timed out"):
            sink.append(sample())
        assert (tmp_path / "rosbag" / "bag.partial").exists()
        assert not (tmp_path / "rosbag" / "bag").exists()
    finally:
        sink.interrupt()
        recorder.connection.close()
        node.destroy_node()
        rclpy.try_shutdown()


def test_actual_crisp_reset_starts_new_recorded_episode(tmp_path):
    from act_lab.adapters.ros2.simulation import snapshot_state
    from act_lab.adapters.ros2.simulation_smoke import MotionSession

    recorder = Recorder(tmp_path / "rosbag")
    session = None
    ids = []
    try:
        session = MotionSession(
            tmp_path / "motion", config_path=Path("configs/sim/ur5e_2f85_d405.toml")
        )
        for seed in (0, 1):
            session.rpc(dict(kind="record_start", seed=seed))
            ids.append(session.value["episode"])
            session.motion_command(snapshot_state(session.value).end_effector_pose)
            session.rpc(dict(kind="record_stop"))
        session.close()
        recorder.close()
    finally:
        if session is not None and not getattr(session, "closed", False):
            session.close()
        if recorder.process.is_alive():
            recorder.close(False)
    assert len(set(ids)) == 2
    result = import_bag(tmp_path / "rosbag" / "bag", tmp_path / "imported")
    assert set(result["episodes"]) == set(ids)
    assert all(case["quality"]["valid"] for case in result["cases"])
