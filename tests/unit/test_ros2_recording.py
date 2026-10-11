"""Acquisition contracts run without rclpy, generated interfaces or rosbag2."""

import json
from dataclasses import replace
from uuid import uuid4

import pytest

from act_lab.adapters.mcap.recording import McapEpisodeSink
from act_lab.adapters.ros2.recording_contracts import (
    PacketConsumer,
    decode_sample,
    encode_sample,
    packet,
    provenance_json,
)
from act_lab.domain import (
    Action,
    CommandOutcome,
    CommandReport,
    Observation,
    Pose,
    RobotState,
)
from act_lab.domain.models import CameraFrame
from act_lab.domain.recording import EpisodeProvenance, EpisodeSample


def sample(stamp=20_000_000):
    pose = Pose("world", (0.4, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0))
    state = RobotState(stamp, (0.0,) * 6, (0.0,) * 6, pose, 0.6)
    action = Action(stamp - 20_000_000, pose, 0.6, True)
    return EpisodeSample(
        Observation(stamp, state, ("wrist",)),
        CommandReport(CommandOutcome.APPLIED, action, action, state, "accepted"),
        (("wrist", CameraFrame(stamp, 2, 1, bytes(range(6)))),),
    )


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


def start(episode, sequence=1):
    return packet(
        episode, sequence, 0, "start", metadata_json=provenance_json(provenance())
    )


def consumer(tmp_path):
    return PacketConsumer(lambda episode: McapEpisodeSink(tmp_path, episode))


def test_full_sample_roundtrip_keeps_source_stamp_and_gripper():
    episode = str(uuid4())
    original = sample()
    encoded = encode_sample(original, episode, 2)
    assert decode_sample(encoded, episode, 2) == original
    assert decode_sample(encoded, episode, 2).command.requested_action.timestamp_ns == 0
    rejected = replace(
        original,
        command=replace(
            original.command, outcome=CommandOutcome.STALE, executed_action=None
        ),
    )
    assert decode_sample(encode_sample(rejected, episode, 2), episode, 2) == rejected


@pytest.mark.parametrize(
    "corruption",
    [
        "camera_missing",
        "wrong_episode",
        "image_clock",
        "sample_clock",
        "jpeg",
        "row_padding",
        "truncated_rgb",
        "duplicate_camera",
    ],
)
def test_malformed_samples_rejected(corruption):
    episode = str(uuid4())
    value = encode_sample(sample(), episode, 2)
    if corruption == "camera_missing":
        value["image_keys"].append("policy")
    elif corruption == "wrong_episode":
        value["report"]["episode_id"] = str(uuid4())
    elif corruption == "image_clock":
        value["images"][0]["header"]["stamp"]["nanosec"] += 1
    elif corruption == "sample_clock":
        value["header"]["stamp"]["nanosec"] += 1
    elif corruption == "jpeg":
        value["images"][0]["encoding"] = "jpeg"
    elif corruption == "row_padding":
        value["images"][0]["step"] += 1
    elif corruption == "truncated_rgb":
        value["images"][0]["data"] = value["images"][0]["data"][:-1]
    elif corruption == "duplicate_camera":
        value["camera_ids"].append("wrist")
    with pytest.raises(ValueError):
        decode_sample(value, episode, 2)


def test_reset_new_episode_segmentation_and_discard(tmp_path):
    value = consumer(tmp_path)
    first, second = str(uuid4()), str(uuid4())
    value.consume(start(first))
    value.consume(
        packet(first, 2, 20_000_000, "sample", sample=encode_sample(sample(), first, 2))
    )
    value.consume(
        packet(first, 3, 20_000_000, "stop", outcome="failure", reason="bounded")
    )
    value.consume(start(second, 4))
    value.consume(
        packet(
            second, 5, 20_000_000, "sample", sample=encode_sample(sample(), second, 5)
        )
    )
    value.consume(
        packet(second, 6, 20_000_000, "discard", outcome="discarded", reason="operator")
    )
    value.finish()
    assert value.closed == {first, second}
    assert len(list(tmp_path.glob("*.mcap"))) == 2


@pytest.mark.parametrize(
    "fault", ["gap", "replay", "episode", "clock", "pause", "version", "missing_start"]
)
def test_transport_faults_fail_closed(tmp_path, fault):
    value = consumer(tmp_path)
    episode = str(uuid4())
    if fault != "missing_start":
        value.consume(start(episode))
    value.consume(
        packet(
            episode,
            2 if fault != "missing_start" else 1,
            20_000_000,
            "sample",
            sample=encode_sample(
                sample(), episode, 2 if fault != "missing_start" else 1
            ),
        )
    ) if fault != "missing_start" else None
    next_value = packet(
        episode,
        3,
        40_000_000,
        "sample",
        sample=encode_sample(sample(40_000_000), episode, 3),
    )
    if fault == "gap":
        next_value["sequence"] += 1
    elif fault == "replay":
        next_value["sequence"] -= 1
    elif fault == "episode":
        next_value["episode_id"] = str(uuid4())
    elif fault == "clock":
        next_value["header"]["stamp"]["nanosec"] = 0
    elif fault == "pause":
        next_value["sample"] = encode_sample(sample(), episode, 3)
        next_value["header"]["stamp"]["nanosec"] = 20_000_000
    elif fault == "version":
        next_value["schema_version"] = 2
    elif fault == "missing_start":
        next_value["sequence"] = 1
    with pytest.raises(ValueError):
        value.consume(next_value)
    if value.sink:
        value.sink.interrupt()
    assert not list(tmp_path.glob("*.mcap"))


def test_missing_stop_retains_interrupted_attempt(tmp_path):
    value = consumer(tmp_path)
    episode = str(uuid4())
    value.consume(start(episode))
    with pytest.raises(ValueError, match="interrupted"):
        value.finish()
    assert (tmp_path / f"{episode}.mcap.partial").exists()


def test_bad_provenance_before_creating_output(tmp_path):
    value = consumer(tmp_path)
    first = start(str(uuid4()))
    metadata = json.loads(first["metadata_json"])
    metadata["camera_ids"] = ["wrist", "wrist"]
    first["metadata_json"] = json.dumps(metadata)
    with pytest.raises(ValueError):
        value.consume(first)
    assert not list(tmp_path.iterdir())


def test_command_identity_is_independent_from_packet_sequence():
    episode = str(uuid4())
    value = encode_sample(sample(), episode, 12, command_sequence=7)
    assert value["report"]["command_sequence"] == 7
    assert decode_sample(value, episode, 12) == sample()
