"""Small generated records exercise durability without operator camera fixtures."""

import json
from dataclasses import replace

import pytest
from mcap.reader import make_reader

from act_lab.adapters.mcap.foxglove import export_foxglove
from act_lab.adapters.mcap.inspection import inspect_episode, recover_episode
from act_lab.adapters.mcap.recording import McapEpisodeSink
from act_lab.adapters.mcap.schema import decode, qualified_message_class
from act_lab.domain.models import (
    Action,
    CameraFrame,
    CommandOutcome,
    CommandReport,
    Observation,
    Pose,
    RobotState,
)
from act_lab.domain.recording import EpisodeOutcome, EpisodeProvenance, EpisodeSample


@pytest.fixture
def provenance() -> EpisodeProvenance:
    return EpisodeProvenance(
        "0.1.0",
        "test-revision",
        "false",
        7,
        "test",
        "ur5e-v1",
        "pick-place-v1",
        ("policy",),
        "test-operator",
        "test-image",
        "simulation-fixed-step-ns",
        123456789,
        "2026-09-13T00:00:00+00:00",
        '{"test":true}',
        '{"control_hz":50}',
        '{"status":"not_applicable"}',
    )


def sample(stamp: int = 20_000_000) -> EpisodeSample:
    pose = Pose("world", (0.4, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0))
    state = RobotState(stamp, (0.0,) * 6, (0.0,) * 6, pose, 0.5)
    action = Action(stamp - 20_000_000, pose, 0.5, True)
    return EpisodeSample(
        Observation(stamp, state, ("policy",)),
        CommandReport(CommandOutcome.APPLIED, action, action, state, "accepted"),
        (("policy", CameraFrame(stamp, 2, 1, bytes((1, 2, 3, 4, 5, 6)))),),
    )


def test_round_trip_streams_rates_and_embedded_schema(tmp_path, provenance):
    sink = McapEpisodeSink(tmp_path)
    episode_id = sink.start(provenance, 0)
    assert not sink.final_path.exists()
    for stamp in (20_000_000, 40_000_000, 60_000_000):
        sink.append(sample(stamp))
    sink.stop(EpisodeOutcome.SUCCESS, "settled")
    assert not sink.partial_path.exists()
    report = inspect_episode(sink.final_path)
    assert report["complete"] and report["monotonic"]
    assert report["outcome"] == "success"
    assert report["provenance"]["episode_id"] == episode_id
    assert report["training_validated"] is False
    for topic in ("/observation", "/command", "/camera/policy"):
        assert report["streams"][topic]["count"] == 3
        assert report["streams"][topic]["observed_hz"] == 50
    with sink.final_path.open("rb") as source:
        for schema, channel, message in make_reader(source).iter_messages():
            assert schema.encoding == channel.message_encoding == "protobuf"
            # Decode entirely from the embedded descriptor, independent of our loader.
            from google.protobuf import descriptor_pb2, descriptor_pool, message_factory

            pool = descriptor_pool.DescriptorPool()
            for file in descriptor_pb2.FileDescriptorSet.FromString(schema.data).file:
                pool.Add(file)
            cls = message_factory.GetMessageClass(
                pool.FindMessageTypeByName(schema.name)
            )
            value = cls.FromString(message.data)
            assert value.timestamp_ns == message.log_time == message.publish_time
            if channel.topic == "/camera/policy":
                assert value.data == bytes((1, 2, 3, 4, 5, 6))
            if channel.topic == "/command":
                assert (
                    value.requested_action.timestamp_ns == message.log_time - 20_000_000
                )


def test_foxglove_export_preserves_raw_and_adds_standard_image(tmp_path, provenance):
    sink = McapEpisodeSink(tmp_path)
    sink.start(provenance, 0)
    sink.append(sample())
    sink.stop(EpisodeOutcome.SUCCESS, "settled")
    raw_before = sink.final_path.read_bytes()
    output = tmp_path / "episode.foxglove.mcap"

    result = export_foxglove(sink.final_path, output)

    assert sink.final_path.read_bytes() == raw_before
    assert result["converted_images"] == 1
    with output.open("rb") as source:
        records = list(make_reader(source).iter_messages())
    assert "/observation" in {channel.topic for _, channel, _ in records}
    foxglove = next(
        (schema, message)
        for schema, channel, message in records
        if channel.topic == "/foxglove/camera/policy"
    )
    assert foxglove[0].name == "foxglove.RawImage"
    image = qualified_message_class("foxglove.RawImage").FromString(foxglove[1].data)
    assert (image.width, image.height, image.step, image.encoding) == (
        2,
        1,
        6,
        "rgb8",
    )
    assert image.data == bytes((1, 2, 3, 4, 5, 6))
    assert image.timestamp.nanos == 20_000_000


@pytest.mark.parametrize("outcome", [EpisodeOutcome.FAILURE, EpisodeOutcome.DISCARDED])
def test_rejected_demonstrations_remain_traceable(tmp_path, provenance, outcome):
    sink = McapEpisodeSink(tmp_path)
    sink.start(provenance, 0)
    sink.append(sample())
    if outcome is EpisodeOutcome.DISCARDED:
        sink.discard("operator rejected attempt")
    else:
        sink.stop(outcome, "timeout")
    report = inspect_episode(sink.final_path)
    assert report["outcome"] == outcome.value
    assert report["streams"]["/observation"]["count"] == 1
    assert report["events"][-1]["reason"]


def test_lifecycle_and_time_guards(tmp_path, provenance):
    sink = McapEpisodeSink(tmp_path)
    with pytest.raises(RuntimeError):
        sink.append(sample())
    sink.start(provenance, 0)
    with pytest.raises(RuntimeError):
        sink.start(provenance, 0)
    sink.append(sample())
    with pytest.raises(ValueError, match="strictly increase"):
        sink.append(sample())
    with pytest.raises(ValueError, match="monotonic"):
        sink.event(1, "task", EpisodeOutcome.UNKNOWN, "running")
    with pytest.raises(ValueError, match="reason"):
        sink.discard("")
    with pytest.raises(ValueError, match="requires"):
        sink.stop(EpisodeOutcome.UNKNOWN, "missing label")
    sink.discard("test")
    with pytest.raises(RuntimeError):
        sink.append(sample(40_000_000))
    with pytest.raises(RuntimeError):
        sink.stop(EpisodeOutcome.SUCCESS, "again")


def test_unsynchronized_sample_is_rejected_before_any_sample_write(
    tmp_path, provenance
):
    sink = McapEpisodeSink(tmp_path)
    sink.start(provenance, 0)
    original = sample()
    with pytest.raises(ValueError, match="synchronized"):
        sink.append(
            replace(
                original,
                observation=replace(original.observation, timestamp_ns=21_000_000),
            )
        )
    with pytest.raises(ValueError, match="image"):
        sink.append(
            replace(original, images=(("policy", CameraFrame(1, 1, 1, b"abc")),))
        )
    sink.interrupt()
    assert "/observation" not in inspect_episode(sink.partial_path)["streams"]


def test_recovery_preserves_original_and_refuses_active_writer(tmp_path, provenance):
    sink = McapEpisodeSink(tmp_path)
    sink.start(provenance, 0)
    sink.append(sample())
    with pytest.raises(BlockingIOError):
        recover_episode(sink.partial_path)
    sink.interrupt()
    before = sink.partial_path.read_bytes()
    report = inspect_episode(sink.partial_path)
    assert not report["complete"]
    recovered = recover_episode(sink.partial_path)
    assert sink.partial_path.read_bytes() == before
    result = inspect_episode(recovered)
    assert result["complete"]
    assert result["outcome"] == "interrupted"
    assert result["streams"]["/observation"]["count"] == 1
    with recovered.open("rb") as source:
        metadata = list(make_reader(source).iter_metadata())
    assert metadata[0].metadata["source_name"] == sink.partial_path.name
    assert len(metadata[0].metadata["source_sha256"]) == 64
    with pytest.raises(FileExistsError):
        recover_episode(sink.partial_path)


@pytest.mark.parametrize("remove_bytes", [1, 5, 20, 60])
def test_truncated_tail_recovers_complete_prefix(tmp_path, provenance, remove_bytes):
    sink = McapEpisodeSink(tmp_path)
    sink.start(provenance, 0)
    sink.append(sample())
    sink.append(sample(40_000_000))
    sink.interrupt()
    # Simulate power loss in the final flushed chunk/index.
    data = sink.partial_path.read_bytes()[:-remove_bytes]
    sink.partial_path.write_bytes(data)
    result = inspect_episode(recover_episode(sink.partial_path))
    assert result["complete"] and result["outcome"] == "interrupted"
    assert result["streams"]["/observation"]["count"] >= 1


def test_finalization_never_overwrites_and_partial_remains_recoverable(
    tmp_path,
    provenance,
):
    sink = McapEpisodeSink(tmp_path)
    sink.start(provenance, 0)
    sink.append(sample())
    sink.final_path.write_bytes(b"unrelated existing file")
    with pytest.raises(FileExistsError):
        sink.stop(EpisodeOutcome.SUCCESS, "test")
    assert sink.final_path.read_bytes() == b"unrelated existing file"
    assert (
        inspect_episode(recover_episode(sink.partial_path))["outcome"] == "interrupted"
    )


def test_disabled_report_has_no_executed_action_and_preserves_stale_time(
    tmp_path,
    provenance,
):
    sink = McapEpisodeSink(tmp_path)
    sink.start(provenance, 0)
    original = sample(200_000_000)
    report = replace(
        original.command,
        outcome=CommandOutcome.STALE,
        requested_action=replace(original.command.requested_action, timestamp_ns=0),
        executed_action=None,
    )
    sink.append(replace(original, command=report))
    sink.stop(EpisodeOutcome.FAILURE, "stale")
    with sink.final_path.open("rb") as source:
        records = list(make_reader(source).iter_messages(topics=["/command"]))
    msg = decode("Command", records[0][2].data)
    assert msg.requested_action.timestamp_ns == 0
    assert not msg.HasField("executed_action")


def test_corrupt_magic_is_not_recovered(tmp_path):
    path = tmp_path / "bad.mcap.partial"
    path.write_bytes(b"not mcap data")
    from mcap.exceptions import McapError

    with pytest.raises(McapError):
        recover_episode(path)
    assert not (tmp_path / "bad.recovered.mcap").exists()


def test_corrupt_chunk_crc_is_not_silently_salvaged(tmp_path, provenance):
    from mcap.stream_reader import CRCValidationError

    sink = McapEpisodeSink(tmp_path)
    sink.start(provenance, 0)
    sink.append(sample())
    sink.interrupt()
    data = bytearray(sink.partial_path.read_bytes())
    offset = 8
    while data[offset] != 6:  # MCAP Chunk opcode
        offset += 9 + int.from_bytes(data[offset + 1 : offset + 9], "little")
    data[offset + 9 + 24] ^= 1  # corrupt chunk's uncompressed CRC
    sink.partial_path.write_bytes(data)
    with pytest.raises(CRCValidationError):
        recover_episode(sink.partial_path)
    assert not list(tmp_path.glob("*.recovered.mcap"))


def test_process_exit_without_cleanup_preserves_durable_sample(tmp_path, provenance):
    import multiprocessing
    import os

    def acquire_and_crash():
        sink = McapEpisodeSink(tmp_path)
        sink.start(provenance, 0)
        sink.append(sample())
        os._exit(23)

    process = multiprocessing.get_context("fork").Process(target=acquire_and_crash)
    process.start()
    process.join(timeout=10)
    if process.is_alive():
        process.kill()
        process.join()
        pytest.fail("recording subprocess did not exit")
    assert process.exitcode == 23
    partial = next(tmp_path.glob("*.partial"))
    result = inspect_episode(recover_episode(partial))
    assert result["outcome"] == "interrupted"
    assert result["streams"]["/camera/policy"]["count"] == 1


@pytest.mark.parametrize("stop_survived", [False, True])
def test_partial_task_success_is_not_a_finalized_episode_outcome(
    tmp_path,
    provenance,
    stop_survived,
):
    sink = McapEpisodeSink(tmp_path)
    sink.start(provenance, 0)
    sink.append(sample())
    sink.event(20_000_000, "task", EpisodeOutcome.SUCCESS, "settled")
    if stop_survived:
        sink.event(20_000_000, "stop", EpisodeOutcome.SUCCESS, "operator label")
    sink.interrupt()
    report = inspect_episode(sink.partial_path)
    assert not report["complete"]
    assert report["outcome"] == "unknown"
    assert report["last_task_outcome"] == "success"
    recovered = inspect_episode(recover_episode(sink.partial_path))
    assert recovered["outcome"] == "interrupted"
    assert recovered["last_task_outcome"] == "success"


def test_quality_manifest_and_replay_cli(tmp_path, provenance, capsys):
    from act_lab.cli import main

    accepted = McapEpisodeSink(tmp_path / "raw")
    accepted.start(provenance, 0)
    for stamp in (20_000_000, 40_000_000, 60_000_000):
        accepted.append(sample(stamp))
    accepted.stop(EpisodeOutcome.SUCCESS, "settled")

    rejected = McapEpisodeSink(tmp_path / "raw")
    rejected.start(provenance, 0)
    rejected.append(sample())
    rejected.discard("operator rejected attempt")

    assert main(["recording", "validate", str(accepted.final_path)]) == 0
    validation = json.loads(capsys.readouterr().out)
    assert validation["valid"]
    assert validation["reports"][0]["training_eligible"]

    replay = tmp_path / "replay"
    assert (
        main(
            [
                "recording",
                "replay",
                str(accepted.final_path),
                "--camera",
                "policy",
                "--render-dir",
                str(replay),
                "--max-frames",
                "2",
            ]
        )
        == 0
    )
    report = json.loads(capsys.readouterr().out)
    assert report["camera_frames"] == 2
    assert len(list(replay.glob("*.ppm"))) == 2

    manifest = tmp_path / "selection.json"
    assert (
        main(
            [
                "recording",
                "manifest",
                str(accepted.final_path),
                str(rejected.final_path),
                "--output",
                str(manifest),
                "--validation-fraction",
                "0.5",
            ]
        )
        == 0
    )
    summary = json.loads(capsys.readouterr().out)
    assert summary["selected"] == summary["rejected"] == 1
    contents = json.loads(manifest.read_text())
    assert {entry["split"] for entry in contents["episodes"] if entry["selected"]} <= {
        "train",
        "validation",
    }
    assert any(not entry["selected"] for entry in contents["episodes"])
