"""Acknowledged DDS acquisition, rosbag2 MCAP storage and canonical import."""

from __future__ import annotations

import gc
import importlib
import json
import os
import pickle
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any, TextIO
from uuid import uuid4

from act_lab.adapters.ros2.recording_contracts import (
    ACK_TOPIC,
    PACKET_TOPIC,
    PACKET_TYPE,
    PacketConsumer,
    encode_sample,
    packet,
    provenance_json,
)
from act_lab.adapters.ros2.runtime import make_message, message_fields
from act_lab.domain.ports import EpisodeSink
from act_lab.domain.recording import EpisodeOutcome, EpisodeProvenance, EpisodeSample


def retry_identity(message: Any) -> bytes:
    """Compare every generated field, excluding unspecified CDR padding.

    Pickle is only produced for equality, never loaded. RGB remains bytes and
    floating-point bit patterns remain intact; no payload is normalized.
    """
    return pickle.dumps(message_fields(message), protocol=5)


HINT = "Use docker compose --profile ros2-recording run --build --rm ros2-recording"


def require_recording() -> None:
    try:
        for name in (
            "rclpy",
            "rosbag2_py",
            "act_lab_interfaces.msg",
            "mcap",
            "google.protobuf",
        ):
            importlib.import_module(name)
        _ = importlib.import_module("act_lab_interfaces.msg").RecordingPacket
    except (ImportError, AttributeError) as error:
        raise RuntimeError(HINT) from error


class RosEpisodeSink:
    """A bounded acknowledged stream; recording errors propagate to shared hold."""

    def __init__(
        self,
        node: Any,
        mirror: Callable[[str], EpisodeSink] | None = None,
        *,
        episode: Callable[[], str] | None = None,
        sequence_start: int = 0,
        timeout_s: float = 10.0,
        command_sequence: Callable[[], int] | None = None,
    ) -> None:
        require_recording()
        from rclpy.qos import QoSProfile  # type: ignore[import-not-found]
        from std_msgs.msg import String  # type: ignore[import-not-found]

        if timeout_s <= 0:
            raise ValueError("positive recording timeout required")
        self.timeout_s = timeout_s
        self.command_sequence = command_sequence
        self.sample_count = 0
        self.node = node
        from rclpy.executors import (  # type: ignore[import-not-found]
            SingleThreadedExecutor,
        )

        self._owns_executor = node.executor is None
        self.executor = node.executor or SingleThreadedExecutor(context=node.context)
        if self._owns_executor:
            self.executor.add_node(node)
        self.publisher = node.create_publisher(
            type(make_message("RecordingPacket", {})),
            PACKET_TOPIC,
            QoSProfile(depth=10),
        )
        self.trace_path: Path | None = None
        self._trace_stream: TextIO | None = None
        self.ack: tuple[str, int] | None = None
        self.subscription = node.create_subscription(
            String, ACK_TOPIC, self._ack, QoSProfile(depth=10)
        )
        self.episode_id = str(uuid4())
        self.sequence = sequence_start
        self.episode_provider = episode
        self.timestamp = 0
        self.mirror_factory = mirror
        self.mirror: EpisodeSink | None = None
        self.started = False
        self.finished = False

    def for_directory(self, directory: Path) -> RosEpisodeSink:
        """Bind this live sink to the existing recording-session factory port."""
        return self

    def _ack(self, message: Any) -> None:
        value = json.loads(message.data)
        self.ack = value["episode_id"], value["sequence"]

    def wait_ready(self, settle_s: float = 0.0) -> None:
        """Drain startup discovery under hold, before any recorded source sample."""
        end = time.monotonic() + self.timeout_s
        matched_at: float | None = None
        while time.monotonic() < end:
            if (
                self.publisher.get_subscription_count() == 1
                and self.subscription.get_publisher_count() == 1
            ):
                matched_at = matched_at or time.monotonic()
                if time.monotonic() - matched_at >= settle_s:
                    return
            else:
                matched_at = None
            self.executor.spin_once(timeout_sec=0.01)
        raise RuntimeError(
            "recording requires exactly one recorder; discovery timed out"
        )

    def _send(self, kind: str, timestamp: int, **values: Any) -> None:
        self.wait_ready()
        self.sequence += 1

        def trace(stage: str, **timings: int) -> None:
            if self.trace_path is not None:
                if self._trace_stream is None:
                    self._trace_stream = self.trace_path.open(
                        "a", buffering=1024 * 1024
                    )
                self._trace_stream.write(
                    json.dumps(
                        dict(
                            sequence=self.sequence,
                            kind=kind,
                            stage=stage,
                            steady_ns=time.monotonic_ns(),
                            **timings,
                        )
                    )
                    + "\n"
                )

        trace("construct")
        construction_start = time.monotonic_ns()
        cpu_start = time.thread_time_ns()
        gc_started = 0
        gc_elapsed = 0

        def construction_gc(phase: str, info: dict[str, Any]) -> None:
            nonlocal gc_started, gc_elapsed
            if phase == "start":
                gc_started = time.monotonic_ns()
            else:
                gc_elapsed += time.monotonic_ns() - gc_started

        if self.trace_path is not None:
            gc.callbacks.append(construction_gc)
        try:
            message = make_message(
                "RecordingPacket",
                packet(self.episode_id, self.sequence, timestamp, kind, **values),
            )
        finally:
            if self.trace_path is not None:
                gc.callbacks.remove(construction_gc)
        trace(
            "constructed",
            wall_ns=time.monotonic_ns() - construction_start,
            cpu_ns=time.thread_time_ns() - cpu_start,
            gc_ns=gc_elapsed,
        )
        trace("publish")
        self.publisher.publish(message)
        trace("ack_wait")
        end = time.monotonic() + self.timeout_s
        retry_at = time.monotonic() + 0.01
        while self.ack != (self.episode_id, self.sequence):
            if time.monotonic() >= end:
                raise RuntimeError(
                    "recording acknowledgement timed out; acquisition interrupted"
                )
            self.executor.spin_once(timeout_sec=0.001)
            if (
                self.ack != (self.episode_id, self.sequence)
                and time.monotonic() >= retry_at
            ):
                self.publisher.publish(message)
                retry_at = time.monotonic() + 0.01
                trace("identical_retry")
        trace("ack_received")
        self.timestamp = timestamp

    def start(self, provenance: EpisodeProvenance, timestamp_ns: int) -> str:
        if self.started:
            raise RuntimeError("episode already started")
        self.started = True
        if self.episode_provider:
            self.episode_id = self.episode_provider()
        self._send("start", timestamp_ns, metadata_json=provenance_json(provenance))
        if self.mirror_factory:
            self.mirror = self.mirror_factory(self.episode_id)
            if self.mirror.start(provenance, timestamp_ns) != self.episode_id:
                raise RuntimeError("mirror must preserve episode identity")
        return self.episode_id

    def append(self, sample: EpisodeSample) -> None:
        if not self.started or self.finished:
            raise RuntimeError("episode is not recording")
        self.sample_count += 1
        self._send(
            "sample",
            sample.observation.timestamp_ns,
            sample=encode_sample(
                sample,
                self.episode_id,
                self.sequence + 1,
                self.command_sequence() if self.command_sequence else self.sample_count,
            ),
        )
        if self.mirror:
            self.mirror.append(sample)

    def event(
        self, timestamp_ns: int, kind: str, outcome: EpisodeOutcome, reason: str
    ) -> None:
        if kind != "task":
            raise ValueError("unsupported acquisition event")
        self._send(kind, timestamp_ns, outcome=outcome.value, reason=reason)
        if self.mirror:
            self.mirror.event(timestamp_ns, kind, outcome, reason)

    def diagnostics(
        self,
        timestamp_ns: int,
        host_monotonic_ns: int,
        values_json: str,
        calibration_json: str,
    ) -> None:
        self._send(
            "diagnostics",
            timestamp_ns,
            metadata_json=json.dumps(
                dict(
                    host_monotonic_ns=host_monotonic_ns,
                    values_json=values_json,
                    calibration_json=calibration_json,
                )
            ),
        )
        if self.mirror:
            self.mirror.diagnostics(
                timestamp_ns, host_monotonic_ns, values_json, calibration_json
            )

    def stop(self, outcome: EpisodeOutcome, reason: str) -> None:
        if self.finished:
            return
        if (
            outcome in {EpisodeOutcome.UNKNOWN, EpisodeOutcome.INTERRUPTED}
            or not reason.strip()
        ):
            raise ValueError("terminal outcome and reason required")
        self._send(
            "discard" if outcome is EpisodeOutcome.DISCARDED else "stop",
            self.timestamp,
            outcome=outcome.value,
            reason=reason,
        )
        if self.mirror:
            self.mirror.stop(outcome, reason)
        self.finished = True
        self.close()

    def discard(self, reason: str) -> None:
        self.stop(EpisodeOutcome.DISCARDED, reason)

    def interrupt(self) -> None:
        if self.mirror:
            self.mirror.interrupt()
        self.close()

    def close(self) -> None:
        if self._trace_stream is not None:
            self._trace_stream.close()
            self._trace_stream = None
        if self.publisher is not None:
            self.node.destroy_publisher(self.publisher)
            self.node.destroy_subscription(self.subscription)
            self.publisher = None
            if self._owns_executor:
                # A later episode on the same node must not reuse a stopped
                # executor through Node's weak reference.
                self.node.executor = None
                self.executor.shutdown(timeout_sec=1)


def _sync_tree(directory: Path) -> None:
    from act_lab.adapters.mcap.recording import sync_directory

    for path in directory.rglob("*"):
        if path.is_file():
            with path.open("rb") as stream:
                os.fsync(stream.fileno())
    sync_directory(directory)


def record_process(
    connection: Any, output: str, drop_ack_once: int | None = None
) -> None:
    """Separate storage owner. Acknowledgement means writer acceptance, not fsync."""
    require_recording()
    rclpy = importlib.import_module("rclpy")
    rosbag2_py = importlib.import_module("rosbag2_py")
    QoSProfile = importlib.import_module("rclpy.qos").QoSProfile
    serialize_message = importlib.import_module("rclpy.serialization").serialize_message
    String = importlib.import_module("std_msgs.msg").String

    root = Path(output)
    root.mkdir(parents=True, exist_ok=False)
    partial, final = root / "bag.partial", root / "bag"
    config = root / "storage.yaml"
    config.write_text("compression: Zstd\nnoChunkCRC: false\nnoDataCRC: false\n")
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(partial), storage_id="mcap", storage_config_uri=str(config)
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=0, name=PACKET_TOPIC, type=PACKET_TYPE, serialization_format="cdr"
        )
    )
    rclpy.init()
    # Transport-only storage uses explicit source stamps and steady receipt time.
    # It never reads ROS time; automatic /clock callbacks are unrelated work.
    node = rclpy.create_node("act_lab_mcap_recorder")
    from rclpy.executors import SingleThreadedExecutor

    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    ack = node.create_publisher(String, ACK_TOPIC, QoSProfile(depth=10))
    sequence = 0
    error: str | None = None
    origin_wall, origin_steady = time.time_ns(), time.monotonic_ns()
    from act_lab.adapters.ros2.recording_contracts import PacketReceipt

    receipts = PacketReceipt()
    dropped_ack = False
    topics = {PACKET_TOPIC}
    calibrations: dict[str, Any] = {}
    from act_lab.adapters.ros2.trace_staging import DiagnosticFile, staging_root

    progress_file = DiagnosticFile(partial / "recorder-progress.jsonl", staging_root())
    progress_stream = progress_file.partial.open("w", buffering=1024 * 1024)
    progress_value: dict[str, Any] | None = None

    def write(topic: str, message: Any, kind: str) -> None:
        if topic not in topics:
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    id=len(topics), name=topic, type=kind, serialization_format="cdr"
                )
            )
            topics.add(topic)
        writer.write(
            topic,
            serialize_message(message),
            origin_wall + time.monotonic_ns() - origin_steady,
        )

    def receive(message: Any) -> None:
        nonlocal sequence, error, calibrations, dropped_ack, progress_value
        received_ns = time.monotonic_ns()
        if error:
            return
        try:
            if message.schema_version != 1:
                raise ValueError("packet gap/replay/version mismatch")
            fresh = receipts.accept(
                message.episode_id, message.sequence, retry_identity(message)
            )
            if not fresh:
                ack.publish(
                    String(
                        data=json.dumps(
                            dict(
                                episode_id=message.episode_id,
                                sequence=message.sequence,
                            )
                        )
                    )
                )
                return
            write(PACKET_TOPIC, message, PACKET_TYPE)
            if message.kind == "start":
                provenance = json.loads(message.metadata_json)
                calibrations = json.loads(provenance["resolved_config_json"]).get(
                    "scene_cameras", {}
                )
            if message.kind == "sample":
                for name, image in zip(
                    message.sample.camera_ids, message.sample.images, strict=True
                ):
                    profile = calibrations.get(name)
                    if profile:
                        set_message_fields = importlib.import_module(
                            "rosidl_runtime_py.set_message"
                        ).set_message_fields

                        CameraInfo = importlib.import_module(
                            "sensor_msgs.msg"
                        ).CameraInfo
                        info = CameraInfo()
                        k = profile["k"]
                        set_message_fields(
                            info,
                            dict(
                                header=dict(
                                    stamp=dict(
                                        sec=image.header.stamp.sec,
                                        nanosec=image.header.stamp.nanosec,
                                    ),
                                    frame_id=profile["frame_id"],
                                ),
                                width=profile["width"],
                                height=profile["height"],
                                distortion_model=profile["distortion_model"],
                                d=profile["d"],
                                k=k,
                                r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                                p=[
                                    k[0],
                                    k[1],
                                    k[2],
                                    0.0,
                                    k[3],
                                    k[4],
                                    k[5],
                                    0.0,
                                    k[6],
                                    k[7],
                                    k[8],
                                    0.0,
                                ],
                            ),
                        )
                        write(
                            f"/act_lab/recording/v1/camera/{name}/camera_info",
                            info,
                            "sensor_msgs/msg/CameraInfo",
                        )
                        # Projection uses calibrated frames; atomic source is unchanged.
                        projected = importlib.import_module("sensor_msgs.msg").Image()
                        projected.header = info.header
                        projected.width, projected.height = image.width, image.height
                        projected.encoding, projected.is_bigendian = (
                            image.encoding,
                            image.is_bigendian,
                        )
                        projected.step, projected.data = image.step, image.data
                    else:
                        projected = image
                    write(
                        f"/act_lab/recording/v1/camera/{name}/image_raw",
                        projected,
                        "sensor_msgs/msg/Image",
                    )
                write(
                    "/act_lab/recording/v1/command_report",
                    message.sample.report,
                    "act_lab_interfaces/msg/CommandReport",
                )
                write(
                    "/act_lab/recording/v1/state",
                    message.sample.report.resulting_state,
                    "act_lab_interfaces/msg/RobotState",
                )
                clock = make_message(
                    "Clock",
                    dict(
                        clock=dict(
                            sec=message.header.stamp.sec,
                            nanosec=message.header.stamp.nanosec,
                        )
                    ),
                )
                write("/clock", clock, "rosgraph_msgs/msg/Clock")
            progress_value = dict(
                sequence=message.sequence,
                kind=message.kind,
                write_ns=time.monotonic_ns() - received_ns,
                steady_ns=time.monotonic_ns(),
            )
            progress_stream.write(json.dumps(progress_value) + "\n")
            sequence = message.sequence
            if message.sequence == drop_ack_once and not dropped_ack:
                dropped_ack = True
                return
            ack.publish(
                String(
                    data=json.dumps(
                        dict(episode_id=message.episode_id, sequence=sequence)
                    )
                )
            )
        except Exception as exc:
            error = str(exc)
            (partial / "recording-failure.json").write_text(
                json.dumps(dict(reason=error, accepted_sequence=sequence)) + "\n"
            )

    node.create_subscription(
        type(make_message("RecordingPacket", {})),
        PACKET_TOPIC,
        receive,
        QoSProfile(depth=10),
    )
    connection.send(dict(ready=True, pid=os.getpid()))
    try:
        while not connection.poll():
            executor.spin_once(timeout_sec=0.02)
        finalize = connection.recv()["finalize"]
        progress_stream.close()
        progress_file.commit(complete=True)
        if progress_value is not None:
            (partial / "recorder-progress.json").write_text(json.dumps(progress_value))
        writer.close()
        _sync_tree(partial)
        if error:
            raise RuntimeError(error)
        if finalize:
            if final.exists():
                raise FileExistsError(str(final))
            partial.rename(final)
            from act_lab.adapters.mcap.recording import sync_directory

            sync_directory(root)
        connection.send(
            dict(
                status="passed",
                packets=sequence,
                topics=sorted(topics),
                pid=os.getpid(),
                finalized=finalize,
                identical_retries=receipts.duplicates,
            )
        )
    except BaseException as exc:
        connection.send(dict(error=str(exc)))
        raise
    finally:
        progress_stream.close()
        executor.shutdown(timeout_sec=1)
        node.destroy_node()
        rclpy.try_shutdown()
        connection.close()


def import_bag(bag: Path, output: Path) -> dict[str, Any]:
    require_recording()
    rosbag2_py = importlib.import_module("rosbag2_py")
    deserialize_message = importlib.import_module(
        "rclpy.serialization"
    ).deserialize_message

    from act_lab.adapters.mcap.reading import read_episode
    from act_lab.adapters.mcap.recording import McapEpisodeSink, sync_directory
    from act_lab.application.dataset import file_sha256, report_dict, validate_episode

    if not (bag / "metadata.yaml").is_file() or bag.name.endswith(".partial"):
        raise ValueError(
            "import requires a finalized rosbag2 directory; retain partial evidence"
        )
    from mcap.reader import make_reader

    metadata = rosbag2_py.Info().read_metadata(str(bag), "mcap")
    if metadata.storage_identifier != "mcap" or not metadata.relative_file_paths:
        raise ValueError("bag must contain MCAP files")
    for name in metadata.relative_file_paths:
        path = bag / name
        if Path(name).name != name or path.is_symlink():
            raise ValueError("bag paths must stay within the acquisition directory")
        with path.open("rb") as source:
            for schema, channel, _ in make_reader(
                source, validate_crcs=True
            ).iter_messages(log_time_order=False):
                if schema is None or channel.message_encoding != "cdr":
                    raise ValueError("expected generated CDR schemas")
    if output.exists() or output.with_name(output.name + ".partial").exists():
        raise FileExistsError(str(output))
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if types.get(PACKET_TOPIC) != PACKET_TYPE:
        raise ValueError("bag is missing the versioned acquisition stream")
    staging = output.with_name(output.name + ".partial")
    staging.mkdir(parents=True, exist_ok=False)
    lineage = {p.name: file_sha256(p) for p in sorted(bag.glob("*.mcap"))}
    consumer = PacketConsumer(
        lambda episode: McapEpisodeSink(staging, episode, lineage=lineage)
    )
    message_type = type(make_message("RecordingPacket", {}))
    try:
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic == PACKET_TOPIC:
                consumer.consume(
                    message_fields(deserialize_message(data, message_type))
                )
        consumer.finish()
        reports = [
            dict(
                path=path.name,
                quality=report_dict(validate_episode(read_episode(path))),
            )
            for path in sorted(staging.glob("*.mcap"))
        ]
        result = dict(
            schema_version=1,
            status="passed",
            cases=reports,
            packets=consumer.sequence,
            samples=consumer.samples,
            lineage=lineage,
            episodes=sorted(consumer.closed),
        )
        (staging / "import.json").write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n"
        )
        _sync_tree(staging)
        staging.rename(output)
        sync_directory(output.parent)
        return result
    except BaseException:
        if consumer.sink:
            consumer.sink.interrupt()
        (staging / "failure.json").write_text(
            json.dumps(
                dict(
                    status="interrupted",
                    packets=consumer.sequence,
                    samples=consumer.samples,
                )
            )
        )
        raise
