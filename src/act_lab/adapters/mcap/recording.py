"""Durable, exclusively owned episode files with explicit finalization."""

from __future__ import annotations

import base64
import fcntl
import os
from dataclasses import asdict
from pathlib import Path
from typing import Any, BinaryIO
from uuid import uuid4

from mcap.writer import CompressionType, Writer

from act_lab.adapters.mcap.schema import descriptor_bytes, encode
from act_lab.domain.recording import EpisodeOutcome, EpisodeProvenance, EpisodeSample

PROFILE = "act_lab.recording.v1"


def sync_directory(directory: Path) -> None:
    fd = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def publish(partial: Path, final: Path) -> None:
    # Same-filesystem hard link is atomic and refuses to overwrite any target.
    os.link(partial, final)
    sync_directory(final.parent)
    partial.unlink()
    sync_directory(final.parent)


class McapEpisodeSink:
    """One attempt per sink; failed/discarded attempts retain all acquisition data."""

    def __init__(self, directory: Path) -> None:
        self.directory = directory
        self.episode_id = str(uuid4())
        self.partial_path = directory / f"{self.episode_id}.mcap.partial"
        self.final_path = directory / f"{self.episode_id}.mcap"
        self._stream: BinaryIO | None = None
        self._writer: Writer | None = None
        self._channels: dict[str, int] = {}
        self._schemas: dict[str, int] = {}
        self._sequences: dict[str, int] = {}
        self._timestamp = 0
        self._sample_timestamp = -1
        self._started = False
        self._closed = False

    def start(self, provenance: EpisodeProvenance, timestamp_ns: int) -> str:
        if self._started:
            raise RuntimeError("episode already started")
        if timestamp_ns < 0:
            raise ValueError("episode time must be nonnegative")
        self.directory.mkdir(parents=True, exist_ok=True)
        self._stream = self.partial_path.open("xb")
        fcntl.flock(self._stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self._started = True
        self._timestamp = timestamp_ns
        self._writer = Writer(
            self._stream,
            chunk_size=1,
            compression=CompressionType.ZSTD,
            enable_crcs=True,
        )
        self._writer.start(profile=PROFILE)
        values = asdict(provenance)
        values.update(
            timestamp_ns=timestamp_ns, episode_id=self.episode_id, schema_version=1
        )
        try:
            self._write("/episode/provenance", "Provenance", values, timestamp_ns)
            self.event(timestamp_ns, "start", EpisodeOutcome.UNKNOWN, "")
            self._sync()
            sync_directory(self.directory)
        except BaseException:
            self.interrupt()
            raise
        return self.episode_id

    def _require_open(self) -> Writer:
        if self._writer is None or self._closed:
            raise RuntimeError("episode is not recording")
        return self._writer

    def _write(
        self, topic: str, name: str, values: dict[str, Any], timestamp_ns: int
    ) -> None:
        writer = self._require_open()
        if timestamp_ns < self._timestamp:
            raise ValueError("episode timestamps must be monotonic")
        payload = encode(name, values)
        if name not in self._schemas:
            self._schemas[name] = writer.register_schema(
                f"{PROFILE}.{name}", "protobuf", descriptor_bytes()
            )
        if topic not in self._channels:
            self._channels[topic] = writer.register_channel(
                topic,
                "protobuf",
                self._schemas[name],
                metadata={"clock": "simulation", "schema_version": "1"},
            )
            self._sequences[topic] = 0
        writer.add_message(
            self._channels[topic],
            timestamp_ns,
            payload,
            timestamp_ns,
            sequence=self._sequences[topic],
        )
        self._sequences[topic] += 1
        self._timestamp = timestamp_ns

    def append(self, sample: EpisodeSample) -> None:
        self._require_open()
        stamp = sample.observation.timestamp_ns
        if stamp <= self._sample_timestamp or stamp < self._timestamp:
            raise ValueError("sample timestamps must strictly increase")
        if (
            sample.observation.robot != sample.command.resulting_state
            or sample.observation.robot.timestamp_ns != stamp
        ):
            raise ValueError("observation and command result must be synchronized")
        names = [name for name, _ in sample.images]
        if len(names) != len(set(names)):
            raise ValueError("duplicate image camera")
        for name, frame in sample.images:
            if (
                name not in sample.observation.image_keys
                or frame.timestamp_ns != stamp
                or frame.width <= 0
                or frame.height <= 0
                or len(frame.rgb_bytes) != frame.width * frame.height * 3
            ):
                raise ValueError("invalid or unsynchronized scene image")
        self._write("/observation", "Observation", asdict(sample.observation), stamp)
        report = asdict(sample.command)
        report["timestamp_ns"] = stamp
        self._write("/command", "Command", report, stamp)
        for name, frame in sample.images:
            self._write(
                f"/camera/{name}",
                "Image",
                {
                    "timestamp_ns": stamp,
                    "camera_id": name,
                    "width": frame.width,
                    "height": frame.height,
                    "encoding": "rgb8",
                    "data": base64.b64encode(frame.rgb_bytes).decode("ascii"),
                },
                stamp,
            )
        self._sample_timestamp = stamp
        self._sync()

    def event(
        self, timestamp_ns: int, kind: str, outcome: EpisodeOutcome, reason: str
    ) -> None:
        self._write(
            "/episode/event",
            "Event",
            {
                "timestamp_ns": timestamp_ns,
                "episode_id": self.episode_id,
                "kind": kind,
                "outcome": outcome.value,
                "reason": reason,
            },
            timestamp_ns,
        )
        self._sync()

    def diagnostics(
        self,
        timestamp_ns: int,
        host_monotonic_ns: int,
        values_json: str,
        calibration_json: str,
    ) -> None:
        self._write(
            "/teleop/diagnostics",
            "Diagnostics",
            {
                "timestamp_ns": timestamp_ns,
                "host_monotonic_ns": host_monotonic_ns,
                "values_json": values_json,
                "calibration_json": calibration_json,
            },
            timestamp_ns,
        )
        self._sync()

    def stop(self, outcome: EpisodeOutcome, reason: str) -> None:
        writer = self._require_open()
        if outcome in {EpisodeOutcome.UNKNOWN, EpisodeOutcome.INTERRUPTED}:
            raise ValueError("stop requires success, failure, or discarded")
        if not reason.strip():
            raise ValueError("an outcome reason is required")
        self.event(
            self._timestamp,
            "discard" if outcome is EpisodeOutcome.DISCARDED else "stop",
            outcome,
            reason,
        )
        try:
            writer.finish()  # type: ignore[no-untyped-call]
            self._sync()
            publish(self.partial_path, self.final_path)
        finally:
            self.interrupt()

    def discard(self, reason: str) -> None:
        self.stop(EpisodeOutcome.DISCARDED, reason)

    def _sync(self) -> None:
        assert self._stream is not None
        self._stream.flush()
        os.fsync(self._stream.fileno())

    def interrupt(self) -> None:
        """Close without a footer; leave the durable prefix for explicit recovery."""
        if self._stream is not None:
            self._stream.close()
        self._closed = True
