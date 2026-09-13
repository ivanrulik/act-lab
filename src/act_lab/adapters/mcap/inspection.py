"""Structural inspection and prefix recovery; no training-quality validation."""

from __future__ import annotations

import fcntl
import hashlib
import io
import os
from pathlib import Path
from typing import Any

from mcap.exceptions import EndOfFile
from mcap.records import Channel, Footer, Header, Message, Schema
from mcap.stream_reader import StreamReader
from mcap.writer import CompressionType, Writer

from act_lab.adapters.mcap.recording import PROFILE, publish
from act_lab.adapters.mcap.schema import decode, descriptor_bytes, encode


class _ExactReader(io.BufferedReader):
    """MCAP 1.3 only checks empty reads; turn short tail reads into EOF too."""

    def read(self, size: int | None = -1) -> bytes:
        data = super().read(size)
        if size is not None and size >= 0 and len(data) != size:
            raise EndOfFile()
        return data


def inspect_episode(path: Path) -> dict[str, Any]:
    channels: dict[int, Channel] = {}
    schemas: dict[int, Schema] = {}
    streams: dict[str, dict[str, Any]] = {}
    events: list[dict[str, Any]] = []
    provenance: dict[str, Any] | None = None
    complete = False
    previous = -1
    monotonic = True
    with path.open("rb") as raw, _ExactReader(raw) as source:
        try:
            for record in StreamReader(source, validate_crcs=True).records:
                if isinstance(record, Header) and record.profile != PROFILE:
                    raise ValueError("unsupported MCAP recording profile")
                if isinstance(record, Schema):
                    schemas[record.id] = record
                elif isinstance(record, Channel):
                    channels[record.id] = record
                elif isinstance(record, Message):
                    channel = channels[record.channel_id]
                    schema = schemas[channel.schema_id]
                    if schema.encoding != "protobuf" or not schema.name.startswith(
                        PROFILE + "."
                    ):
                        raise ValueError("unsupported recording schema")
                    monotonic &= record.log_time >= previous
                    previous = record.log_time
                    stream = streams.setdefault(
                        channel.topic,
                        {
                            "schema": schema.name,
                            "count": 0,
                            "first_timestamp_ns": record.log_time,
                            "last_timestamp_ns": record.log_time,
                        },
                    )
                    stream["count"] += 1
                    stream["last_timestamp_ns"] = record.log_time
                    name = schema.name.rsplit(".", 1)[1]
                    msg = decode(name, record.data)
                    if msg.timestamp_ns != record.log_time:
                        raise ValueError("payload and MCAP timestamps disagree")
                    if name == "Event":
                        events.append(
                            {
                                key: getattr(msg, key)
                                for key in (
                                    "timestamp_ns",
                                    "episode_id",
                                    "kind",
                                    "outcome",
                                    "reason",
                                )
                            }
                        )
                    elif name == "Provenance":
                        provenance = {
                            key: getattr(msg, key)
                            for key in (
                                "episode_id",
                                "schema_version",
                                "application_version",
                                "git_revision",
                                "git_dirty",
                                "seed",
                                "source",
                                "clock_id",
                                "resolved_config_json",
                                "rates_json",
                                "calibration_json",
                                "container_identity",
                            )
                        }
                elif isinstance(record, Footer):
                    complete = True
        except EndOfFile:
            complete = False
    for stream in streams.values():
        duration = stream["last_timestamp_ns"] - stream["first_timestamp_ns"]
        stream["observed_hz"] = (
            (stream["count"] - 1) * 1e9 / duration if duration > 0 else None
        )
    lifecycle_events = [
        event for event in events if event["kind"] in {"stop", "discard", "interrupted"}
    ]
    task_events = [event for event in events if event["kind"] == "task"]
    return {
        "complete": complete,
        "monotonic": monotonic,
        "streams": streams,
        "provenance": provenance,
        "events": events,
        "outcome": (
            lifecycle_events[-1]["outcome"]
            if complete and lifecycle_events
            else "unknown"
        ),
        "last_task_outcome": task_events[-1]["outcome"] if task_events else "unknown",
        "training_validated": False,
    }


def recover_episode(path: Path) -> Path:
    """Copy CRC-checked complete records, retaining original bytes and lineage."""
    if not path.name.endswith(".mcap.partial"):
        raise ValueError("recovery requires a .mcap.partial file")
    final = path.with_name(path.name.removesuffix(".mcap.partial") + ".recovered.mcap")
    partial = final.with_name(final.name + ".partial")
    # Lock the actual source inode, including across processes. Never recover a
    # file still being written by a live acquisition process.
    with path.open("rb") as raw, _ExactReader(raw) as source:
        fcntl.flock(source, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if final.exists():
            raise FileExistsError(final)
        digest = hashlib.file_digest(source, "sha256").hexdigest()
        source.seek(0)
        with partial.open("xb") as target:
            fcntl.flock(target, fcntl.LOCK_EX | fcntl.LOCK_NB)
            writer = Writer(target, chunk_size=1, compression=CompressionType.ZSTD)
            writer.start(profile=PROFILE)
            schemas: dict[int, int] = {}
            channels: dict[int, int] = {}
            channel_names: dict[int, str] = {}
            episode_id = ""
            timestamp = 0
            copied = 0
            try:
                for record in StreamReader(source, validate_crcs=True).records:
                    if isinstance(record, Header) and record.profile != PROFILE:
                        raise ValueError("unsupported MCAP recording profile")
                    if isinstance(record, Schema) and record.id not in schemas:
                        schemas[record.id] = writer.register_schema(
                            record.name, record.encoding, record.data
                        )
                    elif isinstance(record, Channel) and record.id not in channels:
                        channels[record.id] = writer.register_channel(
                            record.topic,
                            record.message_encoding,
                            schemas[record.schema_id],
                            record.metadata,
                        )
                        channel_names[record.id] = record.topic
                    elif isinstance(record, Message):
                        if record.log_time < timestamp:
                            raise ValueError("cannot recover non-monotonic recording")
                        if channel_names[record.channel_id] == "/episode/provenance":
                            episode_id = decode("Provenance", record.data).episode_id
                        writer.add_message(
                            channels[record.channel_id],
                            record.log_time,
                            record.data,
                            record.publish_time,
                            record.sequence,
                        )
                        timestamp = record.log_time
                        copied += 1
            except EndOfFile:
                pass
            if not episode_id:
                raise ValueError("no complete provenance; cannot identify episode")
            schema_id = writer.register_schema(
                PROFILE + ".Event", "protobuf", descriptor_bytes()
            )
            channel_id = writer.register_channel(
                "/episode/recovery", "protobuf", schema_id
            )
            writer.add_message(
                channel_id,
                timestamp,
                encode(
                    "Event",
                    {
                        "timestamp_ns": timestamp,
                        "episode_id": episode_id,
                        "kind": "interrupted",
                        "outcome": "interrupted",
                        "reason": "recovered prefix; trailing records may be lost",
                    },
                ),
                timestamp,
            )
            writer.add_metadata(
                "act_lab.recovery",
                {
                    "source_name": path.name,
                    "source_sha256": digest,
                    "copied_messages": str(copied),
                },
            )
            writer.finish()  # type: ignore[no-untyped-call]
            target.flush()
            os.fsync(target.fileno())
        publish(partial, final)
    return final
