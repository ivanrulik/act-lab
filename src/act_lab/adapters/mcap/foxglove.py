"""Create an immutable Foxglove-viewable derivative of a raw episode."""

from __future__ import annotations

import os
from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import CompressionType, Writer

from act_lab.adapters.mcap.recording import PROFILE, publish
from act_lab.adapters.mcap.schema import (
    decode,
    foxglove_descriptor_bytes,
    qualified_message_class,
)


def export_foxglove(source: Path, output: Path) -> dict[str, object]:
    """Copy an episode and add standard RawImage topics without changing raw data."""
    if source.resolve() == output.resolve():
        raise ValueError("Foxglove output must differ from the raw recording")
    if output.exists():
        raise FileExistsError(f"output already exists: {output}")
    partial = output.with_name(output.name + ".partial")
    if partial.exists():
        raise FileExistsError(f"partial export already exists: {partial}")
    output.parent.mkdir(parents=True, exist_ok=True)

    image_class = qualified_message_class("foxglove.RawImage")
    converted = 0
    with source.open("rb") as raw, partial.open("xb") as target:
        reader = make_reader(raw)
        writer = Writer(target, chunk_size=1, compression=CompressionType.ZSTD)
        writer.start(profile=PROFILE)
        schema_ids: dict[int, int] = {}
        channel_ids: dict[int, int] = {}
        foxglove_schema = writer.register_schema(
            "foxglove.RawImage", "protobuf", foxglove_descriptor_bytes()
        )
        foxglove_channels: dict[str, int] = {}

        for schema, channel, message in reader.iter_messages():
            if schema is None:
                raise ValueError(f"channel {channel.topic} has no schema")
            if schema.id not in schema_ids:
                schema_ids[schema.id] = writer.register_schema(
                    schema.name, schema.encoding, schema.data
                )
            if channel.id not in channel_ids:
                channel_ids[channel.id] = writer.register_channel(
                    channel.topic,
                    channel.message_encoding,
                    schema_ids[schema.id],
                    channel.metadata,
                )
            writer.add_message(
                channel_ids[channel.id],
                message.log_time,
                message.data,
                message.publish_time,
                message.sequence,
            )
            if channel.topic.startswith("/camera/"):
                image = decode("Image", message.data)
                topic = f"/foxglove{channel.topic}"
                if topic not in foxglove_channels:
                    foxglove_channels[topic] = writer.register_channel(
                        topic,
                        "protobuf",
                        foxglove_schema,
                        {"source_topic": channel.topic},
                    )
                raw_image = image_class(
                    frame_id=image.camera_id,
                    width=image.width,
                    height=image.height,
                    encoding=image.encoding,
                    step=image.width * 3,
                    data=image.data,
                )
                raw_image.timestamp.seconds = message.log_time // 1_000_000_000
                raw_image.timestamp.nanos = message.log_time % 1_000_000_000
                writer.add_message(
                    foxglove_channels[topic],
                    message.log_time,
                    raw_image.SerializeToString(deterministic=True),
                    message.publish_time,
                    message.sequence,
                )
                converted += 1
        for metadata in reader.iter_metadata():
            writer.add_metadata(metadata.name, metadata.metadata)
        writer.add_metadata(
            "act_lab.foxglove_export",
            {"source_name": source.name, "converted_images": str(converted)},
        )
        writer.finish()  # type: ignore[no-untyped-call]
        target.flush()
        os.fsync(target.fileno())
    publish(partial, output)
    return {"output": str(output), "converted_images": converted}
