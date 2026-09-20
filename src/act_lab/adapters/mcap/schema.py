"""Load the build-generated descriptor; keep Protobuf types in this adapter."""

import math
from functools import cache
from importlib.metadata import distribution
from pathlib import Path
from typing import Any

from google.protobuf import (  # type: ignore[import-untyped]
    descriptor_pb2,
    descriptor_pool,
    message_factory,
)
from google.protobuf.json_format import ParseDict  # type: ignore[import-untyped]


@cache
def descriptor_bytes() -> bytes:
    return Path(
        str(distribution("act-lab").locate_file("act_lab_episode.desc"))
    ).read_bytes()


@cache
def foxglove_descriptor_bytes() -> bytes:
    return Path(
        str(distribution("act-lab").locate_file("act_lab_foxglove.desc"))
    ).read_bytes()


@cache
def message_class(name: str) -> Any:
    return qualified_message_class(f"act_lab.recording.v1.{name}")


@cache
def qualified_message_class(name: str) -> Any:
    data = (
        foxglove_descriptor_bytes()
        if name.startswith("foxglove.")
        else descriptor_bytes()
    )
    descriptors = descriptor_pb2.FileDescriptorSet.FromString(data)
    pool = descriptor_pool.DescriptorPool()
    for file in descriptors.file:
        pool.Add(file)
    return message_factory.GetMessageClass(pool.FindMessageTypeByName(name))


def encode(name: str, values: dict[str, Any]) -> bytes:
    message = ParseDict(_wire_values(values), message_class(name)())
    return bytes(message.SerializeToString(deterministic=True))


def _wire_values(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _wire_values(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_wire_values(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return "NaN" if math.isnan(value) else "Infinity" if value > 0 else "-Infinity"
    return value


def decode(name: str, data: bytes) -> Any:
    return message_class(name).FromString(data)
