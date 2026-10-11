"""ROS-free acquisition codecs and ordered episode lifecycle validation."""

from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import asdict, fields
from typing import Any

from act_lab.adapters.ros2.contracts import (
    decode_report,
    decode_stamp,
    encode_report,
    encode_stamp,
    integer,
    validate_episode,
)
from act_lab.domain.models import CameraFrame, Observation
from act_lab.domain.ports import EpisodeSink
from act_lab.domain.recording import EpisodeOutcome, EpisodeProvenance, EpisodeSample

PACKET_TOPIC = "/act_lab/recording/v1/packet"
ACK_TOPIC = "/act_lab/recording/v1/ack"
PACKET_TYPE = "act_lab_interfaces/msg/RecordingPacket"


def encode_sample(
    sample: EpisodeSample,
    episode: str,
    sequence: int,
    command_sequence: int | None = None,
) -> dict[str, Any]:
    stamp = sample.observation.timestamp_ns
    if sample.observation.robot != sample.command.resulting_state:
        raise ValueError("recorded observation must match command result")
    result = dict(
        header=dict(stamp=encode_stamp(stamp), frame_id="world"),
        report=encode_report(
            sample.command,
            episode,
            sequence if command_sequence is None else command_sequence,
        ),
        image_keys=list(sample.observation.image_keys),
        camera_ids=[name for name, _ in sample.images],
        images=[
            dict(
                header=dict(stamp=encode_stamp(frame.timestamp_ns), frame_id=name),
                width=frame.width,
                height=frame.height,
                encoding="rgb8",
                is_bigendian=0,
                step=frame.width * 3,
                data=frame.rgb_bytes,
            )
            for name, frame in sample.images
        ],
    )
    decode_sample(result, episode, sequence)
    return result


def decode_sample(value: dict[str, Any], episode: str, sequence: int) -> EpisodeSample:
    integer(sequence, 1, 2**64 - 1)
    stamp = decode_stamp(value["header"]["stamp"])
    report, report_episode, _ = decode_report(value["report"])
    if (
        stamp < 0
        or report.resulting_state.timestamp_ns != stamp
        or report_episode != episode
        or value["header"]["frame_id"] != "world"
    ):
        raise ValueError("sample header and report disagree")
    keys, names, images = value["image_keys"], value["camera_ids"], value["images"]
    if (
        not keys
        or any(not isinstance(name, str) or not name for name in keys)
        or len(set(keys)) != len(keys)
        or len(set(names)) != len(names)
        or set(names) != set(keys)
        or len(images) != len(names)
    ):
        raise ValueError("record every declared camera exactly once")
    frames = []
    for name, image in zip(names, images, strict=True):
        width = integer(image["width"], 1, 16384)
        height = integer(image["height"], 1, 16384)
        if (
            decode_stamp(image["header"]["stamp"]) != stamp
            or image["header"]["frame_id"] != name
            or image["encoding"] != "rgb8"
            or image["is_bigendian"] != 0
            or image["step"] != width * 3
            or len(image["data"]) != width * height * 3
        ):
            raise ValueError("malformed or unsynchronized RGB image")
        data = bytes(image["data"])
        frames.append((name, CameraFrame(stamp, width, height, data)))
    return EpisodeSample(
        Observation(stamp, report.resulting_state, tuple(keys)), report, tuple(frames)
    )


def packet(
    episode: str, sequence: int, timestamp: int, kind: str, **extra: Any
) -> dict[str, Any]:
    validate_episode(episode)
    integer(sequence, 1, 2**64 - 1)
    integer(timestamp, 0, 2**63 - 1)
    result = dict(
        header=dict(stamp=encode_stamp(timestamp), frame_id="world"),
        schema_version=1,
        episode_id=episode,
        sequence=sequence,
        kind=kind,
        metadata_json="{}",
        outcome="unknown",
        reason="",
    )
    result.update(extra)
    return result


def provenance_json(provenance: EpisodeProvenance) -> str:
    return json.dumps(asdict(provenance), sort_keys=True, allow_nan=False)


def decode_provenance(text: str) -> EpisodeProvenance:
    value = json.loads(text)
    if not isinstance(value, dict) or set(value) != {
        f.name for f in fields(EpisodeProvenance)
    }:
        raise ValueError("provenance schema mismatch")
    for name, item in value.items():
        if name in {"seed", "host_monotonic_start_ns"}:
            integer(item, 0 if name != "seed" else -(2**63), 2**63 - 1)
        elif name == "camera_ids":
            if (
                not isinstance(item, list)
                or not item
                or any(not isinstance(n, str) or not n for n in item)
                or len(set(item)) != len(item)
            ):
                raise ValueError("invalid provenance cameras")
            value[name] = tuple(item)
        elif not isinstance(item, str):
            raise ValueError("provenance string required")
    for name in ("resolved_config_json", "rates_json", "calibration_json"):
        if not isinstance(json.loads(value[name]), dict):
            raise ValueError("provenance JSON object required")
    return EpisodeProvenance(**value)


class PacketConsumer:
    """Fail closed on packet loss/replay; unfinished attempts remain partial."""

    def __init__(self, factory: Callable[[str], EpisodeSink]) -> None:
        self.factory = factory
        self.sequence = 0
        self.sink: EpisodeSink | None = None
        self.episode: str | None = None
        self.closed: set[str] = set()
        self.timestamp = -1
        self.sample_timestamp = -1
        self.cameras: tuple[str, ...] = ()
        self.samples = 0

    def consume(self, value: dict[str, Any]) -> None:
        episode = value["episode_id"]
        validate_episode(episode)
        sequence = integer(value["sequence"], 1, 2**64 - 1)
        stamp = decode_stamp(value["header"]["stamp"])
        integer(value["schema_version"], 1, 1)
        if (
            value["schema_version"] != 1
            or value["header"]["frame_id"] != "world"
            or stamp < 0
        ):
            raise ValueError("unsupported recording header")
        if sequence != self.sequence + 1:
            raise ValueError("recording sequence gap or replay")
        kind = value["kind"]
        if kind == "start":
            if self.sink is not None or episode in self.closed:
                raise ValueError(
                    "episode start requires a new identity after stop/reset"
                )
            provenance = decode_provenance(value["metadata_json"])
            sink = self.factory(episode)
            if sink.start(provenance, stamp) != episode:
                sink.interrupt()
                raise ValueError("sink did not retain episode identity")
            self.sink, self.episode = sink, episode
            self.cameras = provenance.camera_ids
            self.sample_timestamp = -1
        else:
            if self.sink is None or episode != self.episode or stamp < self.timestamp:
                raise ValueError("missing lifecycle, wrong episode or backward clock")
            if kind == "sample":
                sample = decode_sample(value["sample"], episode, sequence)
                if (
                    sample.observation.timestamp_ns != stamp
                    or stamp <= self.sample_timestamp
                    or set(sample.observation.image_keys) != set(self.cameras)
                ):
                    raise ValueError("sample clock or cameras disagree with episode")
                self.sink.append(sample)
                self.sample_timestamp = stamp
                self.samples += 1
            elif kind == "task":
                self.sink.event(
                    stamp, kind, EpisodeOutcome(value["outcome"]), value["reason"]
                )
            elif kind == "diagnostics":
                meta = json.loads(value["metadata_json"])
                self.sink.diagnostics(
                    stamp,
                    integer(meta["host_monotonic_ns"], 0, 2**63 - 1),
                    meta["values_json"],
                    meta["calibration_json"],
                )
            elif kind in {"stop", "discard"}:
                outcome = EpisodeOutcome(value["outcome"])
                if stamp != self.timestamp or (kind == "discard") != (
                    outcome is EpisodeOutcome.DISCARDED
                ):
                    raise ValueError("invalid terminal event")
                self.sink.stop(outcome, value["reason"])
                self.closed.add(episode)
                self.sink = None
            else:
                raise ValueError("unknown recording packet")
        self.sequence, self.timestamp = sequence, stamp

    def finish(self) -> None:
        if self.sink is not None:
            self.sink.interrupt()
            self.sink = None
            raise ValueError("interrupted episode: terminal event missing")
        if not self.closed:
            raise ValueError("recording has no completed episodes")
