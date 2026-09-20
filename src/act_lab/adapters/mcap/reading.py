"""Translate versioned MCAP messages into framework-neutral episode records."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from mcap.reader import make_reader

from act_lab.adapters.mcap.inspection import inspect_episode
from act_lab.adapters.mcap.schema import decode
from act_lab.domain.dataset import RecordedEpisode, RecordedImage, RecordedSample


def _tuple(value: Any) -> tuple[float, ...]:
    return tuple(float(item) for item in value)


def read_episode(path: Path) -> RecordedEpisode:
    inspection = inspect_episode(path)
    observations: dict[int, Any] = {}
    commands: dict[int, Any] = {}
    images: dict[int, list[RecordedImage]] = {}
    tracking_states: list[tuple[int, str]] = []
    with path.open("rb") as source:
        for schema, channel, record in make_reader(source).iter_messages():
            if schema is None:
                raise ValueError(f"channel {channel.topic} has no schema")
            name = schema.name.rsplit(".", 1)[-1]
            message = decode(name, record.data)
            if channel.topic == "/observation":
                observations[record.log_time] = message
            elif channel.topic == "/command":
                commands[record.log_time] = message
            elif channel.topic.startswith("/camera/"):
                images.setdefault(record.log_time, []).append(
                    RecordedImage(
                        message.camera_id,
                        int(message.width),
                        int(message.height),
                        message.encoding,
                        bytes(message.data),
                    )
                )
            elif channel.topic == "/teleop/diagnostics":
                values = json.loads(message.values_json)
                tracking_states.append(
                    (record.log_time, str(values.get("state", "unknown")))
                )
    samples: list[RecordedSample] = []
    for timestamp in sorted(set(observations) & set(commands)):
        observation = observations[timestamp]
        command = commands[timestamp]
        state = observation.robot
        action = (
            command.executed_action
            if command.HasField("executed_action")
            else command.requested_action
        )
        samples.append(
            RecordedSample(
                timestamp,
                _tuple(state.joint_positions_rad),
                _tuple(state.joint_velocities_rad_s),
                _tuple(state.end_effector_pose.position_xyz_m),
                _tuple(state.end_effector_pose.quaternion_wxyz),
                float(state.gripper_position),
                _tuple(action.target_pose.position_xyz_m),
                _tuple(action.target_pose.quaternion_wxyz),
                float(action.gripper_position),
                bool(action.enabled),
                command.outcome,
                tuple(
                    sorted(images.get(timestamp, ()), key=lambda item: item.camera_id)
                ),
            )
        )
    provenance = inspection["provenance"] or {}
    return RecordedEpisode(
        str(provenance.get("episode_id", "")),
        str(inspection["outcome"]),
        str(inspection["last_task_outcome"]),
        bool(inspection["complete"]),
        provenance,
        tuple(samples),
        tuple(
            sorted(
                (topic, int(details["count"]))
                for topic, details in inspection["streams"].items()
            )
        ),
        tuple(tracking_states),
    )
