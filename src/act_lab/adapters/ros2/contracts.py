"""Primitive transport envelopes and simulation-only wire conversions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any
from uuid import UUID

from act_lab.domain import Action, CommandOutcome, CommandReport, Pose, RobotState

CLOCK_OFFSET_NS = 1_000_000_000
WATCHDOG_NS = 100_000_000
JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
TOPICS = {
    "command": "/act_lab/v1/command",
    "state": "/act_lab/v1/state",
    "report": "/act_lab/v1/command_report",
    "clock": "/clock",
}


@dataclass(frozen=True, slots=True)
class CommandEnvelope:
    episode_id: str
    sequence: int
    action: Action

    def validate(self) -> None:
        validate_episode(self.episode_id)
        integer(self.sequence, 0, 2**64 - 1)
        if not isinstance(self.action, Action) or not isinstance(
            self.action.target_pose, Pose
        ):
            raise ValueError("envelope must contain domain intent")
        if type(self.action.enabled) is not bool or not isinstance(
            self.action.target_pose.frame_id, str
        ):
            raise ValueError("malformed enabled flag or frame")
        if (
            len(self.action.target_pose.position_xyz_m) != 3
            or len(self.action.target_pose.quaternion_wxyz) != 4
        ):
            raise ValueError("malformed pose dimensions")
        for value in (
            *self.action.target_pose.position_xyz_m,
            *self.action.target_pose.quaternion_wxyz,
            self.action.gripper_position,
        ):
            # Shape/type checks only; finite/range checks belong to Safety.
            number(value)
        if type(self.action.timestamp_ns) is not int:
            raise ValueError("domain timestamp must be an integer")


def validate_episode(value: str) -> None:
    if not isinstance(value, str):
        raise ValueError("episode_id must be a UUID string")
    try:
        if str(UUID(value)) != value:
            raise ValueError("episode_id must be a canonical UUID")
    except (ValueError, AttributeError) as error:
        raise ValueError("episode_id must be a canonical UUID") from error


def integer(value: Any, minimum: int, maximum: int) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"integer must be in [{minimum}, {maximum}]")
    return value


def number(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("wire value must be numeric")
    return float(value)


def encode_stamp(domain_ns: int) -> dict[str, int]:
    if type(domain_ns) is not int:
        raise ValueError("domain timestamp must be an integer")
    seconds, nanos = divmod(domain_ns + CLOCK_OFFSET_NS, CLOCK_OFFSET_NS)
    integer(seconds, -(2**31), 2**31 - 1)
    return {"sec": seconds, "nanosec": nanos}


def decode_stamp(stamp: dict[str, Any]) -> int:
    seconds = integer(stamp["sec"], -(2**31), 2**31 - 1)
    nanos = integer(stamp["nanosec"], 0, CLOCK_OFFSET_NS - 1)
    return seconds * CLOCK_OFFSET_NS + nanos - CLOCK_OFFSET_NS


def encode_pose(pose: Pose) -> dict[str, Any]:
    x, y, z = pose.position_xyz_m
    w, qx, qy, qz = pose.quaternion_wxyz
    return {
        "position": {"x": x, "y": y, "z": z},
        "orientation": {"x": qx, "y": qy, "z": qz, "w": w},
    }


def decode_pose(fields: dict[str, Any], frame: str) -> Pose:
    if not isinstance(frame, str):
        raise ValueError("frame must be a string")
    p, q = fields["position"], fields["orientation"]
    return Pose(
        frame,
        (number(p["x"]), number(p["y"]), number(p["z"])),
        (number(q["w"]), number(q["x"]), number(q["y"]), number(q["z"])),
    )


def encode_command(envelope: CommandEnvelope) -> dict[str, Any]:
    envelope.validate()
    action = envelope.action
    return {
        "header": {
            "stamp": encode_stamp(action.timestamp_ns),
            "frame_id": action.target_pose.frame_id,
        },
        "episode_id": envelope.episode_id,
        "sequence": envelope.sequence,
        "target_pose": encode_pose(action.target_pose),
        "gripper_position": action.gripper_position,
        "enabled": action.enabled,
    }


def decode_command(fields: dict[str, Any]) -> CommandEnvelope:
    if type(fields["enabled"]) is not bool:
        raise ValueError("enabled must be boolean")
    envelope = CommandEnvelope(
        fields["episode_id"],
        fields["sequence"],
        Action(
            decode_stamp(fields["header"]["stamp"]),
            decode_pose(fields["target_pose"], fields["header"]["frame_id"]),
            number(fields["gripper_position"]),
            fields["enabled"],
        ),
    )
    envelope.validate()
    return envelope


def encode_state(state: RobotState, episode_id: str) -> dict[str, Any]:
    validate_episode(episode_id)
    if len(state.joint_positions_rad) != 6 or len(state.joint_velocities_rad_s) != 6:
        raise ValueError("UR5e state requires six positions and velocities")
    return {
        "header": {
            "stamp": encode_stamp(state.timestamp_ns),
            "frame_id": state.end_effector_pose.frame_id,
        },
        "episode_id": episode_id,
        "joint_names": list(JOINT_NAMES),
        "joint_positions_rad": list(state.joint_positions_rad),
        "joint_velocities_rad_s": list(state.joint_velocities_rad_s),
        "end_effector_pose": encode_pose(state.end_effector_pose),
        "gripper_position": state.gripper_position,
    }


def decode_state(fields: dict[str, Any]) -> tuple[RobotState, str]:
    validate_episode(fields["episode_id"])
    names = fields["joint_names"]
    if len(names) != 6 or set(names) != set(JOINT_NAMES):
        raise ValueError("state must identify all six unique UR5e joints")
    positions, velocities = (
        fields["joint_positions_rad"],
        fields["joint_velocities_rad_s"],
    )
    if len(positions) != 6 or len(velocities) != 6:
        raise ValueError("state must contain six positions and velocities")
    order = [names.index(name) for name in JOINT_NAMES]
    return RobotState(
        decode_stamp(fields["header"]["stamp"]),
        tuple(number(positions[index]) for index in order),
        tuple(number(velocities[index]) for index in order),
        decode_pose(fields["end_effector_pose"], fields["header"]["frame_id"]),
        number(fields["gripper_position"]),
    ), fields["episode_id"]


def encode_report(
    report: CommandReport, episode_id: str, sequence: int
) -> dict[str, Any]:
    state = encode_state(report.resulting_state, episode_id)
    result = {
        "header": state["header"],
        "episode_id": episode_id,
        "command_sequence": sequence,
        "outcome": report.outcome.value,
        "requested_command": encode_command(
            CommandEnvelope(episode_id, sequence, report.requested_action)
        ),
        "has_executed_command": report.executed_action is not None,
        "resulting_state": state,
        "detail": report.detail,
        "commanded_cartesian_speed_m_s": report.commanded_cartesian_speed_m_s,
        "measured_cartesian_speed_m_s": report.measured_cartesian_speed_m_s,
        "tracking_error_m": report.tracking_error_m,
        "command_age_ms": report.command_age_ms,
    }
    if report.executed_action is not None:
        result["executed_command"] = encode_command(
            CommandEnvelope(episode_id, sequence, report.executed_action)
        )
    return result


def decode_report(fields: dict[str, Any]) -> tuple[CommandReport, str, int]:
    state, episode = decode_state(fields["resulting_state"])
    sequence = integer(fields["command_sequence"], 0, 2**64 - 1)
    if (
        fields["episode_id"] != episode
        or fields["header"] != fields["resulting_state"]["header"]
    ):
        raise ValueError("report header/episode disagrees with measured state")
    if not isinstance(fields["detail"], str):
        raise ValueError("report detail must be a string")
    requested = decode_command(fields["requested_command"])
    if type(fields["has_executed_command"]) is not bool:
        raise ValueError("has_executed_command must be boolean")
    executed = (
        decode_command(fields["executed_command"])
        if fields["has_executed_command"]
        else None
    )
    for command in (requested, executed):
        if command and (command.episode_id != episode or command.sequence != sequence):
            raise ValueError("report command metadata disagrees with envelope")
    return (
        CommandReport(
            CommandOutcome(fields["outcome"]),
            requested.action,
            executed.action if executed else None,
            state,
            fields["detail"],
            number(fields["commanded_cartesian_speed_m_s"]),
            number(fields["measured_cartesian_speed_m_s"]),
            number(fields["tracking_error_m"]),
            number(fields["command_age_ms"]),
        ),
        episode,
        sequence,
    )
