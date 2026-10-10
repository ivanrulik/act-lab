"""Pure projections for read-only views; ROS imports live in the observer process."""

from __future__ import annotations

import math
from typing import Any

from act_lab.adapters.ros2.contracts import JOINT_NAMES

VIEW_TOPICS = {
    "joint": "/joint_states",
    "telemetry": "/act_lab/view/telemetry",
    "events": "/act_lab/view/events",
    "poses": "/act_lab/view/poses",
    "health": "/act_lab/view/health",
    "description": "/act_lab/view/robot_description",
}


def joint_fields(packet: dict[str, Any]) -> dict[str, Any]:
    state = packet["state"]
    if tuple(state["joint_names"]) != JOINT_NAMES:
        raise ValueError("observational joint order must be canonical")
    return dict(
        header=state["header"],
        name=list(JOINT_NAMES),
        position=state["joint_positions_rad"],
        velocity=state["joint_velocities_rad_s"],
        effort=packet["telemetry"]["total_effort_nm"],
    )


def marker_fields(packet: dict[str, Any], stale: bool = False) -> dict[str, Any]:
    t = packet["telemetry"]
    markers = [dict(header=t["header"], ns="act_lab", id=0, action=3)]
    for index, (key, color) in enumerate(
        (
            ("requested", (1.0, 0.6, 0.1)),
            ("approved", (0.0, 0.9, 0.9)),
            ("measured_pose", (0.2, 1.0, 0.2)),
        ),
        start=1,
    ):
        pose = packet.get(key)
        if (
            pose is None
            or stale
            or (key == "requested" and packet.get("requested_frame") != "world")
        ):
            continue
        try:
            numbers = [
                float(v)
                for v in (*pose["position"].values(), *pose["orientation"].values())
            ]
            if not all(math.isfinite(v) for v in numbers):
                continue
        except (ValueError, TypeError, KeyError):
            continue
        markers.append(
            dict(
                header=t["header"],
                ns="act_lab",
                id=index,
                action=0,
                type=0,
                pose=pose,
                scale=dict(x=0.08, y=0.006, z=0.006),
                color=dict(r=color[0], g=color[1], b=color[2], a=1.0),
                lifetime=dict(sec=0, nanosec=100_000_000),
            )
        )
    label = "STALE / unavailable" if stale else t["mode"] + "\n" + t["reason"]
    markers.append(
        dict(
            header=t["header"],
            ns="act_lab",
            id=4,
            type=9,
            action=0,
            pose=dict(
                position=dict(x=0.35, y=0.0, z=1.1),
                orientation=dict(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            scale=dict(x=0.0, y=0.0, z=0.035),
            color=dict(r=1.0, g=0.2 if stale else 1.0, b=0.2 if stale else 1.0, a=1.0),
            text=f"{label}\n"
            f"gripper {t['accepted_gripper']:.3f}\n"
            f"episode {t['episode_id'][:8]}",
        )
    )
    return dict(markers=markers)


def scene_tf(packet: dict[str, Any]) -> dict[str, Any]:
    pose = packet["measured_pose"]
    return dict(
        transforms=[
            dict(
                header=packet["telemetry"]["header"],
                child_frame_id="act_lab_scene_tip",
                transform=dict(
                    translation=pose["position"], rotation=pose["orientation"]
                ),
            )
        ]
    )
