"""ROS-free privileged task snapshot adapter for the scripted expert only."""

from __future__ import annotations

import math
from dataclasses import replace
from typing import Any
from uuid import UUID

from act_lab.domain import Action, Observation, PickPlaceTaskState, Pose


class ApproachRoute:
    """Opt-in inward/lower approach goals within the existing APPROACH phase."""

    def __init__(
        self, inward_x_m: float | None = None, transition_z_m: float | None = None
    ) -> None:
        if (inward_x_m is None) != (transition_z_m is None):
            raise ValueError("approach route requires both coordinates")
        if inward_x_m is not None and transition_z_m is not None:
            if not all(math.isfinite(v) for v in (inward_x_m, transition_z_m)):
                raise ValueError("approach route coordinates must be finite")
        self.inward_x_m = inward_x_m
        self.transition_z_m = transition_z_m
        self.stage = "inactive"
        self.waypoint: Pose | None = None

    def apply(self, phase: str, action: Action, observation: Observation) -> Action:
        if (
            phase != "approach_cube"
            or not action.enabled
            or self.inward_x_m is None
            or self.transition_z_m is None
        ):
            self.stage = "inactive"
            self.waypoint = None
            return action
        measured = observation.robot.end_effector_pose
        if self.stage == "inactive":
            self.stage = "inward"
            self.waypoint = replace(
                action.target_pose,
                position_xyz_m=(
                    self.inward_x_m,
                    measured.position_xyz_m[1],
                    measured.position_xyz_m[2],
                ),
            )
        if self.stage == "inward":
            assert self.waypoint is not None
            if (
                math.dist(measured.position_xyz_m, self.waypoint.position_xyz_m)
                <= 0.004
            ):
                self.stage = "lower"
                self.waypoint = replace(
                    self.waypoint,
                    position_xyz_m=(
                        self.inward_x_m,
                        self.waypoint.position_xyz_m[1],
                        min(self.waypoint.position_xyz_m[2], self.transition_z_m),
                    ),
                )
        if self.stage == "lower":
            assert self.waypoint is not None
            if (
                math.dist(measured.position_xyz_m, self.waypoint.position_xyz_m)
                <= 0.004
            ):
                self.stage = "final"
        if self.stage == "final":
            return action
        assert self.waypoint is not None
        return replace(action, target_pose=self.waypoint)


def task_state(snapshot: dict[str, Any], timestamp_ns: int) -> PickPlaceTaskState:
    """Require task geometry and observation to come from one owner snapshot."""
    UUID(snapshot["episode"])
    if type(snapshot["task_timestamp_ns"]) is not int or (
        snapshot["task_timestamp_ns"] != timestamp_ns
    ):
        raise ValueError("task and measured state timestamps disagree")
    fields = snapshot["task"]
    for key in ("success", "terminal"):
        if type(fields[key]) is not bool:
            raise ValueError("task flags must be boolean")
    if fields["success"] and not fields["terminal"]:
        raise ValueError("successful task must be terminal")
    if fields["reason"] is not None and not isinstance(fields["reason"], str):
        raise ValueError("task reason must be text")

    def pose(name: str) -> Pose:
        value = fields[name]
        position = tuple(value["position_xyz_m"])
        quaternion = tuple(value["quaternion_wxyz"])
        if value["frame_id"] != "world" or len(position) != 3 or len(quaternion) != 4:
            raise ValueError("invalid privileged task pose")
        if not all(math.isfinite(v) for v in (*position, *quaternion)):
            raise ValueError("nonfinite privileged task pose")
        return Pose("world", position, quaternion)

    return PickPlaceTaskState(
        pose("cube_pose"),
        pose("desired_cube_pose"),
        fields["success"],
        fields["terminal"],
        fields["reason"],
    )


class ExpertTaskSource:
    """Coherent task-only port; it cannot command or step the robot."""

    def __init__(self, require_release: bool = False) -> None:
        self.require_release = require_release
        self.released = False

    def update(self, snapshot: dict[str, Any], observation: Observation) -> None:
        self.update_task(
            task_state(snapshot, observation.timestamp_ns),
            observation,
            snapshot.get("cube_gripper_contacts"),
        )

    def update_task(
        self,
        value: PickPlaceTaskState,
        observation: Observation,
        contacts: int | None,
    ) -> None:
        self.value = value
        self.released = (
            observation.robot.gripper_position >= 0.95
            and type(contacts) is int
            and contacts == 0
        )

    def task_state(self) -> PickPlaceTaskState:
        if self.require_release and self.value.success and not self.released:
            # The generic task can settle while still pinched. Keep the existing
            # expert opening; qualification requires actual release as well.
            return replace(self.value, success=False, terminal=False, reason=None)
        return self.value

    def owner_task_state(self) -> PickPlaceTaskState:
        return self.value


class OpeningPoseHold:
    """Freeze the measured release pose while opening, retaining source identity."""

    def __init__(self) -> None:
        self.pose: Pose | None = None

    def apply(self, phase: str, action: Action, observation: Observation) -> Action:
        if phase != "open_dwell" or not action.enabled:
            self.pose = None
            return action
        if self.pose is None:
            self.pose = observation.robot.end_effector_pose
        return replace(action, target_pose=self.pose)


class GripperRamp:
    """Ramp explicit tool intent using actual source time."""

    def __init__(self, observation: Observation, rate_s: float = 0.1) -> None:
        if not math.isfinite(rate_s) or rate_s <= 0:
            raise ValueError("positive finite gripper rate required")
        self.rate = rate_s
        self.position = observation.robot.gripper_position
        self.timestamp = observation.timestamp_ns

    def apply(self, action: Action) -> Action:
        if action.timestamp_ns < self.timestamp:
            raise ValueError("gripper source clock reversed; create a new episode")
        elapsed = (action.timestamp_ns - self.timestamp) / 1e9
        self.timestamp = action.timestamp_ns
        if action.enabled:
            delta = self.rate * elapsed
            self.position += min(
                max(action.gripper_position - self.position, -delta), delta
            )
            return replace(action, gripper_position=self.position)
        return action


def loaded_transit_action(
    action: Action, observation: Observation, lookahead_m: float
) -> Action:
    """Bound horizontal expert lookahead while retaining its vertical target.

    Short measured-relative impedance targets need vertical authority under load.
    A far diagonal source can consume the entire allowed displacement in XY.
    This source strategy grants no motion authority and still passes shared safety.
    """
    if not math.isfinite(lookahead_m) or lookahead_m <= 0:
        raise ValueError("positive finite transit lookahead required")
    actual = observation.robot.end_effector_pose.position_xyz_m
    target = action.target_pose.position_xyz_m
    distance = math.hypot(target[0] - actual[0], target[1] - actual[1])
    scale = min(1.0, lookahead_m / distance) if distance else 1.0
    return replace(
        action,
        target_pose=replace(
            action.target_pose,
            position_xyz_m=(
                actual[0] + scale * (target[0] - actual[0]),
                actual[1] + scale * (target[1] - actual[1]),
                target[2],
            ),
        ),
    )


def descent_action(
    action: Action, observation: Observation, lookahead_m: float
) -> Action:
    """Limit downward source lookahead for contact approach and placement."""
    if not math.isfinite(lookahead_m) or lookahead_m <= 0:
        raise ValueError("positive finite descent lookahead required")
    if (
        action.target_pose.position_xyz_m[2]
        >= observation.robot.end_effector_pose.position_xyz_m[2]
    ):
        return action
    # Prioritize lowering before precise horizontal alignment near outer reach.
    horizontal = loaded_transit_action(action, observation, lookahead_m / 2)
    x, y, z = horizontal.target_pose.position_xyz_m
    return replace(
        action,
        target_pose=replace(
            action.target_pose,
            position_xyz_m=(
                x,
                y,
                max(
                    z,
                    observation.robot.end_effector_pose.position_xyz_m[2] - lookahead_m,
                ),
            ),
        ),
    )
