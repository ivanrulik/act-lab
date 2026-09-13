"""Application-owned Cartesian safety policy and command reporting."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

from act_lab.domain import (
    Action,
    CommandOutcome,
    CommandReport,
    Observation,
    Pose,
    RobotState,
)

JointVector = tuple[float, float, float, float, float, float]
Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


class SafetyLimits(Protocol):
    @property
    def pose_response_time_s(self) -> float: ...

    @property
    def watchdog_timeout_ns(self) -> int: ...

    @property
    def workspace_x_m(self) -> tuple[float, float]: ...

    @property
    def workspace_y_m(self) -> tuple[float, float]: ...

    @property
    def workspace_z_m(self) -> tuple[float, float]: ...

    @property
    def max_translation_velocity_m_s(self) -> float: ...

    @property
    def max_command_translation_velocity_m_s(self) -> float: ...

    @property
    def max_translation_acceleration_m_s2(self) -> float: ...

    @property
    def max_orientation_velocity_rad_s(self) -> float: ...

    @property
    def max_orientation_acceleration_rad_s2(self) -> float: ...

    @property
    def max_joint_velocity_rad_s(self) -> float: ...

    @property
    def max_joint_acceleration_rad_s2(self) -> float: ...

    @property
    def joint_bound_margin_rad(self) -> float: ...

    @property
    def max_gripper_velocity_s(self) -> float: ...


class CartesianDriver(Protocol):
    """Framework-neutral seam implemented by a simulator or hardware adapter."""

    @property
    def control_period_s(self) -> float: ...

    @property
    def joint_position_bounds_rad(
        self,
    ) -> tuple[tuple[float, float], ...]: ...

    def reset(self, seed: int) -> Observation: ...

    def observe(self) -> Observation: ...

    def solve_ik(self, target_pose: Pose) -> JointVector | None: ...

    def collision_free(self, joints: JointVector, gripper: float) -> bool: ...

    def step(
        self, joints: JointVector, joint_velocity: JointVector, gripper: float
    ) -> RobotState: ...


@dataclass(slots=True)
class _MotionHistory:
    pose: Pose
    gripper: float
    joint_positions: JointVector
    linear_velocity: Vector3 = (0.0, 0.0, 0.0)
    angular_velocity: Vector3 = (0.0, 0.0, 0.0)
    joint_velocity: JointVector = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class SafeCartesianRobot:
    """A domain Robot that holds safely and reports expected interventions."""

    def __init__(self, driver: CartesianDriver, limits: SafetyLimits) -> None:
        self._driver = driver
        self._limits = limits
        self._history: _MotionHistory | None = None
        self._last_report: CommandReport | None = None

    @property
    def last_command_report(self) -> CommandReport | None:
        return self._last_report

    @property
    def last_report(self) -> CommandReport | None:
        return self._last_report

    def reset(self, seed: int) -> Observation:
        observation = self._driver.reset(seed)
        state = observation.robot
        self._history = _MotionHistory(
            pose=state.end_effector_pose,
            gripper=state.gripper_position,
            joint_positions=_joint_vector(state.joint_positions_rad),
        )
        self._last_report = None
        return observation

    def observe(self) -> Observation:
        return self._driver.observe()

    def command(self, action: Action) -> RobotState:
        state = self._driver.observe().robot
        if self._history is None:
            self._history = _MotionHistory(
                state.end_effector_pose,
                state.gripper_position,
                _joint_vector(state.joint_positions_rad),
            )
        else:
            self._sync_measured_state(state)

        timestamp_error = self._timestamp_error(action, state.timestamp_ns)
        if timestamp_error is not None:
            outcome, detail = timestamp_error
            return self._hold(action, state, outcome, detail)
        if not action.enabled:
            return self._hold(
                action, state, CommandOutcome.DISABLED, "command is disabled"
            )

        invalid_detail = self._invalid_detail(action)
        if invalid_detail is not None:
            return self._hold(action, state, CommandOutcome.INVALID, invalid_detail)

        target_pose, workspace_limited = self._clamp_workspace(action.target_pose)
        target_pose, pose_limited = self._limit_pose(target_pose)
        gripper, gripper_limited = self._limit_gripper(action.gripper_position)
        candidate = self._driver.solve_ik(target_pose)
        if candidate is None:
            return self._hold(
                action,
                state,
                CommandOutcome.IK_FAILURE,
                "damped-least-squares IK did not converge",
            )

        joints, joints_limited, joint_velocity = self._limit_joints(candidate)
        if not self._driver.collision_free(joints, gripper):
            return self._hold(
                action,
                state,
                CommandOutcome.COLLISION_STOP,
                "candidate creates a prohibited contact",
            )

        executed = Action(
            timestamp_ns=action.timestamp_ns,
            target_pose=target_pose,
            gripper_position=gripper,
            enabled=True,
        )
        resulting_state = self._driver.step(joints, joint_velocity, gripper)
        assert self._history is not None
        commanded_linear_velocity = self._history.linear_velocity
        commanded_angular_velocity = self._history.angular_velocity
        measured_speed = _translation_speed(state, resulting_state)
        self._sync_measured_state(resulting_state)
        self._history.linear_velocity = commanded_linear_velocity
        self._history.angular_velocity = commanded_angular_velocity
        self._history.gripper = gripper
        self._history.joint_velocity = joint_velocity
        limited = workspace_limited or pose_limited or gripper_limited or joints_limited
        outcome = CommandOutcome.LIMITED if limited else CommandOutcome.APPLIED
        detail = "command limited by safety envelope" if limited else "command applied"
        self._last_report = CommandReport(
            outcome,
            action,
            executed,
            resulting_state,
            detail,
            commanded_cartesian_speed_m_s=_norm(commanded_linear_velocity),
            measured_cartesian_speed_m_s=measured_speed,
            tracking_error_m=math.dist(
                target_pose.position_xyz_m,
                resulting_state.end_effector_pose.position_xyz_m,
            ),
            command_age_ms=(state.timestamp_ns - action.timestamp_ns) / 1_000_000.0,
        )
        return resulting_state

    def _sync_measured_state(self, state: RobotState) -> None:
        """Refresh position feedback without changing commanded rate history."""
        assert self._history is not None
        self._history.pose = state.end_effector_pose
        self._history.joint_positions = _joint_vector(state.joint_positions_rad)

    def _timestamp_error(
        self, action: Action, now_ns: int
    ) -> tuple[CommandOutcome, str] | None:
        if isinstance(action.timestamp_ns, bool) or not isinstance(
            action.timestamp_ns, int
        ):
            return CommandOutcome.INVALID, "timestamp must be an integer"
        if action.timestamp_ns > now_ns:
            return CommandOutcome.INVALID, "timestamp is in the future"
        if now_ns - action.timestamp_ns >= self._limits.watchdog_timeout_ns:
            return CommandOutcome.STALE, "command exceeded the watchdog deadline"
        return None

    @staticmethod
    def _invalid_detail(action: Action) -> str | None:
        pose = action.target_pose
        numeric = (*pose.position_xyz_m, *pose.quaternion_wxyz, action.gripper_position)
        if not all(math.isfinite(value) for value in numeric):
            return "command contains a non-finite value"
        if pose.frame_id != "world":
            return "target pose frame must be 'world'"
        quaternion_norm = math.sqrt(
            sum(value * value for value in pose.quaternion_wxyz)
        )
        if not math.isclose(quaternion_norm, 1.0, abs_tol=1e-3):
            return "target quaternion must have unit norm"
        if not 0.0 <= action.gripper_position <= 1.0:
            return "gripper target must be in [0, 1]"
        return None

    def _clamp_workspace(self, pose: Pose) -> tuple[Pose, bool]:
        bounds = (
            self._limits.workspace_x_m,
            self._limits.workspace_y_m,
            self._limits.workspace_z_m,
        )
        position = tuple(
            min(max(value, axis[0]), axis[1])
            for value, axis in zip(pose.position_xyz_m, bounds, strict=True)
        )
        clamped = (position[0], position[1], position[2])
        return (
            Pose(pose.frame_id, clamped, pose.quaternion_wxyz),
            clamped != pose.position_xyz_m,
        )

    def _limit_pose(self, target: Pose) -> tuple[Pose, bool]:
        assert self._history is not None
        dt = self._driver.control_period_s
        delta = _subtract(target.position_xyz_m, self._history.pose.position_xyz_m)
        # Correct measured error over a response horizon, not in one cycle:
        # a one-cycle correction excites the lagging velocity/position servos.
        response_time = max(dt, self._limits.pose_response_time_s)
        desired_linear = _scale(delta, 1.0 / response_time)
        # Command and measured-motion ceilings are explicit: the application
        # shapes intent while the adapter independently guards observed motion.
        linear = _limit_vector(
            desired_linear,
            self._limits.max_command_translation_velocity_m_s,
        )
        linear = _limit_delta(
            linear,
            self._history.linear_velocity,
            self._limits.max_translation_acceleration_m_s2 * dt,
        )
        position = _add(self._history.pose.position_xyz_m, _scale(linear, dt))

        rotation_error = _quaternion_error_vector(
            target.quaternion_wxyz, self._history.pose.quaternion_wxyz
        )
        desired_angular = _scale(rotation_error, 1.0 / response_time)
        angular = _limit_vector(
            desired_angular, self._limits.max_orientation_velocity_rad_s
        )
        angular = _limit_delta(
            angular,
            self._history.angular_velocity,
            self._limits.max_orientation_acceleration_rad_s2 * dt,
        )
        quaternion = _apply_rotation(
            self._history.pose.quaternion_wxyz, _scale(angular, dt)
        )
        self._history.linear_velocity = linear
        self._history.angular_velocity = angular
        limited = not _vector_close(position, target.position_xyz_m) or not _quat_close(
            quaternion, target.quaternion_wxyz
        )
        return Pose("world", position, quaternion), limited

    def _limit_gripper(self, target: float) -> tuple[float, bool]:
        assert self._history is not None
        maximum_delta = (
            self._limits.max_gripper_velocity_s * self._driver.control_period_s
        )
        delta = min(max(target - self._history.gripper, -maximum_delta), maximum_delta)
        result = self._history.gripper + delta
        return result, not math.isclose(result, target, abs_tol=1e-12)

    def _limit_joints(
        self, target: JointVector
    ) -> tuple[JointVector, bool, JointVector]:
        assert self._history is not None
        dt = self._driver.control_period_s
        values: list[float] = []
        velocities: list[float] = []
        limited = False
        for target_value, current, old_velocity, bounds in zip(
            target,
            self._history.joint_positions,
            self._history.joint_velocity,
            self._driver.joint_position_bounds_rad,
            strict=True,
        ):
            lower = bounds[0] + self._limits.joint_bound_margin_rad
            upper = bounds[1] - self._limits.joint_bound_margin_rad
            desired_velocity = (target_value - current) / dt
            velocity = min(
                max(
                    desired_velocity,
                    old_velocity
                    - self._limits.max_joint_acceleration_rad_s2 * dt,
                ),
                old_velocity + self._limits.max_joint_acceleration_rad_s2 * dt,
            )
            velocity = min(
                max(velocity, -self._limits.max_joint_velocity_rad_s),
                self._limits.max_joint_velocity_rad_s,
            )
            value = current + velocity * dt
            bounded = min(max(value, lower), upper)
            if not math.isclose(bounded, target_value, abs_tol=1e-12):
                limited = True
            values.append(bounded)
            velocities.append((bounded - current) / dt)
        return _joint_vector(values), limited, _joint_vector(velocities)

    def _hold(
        self,
        action: Action,
        state: RobotState,
        outcome: CommandOutcome,
        detail: str,
    ) -> RobotState:
        assert self._history is not None
        joints = _joint_vector(state.joint_positions_rad)
        # Preserve the last accepted, rate-limited aperture to maintain grip
        # force under contact deflection. Never consume the rejected payload.
        gripper = self._history.gripper
        resulting_state = self._driver.step(
            joints, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), gripper
        )
        measured_speed = _translation_speed(state, resulting_state)
        self._history = _MotionHistory(
            resulting_state.end_effector_pose,
            gripper,
            _joint_vector(resulting_state.joint_positions_rad),
        )
        self._last_report = CommandReport(
            outcome,
            action,
            None,
            resulting_state,
            detail,
            measured_cartesian_speed_m_s=measured_speed,
            tracking_error_m=math.dist(
                state.end_effector_pose.position_xyz_m,
                resulting_state.end_effector_pose.position_xyz_m,
            ),
            command_age_ms=max(
                0.0, (state.timestamp_ns - action.timestamp_ns) / 1_000_000.0
            ),
        )
        return resulting_state


def _joint_vector(values: tuple[float, ...] | list[float]) -> JointVector:
    if len(values) != 6:
        raise ValueError("UR5e joint vector must contain six values")
    return (values[0], values[1], values[2], values[3], values[4], values[5])


def _add(left: Vector3, right: Vector3) -> Vector3:
    return left[0] + right[0], left[1] + right[1], left[2] + right[2]


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return left[0] - right[0], left[1] - right[1], left[2] - right[2]


def _scale(value: Vector3, factor: float) -> Vector3:
    return value[0] * factor, value[1] * factor, value[2] * factor


def _norm(value: Vector3) -> float:
    return math.sqrt(sum(component * component for component in value))


def _translation_speed(before: RobotState, after: RobotState) -> float:
    dt_s = (after.timestamp_ns - before.timestamp_ns) / 1_000_000_000.0
    if dt_s <= 0.0:
        return 0.0
    return math.dist(
        before.end_effector_pose.position_xyz_m,
        after.end_effector_pose.position_xyz_m,
    ) / dt_s


def _limit_vector(value: Vector3, maximum: float) -> Vector3:
    magnitude = _norm(value)
    return value if magnitude <= maximum else _scale(value, maximum / magnitude)


def _limit_delta(value: Vector3, previous: Vector3, maximum: float) -> Vector3:
    delta = _subtract(value, previous)
    return _add(previous, _limit_vector(delta, maximum))


def _vector_close(left: Vector3, right: Vector3) -> bool:
    return all(
        math.isclose(a, b, abs_tol=1e-12)
        for a, b in zip(left, right, strict=True)
    )


def _quat_close(left: Quaternion, right: Quaternion) -> bool:
    dot = abs(sum(a * b for a, b in zip(left, right, strict=True)))
    return math.isclose(dot, 1.0, abs_tol=1e-10)


def _quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def _quaternion_error_vector(target: Quaternion, current: Quaternion) -> Vector3:
    conjugate = (current[0], -current[1], -current[2], -current[3])
    error = _quaternion_multiply(target, conjugate)
    if error[0] < 0.0:
        error = (-error[0], -error[1], -error[2], -error[3])
    vector = (error[1], error[2], error[3])
    magnitude = _norm(vector)
    if magnitude < 1e-12:
        return 0.0, 0.0, 0.0
    angle = 2.0 * math.atan2(magnitude, max(error[0], 0.0))
    return _scale(vector, angle / magnitude)


def _apply_rotation(quaternion: Quaternion, rotation: Vector3) -> Quaternion:
    angle = _norm(rotation)
    if angle < 1e-12:
        return quaternion
    half = angle / 2.0
    scale = math.sin(half) / angle
    delta = (
        math.cos(half),
        rotation[0] * scale,
        rotation[1] * scale,
        rotation[2] * scale,
    )
    result = _quaternion_multiply(delta, quaternion)
    norm = math.sqrt(sum(value * value for value in result))
    return tuple(value / norm for value in result)  # type: ignore[return-value]
