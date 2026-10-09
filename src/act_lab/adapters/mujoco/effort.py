"""Optional effort actuation with exclusive dynamic servo hold ownership.

This module is simulator-specific. It supplies no ROS dependencies and does not
claim a physical stop mechanism. Only the physics owner may call it.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import mujoco  # type: ignore[import-untyped]
import numpy as np
from numpy.typing import NDArray

from act_lab.adapters.mujoco.cartesian_driver import MujocoCartesianDriver
from act_lab.adapters.mujoco.config import SimulationConfig
from act_lab.adapters.mujoco.environment import MujocoUR5eEnvironment
from act_lab.domain import Observation, RobotState


@dataclass(frozen=True, slots=True)
class EffortSample:
    state: RobotState
    mode: str
    reason: str
    task_effort_nm: tuple[float, ...]
    servo_effort_nm: tuple[float, ...]
    total_effort_nm: tuple[float, ...]
    accepted_gripper: float
    cartesian_speed_m_s: float
    cartesian_acceleration_m_s2: float
    angular_speed_rad_s: float
    angular_acceleration_rad_s2: float
    maximum_joint_acceleration_rad_s2: float


class MujocoEffortPlant(MujocoCartesianDriver):
    """Reuse scene and scratch feasibility; change only runtime actuator parameters."""

    def __init__(self, config: SimulationConfig) -> None:
        super().__init__(MujocoUR5eEnvironment(config), config)
        self._arm_ids = self.environment._arm_actuator_ids  # noqa: SLF001
        self._finger_id = self.environment._gripper_actuator_id  # noqa: SLF001
        self._servo_ids = (*self._arm_ids, *self._velocity_actuator_ids)
        self._gains = self._model.actuator_gainprm.copy()
        self._bias = self._model.actuator_biasprm.copy()
        self._ctrl_ranges = self._model.actuator_ctrlrange.copy()
        self._force_ranges = self._model.actuator_forcerange.copy()
        self._force_limited = self._model.actuator_forcelimited.copy()
        self._task_effort = np.zeros(6, dtype=np.float64)
        self.mode = "STARTUP_HOLD"
        self.reason = "startup"
        self.accepted_gripper = 0.0
        self._hold_joints: tuple[float, ...] = config.home_joint_positions_rad
        for actuator in self._arm_ids:
            if not np.array_equal(
                self._model.actuator_gear[actuator], [1, 0, 0, 0, 0, 0]
            ):
                raise ValueError("effort actuators require unit joint gear")

    def reset(self, seed: int) -> Observation:
        # Restore original servos before ordinary scene reset applies joint controls.
        self._restore_servos()
        observation = super().reset(seed)
        self.mode = "STARTUP_HOLD"
        self.reason = "startup"
        self._hold_joints = observation.robot.joint_positions_rad
        self.accepted_gripper = observation.robot.gripper_position
        self._task_effort.fill(0.0)
        self.hold("startup")
        self.mode = "STARTUP_HOLD"
        return observation

    def _restore_servos(self) -> None:
        self._model.actuator_gainprm[:] = self._gains
        self._model.actuator_biasprm[:] = self._bias
        self._model.actuator_ctrlrange[:] = self._ctrl_ranges
        self._model.actuator_forcerange[:] = self._force_ranges
        self._model.actuator_forcelimited[:] = self._force_limited

    def hold(self, reason: str, *, shutdown: bool = False) -> None:
        if self.mode == "ENABLED":
            self._hold_joints = self.observe().robot.joint_positions_rad
        self._restore_servos()
        # Use position gains with explicit hold damping: 50 Nm s/rad for
        # proximal joints, 10 for wrists. One bounded PD actuator owns each
        # joint; the second servo cannot add unbounded braking effort.
        for primary, velocity in zip(
            self._arm_ids, self._velocity_actuator_ids, strict=True
        ):
            self._model.actuator_biasprm[primary, 2] = (
                -10.0 if primary in self._arm_ids[3:] else -50.0
            )
            self._model.actuator_gainprm[velocity] = 0.0
            self._model.actuator_biasprm[velocity] = 0.0
            self._model.actuator_forcelimited[primary] = True
        self._task_effort.fill(0.0)
        self.mode = "SHUTDOWN_HOLD" if shutdown else "FAULT_HOLD"
        self.reason = reason

    def enable(self, gripper: float) -> None:
        if not math.isfinite(gripper) or not 0.0 <= gripper <= 1.0:
            self.hold("invalid_gripper")
            raise ValueError("invalid accepted gripper")
        for actuator in self._servo_ids:
            self._model.actuator_gainprm[actuator] = 0.0
            self._model.actuator_biasprm[actuator] = 0.0
            self._data.ctrl[actuator] = 0.0
        for actuator in self._arm_ids:
            self._model.actuator_forcelimited[actuator] = True
            self._model.actuator_gainprm[actuator, 0] = 1.0
            self._model.actuator_ctrlrange[actuator] = (-5.0, 5.0)
            self._model.actuator_forcerange[actuator] = (-5.0, 5.0)
        self.mode = "ENABLED"
        self.reason = "authorized"
        self.accepted_gripper = gripper

    def tick(self, effort_nm: Sequence[float] | None = None) -> EffortSample:
        """Advance one 2 ms tick; hold continues real dynamics and gripper contact."""
        if self.mode == "ENABLED":
            if (
                effort_nm is None
                or len(effort_nm) != 6
                or not all(math.isfinite(v) for v in effort_nm)
            ):
                self.hold("invalid_effort")
            else:
                raw = np.asarray(effort_nm, dtype=np.float64)
                desired = np.clip(raw, -5.0, 5.0)
                self._task_effort += np.clip(desired - self._task_effort, -0.2, 0.2)
                for actuator, torque in zip(
                    self._arm_ids, self._task_effort, strict=True
                ):
                    self._data.ctrl[actuator] = torque
        if self.mode != "ENABLED":
            for actuator, position in zip(
                self._arm_ids, self._hold_joints, strict=True
            ):
                self._data.ctrl[actuator] = position
            for actuator in self._velocity_actuator_ids:
                self._data.ctrl[actuator] = 0.0
        self._data.ctrl[self._finger_id] = self.accepted_gripper * 0.025
        mujoco.mj_copyData(self._step_backup, self._model, self._data)
        before_velocity = self._data.qvel[list(self._dof_addresses)].copy()
        before_twist = self._twist()
        mujoco.mj_step(self._model, self._data)
        mujoco.mj_forward(self._model, self._data)
        if self.mode == "ENABLED":
            reason = self._violation(before_velocity, before_twist)
            if reason:
                # Predictive simulator guard: discard the unsafe candidate and
                # integrate the same tick under dynamic hold, preserving velocity.
                mujoco.mj_copyData(self._data, self._model, self._step_backup)
                self.hold(reason)
                for actuator, position in zip(
                    self._arm_ids, self._hold_joints, strict=True
                ):
                    self._data.ctrl[actuator] = position
                for actuator in self._velocity_actuator_ids:
                    self._data.ctrl[actuator] = 0.0
                mujoco.mj_step(self._model, self._data)
                mujoco.mj_forward(self._model, self._data)
        self.environment._physics_ticks += 1  # noqa: SLF001
        if self.environment.physics_ticks % self._config.substeps == 0:
            self.environment._environment_steps += 1  # noqa: SLF001
            self.environment._update_settling_counter()  # noqa: SLF001
        # State and feasibility must describe the integrated configuration.
        mujoco.mj_forward(self._model, self._data)
        state = self.observe().robot
        if not all(
            math.isfinite(v)
            for v in (*state.joint_positions_rad, *state.joint_velocities_rad_s)
        ):
            self.hold("nonfinite_state")
            raise RuntimeError("nonfinite effort plant state")
        primary = tuple(float(self._data.actuator_force[a]) for a in self._arm_ids)
        velocity = tuple(
            float(self._data.actuator_force[a]) for a in self._velocity_actuator_ids
        )
        servo = (
            tuple(a + b for a, b in zip(primary, velocity, strict=True))
            if self.mode != "ENABLED"
            else (0.0,) * 6
        )
        task = primary if self.mode == "ENABLED" else (0.0,) * 6
        return EffortSample(
            state,
            self.mode,
            self.reason,
            task,
            servo,
            tuple(a + b for a, b in zip(task, servo, strict=True)),
            self.accepted_gripper,
            float(np.linalg.norm(self._twist()[:3])),
            float(
                np.linalg.norm(self._twist()[:3] - before_twist[:3])
                / self._model.opt.timestep
            ),
            float(np.linalg.norm(self._twist()[3:])),
            float(
                np.linalg.norm(self._twist()[3:] - before_twist[3:])
                / self._model.opt.timestep
            ),
            float(
                np.max(
                    np.abs(np.asarray(state.joint_velocities_rad_s) - before_velocity)
                )
                / self._model.opt.timestep
            ),
        )

    def _twist(self) -> NDArray[np.float64]:
        mujoco.mj_jacSite(
            self._model,
            self._data,
            self._jacobian_position,
            self._jacobian_rotation,
            self._site_id,
        )
        return np.concatenate(
            (
                self._jacobian_position @ self._data.qvel,
                self._jacobian_rotation @ self._data.qvel,
            )
        )

    def _violation(
        self, before_velocity: NDArray[np.float64], before_twist: NDArray[np.float64]
    ) -> str | None:
        state = self.observe().robot
        velocity = np.asarray(state.joint_velocities_rad_s)
        twist = self._twist()
        limits = self.limits
        dt = float(self._model.opt.timestep)
        if (
            not np.isfinite(self._data.qpos).all()
            or not np.isfinite(self._data.qvel).all()
        ):
            return "nonfinite_state"
        for q, (lower, upper) in zip(
            state.joint_positions_rad, self.joint_position_bounds_rad, strict=True
        ):
            if (
                not lower + limits.joint_bound_margin_rad
                <= q
                <= upper - limits.joint_bound_margin_rad
            ):
                return "joint_boundary"
        if not self._resulting_state_is_safe(state):
            return "workspace_or_contact"
        for value, bound, reason in (
            (
                np.max(np.abs(velocity)),
                limits.max_joint_velocity_rad_s,
                "joint_velocity",
            ),
            (
                np.max(np.abs(velocity - before_velocity)) / dt,
                limits.max_joint_acceleration_rad_s2,
                "joint_acceleration",
            ),
            (
                np.linalg.norm(twist[:3]),
                limits.max_translation_velocity_m_s,
                "cartesian_velocity",
            ),
            (
                np.linalg.norm(twist[3:]),
                limits.max_orientation_velocity_rad_s,
                "angular_velocity",
            ),
            (
                np.linalg.norm(twist[:3] - before_twist[:3]) / dt,
                limits.max_translation_acceleration_m_s2,
                "cartesian_acceleration",
            ),
            (
                np.linalg.norm(twist[3:] - before_twist[3:]) / dt,
                limits.max_orientation_acceleration_rad_s2,
                "angular_acceleration",
            ),
        ):
            if value > bound + 1e-9:
                return reason
        return None
