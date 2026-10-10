"""MuJoCo damped-least-squares IK and contact feasibility adapter."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter_ns
from types import TracebackType

import mujoco  # type: ignore[import-untyped]
import numpy as np

from act_lab.adapters.mujoco.config import CartesianControlConfig, SimulationConfig
from act_lab.adapters.mujoco.environment import ActuatorTargets, MujocoUR5eEnvironment
from act_lab.application.cartesian_control import JointVector
from act_lab.domain import Observation, Pose, RobotState


@dataclass(frozen=True, slots=True)
class IKDiagnostics:
    """Adapter-local numerical results; wall time never controls the solve."""

    iterations: int
    converged: bool
    position_residual_m: float
    orientation_residual_rad: float
    duration_ns: int | None


class MujocoCartesianDriver:
    """IK and predicted-contact checks around one deterministic environment."""

    def __init__(
        self,
        environment: MujocoUR5eEnvironment,
        config: SimulationConfig,
        *,
        profile: bool = False,
    ) -> None:
        self._environment = environment
        self._config = config
        self._model = environment._model  # noqa: SLF001
        self._data = environment._data  # noqa: SLF001
        self._scratch = mujoco.MjData(self._model)
        self._step_backup = mujoco.MjData(self._model)
        self._jacobian_position = np.zeros((3, self._model.nv), dtype=np.float64)
        self._jacobian_rotation = np.zeros((3, self._model.nv), dtype=np.float64)
        self._jacobian = np.empty((6, 6), dtype=np.float64)
        self._ik_error = np.empty(6, dtype=np.float64)
        self._ik_quaternion = np.empty(4, dtype=np.float64)
        self._damped_system = np.empty((6, 6), dtype=np.float64)
        self._damped_factor = np.empty((6, 6), dtype=np.float64)
        self._damped_solution = np.empty(6, dtype=np.float64)
        self._profile = profile
        self.profile_totals_ns = {"ik": 0, "collision": 0, "step": 0}
        self.last_ik_diagnostics: IKDiagnostics | None = None
        self._joint_ids = environment._arm_joint_ids  # noqa: SLF001
        self._velocity_actuator_ids = tuple(
            self._named_id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_velocity")
            for name in (
                "shoulder_pan",
                "shoulder_lift",
                "elbow",
                "wrist_1",
                "wrist_2",
                "wrist_3",
            )
        )
        self._qpos_addresses = tuple(
            int(self._model.jnt_qposadr[joint_id]) for joint_id in self._joint_ids
        )
        self._dof_addresses = tuple(
            int(self._model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids
        )
        self._qpos_indices = np.asarray(self._qpos_addresses)
        self._joint_ranges = self._model.jnt_range[list(self._joint_ids)].copy()
        self._site_id = environment._end_effector_site_id  # noqa: SLF001
        self._robot_body_ids = self._descendant_body_ids("base")
        self._cube_body_id = self._body_id("cube")
        self._allowed_cube_geoms = {
            self._geom_id(name) for name in environment.gripper.pad_geoms
        }
        for lower, upper in self.joint_position_bounds_rad:
            if lower + config.control.joint_bound_margin_rad >= (
                upper - config.control.joint_bound_margin_rad
            ):
                raise ValueError("joint_bound_margin_rad leaves no valid joint range")

    @classmethod
    def from_config_file(cls, path: Path) -> MujocoCartesianDriver:
        config = SimulationConfig.load(path)
        return cls(MujocoUR5eEnvironment(config), config)

    @property
    def limits(self) -> CartesianControlConfig:
        return self._config.control

    @property
    def simulation_config(self) -> SimulationConfig:
        return self._config

    @property
    def control_period_s(self) -> float:
        return 1.0 / self._config.environment_hz

    @property
    def joint_position_bounds_rad(self) -> tuple[tuple[float, float], ...]:
        return tuple(
            (
                float(self._model.jnt_range[joint_id, 0]),
                float(self._model.jnt_range[joint_id, 1]),
            )
            for joint_id in self._joint_ids
        )

    @property
    def environment(self) -> MujocoUR5eEnvironment:
        return self._environment

    def reset(self, seed: int) -> Observation:
        observation = self._environment.reset(seed)
        self._copy_state_to_scratch()
        self.last_ik_diagnostics = None
        self.clear_profile()
        return observation

    def clear_profile(self) -> None:
        for key in self.profile_totals_ns:
            self.profile_totals_ns[key] = 0

    def observe(self) -> Observation:
        return self._environment.observe()

    def solve_ik(self, target_pose: Pose) -> JointVector | None:
        started = perf_counter_ns() if self._profile else None
        self._copy_state_to_scratch()
        target_position = np.asarray(target_pose.position_xyz_m, dtype=np.float64)
        target_quaternion = np.asarray(target_pose.quaternion_wxyz, dtype=np.float64)
        jacobian_position = self._jacobian_position
        jacobian_rotation = self._jacobian_rotation
        jacobian = self._jacobian
        control = self._config.control
        regularizer = (control.dls_damping**2) * np.eye(6)
        lower_bounds = self._joint_ranges[:, 0] + control.joint_bound_margin_rad
        upper_bounds = self._joint_ranges[:, 1] - control.joint_bound_margin_rad

        current_quaternion = self._ik_quaternion
        error = self._ik_error
        for iteration in range(control.ik_max_iterations + 1):
            current_position = self._scratch.site_xpos[self._site_id]
            mujoco.mju_mat2Quat(
                current_quaternion, self._scratch.site_xmat[self._site_id]
            )
            position_error = target_position - current_position
            rotation_error = _quaternion_error(
                target_quaternion,  # type: ignore[arg-type]
                current_quaternion,
            )
            position_residual = math.sqrt(float(position_error.dot(position_error)))
            orientation_residual = math.sqrt(float(rotation_error.dot(rotation_error)))
            if (
                position_residual <= control.ik_position_tolerance_m
                and orientation_residual <= control.ik_orientation_tolerance_rad
            ):
                values = tuple(
                    float(self._scratch.qpos[address])
                    for address in self._qpos_addresses
                )
                self._record_ik(
                    iteration, True, position_residual, orientation_residual, started
                )
                return _joint_vector(values)
            if iteration == control.ik_max_iterations:
                break

            mujoco.mj_jacSite(
                self._model,
                self._scratch,
                jacobian_position,
                jacobian_rotation,
                self._site_id,
            )
            jacobian[:3] = jacobian_position[:, self._dof_addresses]
            jacobian[3:] = jacobian_rotation[:, self._dof_addresses]
            error[:3] = position_error
            error[3:] = rotation_error
            system = self._damped_system
            np.matmul(jacobian, jacobian.T, out=system)
            system += regularizer
            delta = jacobian.T @ self._solve_damped_system(system, error)
            delta_norm = math.sqrt(float(delta.dot(delta)))
            if delta_norm > 0.25:
                delta *= 0.25 / delta_norm
            projected = self._scratch.qpos[self._qpos_indices] + delta
            np.maximum(projected, lower_bounds, out=projected)
            np.minimum(projected, upper_bounds, out=projected)
            self._scratch.qpos[self._qpos_indices] = projected
            self._update_ik_kinematics()
        self._record_ik(
            iteration, False, position_residual, orientation_residual, started
        )
        return None

    def _solve_damped_system(
        self,
        system: np.ndarray[tuple[int, ...], np.dtype[np.float64]],
        error: np.ndarray[tuple[int, ...], np.dtype[np.float64]],
    ) -> np.ndarray[tuple[int, ...], np.dtype[np.float64]]:
        # Damping makes J J^T + lambda^2 I positive definite. MuJoCo avoids
        # general-purpose NumPy/LAPACK dispatch for this small 6x6 system.
        factor = self._damped_factor
        np.copyto(factor, system)
        if mujoco.mju_cholFactor(factor, 0.0) != 6:
            return np.asarray(np.linalg.solve(system, error), dtype=np.float64)
        result = self._damped_solution
        mujoco.mju_cholSolve(result, factor, error)
        return result

    def _update_ik_kinematics(self) -> None:
        # These are the minimal stages required by mj_jacSite after changing qpos.
        # Full contact/dynamics evaluation remains in collision_free and step.
        mujoco.mj_kinematics(self._model, self._scratch)
        mujoco.mj_comPos(self._model, self._scratch)

    def _record_ik(
        self,
        iterations: int,
        converged: bool,
        position: float,
        orientation: float,
        started: int | None,
    ) -> None:
        duration = perf_counter_ns() - started if started is not None else None
        self.last_ik_diagnostics = IKDiagnostics(
            iterations, converged, position, orientation, duration
        )
        if duration is not None:
            self.profile_totals_ns["ik"] += duration

    def collision_free(self, joints: JointVector, gripper: float) -> bool:
        if not self._profile:
            return self._collision_free(joints, gripper)
        started = perf_counter_ns()
        try:
            return self._collision_free(joints, gripper)
        finally:
            self.profile_totals_ns["collision"] += perf_counter_ns() - started

    def _collision_free(self, joints: JointVector, gripper: float) -> bool:
        self._copy_state_to_scratch()
        for address, value in zip(self._qpos_addresses, joints, strict=True):
            self._scratch.qpos[address] = value
        if not self._environment.binding.adaptive_gripper:
            self._environment.gripper.set_kinematic(self._scratch, gripper)
        # Adaptive linkage pose comes from measured contact dynamics. Substituting
        # an unloaded closure for a blocked finger invents cube penetration.
        # The actual actuator step below predicts and validates its resulting
        # contacts, including motion of the gripper, before committing the step.
        mujoco.mj_forward(self._model, self._scratch)
        contacts = self._scratch.contact[: self._scratch.ncon]
        pad_cube_contact = any(
            self._is_pad_cube_contact(int(contact.geom1), int(contact.geom2))
            for contact in contacts
        )
        for contact in contacts:
            geom_a = int(contact.geom1)
            geom_b = int(contact.geom2)
            body_a = int(self._model.geom_bodyid[geom_a])
            body_b = int(self._model.geom_bodyid[geom_b])
            a_robot = body_a in self._robot_body_ids
            b_robot = body_b in self._robot_body_ids
            if not a_robot and not b_robot:
                continue
            if a_robot and b_robot:
                if (
                    self._environment.binding.adaptive_gripper
                    and geom_a in self._allowed_cube_geoms
                    and geom_b in self._allowed_cube_geoms
                ):
                    continue  # Opposing pads may touch during empty closure.
                return False
            robot_geom = geom_a if a_robot else geom_b
            other_body = body_b if a_robot else body_a
            if (
                other_body == self._cube_body_id
                and robot_geom in self._allowed_cube_geoms
            ):
                continue
            # The vendored wrist_2 capsule coarsely encloses the finger pads.
            # Ignore only its induced cube overlap during an actual pad grasp.
            if (
                other_body == self._cube_body_id
                and pad_cube_contact
                and not self._environment.binding.adaptive_gripper
                and self._model.body(body_a if a_robot else body_b).name
                == "wrist_2_link"
            ):
                continue
            return False
        return True

    def step(
        self, joints: JointVector, joint_velocity: JointVector, gripper: float
    ) -> RobotState:
        if not self._profile:
            return self._step(joints, joint_velocity, gripper)
        started = perf_counter_ns()
        try:
            return self._step(joints, joint_velocity, gripper)
        finally:
            # Includes resulting-state collision checks; collision time is also
            # reported separately and must not be added again to this total.
            self.profile_totals_ns["step"] += perf_counter_ns() - started

    def _step(
        self, joints: JointVector, joint_velocity: JointVector, gripper: float
    ) -> RobotState:
        start_position = self._data.site_xpos[self._site_id].copy()
        current_joints = tuple(
            float(self._data.qpos[address]) for address in self._qpos_addresses
        )
        backup = self._step_backup
        mujoco.mj_copyData(backup, self._model, self._data)
        physics_ticks = self._environment._physics_ticks  # noqa: SLF001
        environment_steps = self._environment._environment_steps  # noqa: SLF001
        settled_steps = self._environment._settled_steps  # noqa: SLF001
        maximum_distance = (
            self._config.control.max_translation_velocity_m_s * self.control_period_s
        )
        for scale in (1.0, 0.5, 0.25, 0.125, 0.0):
            if scale != 1.0:
                mujoco.mj_copyData(self._data, self._model, backup)
                self._environment._physics_ticks = physics_ticks  # noqa: SLF001
                self._environment._environment_steps = environment_steps  # noqa: SLF001
                self._environment._settled_steps = settled_steps  # noqa: SLF001
            scaled = tuple(
                current + (target - current) * scale
                for current, target in zip(current_joints, joints, strict=True)
            )
            for actuator_id, velocity in zip(
                self._velocity_actuator_ids, joint_velocity, strict=True
            ):
                self._data.ctrl[actuator_id] = velocity * scale
            state = self._environment.step(
                ActuatorTargets(_joint_vector(scaled), gripper)
            ).robot
            if float(
                np.linalg.norm(
                    np.asarray(state.end_effector_pose.position_xyz_m) - start_position
                )
            ) <= maximum_distance + 1e-9 and self._resulting_state_is_safe(state):
                return state
        mujoco.mj_copyData(self._data, self._model, backup)
        self._environment._physics_ticks = physics_ticks  # noqa: SLF001
        self._environment._environment_steps = environment_steps  # noqa: SLF001
        self._environment._settled_steps = settled_steps  # noqa: SLF001
        for dof in self._dof_addresses:
            self._data.qvel[dof] = 0.0
        for actuator_id in self._velocity_actuator_ids:
            self._data.ctrl[actuator_id] = 0.0
        return self._environment.step(
            ActuatorTargets(_joint_vector(current_joints), gripper)
        ).robot

    def _resulting_state_is_safe(self, state: RobotState) -> bool:
        position = state.end_effector_pose.position_xyz_m
        control = self._config.control
        in_workspace = all(
            lower - 1e-9 <= value <= upper + 1e-9
            for value, (lower, upper) in zip(
                position,
                (
                    control.workspace_x_m,
                    control.workspace_y_m,
                    control.workspace_z_m,
                ),
                strict=True,
            )
        )
        joints = _joint_vector(tuple(state.joint_positions_rad))
        return in_workspace and self.collision_free(joints, state.gripper_position)

    def close(self) -> None:
        self._environment.close()

    def __enter__(self) -> MujocoCartesianDriver:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()

    def _copy_state_to_scratch(self) -> None:
        self._scratch.qpos[:] = self._data.qpos
        self._scratch.qvel[:] = 0.0
        self._scratch.ctrl[:] = self._data.ctrl
        self._scratch.time = self._data.time
        mujoco.mj_forward(self._model, self._scratch)

    def _is_pad_cube_contact(self, geom_a: int, geom_b: int) -> bool:
        body_a = int(self._model.geom_bodyid[geom_a])
        body_b = int(self._model.geom_bodyid[geom_b])
        return (
            body_a == self._cube_body_id and geom_b in self._allowed_cube_geoms
        ) or (body_b == self._cube_body_id and geom_a in self._allowed_cube_geoms)

    def _descendant_body_ids(self, root_name: str) -> frozenset[int]:
        root_id = self._body_id(root_name)
        result = {root_id}
        changed = True
        while changed:
            changed = False
            for body_id in range(1, self._model.nbody):
                if (
                    int(self._model.body_parentid[body_id]) in result
                    and body_id not in result
                ):
                    result.add(body_id)
                    changed = True
        return frozenset(result)

    def _body_id(self, name: str) -> int:
        return self._named_id(mujoco.mjtObj.mjOBJ_BODY, name)

    def _geom_id(self, name: str) -> int:
        return self._named_id(mujoco.mjtObj.mjOBJ_GEOM, name)

    def _named_id(self, object_type: mujoco.mjtObj, name: str) -> int:
        result = int(mujoco.mj_name2id(self._model, object_type, name))
        if result < 0:
            raise ValueError(f"MJCF is missing required object {name!r}")
        return result


def _joint_vector(values: tuple[float, ...]) -> JointVector:
    if len(values) != 6:
        raise ValueError("UR5e joint vector must contain six values")
    return values[0], values[1], values[2], values[3], values[4], values[5]


def _quaternion_error(
    target: np.ndarray[tuple[int], np.dtype[np.float64]],
    current: np.ndarray[tuple[int], np.dtype[np.float64]],
) -> np.ndarray[tuple[int], np.dtype[np.float64]]:
    conjugate = np.array(
        [current[0], -current[1], -current[2], -current[3]], dtype=np.float64
    )
    error = np.empty(4, dtype=np.float64)
    mujoco.mju_mulQuat(error, target, conjugate)
    if error[0] < 0.0:
        error *= -1.0
    vector = error[1:]
    magnitude = math.sqrt(float(vector.dot(vector)))
    if magnitude < 1e-12:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * math.atan2(magnitude, max(float(error[0]), 0.0))
    return vector * (angle / magnitude)  # type: ignore[return-value]
