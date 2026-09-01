"""MuJoCo damped-least-squares IK and contact feasibility adapter."""

from __future__ import annotations

import math
from pathlib import Path
from types import TracebackType

import mujoco  # type: ignore[import-untyped]
import numpy as np

from act_lab.adapters.mujoco.config import CartesianControlConfig, SimulationConfig
from act_lab.adapters.mujoco.environment import ActuatorTargets, MujocoUR5eEnvironment
from act_lab.application.cartesian_control import JointVector
from act_lab.domain import Observation, Pose, RobotState


class MujocoCartesianDriver:
    """IK and predicted-contact checks around one deterministic environment."""

    def __init__(
        self, environment: MujocoUR5eEnvironment, config: SimulationConfig
    ) -> None:
        self._environment = environment
        self._config = config
        self._model = environment._model  # noqa: SLF001
        self._data = environment._data  # noqa: SLF001
        self._scratch = mujoco.MjData(self._model)
        self._joint_ids = environment._arm_joint_ids  # noqa: SLF001
        self._qpos_addresses = tuple(
            int(self._model.jnt_qposadr[joint_id]) for joint_id in self._joint_ids
        )
        self._dof_addresses = tuple(
            int(self._model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids
        )
        self._site_id = environment._end_effector_site_id  # noqa: SLF001
        self._left_finger_qpos = int(
            self._model.jnt_qposadr[environment._left_finger_joint_id]  # noqa: SLF001
        )
        right_finger = int(
            mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_JOINT, "right_finger_joint"
            )
        )
        self._right_finger_qpos = int(self._model.jnt_qposadr[right_finger])
        self._robot_body_ids = self._descendant_body_ids("base")
        self._cube_body_id = self._body_id("cube")
        self._allowed_cube_geoms = {
            self._geom_id("left_finger_pad"),
            self._geom_id("right_finger_pad"),
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
        return observation

    def observe(self) -> Observation:
        return self._environment.observe()

    def solve_ik(self, target_pose: Pose) -> JointVector | None:
        self._copy_state_to_scratch()
        target_position = np.asarray(target_pose.position_xyz_m, dtype=np.float64)
        target_quaternion = np.asarray(
            target_pose.quaternion_wxyz, dtype=np.float64
        )
        jacobian_position = np.zeros((3, self._model.nv), dtype=np.float64)
        jacobian_rotation = np.zeros((3, self._model.nv), dtype=np.float64)
        control = self._config.control

        for iteration in range(control.ik_max_iterations + 1):
            current_position = self._scratch.site_xpos[self._site_id].copy()
            current_quaternion = np.empty(4, dtype=np.float64)
            mujoco.mju_mat2Quat(
                current_quaternion, self._scratch.site_xmat[self._site_id]
            )
            position_error = target_position - current_position
            rotation_error = _quaternion_error(target_quaternion, current_quaternion)
            if (
                float(np.linalg.norm(position_error))
                <= control.ik_position_tolerance_m
                and float(np.linalg.norm(rotation_error))
                <= control.ik_orientation_tolerance_rad
            ):
                values = tuple(
                    float(self._scratch.qpos[address])
                    for address in self._qpos_addresses
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
            jacobian = np.vstack(
                (
                    jacobian_position[:, self._dof_addresses],
                    jacobian_rotation[:, self._dof_addresses],
                )
            )
            error = np.concatenate((position_error, rotation_error))
            regularizer = (control.dls_damping**2) * np.eye(6)
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + regularizer, error
            )
            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > 0.25:
                delta *= 0.25 / delta_norm
            for index, (joint_id, address) in enumerate(
                zip(self._joint_ids, self._qpos_addresses, strict=True)
            ):
                lower, upper = self._model.jnt_range[joint_id]
                margin = control.joint_bound_margin_rad
                self._scratch.qpos[address] = np.clip(
                    self._scratch.qpos[address] + delta[index],
                    lower + margin,
                    upper - margin,
                )
            mujoco.mj_forward(self._model, self._scratch)
        return None

    def collision_free(self, joints: JointVector, gripper: float) -> bool:
        self._copy_state_to_scratch()
        for address, value in zip(self._qpos_addresses, joints, strict=True):
            self._scratch.qpos[address] = value
        finger_value = gripper * 0.025
        self._scratch.qpos[self._left_finger_qpos] = finger_value
        self._scratch.qpos[self._right_finger_qpos] = finger_value
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
                and self._model.body(body_a if a_robot else body_b).name
                == "wrist_2_link"
            ):
                continue
            return False
        return True

    def step(self, joints: JointVector, gripper: float) -> RobotState:
        return self._environment.step(ActuatorTargets(joints, gripper)).robot

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
    magnitude = float(np.linalg.norm(vector))
    if magnitude < 1e-12:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * math.atan2(magnitude, max(float(error[0]), 0.0))
    return vector * (angle / magnitude)
