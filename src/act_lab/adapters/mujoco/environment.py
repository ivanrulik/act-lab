"""Fixed-step MuJoCo UR5e environment with no controller policy logic."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from importlib.resources import files
from pathlib import Path
from types import TracebackType

import mujoco  # type: ignore[import-untyped]
import numpy as np
from numpy.typing import NDArray

from act_lab.adapters.mujoco.config import SimulationConfig
from act_lab.domain import Observation, PickPlaceTaskState, Pose, RobotState

ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
ARM_ACTUATORS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow",
    "wrist_1",
    "wrist_2",
    "wrist_3",
)


@dataclass(frozen=True, slots=True)
class ActuatorTargets:
    """Adapter-local joint targets; Cartesian intent arrives in PR 3."""

    joint_positions_rad: tuple[float, float, float, float, float, float]
    gripper_position: float


@dataclass(frozen=True, slots=True)
class PickPlaceStatus:
    cube_in_tray: bool
    cube_supported: bool
    cube_settled: bool
    settled_steps: int
    success: bool
    terminal: bool
    reason: str | None


class MujocoUR5eEnvironment:
    """One isolated deterministic simulation instance."""

    def __init__(self, config: SimulationConfig) -> None:
        scene = files("act_lab.adapters.mujoco").joinpath(
            "assets", "ur5e", "act_lab_scene.xml"
        )
        self._model = mujoco.MjModel.from_xml_path(str(scene))
        expected_timestep = 1.0 / config.physics_hz
        if not math.isclose(self._model.opt.timestep, expected_timestep):
            raise ValueError("configured physics_hz does not match the MJCF timestep")
        self._data = mujoco.MjData(self._model)
        self._config = config
        self._rng = np.random.default_rng(0)
        self._renderer: mujoco.Renderer | None = None
        self._physics_ticks = 0
        self._environment_steps = 0
        self._settled_steps = 0
        self._closed = False

        self._arm_joint_ids = tuple(self._joint_id(name) for name in ARM_JOINTS)
        self._arm_actuator_ids = tuple(
            self._actuator_id(name) for name in ARM_ACTUATORS
        )
        self._gripper_actuator_id = self._actuator_id("gripper")
        self._left_finger_joint_id = self._joint_id("left_finger_joint")
        self._cube_joint_id = self._joint_id("cube_free_joint")
        self._end_effector_site_id = self._site_id("end_effector")
        self._camera_ids = {name: self._camera_id(name) for name in config.cameras}

    @classmethod
    def from_config_file(cls, path: Path) -> MujocoUR5eEnvironment:
        return cls(SimulationConfig.load(path))

    @property
    def neutral_targets(self) -> ActuatorTargets:
        return ActuatorTargets(self._config.home_joint_positions_rad, 0.0)

    @property
    def physics_ticks(self) -> int:
        return self._physics_ticks

    @property
    def environment_steps(self) -> int:
        return self._environment_steps

    @property
    def cube_position_xyz_m(self) -> tuple[float, float, float]:
        position = self._data.joint("cube_free_joint").qpos[:3]
        return float(position[0]), float(position[1]), float(position[2])

    def reset(self, seed: int) -> Observation:
        self._ensure_open()
        if isinstance(seed, bool):
            raise ValueError("seed must be an integer")
        self._rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self._model, self._data)
        self._physics_ticks = 0
        self._environment_steps = 0
        self._settled_steps = 0

        for joint_id, value in zip(
            self._arm_joint_ids, self._config.home_joint_positions_rad, strict=True
        ):
            self._data.qpos[self._model.jnt_qposadr[joint_id]] = value
        left_qpos_address = self._model.jnt_qposadr[self._left_finger_joint_id]
        self._data.qpos[left_qpos_address] = 0.0

        cube_qpos_address = self._model.jnt_qposadr[self._cube_joint_id]
        self._data.qpos[cube_qpos_address : cube_qpos_address + 7] = (
            self._rng.uniform(*self._config.cube_spawn_x_m),
            self._rng.uniform(*self._config.cube_spawn_y_m),
            self._config.cube_center_z_m,
            1.0,
            0.0,
            0.0,
            0.0,
        )
        self._apply_targets(self.neutral_targets)
        mujoco.mj_forward(self._model, self._data)
        return self.observe()

    def step(self, targets: ActuatorTargets | None = None) -> Observation:
        self._ensure_open()
        self._apply_targets(targets or self.neutral_targets)
        for _ in range(self._config.substeps):
            mujoco.mj_step(self._model, self._data)
            self._physics_ticks += 1
        self._environment_steps += 1
        self._update_settling_counter()
        if not np.isfinite(self._data.qpos).all() or not np.isfinite(
            self._data.qvel
        ).all():
            raise RuntimeError("MuJoCo produced non-finite state")
        return self.observe()

    def observe(self) -> Observation:
        self._ensure_open()
        timestamp_ns = self._physics_ticks * self._config.physics_step_ns
        joint_positions = tuple(
            float(self._data.qpos[self._model.jnt_qposadr[joint_id]])
            for joint_id in self._arm_joint_ids
        )
        joint_velocities = tuple(
            float(self._data.qvel[self._model.jnt_dofadr[joint_id]])
            for joint_id in self._arm_joint_ids
        )
        site_position = self._data.site_xpos[self._end_effector_site_id]
        quaternion = np.empty(4, dtype=np.float64)
        mujoco.mju_mat2Quat(
            quaternion, self._data.site_xmat[self._end_effector_site_id]
        )
        finger_qpos = self._data.qpos[
            self._model.jnt_qposadr[self._left_finger_joint_id]
        ]
        state = RobotState(
            timestamp_ns=timestamp_ns,
            joint_positions_rad=joint_positions,
            joint_velocities_rad_s=joint_velocities,
            end_effector_pose=Pose(
                frame_id="world",
                position_xyz_m=(
                    float(site_position[0]),
                    float(site_position[1]),
                    float(site_position[2]),
                ),
                quaternion_wxyz=(
                    float(quaternion[0]),
                    float(quaternion[1]),
                    float(quaternion[2]),
                    float(quaternion[3]),
                ),
            ),
            gripper_position=float(np.clip(finger_qpos / 0.025, 0.0, 1.0)),
        )
        return Observation(
            timestamp_ns=timestamp_ns,
            robot=state,
            image_keys=self._config.cameras,
        )

    def task_status(self) -> PickPlaceStatus:
        cube_position = self._data.joint("cube_free_joint").qpos[:3]
        cube_velocity = self._data.joint("cube_free_joint").qvel
        x_margin = (
            self._config.tray_inner_half_extents_xy_m[0]
            - self._config.cube_half_extent_m
        )
        y_margin = (
            self._config.tray_inner_half_extents_xy_m[1]
            - self._config.cube_half_extent_m
        )
        cube_in_tray = bool(
            abs(cube_position[0] - self._config.tray_center_xy_m[0]) <= x_margin
            and abs(cube_position[1] - self._config.tray_center_xy_m[1]) <= y_margin
        )
        cube_bottom = cube_position[2] - self._config.cube_half_extent_m
        cube_supported = bool(
            abs(cube_bottom - self._config.tray_floor_top_z_m)
            <= self._config.support_tolerance_m
        )
        linear_speed = float(np.linalg.norm(cube_velocity[:3]))
        angular_speed = float(np.linalg.norm(cube_velocity[3:]))
        cube_settled = (
            linear_speed <= self._config.linear_speed_threshold_m_s
            and angular_speed <= self._config.angular_speed_threshold_rad_s
        )
        success = (
            cube_in_tray
            and cube_supported
            and cube_settled
            and self._settled_steps >= self._config.settling_steps
        )
        timed_out = self._environment_steps >= self._config.episode_steps
        reason = "success" if success else "timeout" if timed_out else None
        return PickPlaceStatus(
            cube_in_tray=cube_in_tray,
            cube_supported=cube_supported,
            cube_settled=cube_settled,
            settled_steps=self._settled_steps,
            success=success,
            terminal=success or timed_out,
            reason=reason,
        )

    def task_state(self) -> PickPlaceTaskState:
        """Return privileged task geometry without changing policy observations."""
        self._ensure_open()
        cube_qpos = self._data.joint("cube_free_joint").qpos
        status = self.task_status()
        return PickPlaceTaskState(
            cube_pose=Pose(
                frame_id="world",
                position_xyz_m=(
                    float(cube_qpos[0]),
                    float(cube_qpos[1]),
                    float(cube_qpos[2]),
                ),
                quaternion_wxyz=(
                    float(cube_qpos[3]),
                    float(cube_qpos[4]),
                    float(cube_qpos[5]),
                    float(cube_qpos[6]),
                ),
            ),
            desired_cube_pose=Pose(
                frame_id="world",
                position_xyz_m=(
                    self._config.tray_center_xy_m[0],
                    self._config.tray_center_xy_m[1],
                    self._config.tray_floor_top_z_m
                    + self._config.cube_half_extent_m,
                ),
                quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
            ),
            success=status.success,
            terminal=status.terminal,
            reason=status.reason,
        )

    def render(self, camera: str) -> NDArray[np.uint8]:
        self._ensure_open()
        if camera not in self._camera_ids:
            known = ", ".join(sorted(self._camera_ids))
            raise ValueError(f"unknown camera {camera!r}; expected one of: {known}")
        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                self._model,
                height=self._config.render_height,
                width=self._config.render_width,
            )
        self._renderer.update_scene(self._data, camera=camera)
        frame = self._renderer.render()
        return np.asarray(frame, dtype=np.uint8).copy()

    def launch_viewer(self, seed: int, *, max_steps: int | None = None) -> None:
        """Run a real-time passive viewer until its window closes."""
        from mujoco import viewer

        if max_steps is not None and max_steps <= 0:
            raise ValueError("max_steps must be positive when provided")
        self.reset(seed)
        period_s = 1.0 / self._config.environment_hz
        completed_steps = 0
        with viewer.launch_passive(self._model, self._data) as handle:
            while handle.is_running() and (
                max_steps is None or completed_steps < max_steps
            ):
                started_at = time.monotonic()
                self.step()
                completed_steps += 1
                handle.sync()
                remaining_s = period_s - (time.monotonic() - started_at)
                if remaining_s > 0:
                    time.sleep(remaining_s)

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        self._closed = True

    def __enter__(self) -> MujocoUR5eEnvironment:
        self._ensure_open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()

    def _apply_targets(self, targets: ActuatorTargets) -> None:
        if len(targets.joint_positions_rad) != len(ARM_JOINTS):
            raise ValueError("joint target must contain exactly six values")
        values = (*targets.joint_positions_rad, targets.gripper_position)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("actuator targets must be finite")
        if not 0.0 <= targets.gripper_position <= 1.0:
            raise ValueError("gripper target must be in [0, 1]")
        for actuator_id, value in zip(
            self._arm_actuator_ids, targets.joint_positions_rad, strict=True
        ):
            self._data.ctrl[actuator_id] = value
        self._data.ctrl[self._gripper_actuator_id] = (
            targets.gripper_position * 0.025
        )

    def _update_settling_counter(self) -> None:
        status = self.task_status()
        if status.cube_in_tray and status.cube_supported and status.cube_settled:
            self._settled_steps += 1
        else:
            self._settled_steps = 0

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("environment is closed")

    def _joint_id(self, name: str) -> int:
        return self._named_id(mujoco.mjtObj.mjOBJ_JOINT, name)

    def _actuator_id(self, name: str) -> int:
        return self._named_id(mujoco.mjtObj.mjOBJ_ACTUATOR, name)

    def _site_id(self, name: str) -> int:
        return self._named_id(mujoco.mjtObj.mjOBJ_SITE, name)

    def _camera_id(self, name: str) -> int:
        return self._named_id(mujoco.mjtObj.mjOBJ_CAMERA, name)

    def _named_id(self, object_type: mujoco.mjtObj, name: str) -> int:
        object_id = int(mujoco.mj_name2id(self._model, object_type, name))
        if object_id < 0:
            raise ValueError(f"MJCF is missing required object {name!r}")
        return object_id
