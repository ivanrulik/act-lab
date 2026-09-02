"""Event-driven keyboard intent adapter for the MuJoCo passive viewer."""

from __future__ import annotations

import time
from collections import deque
from threading import Lock
from typing import TYPE_CHECKING

from act_lab.adapters.mujoco.config import CartesianControlConfig, KeyboardConfig
from act_lab.domain import Action, Observation, Pose

if TYPE_CHECKING:
    from act_lab.adapters.mujoco.cartesian_driver import MujocoCartesianDriver
    from act_lab.application import SafeCartesianRobot


class KeyboardTeleoperator:
    """Convert queued viewer key events into watchdog-backed Cartesian intent."""

    _TRANSLATION_KEYS = {
        ord("W"): (1.0, 0.0, 0.0),
        ord("S"): (-1.0, 0.0, 0.0),
        ord("A"): (0.0, 1.0, 0.0),
        ord("D"): (0.0, -1.0, 0.0),
        ord("R"): (0.0, 0.0, 1.0),
        ord("F"): (0.0, 0.0, -1.0),
    }

    def __init__(
        self, config: KeyboardConfig, limits: CartesianControlConfig
    ) -> None:
        self._config = config
        self._limits = limits
        self._events: deque[int] = deque()
        self._events_lock = Lock()
        self._target_pose: Pose | None = None
        self._gripper = 0.0
        self._enabled = False
        self._last_input_timestamp_ns = 0
        self._quit_requested = False
        self._accepted_event_count = 0
        self._latest_input = "none"

    @property
    def quit_requested(self) -> bool:
        return self._quit_requested

    @property
    def target_pose(self) -> Pose | None:
        return self._target_pose

    @property
    def accepted_event_count(self) -> int:
        return self._accepted_event_count

    @property
    def latest_input(self) -> str:
        return self._latest_input

    @property
    def controls_text(self) -> str:
        return (
            "Tap W/S: X  A/D: Y  R/F: Z  O/C: open/close  "
            "Space: stop  Q: quit"
        )

    def on_key(self, keycode: int) -> None:
        """Viewer callback: enqueue only, keeping simulator access on its thread."""
        with self._events_lock:
            self._events.append(keycode)

    def poll(self, observation: Observation) -> Action:
        if self._target_pose is None:
            self._target_pose = observation.robot.end_effector_pose
            self._gripper = observation.robot.gripper_position
            self._last_input_timestamp_ns = observation.timestamp_ns

        with self._events_lock:
            events = tuple(self._events)
            self._events.clear()
        for raw_keycode in events:
            self._apply_key(raw_keycode, observation)

        assert self._target_pose is not None
        return Action(
            timestamp_ns=self._last_input_timestamp_ns,
            target_pose=self._target_pose,
            gripper_position=self._gripper,
            enabled=self._enabled,
        )

    def _apply_key(self, raw_keycode: int, observation: Observation) -> None:
        keycode = (
            ord(chr(raw_keycode).upper())
            if 0 <= raw_keycode <= 255
            else raw_keycode
        )
        if keycode == ord("Q"):
            self._quit_requested = True
            self._record_input("Q (quit)")
            self._stop(observation)
            return
        if keycode == ord(" "):
            self._record_input("Space (stop)")
            self._stop(observation)
            return
        direction = self._TRANSLATION_KEYS.get(keycode)
        if direction is not None:
            assert self._target_pose is not None
            current = self._target_pose.position_xyz_m
            requested = tuple(
                current[index]
                + direction[index] * self._config.translation_nudge_m
                for index in range(3)
            )
            bounds = (
                self._limits.workspace_x_m,
                self._limits.workspace_y_m,
                self._limits.workspace_z_m,
            )
            position = tuple(
                min(max(value, axis[0]), axis[1])
                for value, axis in zip(requested, bounds, strict=True)
            )
            self._target_pose = Pose(
                "world",
                (position[0], position[1], position[2]),
                self._target_pose.quaternion_wxyz,
            )
            self._record_input(chr(keycode))
            self._accept_input(observation)
            return
        if keycode in {ord("O"), ord("C")}:
            sign = 1.0 if keycode == ord("O") else -1.0
            self._gripper = min(
                max(self._gripper + sign * self._config.gripper_nudge, 0.0), 1.0
            )
            self._record_input(chr(keycode))
            self._accept_input(observation)

    def _record_input(self, label: str) -> None:
        self._accepted_event_count += 1
        self._latest_input = label

    def _accept_input(self, observation: Observation) -> None:
        self._enabled = True
        self._last_input_timestamp_ns = observation.timestamp_ns

    def _stop(self, observation: Observation) -> None:
        self._target_pose = observation.robot.end_effector_pose
        self._gripper = observation.robot.gripper_position
        self._enabled = False
        self._last_input_timestamp_ns = observation.timestamp_ns


def run_keyboard_session(
    driver: MujocoCartesianDriver,
    robot: SafeCartesianRobot,
    teleoperator: KeyboardTeleoperator,
    seed: int,
    *,
    max_steps: int | None = None,
) -> None:
    """Run the supported passive viewer callback through the safe robot path."""
    from mujoco import mjtGridPos, viewer  # type: ignore[import-untyped]

    if max_steps is not None and max_steps <= 0:
        raise ValueError("max_steps must be positive when provided")
    observation = robot.reset(seed)
    environment = driver.environment
    period_s = driver.control_period_s
    completed_steps = 0
    with viewer.launch_passive(
        environment._model,  # noqa: SLF001
        environment._data,  # noqa: SLF001
        key_callback=teleoperator.on_key,
    ) as handle:
        while handle.is_running() and (
            max_steps is None or completed_steps < max_steps
        ):
            started_at = time.monotonic()
            # MuJoCo's passive viewer owns a render thread. Its documented lock
            # protects the shared MjData while control advances physics.
            with handle.lock():
                action = teleoperator.poll(observation)
                robot.command(action)
                observation = robot.observe()
                report = robot.last_report
                task = environment.task_state()
            completed_steps += 1
            target = action.target_pose.position_xyz_m
            actual = observation.robot.end_effector_pose.position_xyz_m
            handle.set_texts(
                (
                    None,
                    mjtGridPos.mjGRID_TOPLEFT,
                    "ACT Lab keyboard teleoperation",
                    "\n".join(
                        (
                            teleoperator.controls_text,
                            "input: "
                            f"{teleoperator.latest_input} "
                            f"(accepted {teleoperator.accepted_event_count})",
                            "target: "
                            f"({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})",
                            "actual: "
                            f"({actual[0]:.3f}, {actual[1]:.3f}, {actual[2]:.3f})",
                            f"gripper: {action.gripper_position:.2f}",
                            "safety: "
                            f"{report.outcome.value if report else 'none'}"
                            f" ({report.detail if report else 'no command'})",
                            f"task: {task.reason or 'running'}",
                        )
                    ),
                )
            )
            handle.sync()
            if teleoperator.quit_requested:
                handle.close()
                break
            remaining_s = period_s - (time.monotonic() - started_at)
            if remaining_s > 0:
                time.sleep(remaining_s)
