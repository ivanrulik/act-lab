"""Held-key X11 input for safe Cartesian keyboard teleoperation."""

from __future__ import annotations

import math
import time
from threading import Event, Lock, Thread
from typing import TYPE_CHECKING, Any

from act_lab.adapters.mujoco.config import CartesianControlConfig, KeyboardConfig
from act_lab.domain import Action, Observation, Pose

if TYPE_CHECKING:
    from act_lab.adapters.mujoco.cartesian_driver import MujocoCartesianDriver
    from act_lab.application import SafeCartesianRobot


class KeyboardTeleoperator:
    _DIRECTIONS = {
        "w": (1, 0, 0),
        "s": (-1, 0, 0),
        "a": (0, 1, 0),
        "d": (0, -1, 0),
        "r": (0, 0, 1),
        "f": (0, 0, -1),
    }

    def __init__(self, config: KeyboardConfig, limits: CartesianControlConfig) -> None:
        self._config, self._limits = config, limits
        self._lock, self._held = Lock(), set[str]()
        self._focused = self._quit_requested = False
        self._target_pose: Pose | None = None
        self._gripper, self._accepted_cycles = 0.0, 0
        self._latest_input = "waiting for viewer focus"

    @property
    def quit_requested(self) -> bool:
        with self._lock:
            return self._quit_requested

    @property
    def target_pose(self) -> Pose | None:
        return self._target_pose

    @property
    def accepted_event_count(self) -> int:
        return self._accepted_cycles

    @property
    def latest_input(self) -> str:
        return self._latest_input

    @property
    def controls_text(self) -> str:
        return "Hold Shift + W/S: X  A/D: Y  R/F: Z  O/C: open/close  Q: quit"

    def update_key(self, key: str, pressed: bool) -> None:
        key = key.lower()
        with self._lock:
            if key == "q" and pressed:
                self._quit_requested = True
            (self._held.add if pressed else self._held.discard)(key)

    def update_focus(self, focused: bool) -> None:
        with self._lock:
            self._focused = focused
            if not focused:
                self._held.clear()

    def request_quit(self) -> None:
        """Request shutdown from either the X11 listener or MuJoCo callback."""
        with self._lock:
            self._quit_requested = True

    def poll(self, observation: Observation) -> Action:
        measured = observation.robot
        if self._target_pose is None:
            self._target_pose = measured.end_effector_pose
            self._gripper = measured.gripper_position
        with self._lock:
            held, focused = frozenset(self._held), self._focused
        motion = set(self._DIRECTIONS).intersection(held)
        enabled = focused and "shift" in held and bool(motion or held & {"o", "c"})
        if not enabled:
            self._target_pose = measured.end_effector_pose
            self._latest_input = (
                "focus lost" if not focused else "deadman/motion released"
            )
            return Action(
                observation.timestamp_ns, self._target_pose, self._gripper, False
            )
        direction = [
            sum(self._DIRECTIONS[key][axis] for key in motion) for axis in range(3)
        ]
        magnitude = math.sqrt(sum(value * value for value in direction))
        scale = (
            self._config.translation_speed_m_s / magnitude / 50.0 if magnitude else 0.0
        )
        bounds = (
            self._limits.workspace_x_m,
            self._limits.workspace_y_m,
            self._limits.workspace_z_m,
        )
        position = tuple(
            min(max(value + direction[i] * scale, bounds[i][0]), bounds[i][1])
            for i, value in enumerate(measured.end_effector_pose.position_xyz_m)
        )
        grip_direction = float("o" in held) - float("c" in held)
        self._gripper = min(
            max(
                self._gripper + grip_direction * self._config.gripper_speed_s / 50.0,
                0.0,
            ),
            1.0,
        )
        self._target_pose = Pose(
            "world",
            (position[0], position[1], position[2]),
            measured.end_effector_pose.quaternion_wxyz,
        )
        self._accepted_cycles += 1
        self._latest_input = "+".join(sorted(held))
        return Action(observation.timestamp_ns, self._target_pose, self._gripper, True)


class X11KeyboardAdapter:
    """Capture releases globally and fail closed unless MuJoCo owns X11 focus."""

    def __init__(self, teleoperator: KeyboardTeleoperator) -> None:
        self._teleoperator, self._stop = teleoperator, Event()
        self._listener: Any | None = None
        self._thread: Thread | None = None

    def start(self) -> None:
        try:
            from pynput import keyboard  # type: ignore[import-untyped]
            from Xlib import display  # type: ignore[import-untyped]
        except ImportError as error:
            raise RuntimeError(
                "keyboard teleoperation requires the optional 'ui' dependencies"
            ) from error
        xdisplay = display.Display()

        def label(key: object) -> str | None:
            char = getattr(key, "char", None)
            return (
                char.lower()
                if isinstance(char, str)
                else (
                    "shift"
                    if key
                    in {keyboard.Key.shift, keyboard.Key.shift_l, keyboard.Key.shift_r}
                    else None
                )
            )

        def transition(key: object, pressed: bool) -> None:
            name = label(key)
            if name is not None:
                self._teleoperator.update_key(name, pressed)

        self._listener = keyboard.Listener(
            on_press=lambda key: transition(key, True),
            on_release=lambda key: transition(key, False),
        )
        self._listener.start()

        def monitor() -> None:
            while not self._stop.wait(0.02):
                try:
                    focus = xdisplay.get_input_focus().focus
                    title = (
                        focus.get_wm_name() if hasattr(focus, "get_wm_name") else None
                    )
                    self._teleoperator.update_focus(
                        isinstance(title, str) and "mujoco" in title.lower()
                    )
                except Exception:
                    self._teleoperator.update_focus(False)

        self._thread = Thread(target=monitor, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._listener is not None:
            self._listener.stop()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._teleoperator.update_focus(False)


def run_keyboard_session(
    driver: MujocoCartesianDriver,
    robot: SafeCartesianRobot,
    teleoperator: KeyboardTeleoperator,
    seed: int,
    *,
    max_steps: int | None = None,
) -> None:
    from mujoco import mjtGridPos, viewer  # type: ignore[import-untyped]

    if max_steps is not None and max_steps <= 0:
        raise ValueError("max_steps must be positive when provided")
    observation, environment = robot.reset(seed), driver.environment
    started, completed, adapter = time.monotonic(), 0, X11KeyboardAdapter(teleoperator)

    def viewer_key(keycode: int) -> None:
        # MuJoCo's callback is sufficient for the press-only quit action and
        # provides a fallback if the global X11 listener misses the focused key.
        if keycode in {ord("q"), ord("Q")}:
            teleoperator.request_quit()

    with viewer.launch_passive(  # noqa: SLF001
        environment._model,
        environment._data,
        key_callback=viewer_key,
    ) as handle:
        adapter.start()
        try:
            while handle.is_running() and (max_steps is None or completed < max_steps):
                cycle = time.monotonic()
                with handle.lock():
                    action = teleoperator.poll(observation)
                    robot.command(action)
                    observation, report, task = (
                        robot.observe(),
                        robot.last_report,
                        environment.task_state(),
                    )
                completed += 1
                requested, measured = (
                    action.target_pose.position_xyz_m,
                    observation.robot.end_effector_pose.position_xyz_m,
                )
                limited = (
                    report.executed_action.target_pose.position_xyz_m
                    if report and report.executed_action
                    else None
                )
                rtf = (
                    observation.timestamp_ns
                    / 1e9
                    / max(time.monotonic() - started, 1e-9)
                )
                outcome = report.outcome.value if report else "none"
                detail = report.detail if report else "no command"
                lines = (
                    teleoperator.controls_text,
                    f"input: {teleoperator.latest_input}",
                    f"requested: {requested}",
                    f"limited: {limited}",
                    f"measured: {measured}",
                    f"timing: RTF={rtf:.2f}",
                    f"safety: {outcome} ({detail})",
                    f"task: {task.reason or 'running'}",
                )
                handle.set_texts(
                    (
                        None,
                        mjtGridPos.mjGRID_TOPLEFT,
                        "ACT Lab keyboard teleoperation",
                        "\n".join(lines),
                    )
                )
                handle.sync()
                if teleoperator.quit_requested:
                    handle.close()
                    break
                remaining = driver.control_period_s - (time.monotonic() - cycle)
                if remaining > 0:
                    time.sleep(remaining)
        finally:
            adapter.stop()
