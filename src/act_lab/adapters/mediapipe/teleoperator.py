"""Observation-clocked mapping from tracked hands to Cartesian intent."""

from __future__ import annotations

import hashlib
import json
import math
import statistics
import threading
import time
from dataclasses import asdict, dataclass

from act_lab.adapters.mediapipe.config import WebcamConfig
from act_lab.adapters.mediapipe.hand_tracker import HandSignal
from act_lab.domain import Action, CameraFrame, Observation, Pose


@dataclass(frozen=True, slots=True)
class TeleopDiagnostics:
    state: str
    source_status: str
    frames_seen: int
    tracking_losses: int
    clutch_transitions: int
    calibration_samples: int
    handedness: str | None
    confidence: float | None
    frame_age_ms: float | None
    clutch_detected: bool
    clutch_active: bool
    calibration_id: str | None
    palm_xy: tuple[float, float] | None
    clutch_anchor_xy: tuple[float, float] | None
    clutch_anchor_scale: float | None
    command_offset_xyz_m: tuple[float, float, float]
    commanded_gripper_position: float | None


@dataclass(frozen=True, slots=True)
class _Calibration:
    palm_x: float
    palm_y: float
    palm_scale: float
    handedness: str
    robot_position_xyz_m: tuple[float, float, float]
    robot_quaternion_wxyz: tuple[float, float, float, float]
    calibration_id: str


class WebcamTeleoperator:
    """Fail-safe hand mapper implementing the generic Teleoperator contract."""

    def __init__(self, config: WebcamConfig) -> None:
        self._config = config
        self._lock = threading.Lock()
        self._sequence = 0
        self._consumed_sequence = 0
        self._signal: HandSignal | None = None
        self._received_ns: int | None = None
        self._source_status = "waiting_for_camera"
        self._preview: CameraFrame | None = None
        self._frames_seen = 0
        self._tracking_losses = 0
        self._clutch_transitions = 0
        self._calibration_requested = False
        self._calibration_samples: list[HandSignal] = []
        self._calibration: _Calibration | None = None
        self._clutch_count = 0
        self._clutch_active = False
        self._anchor_signal: HandSignal | None = None
        self._anchor_pose: Pose | None = None
        self._filtered_offset = (0.0, 0.0, 0.0)
        self._last_action: Action | None = None
        self._state = "uncalibrated"
        self._quit_requested = False

    @property
    def quit_requested(self) -> bool:
        return self._quit_requested

    @property
    def preview(self) -> CameraFrame | None:
        with self._lock:
            return self._preview

    @property
    def config(self) -> WebcamConfig:
        return self._config

    @property
    def diagnostics(self) -> TeleopDiagnostics:
        with self._lock:
            signal = self._signal
            received_ns = self._received_ns
            return TeleopDiagnostics(
                state=self._state,
                source_status=self._source_status,
                frames_seen=self._frames_seen,
                tracking_losses=self._tracking_losses,
                clutch_transitions=self._clutch_transitions,
                calibration_samples=len(self._calibration_samples),
                handedness=signal.handedness if signal else None,
                confidence=signal.confidence if signal else None,
                frame_age_ms=(
                    (time.monotonic_ns() - received_ns) / 1_000_000
                    if received_ns is not None
                    else None
                ),
                clutch_detected=signal.clutch if signal else False,
                clutch_active=self._clutch_active,
                calibration_id=(
                    self._calibration.calibration_id if self._calibration else None
                ),
                palm_xy=(signal.palm_x, signal.palm_y) if signal else None,
                clutch_anchor_xy=(
                    (self._anchor_signal.palm_x, self._anchor_signal.palm_y)
                    if self._anchor_signal
                    else None
                ),
                clutch_anchor_scale=(
                    self._anchor_signal.palm_scale if self._anchor_signal else None
                ),
                command_offset_xyz_m=self._filtered_offset,
                commanded_gripper_position=(
                    self._last_action.gripper_position if self._last_action else None
                ),
            )

    @property
    def calibration_snapshot(self) -> dict[str, object] | None:
        with self._lock:
            if self._calibration is None:
                return None
            return asdict(self._calibration)

    def on_key(self, keycode: int) -> None:
        # GLFW_KEY_ENTER is outside the ASCII range. Avoid C because MuJoCo's
        # native viewer reserves Ctrl+C for copying state to a keyframe and can
        # abort when its selected keyframe is unset.
        if keycode == 257:
            self.request_calibration()
            return
        if 0 <= keycode <= 255:
            key = chr(keycode).upper()
            if key == "Q":
                self._quit_requested = True

    def request_calibration(self) -> None:
        with self._lock:
            self._calibration_requested = True
            self._calibration_samples.clear()
            self._calibration = None
            self._disengage_locked()
            self._state = "calibrating"

    def update(
        self,
        signal: HandSignal | None,
        received_monotonic_ns: int,
        status: str,
    ) -> None:
        with self._lock:
            self._sequence += 1
            self._signal = signal
            self._received_ns = received_monotonic_ns
            self._source_status = status
            if status in {"tracked", "tracking_lost"}:
                self._frames_seen += 1
            if signal is None:
                if status == "tracking_lost":
                    self._tracking_losses += 1
                self._state = "tracking_lost"
                self._disengage_locked()
            else:
                self._preview = signal.preview

    def poll(self, observation: Observation) -> Action:
        with self._lock:
            if self._last_action is None:
                self._last_action = _disabled_action(
                    observation, observation.timestamp_ns
                )
            if self._sequence == self._consumed_sequence:
                return self._last_action
            self._consumed_sequence = self._sequence
            signal = self._signal
            received_ns = self._received_ns
            if signal is None or received_ns is None:
                return self._disable(observation, "tracking_lost")
            age_ms = (time.monotonic_ns() - received_ns) / 1_000_000
            if age_ms >= self._config.max_frame_age_ms:
                return self._disable(observation, "stale_camera_frame")
            if not _finite_signal(signal):
                return self._disable(observation, "invalid_landmarks")
            if signal.confidence < self._config.min_handedness_confidence:
                return self._disable(observation, "low_confidence")
            if self._calibration_requested:
                return self._collect_calibration(signal, observation)
            if self._calibration is None:
                return self._disable(observation, "uncalibrated")
            if signal.handedness != self._calibration.handedness:
                return self._disable(observation, "handedness_changed")
            if not signal.clutch:
                self._disengage_locked()
                return self._disable(observation, "clutch_released")
            self._clutch_count += 1
            if not self._clutch_active:
                if self._clutch_count < self._config.clutch_engage_frames:
                    return self._disable(observation, "clutch_debounce")
                self._clutch_active = True
                self._clutch_transitions += 1
                self._anchor_signal = signal
                self._anchor_pose = observation.robot.end_effector_pose
                self._filtered_offset = (0.0, 0.0, 0.0)
            assert self._anchor_signal is not None
            assert self._anchor_pose is not None
            action = self._map(signal, observation)
            self._last_action = action
            self._state = "active"
            return action

    def _collect_calibration(
        self, signal: HandSignal, observation: Observation
    ) -> Action:
        self._calibration_samples.append(signal)
        if len(self._calibration_samples) > self._config.calibration_sample_frames:
            self._calibration_samples.pop(0)
        if len(self._calibration_samples) == self._config.calibration_sample_frames:
            xs = [item.palm_x for item in self._calibration_samples]
            ys = [item.palm_y for item in self._calibration_samples]
            scales = [item.palm_scale for item in self._calibration_samples]
            anchor_spread = max(max(xs) - min(xs), max(ys) - min(ys))
            median_scale = statistics.median(scales)
            scale_spread = (max(scales) - min(scales)) / median_scale
            handedness = {item.handedness for item in self._calibration_samples}
            if (
                anchor_spread <= self._config.calibration_max_anchor_spread
                and scale_spread <= self._config.calibration_max_scale_spread
                and len(handedness) == 1
            ):
                median_x = statistics.median(xs)
                median_y = statistics.median(ys)
                calibrated_hand = next(iter(handedness))
                robot_pose = observation.robot.end_effector_pose
                payload = {
                    "palm_x": median_x,
                    "palm_y": median_y,
                    "palm_scale": median_scale,
                    "handedness": calibrated_hand,
                    "robot_position_xyz_m": robot_pose.position_xyz_m,
                    "robot_quaternion_wxyz": robot_pose.quaternion_wxyz,
                }
                calibration_id = hashlib.sha256(
                    json.dumps(payload, sort_keys=True).encode()
                ).hexdigest()[:16]
                self._calibration = _Calibration(
                    palm_x=median_x,
                    palm_y=median_y,
                    palm_scale=median_scale,
                    handedness=calibrated_hand,
                    robot_position_xyz_m=robot_pose.position_xyz_m,
                    robot_quaternion_wxyz=robot_pose.quaternion_wxyz,
                    calibration_id=calibration_id,
                )
                self._calibration_requested = False
                self._calibration_samples.clear()
                self._state = "ready"
            else:
                self._state = "calibration_unstable"
        self._last_action = _disabled_action(observation, observation.timestamp_ns)
        return self._last_action

    def _map(self, signal: HandSignal, observation: Observation) -> Action:
        assert self._anchor_signal is not None
        assert self._anchor_pose is not None
        scale = self._anchor_signal.palm_scale
        image_x = _dead_zone(
            (signal.palm_x - self._anchor_signal.palm_x) / scale,
            self._config.image_xy_dead_zone,
        )
        image_y = _dead_zone(
            (signal.palm_y - self._anchor_signal.palm_y) / scale,
            self._config.image_xy_dead_zone,
        )
        # Log scale makes equal toward/away ratios produce equal and opposite
        # displacement. Apparent palm size is the noisiest monocular signal,
        # so it has its own dead zone and stronger low-pass filter below.
        depth = _dead_zone(
            math.log(signal.palm_scale / scale), self._config.depth_dead_zone
        )
        desired = (
            image_y * self._config.world_x_gain_m,
            image_x * self._config.world_y_gain_m,
            depth * self._config.world_z_gain_m,
        )
        alphas = (
            self._config.ema_alpha,
            self._config.ema_alpha,
            self._config.depth_ema_alpha,
        )
        filtered = tuple(
            previous + alpha * (target - previous)
            for previous, target, alpha in zip(
                self._filtered_offset, desired, alphas, strict=True
            )
        )
        self._filtered_offset = (filtered[0], filtered[1], filtered[2])
        base = self._anchor_pose.position_xyz_m
        position = tuple(
            value + offset
            for value, offset in zip(base, self._filtered_offset, strict=True)
        )
        target_pose = Pose(
            frame_id="world",
            position_xyz_m=(position[0], position[1], position[2]),
            quaternion_wxyz=self._anchor_pose.quaternion_wxyz,
        )
        gripper = (signal.pinch_ratio - self._config.pinch_closed_ratio) / (
            self._config.pinch_open_ratio - self._config.pinch_closed_ratio
        )
        return Action(
            timestamp_ns=observation.timestamp_ns,
            target_pose=target_pose,
            gripper_position=min(max(gripper, 0.0), 1.0),
            enabled=True,
        )

    def _disable(self, observation: Observation, state: str) -> Action:
        self._state = state
        timestamp = (
            self._last_action.timestamp_ns
            if self._last_action is not None
            else observation.timestamp_ns
        )
        self._last_action = _disabled_action(observation, timestamp)
        return self._last_action

    def _disengage_locked(self) -> None:
        if self._clutch_active:
            self._clutch_transitions += 1
        self._clutch_active = False
        self._clutch_count = 0
        self._anchor_signal = None
        self._anchor_pose = None
        self._filtered_offset = (0.0, 0.0, 0.0)


def _disabled_action(observation: Observation, timestamp_ns: int) -> Action:
    return Action(
        timestamp_ns=timestamp_ns,
        target_pose=observation.robot.end_effector_pose,
        gripper_position=observation.robot.gripper_position,
        enabled=False,
    )


def _dead_zone(value: float, threshold: float) -> float:
    magnitude = abs(value)
    if magnitude <= threshold:
        return 0.0
    return math.copysign((magnitude - threshold) / (1.0 - threshold), value)


def _finite_signal(signal: HandSignal) -> bool:
    return signal.palm_scale > 0.0 and all(
        math.isfinite(value)
        for value in (
            signal.confidence,
            signal.palm_x,
            signal.palm_y,
            signal.palm_scale,
            signal.pinch_ratio,
        )
    )
