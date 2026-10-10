"""Actuator-owner authorization state machine, independent of ROS and physics.

Time values are injected. Refreshing controller output never refreshes the source
command, receipt age or clock-progress lease. This is for simulation only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from uuid import UUID

from act_lab.adapters.ros2.contracts import WATCHDOG_NS
from act_lab.domain import Pose


@dataclass(frozen=True, slots=True)
class Authorization:
    episode: UUID
    sequence: int
    source_ns: int
    receipt_ns: int
    pose: Pose
    gripper: float


class SimulationGuard:
    def __init__(self, episode: UUID) -> None:
        self.episode = episode
        self.mode = "STARTUP_HOLD"
        self.reason = "startup"
        self.generation = 0
        self.domain_ns = 0
        self._progress_ns = 0
        self._steady_ns = 0
        self._sequence = -1
        self._output_tick = -1
        self._output_progress_ns = 0
        self._authorization: Authorization | None = None
        self._reset_required = False
        self.gripper = 0.0

    def _steady(self, now: int) -> None:
        if type(now) is not int or now < self._steady_ns:
            self.fault("steady_reversed")
            raise ValueError("steady clock must be monotonic integer nanoseconds")
        self._steady_ns = now

    def fault(self, reason: str) -> None:
        if self.mode in {"ENABLED", "RECOVERING"}:
            self.generation += 1
        self.mode = "FAULT_HOLD"
        self.reason = reason
        self._authorization = None

    def clock(self, domain_ns: int, episode: UUID, steady_ns: int) -> None:
        self._steady(steady_ns)
        if type(domain_ns) is not int or domain_ns < 0:
            self.fault("invalid_clock")
            return
        if episode != self.episode:
            self.fault("episode_reset")
            self.episode = episode
            self._sequence = -1
            self._output_tick = -1
            self._reset_required = False
            self.domain_ns = domain_ns
            self._progress_ns = steady_ns
            return
        if domain_ns < self.domain_ns:
            self._reset_required = True
            self.fault("clock_reversed")
            return
        if steady_ns - self._progress_ns >= WATCHDOG_NS:
            self.fault("clock_paused")
        if domain_ns > self.domain_ns:
            self._progress_ns = steady_ns
        self.domain_ns = domain_ns

    def authorize(self, value: Authorization, steady_ns: int) -> bool:
        self._steady(steady_ns)
        reason = None
        if self._reset_required:
            reason = "new_episode_required"
        elif value.episode != self.episode:
            reason = "wrong_episode"
        elif type(value.sequence) is not int or not 0 <= value.sequence < 2**64:
            reason = "invalid_sequence"
        elif value.sequence <= self._sequence:
            reason = "replay"
        else:
            self._sequence = value.sequence
            if (
                type(value.source_ns) is not int
                or not 0 <= value.source_ns <= self.domain_ns
            ):
                reason = "future_or_invalid_source"
            elif self.domain_ns - value.source_ns >= WATCHDOG_NS:
                reason = "stale_source"
            elif (
                type(value.receipt_ns) is not int
                or not 0 <= value.receipt_ns <= steady_ns
            ):
                reason = "invalid_receipt"
            elif steady_ns - value.receipt_ns >= WATCHDOG_NS:
                reason = "receipt_timeout"
            elif steady_ns - self._progress_ns >= WATCHDOG_NS:
                reason = "clock_paused"
            elif value.pose.frame_id != "world":
                reason = "invalid_frame"
            elif (
                len(value.pose.position_xyz_m) != 3
                or len(value.pose.quaternion_wxyz) != 4
            ):
                reason = "invalid_shape"
            elif not all(
                isinstance(v, (int, float))
                and not isinstance(v, bool)
                and math.isfinite(v)
                for v in (
                    *value.pose.position_xyz_m,
                    *value.pose.quaternion_wxyz,
                    value.gripper,
                )
            ):
                reason = "invalid_numeric"
            elif (
                abs(math.sqrt(sum(v * v for v in value.pose.quaternion_wxyz)) - 1.0)
                > 1e-3
            ):
                reason = "invalid_quaternion"
            elif not 0.0 <= value.gripper <= 1.0:
                reason = "invalid_gripper"
        if reason:
            self.fault(reason)
            return False
        self._authorization = value
        if self.mode != "ENABLED":
            self.generation += 1
            self.mode = "RECOVERING"
            self._output_tick = -1
        self.reason = "authorization_received"
        return True

    def reactivated(self, generation: int, steady_ns: int) -> bool:
        self._steady(steady_ns)
        if self.mode != "RECOVERING" or generation != self.generation:
            return False
        self.mode = "ENABLED"
        self._output_progress_ns = steady_ns
        self.reason = "authorized"
        return self.allowed(steady_ns)

    def allowed(self, steady_ns: int) -> bool:
        self._steady(steady_ns)
        value = self._authorization
        if self.mode != "ENABLED" or value is None:
            return False
        for expired, reason in (
            (self.domain_ns - value.source_ns >= WATCHDOG_NS, "stale_source"),
            (steady_ns - value.receipt_ns >= WATCHDOG_NS, "receipt_timeout"),
            (steady_ns - self._progress_ns >= WATCHDOG_NS, "clock_paused"),
            (steady_ns - self._output_progress_ns >= WATCHDOG_NS, "controller_timeout"),
        ):
            if expired:
                self.fault(reason)
                return False
        return True

    def output(
        self,
        episode: UUID,
        generation: int,
        tick: int,
        effort: tuple[float, ...],
        steady_ns: int,
    ) -> bool:
        self._steady(steady_ns)
        if not self.allowed(steady_ns):
            return False
        if episode != self.episode or generation != self.generation:
            self.fault("old_output_generation")
            return False
        if type(tick) is not int or tick <= self._output_tick:
            self.fault("output_replay")
            return False
        if len(effort) != 6 or not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
            for v in effort
        ):
            self.fault("invalid_effort")
            return False
        self._output_tick = tick
        self._output_progress_ns = steady_ns
        assert self._authorization is not None
        self.gripper = self._authorization.gripper
        return True
