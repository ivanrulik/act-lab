"""Latest intent and independent steady-clock guards; no ROS or driver access."""

from __future__ import annotations

from threading import Lock

from act_lab.adapters.ros2.contracts import (
    CLOCK_OFFSET_NS,
    WATCHDOG_NS,
    CommandEnvelope,
    integer,
    validate_episode,
)
from act_lab.domain import Action, Observation


class CommandInbox:
    def __init__(self, *, use_sim_time: bool = True) -> None:
        if not use_sim_time:
            raise ValueError("ROS contracts require use_sim_time=true")
        self._lock = Lock()
        self._episode: str | None = None
        self._sequence = -1
        self._command: CommandEnvelope | None = None
        self._receipt_ns: int | None = None
        self._ros_ns: int | None = None
        self._progress_ns: int | None = None
        self._last_steady_ns = 0
        self._requires_epoch = False
        self.last_rejection_reason: str | None = "clock_not_ready"

    @property
    def episode_id(self) -> str | None:
        with self._lock:
            return self._episode

    def _clear(self, reason: str) -> None:
        self._command = None
        self._receipt_ns = None
        self.last_rejection_reason = reason

    def _steady(self, now_ns: int) -> None:
        integer(now_ns, 0, 2**63 - 1)
        if now_ns < self._last_steady_ns:
            self._clear("steady_clock_reversed")
            self._requires_epoch = True
            raise ValueError("steady clock must not go backwards")
        self._last_steady_ns = now_ns

    def update_clock(
        self, ros_now_ns: int, episode_id: str, steady_now_ns: int
    ) -> None:
        """Only the trusted simulation/state authority may call this method."""
        with self._lock:
            self._steady(steady_now_ns)
            validate_episode(episode_id)
            integer(ros_now_ns, 0, 2**63 - 1)
            if episode_id != self._episode:
                self._episode = episode_id
                self._sequence = -1
                self._ros_ns = None
                self._progress_ns = None
                self._requires_epoch = False
                self._clear("episode_reset")
            if ros_now_ns < CLOCK_OFFSET_NS:
                self._clear("clock_not_ready")
                self._requires_epoch = self._ros_ns is not None
                return
            if self._requires_epoch:
                self._clear("new_episode_required")
                return
            if self._ros_ns is not None and ros_now_ns < self._ros_ns:
                self._clear("clock_reversed")
                self._requires_epoch = True
                return
            if (
                self._progress_ns is not None
                and steady_now_ns - self._progress_ns >= WATCHDOG_NS
            ):
                self._clear("clock_paused")
            if self._ros_ns is None or ros_now_ns > self._ros_ns:
                self._progress_ns = steady_now_ns
            self._ros_ns = ros_now_ns

    def reject(self, reason: str = "malformed_envelope") -> None:
        with self._lock:
            self._clear(reason)

    def offer(self, envelope: CommandEnvelope, receipt_steady_ns: int) -> bool:
        with self._lock:
            self._steady(receipt_steady_ns)
            try:
                envelope.validate()
            except (ValueError, TypeError, AttributeError, OverflowError):
                self._clear("malformed_envelope")
                return False
            if self._ros_ns is None or self._requires_epoch:
                self._clear("clock_not_ready")
                return False
            if envelope.episode_id != self._episode:
                self._clear("wrong_episode")
                return False
            if envelope.sequence <= self._sequence:
                self._clear("out_of_order")
                return False
            self._sequence = envelope.sequence
            if envelope.action.timestamp_ns < 0:
                self._clear("pre_episode_timestamp")
                return False
            self._command = envelope
            self._receipt_ns = receipt_steady_ns
            self.last_rejection_reason = None
            return True

    def poll(self, observation: Observation, steady_now_ns: int) -> Action:
        with self._lock:
            self._steady(steady_now_ns)
            hold = Action(
                observation.timestamp_ns,
                observation.robot.end_effector_pose,
                observation.robot.gripper_position,
                False,
            )
            if (
                self._requires_epoch
                or self._ros_ns is None
                or self._progress_ns is None
            ):
                return hold
            if self._ros_ns != observation.timestamp_ns + CLOCK_OFFSET_NS:
                self._clear("clock_state_mismatch")
                return hold
            if steady_now_ns - self._progress_ns >= WATCHDOG_NS:
                self._clear("clock_paused")
                return hold
            if self._command is None or self._receipt_ns is None:
                return hold
            action = self._command.action
            if action.timestamp_ns > observation.timestamp_ns:
                # Deliver original timestamp once for the application's INVALID
                # report, then discard it so it cannot become valid later.
                self._clear("future_timestamp")
                return action
            if observation.timestamp_ns - action.timestamp_ns >= WATCHDOG_NS:
                self.last_rejection_reason = "stale_command"
                return action  # Safety reports STALE; source stamp is untouched.
            if steady_now_ns - self._receipt_ns >= WATCHDOG_NS:
                self._clear("receipt_timeout")
                return hold
            return action
