"""ROS-free, bounded observational handoff. No authority crosses back to control."""

from __future__ import annotations

import json
import math
import multiprocessing as mp
from collections import deque
from typing import Any

from act_lab.adapters.ros2.contracts import encode_pose, encode_stamp, encode_state
from act_lab.domain import RobotState

MAX_OBSERVATION_BYTES = 65536
STALE_NS = 100_000_000


class ObservationChannel:
    """One writer, one reader; a busy or dead reader never blocks the owner."""

    def __init__(self) -> None:
        context = mp.get_context("spawn")
        self.buffer = context.RawArray("B", MAX_OBSERVATION_BYTES)
        self.length = context.RawValue("i", 0)
        self.lock = context.Lock()
        self.dropped = context.RawValue("Q", 0)

    def offer(self, packet: dict[str, Any]) -> bool:
        data = json.dumps(packet, allow_nan=False, separators=(",", ":")).encode()
        if len(data) > MAX_OBSERVATION_BYTES or not self.lock.acquire(False):
            self.dropped.value += 1
            return False
        try:
            self.buffer[: len(data)] = data
            self.length.value = len(data)
        finally:
            self.lock.release()
        return True

    def read(self) -> dict[str, Any] | None:
        if not self.lock.acquire(False):
            return None
        try:
            data = bytes(self.buffer[: self.length.value])
        finally:
            self.lock.release()
        return json.loads(data) if data else None


class ObserverFreshness:
    def __init__(self) -> None:
        self.received_ns: int | None = None
        self.identity: tuple[Any, ...] | None = None
        self.episode: str | None = None

    def receive(self, packet: dict[str, Any], now_ns: int) -> bool:
        value = packet["telemetry"]
        identity = (value["episode_id"], packet["sample_id"])
        if identity == self.identity:
            return False
        reset = bool(self.episode != value["episode_id"])
        self.identity = identity
        self.episode = value["episode_id"]
        self.received_ns = now_ns
        return reset

    def stale(self, now_ns: int) -> bool:
        return self.received_ns is None or now_ns - self.received_ns >= STALE_NS


class ObservationCapture:
    def __init__(self, channel: ObservationChannel) -> None:
        self.channel = channel
        self.events: deque[dict[str, Any]] = deque(maxlen=64)
        self.sample_id = 0
        self.event_id = 0
        self.events_dropped = 0
        self.transition: tuple[Any, ...] | None = None
        self.intent: dict[str, Any] | None = None
        self.progress_wall_ns = 0
        self.domain_ns = -1

    def capture(
        self,
        state: RobotState,
        episode: str,
        tick: int,
        mode: str,
        reason: str,
        generation: int,
        steady_ns: int,
        guard: Any,
        sample: Any,
        raw: Any,
        wall_ns: int,
        output_wall_ns: int,
        observation: dict[str, Any] | None = None,
        residual_m: float = 0.0,
        residual_rad: float = 0.0,
    ) -> None:
        if observation is not None:
            self.intent = observation
        if self.transition and self.transition[0] != episode:
            self.intent = None
            self.events.clear()
        if self.domain_ns != state.timestamp_ns:
            self.domain_ns = state.timestamp_ns
            self.progress_wall_ns = wall_ns
        header = dict(stamp=encode_stamp(state.timestamp_ns), frame_id="world")
        source = self.intent
        raw_meta = raw if isinstance(raw, dict) else {}
        raw = raw_meta.get("values") if raw_meta else raw
        raw_valid = (
            isinstance(raw, (list, tuple))
            and len(raw) == 6
            and all(isinstance(v, (int, float)) and math.isfinite(v) for v in raw)
        )
        auth = guard._authorization
        fields = dict(
            header=header,
            schema_version=1,
            episode_id=episode,
            physics_tick=tick,
            generation=generation,
            mode=mode,
            reason=reason,
            source_valid=source is not None,
            source_episode_id=source.get("source_episode_id", episode)
            if source
            else "",
            command_sequence=source["sequence"] if source else 0,
            source_timestamp_ns=source["source_ns"] if source else 0,
            source_age_ns=state.timestamp_ns - source["source_ns"] if source else 0,
            authorization_valid=auth is not None,
            injected_receipt_age_ns=steady_ns - auth.receipt_ns if auth else 0,
            injected_clock_age_ns=steady_ns - guard._progress_ns,
            injected_output_age_ns=steady_ns - guard._output_progress_ns,
            wall_receipt_age_ns=max(0, wall_ns - source["wall_receipt_ns"])
            if source
            else 0,
            wall_clock_age_ns=max(0, wall_ns - self.progress_wall_ns),
            wall_output_age_ns=max(0, wall_ns - output_wall_ns),
            raw_effort_valid=raw_valid,
            raw_output_episode_id=raw_meta.get("episode", ""),
            raw_output_generation=raw_meta.get("generation", 0),
            raw_output_tick=raw_meta.get("tick", 0),
            raw_effort_detail="" if raw_valid else str(raw),
            raw_effort_nm=list(raw) if raw_valid else [0.0] * 6,
            task_effort_nm=list(sample.task_effort_nm),
            hold_effort_nm=list(sample.servo_effort_nm),
            total_effort_nm=list(sample.total_effort_nm),
            accepted_gripper=sample.accepted_gripper,
            frame_residual_m=residual_m,
            frame_residual_rad=residual_rad,
            handoff_drops=self.channel.dropped.value,
            event_drops=self.events_dropped,
        )
        transition = (episode, mode, reason, generation)
        changed = transition != self.transition
        if changed:
            self.transition = transition
            self.event_id += 1
            if len(self.events) == self.events.maxlen:
                self.events_dropped += 1
            self.events.append(
                dict(
                    header=header,
                    schema_version=1,
                    episode_id=episode,
                    event_id=self.event_id,
                    physics_tick=tick,
                    generation=generation,
                    mode=mode,
                    reason=reason,
                    source_valid=source is not None,
                    source_episode_id=fields["source_episode_id"],
                    command_sequence=fields["command_sequence"],
                    source_timestamp_ns=fields["source_timestamp_ns"],
                )
            )
        fields["event_drops"] = self.events_dropped
        if tick % 10 == 0 or changed:
            self.sample_id += 1
            self.channel.offer(
                dict(
                    schema_version=1,
                    sample_id=self.sample_id,
                    telemetry=fields,
                    state=encode_state(state, episode),
                    events=list(self.events),
                    measured_pose=encode_pose(state.end_effector_pose),
                    requested=source.get("requested") if source else None,
                    requested_frame=source.get("requested_frame", "world")
                    if source
                    else "",
                    approved=source.get("approved") if source else None,
                )
            )
