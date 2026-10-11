"""Bounded paced acquisition; no renderer or storage operation runs in control."""

from __future__ import annotations

import hashlib
import json
import math
import multiprocessing as mp
import os
import time
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
from queue import Empty, Full
from typing import Any

from act_lab.domain.recording import EpisodeOutcome, EpisodeProvenance, EpisodeSample


@dataclass(frozen=True)
class Capture:
    sample: EpisodeSample
    qpos: tuple[float, ...]
    qvel: tuple[float, ...]
    episode: str
    command_sequence: int
    receipt_ns: int


class CaptureLease:
    """Injected-clock boundary for ordered, lossless, bounded acquisition."""

    def __init__(self, capacity: int = 4, timeout_ns: int = 100_000_000) -> None:
        if capacity <= 0 or timeout_ns <= 0:
            raise ValueError("positive capture bounds required")
        self.capacity = capacity
        self.timeout_ns = timeout_ns
        self.pending: deque[tuple[int, int]] = deque()
        self.sequence = 0
        self.episode: str | None = None
        self.last_stamp = -1
        self.reason: str | None = None
        self.last_now = -1

    def check(self, now_ns: int) -> None:
        if now_ns < self.last_now:
            self.reason = "capture_steady_clock_reversed"
        self.last_now = now_ns
        if self.pending and now_ns - self.pending[0][1] >= self.timeout_ns:
            self.reason = "capture_timeout"
        if self.reason:
            raise RuntimeError(self.reason)

    def offer(self, episode: str, stamp: int, now_ns: int) -> int:
        self.check(now_ns)
        if self.episode is not None and episode != self.episode:
            self.reason = "capture_episode_changed"
        elif stamp <= self.last_stamp:
            self.reason = "capture_nonincreasing_time"
        elif len(self.pending) >= self.capacity:
            self.reason = "capture_backpressure"
        self.check(now_ns)
        self.episode = episode
        self.last_stamp = stamp
        self.sequence += 1
        self.pending.append((self.sequence, now_ns))
        return self.sequence

    def complete(self, sequence: int, now_ns: int) -> None:
        # Worker completion can precede a later control-thread offer. Validate
        # its actual completion time, then check outstanding work after draining
        # replies; delayed receipt of an already-completed ACK grants no motion.
        if self.reason:
            raise RuntimeError(self.reason)
        if not self.pending or sequence != self.pending[0][0]:
            self.reason = "capture_completion_order"
            raise RuntimeError(self.reason)
        age = now_ns - self.pending[0][1]
        if not 0 <= age < self.timeout_ns:
            self.reason = "capture_timeout"
            raise RuntimeError(self.reason)
        self.pending.popleft()


def capture_worker(
    requests: Any,
    replies: Any,
    config_path: Path,
    directory: Path,
    episode: str,
    delay_s: float = 0.0,
    sequence_start: int = 0,
    capture_profile: str = "full",
) -> None:
    """Own a scratch simulator, ROS node and acknowledged sink in one process."""
    import mujoco  # type: ignore[import-untyped]
    import rclpy  # type: ignore[import-not-found]
    from rclpy.parameter import Parameter  # type: ignore[import-not-found]

    from act_lab.adapters.mcap.recording import McapEpisodeSink
    from act_lab.adapters.mujoco.environment import MujocoUR5eEnvironment
    from act_lab.adapters.ros2.camera_rendering import render_settings, verify_renderer
    from act_lab.adapters.ros2.recording import RosEpisodeSink
    from act_lab.domain import CameraFrame

    env = None
    sink = None
    node = None
    completed = 0
    ages: list[int] = []
    try:
        rclpy.init()
        node = rclpy.create_node(
            "act_lab_paced_acquisition",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        env = MujocoUR5eEnvironment.from_config_file(config_path)
        env.reset(0)
        if capture_profile == "reduced-effects":
            env._model.vis.quality.offsamples = 0

        def render(camera: str) -> bytes:
            if capture_profile == "full":
                return bytes(env.render(camera).tobytes())
            renderer = env._renderer
            assert renderer is not None
            renderer.update_scene(env._data, camera=camera)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
            return bytes(renderer.render().tobytes())

        # Warm the actual acquisition renderer without changing model quality.
        env.render(env._config.cameras[0])  # Create EGL before profile warmup.
        for camera in env._config.cameras:
            render(camera)
        identity = verify_renderer(render_settings())
        sink = RosEpisodeSink(
            node,
            lambda selected: McapEpisodeSink(directory, selected),
            episode=lambda: episode,
            sequence_start=sequence_start,
        )
        replies.put(dict(ready=True, renderer=identity))
        while True:
            kind, value = requests.get()
            if kind == "start":
                provenance, stamp, episode = value
                if capture_profile != "full":
                    resolved = json.loads(provenance.resolved_config_json)
                    for profile in resolved.get("scene_cameras", {}).values():
                        profile.pop("sha256")
                        profile["acquisition_rendering"] = dict(
                            profile=capture_profile,
                            shadows=False,
                            reflections=False,
                            offsamples=0,
                        )
                        profile["sha256"] = hashlib.sha256(
                            json.dumps(
                                profile,
                                sort_keys=True,
                                allow_nan=False,
                            ).encode()
                        ).hexdigest()
                    provenance = replace(
                        provenance,
                        resolved_config_json=json.dumps(
                            resolved,
                            sort_keys=True,
                        ),
                    )
                sink.start(provenance, stamp)
                sink.timeout_s = 0.1
                replies.put(dict(started=True))
            elif kind == "sample":
                sequence, capture = value
                if delay_s:
                    time.sleep(delay_s)
                now = time.monotonic_ns()
                if not 0 <= now - capture.receipt_ns < 100_000_000:
                    raise RuntimeError(
                        f"capture_timeout: queue_age_ns={now - capture.receipt_ns}"
                    )
                sample = capture.sample
                stamp = sample.observation.timestamp_ns
                if capture.episode != episode:
                    raise ValueError("capture episode mismatch")
                if stamp != sample.command.resulting_state.timestamp_ns:
                    raise ValueError("capture report/state timestamp mismatch")
                env._data.qpos[:] = capture.qpos
                env._data.qvel[:] = capture.qvel
                env._data.time = stamp / 1e9
                env._physics_ticks = stamp // 2_000_000
                mujoco.mj_forward(env._model, env._data)

                images = tuple(
                    (
                        camera,
                        CameraFrame(
                            stamp,
                            env._config.render_width,
                            env._config.render_height,
                            render(camera),
                        ),
                    )
                    for camera in sample.observation.image_keys
                )
                age = time.monotonic_ns() - capture.receipt_ns
                if age >= 100_000_000:
                    raise RuntimeError(f"capture_timeout: render_age_ns={age}")
                sink.command_sequence = lambda current=capture: current.command_sequence
                sink.append(replace(sample, images=images))
                age = time.monotonic_ns() - capture.receipt_ns
                if age >= 100_000_000:
                    raise RuntimeError(f"capture_timeout: ack_age_ns={age}")
                ages.append(age)
                completed += 1
                replies.put(dict(completed=sequence, completed_ns=time.monotonic_ns()))
            elif kind == "event":
                sink.event(*value)
            elif kind == "diagnostics":
                sink.diagnostics(*value)
            elif kind == "stop":
                sink.stop(*value)
                replies.put(
                    dict(
                        stopped=True,
                        samples=completed,
                        capture_ages_ns=ages,
                        worker_pid=os.getpid(),
                        renderer=identity,
                        packet_sequence=sink.sequence,
                        capture_profile=capture_profile,
                    )
                )
                break
            else:
                raise ValueError("unknown capture request")
    except BaseException as error:
        replies.put(dict(error=str(error)))
        if sink is not None:
            sink.interrupt()
    finally:
        if env is not None:
            env.close()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


class PacedEpisodeSink:
    """Nonblocking control offers; acknowledged worker progress has a 100 ms lease."""

    def __init__(
        self,
        driver: Any,
        directory: Path,
        capacity: int = 4,
        *,
        delay_s: float = 0.0,
        sequence_start: int = 0,
        capture_profile: str = "full",
    ) -> None:
        if not math.isfinite(delay_s) or not 0 <= delay_s <= 1:
            raise ValueError("capture test delay must be between zero and one second")
        if capture_profile not in {"full", "reduced-effects"}:
            raise ValueError("unknown capture profile")
        self.driver = driver
        self.directory = directory
        self.episode_id = str(driver.runtime.snapshot["episode"])
        self.lease = CaptureLease(capacity)
        context = mp.get_context("spawn")
        # Metadata requests also have a finite bound. No latest-value overwrite.
        self.requests = context.Queue(maxsize=capacity * 4)
        self.replies = context.Queue()
        self.process = context.Process(
            target=capture_worker,
            args=(
                self.requests,
                self.replies,
                driver.runtime.config_path,
                directory,
                self.episode_id,
                delay_s,
                sequence_start,
                capture_profile,
            ),
        )
        self.process.start()
        self.finished = False
        self.result: dict[str, Any] = {}
        try:
            self.renderer = self._wait("ready", 30)["renderer"]
        except BaseException:
            self.interrupt()
            raise

    def for_directory(self, directory: Path) -> PacedEpisodeSink:
        return self

    def _wait(self, field: str, timeout: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                reply = self.replies.get(timeout=0.01)
            except Empty:
                self.lease.check(time.monotonic_ns())
                if not self.process.is_alive():
                    raise RuntimeError("capture_worker_lost") from None
                continue
            self._reply(reply)
            if reply.get(field):
                return dict(reply)
        raise RuntimeError(f"capture {field} timed out")

    def _reply(self, reply: dict[str, Any]) -> None:
        if "error" in reply:
            self.lease.reason = str(reply["error"])
            raise RuntimeError(self.lease.reason)
        if "completed" in reply:
            self.lease.complete(reply["completed"], reply["completed_ns"])

    def check(self) -> None:
        while True:
            try:
                self._reply(self.replies.get_nowait())
            except Empty:
                break
        self.lease.check(time.monotonic_ns())
        if not self.process.is_alive() and not self.finished:
            raise RuntimeError("capture_worker_lost")

    def _offer(self, kind: str, value: Any) -> None:
        self.check()
        try:
            self.requests.put_nowait((kind, value))
        except Full as error:
            self.lease.reason = "capture_backpressure"
            raise RuntimeError(self.lease.reason) from error

    def start(self, provenance: EpisodeProvenance, timestamp_ns: int) -> str:
        self.episode_id = str(self.driver.runtime.snapshot["episode"])
        self._offer("start", (provenance, timestamp_ns, self.episode_id))
        self._wait("started", 15)
        return self.episode_id

    def append(self, sample: EpisodeSample) -> None:
        self.check()
        snapshot = self.driver.runtime.snapshot
        receipt = time.monotonic_ns()
        sequence = self.lease.offer(
            snapshot["episode"], sample.observation.timestamp_ns, receipt
        )
        capture = Capture(
            sample,
            tuple(snapshot["qpos"]),
            tuple(snapshot["qvel"]),
            snapshot["episode"],
            self.driver.sequence,
            receipt,
        )
        self._offer("sample", (sequence, capture))

    def event(
        self, timestamp_ns: int, kind: str, outcome: EpisodeOutcome, reason: str
    ) -> None:
        self._offer("event", (timestamp_ns, kind, outcome, reason))

    def diagnostics(
        self,
        timestamp_ns: int,
        host_monotonic_ns: int,
        values_json: str,
        calibration_json: str,
    ) -> None:
        self._offer(
            "diagnostics",
            (
                timestamp_ns,
                host_monotonic_ns,
                values_json,
                calibration_json,
            ),
        )

    def stop(self, outcome: EpisodeOutcome, reason: str) -> None:
        self._offer("stop", (outcome, reason))
        self.result = self._wait("stopped", 15)
        self.finished = True
        self.process.join(5)
        if self.process.is_alive():
            self.interrupt()
            raise RuntimeError("capture shutdown timed out")
        self.requests.close()
        self.replies.close()

    def interrupt(self) -> None:
        self.directory.mkdir(parents=True, exist_ok=True)
        (self.directory / "capture-failure.json").write_text(
            json.dumps(
                dict(
                    status="interrupted",
                    reason=self.lease.reason or "worker_interrupted",
                    episode_id=self.episode_id,
                    pending=list(self.lease.pending),
                ),
                indent=2,
                sort_keys=True,
            )
            + "\n"
        )
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(5)
            if self.process.is_alive():
                self.process.kill()
                self.process.join(5)
        self.finished = True
        self.requests.cancel_join_thread()
        self.requests.close()
        self.replies.close()
