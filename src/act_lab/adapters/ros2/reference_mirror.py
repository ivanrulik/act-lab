"""Bounded diagnostic mirror; authoritative capture still waits for DDS ACK."""

from __future__ import annotations

import json
import os
import queue
import shutil
import tempfile
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal, cast

from act_lab.adapters.mcap.recording import McapEpisodeSink, publish, sync_directory
from act_lab.domain.ports import EpisodeSink
from act_lab.domain.recording import EpisodeOutcome, EpisodeProvenance, EpisodeSample


class ComparisonMcapSink(McapEpisodeSink):
    """Flush comparison records; commit on finalization or orderly interruption.

    The rosbag MCAP is authoritative. This duplicate artifact has no crash-prefix
    durability promise; successful finalization still requires fsync and atomic
    publication. Ordinary canonical acquisition sinks retain per-operation fsync.
    """

    def __init__(
        self,
        directory: Path,
        episode_id: str | None = None,
        *,
        staging_directory: Path | None = None,
    ) -> None:
        self.output_directory = directory
        self.staging = (
            tempfile.TemporaryDirectory(prefix="comparison-", dir=staging_directory)
            if staging_directory is not None
            else None
        )
        super().__init__(
            Path(self.staging.name) if self.staging is not None else directory,
            episode_id,
        )
        self.finalizing = False
        self.published = False

    def _copy(self, completed: bool) -> None:
        if self.staging is None or self.published:
            return
        source = self.final_path if completed else self.partial_path
        if not source.exists():
            return
        self.output_directory.mkdir(parents=True, exist_ok=True)
        partial = self.output_directory / self.partial_path.name
        with source.open("rb") as reader, partial.open("xb") as writer:
            shutil.copyfileobj(reader, writer, length=1024 * 1024)
            writer.flush()
            os.fsync(writer.fileno())
        sync_directory(self.output_directory)
        if completed:
            publish(partial, self.output_directory / self.final_path.name)
        self.published = True
        self.staging.cleanup()

    def _sync(self) -> None:
        if self.finalizing:
            super()._sync()
        else:
            assert self._stream is not None
            self._stream.flush()

    def stop(self, outcome: EpisodeOutcome, reason: str) -> None:
        self.finalizing = True
        try:
            super().stop(outcome, reason)
        finally:
            self.finalizing = False
        self._copy(completed=True)

    def interrupt(self) -> None:
        try:
            if (
                not self.finalizing
                and self._stream is not None
                and not self._stream.closed
            ):
                super()._sync()
        finally:
            super().interrupt()
        if not self.finalizing:
            self._copy(completed=False)


@dataclass
class _Write:
    method: Literal[
        "start", "append", "event", "diagnostics", "stop", "discard", "interrupt"
    ]
    args: tuple[Any, ...]
    queued_ns: int = field(default_factory=time.monotonic_ns)
    done: threading.Event = field(default_factory=threading.Event)
    result: object = None


class ReferenceMirror:
    """One writer owns the reference file and its ordered operations.

    Immutable samples may wait in at most four reference slots. Queue overload
    or a write failure interrupts acquisition; finalization waits for every
    reference operation. This adapter is only for the duplicate comparison
    artifact, not the authoritative DDS/rosbag acceptance path.
    """

    def __init__(
        self,
        factory: Callable[[], EpisodeSink],
        *,
        on_error: Callable[[str], None],
        trace: Path,
    ) -> None:
        self.factory = factory
        self.on_error = on_error
        self.trace = trace
        self.queue: queue.Queue[_Write] = queue.Queue(maxsize=16)
        self.slots = threading.BoundedSemaphore(4)
        self.lock = threading.Lock()
        self.pending = 0
        self.metrics = dict(
            peak_pending_samples=0,
            operations=0,
            maximum_queue_ns=0,
            maximum_operation_ns=0,
        )
        self.error: BaseException | None = None
        self.closed = False
        self.thread = threading.Thread(
            target=self._run, daemon=True, name="reference-mirror"
        )
        self.thread.start()

    @property
    def stats(self) -> dict[str, int]:
        with self.lock:
            return dict(self.metrics)

    def _check(self) -> None:
        if self.error is not None:
            raise RuntimeError(
                f"reference_mirror_failure: {self.error}"
            ) from self.error
        if self.closed:
            raise RuntimeError("reference mirror is closed")

    def _submit(
        self, write: _Write, *, wait: bool = False, wait_s: float = 2.0
    ) -> _Write:
        self._check()
        sample = write.method == "append"
        if sample:
            if not self.slots.acquire(blocking=False):
                raise RuntimeError("reference_mirror_backpressure")
            with self.lock:
                self.pending += 1
                self.metrics["peak_pending_samples"] = max(
                    self.pending, self.metrics["peak_pending_samples"]
                )
        try:
            if wait:
                self.queue.put(write, timeout=wait_s)
            else:
                self.queue.put_nowait(write)
        except queue.Full as error:
            if sample:
                self.slots.release()
                with self.lock:
                    self.pending -= 1
            raise RuntimeError("reference_mirror_backpressure") from error
        if wait and not write.done.wait(wait_s):
            self._check()
            raise RuntimeError("reference_mirror_drain_timeout")
        self._check()
        return write

    def start(self, provenance: EpisodeProvenance, timestamp_ns: int) -> str:
        write = self._submit(_Write("start", (provenance, timestamp_ns)), wait=True)
        return cast(str, write.result)

    def append(self, sample: EpisodeSample) -> None:
        self._submit(_Write("append", (sample,)))

    def event(
        self, timestamp_ns: int, kind: str, outcome: EpisodeOutcome, reason: str
    ) -> None:
        self._submit(_Write("event", (timestamp_ns, kind, outcome, reason)))

    def diagnostics(
        self,
        timestamp_ns: int,
        host_monotonic_ns: int,
        values_json: str,
        calibration_json: str,
    ) -> None:
        self._submit(
            _Write(
                "diagnostics",
                (timestamp_ns, host_monotonic_ns, values_json, calibration_json),
            )
        )

    def stop(self, outcome: EpisodeOutcome, reason: str) -> None:
        self._submit(_Write("stop", (outcome, reason)), wait=True, wait_s=10)
        self.thread.join(2)
        self.closed = True

    def discard(self, reason: str) -> None:
        self._submit(_Write("discard", (reason,)), wait=True, wait_s=10)
        self.thread.join(2)
        self.closed = True

    def interrupt(self) -> None:
        if self.closed:
            return
        if self.thread.is_alive() and self.error is None:
            try:
                self._submit(_Write("interrupt", ()), wait=True, wait_s=10)
            except RuntimeError:
                pass  # The original failure and explicitly partial file remain.
        self.thread.join(2)
        self.closed = True

    def _run(self) -> None:
        sink: EpisodeSink | None = None
        write: _Write | None = None
        try:
            sink = self.factory()
            with self.trace.open("w", buffering=1024 * 1024) as trace:
                while True:
                    write = self.queue.get()
                    started = time.monotonic_ns()
                    write.result = getattr(sink, write.method)(*write.args)
                    finished = time.monotonic_ns()
                    queued = started - write.queued_ns
                    elapsed = finished - started
                    with self.lock:
                        self.metrics["operations"] += 1
                        self.metrics["maximum_queue_ns"] = max(
                            self.metrics["maximum_queue_ns"], queued
                        )
                        self.metrics["maximum_operation_ns"] = max(
                            self.metrics["maximum_operation_ns"], elapsed
                        )
                        if write.method == "append":
                            self.pending -= 1
                            self.slots.release()
                    trace.write(
                        json.dumps(
                            dict(
                                method=write.method,
                                queue_ns=queued,
                                operation_ns=elapsed,
                                steady_ns=finished,
                            )
                        )
                        + "\n"
                    )
                    write.done.set()
                    if write.method in {"stop", "discard", "interrupt"}:
                        break
        except BaseException as error:
            self.error = error
            if write is not None:
                write.done.set()
            try:
                if sink is not None:
                    sink.interrupt()
            finally:
                try:
                    self.on_error(f"reference_mirror_failure: {error}")
                except (OSError, EOFError):
                    pass
