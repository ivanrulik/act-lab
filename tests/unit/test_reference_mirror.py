"""Reference durability must not block primary capture or grow without bound."""

import threading

import pytest

from act_lab.adapters.ros2.reference_mirror import ReferenceMirror
from act_lab.domain.recording import EpisodeOutcome


class Sink:
    def __init__(self):
        self.calls = []
        self.entered = threading.Event()
        self.release = threading.Event()
        self.fail = False

    def start(self, provenance, stamp):
        self.calls.append(("start", stamp))
        return "episode"

    def append(self, sample):
        self.entered.set()
        assert self.release.wait(2)
        if self.fail:
            raise OSError("disk failure")
        self.calls.append(("sample", sample))

    def event(self, stamp, kind, outcome, reason):
        self.calls.append(("event", stamp))

    def diagnostics(self, stamp, host, values, calibration):
        self.calls.append(("diagnostics", stamp))

    def stop(self, outcome, reason):
        self.calls.append(("stop", reason))

    def interrupt(self):
        self.calls.append(("interrupt",))


def test_disabled_commit_can_finish_after_startup_wait_bound(tmp_path):
    class SlowCommit(Sink):
        def stop(self, outcome, reason):
            assert self.release.wait(5)
            super().stop(outcome, reason)

    sink = SlowCommit()
    errors = []
    mirror = ReferenceMirror(
        lambda: sink, on_error=errors.append, trace=tmp_path / "trace.jsonl"
    )
    mirror.start(None, 0)
    timer = threading.Timer(2.2, sink.release.set)
    timer.start()
    try:
        mirror.stop(EpisodeOutcome.SUCCESS, "disabled commit")
    finally:
        sink.release.set()
        timer.join()
        mirror.interrupt()
    assert not errors
    assert sink.calls[-1] == ("stop", "disabled commit")


def test_slow_durable_reference_preserves_order_and_bounded_samples(tmp_path):
    sink = Sink()
    errors = []
    mirror = ReferenceMirror(
        lambda: sink, on_error=errors.append, trace=tmp_path / "reference.jsonl"
    )
    try:
        assert mirror.start(None, 0) == "episode"
        mirror.append("original sample")
        assert sink.entered.wait(1)
        # These submits complete while the actual writer is blocked. Every
        # immutable source sample and its original timestamp remain queued.
        for sample in ("two", "three", "four"):
            mirror.append(sample)
        mirror.event(4, "task", EpisodeOutcome.FAILURE, "incomplete")
        with pytest.raises(RuntimeError, match="backpressure"):
            mirror.append("five")
        assert mirror.stats["peak_pending_samples"] == 4
        sink.release.set()
        mirror.stop(EpisodeOutcome.FAILURE, "incomplete")
        assert sink.calls == [
            ("start", 0),
            ("sample", "original sample"),
            ("sample", "two"),
            ("sample", "three"),
            ("sample", "four"),
            ("event", 4),
            ("stop", "incomplete"),
        ]
        assert not errors
        assert not mirror.thread.is_alive()
        assert mirror.stats["operations"] == 7
    finally:
        sink.release.set()
        mirror.interrupt()


def test_reference_write_failure_is_reported_and_cannot_finalize(tmp_path):
    sink = Sink()
    sink.fail = True
    errors = []
    failed = threading.Event()

    def failure(error):
        errors.append(error)
        failed.set()

    mirror = ReferenceMirror(
        lambda: sink, on_error=failure, trace=tmp_path / "reference.jsonl"
    )
    try:
        mirror.start(None, 0)
        mirror.append("original")
        assert sink.entered.wait(1)
        sink.release.set()
        assert failed.wait(1)
        with pytest.raises(RuntimeError, match="disk failure"):
            mirror.stop(EpisodeOutcome.SUCCESS, "success")
        assert errors == ["reference_mirror_failure: disk failure"]
        assert ("interrupt",) in sink.calls
        assert not any(call[0] == "stop" for call in sink.calls)
    finally:
        sink.release.set()
        mirror.interrupt()
