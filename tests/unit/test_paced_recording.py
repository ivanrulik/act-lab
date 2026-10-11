"""ROS-free injected-clock tests for paced capture's lossless boundary."""

import pytest

from act_lab.adapters.ros2.paced_recording import CaptureLease


def test_completion_and_exact_watchdog_boundary():
    lease = CaptureLease()
    sequence = lease.offer("episode", 20_000_000, 100)
    lease.complete(sequence, 100_000_099)
    assert not lease.pending
    lease.offer("episode", 40_000_000, 100_000_099)
    with pytest.raises(RuntimeError, match="capture_timeout"):
        lease.complete(2, 200_000_099)
    assert len(lease.pending) == 1


def test_backpressure_is_fault_never_overwrite():
    lease = CaptureLease(capacity=2)
    lease.offer("episode", 20, 1)
    lease.offer("episode", 40, 2)
    with pytest.raises(RuntimeError, match="backpressure"):
        lease.offer("episode", 60, 3)
    assert tuple(lease.pending) == ((1, 1), (2, 2))
    with pytest.raises(RuntimeError, match="backpressure"):
        lease.complete(1, 4)


@pytest.mark.parametrize("stamp", [0, 20, 19])
def test_pause_or_backward_source_clock_faults(stamp):
    lease = CaptureLease()
    lease.offer("episode", 20, 1)
    with pytest.raises(RuntimeError, match="nonincreasing"):
        lease.offer("episode", stamp, 2)


def test_reset_requires_new_session_and_sequence():
    lease = CaptureLease()
    lease.offer("episode", 20, 1)
    with pytest.raises(RuntimeError, match="episode_changed"):
        lease.offer("new-episode", 0, 2)
    fresh = CaptureLease()
    assert fresh.offer("new-episode", 0, 2) == 1


def test_missing_worker_progress_expires_despite_no_new_samples():
    lease = CaptureLease()
    lease.offer("episode", 20, 100)
    lease.check(100_000_099)
    with pytest.raises(RuntimeError, match="timeout"):
        lease.check(100_000_100)


def test_completion_timestamp_survives_delayed_reply_processing():
    lease = CaptureLease()
    lease.offer("episode", 20, 10)
    lease.offer("episode", 40, 30)
    lease.complete(1, 20)
    lease.complete(2, 40)
    lease.check(200_000_000)
    assert not lease.pending


def test_replay_completion_and_steady_reset_fail():
    lease = CaptureLease()
    lease.offer("episode", 20, 10)
    with pytest.raises(RuntimeError, match="completion_order"):
        lease.complete(2, 11)
    fresh = CaptureLease()
    fresh.offer("episode", 20, 10)
    with pytest.raises(RuntimeError, match="clock_reversed"):
        fresh.check(9)


@pytest.mark.parametrize("capacity,timeout", [(0, 1), (1, 0), (-1, 10)])
def test_invalid_bounds(capacity, timeout):
    with pytest.raises(ValueError):
        CaptureLease(capacity, timeout)
