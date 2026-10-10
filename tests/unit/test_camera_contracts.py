"""Exact freshness/reset/replay boundaries without ROS or rendered pixels."""

from uuid import uuid4

import pytest

from act_lab.adapters.ros2.camera_contracts import CameraSource, can_publish_frame
from act_lab.adapters.ros2.observability import ObservationChannel


def packet():
    return dict(
        schema_version=1,
        model_sha256="a" * 64,
        episode=str(uuid4()),
        capture_sequence=1,
        physics_tick=20,
        domain_ns=40_000_000,
        capture_steady_ns=1,
        qpos=[0.0] * 3,
        qvel=[0.0] * 2,
    )


def test_exact_boundary_duplicate_and_pause():
    source = CameraSource("a" * 64, 3, 2)
    first = packet()
    assert source.stale(1)
    assert source.offer(first, 1)
    assert not source.offer(first, 99_999_999)
    assert not source.stale(100_000_000)
    assert source.stale(100_000_001)
    paused = dict(first, capture_sequence=2, capture_steady_ns=100_000_001)
    assert not source.offer(paused, 100_000_001)
    assert source.stale(100_000_001)
    fresh = dict(paused, domain_ns=60_000_000, physics_tick=30)
    assert source.offer(fresh, 100_000_001)
    assert not source.stale(100_000_001)


def test_reset_retires_old_episode_and_discards_inflight_frames():
    source = CameraSource("a" * 64, 3, 2)
    first = packet()
    assert source.offer(first, 1)
    second = dict(
        first,
        episode=str(uuid4()),
        capture_sequence=2,
        physics_tick=0,
        domain_ns=0,
        capture_steady_ns=2,
    )
    assert source.offer(second, 2)
    late = dict(first, capture_sequence=3, capture_steady_ns=3)
    assert not source.offer(late, 3)
    assert source.stale(3)
    assert not can_publish_frame(first, second, 3)
    assert can_publish_frame(second, second, 3)
    assert not can_publish_frame(second, second, 100_000_002)


@pytest.mark.parametrize(
    "changes",
    [
        dict(model_sha256="bad"),
        dict(episode=[]),
        dict(episode="bad"),
        dict(qpos=[float("nan")] * 3),
        dict(qvel=[0.0]),
        dict(domain_ns=True),
        dict(capture_steady_ns=2),
        dict(capture_sequence=-1),
    ],
)
def test_bad_source_does_not_authorize_freshness(changes):
    source = CameraSource("a" * 64, 3, 2)
    assert not source.offer(dict(packet(), **changes), 1)
    assert source.stale(1)


def test_separate_camera_handoff_is_bounded_and_nonblocking():
    channel = ObservationChannel(camera=True)
    assert channel.camera is not None
    assert channel.camera.offer(packet())
    assert channel.read() is None
    assert channel.camera.read()["physics_tick"] == 20
    channel.camera.lock.acquire()
    try:
        assert not channel.camera.offer(packet())
        assert channel.camera.dropped.value == 1
    finally:
        channel.camera.lock.release()
    assert not channel.camera.offer(dict(pixels="x" * 230400))
    assert channel.camera.dropped.value == 2
