from dataclasses import replace
from uuid import UUID

import pytest

from act_lab.adapters.ros2.simulation_guard import Authorization, SimulationGuard
from act_lab.domain import Pose

EPISODE = UUID(int=1)
POSE = Pose("world", (0.45, -0.15, 0.65), (1.0, 0.0, 0.0, 0.0))


def active():
    guard = SimulationGuard(EPISODE)
    value = Authorization(EPISODE, 1, 0, 0, POSE, 0.4)
    assert guard.authorize(value, 0)
    assert guard.mode == "RECOVERING"
    assert guard.reactivated(guard.generation, 0)
    assert guard.output(EPISODE, guard.generation, 0, (0.0,) * 6, 0)
    return guard, value


@pytest.mark.parametrize("lease", ["source", "receipt", "clock", "output"])
def test_exact_independent_lease_boundaries(lease):
    guard, value = active()
    before = 99_999_999
    if lease != "clock":
        guard.clock(1 if lease != "source" else before, EPISODE, before)
    if lease in {"clock", "output"}:
        assert guard.authorize(
            replace(value, sequence=2, source_ns=guard.domain_ns, receipt_ns=before),
            before,
        )
    if lease != "output":
        assert guard.output(EPISODE, guard.generation, 1, (0.0,) * 6, before)
    assert guard.allowed(before)
    if lease == "source":
        guard.clock(100_000_000, EPISODE, 100_000_000)
    assert not guard.allowed(100_000_000)
    assert (
        guard.reason
        == dict(
            source="stale_source",
            receipt="receipt_timeout",
            clock="clock_paused",
            output="controller_timeout",
        )[lease]
    )
    assert guard.gripper == 0.4


def test_reset_pause_generation_and_recovery():
    guard, value = active()
    old = guard.generation
    guard.clock(1, EPISODE, 100_000_000)
    assert not guard.allowed(100_000_000)
    guard.clock(2, EPISODE, 100_000_001)
    assert guard.authorize(
        replace(value, sequence=2, source_ns=2, receipt_ns=100_000_001), 100_000_001
    )
    assert not guard.reactivated(old, 100_000_001)
    assert guard.reactivated(guard.generation, 100_000_001)
    guard.clock(0, EPISODE, 100_000_002)
    assert not guard.authorize(
        replace(value, sequence=3, receipt_ns=100_000_002), 100_000_002
    )
    assert guard.reason == "new_episode_required"
    new = UUID(int=2)
    guard.clock(0, new, 100_000_003)
    assert guard.authorize(
        replace(value, episode=new, receipt_ns=100_000_003), 100_000_003
    )


@pytest.mark.parametrize(
    "change",
    [
        dict(sequence=1),
        dict(episode=UUID(int=2)),
        dict(source_ns=1),
        dict(pose=replace(POSE, frame_id="camera")),
        dict(pose=replace(POSE, quaternion_wxyz=(2.0, 0.0, 0.0, 0.0))),
        dict(gripper=float("nan")),
    ],
)
def test_invalid_authorization_clears_active_intent(change):
    guard, value = active()
    assert not guard.authorize(
        replace(value, sequence=2, **change)
        if "sequence" not in change
        else replace(value, **change),
        0,
    )
    assert not guard.allowed(0)
    assert guard.gripper == 0.4


def test_old_outputs_and_output_replay_cannot_restore_motion():
    guard, _ = active()
    assert not guard.output(EPISODE, guard.generation, 0, (0.0,) * 6, 1)
    assert guard.reason == "output_replay"
    assert not guard.output(EPISODE, guard.generation - 1, 1, (0.0,) * 6, 2)


@pytest.mark.parametrize("effort", [(float("nan"),) * 6, (None,) * 6, (0.0,) * 5])
def test_malformed_controller_output_faults_without_throwing(effort):
    guard, _ = active()
    assert not guard.output(EPISODE, guard.generation, 1, effort, 1)
    assert guard.reason == "invalid_effort"
    assert guard.gripper == 0.4


def test_invalid_clock_cannot_clear_reset_latch():
    guard, value = active()
    guard.clock(1, EPISODE, 1)
    guard.clock(0, EPISODE, 2)
    guard.clock(-1, UUID(int=2), 3)
    assert guard.episode == EPISODE
    assert not guard.authorize(replace(value, sequence=2, receipt_ns=3), 3)
    assert guard.reason == "new_episode_required"


def test_forward_jump_ages_source_and_fresh_sequence_can_recover_same_episode():
    guard, value = active()
    guard.clock(100_000_000, EPISODE, 1)
    assert not guard.allowed(1)
    assert guard.reason == "stale_source"
    assert guard.authorize(
        replace(value, sequence=2, source_ns=100_000_000, receipt_ns=1), 1
    )
    assert guard.reactivated(guard.generation, 1)
