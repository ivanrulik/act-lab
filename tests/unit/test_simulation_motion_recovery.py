from unittest.mock import Mock

import pytest

from act_lab.adapters.ros2.simulation_smoke import MotionSession
from act_lab.domain import Pose


def session_with(*responses):
    session = MotionSession.__new__(MotionSession)
    session.trace = []

    def command(*args):
        value = responses[len(session.trace)]
        session.trace.append(dict(value))
        return value

    session.command = Mock(side_effect=command)
    session.rpc = Mock()
    return session


def test_nominal_recovery_keeps_fault_evidence_and_calls_new_command():
    session = session_with(
        dict(mode="FAULT_HOLD", reason="controller_wall_timeout"),
        dict(mode="ENABLED", reason="authorized"),
    )
    pose = Pose("world", (0.4, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0))
    assert session.motion_command(pose)["mode"] == "ENABLED"
    assert session.command.call_count == 2
    assert session.trace[0]["wall_fault_recovery"] is True
    session.rpc.assert_called_once_with(dict(kind="prepare_controller"))


@pytest.mark.parametrize(
    "reason", ["invalid_frame", "invalid_numeric", "controller_lost"]
)
def test_nominal_recovery_does_not_retry_other_rejections(reason):
    session = session_with(dict(mode="FAULT_HOLD", reason=reason))
    with pytest.raises(RuntimeError, match="nominal motion rejected"):
        session.motion_command(Pose("world", (0.4, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0)))
    assert session.command.call_count == 1
    session.rpc.assert_not_called()


def test_nominal_recovery_is_bounded(monkeypatch):
    times = iter((0.0, 15.0))
    monkeypatch.setattr(
        "act_lab.adapters.ros2.simulation_smoke.time.monotonic", lambda: next(times)
    )
    session = session_with(dict(mode="FAULT_HOLD", reason="clock_paused"))
    with pytest.raises(RuntimeError, match="bounded nominal"):
        session.motion_command(Pose("world", (0.4, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0)))


def test_disabled_intent_never_uses_recovery():
    session = session_with(dict(mode="DISABLED_HOLD", reason="disabled"))
    pose = Pose("world", (0.4, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0))
    assert session.motion_command(pose, 0.4, False)["mode"] == "DISABLED_HOLD"
    session.command.assert_called_once_with(pose, 0.4, False)


def test_nominal_recovery_records_transport_loss_before_fresh_intent():
    session = session_with(
        dict(
            mode="FAULT_HOLD", reason="disabled", transport_rejection="delivery_timeout"
        ),
        dict(mode="ENABLED", reason="authorized"),
    )
    pose = Pose("world", (0.4, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0))
    assert session.motion_command(pose)["mode"] == "ENABLED"
    assert session.trace[0]["transport_loss_recovery"] is True
    assert session.trace[0]["wall_fault_recovery"] is False


@pytest.mark.parametrize(
    "mode,reason,age_ns,expected",
    [
        ("FAULT_HOLD", "stale_source", 100_000_000, True),
        ("FAULT_HOLD", "gateway_wall_timeout", 500_000_000, True),
        ("FAULT_HOLD", "controller_wall_timeout", 500_000_000, True),
        ("ENABLED", "authorized", 500_000_000, False),
        ("FAULT_HOLD", "invalid_numeric", 500_000_000, False),
        ("FAULT_HOLD", "stale_source", 99_999_999, False),
    ],
)
def test_producer_loss_requires_aged_intent_and_a_lease_fault(
    monkeypatch, mode, reason, age_ns, expected
):
    from types import SimpleNamespace

    from act_lab.adapters.ros2.simulation_smoke import producer_loss

    session = MotionSession.__new__(MotionSession)
    session.value = dict(mode="ENABLED")
    session.sequence = 42
    session.trace = [dict(source_timestamp_ns=20_000_000)]
    session.rpc = Mock(return_value=dict(mode=mode, reason=reason))
    monkeypatch.setattr(
        "act_lab.adapters.ros2.simulation_smoke.snapshot_state",
        lambda value: SimpleNamespace(timestamp_ns=20_000_000 + age_ns),
    )
    if expected:
        result = producer_loss(session)
        assert result["reason"] == reason
        assert result["source_timestamp_ns"] == 20_000_000
        assert result["command_sequence"] == 42
    else:
        with pytest.raises(RuntimeError, match="did not inhibit"):
            producer_loss(session)
    session.rpc.assert_called_once_with(dict(kind="advance", ticks=250))
    assert session.sequence == 42
