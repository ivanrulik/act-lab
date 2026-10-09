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


@pytest.mark.parametrize(
    "reason", ["invalid_frame", "invalid_numeric", "controller_lost"]
)
def test_nominal_recovery_does_not_retry_other_rejections(reason):
    session = session_with(dict(mode="FAULT_HOLD", reason=reason))
    with pytest.raises(RuntimeError, match="nominal motion rejected"):
        session.motion_command(Pose("world", (0.4, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0)))
    assert session.command.call_count == 1


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
