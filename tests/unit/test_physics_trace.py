import json

import pytest

from act_lab.adapters.ros2.physics_trace import PhysicsTrace


def test_streamed_trace_retains_every_tick_and_publishes_atomically(tmp_path):
    trace = PhysicsTrace(tmp_path)
    for tick in range(1000):
        trace.append(dict(tick=tick))
        trace[-1]["paced_budget_overrun"] = tick % 2 == 0
        assert trace.count == tick
        assert not trace.final.exists()
    trace.close()
    assert not trace.partial.exists()
    assert json.loads(trace.final.read_text()) == [
        dict(tick=tick, paced_budget_overrun=tick % 2 == 0) for tick in range(1000)
    ]
    trace.close()
    with pytest.raises(RuntimeError, match="finalized"):
        trace.append(dict(tick=1001))


def test_nonfinite_tick_keeps_evidence_partial(tmp_path):
    trace = PhysicsTrace(tmp_path)
    trace.append(dict(effort=float("nan")))
    with pytest.raises(ValueError, match="JSON"):
        trace.close()
    assert trace.partial.exists()
    assert not trace.final.exists()
