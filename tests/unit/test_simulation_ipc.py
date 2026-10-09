import multiprocessing as mp

import pytest

from act_lab.adapters.ros2.simulation import MAX_PACKET, receive, send


def test_versioned_bounded_ipc_round_trip():
    parent, child = mp.Pipe()
    try:
        send(parent, dict(kind="snapshot", sequence=7))
        assert receive(child) == dict(kind="snapshot", sequence=7, schema_version=1)
        with pytest.raises(ValueError, match="too large"):
            send(parent, dict(payload="x" * MAX_PACKET))
        with pytest.raises(ValueError):
            send(parent, dict(effort=float("nan")))
        parent.send_bytes(b'{"schema_version":2}')
        with pytest.raises(ValueError, match="version 1"):
            receive(child)
    finally:
        parent.close()
        child.close()
