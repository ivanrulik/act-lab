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


def test_late_controller_reply_is_drained_without_restoring_authority():
    import subprocess
    import sys
    import time
    from uuid import UUID

    from act_lab.adapters.ros2.simulation import ControllerProcess
    from act_lab.adapters.ros2.simulation_guard import Authorization, SimulationGuard
    from act_lab.domain import Pose

    episode = UUID(int=1)
    guard = SimulationGuard(episode)
    intent = Authorization(
        episode, 1, 0, 0, Pose("world", (0.4, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0)), 0.4
    )
    assert guard.authorize(intent, 0)
    generation = guard.generation
    assert guard.reactivated(generation, 0)
    assert guard.output(episode, generation, 0, (0.0,) * 6, 0)
    process = subprocess.Popen(
        [sys.executable, "-u", "-c",
         "import sys,time;sys.stdin.readline();time.sleep(.2);"
         "print('{\"schema_version\":1,\"effort\":[1,1,1,1,1,1]}')"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
    )
    controller = ControllerProcess.__new__(ControllerProcess)
    controller.process = process
    controller.buffer = b""
    start = time.monotonic_ns()
    expired = []

    def watchdog():
        if time.monotonic_ns() - start >= 100_000_000 and not expired:
            guard.fault("controller_wall_timeout")
            expired.append(True)

    try:
        response = controller.request(dict(tick=1), watchdog)
        assert expired
        assert response["effort"] == [1] * 6
        assert not guard.output(episode, generation, 1, tuple(response["effort"]), 0)
        assert guard.mode == "FAULT_HOLD"
        assert guard.reason == "controller_wall_timeout"
        assert guard.gripper == 0.4
    finally:
        process.stdin.close()
        process.stdout.close()
        process.wait(timeout=5)
