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


def _delayed_shutdown_owner(connection, folder):
    import time
    from pathlib import Path

    from act_lab.adapters.ros2.simulation import finalize_physics_trace

    assert receive(connection)["kind"] == "shutdown"
    # Finalization can outlast the old five-second post-acknowledgment window.
    time.sleep(5.1)
    finalize_physics_trace(Path(folder), [{"mode": "SHUTDOWN_HOLD"}])
    send(connection, {"closed": True})
    connection.close()


def test_shutdown_waits_for_finalized_evidence(tmp_path):
    import json

    from act_lab.adapters.ros2.simulation import SimulationProcess

    context = mp.get_context("spawn")
    runtime = SimulationProcess.__new__(SimulationProcess)
    runtime.connection, child = context.Pipe()
    runtime.process = context.Process(
        target=_delayed_shutdown_owner, args=(child, str(tmp_path))
    )
    runtime.process.start()
    child.close()
    try:
        runtime.close()
        assert runtime.process.exitcode == 0
        assert json.loads((tmp_path / "physics.json").read_text()) == [
            {"mode": "SHUTDOWN_HOLD"}
        ]
        assert not (tmp_path / "physics.json.partial").exists()
    finally:
        if runtime.process.is_alive():
            runtime.process.kill()
            runtime.process.join(timeout=5)
        runtime.connection.close()


def test_invalid_trace_cannot_replace_finalized_evidence(tmp_path):
    from act_lab.adapters.ros2.simulation import finalize_physics_trace

    finalized = tmp_path / "physics.json"
    finalized.write_text('[{"previous":true}]\n')
    with pytest.raises(ValueError):
        finalize_physics_trace(tmp_path, [{"effort": float("nan")}])
    assert finalized.read_text() == '[{"previous":true}]\n'


def test_shutdown_does_not_hide_owner_failure():
    from act_lab.adapters.ros2.simulation import SimulationProcess

    context = mp.get_context("spawn")
    runtime = SimulationProcess.__new__(SimulationProcess)
    runtime.connection, child = context.Pipe()
    runtime.process = context.Process(target=int, args=("invalid",))
    runtime.process.start()
    child.close()
    runtime.process.join(timeout=5)
    with pytest.raises(RuntimeError, match="physics owner exited with code"):
        runtime.close()
