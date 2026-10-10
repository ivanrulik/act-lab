"""Required isolated ROS job: missing dependencies and incomplete evidence fail."""

from act_lab.adapters.ros2.simulation_smoke import run_motion


def fresh_enabled(session, pose, gripper=0.4):
    """Bound discovery/lifecycle recovery without relaxing motion watchdogs."""
    return session.motion_command(pose, gripper)


def test_generated_dds_crisp_motion_and_faults(tmp_path):
    report = run_motion(tmp_path)
    assert report["status"] == "completed"
    assert all(report["telemetry"].values())
    cases = {case["case"] for case in report["cases"]}
    assert {
        "translation",
        "orientation",
        "gripper_open_close",
        "controller_loss",
        "controller_recovery",
        "reset_recovery",
        "grasp_lift_hold_resume",
    } <= cases
    assert report["producer_pid"] != report["gateway_pid"]


def test_actual_system_interface_ordering_and_invalid_feedback():
    import subprocess

    subprocess.run(
        [
            "/opt/simulation/install/act_lab_mujoco_system/lib/act_lab_mujoco_system/check_system"
        ],
        check=True,
        timeout=15,
    )


def test_gateway_sigkill_leaves_independent_dynamic_hold(tmp_path):
    import json
    import time

    from act_lab.adapters.ros2.simulation import snapshot_state
    from act_lab.adapters.ros2.simulation_smoke import MotionSession

    session = MotionSession(tmp_path)
    try:
        fresh_enabled(session, snapshot_state(session.value).end_effector_pose)
        session.process.kill()
        session.process.join(timeout=5)
        deadline = time.monotonic() + 15
        evidence = tmp_path / "physics.json"
        while not evidence.exists():
            if time.monotonic() > deadline:
                raise RuntimeError(
                    "owner failed to finalize gateway-death hold evidence"
                )
            time.sleep(0.01)
        rows = json.loads(evidence.read_text())
        held = rows[-250:]
        assert len(held) == 250
        assert all(row["mode"] == "FAULT_HOLD" for row in held)
        assert all(row["reason"] == "gateway_disconnected" for row in held)
        assert all(not any(row["task_effort_nm"]) for row in held)
        assert (
            held[-1]["state"]["timestamp_ns"] - held[0]["state"]["timestamp_ns"]
            == 498_000_000
        )
        assert held[-1]["accepted_gripper"] > 0
    finally:
        session.close()


def test_controller_stall_cannot_block_owner_hold(tmp_path):
    import os
    import signal

    from act_lab.adapters.ros2.simulation import snapshot_state
    from act_lab.adapters.ros2.simulation_smoke import MotionSession

    session = MotionSession(tmp_path)
    try:
        pose = snapshot_state(session.value).end_effector_pose
        fresh_enabled(session, pose)
        os.kill(session.value["controller_pid"], signal.SIGSTOP)
        result = session.command(pose, gripper=1.0)
        assert result["mode"] == "FAULT_HOLD"
        session.rpc(dict(kind="restart_controller"))
        fresh_enabled(session, snapshot_state(session.value).end_effector_pose)
        assert session.value["mode"] == "ENABLED"
    finally:
        session.close()


def test_paced_owner_expires_without_gateway_control_ticks(tmp_path):
    import time

    from act_lab.adapters.ros2.simulation import snapshot_state
    from act_lab.adapters.ros2.simulation_smoke import MotionSession

    session = MotionSession(tmp_path, paced=True)
    try:
        pose = snapshot_state(session.value).end_effector_pose
        for _ in range(20):
            session.command(pose, gripper=0.4)
        before = snapshot_state(session.value).timestamp_ns
        time.sleep(0.15)
        result = session.rpc(dict(kind="snapshot"))
        assert result["mode"] == "FAULT_HOLD"
        assert snapshot_state(result).timestamp_ns > before
        assert result["reason"] in {
            "stale_source",
            "receipt_timeout",
            "gateway_wall_timeout",
        }
    finally:
        session.close()


def test_stepped_producer_samples_authority_after_wall_watchdog(tmp_path):
    import time

    from act_lab.adapters.ros2.simulation import snapshot_state
    from act_lab.adapters.ros2.simulation_smoke import MotionSession

    session = MotionSession(tmp_path)
    try:
        pose = snapshot_state(session.value).end_effector_pose
        fresh_enabled(session, pose)
        previous_source = session.trace[-1]["source_timestamp_ns"]
        previous_sequence = session.trace[-1]["sequence"]
        time.sleep(0.25)
        held = session.rpc(dict(kind="snapshot"))
        assert held["mode"] == "FAULT_HOLD"
        assert held["reason"] == "gateway_wall_timeout"
        result = fresh_enabled(session, snapshot_state(held).end_effector_pose)
        assert result["mode"] == "ENABLED"
        assert session.trace[-1]["source_timestamp_ns"] > previous_source
        assert session.trace[-1]["sequence"] > previous_sequence
    finally:
        session.close()


def test_expected_dds_publication_loss_returns_disabled_hold(tmp_path):
    import json

    from act_lab.adapters.ros2.simulation import receive, send, snapshot_state
    from act_lab.adapters.ros2.simulation_smoke import MotionSession

    session = MotionSession(tmp_path)
    try:
        fresh_enabled(session, snapshot_state(session.value).end_effector_pose)
        aperture = session.value["report"]["executed_command"]["gripper_position"]
        send(
            session.connection,
            dict(kind="poll", wait_delivery=True, delivery_barrier=True),
        )
        assert receive(session.connection)["ready_delivery"] is True
        held = receive(session.connection)
        assert "error" not in held
        session.value = held
        assert held["mode"] != "ENABLED"
        assert held["transport_rejection"] == "delivery_timeout"
        assert held["report"]["outcome"] == "disabled"
        assert not held["report"]["requested_command"]["enabled"]
        assert not held["report"]["has_executed_command"]
        fresh_enabled(session, snapshot_state(held).end_effector_pose)
    finally:
        session.close()
    rows = json.loads((tmp_path / "physics.json").read_text())
    held_rows = [row for row in rows if row["reason"] == "disabled"]
    assert held_rows
    assert all(row["accepted_gripper"] == aperture for row in held_rows)
    assert all(not any(row["task_effort_nm"]) for row in held_rows)


def test_controller_preparation_requires_separate_fresh_intent(tmp_path):
    from act_lab.adapters.ros2.simulation import snapshot_state
    from act_lab.adapters.ros2.simulation_smoke import MotionSession

    session = MotionSession(tmp_path)
    try:
        pose = snapshot_state(session.value).end_effector_pose
        fresh_enabled(session, pose)
        previous_sequence = session.sequence
        ready = session.rpc(dict(kind="prepare_controller"))
        assert ready["mode"] != "ENABLED"
        assert ready["reason"] == "fresh_authorization_required"
        assert session.sequence == previous_sequence
        fresh_enabled(session, snapshot_state(ready).end_effector_pose)
        assert session.sequence > previous_sequence
    finally:
        session.close()


def test_producer_loss_when_wall_watchdog_expires_before_simulation_time(tmp_path):
    import json
    import time

    from act_lab.adapters.ros2.simulation import snapshot_state
    from act_lab.adapters.ros2.simulation_smoke import MotionSession, producer_loss

    session = MotionSession(tmp_path)
    try:
        pose = snapshot_state(session.value).end_effector_pose
        fresh_enabled(session, pose)
        source_ns = session.trace[-1]["source_timestamp_ns"]
        sequence = session.sequence
        # No new intent: force wall expiry before the subsequent tick batch.
        time.sleep(0.12)
        result = producer_loss(session)
        assert result["reason"] == "gateway_wall_timeout"
        assert result["source_timestamp_ns"] == source_ns
        assert result["command_sequence"] == sequence
        assert result["final_source_age_ns"] >= 100_000_000
    finally:
        session.close()
    rows = json.loads((tmp_path / "physics.json").read_text())
    fault_rows = [r for r in rows if r["reason"] == "gateway_wall_timeout"]
    assert fault_rows
    assert all(not any(r["task_effort_nm"]) for r in fault_rows)
