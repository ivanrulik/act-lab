"""Required isolated ROS job: missing dependencies and incomplete evidence fail."""

from act_lab.adapters.ros2.simulation_smoke import run_motion


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
        session.command(snapshot_state(session.value).end_effector_pose, gripper=0.4)
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
        session.command(pose, gripper=0.4)
        os.kill(session.value["controller_pid"], signal.SIGSTOP)
        result = session.command(pose, gripper=1.0)
        assert result["mode"] == "FAULT_HOLD"
        session.rpc(dict(kind="restart_controller"))
        session.command(snapshot_state(session.value).end_effector_pose, gripper=0.4)
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
        session.command(pose, gripper=0.4)
        previous_source = session.trace[-1]["source_timestamp_ns"]
        time.sleep(0.25)
        held = session.rpc(dict(kind="snapshot"))
        assert held["mode"] == "FAULT_HOLD"
        assert held["reason"] == "gateway_wall_timeout"
        result = session.command(snapshot_state(held).end_effector_pose, gripper=0.4)
        assert result["mode"] == "ENABLED"
        assert session.trace[-1]["source_timestamp_ns"] > previous_source
        assert session.trace[-1]["sequence"] == 2
    finally:
        session.close()
