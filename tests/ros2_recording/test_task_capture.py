"""Required full-task evidence through actual generated DDS and CRISP."""

import json
from pathlib import Path

import pytest

from act_lab.adapters.ros2.recording_demo import run_recording
from act_lab.adapters.ros2.task_qualification import physics_summary


def test_complete_crisp_task_records_owner_outcome(tmp_path):
    result = run_recording(
        tmp_path / "task",
        controller="crisp",
        paced=True,
        full_task=True,
        capture_profile="reduced-effects",
        config=Path("configs/sim/ur5e_2f85_d405_crisp_task_bench.toml"),
    )
    assert result["acquisition"]["task"]["success"]
    assert result["acquisition"]["task"]["released"]
    assert result["acquisition"]["task"]["reason"] == "success"
    assert all(case["quality"]["valid"] for case in result["import_report"]["cases"])
    physics = physics_summary(tmp_path / "task" / "motion" / "physics.json")
    assert not physics["violations"]
    assert physics["zero_task_effort_in_hold"]
    rows = [
        json.loads(line)
        for line in (tmp_path / "task" / "motion" / "expert.jsonl")
        .read_text()
        .splitlines()
    ]
    assert {"lift", "transit", "lower", "open_dwell"} <= {row["phase"] for row in rows}
    assert max(row["task"]["cube_pose"]["position_xyz_m"][2] for row in rows) > 0.465
    assert all(
        row["action"]["timestamp_ns"]
        == row["report"]["requested_action"]["timestamp_ns"]
        for row in rows
    )
    assert result["acquisition"]["capture"]["capture_profile"] == "reduced-effects"


def test_ack_loss_retries_without_duplicate_samples(tmp_path):
    from act_lab.adapters.ros2.recording import import_bag
    from act_lab.adapters.ros2.recording_demo import Recorder
    from act_lab.adapters.ros2.simulation import snapshot_state
    from act_lab.adapters.ros2.simulation_smoke import MotionSession

    recorder = Recorder(tmp_path / "rosbag", drop_ack_once=2)
    session = MotionSession(
        tmp_path / "motion",
        paced=True,
        config_path=Path("configs/sim/ur5e_2f85_d405_capture_bench.toml"),
    )
    try:
        session.rpc(
            dict(kind="record_start", seed=0, capture_profile="reduced-effects")
        )
        session.motion_command(snapshot_state(session.value).end_effector_pose)
        session.rpc(dict(kind="record_stop"))
        capture = session.value["capture"]
        storage = recorder.close()
    finally:
        session.close()
        if recorder.process.is_alive():
            recorder.close(False)
    assert storage["identical_retries"] >= 1
    imported = import_bag(tmp_path / "rosbag" / "bag", tmp_path / "imported")
    assert imported["samples"] == capture["samples"] == 2
    assert all(age < 100_000_000 for age in capture["capture_ages_ns"])


def test_retry_identity_ignores_cdr_padding_and_checks_pixels():
    from act_lab_interfaces.msg import RecordingPacket
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import Image

    from act_lab.adapters.ros2.recording import retry_identity

    packet = RecordingPacket(episode_id="00000000-0000-0000-0000-000000000001")
    packet.sample.images = [
        Image(height=1, width=1, encoding="rgb8", step=3, data=b"abc")
    ]
    identity = retry_identity(packet)
    for _ in range(50):
        decoded = deserialize_message(serialize_message(packet), RecordingPacket)
        assert retry_identity(decoded) == identity
    decoded.sample.images[0].data[0] = 0
    assert retry_identity(decoded) != identity


@pytest.mark.parametrize(
    "fault",
    [
        "producer_loss",
        "controller_loss",
        "worker_loss",
        "worker_stall",
        "recorder_loss",
        "gateway_loss",
        "clock_pause",
        "active_reset",
    ],
)
def test_loaded_fault_retains_grip_and_independent_hold(tmp_path, fault):
    import math
    import time

    from act_lab.adapters.ros2.recording_demo import Recorder, _expert_task
    from act_lab.adapters.ros2.simulation import snapshot_state
    from act_lab.adapters.ros2.simulation_smoke import MotionSession

    config = Path("configs/sim/ur5e_2f85_d405_crisp_task_bench.toml")
    recorder = Recorder(tmp_path / "rosbag")
    session = MotionSession(tmp_path / "motion", paced=True, config_path=config)
    interrupted = fault in {
        "worker_loss",
        "worker_stall",
        "recorder_loss",
        "gateway_loss",
        "active_reset",
    }
    try:
        session.rpc(
            dict(
                kind="record_start",
                seed=0,
                full_task=True,
                capture_profile="reduced-effects",
            )
        )
        _expert_task(session, config, 2500, stop_phase="transit")
        before = session.value
        pose = snapshot_state(before).end_effector_pose
        assert before["cube_gripper_contacts"] > 0
        cube_z = before["task"]["cube_pose"]["position_xyz_m"][2]
        assert cube_z > 0.465
        aperture = before["accepted_gripper"]
        if fault == "worker_loss":
            session.rpc(dict(kind="kill_capture_worker"))
        elif fault == "worker_stall":
            session.rpc(dict(kind="stall_capture_worker"))
        elif fault == "recorder_loss":
            recorder.process.kill()
            recorder.process.join(5)
        elif fault == "controller_loss":
            session.rpc(dict(kind="kill_controller"))
        elif fault == "clock_pause":
            session.rpc(dict(kind="clock", pause_ns=100_000_000))
        elif fault == "gateway_loss":
            session.process.kill()
            session.process.join(5)
        elif fault == "active_reset":
            with pytest.raises(RuntimeError, match="before resetting"):
                session.rpc(dict(kind="reset", seed=1))
            session.process.join(5)
        # Do not submit a disabled command: independently observed loss must
        # inhibit the owner while it still carries the cube.
        time.sleep(0.25)
        if fault in {"gateway_loss", "active_reset"}:
            deadline = time.monotonic() + 15
            evidence = tmp_path / "motion" / "physics.json"
            while not evidence.exists():
                if time.monotonic() > deadline:
                    raise RuntimeError("loaded gateway loss evidence missing")
                time.sleep(0.01)
            held = json.loads(evidence.read_text())[-250:]
            final = held[-1]
            expected_mode = (
                "SHUTDOWN_HOLD" if fault == "active_reset" else "FAULT_HOLD"
            )
            assert all(row["mode"] == expected_mode for row in held)
            assert all(not any(row["task_effort_nm"]) for row in held)
            assert final["accepted_gripper"] == aperture
            assert final["episode"] == before["episode"]
            assert final["cube_xyz_m"][2] >= cube_z - 0.01
            assert final["cube_gripper_contacts"] > 0
            assert (
                math.dist(
                    final["state"]["end_effector_pose"]["position_xyz_m"],
                    pose.position_xyz_m,
                )
                <= 0.002
            )
        else:
            held = session.rpc(dict(kind="snapshot"))
            assert held["mode"] == "FAULT_HOLD"
            assert held["accepted_gripper"] == aperture
            assert held["task"]["cube_pose"]["position_xyz_m"][2] >= cube_z - 0.01
            assert held["cube_gripper_contacts"] > 0
            assert (
                math.dist(
                    snapshot_state(held).end_effector_pose.position_xyz_m,
                    pose.position_xyz_m,
                )
                <= 0.002
            )
            if fault in {"producer_loss", "controller_loss", "clock_pause"}:
                previous_sequence = session.sequence
                if fault == "controller_loss":
                    session.rpc(dict(kind="restart_controller"))
                session.motion_command(
                    snapshot_state(session.value).end_effector_pose, aperture
                )
                assert session.value["mode"] == "ENABLED"
                assert session.sequence > previous_sequence
                session.rpc(dict(kind="record_stop"))
                recorder.close()
            else:
                with pytest.raises(RuntimeError, match="capture|acknowledgement"):
                    session.command(pose)
    finally:
        try:
            try:
                session.close()
            except (RuntimeError, OSError, EOFError):
                if not interrupted:
                    raise
        finally:
            if recorder.process.is_alive():
                recorder.close(False)
            else:
                recorder.connection.close()
    if interrupted:
        assert (tmp_path / "rosbag" / "bag.partial").exists()
        assert not (tmp_path / "rosbag" / "bag").exists()
