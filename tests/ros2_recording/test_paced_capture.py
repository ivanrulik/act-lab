"""Required generated DDS, scratch rendering and moving-owner paced tests."""

import json
import time
from dataclasses import asdict
from pathlib import Path

import pytest

from act_lab.adapters.ros2.recording import import_bag
from act_lab.adapters.ros2.recording_demo import Recorder, run_recording
from act_lab.adapters.ros2.simulation import snapshot_state
from act_lab.adapters.ros2.simulation_smoke import MotionSession

CONFIG = Path("configs/sim/ur5e_2f85_d405_capture_bench.toml")


def finish_failed_session(session):
    # The expected acquisition error exits the gateway after owner hold/cleanup.
    try:
        session.close()
    except (RuntimeError, OSError):
        pass


def test_paced_original_rgb_and_canonical_identity(tmp_path):
    result = run_recording(
        tmp_path / "paced",
        controller="crisp",
        paced=True,
        capture_profile="reduced-effects",
        config=CONFIG,
        steps=3,
        smoke=True,
    )
    capture = result["acquisition"]["capture"]
    assert capture["worker_pid"] not in {
        result["acquisition"]["producer_pid"],
        result["acquisition"]["physics_pid"],
        result["storage"]["pid"],
    }
    assert capture["samples"] >= 4
    assert result["import_report"]["samples"] == capture["samples"]
    assert all(age < 100_000_000 for age in capture["capture_ages_ns"])
    assert any(c["name"] == "exact_local_ros_equivalence" for c in result["cases"])
    assert all(c["quality"]["valid"] for c in result["import_report"]["cases"])
    from act_lab.adapters.mcap.reading import read_episode
    from act_lab.adapters.mujoco import MujocoCartesianDriver
    from act_lab.application.model_compatibility import (
        model_contract,
        require_compatible,
    )

    episode = read_episode(
        tmp_path / "paced" / "imported" / f"{result['acquisition']['episode_id']}.mcap"
    )
    recorded = model_contract(json.loads(episode.provenance["resolved_config_json"]))
    with MujocoCartesianDriver.from_config_file(CONFIG) as driver:
        full = model_contract(
            dict(
                simulation=asdict(driver.simulation_config),
                model_identity=driver.environment.model_identity,
                scene_cameras={
                    name: driver.environment.camera_calibration(name)
                    for name in driver.simulation_config.cameras
                },
            )
        )
    with pytest.raises(ValueError, match="incompatible"):
        require_compatible(full, recorded)


@pytest.mark.parametrize("fault", ["slow_render", "worker_loss", "recorder_loss"])
def test_paced_loss_preserves_partial_and_independent_hold(tmp_path, fault):
    recorder = Recorder(tmp_path / "rosbag")
    session = MotionSession(tmp_path / "motion", paced=True, config_path=CONFIG)
    try:
        session.rpc(
            dict(
                kind="record_start",
                seed=0,
                capture_profile="reduced-effects",
                capture_delay_s=0.25 if fault == "slow_render" else 0.0,
            )
        )
        target = snapshot_state(session.value).end_effector_pose
        if fault == "worker_loss":
            session.rpc(dict(kind="kill_capture_worker"))
        else:
            session.motion_command(target, gripper=0.4)
            before = session.value
            if fault == "recorder_loss":
                recorder.process.terminate()
                recorder.process.join(5)
            # Stop producing commands: owner holds independently while the
            # worker is blocked. No machine-dependent latency comparison.
            time.sleep(0.2)
            held = session.rpc(dict(kind="snapshot"))
            assert held["mode"] == "FAULT_HOLD"
            assert held["accepted_gripper"] == before["accepted_gripper"]
            assert (
                snapshot_state(held).timestamp_ns > snapshot_state(before).timestamp_ns
            )
        with pytest.raises(RuntimeError, match="capture|acknowledgement"):
            session.command(target)
    finally:
        finish_failed_session(session)
        if recorder.process.is_alive():
            recorder.close(False)
        else:
            recorder.connection.close()
    assert (tmp_path / "rosbag" / "bag.partial").exists()
    assert not (tmp_path / "rosbag" / "bag").exists()
    failure = json.loads(
        (tmp_path / "motion" / "acquisition" / "capture-failure.json").read_text()
    )
    assert failure["status"] == "interrupted"
    with pytest.raises(ValueError, match="partial|final"):
        import_bag(tmp_path / "rosbag" / "bag.partial", tmp_path / "imported")


def test_paced_reset_uses_new_episode_and_contiguous_packets(tmp_path):
    recorder = Recorder(tmp_path / "rosbag")
    session = MotionSession(tmp_path / "motion", paced=True, config_path=CONFIG)
    episodes = []
    try:
        for _ in range(2):
            session.rpc(
                dict(
                    kind="record_start",
                    seed=0,
                    capture_profile="reduced-effects",
                )
            )
            episodes.append(session.value["episode"])
            target = snapshot_state(session.value).end_effector_pose
            session.motion_command(target)
            session.rpc(dict(kind="record_stop"))
        recorder.close()
    finally:
        session.close()
        if recorder.process.is_alive():
            recorder.close(False)
    imported = import_bag(tmp_path / "rosbag" / "bag", tmp_path / "imported")
    assert len(set(episodes)) == 2
    assert set(imported["episodes"]) == set(episodes)
