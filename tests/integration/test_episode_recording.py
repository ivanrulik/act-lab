"""Exercise recording through real headless simulation."""

import argparse
from pathlib import Path

import pytest

from act_lab.adapters.mcap.inspection import inspect_episode, recover_episode
from act_lab.adapters.mcap.session import recording_robot
from act_lab.adapters.mujoco import MujocoCartesianDriver
from act_lab.cli import main
from act_lab.domain import Action


def test_cli_records_failed_and_discarded_attempts(tmp_path: Path, capsys):
    config = tmp_path / "short.toml"
    config.write_text(
        Path("configs/sim/ur5e_pick_place.toml")
        .read_text()
        .replace("episode_steps = 500", "episode_steps = 3")
    )
    for outcome in ("failure", "discarded"):
        output = tmp_path / outcome
        assert (
            main(
                [
                    "sim",
                    "expert",
                    "--episodes",
                    "1",
                    "--config",
                    str(config),
                    "--record-dir",
                    str(output),
                    "--outcome",
                    outcome,
                    "--reason",
                    "test attempt",
                    "--json",
                ]
            )
            == 1
        )
        files = list(output.glob("*.mcap"))
        assert len(files) == 1
        report = inspect_episode(files[0])
        assert report["outcome"] == outcome
        assert report["complete"] and report["monotonic"]
        assert report["provenance"]["source"] == "expert"
        topics = ("/observation", "/command", "/camera/overview", "/camera/policy")
        for topic in topics:
            assert report["streams"][topic]["count"] == 4
            assert report["streams"][topic]["observed_hz"] == 50
        assert not list(output.glob("*.partial"))
        assert main(["recording", "inspect", str(files[0])]) == 0
    capsys.readouterr()


def test_session_exception_leaves_recoverable_attempt(tmp_path: Path):
    args = argparse.Namespace(
        record_dir=tmp_path, outcome="auto", reason=None, operator="tester"
    )
    with MujocoCartesianDriver.from_config_file(
        Path("configs/sim/ur5e_pick_place.toml")
    ) as driver:
        with pytest.raises(RuntimeError, match="injected"):
            with recording_robot(driver, args, "keyboard", 0) as robot:
                observation = robot.reset(0)
                robot.command(
                    Action(0, observation.robot.end_effector_pose, 0.5, False)
                )
                raise RuntimeError("injected session interruption")
    assert not list(tmp_path.glob("*.mcap"))
    partial = next(tmp_path.glob("*.partial"))
    result = inspect_episode(recover_episode(partial))
    assert result["outcome"] == "interrupted"
    assert result["streams"]["/command"]["count"] == 1


def test_invalid_explicit_label_does_not_create_recording(tmp_path: Path):
    assert (
        main(
            [
                "sim",
                "expert",
                "--episodes",
                "1",
                "--record-dir",
                str(tmp_path),
                "--outcome",
                "discarded",
            ]
        )
        == 2
    )
    assert not list(tmp_path.glob("*.mcap*"))


def test_webcam_calibration_and_diagnostics_are_recorded_without_preview(tmp_path):
    import json
    import time
    from dataclasses import replace

    from mcap.reader import make_reader

    from act_lab.adapters.mcap.schema import decode
    from act_lab.adapters.mediapipe import HandSignal, WebcamConfig, WebcamTeleoperator
    from act_lab.domain import CameraFrame

    config = replace(
        WebcamConfig.load(Path("configs/teleop/webcam.toml")),
        calibration_sample_frames=1,
        clutch_engage_frames=1,
    )
    teleoperator = WebcamTeleoperator(config)
    teleoperator.request_calibration()
    preview = CameraFrame(0, 1, 1, b"xyz")
    signal = HandSignal(0, "Right", 0.99, 0.5, 0.5, 0.2, 0.2, 0.5, 0.2, True, preview)
    model = tmp_path / "synthetic-model.task"
    model.write_bytes(b"synthetic model identity for injected hand signals")
    args = argparse.Namespace(
        record_dir=tmp_path,
        outcome="auto",
        reason=None,
        operator="tester",
        model=model,
        video=None,
        camera="synthetic-camera",
        headless=True,
        auto_calibrate=False,
        max_steps=3,
    )
    with MujocoCartesianDriver.from_config_file(
        Path("configs/sim/ur5e_pick_place.toml")
    ) as driver:
        with recording_robot(driver, args, "webcam", 0, teleoperator) as robot:
            observation = robot.reset(0)
            for step in range(3):
                teleoperator.update(
                    replace(signal, source_timestamp_ns=step * 20_000_000),
                    time.monotonic_ns(),
                    "tracked",
                )
                robot.command(teleoperator.poll(observation))
                observation = robot.observe()
    path = next(tmp_path.glob("*.mcap"))
    report = inspect_episode(path)
    acquisition = json.loads(report["provenance"]["resolved_config_json"])[
        "acquisition"
    ]
    assert acquisition["input"] == {
        "kind": "live_camera",
        "camera_id": "synthetic-camera",
    }
    assert len(acquisition["tracking_model"]["sha256"]) == 64
    assert report["streams"]["/teleop/diagnostics"]["count"] == 3
    assert set(report["streams"]) == {
        "/observation",
        "/command",
        "/camera/policy",
        "/camera/overview",
        "/episode/event",
        "/episode/provenance",
        "/teleop/diagnostics",
    }
    with path.open("rb") as source:
        records = list(
            make_reader(source).iter_messages(topics=["/teleop/diagnostics"])
        )
    last = decode("Diagnostics", records[-1][2].data)
    assert json.loads(last.calibration_json)["calibration_id"]
    assert "source_timestamp_ns" in json.loads(last.values_json)
    assert "preview" not in json.loads(last.values_json)
    assert last.host_monotonic_ns > last.timestamp_ns


def test_recording_failure_applies_disabled_hold(tmp_path, monkeypatch):
    from act_lab.adapters.mcap.recording import McapEpisodeSink
    from act_lab.domain import CommandOutcome

    def disk_full(self, sample):
        raise OSError("injected disk full")

    monkeypatch.setattr(McapEpisodeSink, "append", disk_full)
    args = argparse.Namespace(
        record_dir=tmp_path, outcome="auto", reason=None, operator="tester"
    )
    with MujocoCartesianDriver.from_config_file(
        Path("configs/sim/ur5e_pick_place.toml")
    ) as driver:
        with pytest.raises(OSError, match="disk full"):
            with recording_robot(driver, args, "keyboard", 0) as robot:
                observation = robot.reset(0)
                robot.command(Action(0, observation.robot.end_effector_pose, 0.5, True))
        assert robot.last_report.outcome is CommandOutcome.DISABLED
    assert len(list(tmp_path.glob("*.partial"))) == 1
    assert not list(tmp_path.glob("*.mcap"))


@pytest.mark.parametrize(
    "key,outcome", [(295, "success"), (296, "failure"), (297, "discarded")]
)
def test_viewer_labels_finalize_once_on_control_thread(tmp_path, key, outcome):
    from act_lab.adapters.mcap.session import recording_key, recording_status

    args = argparse.Namespace(
        record_dir=tmp_path, outcome="auto", reason=None, operator="tester"
    )
    with MujocoCartesianDriver.from_config_file(
        Path("configs/sim/ur5e_pick_place.toml")
    ) as driver:
        with recording_robot(driver, args, "keyboard", 0) as robot:
            observation = robot.reset(0)
            recording_key(robot, key)
            assert not list(tmp_path.glob("*.mcap"))
            robot.command(Action(0, observation.robot.end_effector_pose, 0.5, False))
            status = recording_status(robot)
            assert f"EPISODE SAVED: {outcome.upper()}" in status
            assert "NOT recorded" in status
            assert "F6" not in status
            path = next(tmp_path.glob("*.mcap"))
            before = path.read_bytes()
            recording_key(robot, 297)  # finalized labels cannot be overwritten
            robot.command(
                Action(20_000_000, observation.robot.end_effector_pose, 0.5, False)
            )
        assert path.read_bytes() == before
    result = inspect_episode(path)
    assert result["outcome"] == outcome
    assert result["streams"]["/command"]["count"] == 1
