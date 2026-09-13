import argparse
import hashlib
import json

import pytest

from act_lab.adapters.mcap.session import webcam_acquisition_provenance


def test_model_and_video_identity_follow_content_not_machine_paths(tmp_path):
    model = tmp_path / "model.task"
    video = tmp_path / "input.mp4"
    model.write_bytes(b"model one")
    video.write_bytes(b"synthetic video identity")
    args = argparse.Namespace(
        model=model,
        video=video,
        camera="/dev/video0",
        headless=True,
        auto_calibrate=True,
        max_steps=10,
    )
    first = webcam_acquisition_provenance(args)
    assert first["input"] == {
        "kind": "recorded_video",
        "sha256": hashlib.sha256(video.read_bytes()).hexdigest(),
    }
    assert first["tracking_model"]["sha256"] == hashlib.sha256(b"model one").hexdigest()
    assert str(tmp_path) not in json.dumps(first)
    model.write_bytes(b"model two")
    assert (
        webcam_acquisition_provenance(args)["tracking_model"] != first["tracking_model"]
    )
    args.video = None
    assert webcam_acquisition_provenance(args)["input"] == {
        "kind": "live_camera",
        "camera_id": "/dev/video0",
    }
    args.model = tmp_path / "missing.task"
    with pytest.raises(FileNotFoundError):
        webcam_acquisition_provenance(args)
