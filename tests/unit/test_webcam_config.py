from pathlib import Path

import pytest

from act_lab.adapters.mediapipe import WebcamConfig


def test_default_webcam_config_is_valid() -> None:
    config = WebcamConfig.load(Path("configs/teleop/webcam.toml"))
    assert config.max_frame_age_ms < 100
    assert config.pinch_closed_ratio < config.pinch_open_ratio
    assert config.depth_ema_alpha < config.ema_alpha


def test_invalid_pinch_range_is_rejected(tmp_path: Path) -> None:
    original = Path("configs/teleop/webcam.toml").read_text()
    invalid = original.replace("pinch_open_ratio = 1.50", "pinch_open_ratio = 0.20")
    path = tmp_path / "invalid.toml"
    path.write_text(invalid)

    with pytest.raises(ValueError, match="pinch_closed_ratio"):
        WebcamConfig.load(path)
