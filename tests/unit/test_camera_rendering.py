"""GPU requests never silently change backend; preview encoding preserves RGB."""

import io
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image

from act_lab.adapters.ros2.camera_rendering import (
    encode_jpeg,
    render_settings,
    verify_renderer,
)


def test_explicit_render_profiles_and_unknown_backend(monkeypatch):
    monkeypatch.setenv("ACT_LAB_RENDERER", "software")
    cpu = render_settings()
    assert not cpu["shadows"] and cpu["offsamples"] == 0
    monkeypatch.setenv("ACT_LAB_RENDERER", "nvidia")
    gpu = render_settings()
    assert gpu["shadows"] and gpu["sha256"] != cpu["sha256"]
    monkeypatch.setenv("ACT_LAB_RENDERER", "automatic")
    with pytest.raises(ValueError):
        render_settings()


def test_gpu_request_rejects_software_gl_even_when_device_exists(monkeypatch):
    def get_string(code):
        return {0x1F00: b"Mesa", 0x1F01: b"llvmpipe", 0x1F02: b"4.5"}[code]

    monkeypatch.setattr(
        "act_lab.adapters.ros2.camera_rendering.ctypes.CDLL",
        lambda _: SimpleNamespace(glGetString=get_string),
    )
    with pytest.raises(RuntimeError, match="NVIDIA renderer unavailable"):
        verify_renderer(dict(backend="nvidia"))
    assert verify_renderer(dict(backend="software"))["renderer"] == "llvmpipe"


def test_jpeg_keeps_rgb_order_and_original_array():
    pixels = np.zeros((24, 32, 3), dtype=np.uint8)
    pixels[:, :, 0] = 220
    before = pixels.copy()
    encoded = encode_jpeg(pixels)
    decoded = np.asarray(Image.open(io.BytesIO(encoded)).convert("RGB"))
    assert decoded.shape == pixels.shape
    assert decoded[:, :, 0].mean() > 200 and decoded[:, :, 2].mean() < 10
    assert np.array_equal(pixels, before)
