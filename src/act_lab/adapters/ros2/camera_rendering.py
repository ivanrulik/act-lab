"""Explicit viewer-only rendering policy and lazy JPEG codec."""

from __future__ import annotations

import ctypes
import hashlib
import io
import json
import os
from importlib import import_module
from typing import Any


def render_settings() -> dict[str, Any]:
    backend = os.environ.get("ACT_LAB_RENDERER", "software")
    if backend not in {"software", "nvidia"}:
        raise ValueError("ACT_LAB_RENDERER must be software or nvidia")
    settings = dict(
        schema_version=1,
        backend=backend,
        shadows=backend == "nvidia",
        reflections=backend == "nvidia",
        offsamples=4 if backend == "nvidia" else 0,
        jpeg_quality=90,
        target_hz=25,
        acquisition_modified=False,
    )
    settings["sha256"] = hashlib.sha256(
        json.dumps(settings, sort_keys=True).encode()
    ).hexdigest()
    return settings


def verify_renderer(settings: dict[str, Any]) -> dict[str, str]:
    """Inspect the current EGL context; never substitute a requested GPU."""
    gl = ctypes.CDLL("libGL.so.1")
    gl.glGetString.argtypes = [ctypes.c_uint]
    gl.glGetString.restype = ctypes.c_char_p
    identity = {
        name: (gl.glGetString(code) or b"").decode()
        for name, code in (
            ("vendor", 0x1F00),
            ("renderer", 0x1F01),
            ("version", 0x1F02),
        )
    }
    if not identity["renderer"]:
        raise RuntimeError("cannot verify the active OpenGL renderer")
    nvidia = "nvidia" in identity["vendor"].lower()
    if settings["backend"] == "nvidia" and not nvidia:
        raise RuntimeError(f"requested NVIDIA renderer unavailable: {identity}")
    if settings["backend"] == "software" and not any(
        name in identity["renderer"].lower() for name in ("llvmpipe", "softpipe")
    ):
        raise RuntimeError(f"requested software renderer unavailable: {identity}")
    return identity


def encode_jpeg(pixels: Any, quality: int = 90) -> bytes:
    """Encode RGB directly; no BGR conversion or acquisition mutation."""
    image = import_module("PIL.Image")
    output = io.BytesIO()
    image.fromarray(pixels, mode="RGB").save(
        output, format="JPEG", quality=quality, optimize=False
    )
    return output.getvalue()
