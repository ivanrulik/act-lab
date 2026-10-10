"""Primitive model/camera identities shared by acquisition and learning."""

from __future__ import annotations

import hashlib
import json
from typing import Any

LEGACY_MODEL = "ur5e_educational_v1"
TOOL_MODEL = "ur5e_2f85_d405_v1"
CONTRACT_FILE = "act_lab_model.json"


def _digest(value: object) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, allow_nan=False).encode()
    ).hexdigest()


def model_contract(resolved: dict[str, Any]) -> dict[str, Any] | None:
    """Validate and condense an acquisition identity; retain legacy compatibility."""
    if not isinstance(resolved, dict):
        raise ValueError("resolved acquisition configuration must be an object")
    simulation = resolved.get("simulation", {})
    if not isinstance(simulation, dict):
        raise ValueError("simulation acquisition configuration must be an object")
    model_id = simulation.get("model_id", LEGACY_MODEL)
    if model_id == LEGACY_MODEL:
        return None
    if model_id != TOOL_MODEL:
        raise ValueError(f"unsupported acquisition model: {model_id}")
    identity = resolved.get("model_identity", {})
    profiles = resolved.get("scene_cameras", {})
    if identity.get("model_id") != model_id:
        raise ValueError("acquisition model identity is missing or inconsistent")
    model_hash = identity.get("sha256")
    if not isinstance(model_hash, str) or len(model_hash) != 64:
        raise ValueError("acquisition model fingerprint is missing")
    try:
        int(model_hash, 16)
    except ValueError as error:
        raise ValueError("acquisition model fingerprint is invalid") from error
    files = identity.get("files")
    if not isinstance(files, dict) or not files or _digest(files) != model_hash:
        raise ValueError("acquisition asset manifest does not match model fingerprint")
    cameras = simulation.get("cameras", [])
    if not isinstance(cameras, (list, tuple)) or "wrist" not in cameras:
        raise ValueError("articulated model requires the wrist camera")
    if set(profiles) != set(cameras):
        raise ValueError("camera calibration identities do not match selected cameras")
    parsed = {}
    for camera in sorted(cameras):
        profile = profiles[camera]
        fingerprint = profile.get("sha256")
        fields = {key: value for key, value in profile.items() if key != "sha256"}
        if fingerprint != _digest(fields):
            raise ValueError(f"camera calibration fingerprint mismatch: {camera}")
        if (
            profile.get("camera_id") != camera
            or profile.get("encoding") != "rgb8"
            or profile.get("width") != simulation.get("render_width")
            or profile.get("height") != simulation.get("render_height")
        ):
            raise ValueError(f"camera calibration profile mismatch: {camera}")
        parsed[camera] = dict(
            shape=[profile["height"], profile["width"], 3], sha256=fingerprint
        )
    return dict(
        schema_version=1, model_id=model_id, model_sha256=model_hash, cameras=parsed
    )


def require_compatible(
    expected: dict[str, Any] | None, recorded: dict[str, Any] | None
) -> None:
    if expected != recorded:
        raise ValueError(
            "checkpoint model/camera calibration is incompatible with this preset; "
            "use its original preset or train a checkpoint from the new assembly"
        )
