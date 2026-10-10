"""Explicit simulator model/tool bindings; no ROS or domain-specific frameworks."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from importlib.resources import files
from pathlib import Path
from typing import Any

import numpy as np


@dataclass(frozen=True, slots=True)
class ModelBinding:
    model_id: str
    scene_filename: str
    robot_filename: str
    tool_translation_m: tuple[float, float, float]
    adaptive_gripper: bool

    @property
    def scene_path(self) -> Path:
        return Path(
            str(
                files("act_lab.adapters.mujoco").joinpath(
                    "assets", "ur5e", self.scene_filename
                )
            )
        )

    def identity(self) -> dict[str, Any]:
        root = self.scene_path.parent
        paths = [root / self.scene_filename, root / self.robot_filename]
        if self.adaptive_gripper:
            paths += sorted((root.parent / "robotiq_2f85/assets").glob("*.stl"))
            paths.append(root.parent / "robotiq_2f85/aperture-calibration.json")
        hashes = {
            str(path.relative_to(root.parent)): hashlib.sha256(
                path.read_bytes()
            ).hexdigest()
            for path in paths
        }
        # Include shared UR geometry as well as the assembled XML and tool assets.
        for path in sorted((root / "assets").glob("*.obj")):
            hashes[str(path.relative_to(root.parent))] = hashlib.sha256(
                path.read_bytes()
            ).hexdigest()
        fingerprint = hashlib.sha256(
            json.dumps(hashes, sort_keys=True).encode()
        ).hexdigest()
        return dict(model_id=self.model_id, sha256=fingerprint, files=hashes)


MODELS = {
    "ur5e_educational_v1": ModelBinding(
        "ur5e_educational_v1", "act_lab_scene.xml", "ur5e.xml", (0.0, 0.0, 0.11), False
    ),
    "ur5e_2f85_d405_v1": ModelBinding(
        "ur5e_2f85_d405_v1",
        "act_lab_2f85_scene.xml",
        "ur5e_2f85.xml",
        (0.0, 0.0, 0.1558),
        True,
    ),
}


class GripperBinding:
    """Translate normalized jaw intent into model-specific actuation and feedback."""

    def __init__(self, model: Any, binding: ModelBinding) -> None:
        self.adaptive = binding.adaptive_gripper
        self.model = model
        self.pad_geoms = (
            ("rq_left_pad1", "rq_left_pad2", "rq_right_pad1", "rq_right_pad2")
            if self.adaptive
            else ("left_finger_pad", "right_finger_pad")
        )
        self.joint_names = (
            tuple(
                model.joint(j).name
                for j in range(model.njnt)
                if model.joint(j).name.startswith("rq_")
            )
            if self.adaptive
            else ("left_finger_joint", "right_finger_joint")
        )
        if self.adaptive:
            path = (
                binding.scene_path.parent.parent
                / "robotiq_2f85/aperture-calibration.json"
            )
            raw = json.loads(path.read_text())
            if (
                raw["assembly_sha256"]
                != hashlib.sha256(
                    (binding.scene_path.parent / binding.robot_filename).read_bytes()
                ).hexdigest()
            ):
                raise ValueError("gripper aperture calibration does not match model")
            samples = list(reversed(raw["samples"]))
            self._apertures = np.asarray([r["aperture_m"] for r in samples])
            self._controls = np.asarray([r["control"] for r in samples])
            self._joint_table = {
                name: np.asarray([r["joints"][name] for r in samples])
                for name in self.joint_names
            }
            if (
                raw.get("schema_version") != 1
                or len(samples) < 2
                or not np.isfinite(self._controls).all()
                or np.any(self._controls < 0)
                or np.any(self._controls > 255)
                or any(not np.isfinite(v).all() for v in self._joint_table.values())
            ):
                raise ValueError("invalid gripper aperture calibration table")
            if not np.isfinite(self._apertures).all() or not np.all(
                np.diff(self._apertures) > 0
            ):
                raise ValueError(
                    "gripper aperture calibration must be finite and increasing"
                )

    def control(self, aperture: float) -> float:
        if not np.isfinite(aperture) or not 0.0 <= aperture <= 1.0:
            raise ValueError("gripper target must be finite and in [0, 1]")
        if not self.adaptive:
            return aperture * 0.025
        return float(np.interp(aperture * 0.085, self._apertures, self._controls))

    def set_kinematic(self, data: Any, aperture: float) -> None:
        self.control(aperture)  # Validate before mutating any joint.
        for name in self.joint_names:
            value = (
                float(
                    np.interp(
                        aperture * 0.085, self._apertures, self._joint_table[name]
                    )
                )
                if self.adaptive
                else aperture * 0.025
            )
            data.joint(name).qpos[0] = value

    def measured(self, data: Any) -> float:
        if not self.adaptive:
            return float(np.clip(data.joint("left_finger_joint").qpos[0] / 0.025, 0, 1))
        gap = data.site("rq_right_inner_pad").xpos - data.site("rq_left_inner_pad").xpos
        axis = data.body("rq_base").xmat.reshape(3, 3)[:, 1]
        return float(np.clip(np.dot(gap, axis) / 0.085, 0, 1))

    def joints(self, data: Any) -> dict[str, float]:
        return {name: float(data.joint(name).qpos[0]) for name in self.joint_names}
