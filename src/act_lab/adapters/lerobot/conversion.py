"""Write validated domain records through the public LeRobotDataset API."""

from __future__ import annotations

import hashlib
import json
import os
import struct
from pathlib import Path
from typing import Any

import numpy as np

from act_lab.application.dataset import CONVERTER_VERSION, resample_episode
from act_lab.application.training import storage_fingerprint
from act_lab.domain.dataset import RecordedEpisode


def _fingerprint(episodes: list[RecordedEpisode], fps: int) -> str:
    digest = hashlib.sha256()
    digest.update(f"act-lab-converter:{CONVERTER_VERSION}:fps:{fps}\n".encode())
    for episode in episodes:
        digest.update(f"episode:{episode.episode_id}\n".encode())
        for sample in episode.samples:
            digest.update(struct.pack(">q", sample.timestamp_ns))
            values = (
                sample.joint_positions_rad
                + (sample.gripper_position,)
                + sample.action_position_xyz_m
                + sample.action_quaternion_wxyz
                + (sample.action_gripper_position,)
            )
            digest.update(struct.pack(f">{len(values)}d", *values))
            for image in sample.images:
                digest.update(image.camera_id.encode() + b"\0")
                digest.update(image.rgb_bytes)
    return digest.hexdigest()


def convert_episodes(
    episodes: list[RecordedEpisode],
    output: Path,
    repo_id: str,
    fps: int,
    lineage: dict[str, Any],
    *,
    dataset_class: Any | None = None,
) -> dict[str, Any]:
    """Create one atomically published local LeRobotDataset v3 directory."""
    if not episodes:
        raise ValueError("manifest selects no training episodes")
    if output.exists():
        raise FileExistsError(f"output already exists: {output}")
    stage = output.with_name(output.name + ".partial")
    if stage.exists():
        raise FileExistsError(f"partial conversion already exists: {stage}")
    converted = [resample_episode(episode, fps) for episode in episodes]
    if any(not episode.samples for episode in converted):
        raise ValueError("resampling produced an empty episode")
    first = converted[0].samples[0]
    state_size = len(first.joint_positions_rad) + 1
    expected_cameras = {
        image.camera_id: (image.width, image.height) for image in first.images
    }
    for episode in converted:
        for sample in episode.samples:
            cameras = {
                image.camera_id: (image.width, image.height) for image in sample.images
            }
            if len(sample.joint_positions_rad) + 1 != state_size:
                raise ValueError(
                    f"episode {episode.episode_id} has an incompatible state size"
                )
            if cameras != expected_cameras:
                raise ValueError(
                    f"episode {episode.episode_id} has an incompatible camera schema"
                )
    features: dict[str, dict[str, Any]] = {
        "observation.state": {
            "dtype": "float32",
            "shape": (state_size,),
            "names": [
                *[f"joint_{index}" for index in range(state_size - 1)],
                "gripper",
            ],
        },
        "action": {
            "dtype": "float32",
            "shape": (8,),
            "names": ["x", "y", "z", "qw", "qx", "qy", "qz", "gripper"],
        },
    }
    for image in first.images:
        features[f"observation.images.{image.camera_id}"] = {
            "dtype": "image",
            "shape": (image.height, image.width, 3),
            "names": ["height", "width", "channels"],
        }
    if dataset_class is None:
        try:
            from lerobot.datasets.lerobot_dataset import (  # type: ignore[import-not-found]
                LeRobotDataset,
            )
        except ImportError as error:
            raise RuntimeError(
                "LeRobot conversion requires the pinned data dependencies; "
                "use Docker Compose"
            ) from error
        dataset_class = LeRobotDataset
    dataset = dataset_class.create(
        repo_id=repo_id,
        fps=fps,
        root=stage,
        robot_type="ur5e-parallel-gripper-v1",
        features=features,
        use_videos=False,
    )
    for episode in converted:
        for sample in episode.samples:
            frame: dict[str, Any] = {
                "observation.state": np.asarray(
                    sample.joint_positions_rad + (sample.gripper_position,),
                    dtype=np.float32,
                ),
                "action": np.asarray(
                    sample.action_position_xyz_m
                    + sample.action_quaternion_wxyz
                    + (sample.action_gripper_position,),
                    dtype=np.float32,
                ),
                "task": "Pick and place the cube into the tray",
            }
            for image in sample.images:
                frame[f"observation.images.{image.camera_id}"] = np.frombuffer(
                    image.rgb_bytes, dtype=np.uint8
                ).reshape(image.height, image.width, 3)
            dataset.add_frame(frame)
        dataset.save_episode()
    dataset.finalize()
    fingerprint = _fingerprint(converted, fps)
    selected_entries = {
        str(entry["episode_id"]): entry.get("split")
        for entry in lineage.get("selection_manifest", {}).get("episodes", [])
        if entry.get("selected")
    }
    splits: dict[str, list[dict[str, Any]]] = {}
    for episode_index, episode in enumerate(converted):
        split = selected_entries.get(episode.episode_id)
        if split is not None:
            splits.setdefault(str(split), []).append(
                {"episode_id": episode.episode_id, "episode_index": episode_index}
            )
    (stage / "act_lab_splits.json").write_text(
        json.dumps({"unit": "episode", "splits": splits}, indent=2, sort_keys=True)
        + "\n"
    )
    lineage.update(
        converter_version=CONVERTER_VERSION,
        dataset_fingerprint=fingerprint,
        fps=fps,
        repo_id=repo_id,
        source_episode_ids=[episode.episode_id for episode in converted],
        storage_fingerprint=storage_fingerprint(stage),
    )
    (stage / "act_lab_lineage.json").write_text(
        json.dumps(lineage, indent=2, sort_keys=True) + "\n"
    )
    os.rename(stage, output)
    return {
        "dataset_fingerprint": fingerprint,
        "episodes": len(converted),
        "frames": sum(len(episode.samples) for episode in converted),
        "output": str(output),
    }
