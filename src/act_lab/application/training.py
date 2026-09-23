"""Resolve training configuration, verify datasets, and manage run metadata."""

from __future__ import annotations

import hashlib
import json
import os
import platform
import re
import subprocess
import tomllib
from dataclasses import replace
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from act_lab.domain.training import ActTrainingConfig, DatasetIdentity

RUN_MANIFEST = "act_lab_run.json"
METRICS_FILE = "metrics.jsonl"
TRAIN_LOG = "train.log"


def utc_now() -> str:
    return datetime.now(UTC).isoformat()


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    partial = path.with_name(path.name + ".partial")
    partial.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    os.replace(partial, path)


def load_act_config(path: Path) -> ActTrainingConfig:
    try:
        raw = tomllib.loads(path.read_text())
    except (OSError, tomllib.TOMLDecodeError) as error:
        raise ValueError(
            f"cannot read training configuration {path}: {error}"
        ) from error
    allowed = set(ActTrainingConfig.__dataclass_fields__)
    values = raw.get("act")
    if not isinstance(values, dict):
        raise ValueError("training configuration requires an [act] table")
    unknown = set(values) - allowed
    missing = allowed - set(values)
    if unknown:
        raise ValueError(
            "unknown ACT configuration keys: " + ", ".join(sorted(unknown))
        )
    if missing:
        raise ValueError(
            "missing ACT configuration keys: " + ", ".join(sorted(missing))
        )
    try:
        config = ActTrainingConfig(**values)
    except TypeError as error:
        raise ValueError(f"invalid ACT configuration: {error}") from error
    if config.version != 1:
        raise ValueError(f"unsupported ACT configuration version: {config.version}")
    if config.pretrained_backbone_weights == "":
        config = replace(config, pretrained_backbone_weights=None)
    positive = (
        "batch_size",
        "steps",
        "log_freq",
        "save_freq",
        "chunk_size",
        "n_action_steps",
    )
    if any(getattr(config, name) <= 0 for name in positive) or config.num_workers < 0:
        raise ValueError(
            "step, batch, frequency and chunk values must be positive; "
            "workers may be zero"
        )
    if config.n_action_steps > config.chunk_size:
        raise ValueError("n_action_steps cannot exceed chunk_size")
    if (
        config.dim_model <= 0
        or config.n_heads <= 0
        or config.dim_model % config.n_heads
        or config.dim_feedforward <= 0
        or config.n_encoder_layers <= 0
        or config.n_decoder_layers <= 0
        or config.latent_dim <= 0
        or config.n_vae_encoder_layers <= 0
        or not 0.0 <= config.dropout < 1.0
        or config.kl_weight < 0.0
    ):
        raise ValueError("ACT architecture values are invalid")
    if config.vision_backbone != "resnet18":
        raise ValueError("PR 8 supports the upstream ACT ResNet-18 recipe only")
    if config.temporal_ensembling:
        raise ValueError("temporal ensembling is disabled for the PR 8 recipe")
    if config.use_amp:
        raise ValueError("AMP must remain disabled for the reproducible PR 8 recipe")
    return config


def storage_fingerprint(root: Path) -> str:
    """Hash finalized dataset bytes, excluding ACT Lab's self-referential lineage."""
    digest = hashlib.sha256()
    excluded = {"act_lab_lineage.json", "act_lab_lineage.json.partial"}
    files = sorted(
        path for path in root.rglob("*") if path.is_file() and path.name not in excluded
    )
    if not files:
        raise ValueError(f"dataset has no finalized storage files: {root}")
    for path in files:
        relative = path.relative_to(root).as_posix()
        digest.update(relative.encode() + b"\0")
        with path.open("rb") as source:
            while chunk := source.read(1024 * 1024):
                digest.update(chunk)
    return digest.hexdigest()


def _dataset_episode_count(root: Path) -> int:
    info = root / "meta" / "info.json"
    if info.is_file():
        value = json.loads(info.read_text()).get("total_episodes")
        if isinstance(value, int):
            return value
    episodes = root / "meta" / "episodes.jsonl"
    if episodes.is_file():
        return sum(bool(line.strip()) for line in episodes.read_text().splitlines())
    raise ValueError("dataset metadata does not declare its episode count")


def _dataset_repo_id(root: Path, lineage: dict[str, Any]) -> str:
    value = lineage.get("repo_id")
    if isinstance(value, str) and value:
        return value
    info = root / "meta" / "info.json"
    if info.is_file():
        candidate = json.loads(info.read_text()).get("repo_id")
        if isinstance(candidate, str) and candidate:
            return candidate
    raise ValueError("dataset lineage does not record repo_id; regenerate it with PR 8")


def verify_dataset(root: Path) -> DatasetIdentity:
    root = root.resolve()
    lineage_path = root / "act_lab_lineage.json"
    splits_path = root / "act_lab_splits.json"
    if not lineage_path.is_file() or not splits_path.is_file():
        raise ValueError(
            "dataset requires act_lab_lineage.json and act_lab_splits.json"
        )
    lineage = json.loads(lineage_path.read_text())
    logical = lineage.get("dataset_fingerprint")
    expected_storage = lineage.get("storage_fingerprint")
    if not isinstance(logical, str) or not logical:
        raise ValueError("dataset logical fingerprint is missing")
    actual_storage = storage_fingerprint(root)
    if not isinstance(expected_storage, str) or expected_storage != actual_storage:
        raise ValueError(
            "dataset storage fingerprint does not match; regenerate or restore "
            "the dataset"
        )
    episode_count = _dataset_episode_count(root)
    splits = json.loads(splits_path.read_text())
    if splits.get("unit") != "episode" or not isinstance(splits.get("splits"), dict):
        raise ValueError("dataset splits must use episode units")
    parsed: dict[str, list[tuple[str, int]]] = {}
    all_indices: list[int] = []
    all_ids: list[str] = []
    for split_name in ("train", "validation"):
        entries = splits["splits"].get(split_name, [])
        if not isinstance(entries, list):
            raise ValueError(f"{split_name} split must be a list")
        parsed[split_name] = []
        for entry in entries:
            if (
                not isinstance(entry, dict)
                or not isinstance(entry.get("episode_id"), str)
                or not isinstance(entry.get("episode_index"), int)
            ):
                raise ValueError(f"invalid {split_name} split entry")
            episode_id, index = entry["episode_id"], entry["episode_index"]
            if not 0 <= index < episode_count:
                raise ValueError(f"{split_name} episode index {index} is out of range")
            parsed[split_name].append((episode_id, index))
            all_indices.append(index)
            all_ids.append(episode_id)
    if not parsed["train"]:
        raise ValueError("dataset must contain at least one training episode")
    if len(all_indices) != len(set(all_indices)) or len(all_ids) != len(set(all_ids)):
        raise ValueError("training and validation splits overlap or contain duplicates")
    return DatasetIdentity(
        str(root),
        _dataset_repo_id(root, lineage),
        logical,
        actual_storage,
        tuple(item[0] for item in parsed["train"]),
        tuple(item[1] for item in parsed["train"]),
        tuple(item[0] for item in parsed["validation"]),
        tuple(item[1] for item in parsed["validation"]),
        episode_count,
    )


def git_identity(root: Path) -> dict[str, object]:
    def run(*args: str) -> str | None:
        result = subprocess.run(
            args, cwd=root, text=True, capture_output=True, check=False
        )
        return result.stdout.strip() if result.returncode == 0 else None

    revision = run("git", "rev-parse", "HEAD")
    status = run("git", "status", "--porcelain")
    return {"revision": revision, "dirty": bool(status) if status is not None else None}


def environment_identity() -> dict[str, object]:
    packages: dict[str, str] = {}
    try:
        from importlib.metadata import version

        for name in ("act-lab", "lerobot", "numpy", "torch", "torchvision"):
            try:
                packages[name] = version(name)
            except Exception:  # package metadata is diagnostic only
                packages[name] = "unavailable"
    except ImportError:
        pass
    cuda: dict[str, object] = {"available": False}
    try:
        import torch  # type: ignore[import-not-found]

        cuda = {
            "available": torch.cuda.is_available(),
            "runtime": torch.version.cuda,
            "device_count": torch.cuda.device_count(),
            "devices": [
                torch.cuda.get_device_name(i) for i in range(torch.cuda.device_count())
            ],
        }
    except ImportError:
        pass
    return {
        "python": platform.python_version(),
        "platform": platform.platform(),
        "packages": packages,
        "cuda": cuda,
    }


def preflight_device(device: str) -> None:
    if device == "cpu":
        return
    if device != "cuda":
        raise ValueError(f"unsupported device: {device}")
    try:
        import torch
    except ImportError as error:
        raise RuntimeError("CUDA was requested but PyTorch is not installed") from error
    if not torch.cuda.is_available() or torch.cuda.device_count() < 1:
        raise RuntimeError(
            "CUDA was requested but no CUDA device is available; CPU fallback "
            "is forbidden"
        )


_METRIC = re.compile(
    r"(?:^|\s)([A-Za-z_][\w/]*)\s*:\s*([-+]?\d+(?:\.\d+)?(?:e[-+]?\d+)?)", re.I
)
_ALIASES = {
    "grdn": "gradient_norm",
    "lr": "learning_rate",
    "updt_s": "update_s",
    "data_s": "dataloading_s",
    "smp/s": "samples_per_s",
    "mem_gb": "gpu_memory_gb",
    "step": "step",
}


def parse_metric_line(message: str) -> dict[str, int | float] | None:
    values: dict[str, int | float] = {}
    for key, raw in _METRIC.findall(message):
        key = _ALIASES.get(key, key)
        value = float(raw)
        values[key] = int(value) if key == "step" and value.is_integer() else value
    return values if "step" in values and "loss" in values else None


def resume_config(config: ActTrainingConfig, steps: int | None) -> ActTrainingConfig:
    if steps is None:
        return config
    if steps <= config.steps:
        raise ValueError(f"resume steps must increase beyond {config.steps}")
    return replace(config, steps=steps)
