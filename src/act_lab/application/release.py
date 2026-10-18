"""Curate immutable learning artifacts without loading Hub or ROS packages."""

from __future__ import annotations

import hashlib
import json
import math
import shutil
import tomllib
from pathlib import Path
from typing import Any

from act_lab.application.training import verify_dataset


def file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        while chunk := source.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def files(root: Path) -> list[Path]:
    if root.is_symlink():
        raise ValueError("release root must not be a symlink")
    paths = sorted(root.rglob("*"))
    if any(p.is_symlink() for p in paths):
        raise ValueError("release inputs must not contain symlinks")
    return [p for p in paths if p.is_file()]


def checkpoint_hash(root: Path) -> str:
    digest = hashlib.sha256()
    for path in files(root):
        digest.update(path.relative_to(root).as_posix().encode())
        with path.open("rb") as source:
            while chunk := source.read(1024 * 1024):
                digest.update(chunk)
    return digest.hexdigest()


def sanitized(value: Any) -> Any:
    """Redact absolute paths from public sidecars; frozen payloads stay intact."""
    if isinstance(value, dict):
        return {k: sanitized(v) for k, v in value.items()}
    if isinstance(value, list):
        return [sanitized(v) for v in value]
    if isinstance(value, str) and value.startswith("/"):
        return "<original-path>/" + Path(value).name
    return value


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def verify_inference(root: Path, manifest: dict[str, Any]) -> None:
    """Bind explicit inference settings to the evaluated, unchanged checkpoint."""
    if "inference" not in manifest:
        return  # Historical releases predate explicit inference metadata.
    inference = json.loads((root / "inference.json").read_text())
    if inference != manifest["inference"]:
        raise ValueError("inference metadata differs from release manifest")
    for key in ("evaluation", "simulation_config"):
        if inference[key] not in manifest["files"]:
            raise ValueError("inference references a file outside the inventory")
    evaluation = json.loads((root / inference["evaluation"]).read_text())
    policy = evaluation["policy"]
    for key, recorded in (
        ("policy_hz", "policy_hz"),
        ("n_action_steps", "act_n_action_steps"),
        ("temporal_ensemble_coeff", "act_temporal_ensemble_coeff"),
    ):
        if inference[key] != policy[recorded]:
            raise ValueError(f"inference {key} differs from evaluation")
    coefficient = inference["temporal_ensemble_coeff"]
    if (
        type(inference["policy_hz"]) is not int
        or inference["policy_hz"] <= 0
        or type(inference["n_action_steps"]) is not int
        or inference["n_action_steps"] != 1
        or (
            coefficient is not None
            and (
                type(coefficient) not in (int, float)
                or not math.isfinite(coefficient)
                or coefficient < 0
            )
        )
    ):
        raise ValueError("invalid explicit inference settings")
    if policy["checkpoint_sha256"] != manifest["checkpoint_sha256"]:
        raise ValueError("inference evaluation checkpoint differs")
    config_path = root / inference["simulation_config"]
    if file_hash(config_path) != inference["simulation_config_sha256"]:
        raise ValueError("inference simulation configuration differs")
    if manifest["kind"] == "dataset":
        info = json.loads((root / "payload/meta/info.json").read_text())
        if info["fps"] != inference["policy_hz"]:
            raise ValueError("inference rate differs from dataset FPS")


def verify_package(root: Path) -> dict[str, Any]:
    manifest: dict[str, Any] = json.loads((root / "release.json").read_text())
    if manifest.get("schema_version") != 1:
        raise ValueError("unsupported release manifest")
    expected = manifest["files"]
    actual = {
        p.relative_to(root).as_posix(): p
        for p in files(root)
        if p != root / "release.json"
        and ".cache" not in p.parts
        and p.name != ".gitattributes"
    }
    if set(actual) != set(expected):
        raise ValueError("release file inventory differs")
    for name, record in expected.items():
        path = actual[name]
        if (
            path.stat().st_size != record["bytes"]
            or file_hash(path) != record["sha256"]
        ):
            raise ValueError(f"release bytes changed: {name}")
    if manifest["kind"] == "dataset":
        identity = verify_dataset(root / "payload")
        if identity.storage_fingerprint != manifest["storage_fingerprint"]:
            raise ValueError("downloaded dataset identity differs")
    elif manifest["kind"] == "model":
        if checkpoint_hash(root / "payload") != manifest["checkpoint_sha256"]:
            raise ValueError("downloaded checkpoint identity differs")
    else:
        raise ValueError("unknown release kind")
    verify_inference(root, manifest)
    return manifest


def stage(config: Path, output: Path) -> dict[str, Any]:
    settings = tomllib.loads(config.read_text())["release"]
    dataset = Path(settings["dataset"])
    checkpoint = Path(settings["checkpoint"])
    # Preflight before creating output. Uploads never discover arbitrary run files.
    files(dataset)
    files(checkpoint)
    identity = verify_dataset(dataset)
    training = json.loads(Path(settings["training_manifest"]).read_text())
    if (
        training["dataset"]["logical_fingerprint"] != identity.logical_fingerprint
        or training["dataset"]["storage_fingerprint"] != identity.storage_fingerprint
    ):
        raise ValueError("dataset does not match the training run")
    evaluation = json.loads(Path(settings["evaluation"]).read_text())
    if checkpoint_hash(checkpoint) != evaluation["policy"]["checkpoint_sha256"]:
        raise ValueError("checkpoint does not match evaluated artifact")
    if identity.logical_fingerprint != settings["dataset_fingerprint"]:
        raise ValueError("unexpected dataset identity")
    info = json.loads((dataset / "meta/info.json").read_text())
    if info["total_episodes"] != settings["episodes"]:
        raise ValueError("unexpected episode count")
    inference = None
    if "inference" in settings:
        requested = settings["inference"]
        simulation = Path(requested["simulation_config"])
        inference = {
            "schema_version": 1,
            "policy_hz": requested["policy_hz"],
            "n_action_steps": requested["n_action_steps"],
            "temporal_ensemble_coeff": requested["temporal_ensemble_coeff"],
            "simulation_config": f"reproduction/{simulation.name}",
            "simulation_config_sha256": file_hash(simulation),
            "evaluation": "evidence/qualification.json",
        }
        lineage = json.loads((dataset / "act_lab_lineage.json").read_text())
        recorded_contract = json.loads((checkpoint / "act_lab_model.json").read_text())
        if recorded_contract != lineage.get("model_contract"):
            raise ValueError("checkpoint and dataset model contracts differ")
        if training.get("status") != "completed":
            raise ValueError("release training run must be completed")
        for key, actual in (
            ("train_episode_indices", identity.train_episode_indices),
            ("reserved_episode_indices", identity.reserved_episode_indices),
        ):
            if list(actual) != training["dataset"][key]:
                raise ValueError("release training episode split differs")
    if output.exists():
        raise ValueError("release output already exists; use a new path")
    output.mkdir(parents=True)
    for kind, source in (("dataset", dataset), ("model", checkpoint)):
        target = output / kind
        shutil.copytree(source, target / "payload")
        card = Path(settings[f"{kind}_card"]).read_text()
        (target / "README.md").write_text(card)
        shutil.copy2(settings["asset_notice"], target / "ASSET-NOTICE.txt")
        shutil.copy2(settings["license_file"], target / "LICENSE")
        for notice in settings.get("asset_notices", []):
            path = Path(notice)
            destination = target / "notices" / f"{path.parent.name}-{path.name}"
            destination.parent.mkdir(exist_ok=True)
            shutil.copy2(path, destination)
        for source_path in settings.get("reproduction_files", []):
            path = Path(source_path)
            destination = target / "reproduction" / path.name
            destination.parent.mkdir(exist_ok=True)
            if destination.exists():
                raise ValueError("duplicate reproduction filename")
            shutil.copy2(path, destination)
        if kind == "model":
            for source_path in settings.get("model_examples", []):
                path = Path(source_path)
                destination = target / "examples" / path.name
                destination.parent.mkdir(exist_ok=True)
                if destination.exists():
                    raise ValueError("duplicate example filename")
                shutil.copy2(path, destination)
        for evidence in settings["evidence"]:
            path = Path(evidence)
            value = json.loads(path.read_text())
            write_json(
                target / "evidence" / f"{path.parent.name}-{path.name}",
                sanitized(value),
            )
        if inference is not None:
            simulation = Path(settings["inference"]["simulation_config"])
            destination = target / inference["simulation_config"]
            destination.parent.mkdir(exist_ok=True)
            shutil.copy2(simulation, destination)
            write_json(target / "inference.json", inference)
            write_json(target / inference["evaluation"], sanitized(evaluation))
        manifest: dict[str, Any] = {
            "schema_version": 1,
            "kind": kind,
            "repo_id": settings[f"{kind}_repo"],
            "storage_fingerprint": identity.storage_fingerprint,
            "checkpoint_sha256": evaluation["policy"]["checkpoint_sha256"],
            "files": {
                p.relative_to(target).as_posix(): {
                    "bytes": p.stat().st_size,
                    "sha256": file_hash(p),
                }
                for p in files(target)
            },
        }
        if inference is not None:
            manifest["inference"] = inference
        write_json(target / "release.json", manifest)
        verify_package(target)
    return {"output": str(output), "status": "verified", "schema_version": 1}
