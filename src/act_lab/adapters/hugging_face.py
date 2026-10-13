"""Optional, explicit Hub publication and commit-pinned download."""

from __future__ import annotations

import json
from importlib import import_module
from pathlib import Path
from typing import Any

from act_lab.application.release import verify_package, write_json


def publish(root: Path, *, public: bool) -> dict[str, Any]:
    manifest = verify_package(root)
    hub = import_module("huggingface_hub")
    api = hub.HfApi()
    api.whoami()  # Fail before creating anything if credentials are missing.
    repo_id, kind = manifest["repo_id"], manifest["kind"]
    api.create_repo(repo_id, repo_type=kind, private=not public, exist_ok=True)
    info = api.repo_info(repo_id, repo_type=kind)
    if info.private != (not public):
        raise ValueError("existing Hub repository visibility differs")
    # Refuse to mix a release with existing/unreviewed files. Retrying the same
    # package is safe; a changed release needs a new repository name.
    remote = set(api.list_repo_files(repo_id, repo_type=kind)) - {".gitattributes"}
    local = set(manifest["files"]) | {"release.json"}
    if remote and remote != local:
        raise ValueError("existing Hub repository inventory differs")
    if remote:
        old = hub.hf_hub_download(repo_id, "release.json", repo_type=kind)
        if json.loads(Path(old).read_text()) != manifest:
            raise ValueError("existing Hub repository is a different release")
    commit = api.upload_folder(
        repo_id=repo_id,
        repo_type=kind,
        folder_path=str(root),
        allow_patterns=sorted(local),
        commit_message="Publish verified ACT Lab release",
    )
    receipt = {
        "schema_version": 1,
        "repo_id": repo_id,
        "kind": kind,
        "revision": commit.oid,
        "public": public,
    }
    write_json(root.parent / f"{kind}-receipt.json", receipt)
    return receipt


def download(receipt_path: Path, output: Path) -> dict[str, Any]:
    receipt = json.loads(receipt_path.read_text())
    if output.exists():
        raise ValueError("download output already exists; use a clean directory")
    hub = import_module("huggingface_hub")
    hub.snapshot_download(
        receipt["repo_id"],
        repo_type=receipt["kind"],
        revision=receipt["revision"],
        local_dir=str(output),
    )
    verify_package(output)
    return {"status": "verified", "output": str(output), **receipt}
