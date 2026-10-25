"""Optional, explicit Hub publication and commit-pinned download."""

from __future__ import annotations

import json
import re
from importlib import import_module
from pathlib import Path
from typing import Any

from act_lab.application.release import (
    verify_package,
    verify_presentation_update,
    write_json,
)


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
    if receipt.get("schema_version") != 1 or not re.fullmatch(
        r"[0-9a-f]{40}", receipt.get("revision", "")
    ):
        raise ValueError("download requires a versioned receipt with a full commit SHA")
    if output.exists():
        raise ValueError("download output already exists; use a clean directory")
    hub = import_module("huggingface_hub")
    hub.snapshot_download(
        receipt["repo_id"],
        repo_type=receipt["kind"],
        revision=receipt["revision"],
        local_dir=str(output),
    )
    manifest = verify_package(output)
    if (manifest["repo_id"], manifest["kind"]) != (receipt["repo_id"], receipt["kind"]):
        raise ValueError("downloaded release does not match the receipt repository")
    return {"status": "verified", "output": str(output), **receipt}


def promote(root: Path, receipt_path: Path) -> dict[str, Any]:
    """Commit a verified presentation edition, then explicitly make it public."""
    manifest = verify_package(root)
    baseline = json.loads(receipt_path.read_text())
    if baseline.get("schema_version") != 1 or not re.fullmatch(
        r"[0-9a-f]{40}", baseline.get("revision", "")
    ):
        raise ValueError("promotion requires a commit-pinned baseline receipt")
    kind, repo_id = manifest["kind"], manifest["repo_id"]
    if (baseline["kind"], baseline["repo_id"]) != (kind, repo_id):
        raise ValueError("promotion receipt repository differs")
    hub = import_module("huggingface_hub")
    api = hub.HfApi()
    api.whoami()

    def remote_manifest(revision: str) -> dict[str, Any]:
        path = hub.hf_hub_download(
            repo_id, "release.json", repo_type=kind, revision=revision
        )
        value: dict[str, Any] = json.loads(Path(path).read_text())
        return value

    old = remote_manifest(baseline["revision"])
    verify_presentation_update(old, manifest)
    head = api.repo_info(repo_id, repo_type=kind).sha
    if head == baseline["revision"]:
        remote = set(api.list_repo_files(repo_id, repo_type=kind, revision=head))
        if remote - {".gitattributes"} != set(old["files"]) | {"release.json"}:
            raise ValueError("remote baseline inventory differs")
        changed = {
            name
            for name, record in manifest["files"].items()
            if old["files"].get(name) != record
        } | {"release.json"}
        commit = api.create_commit(
            repo_id,
            repo_type=kind,
            parent_commit=head,
            operations=[
                hub.CommitOperationAdd(
                    path_in_repo=name, path_or_fileobj=str(root / name)
                )
                for name in sorted(changed)
            ],
            commit_message="Publish ACT Lab presentation edition",
        )
        revision = commit.oid
    elif remote_manifest(head) == manifest:
        revision = head  # Retry after a successful commit but interrupted promotion.
    else:
        raise ValueError("remote head changed since the baseline receipt")
    if api.repo_info(repo_id, repo_type=kind).sha != revision:
        raise ValueError("remote head changed before public promotion")
    remote = set(api.list_repo_files(repo_id, repo_type=kind, revision=revision))
    if remote - {".gitattributes"} != set(manifest["files"]) | {"release.json"}:
        raise ValueError("presentation inventory differs before public promotion")
    api.update_repo_settings(repo_id, repo_type=kind, private=False)
    if api.repo_info(repo_id, repo_type=kind).private:
        raise ValueError("repository did not become public")
    receipt = {
        "schema_version": 1,
        "kind": kind,
        "repo_id": repo_id,
        "revision": revision,
        "public": True,
        "supersedes_revision": baseline["revision"],
    }
    write_json(root.parent / f"{kind}-public-receipt.json", receipt)
    return receipt
