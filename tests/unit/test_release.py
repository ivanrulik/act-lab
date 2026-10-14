"""Release rejects artifact drift before any remote publication."""

import json
import shutil
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from act_lab.application.release import checkpoint_hash, stage, verify_package
from act_lab.application.training import storage_fingerprint


@pytest.fixture
def staged(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    monkeypatch.chdir(tmp_path)
    data = tmp_path / "data"
    (data / "meta").mkdir(parents=True)
    (data / "meta/info.json").write_text('{"total_episodes": 1}')
    (data / "act_lab_splits.json").write_text(
        json.dumps(
            {
                "unit": "episode",
                "splits": {"train": [{"episode_id": "one", "episode_index": 0}]},
            }
        )
    )
    (data / "act_lab_lineage.json").write_text(
        json.dumps(
            {
                "dataset_fingerprint": "logical",
                "storage_fingerprint": storage_fingerprint(data),
                "repo_id": "local/test",
            }
        )
    )
    model = tmp_path / "model"
    model.mkdir()
    (model / "model.safetensors").write_bytes(b"test weights")
    (tmp_path / "evaluation.json").write_text(
        json.dumps({"policy": {"checkpoint_sha256": checkpoint_hash(model)}})
    )
    (tmp_path / "sidecar.json").write_text('{"path": "/home/private/checkpoint"}')
    (tmp_path / "training.json").write_text(
        json.dumps(
            {
                "dataset": {
                    "logical_fingerprint": "logical",
                    "storage_fingerprint": storage_fingerprint(data),
                }
            }
        )
    )
    (tmp_path / "card.md").write_text("# Card\n")
    (tmp_path / "license.txt").write_text("Fixture license\n")
    config = tmp_path / "release.toml"
    config.write_text("""[release]
dataset = "data"
checkpoint = "model"
evaluation = "evaluation.json"
training_manifest = "training.json"
dataset_fingerprint = "logical"
episodes = 1
dataset_repo = "test/dataset"
model_repo = "test/model"
dataset_card = "card.md"
model_card = "card.md"
asset_notice = "license.txt"
license_file = "license.txt"
evidence = ["sidecar.json"]
""")
    output = tmp_path / "staged"
    stage(config, output)
    return output


def test_staging_preserves_identity_and_redacts_sidecars(staged: Path) -> None:
    assert verify_package(staged / "dataset")["kind"] == "dataset"
    assert verify_package(staged / "model")["kind"] == "model"
    sidecar = next((staged / "model/evidence").glob("*.json"))
    assert "/home/private" not in sidecar.read_text()
    # A clean copy remains independent of the original source inputs.
    downloaded = staged.parent / "downloaded"
    shutil.copytree(staged / "dataset", downloaded)
    shutil.rmtree(staged.parent / "data")
    assert verify_package(downloaded)["kind"] == "dataset"


@pytest.mark.parametrize("change", ["weights", "extra", "missing", "symlink"])
def test_release_inventory_rejects_drift(staged: Path, change: str) -> None:
    package = staged / "model"
    weights = package / "payload/model.safetensors"
    if change == "weights":
        weights.write_bytes(b"different weights")
    elif change == "extra":
        (package / "unexpected.txt").write_text("unexpected")
    elif change == "missing":
        weights.unlink()
    else:
        (package / "link").symlink_to(weights)
    with pytest.raises(ValueError):
        verify_package(package)


def test_mismatched_evaluation_and_existing_output_fail(staged: Path) -> None:
    with pytest.raises(ValueError, match="already exists"):
        stage(staged.parent / "release.toml", staged)
    (staged.parent / "evaluation.json").write_text(
        '{"policy": {"checkpoint_sha256": "wrong"}}'
    )
    with pytest.raises(ValueError, match="does not match"):
        stage(staged.parent / "release.toml", staged.parent / "other")


def test_publish_only_inventory_and_private_default(
    staged: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from act_lab.adapters import hugging_face

    calls = []

    class API:
        def whoami(self):
            return {"name": "test"}

        def create_repo(self, *args, **kwargs):
            assert kwargs["private"]

        def repo_info(self, *args, **kwargs):
            return SimpleNamespace(private=True)

        def list_repo_files(self, *args, **kwargs):
            return [".gitattributes"]

        def upload_folder(self, **kwargs):
            calls.append(kwargs)
            return SimpleNamespace(oid="a" * 40)

    monkeypatch.setattr(
        hugging_face, "import_module", lambda _: SimpleNamespace(HfApi=API)
    )
    package = staged / "model"
    receipt = hugging_face.publish(package, public=False)
    assert receipt["revision"] == "a" * 40
    expected = set(verify_package(package)["files"]) | {"release.json"}
    assert set(calls[0]["allow_patterns"]) == expected
    assert (staged / "model-receipt.json").is_file()


def test_hub_import_is_lazy() -> None:
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; import act_lab.application.release; "
            "import act_lab.adapters.hugging_face; "
            "assert 'huggingface_hub' not in sys.modules; "
            "assert 'rclpy' not in sys.modules",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_download_pins_commit_and_verifies(staged: Path, monkeypatch) -> None:
    from act_lab.adapters import hugging_face

    receipt = staged / "receipt.json"
    receipt.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "kind": "model",
                "repo_id": "test/model",
                "revision": "a" * 40,
                "public": False,
            }
        )
    )

    def snapshot(repo_id, **kwargs):
        assert repo_id == "test/model"
        assert kwargs["revision"] == "a" * 40
        shutil.copytree(staged / "model", kwargs["local_dir"])

    monkeypatch.setattr(
        hugging_face,
        "import_module",
        lambda _: SimpleNamespace(snapshot_download=snapshot),
    )
    output = staged.parent / "remote-copy"
    assert hugging_face.download(receipt, output)["status"] == "verified"
    receipt.write_text(json.dumps({"schema_version": 1, "revision": "main"}))
    with pytest.raises(ValueError, match="full commit SHA"):
        hugging_face.download(receipt, staged.parent / "other-copy")
