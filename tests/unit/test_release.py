"""Release rejects artifact drift before any remote publication."""

import json
import shutil
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from act_lab.application.release import (
    checkpoint_hash,
    file_hash,
    stage,
    verify_package,
    verify_presentation_update,
    write_json,
)
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


@pytest.fixture
def paired_recipe(staged: Path) -> Path:
    root = staged.parent
    data = root / "data"
    (data / "meta/info.json").write_text('{"total_episodes": 1, "fps": 25}')
    lineage_path = data / "act_lab_lineage.json"
    lineage = json.loads(lineage_path.read_text())
    lineage.update(storage_fingerprint=storage_fingerprint(data), model_contract={})
    lineage_path.write_text(json.dumps(lineage))
    (root / "model/act_lab_model.json").write_text("{}")
    (root / "training.json").write_text(
        json.dumps(
            {
                "status": "completed",
                "dataset": {
                    "logical_fingerprint": "logical",
                    "storage_fingerprint": lineage["storage_fingerprint"],
                    "train_episode_indices": [0],
                    "reserved_episode_indices": [],
                },
            }
        )
    )
    (root / "evaluation.json").write_text(
        json.dumps(
            {
                "policy": {
                    "checkpoint_sha256": checkpoint_hash(root / "model"),
                    "policy_hz": 25,
                    "act_n_action_steps": 1,
                    "act_temporal_ensemble_coeff": 0.01,
                }
            }
        )
    )
    (root / "simulation.toml").write_text("# Frozen simulation configuration\n")
    (root / "reproduce.md").write_text("# Reproduction\n")
    (root / "example.mcap").write_bytes(b"fixture replay")
    config = root / "release.toml"
    with config.open("a") as stream:
        stream.write("""
asset_notices = ["license.txt"]
reproduction_files = ["reproduce.md"]
model_examples = ["example.mcap"]
[release.inference]
simulation_config = "simulation.toml"
policy_hz = 25
n_action_steps = 1
temporal_ensemble_coeff = 0.01
""")
    return config


def test_paired_release_preserves_recipe_and_model_only_examples(
    paired_recipe: Path,
) -> None:
    output = paired_recipe.parent / "paired"
    stage(paired_recipe, output)
    for kind in ("dataset", "model"):
        manifest = verify_package(output / kind)
        assert manifest["inference"]["temporal_ensemble_coeff"] == 0.01
        assert (output / kind / "reproduction/simulation.toml").is_file()
        assert (output / kind / "evidence/qualification.json").is_file()
        assert list((output / kind / "notices").iterdir())
    assert (output / "model/examples/example.mcap").read_bytes() == b"fixture replay"
    assert not (output / "dataset/examples").exists()


@pytest.mark.parametrize(
    "field,value",
    [
        ("policy_hz", 50),
        ("act_n_action_steps", 100),
        ("act_temporal_ensemble_coeff", None),
    ],
)
def test_paired_inference_must_match_evaluation(
    paired_recipe: Path, field, value
) -> None:
    path = paired_recipe.parent / "evaluation.json"
    evaluation = json.loads(path.read_text())
    evaluation["policy"][field] = value
    path.write_text(json.dumps(evaluation))
    with pytest.raises(ValueError, match="differs from evaluation"):
        stage(paired_recipe, paired_recipe.parent / "bad-inference")


@pytest.mark.parametrize("change", ["model_contract", "split", "incomplete"])
def test_paired_lineage_must_match(paired_recipe: Path, change: str) -> None:
    root = paired_recipe.parent
    if change == "model_contract":
        path = root / "data/act_lab_lineage.json"
        value = json.loads(path.read_text())
        value["model_contract"] = {"wrong": True}
    else:
        path = root / "training.json"
        value = json.loads(path.read_text())
        if change == "split":
            value["dataset"]["train_episode_indices"] = [1]
        else:
            value["status"] = "running"
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError):
        stage(paired_recipe, root / "bad-lineage")


@pytest.mark.parametrize(
    "change", ["weights", "identity", "remove", "addition", "size"]
)
def test_presentation_update_protects_frozen_release(staged: Path, change: str) -> None:
    old = verify_package(staged / "model")
    new = json.loads(json.dumps(old))
    if change == "weights":
        new["files"]["payload/model.safetensors"]["sha256"] = "changed"
    elif change == "identity":
        new["checkpoint_sha256"] = "changed"
    elif change == "remove":
        del new["files"]["LICENSE"]
    else:
        name = "unreviewed.txt" if change == "addition" else "media/demo.mp4"
        new["files"][name] = {"sha256": "added", "bytes": 21 * 1024 * 1024}
    with pytest.raises(ValueError):
        verify_presentation_update(old, new)


@pytest.mark.parametrize("stale", [False, True])
def test_public_promotion_commits_before_visibility(staged: Path, monkeypatch, stale):
    from act_lab.adapters import hugging_face

    package = staged / "model"
    old = verify_package(package)
    old_path = staged / "baseline.json"
    write_json(old_path, old)
    (package / "README.md").write_text("# Public card\n")
    media = package / "media/demo.mp4"
    media.parent.mkdir()
    media.write_bytes(b"preview")
    new = json.loads(json.dumps(old))
    for path in (package / "README.md", media):
        new["files"][path.relative_to(package).as_posix()] = {
            "sha256": file_hash(path),
            "bytes": path.stat().st_size,
        }
    write_json(package / "release.json", new)
    receipt = staged / "baseline-receipt.json"
    write_json(
        receipt,
        {
            "schema_version": 1,
            "kind": "model",
            "repo_id": "test/model",
            "revision": "a" * 40,
            "public": False,
        },
    )
    calls = []

    class API:
        head = "c" * 40 if stale else "a" * 40
        private = True

        def whoami(self):
            return {"name": "test"}

        def repo_info(self, *args, **kwargs):
            return SimpleNamespace(sha=self.head, private=self.private)

        def list_repo_files(self, *args, **kwargs):
            current = new if self.head == "b" * 40 else old
            return list(current["files"]) + ["release.json", ".gitattributes"]

        def create_commit(self, *args, **kwargs):
            assert kwargs["parent_commit"] == "a" * 40
            assert {op.path_in_repo for op in kwargs["operations"]} == {
                "README.md",
                "media/demo.mp4",
                "release.json",
            }
            calls.append("commit")
            self.head = "b" * 40
            return SimpleNamespace(oid=self.head)

        def update_repo_settings(self, *args, **kwargs):
            assert calls == ["commit"]
            assert kwargs["private"] is False
            calls.append("public")
            self.private = False

    monkeypatch.setattr(
        hugging_face,
        "import_module",
        lambda _: SimpleNamespace(
            HfApi=API,
            hf_hub_download=lambda *args, **kwargs: old_path,
            CommitOperationAdd=lambda **kwargs: SimpleNamespace(**kwargs),
        ),
    )
    if stale:
        with pytest.raises(ValueError, match="remote head changed"):
            hugging_face.promote(package, receipt)
        assert not calls
    else:
        result = hugging_face.promote(package, receipt)
        assert calls == ["commit", "public"]
        assert result["public"] and result["supersedes_revision"] == "a" * 40
        assert (staged / "model-public-receipt.json").is_file()
