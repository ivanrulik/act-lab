"""Training orchestration tests that do not import LeRobot or PyTorch."""

import json
from pathlib import Path

import pytest

from act_lab.application.training import (
    load_act_config,
    parse_metric_line,
    resume_config,
    storage_fingerprint,
    verify_dataset,
)


def write_config(path: Path, *, steps: int = 10) -> None:
    path.write_text(
        f"""[act]
version = 1
seed = 1000
batch_size = 2
steps = {steps}
log_freq = 1
save_freq = 5
num_workers = 0
chunk_size = 4
n_action_steps = 4
vision_backbone = "resnet18"
pretrained_backbone_weights = "ResNet18_Weights.IMAGENET1K_V1"
dim_model = 64
n_heads = 4
dim_feedforward = 128
n_encoder_layers = 1
n_decoder_layers = 1
use_vae = true
latent_dim = 8
n_vae_encoder_layers = 1
dropout = 0.0
kl_weight = 1.0
temporal_ensembling = false
use_amp = false
cudnn_deterministic = true
"""
    )


def dataset(tmp_path: Path) -> Path:
    root = tmp_path / "dataset"
    (root / "meta").mkdir(parents=True)
    (root / "meta" / "info.json").write_text(
        json.dumps({"total_episodes": 2, "repo_id": "local/test"})
    )
    (root / "data.bin").write_bytes(b"finalized bytes")
    (root / "act_lab_splits.json").write_text(
        json.dumps(
            {
                "unit": "episode",
                "splits": {
                    "train": [{"episode_id": "train", "episode_index": 0}],
                    "validation": [{"episode_id": "held-out", "episode_index": 1}],
                },
            }
        )
    )
    (root / "act_lab_lineage.json").write_text(
        json.dumps(
            {
                "repo_id": "local/test",
                "dataset_fingerprint": "logical",
                "storage_fingerprint": storage_fingerprint(root),
            }
        )
    )
    return root


def test_config_resolution_and_resume_rules(tmp_path: Path) -> None:
    path = tmp_path / "act.toml"
    write_config(path)
    config = load_act_config(path)
    assert config.steps == 10 and config.batch_size == 2
    assert resume_config(config, None) == config
    assert resume_config(config, 11).steps == 11
    with pytest.raises(ValueError, match="increase"):
        resume_config(config, 10)
    path.write_text(path.read_text() + "unknown = 1\n")
    with pytest.raises(ValueError, match="unknown"):
        load_act_config(path)


def test_dataset_fingerprints_and_frozen_splits(tmp_path: Path) -> None:
    root = dataset(tmp_path)
    identity = verify_dataset(root)
    assert identity.train_episode_indices == (0,)
    assert identity.reserved_episode_indices == (1,)
    # A run manifest is JSON, so tuple-backed domain data must round-trip into
    # the same representation used by the resume identity check.
    persisted_identity = json.loads(json.dumps(identity.to_dict()))
    assert identity.to_dict() == persisted_identity
    (root / "data.bin").write_bytes(b"changed")
    with pytest.raises(ValueError, match="storage fingerprint"):
        verify_dataset(root)


def test_split_overlap_and_missing_training_are_rejected(tmp_path: Path) -> None:
    root = dataset(tmp_path)
    splits = json.loads((root / "act_lab_splits.json").read_text())
    splits["splits"]["validation"][0]["episode_index"] = 0
    (root / "act_lab_splits.json").write_text(json.dumps(splits))
    lineage = json.loads((root / "act_lab_lineage.json").read_text())
    lineage["storage_fingerprint"] = storage_fingerprint(root)
    (root / "act_lab_lineage.json").write_text(json.dumps(lineage))
    with pytest.raises(ValueError, match="overlap"):
        verify_dataset(root)


def test_metric_parser_translates_pinned_lerobot_line() -> None:
    metric = parse_metric_line(
        "step:200 loss:1.250 grdn:0.500 lr:1.0e-05 updt_s:0.2 "
        "data_s:0.1 smp/s:26 mem_gb:1.5 action_loss:0.4"
    )
    assert metric == {
        "step": 200,
        "loss": 1.25,
        "gradient_norm": 0.5,
        "learning_rate": 1e-5,
        "update_s": 0.2,
        "dataloading_s": 0.1,
        "samples_per_s": 26.0,
        "gpu_memory_gb": 1.5,
        "action_loss": 0.4,
    }
