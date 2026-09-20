"""Framework-neutral contracts for reproducible policy training."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

Device = Literal["cpu", "cuda"]
WandbMode = Literal["disabled", "offline", "online"]


@dataclass(frozen=True, slots=True)
class TrainingRequest:
    dataset: Path
    output: Path
    device: Device
    config: Path
    wandb_mode: WandbMode = "disabled"


@dataclass(frozen=True, slots=True)
class ActTrainingConfig:
    version: int
    seed: int
    batch_size: int
    steps: int
    log_freq: int
    save_freq: int
    num_workers: int
    chunk_size: int
    n_action_steps: int
    vision_backbone: str
    pretrained_backbone_weights: str | None
    dim_model: int
    n_heads: int
    dim_feedforward: int
    n_encoder_layers: int
    n_decoder_layers: int
    use_vae: bool
    latent_dim: int
    n_vae_encoder_layers: int
    dropout: float
    kl_weight: float
    temporal_ensembling: bool
    use_amp: bool
    cudnn_deterministic: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class DatasetIdentity:
    path: str
    repo_id: str
    logical_fingerprint: str
    storage_fingerprint: str
    train_episode_ids: tuple[str, ...]
    train_episode_indices: tuple[int, ...]
    reserved_episode_ids: tuple[str, ...]
    reserved_episode_indices: tuple[int, ...]
    episode_count: int

    def to_dict(self) -> dict[str, object]:
        return asdict(self)
