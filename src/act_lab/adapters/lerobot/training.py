"""Pinned LeRobot 0.6.1 ACT training adapter."""

from __future__ import annotations

import json
import os
import re
import sys
from contextlib import redirect_stderr
from pathlib import Path
from types import MethodType
from typing import Any

from act_lab.application.training import METRICS_FILE, TRAIN_LOG, parse_metric_line
from act_lab.domain.training import ActTrainingConfig, DatasetIdentity


class _ArtifactStream:
    """Tee LeRobot's pinned log stream into text and structured local metrics."""

    def __init__(self, run: Path, target: Any) -> None:
        self._target = target
        self._log = (run / TRAIN_LOG).open("a", encoding="utf-8")
        self._metrics = (run / METRICS_FILE).open("a", encoding="utf-8")
        self._buffer = ""

    def write(self, value: str) -> int:
        self._target.write(value)
        self._log.write(value)
        self._buffer += value
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            self._record_metric(line)
        return len(value)

    def _record_metric(self, line: str) -> None:
        plain = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", line)
        metric = parse_metric_line(plain)
        if metric is not None:
            self._metrics.write(json.dumps(metric, sort_keys=True) + "\n")

    def flush(self) -> None:
        self._target.flush()
        self._log.flush()
        self._metrics.flush()

    def close(self) -> None:
        if self._buffer:
            self._record_metric(self._buffer)
            self._buffer = ""
        self.flush()
        self._log.close()
        self._metrics.close()


def train_act(
    run: Path,
    dataset: DatasetIdentity,
    config: ActTrainingConfig,
    device: str,
    wandb_mode: str,
    *,
    checkpoint: Path | None = None,
) -> None:
    """Translate neutral values and call LeRobot's official training pipeline."""
    try:
        from lerobot.configs.default import (  # type: ignore[import-not-found]
            DatasetConfig,
            WandBConfig,
        )
        from lerobot.configs.train import (  # type: ignore[import-not-found]
            TrainPipelineConfig,
        )
        from lerobot.policies.act.configuration_act import (  # type: ignore[import-not-found]
            ACTConfig,
        )
        from lerobot.scripts.lerobot_train import (  # type: ignore[import-not-found]
            train,
        )
    except ImportError as error:
        raise RuntimeError(
            "ACT training requires the pinned training image; use the train-cpu "
            "or train-gpu Compose service"
        ) from error

    policy = ACTConfig(
        device=device,
        use_amp=config.use_amp,
        push_to_hub=False,
        chunk_size=config.chunk_size,
        n_action_steps=config.n_action_steps,
        vision_backbone=config.vision_backbone,
        pretrained_backbone_weights=config.pretrained_backbone_weights,
        dim_model=config.dim_model,
        n_heads=config.n_heads,
        dim_feedforward=config.dim_feedforward,
        n_encoder_layers=config.n_encoder_layers,
        n_decoder_layers=config.n_decoder_layers,
        use_vae=config.use_vae,
        latent_dim=config.latent_dim,
        n_vae_encoder_layers=config.n_vae_encoder_layers,
        dropout=config.dropout,
        kl_weight=config.kl_weight,
        temporal_ensemble_coeff=None,
        pretrained_path=(checkpoint / "pretrained_model" if checkpoint else None),
    )
    framework_dir = run / "lerobot"
    cfg = TrainPipelineConfig(
        dataset=DatasetConfig(
            repo_id=dataset.repo_id,
            root=dataset.path,
            episodes=list(dataset.train_episode_indices),
            use_imagenet_stats=True,
            eval_split=0.0,
        ),
        policy=policy,
        output_dir=framework_dir,
        job_name="act-lab-act",
        resume=checkpoint is not None,
        seed=config.seed,
        cudnn_deterministic=config.cudnn_deterministic,
        num_workers=config.num_workers,
        persistent_workers=config.num_workers > 0,
        prefetch_factor=4,
        batch_size=config.batch_size,
        steps=config.steps,
        env_eval_freq=0,
        eval_steps=0,
        log_freq=config.log_freq,
        save_checkpoint=True,
        save_freq=config.save_freq,
        use_policy_training_preset=True,
        optimizer=policy.get_optimizer_preset(),
        scheduler=policy.get_scheduler_preset(),
        save_checkpoint_to_hub=False,
        wandb=WandBConfig(
            enable=wandb_mode != "disabled",
            mode=wandb_mode,
            project="act-lab",
            disable_artifact=True,
        ),
    )
    # ACT Lab already resolved and verified the local checkpoint. Avoid LeRobot's
    # CLI-global config_path lookup while retaining its official resume pipeline.
    cfg._resolve_pretrained_from_cli = MethodType(lambda self: None, cfg)
    if checkpoint is not None:
        cfg.checkpoint_path = checkpoint
    os.environ["WANDB_MODE"] = wandb_mode
    stream = _ArtifactStream(run, sys.stderr)
    try:
        with redirect_stderr(stream):
            train(cfg)
    finally:
        stream.close()


def latest_checkpoint(run: Path) -> Path:
    link = run / "lerobot" / "checkpoints" / "last"
    if not link.exists() or not link.is_dir():
        raise ValueError(f"run has no usable LeRobot checkpoint: {run}")
    required = (
        link / "pretrained_model" / "train_config.json",
        link / "training_state" / "training_step.json",
        link / "training_state" / "rng_state.safetensors",
    )
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise ValueError("checkpoint is incomplete: " + ", ".join(missing))
    return link.resolve()


def checkpoint_step(checkpoint: Path) -> int:
    value: Any = json.loads(
        (checkpoint / "training_state" / "training_step.json").read_text()
    ).get("step")
    if not isinstance(value, int) or value < 0:
        raise ValueError("checkpoint training step is invalid")
    return value
