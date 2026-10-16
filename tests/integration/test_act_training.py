"""Real reduced ACT checkpoint/resume smoke test for the training image."""

import json
from dataclasses import asdict, replace
from pathlib import Path

import pytest

from act_lab.adapters.lerobot import convert_episodes
from act_lab.adapters.lerobot.training import (
    checkpoint_step,
    latest_checkpoint,
    train_act,
)
from act_lab.application.training import load_act_config, verify_dataset
from act_lab.domain.dataset import RecordedEpisode, RecordedImage, RecordedSample


def _mini_episode(tooling: bool = False) -> RecordedEpisode:
    samples = []
    pixels = bytes((32, 64, 96)) * (32 * 32)
    cameras = ("overview", "policy", "wrist") if tooling else ("policy",)
    provenance = {"resolved_config_json": "{}"}
    if tooling:
        from act_lab.adapters.mujoco import SimulationConfig
        from act_lab.adapters.mujoco.environment import MujocoUR5eEnvironment

        config = replace(
            SimulationConfig.load(Path("configs/sim/ur5e_2f85_d405.toml")),
            render_width=32,
            render_height=32,
        )
        with MujocoUR5eEnvironment(config) as env:
            provenance["resolved_config_json"] = json.dumps(
                dict(
                    simulation=asdict(config),
                    model_identity=env.model_identity,
                    scene_cameras={c: env.camera_calibration(c) for c in cameras},
                )
            )
    for index in range(12):
        value = index / 11
        samples.append(
            RecordedSample(
                index * 40_000_000,
                (value,) * 6,
                (0.0,) * 6,
                (0.4, 0.0, 0.5),
                (1.0, 0.0, 0.0, 0.0),
                value,
                (value, value, value),
                (1.0, 0.0, 0.0, 0.0),
                value,
                True,
                "applied",
                tuple(RecordedImage(c, 32, 32, "rgb8", pixels) for c in cameras),
            )
        )
    return RecordedEpisode(
        "mini-train",
        "success",
        "success",
        True,
        provenance,
        tuple(samples),
        (("/observation", 12), ("/command", 12), ("/camera/policy", 12)),
        (),
    )


@pytest.mark.parametrize("tooling", [False, True, "wrist", "paired"])
def test_reduced_act_checkpoint_and_resume(
    tmp_path: Path, tooling: bool | str, monkeypatch: pytest.MonkeyPatch
) -> None:
    pytest.importorskip("lerobot")
    dataset_path = tmp_path / "dataset"
    convert_episodes(
        [_mini_episode(bool(tooling))],
        dataset_path,
        "test/mini-act",
        25,
        {
            "selection_manifest": {
                "episodes": [
                    {"episode_id": "mini-train", "selected": True, "split": "train"}
                ]
            }
        },
        cameras=("wrist",)
        if tooling == "wrist"
        else ("wrist", "policy")
        if tooling == "paired"
        else None,
    )
    config_path = tmp_path / "tiny.toml"
    config_path.write_text(
        """[act]
version = 1
seed = 1000
batch_size = 2
steps = 2
log_freq = 1
save_freq = 1
num_workers = 0
chunk_size = 2
n_action_steps = 2
vision_backbone = "resnet18"
pretrained_backbone_weights = ""
dim_model = 32
n_heads = 4
dim_feedforward = 64
n_encoder_layers = 1
n_decoder_layers = 1
use_vae = true
latent_dim = 4
n_vae_encoder_layers = 1
dropout = 0.0
kl_weight = 1.0
temporal_ensembling = false
use_amp = false
cudnn_deterministic = true
"""
    )
    config = load_act_config(config_path)
    identity = verify_dataset(dataset_path)
    run = tmp_path / "run"
    run.mkdir()
    train_act(run, identity, config, "cpu", "disabled")
    checkpoint = latest_checkpoint(run)
    assert checkpoint_step(checkpoint) == 2
    metrics = [
        json.loads(line) for line in (run / "metrics.jsonl").read_text().splitlines()
    ]
    assert [item["step"] for item in metrics] == [1, 2]
    assert metrics[-1]["loss"] < metrics[0]["loss"] * 0.95
    train_act(
        run,
        identity,
        replace(config, steps=3),
        "cpu",
        "disabled",
        checkpoint=checkpoint,
    )
    resumed = latest_checkpoint(run)
    assert checkpoint_step(resumed) == 3
    metrics = [
        json.loads(line) for line in (run / "metrics.jsonl").read_text().splitlines()
    ]
    assert [item["step"] for item in metrics] == [1, 2, 3]
    assert (resumed / "training_state" / "optimizer_state.safetensors").is_file()
    assert (resumed / "training_state" / "rng_state.safetensors").is_file()
    if tooling:
        contract = json.loads(
            (resumed / "pretrained_model/act_lab_model.json").read_text()
        )
        assert contract["model_id"] == "ur5e_2f85_d405_v1"
        expected = (
            {"wrist"}
            if tooling == "wrist"
            else (
                {"wrist", "policy"}
                if tooling == "paired"
                else {"overview", "policy", "wrist"}
            )
        )
        assert set(contract["cameras"]) == expected
        from act_lab.adapters.lerobot.policy import LeRobotACTPolicy

        with pytest.raises(ValueError, match="incompatible"):
            LeRobotACTPolicy(resumed, "cpu", ("policy",))
        if tooling in ("wrist", "paired"):
            from act_lab.adapters.mujoco import (
                MujocoUR5eEnvironment,
                SimulationConfig,
            )
            from act_lab.application.model_compatibility import model_contract

            simulation = replace(
                SimulationConfig.load(Path("configs/sim/ur5e_2f85_d405.toml")),
                render_width=32,
                render_height=32,
                cameras=tuple(expected),
            )
            with MujocoUR5eEnvironment(simulation) as env:
                observation = env.reset(0)
                current = model_contract(
                    {
                        "simulation": asdict(simulation),
                        "model_identity": env.model_identity,
                        "scene_cameras": {
                            c: env.camera_calibration(c) for c in simulation.cameras
                        },
                    }
                )
                assert current == contract
                policy = LeRobotACTPolicy(
                    resumed, "cpu", simulation.cameras, model_contract=current
                )
                assert policy.n_action_steps == 1
                assert policy._policy._action_queue.maxlen == 1
                predictions = []
                predict = policy._policy.predict_action_chunk

                def counted_prediction(batch):
                    predictions.append(True)
                    return predict(batch)

                monkeypatch.setattr(
                    policy._policy, "predict_action_chunk", counted_prediction
                )
                action = policy.act(
                    observation,
                    {c: env.render(c) for c in simulation.cameras},
                    capture_timestamp_ns=observation.timestamp_ns,
                )
                assert action.enabled
                assert policy.act(
                    observation,
                    {c: env.render(c) for c in simulation.cameras},
                    capture_timestamp_ns=observation.timestamp_ns,
                ).enabled
                assert len(predictions) == 2
                assert not policy.act(observation, {}).enabled
