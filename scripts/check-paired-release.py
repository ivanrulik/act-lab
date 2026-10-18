"""Verify downloaded paired packages and rerun consumed simulation seeds."""

import argparse
import contextlib
import json
import sys
from dataclasses import asdict
from pathlib import Path

from act_lab.application.release import verify_package
from act_lab.application.training import verify_dataset


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--device", choices=("cpu", "cuda"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output exists; use a clean directory")
    dataset_manifest = verify_package(args.dataset)
    model_manifest = verify_package(args.model)
    if dataset_manifest["kind"] != "dataset" or model_manifest["kind"] != "model":
        raise ValueError("provide dataset and model packages in the correct order")
    for key in ("storage_fingerprint", "checkpoint_sha256", "inference"):
        if dataset_manifest[key] != model_manifest[key]:
            raise ValueError(f"release packages disagree on {key}")
    inference = model_manifest["inference"]
    root = args.dataset / "payload"
    identity = verify_dataset(root)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(
        identity.repo_id, root=root, episodes=list(identity.train_episode_indices)
    )
    sample = dataset[0]
    assert tuple(sample["observation.state"].shape) == (7,)
    assert tuple(sample["action"].shape) == (8,)
    for camera in ("wrist", "policy"):
        assert tuple(sample[f"observation.images.{camera}"].shape) == (3, 240, 320)
    assert len(identity.train_episode_indices) == 30
    assert len(identity.reserved_episode_indices) == 10

    from act_lab.adapters.mujoco import SimulationConfig
    from act_lab.cli import main as cli

    config = args.model / inference["simulation_config"]
    evaluation = json.loads((args.model / inference["evaluation"]).read_text())
    # JSON canonicalization handles tuple/list representation without altering values.
    resolved = json.loads(json.dumps(asdict(SimulationConfig.load(config))))
    if resolved != evaluation["simulation_config"]:
        raise ValueError("packaged simulation configuration differs from qualification")
    args.output.mkdir(parents=True)
    with (args.output / "evaluation.log").open("x") as log:
        with contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
            result = cli(
                [
                    "sim",
                    "evaluate",
                    "--checkpoint",
                    str(args.model / "payload"),
                    "--config",
                    str(config),
                    "--device",
                    args.device,
                    "--policy-hz",
                    str(inference["policy_hz"]),
                    "--temporal-ensemble-coeff",
                    str(inference["temporal_ensemble_coeff"]),
                    "--seed-start",
                    "23000",
                    "--episodes",
                    "2",
                    "--no-video",
                    "--output",
                    str(args.output / "evaluation"),
                    "--json",
                ]
            )
    if result:
        raise RuntimeError(f"downloaded-model evaluation failed; see {args.output}")
    report = json.loads((args.output / "evaluation/evaluation.json").read_text())
    assert report["policy"]["checkpoint_sha256"] == model_manifest["checkpoint_sha256"]
    assert report["policy"]["act_n_action_steps"] == inference["n_action_steps"]
    assert (
        report["policy"]["act_temporal_ensemble_coeff"]
        == inference["temporal_ensemble_coeff"]
    )
    assert report["policy"]["device"] == args.device
    assert all(item["success"] for item in report["results"]["act"]["episodes"])
    assert "rclpy" not in sys.modules
    # Evaluation must not rewrite the frozen inference files or dataset.
    verify_package(args.model)
    verify_package(args.dataset)
    summary = {
        "status": "passed",
        "device": args.device,
        "scope": "two consumed-seed integration reruns, not fresh qualification",
        "train_episodes": 30,
        "reserved_episodes": 10,
        "training_frames": len(dataset),
        "seeds": [23000, 23001],
        "checkpoint_sha256": model_manifest["checkpoint_sha256"],
        "ros_imported": False,
    }
    (args.output / "report.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
