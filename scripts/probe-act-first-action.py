"""Inspect first-action error on five fixed frames per pilot episode."""

import argparse
import json
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from act_lab.adapters.lerobot.policy import LeRobotACTPolicy

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--pilot", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    parser.error("output already exists")
root = args.pilot
checkpoint = root / "paired/training/lerobot/checkpoints/005000/pretrained_model"
contract = json.loads((checkpoint / "act_lab_model.json").read_text())
policy = LeRobotACTPolicy(
    checkpoint, "cuda", ("wrist", "policy"), model_contract=contract
)
data = LeRobotDataset("local/camera-paired", root=root / "paired/dataset")
run = json.loads((root / "paired/training/act_lab_run.json").read_text())
train = set(run["dataset"]["train_episode_indices"])
groups = {}
for file in sorted((root / "paired/dataset/data").rglob("*.parquet")):
    for row in pq.read_table(file, columns=["index", "episode_index"]).to_pylist():
        groups.setdefault(row["episode_index"], []).append(row["index"])
rows = []
for ep, indices in sorted(groups.items()):
    for fraction in [0, 0.25, 0.5, 0.75, 1]:
        index = indices[round(fraction * (len(indices) - 1))]
        row = data[index]
        batch = {
            k: row[k].unsqueeze(0).to("cuda")
            for k in [
                "observation.state",
                "observation.images.wrist",
                "observation.images.policy",
            ]
        }
        with torch.inference_mode():
            processed = policy._preprocessor(batch)
            chunk = (
                policy._postprocessor(policy._policy.predict_action_chunk(processed))
                .cpu()
                .numpy()[0]
            )
        truth = row["action"].numpy()
        rows.append(
            {
                "episode": ep,
                "index": index,
                "fraction": fraction,
                "split": "train" if ep in train else "reserved",
                "truth": truth.tolist(),
                "first": chunk[0].tolist(),
                "future_25": chunk[25].tolist(),
                "xyz_error_m": float(np.linalg.norm(chunk[0, :3] - truth[:3])),
                "gripper_error": float(abs(chunk[0, 7] - truth[7])),
            }
        )
result = {
    "schema_version": 1,
    "sampling": (
        "five uniform-index frames per frozen episode; "
        "first chunk prediction against same-row dataset label"
    ),
    "rows": rows,
    "summary": {},
}
for split in ["train", "reserved"]:
    selected = [r for r in rows if r["split"] == split]
    result["summary"][split] = {
        key: {
            "median": float(np.median([r[key] for r in selected])),
            "p95": float(np.percentile([r[key] for r in selected], 95)),
        }
        for key in ["xyz_error_m", "gripper_error"]
    }
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result["summary"]))
