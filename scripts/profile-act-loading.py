"""Compare pinned future-action reads with numeric column projection, read-only."""

import argparse
import cProfile
import io
import json
import pstats
import time
from importlib.metadata import version
from pathlib import Path
from types import MethodType

import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--run", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    parser.error("output already exists")
torch.set_num_threads(1)
manifest = json.loads((args.run / "act_lab_run.json").read_text())
root = Path(manifest["dataset"]["path"])
fps = json.loads((root / "meta/info.json").read_text())["fps"]
chunk = manifest["configuration"]["chunk_size"]
data = LeRobotDataset(
    manifest["dataset"]["repo_id"],
    root=root,
    episodes=manifest["dataset"]["train_episode_indices"],
    delta_timestamps={"action": [i / fps for i in range(chunk)]},
)
reader = data.reader
original = reader._query_hf_dataset


def projected(self, queries):
    others = {key: indices for key, indices in queries.items() if key != "action"}
    result = original(others)
    if "action" in queries:
        indices = queries["action"]
        if self._absolute_to_relative_idx is not None:
            indices = [self._absolute_to_relative_idx[i] for i in indices]
        values = self.hf_dataset.select_columns(["action"])[indices]["action"]
        result["action"] = torch.stack(values)
    return result


records = []
for index in [0, 100, 292, len(data) - 1]:
    reader._query_hf_dataset = original
    profile = cProfile.Profile()
    profile.enable()
    start = time.perf_counter()
    old = data[index]
    slow = time.perf_counter() - start
    profile.disable()
    old_stats = pstats.Stats(profile)
    stock_decodes = sum(
        v[1]
        for key, v in old_stats.stats.items()
        if key[2] == "decode_example" and "features/image.py" in key[0]
    )
    reader._query_hf_dataset = MethodType(projected, reader)
    profile = cProfile.Profile()
    profile.enable()
    start = time.perf_counter()
    new = data[index]
    fast = time.perf_counter() - start
    profile.disable()
    new_stats = pstats.Stats(profile)
    projected_decodes = sum(
        v[1]
        for key, v in new_stats.stats.items()
        if key[2] == "decode_example" and "features/image.py" in key[0]
    )
    assert old.keys() == new.keys()
    for key in old:
        if isinstance(old[key], torch.Tensor):
            assert torch.equal(old[key], new[key]), key
        else:
            assert old[key] == new[key], key
    records.append(
        {
            "index": index,
            "stock_s": slow,
            "projected_s": fast,
            "stock_image_decodes": stock_decodes,
            "projected_image_decodes": projected_decodes,
            "all_fields_equal": True,
        }
    )
reader._query_hf_dataset = original
text = io.StringIO()
pstats.Stats(profile, stream=text).sort_stats("cumulative").print_stats(12)
result = {
    "schema_version": 1,
    "scope": (
        "single-thread CPU reads during ongoing training; not end-to-end throughput"
    ),
    "dataset_fingerprint": manifest["dataset"]["logical_fingerprint"],
    "datasets_version": version("datasets"),
    "lerobot_version": version("lerobot"),
    "records": records,
}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result))
print(text.getvalue())
