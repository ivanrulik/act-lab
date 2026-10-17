"""Compare first-action XYZ errors with a same-frame identity baseline."""

import argparse
import json
import math
import statistics
from pathlib import Path

import pyarrow.parquet as pq
from mcap.reader import make_reader

from act_lab.adapters.mcap.schema import decode

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--pilot", type=Path, required=True)
parser.add_argument("--probe", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    parser.error("output already exists")
root = args.pilot
probe = json.loads(args.probe.read_text())
selected = [
    e
    for e in json.loads((root / "selection.json").read_text())["episodes"]
    if e["selected"]
]
index = {}
for p in (root / "paired/dataset/data").rglob("*.parquet"):
    for row in pq.read_table(
        p, columns=["index", "episode_index", "frame_index"]
    ).to_pylist():
        index[row["index"]] = row
samples = []
for ep, entry in enumerate(selected):
    raw = {}
    with (root / entry["path"]).resolve().open("rb") as source:
        for schema, channel, record in make_reader(source).iter_messages(
            topics=["/observation", "/command"]
        ):
            msg = decode(schema.name.rsplit(".", 1)[-1], record.data)
            raw.setdefault(record.log_time, {})[channel.topic] = msg
    start = min(raw)
    for r in [r for r in probe["rows"] if r["episode"] == ep]:
        stamp = start + index[r["index"]]["frame_index"] * 40_000_000
        pose = list(raw[stamp]["/observation"].robot.end_effector_pose.position_xyz_m)
        truth = r["truth"][:3]
        # Require exact source-to-derived target agreement, up to float32 encoding.
        cmd = raw[stamp]["/command"]
        applied = (
            cmd.executed_action
            if cmd.HasField("executed_action")
            else cmd.requested_action
        )
        target = list(applied.target_pose.position_xyz_m)
        assert math.dist(target, truth) < 1e-6
        future = raw.get(stamp + 20_000_000)
        if future:
            fc = future["/command"]
            fa = (
                fc.executed_action
                if fc.HasField("executed_action")
                else fc.requested_action
            )
        samples.append(
            {
                "episode": ep,
                "index": r["index"],
                "split": r["split"],
                "identity_error_m": math.dist(pose, truth),
                "model_error_m": r["xyz_error_m"],
                "next_command_identity_error_m": math.dist(
                    pose, list(fa.target_pose.position_xyz_m)
                )
                if future
                else None,
            }
        )
result = {"samples": len(samples), "summary": {}}
for split in ["train", "reserved"]:
    ss = [s for s in samples if s["split"] == split]
    result["summary"][split] = {
        key: {
            "median": statistics.median(s[key] for s in ss if s[key] is not None),
            "p95": sorted(s[key] for s in ss if s[key] is not None)[
                int(0.95 * sum(s[key] is not None for s in ss))
            ],
        }
        for key in [
            "identity_error_m",
            "model_error_m",
            "next_command_identity_error_m",
        ]
    }
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(result, indent=2))
print(json.dumps(result))
