"""Recover exact scheduled update counters without overwriting original artifacts."""

import argparse
import hashlib
import json
from pathlib import Path

from act_lab.application.training import parse_metric_line

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--run", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--start-step", type=int, default=0)
args = parser.parse_args()
provenance_path = args.output.with_suffix(".provenance.json")
if args.output.exists() or provenance_path.exists():
    parser.error("output or recovery provenance already exists")
manifest = json.loads((args.run / "act_lab_run.json").read_text())
if manifest["status"] == "running":
    parser.error("wait for the run to stop before recovering metrics")
config = manifest["configuration"]
freq = config["log_freq"]
step = (args.start_step // freq + 1) * freq
raw = (args.run / "train.log").read_bytes()
rows = []
for line in raw.decode().splitlines():
    metric = parse_metric_line(line)
    if metric is None:
        continue
    metric["step"] = step
    if "smpl" in metric:
        metric["smpl"] = step * config["batch_size"]
    rows.append(metric)
    step += freq
if manifest["status"] == "completed":
    assert len(rows) == config["steps"] // freq - args.start_step // freq
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text("".join(json.dumps(row, sort_keys=True) + "\n" for row in rows))
provenance_path.write_text(
    json.dumps(
        {
            "schema_version": 1,
            "source_train_log_sha256": hashlib.sha256(raw).hexdigest(),
            "source_git": manifest["git"],
            "dataset_fingerprint": manifest["dataset"]["logical_fingerprint"],
            "start_step": args.start_step,
            "log_freq": freq,
            "batch_size": config["batch_size"],
            "rows": len(rows),
            "method": (
                "Pinned single-process LeRobot logs after each log_freq updates; "
                "recover exact step/sample counters from that schedule. "
                "Other scalars retain upstream display precision."
            ),
        },
        indent=2,
    )
    + "\n"
)
print(json.dumps({"rows": len(rows), "output": str(args.output)}))
