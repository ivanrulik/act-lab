"""Measure command/observation temporal alignment in the frozen raw cohort."""

import argparse
import json
import math
import statistics
from pathlib import Path

from mcap.reader import make_reader

from act_lab.adapters.mcap.schema import decode

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--pilot", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    parser.error("output already exists")
root = args.pilot / "raw"
rows = []
for path in sorted(root.glob("*.mcap")):
    observations = {}
    commands = {}
    with path.open("rb") as stream:
        for schema, channel, record in make_reader(stream).iter_messages(
            topics=["/observation", "/command"]
        ):
            msg = decode(schema.name.rsplit(".", 1)[-1], record.data)
            (observations if channel.topic == "/observation" else commands)[
                record.log_time
            ] = msg
    stamps = sorted(observations)
    for before, after in zip(stamps, stamps[1:], strict=False):
        cmd = commands[after]
        if not cmd.HasField("executed_action") or not cmd.executed_action.enabled:
            continue
        target = list(cmd.executed_action.target_pose.position_xyz_m)
        pre = list(observations[before].robot.end_effector_pose.position_xyz_m)
        post = list(observations[after].robot.end_effector_pose.position_xyz_m)
        rows.append(
            {
                "post_error_m": math.dist(target, post),
                "pre_error_m": math.dist(target, pre),
                "actual_motion_m": math.dist(pre, post),
                "requested_age_ns": after - cmd.requested_action.timestamp_ns,
                "period_ns": after - before,
            }
        )
moving = [r for r in rows if r["actual_motion_m"] > 0.0001]
result = {
    "episodes": len(list(root.glob("*.mcap"))),
    "enabled_samples": len(rows),
    "moving_samples": len(moving),
    "moving_threshold_m": 0.0001,
    "moving_summary": {
        k: {
            "median": statistics.median(r[k] for r in moving),
            "p95": sorted(r[k] for r in moving)[int(0.95 * len(moving))],
        }
        for k in ["post_error_m", "pre_error_m", "actual_motion_m"]
    },
    "source_age_ns": sorted(set(r["requested_age_ns"] for r in rows)),
    "sample_period_ns": sorted(set(r["period_ns"] for r in rows)),
}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(result, indent=2))
print(json.dumps(result))
