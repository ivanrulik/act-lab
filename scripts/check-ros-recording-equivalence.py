"""Required ROS-free conversion acceptance, using the public validated CLI."""

import argparse
import json
import sys
from pathlib import Path

from act_lab.adapters.mcap.reading import read_episode
from act_lab.application.training import verify_dataset
from act_lab.cli import main

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--capture", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    raise FileExistsError(str(args.output))
args.output.mkdir(parents=True)
local_paths = sorted((args.capture / "local").glob("*.mcap"))
imported_paths = sorted((args.capture / "imported").glob("*.mcap"))
if not local_paths or [p.name for p in local_paths] != [p.name for p in imported_paths]:
    raise RuntimeError("local/imported episode identities disagree")
for local, imported in zip(local_paths, imported_paths, strict=True):
    if read_episode(local) != read_episode(imported):
        raise RuntimeError("local/imported contents disagree")
results = []
for name, paths in (("local", local_paths), ("imported", imported_paths)):
    manifest = args.output / f"{name}-selection.json"
    if (
        main(
            [
                "recording",
                "manifest",
                *map(str, paths),
                "--output",
                str(manifest),
                "--validation-fraction",
                "0",
            ]
        )
        != 0
    ):
        raise RuntimeError("quality validation/selection failed")
    dataset = args.output / name
    if (
        main(
            [
                "recording",
                "convert",
                str(manifest),
                "--output",
                str(dataset),
                "--repo-id",
                "test/ros-equivalence",
                "--fps",
                "25",
            ]
        )
        != 0
    ):
        raise RuntimeError("validated conversion failed")
    verify_dataset(dataset)
    results.append(json.loads((dataset / "act_lab_lineage.json").read_text()))
if results[0]["dataset_fingerprint"] != results[1]["dataset_fingerprint"]:
    raise RuntimeError("derived dataset fingerprints disagree")
if any(n in sys.modules for n in ("rclpy", "rosbag2_py", "act_lab_interfaces")):
    raise RuntimeError("conversion imported ROS")
report = dict(
    status="passed",
    schema_version=1,
    ros_free=True,
    dataset_fingerprint=results[0]["dataset_fingerprint"],
    episode_ids=results[0]["source_episode_ids"],
)
(args.output / "report.json").write_text(
    json.dumps(report, indent=2, sort_keys=True) + "\n"
)
print(json.dumps(report, indent=2))
