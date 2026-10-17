"""Run the fixed, resource-bounded study through the authoritative CLI."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = args.output.resolve()
    if root.exists():
        parser.error("output already exists")
    root.mkdir(parents=True)
    commands: list[list[str]] = []

    def run(label: str, *parts: str) -> None:
        commands.append(list(parts))
        (root / "commands.json").write_text(json.dumps(commands, indent=2))
        with (root / f"{label}.log").open("w") as log:
            subprocess.run(
                [sys.executable, "-m", "act_lab.cli", *parts],
                stdout=log,
                stderr=subprocess.STDOUT,
                check=True,
            )
        used = sum(p.stat().st_size for p in root.rglob("*") if p.is_file())
        if used > 5 * 1024**3:
            raise RuntimeError("5 GiB pilot artifact budget exceeded; no next stage")

    try:
        paired_config = "configs/sim/ur5e_2f85_d405_paired.toml"
        run(
            "visibility",
            "experiment",
            "camera-audit",
            "--config",
            paired_config,
            "--output",
            str(root / "audit"),
        )
        visibility = json.loads((root / "audit/visibility.json").read_text())
        if not all(v["passed"] for v in visibility["variants"].values()):
            result = {
                "status": "completed_observation_no_go",
                "training_started": False,
                "variants": {k: v["passed"] for k, v in visibility["variants"].items()},
            }
        else:
            from act_lab.application.training import preflight_device

            preflight_device("cuda")
            for seed in range(20000, 20040):
                run(
                    f"expert-{seed}",
                    "sim",
                    "expert",
                    "--config",
                    paired_config,
                    "--seed-start",
                    str(seed),
                    "--episodes",
                    "1",
                    "--min-success-rate",
                    "0",
                    "--record-dir",
                    str(root / "raw"),
                    "--json",
                )
            run(
                "manifest",
                "recording",
                "manifest",
                *map(str, sorted((root / "raw").glob("*.mcap"))),
                "--output",
                str(root / "selection.json"),
                "--validation-fraction",
                "0.2",
                "--split-seed",
                "0",
                "--include-source",
                "expert",
                "--include-seed-range",
                "20000:40",
            )
            manifest = json.loads((root / "selection.json").read_text())
            selected = [e for e in manifest["episodes"] if e["selected"]]
            if (
                len(selected) < 30
                or sum(e["split"] == "validation" for e in selected) < 6
            ):
                raise RuntimeError("pilot requires 30 selected episodes and 6 reserved")
            for name, cameras in (
                ("paired", ["wrist", "policy"]),
                ("wrist", ["wrist"]),
            ):
                run(
                    f"convert-{name}",
                    "recording",
                    "convert",
                    str(root / "selection.json"),
                    "--output",
                    str(root / name / "dataset"),
                    "--repo-id",
                    f"local/camera-{name}",
                    "--fps",
                    "25",
                    "--cameras",
                    *cameras,
                )
                run(
                    f"train-{name}",
                    "train",
                    "act",
                    "--dataset",
                    str(root / name / "dataset"),
                    "--config",
                    "configs/training/act-camera-pilot.toml",
                    "--device",
                    "cuda",
                    "--output",
                    str(root / name / "training"),
                )
            # Freeze both trained checkpoints before observing any final outcomes.
            for name in ("paired", "wrist"):
                checkpoint = root / name / "training/lerobot/checkpoints/005000"
                for label, start, count in (
                    ("diagnostic", 21000, 20),
                    ("final", 22000, 100),
                ):
                    run(
                        f"{name}-{label}",
                        "sim",
                        "evaluate",
                        "--checkpoint",
                        str(checkpoint),
                        "--device",
                        "cuda",
                        "--policy-hz",
                        "25",
                        "--no-video",
                        "--seed-start",
                        str(start),
                        "--episodes",
                        str(count),
                        "--config",
                        f"configs/sim/ur5e_2f85_d405_{name}.toml",
                        "--output",
                        str(root / name / label),
                    )
            parts = ["experiment", "camera-compare"]
            for name in ("paired", "wrist"):
                for item, path in (
                    ("evaluation", "final/evaluation.json"),
                    ("dataset", "dataset"),
                    ("run", "training"),
                    (
                        "checkpoint",
                        "training/lerobot/checkpoints/005000/pretrained_model",
                    ),
                ):
                    parts.extend([f"--{name}-{item}", str(root / name / path)])
            run("comparison", *parts, "--output", str(root / "comparison.json"))
            result = {
                "status": "completed",
                "training_started": True,
                "comparison": json.loads((root / "comparison.json").read_text()),
            }
        (root / "study.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        return 0
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        (root / "failure.json").write_text(
            json.dumps({"status": "error", "detail": str(error)})
        )
        print(str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
