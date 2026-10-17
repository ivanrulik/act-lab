"""Isolated, matched-checkpoint stock/projected ACT training throughput A/B."""

import argparse
import gc
import hashlib
import json
import shutil
import statistics
import time
from dataclasses import replace
from pathlib import Path

import torch
from safetensors.torch import load_file

from act_lab.adapters.lerobot.training import (
    checkpoint_step,
    latest_checkpoint,
    train_act,
)
from act_lab.application.training import (
    environment_identity,
    git_identity,
    preflight_device,
    verify_dataset,
)
from act_lab.domain.training import ActTrainingConfig

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--run", type=Path, required=True)
parser.add_argument("--checkpoint", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--updates", type=int, default=100)
parser.add_argument("--warmup", type=int, default=20)
parser.add_argument("--repeats", type=int, default=3)
parser.add_argument("--log-freq", type=int, default=20)
parser.add_argument("--device", choices=["cpu", "cuda"], required=True)
args = parser.parse_args()
if args.output.exists():
    parser.error("output already exists")
if not (0 <= args.warmup < args.updates and args.log_freq > 0 and args.repeats > 0):
    parser.error("invalid benchmark budget")
if args.updates % args.log_freq or args.warmup % args.log_freq:
    parser.error("updates and warmup must be multiples of log-freq")
preflight_device(args.device)
manifest = json.loads((args.run / "act_lab_run.json").read_text())
if manifest["status"] != "completed":
    parser.error("source run must be completed")
dataset = verify_dataset(Path(manifest["dataset"]["path"]))
assert dataset.to_dict() == manifest["dataset"]
start_step = checkpoint_step(args.checkpoint)
config = replace(
    ActTrainingConfig(**manifest["configuration"]),
    steps=start_step + args.updates,
    log_freq=args.log_freq,
)
source_hashes = {
    p.relative_to(args.checkpoint).as_posix(): hashlib.sha256(
        p.read_bytes()
    ).hexdigest()
    for p in args.checkpoint.rglob("*")
    if p.is_file()
}
args.output.mkdir(parents=True)
report = {
    "schema_version": 1,
    "status": "running",
    "source_git": git_identity(Path.cwd()),
    "environment": environment_identity(),
    "device": args.device,
    "dataset_fingerprint": dataset.logical_fingerprint,
    "source_checkpoint_hashes": source_hashes,
    "configuration": config.to_dict(),
    "updates": args.updates,
    "warmup_updates": args.warmup,
    "repeats": args.repeats,
    "scope": (
        "local single-process training, alternating order; "
        "timings include display rounding"
    ),
    "trials": [],
    "comparisons": [],
}
report_path = args.output / "benchmark.json"
report_path.write_text(json.dumps(report, indent=2) + "\n")
try:
    for repeat in range(args.repeats):
        order = [False, True] if repeat % 2 == 0 else [True, False]
        checkpoints = {}
        for projected in order:
            mode = "projected" if projected else "stock"
            run = args.output / f"repeat-{repeat}" / mode
            run.mkdir(parents=True)
            if args.device == "cuda":
                torch.cuda.synchronize()
            begin = time.perf_counter()
            train_act(
                run,
                dataset,
                replace(config, project_action_columns=projected),
                args.device,
                "disabled",
                checkpoint=args.checkpoint,
            )
            if args.device == "cuda":
                torch.cuda.synchronize()
            elapsed = time.perf_counter() - begin
            checkpoint = latest_checkpoint(run)
            assert checkpoint_step(checkpoint) == config.steps
            checkpoints[mode] = checkpoint
            rows = [
                json.loads(line)
                for line in (run / "metrics.jsonl").read_text().splitlines()
            ]
            assert [row["step"] for row in rows] == list(
                range(start_step + args.log_freq, config.steps + 1, args.log_freq)
            )
            steady = [row for row in rows if row["step"] > start_step + args.warmup]
            period = statistics.mean(
                row["update_s"] + row["dataloading_s"] for row in steady
            )
            trial = {
                "repeat": repeat,
                "mode": mode,
                "wall_s": elapsed,
                "steady_period_s": period,
                "steady_updates_s": 1 / period,
                "metrics": rows,
            }
            report["trials"].append(trial)
            report_path.write_text(json.dumps(report, indent=2) + "\n")
            print(
                json.dumps(
                    {key: value for key, value in trial.items() if key != "metrics"}
                ),
                flush=True,
            )
            gc.collect()
            if args.device == "cuda":
                torch.cuda.empty_cache()
        matches = {}
        for relative in [
            "pretrained_model/model.safetensors",
            "training_state/optimizer_state.safetensors",
            "training_state/rng_state.safetensors",
        ]:
            stock = load_file(checkpoints["stock"] / relative)
            projected = load_file(checkpoints["projected"] / relative)
            same = stock.keys() == projected.keys() and all(
                torch.equal(stock[key], projected[key]) for key in stock
            )
            matches[relative] = same
            del stock, projected
        report["comparisons"].append({"repeat": repeat, "exact_tensors": matches})
        report_path.write_text(json.dumps(report, indent=2) + "\n")
        if not all(matches.values()):
            raise RuntimeError(
                "stock/projected tensor mismatch; keep failed checkpoints"
            )
        # These are disposable throughput replicas, never policy candidates.
        # Keep logs/metrics; delete only this successful pair's generated replicas.
        for mode in ["stock", "projected"]:
            shutil.rmtree(
                args.output / f"repeat-{repeat}" / mode / "lerobot/checkpoints"
            )
        assert all(
            hashlib.sha256((args.checkpoint / key).read_bytes()).hexdigest() == digest
            for key, digest in source_hashes.items()
        )
    medians = {
        mode: statistics.median(
            t["steady_updates_s"] for t in report["trials"] if t["mode"] == mode
        )
        for mode in ["stock", "projected"]
    }
    report.update(
        status="completed",
        median_steady_updates_s=medians,
        steady_speed_ratio=medians["projected"] / medians["stock"],
        source_checkpoint_unchanged=True,
        successful_replica_checkpoints_removed=True,
    )
except BaseException as error:
    report.update(status="failed", error=f"{type(error).__name__}: {error}")
    raise
finally:
    report_path.write_text(json.dumps(report, indent=2) + "\n")
