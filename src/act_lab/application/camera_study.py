"""Primitive visibility gates and deliberately matched camera comparisons."""

from __future__ import annotations

import copy
import math
from collections.abc import Sequence
from datetime import datetime
from typing import Any

from act_lab.application.experiment import compare_evaluations

REQUIRED_PHASES = {
    "approach_cube": "cube",
    "descend": "cube",
    "transit": "tray",
    "lower": "tray",
}


def visibility_summary(
    episodes: Sequence[dict[str, Any]],
    *,
    min_pixels: int = 16,
    min_fraction: float = 0.95,
) -> dict[str, Any]:
    if min_pixels <= 0 or not math.isfinite(min_fraction) or not 0 < min_fraction <= 1:
        raise ValueError("invalid visibility thresholds")
    if not episodes or len({e["seed"] for e in episodes}) != len(episodes):
        raise ValueError("visibility episodes must be nonempty and have unique seeds")
    result: dict[str, Any] = {"schema_version": 1, "variants": {}}
    for name, cameras in (("wrist", ("wrist",)), ("paired", ("wrist", "policy"))):
        checks = []
        for episode in episodes:
            for phase, target in REQUIRED_PHASES.items():
                rows = [row for row in episode["samples"] if row["phase"] == phase]
                visible = 0
                for row in rows:
                    counts = [row["cameras"][camera][target] for camera in cameras]
                    if any(type(count) is not int or count < 0 for count in counts):
                        raise ValueError("invalid rendered pixel count")
                    visible += max(counts) >= min_pixels
                fraction = visible / len(rows) if rows else 0.0
                checks.append(
                    {
                        "seed": episode["seed"],
                        "phase": phase,
                        "target": target,
                        "samples": len(rows),
                        "visible_samples": visible,
                        "fraction": fraction,
                        "passed": bool(rows) and fraction >= min_fraction,
                    }
                )
        controls_pass = all(e["expert_success"] is True for e in episodes)
        result["variants"][name] = {
            "passed": controls_pass and all(c["passed"] for c in checks),
            "control_baseline_passed": controls_pass,
            "checks": checks,
        }
    result["thresholds"] = {
        "minimum_pixels": min_pixels,
        "minimum_fraction": min_fraction,
    }
    return result


def latency_summary(seconds: Sequence[float]) -> dict[str, Any]:
    if not seconds or any(not math.isfinite(x) or x < 0 for x in seconds):
        raise ValueError("decision latencies must be finite, nonnegative and nonempty")
    ordered = sorted(seconds)
    return {
        "samples": len(ordered),
        **{
            f"p{p}_s": ordered[math.ceil(p / 100 * len(ordered)) - 1]
            for p in (50, 95, 99)
        },
        "maximum_s": ordered[-1],
        "scope": "RGB rendering plus policy.act; offline wall time",
    }


def compare_camera_evaluations(
    paired: dict[str, Any],
    wrist: dict[str, Any],
    paired_run: dict[str, Any],
    wrist_run: dict[str, Any],
    paired_lineage: dict[str, Any],
    wrist_lineage: dict[str, Any],
    paired_bytes: int,
    wrist_bytes: int,
) -> dict[str, Any]:
    """Allow exactly the declared camera difference; retain all other pairing gates."""
    if set(paired["simulation_config"]["cameras"]) != {"wrist", "policy"} or (
        wrist["simulation_config"]["cameras"] != ["wrist"]
    ):
        raise ValueError("expected wrist versus wrist+policy camera inputs")
    for run, evaluation, lineage in (
        (paired_run, paired, paired_lineage),
        (wrist_run, wrist, wrist_lineage),
    ):
        if run["status"] != "completed":
            raise ValueError("training run is incomplete")
        if run["dataset"]["logical_fingerprint"] != lineage["dataset_fingerprint"]:
            raise ValueError("training dataset lineage differs")
        if run["device"] != evaluation["policy"]["device"]:
            raise ValueError("training/evaluation devices differ")
    for key in ("configuration",):
        if paired_run[key] != wrist_run[key]:
            raise ValueError("training configurations differ")
    for key in ("train_episode_ids", "reserved_episode_ids"):
        if paired_run["dataset"][key] != wrist_run["dataset"][key]:
            raise ValueError("frozen training/validation episodes differ")
    for key in ("train_episode_indices", "reserved_episode_indices"):
        if paired_run["dataset"][key] != wrist_run["dataset"][key]:
            raise ValueError("frozen episode order differs")
    if paired_lineage["selection_manifest"] != wrist_lineage["selection_manifest"]:
        raise ValueError("raw selection manifests differ")
    train_ids = set(wrist_run["dataset"]["train_episode_ids"])
    reserved_ids = set(wrist_run["dataset"]["reserved_episode_ids"])
    if (
        train_ids & reserved_ids
        or len(train_ids | reserved_ids) < 30
        or len(reserved_ids) < 6
    ):
        raise ValueError("pilot requires 30 disjoint selected episodes and 6 reserved")
    if set(wrist["seeds"]) != set(range(22000, 22100)):
        raise ValueError("final comparison requires frozen seeds 22000–22099")
    for lineage in (paired_lineage, wrist_lineage):
        entries = lineage["selection_manifest"]["episodes"]
        if any(
            type(e.get("seed")) is not int or not 20000 <= e["seed"] < 20040
            for e in entries
        ):
            raise ValueError("collection seeds must be 20000–20039")
        selected_ids = {e["episode_id"] for e in entries if e["selected"]}
        if selected_ids != train_ids | reserved_ids:
            raise ValueError(
                "selected raw episodes differ from the frozen training split"
            )
    settings = wrist_run["configuration"]
    if (settings["seed"], settings["steps"], settings["batch_size"]) != (1000, 5000, 8):
        raise ValueError("pilot training seed, update or batch budget differs")
    before, after = copy.deepcopy(paired), copy.deepcopy(wrist)
    before["simulation_config"]["cameras"] = ["wrist"]
    comparison = compare_evaluations(before, after)
    if paired_bytes <= 0 or wrist_bytes <= 0:
        raise ValueError("dataset byte counts must be positive")
    saving = 1 - wrist_bytes / paired_bytes
    events_before = comparison["baseline"]["events"]
    events_after = comparison["candidate"]["events"]
    training_seconds = {}
    for label, run in (("paired", paired_run), ("wrist", wrist_run)):
        duration = (
            datetime.fromisoformat(run["completed_at"])
            - datetime.fromisoformat(run["created_at"])
        ).total_seconds()
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("invalid training wall duration")
        training_seconds[label] = duration
    if any(
        not isinstance(e.get("decision_latency"), dict)
        or not math.isfinite(e["decision_latency"].get("p95_s", float("nan")))
        or e["decision_latency"]["p95_s"] < 0
        for e in (paired, wrist)
    ):
        raise ValueError("missing or invalid decision latency")
    gates = {
        "success_at_least_90_of_100": comparison["candidate"]["successes"] >= 90,
        "paired_success_margin": comparison["paired_difference_bootstrap_ci_95"][0]
        >= -0.05,
        "no_extra_drops": events_after["drop"] <= events_before["drop"],
        "no_extra_rejections": events_after["safety_rejection"]
        <= events_before["safety_rejection"],
        "dataset_saving_at_least_25_percent": saving >= 0.25,
        "decision_p95_not_worse": wrist["decision_latency"]["p95_s"]
        <= paired["decision_latency"]["p95_s"],
    }
    return {
        **comparison,
        "study": "wrist-camera-v1",
        "gates": gates,
        "dataset_saving_fraction": saving,
        "qualified": all(gates.values()),
        "paired_quality_passed": comparison["baseline"]["successes"] >= 90,
        "training_wall_seconds": training_seconds,
        "dataset_bytes": {"paired": paired_bytes, "wrist": wrist_bytes},
        "paired_decision_latency": paired["decision_latency"],
        "wrist_decision_latency": wrist["decision_latency"],
    }
