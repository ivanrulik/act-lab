"""Framework-neutral coverage and paired closed-loop evaluation reports."""

from __future__ import annotations

import random
from collections.abc import Mapping, Sequence
from typing import Any

from act_lab.application.evaluation import wilson_interval

EVENT_KEYS = ("grasp", "drop", "workspace_or_rate_limit", "safety_rejection")


def parse_seed_ranges(values: Sequence[str]) -> list[tuple[int, int]]:
    """Parse repeatable START:COUNT selectors into half-open seed intervals."""
    intervals = []
    for value in values:
        try:
            start_text, count_text = value.split(":")
            start, count = int(start_text), int(count_text)
        except ValueError as error:
            raise ValueError(
                f"invalid seed range {value!r}; use START:COUNT"
            ) from error
        if start < 0 or count <= 0:
            raise ValueError(f"invalid seed range {value!r}; start >= 0, count > 0")
        intervals.append((start, start + count))
    return intervals


def episode_requested(
    episode_id: str,
    seed: int,
    source: str,
    episode_ids: Sequence[str],
    seed_ranges: Sequence[tuple[int, int]],
    required_source: str | None,
) -> bool:
    """Select explicit IDs or seeded expert cohorts, with an optional source gate."""
    if required_source is not None and source != required_source:
        return False
    if not episode_ids and not seed_ranges:
        return True
    return episode_id in episode_ids or any(
        start <= seed < stop for start, stop in seed_ranges
    )


def coverage_report(
    episodes: Sequence[Mapping[str, Any]],
    x_bounds: tuple[float, float],
    y_bounds: tuple[float, float],
    *,
    bins: int = 4,
) -> dict[str, Any]:
    """Count selected train and validation starts in a fixed XY grid."""
    if bins <= 0 or x_bounds[0] >= x_bounds[1] or y_bounds[0] >= y_bounds[1]:
        raise ValueError("coverage requires positive bins and increasing bounds")
    if not episodes:
        raise ValueError("coverage requires at least one selected episode")
    counts = {
        split: [[0] * bins for _ in range(bins)] for split in ("train", "validation")
    }
    seeds: set[int] = set()
    rows: list[dict[str, Any]] = []
    for episode in episodes:
        seed = episode["seed"]
        split = episode["split"]
        x, y = episode["cube_xy_m"]
        if isinstance(seed, bool) or not isinstance(seed, int) or seed in seeds:
            raise ValueError("selected episodes require distinct integer seeds")
        if split not in counts:
            raise ValueError(f"invalid selected episode split: {split}")
        if not (x_bounds[0] <= x <= x_bounds[1] and y_bounds[0] <= y <= y_bounds[1]):
            raise ValueError(f"seed {seed} lies outside configured spawn bounds")
        seeds.add(seed)
        x_bin = min(
            bins - 1, int((x - x_bounds[0]) / (x_bounds[1] - x_bounds[0]) * bins)
        )
        y_bin = min(
            bins - 1, int((y - y_bounds[0]) / (y_bounds[1] - y_bounds[0]) * bins)
        )
        counts[split][y_bin][x_bin] += 1
        rows.append(
            {
                "episode_id": episode["episode_id"],
                "seed": seed,
                "split": split,
                "cube_xy_m": [x, y],
                "x_bin": x_bin,
                "y_bin": y_bin,
            }
        )
    rows.sort(key=lambda row: row["seed"])
    return {
        "schema_version": 1,
        "bounds_xy_m": {"x": list(x_bounds), "y": list(y_bounds)},
        "bins_per_axis": bins,
        "counts_by_split": counts,
        "occupied_train_bins": sum(
            value > 0 for row in counts["train"] for value in row
        ),
        "selected_episodes": rows,
    }


def _act_episodes(report: Mapping[str, Any]) -> dict[int, Mapping[str, Any]]:
    if report.get("schema_version") != 1:
        raise ValueError("unsupported evaluation report version")
    episodes = report["results"]["act"]["episodes"]
    declared_seeds = report["seeds"]
    if len(declared_seeds) != len(set(declared_seeds)):
        raise ValueError("duplicate declared evaluation seed")
    by_seed: dict[int, Mapping[str, Any]] = {}
    for episode in episodes:
        seed = episode["seed"]
        if seed in by_seed:
            raise ValueError(f"duplicate evaluation seed: {seed}")
        if type(episode["success"]) is not bool:
            raise ValueError(f"non-boolean success for seed: {seed}")
        by_seed[seed] = episode
    if not by_seed or set(by_seed) != set(declared_seeds):
        raise ValueError("ACT episodes do not match declared evaluation seeds")
    return by_seed


def compare_evaluations(
    baseline: Mapping[str, Any], candidate: Mapping[str, Any]
) -> dict[str, Any]:
    """Compare two ACT checkpoints on identical seeded simulation conditions."""
    before = _act_episodes(baseline)
    after = _act_episodes(candidate)
    if set(before) != set(after):
        raise ValueError("evaluation seed sets differ")
    if baseline["simulation_config"] != candidate["simulation_config"]:
        raise ValueError("simulation configurations differ")
    if baseline.get("safety_path") != candidate.get("safety_path"):
        raise ValueError("safety paths differ")
    for key in ("device", "policy_hz", "environment_hz", "policy_inference_stride"):
        if baseline["policy"].get(key) != candidate["policy"].get(key):
            raise ValueError(f"policy execution setting differs: {key}")
    seeds = sorted(before)
    delta = [
        int(bool(after[s]["success"])) - int(bool(before[s]["success"])) for s in seeds
    ]
    rng = random.Random(0)
    bootstraps = sorted(
        sum(rng.choice(delta) for _ in delta) / len(delta) for _ in range(5000)
    )

    def summary(episodes: Mapping[int, Mapping[str, Any]]) -> dict[str, Any]:
        successes = sum(bool(item["success"]) for item in episodes.values())
        completed = [
            float(item["completion_time_s"])
            for item in episodes.values()
            if item["success"]
        ]
        return {
            "successes": successes,
            "success_rate": successes / len(episodes),
            "success_rate_ci_95": wilson_interval(successes, len(episodes)),
            "mean_completion_time_s": sum(completed) / len(completed)
            if completed
            else None,
            "events": {
                key: sum(int(item["events"][key]) for item in episodes.values())
                for key in EVENT_KEYS
            },
        }

    return {
        "schema_version": 1,
        "seeds": seeds,
        "baseline_checkpoint_sha256": baseline["policy"]["checkpoint_sha256"],
        "candidate_checkpoint_sha256": candidate["policy"]["checkpoint_sha256"],
        "baseline": summary(before),
        "candidate": summary(after),
        "paired_success_rate_difference": sum(delta) / len(delta),
        "paired_difference_bootstrap_ci_95": [bootstraps[125], bootstraps[4874]],
        "improved_seeds": [
            seed for seed, change in zip(seeds, delta, strict=True) if change == 1
        ],
        "regressed_seeds": [
            seed for seed, change in zip(seeds, delta, strict=True) if change == -1
        ],
        "per_seed": [
            {
                "seed": seed,
                "baseline_success": bool(before[seed]["success"]),
                "candidate_success": bool(after[seed]["success"]),
                "baseline_failure_reason": before[seed]["failure_reason"],
                "candidate_failure_reason": after[seed]["failure_reason"],
            }
            for seed in seeds
        ],
    }
