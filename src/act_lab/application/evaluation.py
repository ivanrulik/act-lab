"""Framework-neutral evaluation summaries and binomial confidence intervals."""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Any


def wilson_interval(
    successes: int, trials: int, z: float = 1.959963984540054
) -> list[float]:
    """Two-sided 95% Wilson score interval for a binomial success rate."""
    if trials <= 0 or not 0 <= successes <= trials:
        raise ValueError("successes and trials must define a nonempty binomial sample")
    rate = successes / trials
    denominator = 1.0 + z * z / trials
    center = (rate + z * z / (2.0 * trials)) / denominator
    margin = (
        z
        * math.sqrt(rate * (1.0 - rate) / trials + z * z / (4.0 * trials**2))
        / denominator
    )
    return [max(0.0, center - margin), min(1.0, center + margin)]


def summarize_episodes(episodes: Sequence[dict[str, Any]]) -> dict[str, Any]:
    if not episodes:
        raise ValueError("evaluation requires at least one episode")
    successes = sum(episode["success"] for episode in episodes)
    count = len(episodes)
    completed = [
        episode["completion_time_s"] for episode in episodes if episode["success"]
    ]
    return {
        "episodes": count,
        "successes": successes,
        "success_rate": successes / count,
        "success_rate_ci_95": wilson_interval(successes, count),
        "mean_completion_time_s": sum(completed) / len(completed)
        if completed
        else None,
        "median_completion_time_s": sorted(completed)[len(completed) // 2]
        if completed
        else None,
        "events": {
            key: sum(int(episode["events"][key]) for episode in episodes)
            for key in ("grasp", "drop", "workspace_or_rate_limit", "safety_rejection")
        },
        "failures": [episode for episode in episodes if not episode["success"]],
    }
