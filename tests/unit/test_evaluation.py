"""Evaluation summaries stay deterministic and include every failed attempt."""

from __future__ import annotations

import pytest

from act_lab.application.evaluation import summarize_episodes, wilson_interval
from act_lab.cli import build_parser


def test_wilson_interval_bounds_small_sample() -> None:
    assert wilson_interval(0, 10) == pytest.approx([0.0, 0.27754], abs=1e-5)
    assert wilson_interval(10, 10) == pytest.approx([0.72246, 1.0], abs=1e-5)


def test_summary_includes_failures_and_all_event_counts() -> None:
    episodes = [
        {
            "seed": 10,
            "success": True,
            "completion_time_s": 4.0,
            "events": {
                "grasp": 1,
                "drop": 0,
                "workspace_or_rate_limit": 2,
                "safety_rejection": 0,
            },
        },
        {
            "seed": 11,
            "success": False,
            "completion_time_s": None,
            "failure_reason": "timeout",
            "events": {
                "grasp": 1,
                "drop": 1,
                "workspace_or_rate_limit": 0,
                "safety_rejection": 1,
            },
        },
    ]

    summary = summarize_episodes(episodes)

    assert summary["success_rate"] == 0.5
    assert summary["success_rate_ci_95"] == wilson_interval(1, 2)
    assert summary["mean_completion_time_s"] == 4.0
    assert summary["events"] == {
        "grasp": 2,
        "drop": 1,
        "workspace_or_rate_limit": 2,
        "safety_rejection": 1,
    }
    assert summary["failures"] == [episodes[1]]


def test_evaluate_cli_uses_common_seed_range_and_required_checkpoint() -> None:
    args = build_parser().parse_args(
        [
            "sim",
            "evaluate",
            "--checkpoint",
            "runs/checkpoint",
            "--output",
            "runs/eval",
            "--seed-start",
            "10000",
            "--episodes",
            "7",
        ]
    )

    assert args.sim_command == "evaluate"
    assert args.seed_start == 10000
    assert args.episodes == 7
    with pytest.raises(SystemExit):
        build_parser().parse_args(["sim", "evaluate", "--output", "runs/eval"])
