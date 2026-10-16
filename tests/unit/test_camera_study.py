"""Frozen study gates, pairing exclusions and exact boundary behavior."""

import copy
import importlib.util
import json
import sys
from pathlib import Path

import pytest

from act_lab.application.camera_study import (
    REQUIRED_PHASES,
    compare_camera_evaluations,
    latency_summary,
    visibility_summary,
)


def audit_episode():
    return {
        "seed": 19000,
        "expert_success": True,
        "samples": [
            {
                "phase": phase,
                "cameras": {
                    "wrist": {"cube": 16 if i else 0, "tray": 16 if i else 0},
                    "policy": {"cube": 20, "tray": 20},
                },
            }
            for phase in REQUIRED_PHASES
            for i in range(20)
        ],
    }


def test_visibility_exact_95_percent_and_16_pixels() -> None:
    episode = audit_episode()
    result = visibility_summary([episode])
    assert result["variants"]["wrist"]["passed"]
    episode["samples"][1]["cameras"]["wrist"]["cube"] = 15
    result = visibility_summary([episode])
    assert not result["variants"]["wrist"]["passed"]
    assert result["variants"]["paired"]["passed"]


def test_missing_phase_and_failed_control_cannot_qualify() -> None:
    episode = audit_episode()
    episode["samples"] = [r for r in episode["samples"] if r["phase"] != "lower"]
    assert not visibility_summary([episode])["variants"]["paired"]["passed"]
    episode = audit_episode()
    episode["expert_success"] = False
    assert not visibility_summary([episode])["variants"]["paired"]["passed"]
    with pytest.raises(ValueError, match="unique seeds"):
        visibility_summary([episode, episode])


def test_visibility_and_latency_reject_nonfinite_metrics() -> None:
    with pytest.raises(ValueError):
        visibility_summary([audit_episode()], min_fraction=float("nan"))
    episode = audit_episode()
    episode["samples"][0]["cameras"]["wrist"]["cube"] = float("nan")
    with pytest.raises(ValueError, match="pixel"):
        visibility_summary([episode])
    for values in ([], [float("nan")], [-1]):
        with pytest.raises(ValueError):
            latency_summary(values)
    assert latency_summary(list(range(100)))["p95_s"] == 94


def comparison_inputs():
    seeds = list(range(22000, 22100))
    events = {
        "drop": 0,
        "grasp": 1,
        "workspace_or_rate_limit": 0,
        "safety_rejection": 0,
    }
    paired = {
        "schema_version": 1,
        "seeds": seeds,
        "simulation_config": {"cameras": ["wrist", "policy"], "physics_hz": 500},
        "safety_path": "SafeCartesianRobot.command(Action)",
        "policy": {
            "device": "cuda",
            "policy_hz": 25,
            "environment_hz": 50,
            "policy_inference_stride": 2,
            "checkpoint_sha256": "paired",
        },
        "decision_latency": {"p95_s": 0.02},
        "results": {
            "act": {
                "episodes": [
                    {
                        "seed": seed,
                        "success": True,
                        "completion_time_s": 10.0,
                        "failure_reason": None,
                        "events": events.copy(),
                    }
                    for seed in seeds
                ]
            }
        },
    }
    wrist = copy.deepcopy(paired)
    wrist["simulation_config"]["cameras"] = ["wrist"]
    wrist["policy"]["checkpoint_sha256"] = "wrist"
    wrist["decision_latency"]["p95_s"] = 0.01
    run = {
        "created_at": "2026-10-15T00:00:00+00:00",
        "completed_at": "2026-10-15T00:10:00+00:00",
        "status": "completed",
        "device": "cuda",
        "configuration": {"steps": 5000, "seed": 1000, "batch_size": 8},
        "dataset": {
            "logical_fingerprint": "data",
            "train_episode_ids": [f"episode-{i}" for i in range(24)],
            "reserved_episode_ids": [f"episode-{i}" for i in range(24, 30)],
            "train_episode_indices": list(range(24)),
            "reserved_episode_indices": list(range(24, 30)),
        },
    }
    lineage = {
        "dataset_fingerprint": "data",
        "selection_manifest": {
            "episodes": [
                {"seed": 20000 + i, "episode_id": f"episode-{i}", "selected": True}
                for i in range(30)
            ]
        },
    }
    return (
        paired,
        wrist,
        copy.deepcopy(run),
        copy.deepcopy(run),
        copy.deepcopy(lineage),
        copy.deepcopy(lineage),
    )


def test_matched_camera_comparison_and_cost_gate() -> None:
    args = comparison_inputs()
    result = compare_camera_evaluations(*args, 100, 75)
    assert result["qualified"]
    assert result["paired_difference_bootstrap_ci_95"] == [0.0, 0.0]
    assert args[0]["simulation_config"]["cameras"] == ["wrist", "policy"]
    assert not compare_camera_evaluations(*args, 100, 76)["qualified"]


@pytest.mark.parametrize("change", ["physics", "split", "seed", "training", "timing"])
def test_unmatched_study_is_rejected(change: str) -> None:
    args = comparison_inputs()
    if change == "physics":
        args[1]["simulation_config"]["physics_hz"] = 1000
    elif change == "split":
        args[3]["dataset"]["train_episode_ids"] = ["other"]
    elif change == "seed":
        args[4]["selection_manifest"]["episodes"][0]["seed"] = 22000
        args[5]["selection_manifest"]["episodes"][0]["seed"] = 22000
    elif change == "training":
        args[3]["configuration"]["steps"] = 100000
    else:
        args[1]["decision_latency"]["p95_s"] = float("nan")
    with pytest.raises(ValueError):
        compare_camera_evaluations(*args, 100, 50)


def test_success_and_noninferiority_are_separate_gates() -> None:
    args = comparison_inputs()
    for episode in args[1]["results"]["act"]["episodes"][:10]:
        episode.update(success=False, completion_time_s=None, failure_reason="timeout")
    result = compare_camera_evaluations(*args, 100, 50)
    assert result["gates"]["success_at_least_90_of_100"]
    assert not result["gates"]["paired_success_margin"]


def test_workflow_stops_before_collection_or_gpu_when_visibility_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = Path(__file__).resolve().parents[2] / "scripts/run-camera-study.py"
    spec = importlib.util.spec_from_file_location("camera_runner", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    output = tmp_path / "study"
    calls = []

    def command(argv, **kwargs):
        calls.append(argv)
        audit = output / "audit"
        audit.mkdir()
        (audit / "visibility.json").write_text(
            json.dumps(
                {"variants": {"wrist": {"passed": False}, "paired": {"passed": True}}}
            )
        )

    monkeypatch.setattr(module.subprocess, "run", command)
    monkeypatch.setattr(sys, "argv", [str(path), "--output", str(output)])
    assert module.main() == 0
    assert len(calls) == 1
    assert json.loads((output / "study.json").read_text())["training_started"] is False


@pytest.mark.parametrize("paired_passes", [False, True])
def test_paired_workflow_preserves_gates_and_never_trains_wrist(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, paired_passes: bool
) -> None:
    from act_lab.application import training

    path = Path(__file__).resolve().parents[2] / "scripts/run-camera-study.py"
    spec = importlib.util.spec_from_file_location("paired_runner", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    output = tmp_path / "pilot"
    calls: list[list[str]] = []
    devices: list[str] = []
    monkeypatch.setattr(training, "preflight_device", devices.append)

    def command(argv, **kwargs):
        parts = argv[3:]
        calls.append(parts)
        if parts[:2] == ["experiment", "camera-audit"]:
            audit = output / "audit"
            audit.mkdir()
            (audit / "visibility.json").write_text(
                json.dumps(
                    {
                        "variants": {
                            "wrist": {"passed": False},
                            "paired": {"passed": paired_passes},
                        }
                    }
                )
            )
        elif parts[:2] == ["recording", "manifest"]:
            (output / "selection.json").write_text(
                json.dumps(
                    {
                        "episodes": [
                            {
                                "selected": True,
                                "split": "validation" if i < 6 else "train",
                            }
                            for i in range(30)
                        ]
                    }
                )
            )
        elif parts[:2] == ["sim", "evaluate"]:
            destination = Path(parts[parts.index("--output") + 1])
            destination.mkdir(parents=True)
            (destination / "evaluation.json").write_text(
                json.dumps({"results": {"act": {"summary": {"successes": 90}}}})
            )

    monkeypatch.setattr(module.subprocess, "run", command)
    monkeypatch.setattr(
        sys, "argv", [str(path), "--variant", "paired", "--output", str(output)]
    )
    assert module.main() == 0
    result = json.loads((output / "study.json").read_text())
    if not paired_passes:
        assert len(calls) == 1
        assert not devices
        assert not result["training_started"]
        return
    assert devices == ["cuda"]
    experts = [p for p in calls if p[:2] == ["sim", "expert"]]
    assert [int(p[p.index("--seed-start") + 1]) for p in experts] == list(
        range(20000, 20040)
    )
    conversions = [p for p in calls if p[:2] == ["recording", "convert"]]
    assert len(conversions) == 1
    assert conversions[0][-3:] == ["--cameras", "wrist", "policy"]
    assert sum(p[:2] == ["train", "act"] for p in calls) == 1
    evaluations = [p for p in calls if p[:2] == ["sim", "evaluate"]]
    assert [p[p.index("--seed-start") + 1] for p in evaluations] == ["21000", "22000"]
    assert [p[p.index("--episodes") + 1] for p in evaluations] == ["20", "100"]
    assert result["qualified"]
    assert result["comparison_claim"] is False
