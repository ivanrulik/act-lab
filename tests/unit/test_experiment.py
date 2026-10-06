"""Coverage and paired evaluation comparisons remain deterministic and strict."""

from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any

import pytest

from act_lab.adapters.mujoco import MujocoUR5eEnvironment
from act_lab.adapters.mujoco.config import SimulationConfig
from act_lab.adapters.mujoco.seed_geometry import cube_spawn_xy
from act_lab.application.experiment import (
    compare_evaluations,
    coverage_report,
    episode_requested,
    parse_seed_ranges,
)
from act_lab.cli import main
from act_lab.domain.dataset import QualityReport, RecordedEpisode


def _evaluation(outcomes: tuple[bool, ...], checkpoint: str) -> dict[str, Any]:
    seeds = [11000 + index for index in range(len(outcomes))]
    return {
        "schema_version": 1,
        "seeds": seeds,
        "simulation_config": {"cube_spawn_x_m": [0.35, 0.52]},
        "policy": {
            "checkpoint_sha256": checkpoint,
            "policy_hz": 25,
            "environment_hz": 50,
            "policy_inference_stride": 2,
        },
        "results": {
            "act": {
                "episodes": [
                    {
                        "seed": seed,
                        "success": success,
                        "completion_time_s": 8.0 if success else None,
                        "failure_reason": None if success else "timeout",
                        "events": {
                            "grasp": int(success),
                            "drop": 0,
                            "workspace_or_rate_limit": 2,
                            "safety_rejection": 0,
                        },
                    }
                    for seed, success in zip(seeds, outcomes, strict=True)
                ]
            }
        },
    }


def test_seed_geometry_matches_environment_reset() -> None:
    config = SimulationConfig.load(Path("configs/sim/ur5e_pick_place.toml"))
    with MujocoUR5eEnvironment(config) as environment:
        for seed in (200, 300, 10004, 11000):
            environment.reset(seed)
            assert cube_spawn_xy(seed, config) == pytest.approx(
                environment.cube_position_xyz_m[:2]
            )


def test_coverage_counts_splits_and_rejects_duplicate_seed() -> None:
    episodes = [
        {"episode_id": "a", "seed": 1, "split": "train", "cube_xy_m": (0.0, 0.0)},
        {"episode_id": "b", "seed": 2, "split": "validation", "cube_xy_m": (1.0, 1.0)},
    ]
    report = coverage_report(episodes, (0.0, 1.0), (0.0, 1.0), bins=2)
    assert report["counts_by_split"]["train"] == [[1, 0], [0, 0]]
    assert report["counts_by_split"]["validation"] == [[0, 0], [0, 1]]
    assert report["occupied_train_bins"] == 1
    with pytest.raises(ValueError, match="distinct integer seeds"):
        coverage_report(episodes + [episodes[0]], (0.0, 1.0), (0.0, 1.0))
    with pytest.raises(ValueError, match="at least one"):
        coverage_report([], (0.0, 1.0), (0.0, 1.0))


def test_manifest_seed_range_selector_is_half_open_and_source_gated() -> None:
    ranges = parse_seed_ranges(["200:20", "300:60"])
    assert ranges == [(200, 220), (300, 360)]
    assert episode_requested("a", 300, "expert", (), ranges, "expert")
    assert episode_requested("a", 359, "expert", (), ranges, "expert")
    assert not episode_requested("a", 360, "expert", (), ranges, "expert")
    assert not episode_requested("a", 300, "webcam", (), ranges, "expert")
    assert episode_requested("a", 2, "expert", ("a",), (), "expert")
    with pytest.raises(ValueError, match="START:COUNT"):
        parse_seed_ranges(["300"])
    with pytest.raises(ValueError, match="count > 0"):
        parse_seed_ranges(["300:0"])


def test_manifest_records_seed_selection_and_source(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: Any
) -> None:
    from act_lab.adapters.mcap import reading
    from act_lab.application import dataset

    sources = {"a": (300, "expert"), "b": (300, "webcam"), "c": (360, "expert")}
    paths = []
    for episode_id in sources:
        path = tmp_path / f"{episode_id}.mcap"
        path.write_bytes(episode_id.encode())
        paths.append(path)

    def read(path: Path) -> RecordedEpisode:
        seed, source = sources[path.stem]
        return RecordedEpisode(
            path.stem,
            "success",
            "success",
            True,
            {"seed": seed, "source": source},
            (),
            (),
            (),
        )

    def validate(episode: RecordedEpisode) -> QualityReport:
        return QualityReport(episode.episode_id, "success", True, True, 1, 1, ())

    monkeypatch.setattr(reading, "read_episode", read)
    monkeypatch.setattr(dataset, "validate_episode", validate)
    output = tmp_path / "selection.json"
    assert (
        main(
            [
                "recording",
                "manifest",
                *(str(path) for path in paths),
                "--include-source",
                "expert",
                "--include-seed-range",
                "300:60",
                "--output",
                str(output),
            ]
        )
        == 0
    )
    capsys.readouterr()
    manifest = json.loads(output.read_text())
    assert [entry["selected"] for entry in manifest["episodes"]] == [True, False, False]
    assert manifest["selection"] == {
        "episode_ids": [],
        "seed_ranges_start_count": ["300:60"],
        "source": "expert",
    }


def test_paired_comparison_tracks_improvement_and_regression() -> None:
    before = _evaluation((True, False, False, True), "before")
    after = _evaluation((True, True, False, False), "after")
    report = compare_evaluations(before, after)
    assert report["baseline"]["successes"] == 2
    assert report["candidate"]["successes"] == 2
    assert report["paired_success_rate_difference"] == 0
    assert report["improved_seeds"] == [11001]
    assert report["regressed_seeds"] == [11003]
    assert report["paired_difference_bootstrap_ci_95"][0] < 0
    assert report["paired_difference_bootstrap_ci_95"][1] > 0
    assert report == compare_evaluations(before, after)


@pytest.mark.parametrize(
    "change", ["seed", "config", "frequency", "duplicate", "invalid_success"]
)
def test_paired_comparison_rejects_incompatible_reports(change: str) -> None:
    before = _evaluation((True, False), "before")
    after = deepcopy(before)
    if change == "seed":
        after["seeds"][1] = 12000
        after["results"]["act"]["episodes"][1]["seed"] = 12000
    elif change == "config":
        after["simulation_config"]["cube_spawn_x_m"] = [0.2, 0.5]
    elif change == "frequency":
        after["policy"]["policy_hz"] = 50
    elif change == "invalid_success":
        after["results"]["act"]["episodes"][1]["success"] = "false"
    else:
        after["results"]["act"]["episodes"][1]["seed"] = 11000
    with pytest.raises(ValueError):
        compare_evaluations(before, after)


def test_comparison_cli_writes_report_once(tmp_path: Path, capsys: Any) -> None:
    baseline = tmp_path / "baseline.json"
    candidate = tmp_path / "candidate.json"
    output = tmp_path / "comparison.json"
    baseline.write_text(json.dumps(_evaluation((False, True), "before")))
    candidate.write_text(json.dumps(_evaluation((True, True), "after")))
    args = [
        "experiment",
        "compare",
        "--baseline",
        str(baseline),
        "--candidate",
        str(candidate),
        "--output",
        str(output),
    ]
    assert main(args) == 0
    assert json.loads(output.read_text())["paired_success_rate_difference"] == 0.5
    assert main(args) == 2
    assert "already exists" in capsys.readouterr().err
