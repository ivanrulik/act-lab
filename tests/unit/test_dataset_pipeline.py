"""Small framework-neutral fixtures for validation, splits and conversion."""

import json
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from act_lab.adapters.lerobot import convert_episodes
from act_lab.application.dataset import (
    resample_episode,
    split_for_episode,
    validate_episode,
)
from act_lab.domain.dataset import RecordedEpisode, RecordedImage, RecordedSample


def episode() -> RecordedEpisode:
    samples = []
    for index, timestamp in enumerate((20_000_000, 40_000_000, 60_000_000)):
        value = float(index)
        samples.append(
            RecordedSample(
                timestamp,
                (value,) * 6,
                (0.0,) * 6,
                (0.4 + value / 100, 0.0, 0.5),
                (1.0, 0.0, 0.0, 0.0),
                0.5,
                (0.4 + value / 100, 0.0, 0.5),
                (1.0, 0.0, 0.0, 0.0),
                0.5,
                True,
                "applied",
                (
                    RecordedImage(
                        "policy",
                        2,
                        2,
                        "rgb8",
                        bytes((index, 2, 3)) * 4,
                    ),
                ),
            )
        )
    return RecordedEpisode(
        "episode-001",
        "success",
        "success",
        True,
        {
            "episode_id": "episode-001",
            "source": "webcam",
            "resolved_config_json": "{}",
        },
        tuple(samples),
        (("/observation", 3), ("/command", 3), ("/camera/policy", 3)),
        (),
    )


def test_validation_is_actionable_and_keeps_legacy_metadata_eligible() -> None:
    report = validate_episode(episode())
    assert report.valid and report.training_eligible
    assert [issue.code for issue in report.issues] == ["legacy_acquisition_metadata"]

    invalid = replace(episode(), complete=False, outcome="interrupted")
    report = validate_episode(invalid)
    assert not report.valid and not report.training_eligible
    assert {issue.code for issue in report.issues} >= {"incomplete", "unfinished"}


def test_resampling_and_episode_split_are_deterministic() -> None:
    first = resample_episode(episode(), 25)
    second = resample_episode(episode(), 25)
    assert first == second
    assert [sample.timestamp_ns for sample in first.samples] == [20_000_000, 60_000_000]
    assert split_for_episode("episode-001", 7, 0.25) == split_for_episode(
        "episode-001", 7, 0.25
    )
    assert split_for_episode("episode-001", 7, 0.0) == "train"
    assert split_for_episode("episode-001", 7, 1.0) == "validation"


class FakeDataset:
    latest: "FakeDataset"

    def __init__(self, root: Path) -> None:
        self.root = root
        self.frames: list[dict[str, Any]] = []
        self.episodes: list[list[dict[str, Any]]] = []

    @classmethod
    def create(cls, **values: Any) -> "FakeDataset":
        root = Path(values["root"])
        root.mkdir(parents=True)
        result = cls(root)
        cls.latest = result
        return result

    def add_frame(self, frame: dict[str, Any]) -> None:
        self.frames.append(frame)

    def save_episode(self) -> None:
        self.episodes.append(self.frames)
        self.frames = []

    def finalize(self) -> None:
        (self.root / "fake_dataset.json").write_text("{}")


def test_conversion_round_trips_values_and_has_stable_fingerprint(
    tmp_path: Path,
) -> None:
    reports = []
    for name in ("one", "two"):
        reports.append(
            convert_episodes(
                [episode()],
                tmp_path / name,
                "test/act-lab",
                25,
                {"selection_manifest": {"manifest_version": 1}},
                dataset_class=FakeDataset,
            )
        )
        frame = FakeDataset.latest.episodes[0][0]
        np.testing.assert_array_equal(
            frame["observation.state"],
            np.asarray((0.0,) * 6 + (0.5,), dtype=np.float32),
        )
        assert frame["observation.images.policy"].shape == (2, 2, 3)
        lineage = json.loads((tmp_path / name / "act_lab_lineage.json").read_text())
        assert lineage["source_episode_ids"] == ["episode-001"]
        splits = json.loads((tmp_path / name / "act_lab_splits.json").read_text())
        assert splits == {"splits": {}, "unit": "episode"}
    assert reports[0]["dataset_fingerprint"] == reports[1]["dataset_fingerprint"]


def test_real_lerobot_dataset_round_trip_when_data_profile_is_installed(
    tmp_path: Path,
) -> None:
    pytest.importorskip("lerobot")
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    output = tmp_path / "dataset"
    convert_episodes(
        [episode()],
        output,
        "test/act-lab",
        25,
        {"selection_manifest": {"manifest_version": 1}},
    )
    dataset = LeRobotDataset("test/act-lab", root=output)
    assert len(dataset) == 2
    np.testing.assert_allclose(
        dataset[0]["observation.state"].numpy(),
        np.asarray((0.0,) * 6 + (0.5,), dtype=np.float32),
    )
    assert tuple(dataset[0]["observation.images.policy"].shape) == (3, 2, 2)
