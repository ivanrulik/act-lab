"""Small framework-neutral fixtures for validation, splits and conversion."""

import argparse
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


def test_manifest_seed_metadata_checks_preserve_legacy_episodes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from act_lab.adapters import lerobot
    from act_lab.adapters.mcap import reading
    from act_lab.application.dataset import file_sha256
    from act_lab.cli import _recording_convert

    raw = tmp_path / "raw.mcap"
    raw.write_bytes(b"fixture")
    monkeypatch.setattr(reading, "read_episode", lambda _: episode())
    monkeypatch.setattr(lerobot, "convert_episodes", lambda *a, **k: {"passed": True})
    entry = {
        "selected": True,
        "path": raw.name,
        "sha256": file_sha256(raw),
        "episode_id": "episode-001",
        "seed": -1,
        "source": "webcam",
    }
    manifest = tmp_path / "selection.json"
    args = argparse.Namespace(
        manifest=manifest,
        output=tmp_path / "out",
        repo_id="local/test",
        fps=25,
        cameras=None,
    )
    manifest.write_text(json.dumps({"manifest_version": 1, "episodes": [entry]}))
    assert _recording_convert(args) == {"passed": True}
    entry["seed"] = 22000
    manifest.write_text(json.dumps({"manifest_version": 1, "episodes": [entry]}))
    with pytest.raises(ValueError, match="raw seed"):
        _recording_convert(args)


def test_camera_projection_preserves_numeric_frames_and_split(tmp_path: Path) -> None:
    raw = episode()
    paired = replace(
        raw,
        samples=tuple(
            replace(s, images=(s.images[0], replace(s.images[0], camera_id="wrist")))
            for s in raw.samples
        ),
    )
    selection = {
        "selection_manifest": {
            "episodes": [
                {"episode_id": raw.episode_id, "selected": True, "split": "train"}
            ]
        }
    }
    before = convert_episodes(
        [paired],
        tmp_path / "paired",
        "local/paired",
        25,
        selection.copy(),
        dataset_class=FakeDataset,
    )
    paired_frames = FakeDataset.latest.episodes[0]
    after = convert_episodes(
        [paired],
        tmp_path / "wrist",
        "local/wrist",
        25,
        selection.copy(),
        dataset_class=FakeDataset,
        cameras=("wrist",),
    )
    wrist_frames = FakeDataset.latest.episodes[0]
    assert before["dataset_fingerprint"] != after["dataset_fingerprint"]
    for p, w in zip(paired_frames, wrist_frames, strict=True):
        for key in ("observation.state", "action", "observation.images.wrist"):
            np.testing.assert_array_equal(p[key], w[key])
        assert "observation.images.policy" not in w
    assert (tmp_path / "paired/act_lab_splits.json").read_bytes() == (
        tmp_path / "wrist/act_lab_splits.json"
    ).read_bytes()
    for cameras in ((), ("wrist", "wrist"), ("absent",)):
        with pytest.raises(ValueError):
            convert_episodes(
                [paired],
                tmp_path / "invalid",
                "local/invalid",
                25,
                selection.copy(),
                dataset_class=FakeDataset,
                cameras=cameras,
            )


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


@pytest.mark.parametrize(
    "alignment,source",
    [
        ("same-sample", "applied"),
        ("next-command", "applied"),
        ("next-command", "requested"),
    ],
)
def test_real_lerobot_dataset_round_trip_when_data_profile_is_installed(
    tmp_path: Path,
    alignment: str,
    source: str,
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
        action_alignment=alignment,
        action_source=source,
    )
    dataset = LeRobotDataset("test/act-lab", root=output)
    assert len(dataset) == (1 if alignment == "next-command" else 2)
    assert dataset[0]["action"][0].item() == pytest.approx(
        0.41 if alignment == "next-command" else 0.4
    )
    np.testing.assert_allclose(
        dataset[0]["observation.state"].numpy(),
        np.asarray((0.0,) * 6 + (0.5,), dtype=np.float32),
    )
    assert tuple(dataset[0]["observation.images.policy"].shape) == (3, 2, 2)


def test_causal_alignment_preserves_observation_and_uses_next_raw_command() -> None:
    from act_lab.application.dataset import align_next_commands

    original = episode()
    original = replace(
        original,
        samples=(
            original.samples[0],
            replace(
                original.samples[1],
                action_quaternion_wxyz=(0.0, 1.0, 0.0, 0.0),
                action_gripper_position=0.8,
                action_enabled=False,
                command_outcome="disabled",
            ),
            original.samples[2],
        ),
    )
    aligned = align_next_commands(original)
    assert len(aligned.samples) == 2
    for current, result, following in zip(
        original.samples, aligned.samples, original.samples[1:], strict=False
    ):
        assert result.timestamp_ns == current.timestamp_ns
        assert result.joint_positions_rad == current.joint_positions_rad
        assert result.images == current.images
        assert result.action_position_xyz_m == following.action_position_xyz_m
        assert result.action_quaternion_wxyz == following.action_quaternion_wxyz
        assert result.action_gripper_position == following.action_gripper_position
        assert result.action_enabled == following.action_enabled
        assert result.command_outcome == following.command_outcome
        assert result.gripper_position == current.gripper_position
    # Alignment precedes 25 Hz resampling: the label is 40 ms, not 60 ms.
    sampled = resample_episode(aligned, 25)
    assert len(sampled.samples) == 1
    assert (
        sampled.samples[0].action_position_xyz_m
        == original.samples[1].action_position_xyz_m
    )
    assert len(original.samples) == 3


@pytest.mark.parametrize("gap", [0, -1, 100_000_000])
def test_causal_alignment_rejects_invalid_command_intervals(gap: int) -> None:
    from act_lab.application.dataset import align_next_commands

    original = episode()
    first = original.samples[0]
    following = replace(original.samples[1], timestamp_ns=first.timestamp_ns + gap)
    with pytest.raises(ValueError, match="interval"):
        align_next_commands(replace(original, samples=(first, following)))
    with pytest.raises(ValueError, match="at least two"):
        align_next_commands(replace(original, samples=(first,)))


def test_causal_conversion_has_distinct_version_and_fingerprint(tmp_path: Path) -> None:
    legacy = convert_episodes(
        [episode()],
        tmp_path / "legacy",
        "test/legacy",
        25,
        {},
        dataset_class=FakeDataset,
    )
    causal = convert_episodes(
        [episode()],
        tmp_path / "causal",
        "test/causal",
        25,
        {},
        dataset_class=FakeDataset,
        action_alignment="next-command",
    )
    assert legacy["dataset_fingerprint"] != causal["dataset_fingerprint"]
    assert legacy["frames"] == 2 and causal["frames"] == 1
    lineage = json.loads((tmp_path / "causal/act_lab_lineage.json").read_text())
    assert lineage["converter_version"] == "3"
    assert lineage["action_alignment"] == "next-command"
    assert FakeDataset.latest.episodes[0][0]["action"][0] == pytest.approx(0.41)


def test_requested_conversion_versions_labels_and_rejects_noncausal_use(
    tmp_path: Path,
) -> None:
    applied = convert_episodes(
        [episode()],
        tmp_path / "applied",
        "local/test",
        25,
        {},
        dataset_class=FakeDataset,
        action_alignment="next-command",
    )
    requested = convert_episodes(
        [episode()],
        tmp_path / "requested",
        "local/test",
        25,
        {},
        dataset_class=FakeDataset,
        action_alignment="next-command",
        action_source="requested",
    )
    assert applied["dataset_fingerprint"] != requested["dataset_fingerprint"]
    lineage = json.loads((tmp_path / "requested/act_lab_lineage.json").read_text())
    assert lineage["converter_version"] == "4"
    assert lineage["action_source"] == "requested"
    assert lineage["action_alignment"] == "next-command"
    with pytest.raises(ValueError, match="requires next-command"):
        convert_episodes(
            [episode()],
            tmp_path / "bad",
            "local/test",
            25,
            {},
            dataset_class=FakeDataset,
            action_source="requested",
        )


@pytest.mark.parametrize("invalid_source", [None, "applied", "requested"])
def test_requested_conversion_rechecks_both_source_quality_paths(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    invalid_source: str | None,
) -> None:
    from act_lab.adapters import lerobot
    from act_lab.adapters.mcap import reading
    from act_lab.application.dataset import file_sha256
    from act_lab.cli import _recording_convert

    raw = tmp_path / "raw.mcap"
    raw.write_bytes(b"fixture")
    calls = []

    def read(path, *, action_source="applied"):
        calls.append(action_source)
        result = episode()
        if action_source == invalid_source:
            result = replace(
                result,
                samples=(
                    replace(
                        result.samples[0],
                        action_position_xyz_m=(float("nan"), 0.0, 0.0),
                    ),
                )
                + result.samples[1:],
            )
        return result

    monkeypatch.setattr(reading, "read_episode", read)
    monkeypatch.setattr(lerobot, "convert_episodes", lambda *a, **k: k)
    manifest = tmp_path / "selection.json"
    manifest.write_text(
        json.dumps(
            {
                "manifest_version": 1,
                "episodes": [
                    {
                        "selected": True,
                        "path": raw.name,
                        "sha256": file_sha256(raw),
                        "episode_id": "episode-001",
                    }
                ],
            }
        )
    )
    args = argparse.Namespace(
        manifest=manifest,
        output=tmp_path / "out",
        repo_id="local/test",
        fps=25,
        cameras=None,
        action_source="requested",
        action_alignment="next-command",
    )
    if invalid_source is None:
        result = _recording_convert(args)
        assert result["action_source"] == "requested"
        assert calls == ["applied", "requested"]
    else:
        with pytest.raises(ValueError, match="passes selection|quality"):
            _recording_convert(args)
        assert calls == (
            ["applied"] if invalid_source == "applied" else ["applied", "requested"]
        )
