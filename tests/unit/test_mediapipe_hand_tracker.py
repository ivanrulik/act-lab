from __future__ import annotations

import pytest

from act_lab.adapters.mediapipe.hand_tracker import _apparent_scale


def test_apparent_scale_tracks_projected_size_in_both_directions() -> None:
    landmarks = tuple(
        (index / 100.0, index / 200.0, index / 50.0) for index in range(21)
    )
    smaller = tuple((x * 0.5, y * 0.5, z) for x, y, z in landmarks)
    larger = tuple((x * 2.0, y * 2.0, z) for x, y, z in landmarks)

    baseline = _apparent_scale(landmarks)

    assert _apparent_scale(smaller) == pytest.approx(baseline * 0.5)
    assert _apparent_scale(larger) == pytest.approx(baseline * 2.0)


def test_apparent_scale_ignores_pose_relative_landmark_depth() -> None:
    landmarks = tuple((index / 100.0, index / 200.0, 0.0) for index in range(21))
    changed_depth = tuple(
        (x, y, index * 10.0)
        for index, (x, y, _z) in enumerate(landmarks)
    )

    assert _apparent_scale(changed_depth) == pytest.approx(_apparent_scale(landmarks))
