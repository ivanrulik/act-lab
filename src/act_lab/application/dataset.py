"""Quality validation, deterministic resampling, selection and lineage."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any

from act_lab.domain.dataset import (
    IssueSeverity,
    QualityIssue,
    QualityReport,
    RecordedEpisode,
    RecordedSample,
)

CONVERTER_VERSION = "1"


def validate_episode(episode: RecordedEpisode) -> QualityReport:
    issues: list[QualityIssue] = []

    def issue(code: str, severity: IssueSeverity, message: str) -> None:
        issues.append(QualityIssue(code, severity, message))

    if not episode.complete:
        issue(
            "incomplete",
            IssueSeverity.ERROR,
            "MCAP has no valid footer; recover it first",
        )
    if not episode.episode_id:
        issue(
            "missing_provenance",
            IssueSeverity.ERROR,
            "episode provenance or ID is missing",
        )
    if episode.outcome in {"unknown", "interrupted"}:
        issue(
            "unfinished", IssueSeverity.ERROR, f"episode outcome is {episode.outcome}"
        )
    elif episode.outcome != "success":
        issue(
            "not_successful",
            IssueSeverity.WARNING,
            f"{episode.outcome} episodes are preserved but excluded from training",
        )
    if (
        episode.outcome in {"success", "failure"}
        and episode.last_task_outcome in {"success", "failure"}
        and episode.outcome != episode.last_task_outcome
    ):
        issue(
            "inconsistent_task_label",
            IssueSeverity.ERROR,
            "terminal episode label disagrees with the last simulator task label",
        )
    elif episode.outcome == "success" and episode.last_task_outcome == "unknown":
        issue(
            "unconfirmed_task_success",
            IssueSeverity.WARNING,
            "success label has no terminal simulator task confirmation",
        )
    if not episode.samples:
        issue(
            "no_samples",
            IssueSeverity.ERROR,
            "no synchronized observation/command samples",
        )
    stream_counts = dict(episode.stream_counts)
    observation_count = stream_counts.get("/observation", 0)
    command_count = stream_counts.get("/command", 0)
    camera_counts = {
        topic: count
        for topic, count in stream_counts.items()
        if topic.startswith("/camera/")
    }
    if observation_count != command_count or any(
        count != observation_count for count in camera_counts.values()
    ):
        issue(
            "unsynchronized_streams",
            IssueSeverity.ERROR,
            "observation, command, and scene-camera counts do not match",
        )
    if not camera_counts:
        issue("missing_camera", IssueSeverity.ERROR, "no policy scene-camera stream")
    acquisition = {}
    try:
        resolved = json.loads(str(episode.provenance.get("resolved_config_json", "{}")))
        acquisition = resolved.get("acquisition", {})
    except (TypeError, ValueError):
        issue(
            "bad_config",
            IssueSeverity.ERROR,
            "resolved configuration is not valid JSON",
        )
    if episode.provenance.get("source") == "webcam" and not acquisition:
        issue(
            "legacy_acquisition_metadata",
            IssueSeverity.WARNING,
            "webcam acquisition identity is absent "
            "(recorded before late PR 6 metadata)",
        )
    timestamps = [sample.timestamp_ns for sample in episode.samples]
    if any(
        right <= left for left, right in zip(timestamps, timestamps[1:], strict=False)
    ):
        issue(
            "non_monotonic",
            IssueSeverity.ERROR,
            "sample timestamps are not strictly increasing",
        )
    if len(timestamps) > 2:
        periods = [
            right - left
            for left, right in zip(timestamps, timestamps[1:], strict=False)
        ]
        nominal = sorted(periods)[len(periods) // 2]
        if any(period > nominal * 2 for period in periods):
            issue(
                "sample_gap",
                IssueSeverity.ERROR,
                "sample gap exceeds two nominal periods",
            )
    gap_start: int | None = None
    longest_tracking_gap = 0
    for timestamp, state in episode.tracking_states:
        unavailable = state in {
            "tracking_lost",
            "stale_camera_frame",
            "low_confidence",
            "worker_failure",
        }
        if unavailable and gap_start is None:
            gap_start = timestamp
        elif not unavailable and gap_start is not None:
            longest_tracking_gap = max(longest_tracking_gap, timestamp - gap_start)
            gap_start = None
    if gap_start is not None and episode.tracking_states:
        longest_tracking_gap = max(
            longest_tracking_gap, episode.tracking_states[-1][0] - gap_start
        )
    if longest_tracking_gap > 100_000_000:
        issue(
            "tracking_gap",
            IssueSeverity.ERROR,
            f"tracking was unavailable for {longest_tracking_gap / 1e6:.1f} ms",
        )
    expected_joints = (
        len(episode.samples[0].joint_positions_rad) if episode.samples else 0
    )
    cameras = {image.camera_id for sample in episode.samples for image in sample.images}
    camera_shapes = (
        {
            image.camera_id: (image.width, image.height)
            for image in episode.samples[0].images
        }
        if episode.samples
        else {}
    )
    for index, sample in enumerate(episode.samples):
        numeric = (
            sample.joint_positions_rad
            + sample.joint_velocities_rad_s
            + sample.end_effector_position_xyz_m
            + sample.end_effector_quaternion_wxyz
            + (sample.gripper_position,)
            + sample.action_position_xyz_m
            + sample.action_quaternion_wxyz
            + (sample.action_gripper_position,)
        )
        if not all(math.isfinite(value) for value in numeric):
            issue(
                "non_finite",
                IssueSeverity.ERROR,
                f"sample {index} contains NaN or infinity",
            )
            break
        if (
            math.sqrt(
                sum(value * value for value in sample.end_effector_quaternion_wxyz)
            )
            < 1e-12
            or math.sqrt(sum(value * value for value in sample.action_quaternion_wxyz))
            < 1e-12
        ):
            issue(
                "zero_quaternion",
                IssueSeverity.ERROR,
                f"sample {index} contains a zero-norm quaternion",
            )
            break
        if (
            expected_joints == 0
            or len(sample.joint_positions_rad) != expected_joints
            or len(sample.joint_velocities_rad_s) != expected_joints
            or len(sample.end_effector_position_xyz_m) != 3
            or len(sample.end_effector_quaternion_wxyz) != 4
            or len(sample.action_position_xyz_m) != 3
            or len(sample.action_quaternion_wxyz) != 4
        ):
            issue(
                "bad_dimensions",
                IssueSeverity.ERROR,
                f"sample {index} has incompatible dimensions",
            )
            break
        present = {image.camera_id for image in sample.images}
        if present != cameras:
            issue(
                "missing_image",
                IssueSeverity.ERROR,
                f"sample {index} is missing a camera frame",
            )
            break
        if any(
            image.encoding != "rgb8"
            or image.width <= 0
            or image.height <= 0
            or len(image.rgb_bytes) != image.width * image.height * 3
            for image in sample.images
        ):
            issue(
                "bad_image",
                IssueSeverity.ERROR,
                f"sample {index} has invalid RGB image data",
            )
            break
        if any(
            camera_shapes.get(image.camera_id) != (image.width, image.height)
            for image in sample.images
        ):
            issue(
                "image_dimensions_changed",
                IssueSeverity.ERROR,
                f"sample {index} changes camera dimensions",
            )
            break
    rejected = sum(
        sample.command_outcome not in {"applied", "limited"}
        for sample in episode.samples
    )
    if rejected:
        issue(
            "rejected_commands",
            IssueSeverity.WARNING,
            f"{rejected}/{len(episode.samples)} commands were not applied",
        )
    valid = not any(item.severity is IssueSeverity.ERROR for item in issues)
    duration = timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0
    return QualityReport(
        episode.episode_id,
        episode.outcome,
        valid,
        valid and episode.outcome == "success",
        len(episode.samples),
        duration,
        tuple(issues),
    )


def _lerp(
    left: tuple[float, ...], right: tuple[float, ...], ratio: float
) -> tuple[float, ...]:
    return tuple(a + (b - a) * ratio for a, b in zip(left, right, strict=True))


def resample_episode(episode: RecordedEpisode, fps: int) -> RecordedEpisode:
    if fps <= 0 or 1_000_000_000 % fps:
        raise ValueError("fps must be a positive integer divisor of 1,000,000,000")
    if not episode.samples:
        return episode
    period = 1_000_000_000 // fps
    source = episode.samples
    targets = range(source[0].timestamp_ns, source[-1].timestamp_ns + 1, period)
    result: list[RecordedSample] = []
    right = 0
    for timestamp in targets:
        while right < len(source) and source[right].timestamp_ns < timestamp:
            right += 1
        if right == 0:
            result.append(replace(source[0], timestamp_ns=timestamp))
            continue
        if right == len(source):
            break
        before, after = source[right - 1], source[right]
        ratio = (timestamp - before.timestamp_ns) / (
            after.timestamp_ns - before.timestamp_ns
        )
        nearest = before if ratio <= 0.5 else after
        quaternion = _lerp(
            before.action_quaternion_wxyz, after.action_quaternion_wxyz, ratio
        )
        norm = math.sqrt(sum(value * value for value in quaternion))
        result.append(
            RecordedSample(
                timestamp,
                _lerp(before.joint_positions_rad, after.joint_positions_rad, ratio),
                _lerp(
                    before.joint_velocities_rad_s, after.joint_velocities_rad_s, ratio
                ),
                _lerp(
                    before.end_effector_position_xyz_m,
                    after.end_effector_position_xyz_m,
                    ratio,
                ),
                _lerp(
                    before.end_effector_quaternion_wxyz,
                    after.end_effector_quaternion_wxyz,
                    ratio,
                ),
                before.gripper_position
                + (after.gripper_position - before.gripper_position) * ratio,
                _lerp(before.action_position_xyz_m, after.action_position_xyz_m, ratio),
                tuple(value / norm for value in quaternion),
                before.action_gripper_position
                + (after.action_gripper_position - before.action_gripper_position)
                * ratio,
                nearest.action_enabled,
                nearest.command_outcome,
                nearest.images,
            )
        )
    return replace(episode, samples=tuple(result))


def file_sha256(path: Path) -> str:
    with path.open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def split_for_episode(episode_id: str, seed: int, validation_fraction: float) -> str:
    if not 0.0 <= validation_fraction <= 1.0:
        raise ValueError("validation fraction must be in [0, 1]")
    value = int.from_bytes(
        hashlib.sha256(f"{seed}:{episode_id}".encode()).digest()[:8], "big"
    )
    return "validation" if value / 2**64 < validation_fraction else "train"


def report_dict(report: QualityReport) -> dict[str, Any]:
    return asdict(report)
