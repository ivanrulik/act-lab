"""Verify every causal requested label and unchanged paired input against sources."""

import argparse
import json
import math
from pathlib import Path

import pyarrow.parquet as pq
from mcap.reader import make_reader

from act_lab.adapters.mcap.schema import decode
from act_lab.application.dataset import file_sha256
from act_lab.application.training import verify_dataset

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--pilot", type=Path, required=True)
parser.add_argument("--raw-pilot", type=Path, required=True)
parser.add_argument("--reference", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    parser.error("output already exists")
root = args.pilot / "paired/dataset"
identity = verify_dataset(root)
reference = verify_dataset(args.reference)
assert identity.train_episode_ids == reference.train_episode_ids
assert identity.reserved_episode_ids == reference.reserved_episode_ids
lineage = json.loads((root / "act_lab_lineage.json").read_text())
assert lineage["converter_version"] == "4"
assert lineage["action_alignment"] == "next-command"
assert lineage["action_source"] == "requested"
selection = json.loads((args.raw_pilot / "selection.json").read_text())
assert lineage["selection_manifest"] == selection
period = 1_000_000_000 // lineage["fps"]
expected = {}
reconstructed = 0
for episode_index, entry in enumerate(
    e for e in selection["episodes"] if e["selected"]
):
    path = args.raw_pilot / entry["path"]
    assert file_sha256(path) == entry["sha256"]
    raw = {}
    with path.open("rb") as source:
        for schema, channel, record in make_reader(source).iter_messages(
            topics=["/observation", "/command"]
        ):
            value = decode(schema.name.rsplit(".", 1)[-1], record.data)
            raw.setdefault(record.log_time, {})[channel.topic] = value
    stamps = sorted(raw)
    targets = {}
    aperture = None
    for stamp in stamps:
        command = raw[stamp]["/command"]
        if command.outcome in {"applied", "limited"}:
            assert command.HasField("executed_action")
            aperture = command.executed_action.gripper_position
            action = command.requested_action
            pose, gripper = action.target_pose, action.gripper_position
        elif command.HasField("executed_action"):
            assert not command.executed_action.enabled
            pose = command.executed_action.target_pose
            gripper = command.executed_action.gripper_position
        else:
            assert aperture is not None
            pose = raw[stamp]["/observation"].robot.end_effector_pose
            gripper = aperture
            reconstructed += 1
        targets[stamp] = [*pose.position_xyz_m, *pose.quaternion_wxyz, gripper]
    following = dict(zip(stamps, stamps[1:], strict=False))
    for frame_index, stamp in enumerate(range(stamps[0], stamps[-2] + 1, period)):
        state = raw[stamp]["/observation"].robot
        expected[episode_index, frame_index] = (
            [*state.joint_positions_rad, state.gripper_position],
            targets[following[stamp]],
        )


def batches(path):
    for file in sorted((path / "data").rglob("*.parquet")):
        yield from pq.ParquetFile(file).iter_batches(batch_size=64)


frames = 0
images = 0
max_action_error = 0.0
max_state_error = 0.0
for new_batch, old_batch in zip(batches(root), batches(args.reference), strict=True):
    for new, old in zip(new_batch.to_pylist(), old_batch.to_pylist(), strict=True):
        key = (new["episode_index"], new["frame_index"])
        state, target = expected.pop(key)
        for field in [
            "index",
            "episode_index",
            "frame_index",
            "timestamp",
            "observation.state",
        ]:
            assert new[field] == old[field], (key, field)
        for camera in lineage["camera_selection"]:
            field = f"observation.images.{camera}"
            assert new[field] == old[field], (key, field)
            images += 1
        max_action_error = max(
            max_action_error,
            max(abs(a - b) for a, b in zip(new["action"], target, strict=True)),
        )
        max_state_error = max(
            max_state_error,
            max(
                abs(a - b) for a, b in zip(new["observation.state"], state, strict=True)
            ),
        )
        frames += 1
assert not expected
assert math.isfinite(max_action_error) and max_action_error < 1e-6
assert math.isfinite(max_state_error) and max_state_error < 1e-6
result = {
    "schema_version": 1,
    "dataset_fingerprint": identity.logical_fingerprint,
    "storage_fingerprint": identity.storage_fingerprint,
    "reference_fingerprint": reference.logical_fingerprint,
    "frames": frames,
    "unchanged_encoded_images": images,
    "reconstructed_raw_holds": reconstructed,
    "max_action_float32_error": max_action_error,
    "max_state_float32_error": max_state_error,
    "raw_hashes_unchanged": True,
    "episode_splits_unchanged": True,
}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result))
