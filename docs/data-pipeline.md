# Validate, replay, select, and convert episodes

Raw MCAP remains immutable. The commands below only create reports, replay
exports, manifests, and derived datasets; rejected raw episodes are never moved
or deleted.

## Quick demo

```bash
./scripts/demo-data.sh record
./scripts/demo-data.sh replay
./scripts/demo-data.sh validate
./scripts/demo-data.sh convert
./scripts/demo-data.sh inspect
```

`record` creates five scripted pick-and-place attempts. `replay` opens the most
recent scene-camera recording. `convert` selects valid successful attempts and
writes `data/lerobot/pick-place`; `inspect` loads that directory through
LeRobotDataset and prints its episodes, frames, and features.

The manifest and output directory are intentionally not overwritten. To repeat
conversion, choose new paths with the full commands below or deliberately remove
the previous derived outputs. Raw MCAP files are never removed by the helper.

## Quality validation

```bash
docker compose run --rm dev act-lab recording validate data/raw/*.mcap
```

Validation returns exit status 1 when any file has a quality error and status 2
for an unreadable/corrupt input or invalid command. Reports identify incomplete
files, missing or unsynchronized streams, non-monotonic timestamps, gaps larger
than two nominal periods, non-finite or incompatible state/action values,
malformed/changing RGB frames, and contradictory terminal task labels. Rejected
commands and non-success outcomes are reported. Only complete, valid, successful
episodes are training-eligible.

Webcam recordings made before the late PR 6 acquisition-identity addition remain
eligible when otherwise valid. They receive a
`legacy_acquisition_metadata` warning so that limitation is never silent.

## Replay

Inspect synchronization without a display, or export portable PPM frames:

```bash
docker compose run --rm dev act-lab recording replay EPISODE.mcap
docker compose run --rm dev act-lab recording replay EPISODE.mcap \
  --camera policy --render-dir runs/replay --max-frames 100
```

An interactive OpenCV window is available with `--display` in an appropriately
configured display container; `q` or Escape closes it. Replay reads scene-camera
frames only. Operator webcam pixels were never recorded.

## Freeze selection and episode splits

```bash
docker compose run --rm dev act-lab recording manifest data/raw/*.mcap \
  --output data/selection.json --split-seed 0 --validation-fraction 0.2
```

By default every eligible episode is selected. Repeat
`--include-episode EPISODE_ID` to freeze an explicit reviewed subset.

The versioned JSON manifest records every candidate's relative path, SHA-256,
quality report, selection decision, and split. Splits hash `seed:episode_id`, so
all frames from an episode stay together and repeated runs are stable. Failure,
discarded, interrupted, and invalid entries remain in the manifest as rejected
evidence. Review the manifest before conversion.

## Convert to LeRobotDataset v3

Build the isolated, CPU-only conversion image and convert the frozen manifest:

```bash
docker compose --profile data build data
docker compose --profile data run --rm data act-lab recording convert \
  data/selection.json --output data/lerobot/pick-place \
  --repo-id local/act-lab-pick-place --fps 25
```

The converter verifies every selected raw hash and reruns quality validation.
State is six joint positions plus gripper position. Action is the safe executed
Cartesian position, normalized quaternion, and gripper target; a rejected command
uses its recorded request and remains visible through the quality warning. Scene
images use LeRobot image features (not video) to keep the small conversion path
simple and deterministic.

Numeric streams use linear interpolation on an integer-nanosecond grid. Discrete
fields and RGB frames use nearest-neighbor selection with earlier-frame tie
breaking. Output is first written to a `.partial` sibling and published only
after LeRobot finalization. `act_lab_lineage.json` embeds the selection manifest,
converter version, source episode IDs, FPS, and a SHA-256 fingerprint over the
canonical resampled values and image bytes. `act_lab_splits.json` maps each
source episode and its LeRobot episode index to the frozen train or validation
split. Existing output or partial paths are never overwritten.

LeRobot 0.4.4 is pinned because it provides Dataset v3 while supporting the
project's Python 3.11 baseline. Later LeRobot releases require Python 3.12. The
data image pins CPU-only PyTorch because PR 7 converts data and does not train.
ACT integration and explicit training device selection remain PR 8.
