# Record and convert ROS simulation episodes

```bash
docker compose --profile ros2-recording build ros2-recording
docker compose --profile ros2-recording run --rm ros2-recording \
  act-lab ros2 recording-smoke --output runs/ros-recording-smoke --json
```

The smoke runs real local simulation acquisition, original RGB, two-process DDS,
rosbag2 MCAP and exact canonical import comparison. It is deliberately short and
normally labelled failure; it is evidence of transport, not a training demonstration.

## Complete expert attempt

```bash
docker compose --profile ros2-recording run --rm ros2-recording \
  act-lab ros2 recording-demo --output runs/ros-expert --json
```

Default seed zero uses the articulated gripper/wrist RGB preset. `--config` selects
an explicit model, `--seed` selects a reproducible reset, and `--steps` bounds a
run (zero uses the configured episode limit). One session writes one bag. Run a
new output directory for each CLI session. The demo retains a canonical local
reference and imports the separate rosbag2 recording into `imported/`.
The successful/failed outcome comes from the task; original frames and safety
reports remain inspectable. Read `report.json` and the quality decisions.
CPU capture/import can take several minutes at full resolution; wall duration is
not simulation duration or a claim of real-time acquisition throughput.

For actual CRISP-controlled motion with independent physics ownership:

```bash
docker compose --profile ros2-recording run --rm ros2-recording \
  act-lab ros2 recording-demo --controller crisp --steps 20 \
  --output runs/ros-crisp-recording --json
```

This records a bounded nearby movement and disabled final hold as a **failure**
attempt, excluded from training. It demonstrates synchronized stepped capture of
the real controller, not pick/place success. Rendering occurs in the scratch gateway.
Paced capture is unsupported. Finish a recorded attempt before resetting; the
low-level sink supports subsequent attempts using new episode UUIDs and continuing
packet sequences. Never run multiple producers or recorders on the same ROS domain.

## Replay in Foxglove

Open `rosbag/bag/bag.partial_0.mcap` as a local MCAP file in Foxglove and import
`configs/ros2/foxglove/act-lab-recording.json`. The filename retains the original
staging basename even though the directory is finalized. The layout shows original
wrist RGB, source command age, command reports and acquisition metadata. All three
camera raw-image topics and their CameraInfo projections are present. Scrub using
bag receipt time; source timestamps retain the simulation clock and reset offset.
This layout has no robot model or live command controls. The existing live tooling
layout serves a different stream; the deferred flange appearance review remains open.
File CDR/schema tests do not constitute interactive Foxglove visual acceptance.

## Import an existing complete acquisition bag

```bash
docker compose --profile ros2-recording run --rm ros2-recording \
  act-lab ros2 recording-import --bag runs/ros-expert/rosbag/bag \
  --output runs/ros-expert/import-again --json
```

Import requires the generated acquisition stream and complete lifecycle. Generic
viewer-only bags cannot become policy datasets by joining image/state topics.
Unfinished streams fail, retaining source evidence and a staged import directory.
No existing output is overwritten. The importer verifies MCAP CRCs and embeds
source hashes in canonical files and `import.json`.

## Validate, select and convert without ROS

```bash
docker compose run --rm dev act-lab recording manifest \
  runs/ros-expert/imported --output runs/ros-selection.json \
  --validation-fraction 0.2
docker compose --profile training run --rm train-cpu act-lab recording convert \
  runs/ros-selection.json --output runs/ros-dataset \
  --repo-id local/ros-expert --fps 25
```

The existing manifest retains rejected attempts and splits complete episodes.
Conversion rechecks raw hashes and eligibility. One episode may hash to validation;
use an adequate demonstration cohort for training. Do not adjust labels to force
selection. For reproducible equivalence qualification in the CPU image:

```bash
docker compose --profile training run --rm train-cpu \
  python scripts/check-ros-recording-equivalence.py \
  --capture runs/ros-expert --output runs/ros-conversion-equivalence
```

This runs the public quality/manifest/conversion CLI for both canonical versions,
verifies dataset lineage and compares logical fingerprints without loading ROS.
See the [contract and safety matrix](contracts/ros2-recording.md) for interruption,
reset, loss, backpressure and deferred paced/hardware requirements.
