# Record and convert ROS simulation episodes

## Continuous simulation capture

The existing commands below remain stepped by default. To capture while the
CRISP owner continues integrating physics, select `--controller crisp --paced`:

```bash
docker compose --profile ros2-recording-gpu run --build --rm ros2-recording-gpu \
  act-lab ros2 recording-demo --controller crisp --paced --steps 30 \
  --output runs/continuous-ros --json
```

This explicitly requests a verified NVIDIA EGL renderer. Original full-quality
RGB, complete command reports, metadata, canonical mirror and rosbag import are
retained. `capture` in the report contains worker PID, renderer identity, packet
sequence, per-sample receipt-to-ACK age and selected capture profile. This bounded
movement is labelled failure, not a successful task demonstration.

For CPU fault/identity qualification with a small wrist-only fixture:

```bash
docker compose --profile ros2-recording run --rm ros2-recording \
  act-lab ros2 recording-smoke --controller crisp --paced \
  --capture-profile reduced-effects \
  --config configs/sim/ur5e_2f85_d405_capture_bench.toml \
  --output runs/continuous-cpu --json
```

Reduced effects are an explicit quality choice with a different camera fingerprint.
They are never an automatic fallback and are incompatible with full-quality
checkpoints. Four pending samples maximum and a 100 ms deadline bound acquisition.
Slow rendering/ACKs, queue overload and worker loss fail acquisition through the
existing disabled hold. `capture-failure.json`, canonical partial files and
`bag.partial` remain evidence; do not train from them. Finish an attempt before
resetting. A fresh attempt uses a new worker/UUID and continuous packet ordering.
Long-duration throughput, hardware safety and interactive Foxglove acceptance
remain separate qualification; use the existing saved bag layout for replay.

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
reset, loss, backpressure and paced/hardware requirements.
