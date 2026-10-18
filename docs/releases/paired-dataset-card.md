---
license: apache-2.0
tags:
- robotics
- lerobot
- mujoco
- imitation-learning
pretty_name: ACT Lab articulated UR5e paired-camera pick and place
---

# ACT Lab paired wrist/fixed-camera demonstrations

40 simulated scripted-expert episodes, seeds 20000–20039, with the articulated
Robotiq 2F-85 and D405-inspired synthetic RGB wrist camera. **30 training / 10
reserved episodes**, 11,624 frames at 25 Hz. Original wrist and fixed policy RGB
are 240×320. No overview input, physical sensor footage or depth is included.

## Data contract

LeRobotDataset v3 for pinned LeRobot 0.6.1. State: six canonical UR5e joint
positions in radians and normalized aperture. Action: world XYZ in meters,
WXYZ quaternion and normalized aperture, eight float values. Aperture zero is
closed and one is open. `payload/act_lab_splits.json` is authoritative; the
reserved episode indices are 0, 4, 9, 10, 12, 13, 22, 25, 26, 34. Never split
individual frames across training and validation.

Converter v4 uses **next-command requested intent**. Images and robot state
remain current; next raw-step labels are selected before resampling. Accepted
requests teach intent before application response filtering. Rejected/disabled
requests teach safe holds with the last accepted aperture. This avoids teaching
already-achieved post-action state or filtering an applied target twice.

Logical fingerprint:
`49e3f6f6fa7f016bef373cf4050b88f8b6c81905106332cdde70813291a2be04`.
Storage fingerprint:
`baace059af172bc39a3df8c4a06d7066493dc61c82681a5b845da07860ce6d52`.
Lineage includes source MCAP hashes, quality decisions, conversion rules and
camera/model contracts. Raw acquisition MCAP is excluded; full conversion from
raw data requires the separately retained acquisition archive.

## Use and limits

The paired ACT model trained on these data achieved 90/100 fresh simulation
tasks under its documented configuration. This is a model result, not a claim
that this dataset solves arbitrary manipulation. Small scripted dataset, one
cube/tray task, fixed optics and bounded spawn distribution; no domain
randomization or physical validation. Wrist-only sufficiency is unresolved.

See `reproduction/paired-reproduction.md`, `inference.json`, retained evidence,
and [ACT Lab](https://github.com/ivanrulik/act-lab). Dataset and source metadata
are Apache-2.0 under ACT Lab's declaration. UR5e and Robotiq Menagerie source
notices are retained separately. Training needs no ROS, Hub auth or CRISP.
