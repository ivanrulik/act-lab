---
license: apache-2.0
tags:
- robotics
- lerobot
- mujoco
- imitation-learning
pretty_name: ACT Lab educational UR5e pick and place
---

# ACT Lab educational UR5e demonstrations

78 quality-selected scripted demonstrations of simulated cube pick/place:
58 train episodes and 20 reserved episodes, 16,118 frames at 25 Hz.
The immutable episode split is in `payload/act_lab_splits.json`.
Each episode is linked to original MCAP hashes and quality decisions in
`payload/act_lab_lineage.json`. Raw MCAP is not included in this first release;
the complete dataset is usable, but full raw-to-derived reconstruction requires
the separately retained acquisition archive.

## Features and use

LeRobotDataset v3, LeRobot 0.6.1, Python 3.12. Two RGB cameras (`overview`,
`policy`), 240×320 pixels. State is six joint positions in radians plus
normalized aperture. Action is world-frame Cartesian XYZ in meters, WXYZ
quaternion, and normalized gripper intent. Feed intent through ACT Lab's
`SafeCartesianRobot`; these values are not joint-effort or hardware commands.

Download the commit in the release receipt using `act-lab release download`.
Point the training CLI to the downloaded `payload` directory, which retains the
original finalized-byte fingerprint. Never change the train/reserved split to
match a desired result. Dataset cards and evidence are outside the payload so
adding publication metadata does not change dataset identity.

## Scope and limitations

This is the legacy educational MuJoCo robot with two cameras. It does not
describe the articulated 2F85/D405-inspired wrist model or physical UR hardware.
All images are simulator scene images; no operator webcam images are included.
Demonstrations use bounded seeded cube placement and one scripted task.
Selected successful expert demonstrations do not represent diverse human
behavior, arbitrary tasks, or failure recovery demonstrations.

The training run records a dirty Git checkout: its source commit alone cannot
fully reconstruct historical source state. Release smoke checks establish
current artifact loading, not bit-identical retraining or hardware safety.

## Attribution and license

ACT Lab publishes its generated demonstration data under Apache-2.0, matching
the project declaration. The simulated UR5e assets carry ROS Industrial source
notices, retained in `ASSET-NOTICE.txt`; no meshes are included in this dataset.
See [ACT Lab](https://github.com/ivanrulik/act-lab) for pinned simulation assets,
the source licenses, validation rules, conversion and evaluation commands.
