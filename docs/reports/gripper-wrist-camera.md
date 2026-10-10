# Articulated gripper and wrist RGB qualification

Date: 2026-10-10. Branch: `feature/gripper-wrist-camera`.
Base: merged main `25c6e3220e6f4a7742a5cdb8fc83e91fe022a051`.
Decision: [ADR 019](../adr/019-articulated-tool-and-wrist-rgb.md).
Scope: simulation, local learning and optional read-only ROS/Foxglove projection.

## Delivered assembly

`configs/sim/ur5e_2f85_d405.toml` selects the articulated Robotiq 2F-85 and
D405-inspired synthetic RGB camera. The legacy educational preset remains the
default. Generated articulated and nominal 50 mm rigid-payload URDFs are under
`configs/models/ur5e_2f85_d405/`; the matching MJCF and reproducible assembly,
calibration and URDF exporters are checked in.

The final model fingerprint is
`6e414b220b78ca8f7ecb84ec6ac16ab97b9922ae7d43468db1c91408157af125`.
The TCP is 155.8 mm along flange Z. Actual pad separation / 85 mm is feedback;
eight measured passive joints describe the closed linkage. The controller still
owns six arm effort interfaces. Its rigid payload approximates moving fingers.

The camera is offset laterally and tilted 24 degrees toward the grasp. The
parallel mount clipped the loaded cube; the final image-bound assertions pass
for actual contact lift and two-second holds. RGB is 320×240, vertical FOV 58°,
zero distortion, with calculated aspect-correct intrinsics. This is a synthetic
profile, not a physical D405 mode or validated optical model.

### Source and notice audit

- [Menagerie 2F-85](https://github.com/google-deepmind/mujoco_menagerie/tree/0059d4335f8156206f63a35662313385f7ad6d74/robotiq_2f85), revision
  `0059d4335f8156206f63a35662313385f7ad6d74`: source XML, eight STL meshes,
  README, changelog and BSD-2-Clause license retained byte for byte. Manifest
  records hashes and assembly modifications. The upstream simplified linkage,
  contact tuning and original 5 Nm gripper drive remain simulation assumptions.
- [ROS Industrial description](https://github.com/ros-industrial/robotiq/tree/45196f6558fe8ba9d89bc8a105396c68c3e7e892/robotiq_2f_85_gripper_visualization), revision
  `45196f6558fe8ba9d89bc8a105396c68c3e7e892`: inspected package declaration
  says BSD; its macro uses ±1 mimic multipliers. No assets or ROS 1 drivers were
  imported. Actual measured Menagerie linkage coordinates avoid claiming those
  idealized mimic relationships are exact under contact.
- [Official UR description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/6b639c2efd9f4f12da858a72cf1a9e40da365ed8), revision
  `6b639c2efd9f4f12da858a72cf1a9e40da365ed8`: retain BSD-3-Clause source
  notices and embedded robot comments. UR5e meshes are package references, not
  copied here. Special upstream terms for other UR models do not authorize
  importing those models into this branch.
- Camera housing/bracket are original primitive approximations (60 g camera,
  40 g bracket), not imported manufacturer CAD or validated mounting hardware.
  [Robotiq](https://robotiq.com/products/adaptive-grippers) and
  [D405](https://www.realsenseai.com/product-family/d405-series/) are hardware
  reference choices; vendor ratings do not qualify this simulation.

## Staged evidence

All paths below are ignored local artifacts under `runs/`. CI uploads equivalent
qualification artifacts; hosted CI must pass before merging.

| Stage | Result | Local evidence |
|---|---|---|
| Main watchdog regression | Reproduced 5.214 rad/s² spike; bounded hold damping 30/6 fixes transition without changing stiffness/force limits | `gripper-camera-baseline/`, first branch commit |
| Assembly and contact | Eight assembly tests; real loaded image bounds, disabled/stale/future/frame holds; reproducible assembly | `2f85-angle-local-qualified.log`, ordinary pytest |
| Seeded local task | 20/20 seeds succeed through pick, place and release | `2f85-angled-expert.json` |
| Independent kinematics | Pinocchio agrees with MuJoCo across aperture and contact-loaded joint states, under 1e-9; rigid controller model has six DOFs | `2f85-angled-observability-tests.log` |
| Native moving controller | All 39 bounded cases complete, including ownership, faults, reset and loaded hold/recovery | `2f85-angled-motion/report.json` |
| Generated messages / DDS | Required generated RGB, calibration and sample serialization, separate-process delivery and independent renderer-loss diagnosis pass | `2f85-camera-health-dds.log` |
| Clock and camera faults | Eleven ROS-free injected-clock cases cover exact 100 ms, pause, reset, replay and rejected sources | ordinary pytest |
| Viewer / restrictions | Four-case smoke completes; image topics deliver; tool/base assets allowed and traversal/client commands/services/parameters denied | `2f85-angled-observability/` |
| Actual Foxglove | Assembled articulated robot and calibrated wrist image viewed together; held cube fully in image | `2f85-foxglove-angled-qualified.png`, `2f85-foxglove-live-qualified.png` |
| Acquisition and conversion | Two successful final-model MCAP episodes; both quality-selected; 1,101 frames, three RGB features | `2f85-final-selection.json`, `2f85-final-dataset/` |
| CPU ACT | Tiny two-update checkpoint and resume to three; real three-camera inference, old calibration rejected, missing wrist disabled, no ROS imports | `2f85-final-training.log`, `2f85-final-policy-smoke.log` |
| Required dev checks | Build, Ruff, mypy (67 files), pytest (303 passed, four optional skips), doctor pass | `2f85-final-*.log` |
| Existing isolated jobs | ROS contracts: three pass; CRISP bench: five pass; moving/observer suites: 18 pass in observer runtime; CPU training focused suite: ten pass | `2f85-contract-required.log`, `2f85-crisp-required.log`, `2f85-final-training-tests.log` |
| Default workflows | ROS-free default simulation image builds and control smoke passes | `2f85-default-sim-build.log`, `2f85-default-sim-smoke.json` |

### Controller findings

The tool preset uses 1000 N/m translational stiffness and a 0.08 m/s command
cap. At 500 N/m the short-horizon target produced approximately the supporting
force of the 80 g cube and lift stalled. Loaded reference closure uses 0.1
normalized aperture/s; faster contact inputs can trigger the existing guard.
Opening requires 0.05 normalized tolerance. These settings are recorded
explicitly; torque, slew, acceleration and freshness limits were not relaxed.

Final qualification: cube lift **78.632 mm**, two-second loss hold drift
**0.179 mm**. Peak joint acceleration **2.820 rad/s²** (ceiling 4), Cartesian
acceleration **0.540 m/s²** (ceiling 1), joint speed **0.153 rad/s**. Maximum
observed task effort **1.227 Nm** (test ceiling ±5 Nm). Repeatability is within
2 mm tolerance, not bitwise identity: independent steady watchdogs may inhibit
motion during host scheduling stalls.

## Safety review and limits

The dedicated [safety matrix](../contracts/tooling.md#dedicated-simulation-safety-review)
requires command loss holds, original timestamp preservation, reset isolation,
renderer independence, exact camera expiry and gripper-preserving policy faults.
The renderer has no command authority and never steps physics. The independent
observer is the sole camera health publisher, including when the renderer dies.
A final freshness check follows image serialization immediately before DDS.

Software RGB projection targets 25 Hz but is not a rate guarantee. The concurrent
long run delivered only one valid frame while other render-heavy jobs competed;
1,283 expired frames were discarded. The short final-mount smoke delivered 16
frames (2.59 Hz including startup/teardown), averaging 84 ms successful rendering
and publishing. A further bounded loaded demo, with fewer competing jobs,
delivered 189 frames and discarded 644 over 75.77 s (2.49 Hz overall, 82.7 ms
mean successful render/publish). Its actual loaded grasp and fault sequence
completed. These results are in `2f85-camera-delivery/`. Motion checks still completed. Stale pixels may remain displayed
by Foxglove; inspect independent health and original capture identity. This
limitation must not be disguised by extending the 100 ms expiry.

The Image and 3D component configurations were tested in the actual Foxglove app.
Whole-layout file import was not completed: its native file chooser was not
available to the browser automation. The checked-in full layout is a manual
import handoff; no claim is made that its complete import flow passed.

The training smoke proves data plumbing, checkpoint compatibility and execution,
not useful learned task success. No full new-model policy training/evaluation
campaign, hardware, GPU or interactive physical camera validation ran. MCAP
acquisition and training remain local and ROS-free. Depth/stereo/RealSense SDK,
ROS recording, hardware drivers/mount/stop/watchdog qualification, Gazebo and
RViz are deferred.

One attempted combined ROS test invocation used the observer image for the CRISP
bench and contract runner, and failed their explicit missing-runtime checks.
Those suites subsequently passed in their designated Compose services. Missing
ROS dependencies are not treated as skips.
