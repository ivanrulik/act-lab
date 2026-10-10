# Articulated gripper and wrist camera — implementation plan

Status: **Implemented and locally qualified; draft PR and hosted CI pending.**
Branch: `feature/gripper-wrist-camera`.
Inspected base: merged main `25c6e3220e6f4a7742a5cdb8fc83e91fe022a051`.
Sequence: an intermediate upgrade after ROS observability, before PR 14 recording.

## 1. Result users should see

An explicit simulation preset runs a UR5e with an articulated Robotiq 2F-85
gripper and a wrist-mounted, D405-inspired RGB camera. Users can open and close
the fingers, grasp the existing cube through contact, lift it, hold it during a
command fault, and release it. Foxglove displays the assembled robot and a live
wrist image alongside the existing safety telemetry.

The same preset works locally without ROS. Wrist images can enter the existing
MCAP acquisition, quality validation, LeRobot conversion and ACT pipeline.
Deliver both an assembled URDF/Xacro and matching MuJoCo model; a visualization
mesh alone is insufficient.

This establishes hardware-compatible component names, mounting frames, units and
interfaces. It does not establish a physically validated digital twin, camera
driver, hardware mounting design or safe hardware operation.

## 2. Scope and proposed defaults

- Use **Robotiq 2F-85** geometry and articulated linkage, with measured aperture.
- Use a **RealSense D405-inspired wrist camera** with explicit mounting and
  optical frames. Synthetic RGB is required; depth, stereo, noise and the
  RealSense SDK are deferred.
- Add `configs/sim/ur5e_2f85_d405.toml` as an explicit preset. Keep the existing
  educational model as the legacy/default preset for this branch.
- Keep `overview` and `policy`; add camera ID `wrist`. The first synthetic
  acquisition profile is RGB8, 320 × 240, at the existing 50 Hz sample grid.
  The Foxglove projection targets 25 Hz and may drop frames.
- Define synthetic intrinsics explicitly for this image aspect ratio. Do not
  copy nominal D405 field-of-view values onto an incompatible aspect ratio.
  The synthetic 50 Hz profile is not a claim about physical D405 RGB modes.
- Foxglove remains the bundled viewer. Gazebo, RViz, ROS recording, physical
  drivers and a full policy retraining campaign remain later work.

### Sources and asset strategy

Use the inspected MuJoCo Menagerie revision
`0059d4335f8156206f63a35662313385f7ad6d74`, specifically
[robotiq_2f85](https://github.com/google-deepmind/mujoco_menagerie/tree/0059d4335f8156206f63a35662313385f7ad6d74/robotiq_2f85).
It is a simplified model, including tuned contact parameters, not a manufacturer
validation. Retain its BSD-2-Clause notices and document every local change.

Inspect the corresponding ROS description assets at
[ros-industrial/robotiq](https://github.com/ros-industrial/robotiq/tree/45196f6558fe8ba9d89bc8a105396c68c3e7e892/robotiq_2f_85_gripper_visualization)
revision `45196f6558fe8ba9d89bc8a105396c68c3e7e892`. Audit asset-level licenses
and linkage conventions before import; do not install its ROS 1 drivers.
Retain official UR description revision
`6b639c2efd9f4f12da858a72cf1a9e40da365ed8`.

Use [Robotiq specifications](https://robotiq.com/products/adaptive-grippers) and
[D405 specifications](https://www.realsenseai.com/product-family/d405-series/)
to document nominal dimensions, mass, stroke and working range. Use original
primitive geometry for the camera housing and mounting bracket unless a
redistributable CAD asset is verified. Record source, revision, license, hashes,
approximations and modifications in an asset manifest. Vendor force and payload
ratings are not validation of simulated grasp forces or robot payload capacity.

## 3. Model, tool and gripper contracts

Add an explicit model binding rather than another hardcoded scene path. It must
resolve the MJCF, URDF, named joints/actuators/sites, tool transform, camera
profile and model/calibration hashes from the selected preset.

Keep six canonical UR5e arm joints. CRISP owns only those six effort interfaces;
the existing gripper intent follows a separate gripper binding. Keep normalized
intent **0 = closed, 1 = open**, but compute measured aperture from finger-pad
separation. Do not reuse the current 25 mm slide-joint conversion for hinge joints.
Record commanded aperture, measured aperture and actual articulated joint state.
Check the command mapping throughout the stroke and under contact deflection.

Define `tool0` as the existing flange frame, then explicit gripper mount,
grasp TCP, camera housing and camera optical frames. Optical axes follow ROS
conventions: +Z forward, +X right, +Y down. Domain poses retain metres, WXYZ and
the existing `world` convention. The camera mount must leave a useful view of
the grasp region through opening, closure and lift.

The current CRISP mapping includes a fixed 0.11 m tool offset. Replace that
assumption with the selected preset's transform and qualify it against measured
FK. Reconcile rigid tool mass/inertia and the controller's dynamics model,
including the effect of moving fingers; document any remaining approximation.
Preserve the accepted compensation convention and prohibit duplicate gravity
compensation. A visual URDF and a dynamically different control model cannot be
treated as equivalent without evidence.

Add ADR 019 covering these choices. It partially supersedes ADR 016's fixed tool
offset and ADR 009's educational gripper binding for the new preset. Existing
safety, stale-command, accepted-aperture hold and controller ownership decisions
remain applicable. Requalify reachability, collision exclusions and loaded holds
without raising existing safety limits to obtain a passing result.

For URDF linkage, use mimic relationships only where they are exact. Where a
tree approximation cannot represent the physical closed linkage, publish the
actual measured passive joints and document the approximation. Compare visual
link poses to MuJoCo through the full stroke, including contact-loaded poses.

## 4. Camera acquisition and process isolation

For local acquisition, render each canonical sample from its observed simulator
state and retain the original simulation timestamp. Preserve the existing
post-command synchronization contract and record intrinsics, extrinsics,
resolution, model identity and calibration identity in provenance.

For the moving ROS runtime, add a separate scratch-model renderer. The physics
owner copies bounded primitive state, including all required arm, finger and
object coordinates; it never performs rendering, waits for images or serializes
ROS messages. The renderer updates a scratch model without stepping physics.

Use a separate bounded handoff for camera state/images. A 320 × 240 RGB frame
is 230,400 bytes and must not enter the existing 64 KiB telemetry slot.
Publish only the latest complete frame; expose drops, capture identity, source
age and receipt age. Each internal frame carries episode UUID, capture sequence,
physics tick and model/calibration hashes. Repeated publication cannot refresh
its capture time. Reset clears pending frames and rejects delayed old-episode
results.

Viewer camera loss marks the image stale at 100 ms using steady time; it cannot
renew motion authority. A policy configured to require `wrist` must refuse
missing, stale or mismatched frames through the shared disabled/hold path.
Viewer rendering and policy acquisition are separate consumers; a slow viewer
does not determine which images enter a recorded training sample.

## 5. ROS and Foxglove projections

Keep v1 command, RobotState and CommandReport definitions unchanged. RobotState
continues to carry six arm joints. Standard `/joint_states` additionally carries
measured gripper joints for the assembled visualization model.

| Topic | Message | Proposed QoS / purpose |
|---|---|---|
| `/act_lab/view/wrist/image_raw` | sensor_msgs/Image | RGB8, best effort, volatile, depth 1 |
| `/act_lab/view/wrist/camera_info` | sensor_msgs/CameraInfo | Paired stamp/frame/calibration; best effort, volatile, depth 1 |
| `/act_lab/view/wrist/sample` | Additive CameraSample | Image and CameraInfo with episode, capture sequence, tick and hashes; best effort, volatile, depth 1 |
| `/act_lab/view/health` | Existing DiagnosticArray | Camera freshness, drops and failures alongside existing observer health |

The typed sample prevents future recording from joining repeated timestamps
across reset episodes ambiguously. Standard Image/CameraInfo remain projections
for independent viewers. Keep typed primitive codecs ROS-free and preserve the
existing +1 second domain-to-ROS mapping. Document how viewer caches behave on
backward reset; cached pixels are not proof of a live feed.

Extend the saved Foxglove layout with the wrist image, measured aperture and
camera health. Extend bridge asset access only to the installed gripper/camera
assets and continue testing denied filesystem access. Viewer command publishing,
services and parameter mutation stay disabled.

Use the existing optional ROS image and Compose profile where possible. Add an
explicit preset argument to the demonstrator and document one Compose command
for the new assembly. Add packages only with dependency rationale; synthetic
RGB requires no hardware camera SDK. Default simulation/training stay ROS-free.

## 6. Data and checkpoint compatibility

Reuse the RGB recording schema and existing provenance fields; calibration and
model identities belong in resolved configuration/calibration metadata. Extend
quality validation to require the new preset's cameras, image dimensions and
consistent calibration. Conversion produces `observation.images.wrist` and
retains deterministic episode-level splits and source lineage.

New training runs/checkpoints record tool/model identity, camera IDs, shapes and
calibration fingerprints. Evaluate compatibility before enabled inference.
Reject missing wrist input and incompatible model/calibration identities with
an actionable error. Legacy checkpoints lacking these identities remain usable
only through the explicit legacy setup; they cannot silently run the new model.
Preserve existing raw logs and datasets rather than rewriting their identity.

## 7. Implementation and testing stages

Complete each gate before starting the next stage. Store raw traces, timing,
images, videos and generated datasets under ignored `runs/`.

### Stage 0 — Restore and freeze the baseline

Main's post-merge CI run
[38058439723](https://github.com/ivanrulik/act-lab/actions/runs/38058439723)
failed the ROS simulation smoke check with measured maximum joint acceleration
**5.214131925418197 rad/s²**; its ten motion tests passed. Other jobs passed.
This is an unresolved baseline issue, not evidence caused by the proposed tool.

Inspect its traces, reproduce the failing transition and distinguish dynamics,
timing and measurement causes. Resolve it as a focused baseline fix before
proceeding with feature model changes. The user requested immediate branch
creation, so this prerequisite is isolated in the first feature-branch commit.
Preserve acceleration
limits. Freeze same-seed legacy trajectories, controller configuration, existing
camera/model identities and successful checks as comparison evidence.

**Gate:** main CI green, failure explained and covered by a targeted regression
check; feature base SHA recorded. A rerun alone does not explain the failure.

### Stage 1 — Assemble models and verify contracts

Import audited assets; implement preset/model bindings, gripper mapping,
mounting/TCP transforms, camera frames and ADR 019. Generate the assembled URDF.

Tests: MJCF loads headlessly; URDF parses with a connected tree and valid joint
names; mass/inertia and actuator ownership are explicit; aperture mapping is
monotonic over sampled opening fractions; measured articulated FK agrees with
the documented tree approximation. Check TCP/camera transforms across home,
nearby and wrist-singular configurations. Re-run the accepted controller FK
bounds of 2 mm / 0.02 rad with the new transform. Check camera optical axes.

**Gate:** models and transforms agree within declared tolerances, no undeclared
collision exclusions, unchanged six-arm control interfaces, legacy preset passes.

### Stage 2 — Prove physical grasp and safe holds

Exercise empty closure, cube contact, grasp, 50 mm lift, two-second simulation
hold and release. Use real contact dynamics; no attachment constraint or scripted
cube motion. Measure aperture, contact forces, slip, arm motion and effort.
Tune only documented model/controller parameters within existing safety limits.

Tests: fixed seeds; symmetric empty aperture; object blocks closure; release
separates contacts; loaded hold slip no more than 5 mm during the two-second
fixture; stale/disabled/invalid commands retain the last accepted gripper target.
Test exact 100 ms, future commands, loss, pause, reset and fresh recovery on the
actual shared safety path. Re-run CRISP dynamics/hold qualification with the tool.

**Gate:** all mechanical/fault fixtures pass. Run the expert pick/place task on
seeds 0–19 with at least 18 successes, no safety-bound violation and individual
failure reports. This proposed task threshold is for review; it is not a policy
performance claim. Report confidence intervals and compare the legacy preset.

### Stage 3 — Validate wrist RGB and rendering isolation

Add the camera and provenance/calibration profile; implement local synchronized
capture and the separate ROS-runtime renderer.

Tests: RGB dimensions/encoding; known-point projection against K and extrinsics;
optical-axis orientation; useful cube visibility before/during grasp; expected
finger occlusion; matching sample/state identity. Same-seed captures repeat on
the same pinned rendering environment. Kill/slow the renderer, fill its handoff,
pause/reset time and deliver an old-episode result. Verify exact stale boundaries
with injected clocks and unchanged motion authorization. Compare physics traces
with rendering enabled and disabled; never claim cross-GPU pixel equality.

**Gate:** calibrated images tied to coherent states, bounded drops/latency
reported, no renderer dependency in physics or watchdog decisions.

### Stage 4 — Verify DDS and Foxglove

Extend generated interfaces/codecs, assembled robot publication, bridge assets
and layout. Test actual generated-message serialization and separate-process DDS
delivery, paired image/info identity, finger animation, pause/reset, publisher
loss and camera-health warnings. Missing ROS packages fail these tests.
Use bounded discovery waits and injected clocks for exact watchdog assertions.

**Gate:** automated DDS/bridge restrictions pass. Manually inspect the live
assembled robot and wrist feed in the user's Foxglove session, including grasp,
fault, reset and recovery. Record manual acceptance separately; do not claim it
from screenshots or headless tests alone.

### Stage 5 — Prove the local learning path

Record small seeded episodes using the new preset. Inspect and validate MCAP;
test rejection of missing/malformed wrist frames and calibration drift. Convert
twice and compare logical fingerprints, shapes and episode splits. Run a tiny
CPU ACT train/checkpoint/reload/inference smoke with wrist input. Exercise
incompatible-checkpoint rejection and stale/missing-input safe hold.

**Gate:** actual RGB bytes and provenance survive the complete pipeline; new
checkpoints resolve the right model/cameras; legacy CPU training still passes.
The smoke proves integration, not useful learned grasp performance. Larger
matched held-out ACT evaluation is a later experiment with a fresh dataset.

### Stage 6 — Full regression, documentation and delivery

Run the repository-required Compose checks:

```bash
docker compose build dev
docker compose run --rm dev ruff check .
docker compose run --rm dev mypy src
docker compose run --rm dev pytest
docker compose run --rm dev act-lab doctor
```

Also run existing CPU training, ROS contracts, CRISP feasibility, moving ROS and
observability jobs, plus the new model/camera integration tests. Build/run default
simulation and learning images without ROS. Keep hosted CI timing descriptive
where shared-runner scheduling makes latency thresholds unreliable; deterministic
safety and numerical bounds remain required gates.

Update simulation/control/data/ROS contracts, architecture, roadmap, ADR index,
dependency rationale, asset notices and README demo instructions. Commit a concise
report with successes, failures, safety matrix and known fidelity limits. Open
one draft feature PR with validation evidence. Review before merge; hardware and
ROS durable recording stay deferred.

## 8. Dedicated safety review matrix

| Scenario | Required behavior / evidence |
|---|---|
| Startup or mismatched model | Reject unresolved bindings/calibration before enabled control |
| Bad gripper intent | Existing numeric/range/rate checks; last accepted aperture retained |
| Contact deflection | Measured aperture can differ from target; hold retains accepted target |
| Command loss / 100 ms / future stamp | Existing independent inhibition and qualified loaded hold |
| Clock pause | Steady watchdog expires; cached image cannot refresh command or input age |
| Backward reset | New episode; clear camera/intent caches; reject late old-episode frames |
| Renderer/bridge crash or stalled consumer | Physics handoff stays nonblocking; health stale; no authority renewal |
| Required policy image missing/stale | Shared disabled/hold path; fresh input required for recovery |
| Invalid frame / model / calibration | Reject before motion; no silent legacy-checkpoint reuse |
| Viewer write or forbidden asset access | Protocol requests denied by existing restrictions |
| Fresh recovery | Existing controller recovery and new command sequence; new current frames |
| Shutdown with a grasp | Existing qualified dynamic hold and bounded child shutdown; no hardware claim |

## 9. Review decisions

1. Approve Robotiq 2F-85 + D405-inspired **RGB first**, with depth deferred.
2. Approve an explicit new preset while retaining the legacy default this PR.
3. Approve the proposed contact-hold and 20-seed expert gates, plus a small CPU
   learning integration test rather than a full retraining campaign.
4. Treat the current main CI acceleration failure as a prerequisite fix, then
   implement this feature in the six subsequent stages.

## Implementation evidence

See the [qualification report](../reports/gripper-wrist-camera.md) for staged
results, controller changes, viewer checks and remaining limits.
