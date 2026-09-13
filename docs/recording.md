# Recording episodes

Recording is opt-in. It retains simulator RGB images, observations, requested
and executed intent, safety reports, task labels, provenance and webcam
diagnostic/calibration snapshots. It never retains operator webcam pixels or
privileged cube/goal coordinates.

```bash
docker compose build dev
docker compose run --rm dev act-lab sim expert --episodes 1 --record-dir data/raw --json
docker compose --profile ui build keyboard-teleop
docker compose --profile ui run --rm keyboard-teleop act-lab sim keyboard-teleop --seed 0 --record-dir data/raw --operator learner
```

For webcam use, append `--record-dir data/raw --operator learner` to the
`act-lab sim webcam-teleop` command in the documented `teleop` service.
First run `docker compose --profile teleop build teleop`: Compose tags this
service separately, so rebuilding `dev` alone does not update an existing
webcam service image.
Recording starts at reset, including calibration. `Q` or closing the viewer
stops and finalizes the attempt. Restart the command for another attempt.
Each expert seed produces a separate file.

While recording, press **F6** to stop with a success label, **F7** for failure,
or **F8** to discard. The viewer shows recording status. These requests finalize
after the next control sample (or on session exit), with reason
`operator_viewer_label`; they take precedence over CLI labels. The viewer remains
usable afterward but no further samples are recorded. Restart for a new attempt.
An enlarged bottom-left banner changes from `RECORDING ACTIVE` to
`EPISODE SAVED: SUCCESS`, `FAILURE`, or `DISCARDED`, and explicitly says that
further movement is not recorded. A discarded episode is still saved as evidence.
Keyboard sessions also capture these function keys through the focus-gated X11
listener. Outcome requests are logged before finalization for live diagnosis.
F9 is reserved by the operator desktop's global push-to-talk dictation binding;
it is intentionally not a recording shortcut.

Default outcome uses the simulator result at session exit. Override it with
`--outcome success|failure|discarded --reason 'explanation'`; explicit labels
require a reason. Discard retains all data, with a rejection label for future
episode selection. No label implies training eligibility. Store recordings in
ignored `data/`; never commit or publish raw recordings without review.

```bash
docker compose run --rm dev act-lab recording inspect data/raw/EPISODE.mcap
docker compose run --rm dev act-lab recording recover data/raw/EPISODE.mcap.partial
```

Inspection emits JSON with schemas, counts, timestamps, observed rates,
provenance and lifecycle events. `training_validated` is always false. This is
structural inspection, not PR 7 quality validation.
An incomplete file reports outcome `unknown`, even if task success or a stop
event survived. Only a complete file's stop/discard/recovery event defines its
episode outcome; `last_task_outcome` separately reports the last task result.

Recovery preserves the original, refuses active writers/existing destinations,
and publishes `EPISODE.recovered.mcap` with an interrupted label and source hash.
It tolerates truncated tails, not corrupt chunks, and cannot restore unwritten
bytes. Failed recovery output remains as evidence; retain it and use a separate
copy of the original in another directory for retry.

State, commands and scene images use the fixed-step simulation clock at the
configured control rate (default 50 Hz). Original action timestamps preserve
command age. Host monotonic diagnostic time and wall start time are separate
metadata; they do not imply real-time execution.

Provenance includes resolved settings, seed, robot/task/camera identifiers,
calibration snapshots, application/schema versions and Git revision/dirty state.
Where Git is unavailable, `ACT_LAB_GIT_REVISION` can supply the revision; dirty
state remains explicitly unknown. `ACT_LAB_CONTAINER_IDENTITY` accepts a resolved
image digest when available, otherwise identity is explicitly unknown. Use an
operator pseudonym. Rebuild the Docker image after changing Protobuf schemas.
New webcam episodes also record the tracking-model SHA-256, live-camera versus
recorded-video input kind, camera identifier or video SHA-256, and resolved
headless/calibration/step-limit options. Model/video paths and video content are
not embedded. Earlier PR 6 test recordings remain unchanged and lack this metadata.

Synchronous rendering and durable writes can slow interactive control. Storage
failure applies the existing grasp-preserving disabled hold and ends acquisition.
Interactive recording performance requires operator validation; no physical
robot behavior is claimed. See ADR 010 for durability details.

## PR 6 local validation

The Docker dev and UI builds, Ruff, mypy, doctor and 129 tests passed locally.
The unrecorded scripted benchmark passed 20/20 seeds. Seed 0 also completed
successfully with full scene/state MCAP recording enabled.
The suite includes independent decoding from embedded schemas, synchronized
scene/state rates, stale action timestamps, failed/discarded retention, abrupt
process exit, truncated tails, corrupt-chunk rejection, active-writer locking,
disk-failure holds, synthetic webcam calibration/diagnostics, and queued viewer
outcome controls. It also runs the existing controller and recorded MediaPipe
regressions. No operator webcam recording is used or committed by these tests.
Review regressions also cover incomplete files with task-success/stop events,
model/video content hashes, and a changed model's acquisition identity.

```bash
docker compose build dev
docker compose --profile ui build keyboard-teleop
docker compose run --rm dev ruff check .
docker compose run --rm dev mypy src
docker compose run --rm dev pytest -q
docker compose run --rm dev act-lab doctor
```

Manual keyboard seed-0 pick/place produced a finalized success recording with
743 synchronized state/command/image samples. An initial F9 label was missed
because the desktop consumes F9 for dictation. After switching to F6/F7/F8, a
physical F6 press produced `operator_viewer_label` at 3.40 simulated seconds.
The operator found the status change unclear, prompting the enlarged banner.
Live MuJoCo viewer tests with injected X11 F7/F8 events subsequently finalized
failure/discarded files with `operator_viewer_label`; discarded data was retained.
The subsequent physical-webcam session was reported by the operator as working
correctly, including the banner. Its MCAP finalized with the operator's failure
label (`operator_viewer_label`), 394 samples each for state, command, both scene
cameras and tracking diagnostics, and a nonempty calibration snapshot. Timestamps
were monotonic at 50 Hz simulation time, and Q exited with status 0. This confirms
physical F7 operation; physical F8 remains untested (injected X11 F8 passed).
This was a functional recording test, not a successful pick/place benchmark or
quantitative real-time/latency acceptance. Hardware and GPU execution remain
unvalidated. Operator webcam pixels were not retained.
Remote CI is not claimed to have passed. GitHub PR #9's previous operator evidence
remains documented separately in `controller-readiness.md`.
