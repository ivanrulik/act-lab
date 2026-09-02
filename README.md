# ACT Lab

ACT Lab is a container-first learning workspace for building an end-to-end
robot imitation-learning system:

```text
teleoperate -> record MCAP -> validate/convert -> train ACT -> evaluate
```

The target platform is a simulated UR5e with a parallel-jaw gripper in MuJoCo.
Webcam hand tracking is the initial teleoperation device. LeRobotDataset is the
training representation and LeRobot's ACT implementation is the initial policy.

This repository currently contains the **project scaffold and engineering
contracts**. Milestones are deliberately implemented in small pull requests;
see [the roadmap](docs/ROADMAP.md).

## Quick start

Requirements: Docker Engine with Docker Compose v2.

```bash
docker compose build dev
docker compose run --rm dev act-lab doctor
docker compose run --rm dev pytest
docker compose run --rm sim
docker compose run --rm expert
```

Convenience wrappers open the interactive MuJoCo viewer through the host's
X11/XWayland display and stop it from any working directory:

```bash
./scripts/start-sim.sh
./scripts/stop-sim.sh
```

The window stays open until you close it or run the stop script. The separate
`sim` service runs a finite headless 50-step safe Cartesian-controller smoke
rollout for CI. The UI
uses Mesa software rendering and grants the container read-only access only to
the host X11 socket; it does not require a privileged container or GPU access.

Keyboard teleoperation uses the same narrow X11 forwarding and safety path:

```bash
docker compose --profile ui run --rm keyboard-teleop
```

Tap `W/S` for world X, `A/D` for world Y, `R/F` for world Z, and `O/C` to
open/close. Space stops and `Q` quits. Letter bindings are case-insensitive.
MuJoCo's supported passive-viewer callback reports presses rather than held-key
state, so holding a key does not provide continuous motion. Each press adds one
configured target nudge; the safety controller may execute only part of that
nudge before input becomes stale at 100 ms. The overlay distinguishes accepted
events, target pose, actual pose, and the latest safety outcome. Tap repeatedly
to jog; continuous hold-to-move control is intentionally deferred.

MuJoCo physics is CPU-based in this service. GPU rendering would change viewer
rendering performance, not keyboard command cadence or controller response.

The deterministic privileged baseline is headless and reports per-seed results:

```bash
docker compose run --rm expert
docker compose run --rm dev act-lab sim expert \
  --seed-start 0 --episodes 20 --min-success-rate 0.90 --json
```

For a local, non-container fallback:

```bash
python -m venv .venv
. .venv/bin/activate
python -m pip install -e '.[dev]'
act-lab doctor
pytest
```

The Docker workflow is authoritative. Native installation exists for editor
tooling and diagnosis only.

## Project status

- [x] Repository contracts, architecture records, container scaffold, and CI
- [x] MuJoCo UR5e environment and task
- [x] Cartesian controller and safety envelope
- [x] Keyboard teleoperation and deterministic scripted expert
- [ ] Webcam hand teleoperation
- [ ] MCAP recording, validation, replay, and conversion
- [ ] ACT training notebook and CLI
- [ ] Seeded closed-loop evaluation
- [ ] Optional ROS 2 adapters and physical robot integration

## Data policy

Large recordings, datasets, checkpoints, and generated reports are never
committed to Git. They belong under ignored `data/` and `runs/` directories or
in an artifact store. Small, reviewed test fixtures may live in
`tests/fixtures/`.

## Documentation map

- [Agent operating contract](AGENTS.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Roadmap and PR boundaries](docs/ROADMAP.md)
- [Definition of done](docs/DEFINITION_OF_DONE.md)
- [Data contract](docs/contracts/data.md)
- [Control and safety contract](docs/contracts/control.md)
- [Simulation contract](docs/contracts/simulation.md)
- [Experiment contract](docs/contracts/experiments.md)
- [ADRs](docs/adr/README.md)
- [Contributing](CONTRIBUTING.md)
