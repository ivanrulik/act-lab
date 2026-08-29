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
- [ ] MuJoCo UR5e environment and task
- [ ] Cartesian controller and safety envelope
- [ ] Keyboard and webcam teleoperation
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
- [Experiment contract](docs/contracts/experiments.md)
- [ADRs](docs/adr/README.md)
- [Contributing](CONTRIBUTING.md)

