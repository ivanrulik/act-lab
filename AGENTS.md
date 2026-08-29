# AGENTS.md

## Mission

Build a reproducible educational robotics platform that demonstrates the full
ACT imitation-learning lifecycle on a simulated UR5e. Optimize for clear
interfaces, inspectable data, safe behavior, and repeatable experiments.

## Read before changing code

1. `README.md`
2. `docs/ARCHITECTURE.md`
3. `docs/ROADMAP.md`
4. The relevant file in `docs/contracts/`
5. Existing decisions in `docs/adr/`

If a proposed change contradicts an accepted ADR, add a superseding ADR in the
same PR. Do not silently change the architecture.

## Non-negotiable boundaries

- Docker Compose is the authoritative execution interface.
- Domain models in `act_lab.domain` must not import MuJoCo, ROS, MediaPipe,
  MCAP, or LeRobot types.
- Simulation, devices, storage formats, and learning frameworks are adapters
  around the domain contracts.
- MCAP is the raw acquisition record; LeRobotDataset is the derived training
  representation. Never train directly from an unvalidated raw log.
- Training and evaluation must not require ROS 2.
- ROS 2, when added, is an optional transport/hardware adapter.
- Interactive commands must fail safe on stale commands or lost tracking.
- Never silently fall back from requested GPU execution to CPU.
- Never commit raw webcam data, large datasets, checkpoints, credentials, or
  machine-specific paths.
- Do not introduce a dependency without documenting why it is needed.

## Pull request discipline

- Work within one roadmap PR boundary unless the user explicitly expands scope.
- Keep main runnable after every PR.
- Include tests and documentation with behavior changes.
- Add or update an ADR for consequential architectural choices.
- Record deferred work as roadmap items; do not hide it in broad TODO comments.
- Do not claim hardware, GPU, or interactive-camera validation unless it ran.

## Required checks

Run the closest available equivalent of:

```bash
docker compose build dev
docker compose run --rm dev ruff check .
docker compose run --rm dev mypy src
docker compose run --rm dev pytest
docker compose run --rm dev act-lab doctor
```

During the initial scaffold, native commands are acceptable when Docker is not
available. State exactly which checks ran and which did not.

## Repository layout

- `src/act_lab/domain/`: dependency-free contracts and data models
- `src/act_lab/adapters/`: simulator, device, storage, and framework adapters
- `src/act_lab/application/`: use cases that coordinate domain ports
- `configs/`: versioned runtime and experiment configuration
- `proto/`: versioned raw-recording schemas
- `notebooks/`: thin educational clients of production APIs
- `docs/contracts/`: stable behavioral/data expectations
- `docs/adr/`: architectural decisions and their consequences
- `tests/`: unit, integration, and deliberately small fixtures

## Change-specific guidance

- Simulation: fixed seeds, explicit clocks, and headless tests are mandatory.
- Recording: use monotonic timestamps, schema versions, atomic finalization, and
  provenance metadata.
- Conversion: deterministic output, episode-level splits, and quality reports.
- Training: configuration-driven CLI is authoritative; notebooks contain no
  unique pipeline logic.
- Evaluation: fixed held-out seeds, failure reporting, artifacts, and confidence
  intervals. A rollout video alone is not evaluation.
- ROS/hardware: require a dedicated safety review and preserve local adapters.

