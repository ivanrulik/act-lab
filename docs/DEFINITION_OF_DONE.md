# Definition of done

A change is done when all applicable statements are true.

## Correctness

- Acceptance criteria are encoded in tests where practical.
- Time, units, coordinate frames, shapes, and valid ranges are explicit.
- Deterministic components accept and record a seed.
- Failure behavior is tested, not only the happy path.

## Architecture

- Dependency direction in `docs/ARCHITECTURE.md` is preserved.
- External framework types stop at adapter boundaries.
- Public contract changes update documentation and compatibility strategy.
- Consequential decisions add or supersede an ADR.

## Reproducibility

- Docker remains the authoritative workflow.
- New dependencies are pinned by the project lock strategy.
- Generated artifacts capture code, configuration, data, and environment IDs.
- Commands in documentation have been exercised or clearly marked illustrative.

## Safety and privacy

- Motion changes identify stale-input and disabled-input behavior.
- Hardware access is least-privilege and opt-in.
- Webcam-derived artifacts have documented retention/privacy behavior.
- Secrets, personal recordings, datasets, and checkpoints are not committed.

## Reviewability

- The PR stays within its declared roadmap boundary.
- Tests, typing, linting, and the doctor command pass.
- The PR describes verification and untested environment-specific behavior.
- Deferred work is explicit.

