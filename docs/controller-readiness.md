# Controller readiness check — 2026-09-12

This separates automated simulation evidence from operator-reported manual
trials. The webcam usability change includes the controller changes in ADR 009.

## Findings and changes

- An isolated `main` snapshot passed 18/20 scripted seeds. The earlier webcam
  working tree had regressed that baseline; restored measured feedback with
  a 100 ms correction horizon and velocity gain 600 restores execution.
- Replacing a loaded gripper's target with measured aperture removes squeeze.
  Holds now preserve only the last accepted rate-limited aperture.
- A one-second pause after lifting still produced about 19 mm of slip at gripper
  gain 200. Gain 800 reduces this to about 5 mm without changing the existing
  20 N force limit, geometry, mass, or friction. Pause/resume is regression-tested
  for disabled and stale input, including an untrusted open payload.
- Webcam re-engagement preserves pinch intent and re-anchors the arm to measured
  pose. Keyboard velocity targets account for the controller response horizon.

## Reproduction and results

```bash
docker compose build dev
docker compose --profile ui build keyboard-teleop
docker compose run --rm dev ruff check .
docker compose run --rm dev mypy src
docker compose run --rm dev pytest -q
docker compose run --rm dev act-lab doctor
docker compose run --rm dev act-lab sim control-diagnostic --json
docker compose run --rm dev act-lab sim expert --seed-start 0 --episodes 20 --min-success-rate 0.9 --json
```

Both builds, lint, typing, doctor, and the full test suite passed. The suite
includes the 5,000-step seeded safety rollout, recorded MediaPipe fixture,
keyboard release under real servo dynamics, and synthetic-versus-camera velocity
trajectory comparison. Scripted seeds 0–19: 20/20 successful.

The reachable −100 mm X diagnostic travels 99.54 mm in 1.2 simulated seconds,
with 0.49 mm final position error. Peak commanded speed is 0.125 m/s; peak
measured control-cycle speed is 0.121 m/s. The old +100 mm X endpoint is not
IK-reachable from home and produced cross-axis drift: it is now rejected as a
diagnostic endpoint, rather than counted as successful merely for X progress.

## Operator-reported manual trials and remaining limitations

The operator reported successful keyboard pick/place on all five seeds (0–4).
After a shutdown fix, Shift+Q also exited the process and removed the container.
Live webcam pick/place and clutch pause/resume succeeded. The operator closed
the webcam testing round while noting intermittent frustration; per-seed timing,
latency percentiles, and a complete scored webcam result were not collected.
Do not report the quantitative webcam acceptance thresholds as passed.

Proceed to PR 6 recording with webcam ergonomics and a measured usability
benchmark explicitly deferred. Record future failures and discarded attempts,
not only successful episodes. These seeds are tuning/acceptance seeds, not
held-out learned-policy evaluation. MCAP recording and validated conversion
remain prerequisites for ACT training. No physical robot validation was run.
