# Paired-camera ACT pilot

Extends ADR 028 and the frozen wrist study. The user authorized the viable
wrist + fixed `policy` candidate after wrist-only failed visibility. This is a
single-candidate qualification; it cannot establish camera superiority or wrist
noninferiority. Overview remains excluded. No safety, optics or control changes.

## Frozen stages

1. Run the original ten-seed visibility audit. Require paired to pass every
   original gate; retain and disclose wrist failure. Never train failed wrist.
2. Explicit CUDA preflight, then expert acquisition on seeds 20000–20039.
   Validate immutable MCAP and freeze a 20% reserved split (seed 0). Require
   at least 30 eligible successful selected episodes and six reserved episodes.
3. Convert only wrist + policy at 25 Hz. Train the unchanged
   `act-camera-pilot.toml`: seed 1000, batch 8, 5,000 updates, one checkpoint.
   No automatic longer run, sweep or CPU fallback.
4. Freeze the checkpoint before diagnostics 21000–21019 and final 22000–22099.
   Qualify at >=90/100 final successes. Report Wilson 95% CI, every failure,
   completion times, drops and safety rejections, RGB+decision latency
   p50/p95/p99, training wall time and bytes. Qualification is simulation only.
5. Enforce the existing 5 GiB soft artifact budget between commands; retain
   failure evidence and stop on missing dependencies or broken assertions.
   This is not a hard per-command quota. No Hub publication in this pilot.

## Execution

```bash
docker compose --profile gpu run --rm train-gpu \
  python scripts/run-camera-study.py --variant paired \
  --output runs/paired-camera-pilot
```

The default runner remains the original two-candidate comparison. Tests must
verify that paired mode passes its own gate, never converts/trains wrist, and
keeps the frozen data and evaluation seeds. Run dev build, Ruff, mypy, pytest
and doctor before execution. Generated data/models remain ignored.

GPU training reserves 1 GiB `/dev/shm` for the existing four-worker,
four-prefetch DataLoader; Docker's 64 MiB default is insufficient for this
paired RGB batch pipeline. This changes runtime capacity, not model settings.
