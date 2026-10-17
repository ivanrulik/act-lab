# Numeric target-loading equivalence and throughput probe

Status: adapter and CPU equivalence gates passed; isolated GPU A/B pending.
Predeclared before production adapter changes.

A read-only profile of LeRobot 0.6.1 / datasets 4.8.5 found 202 image decodes for
one current paired observation plus 100 future action targets. Four explicit
numeric-only projection probes returned identical values for every field,
including padded terminal targets, while decoding two images. Sample read times
were 0.248–0.369 s stock and 0.0069–0.0076 s projected during ongoing training.
These are microbenchmarks, not end-to-end training throughput or a model gain.

The active 5,000 -> 10,000 optimization trial remains on its original loader.
Do not change its configuration or replace its optimizer/RNG state mid-run.

1. Add an opt-in adapter that projects only the future action column before
   framework formatting; leave current observation/image reads and other query
   keys unchanged. Preserve episode mapping, repeated/padded indices and dtype.
2. Default false for historical recipes/manifests. Record explicit opt-in in
   resolved config. Keep pinned upstream files unchanged and optional imports lazy.
3. Compare all fields at episode starts/ends and selected-episode gaps. Prove the
   adapter survives spawned workers. Compare exact trained tensors against stock
   on the same tiny seeded model/data/configuration before any pilot use.
4. Run the required local gates and real CPU learning checks. Then measure an
   isolated GPU training A/B after the current continuation finishes; use the
   same checkpoint, samples, RNG, optimizer, batch and update budget. Compare
   output tensors before attributing any measured throughput improvement.
5. Only after equivalence passes may a later predeclared optimization-budget trial
   opt in. Do not claim the microbenchmark's ~40x read ratio as training speedup.

Validation: dev build, Ruff, mypy (82 files), doctor, 439 dev tests and 37 real
CPU learning/conversion/parser checks passed. Fourteen optional dev skips were
reported; learning checks ran in the separate image. Exact model, optimizer
and RNG tensor equality passed with spawned workers. Active budget trial kept
its original loader.
