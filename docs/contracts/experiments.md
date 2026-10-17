# Experiment contract

## Training run

Every run captures:

- immutable or resolved container identity;
- Git revision and dirty state;
- complete resolved configuration;
- dataset fingerprint and episode split;
- random seeds;
- hardware, driver, CUDA, framework, and policy versions;
- checkpoints and scalar metrics;
- start/end status and failure reason.

The CLI is authoritative. Notebooks call the same APIs and may not implement a
separate training pipeline.

## Evaluation

Evaluation uses held-out episode splits and initial-condition seeds. It reports
aggregate results and individual failures. At minimum: task success, completion
time, grasp/drop events, safety-limit events, and confidence intervals.

Preprocessing, state/action conventions, camera ordering, normalization, and
control frequency must match training. A single successful video is an
illustration, not evidence of policy quality.

The evaluator infers ACT's policy frequency from the training dataset metadata,
runs inference at that rate with one action per inference, and holds each target
until the next policy tick. Every held target is retimestamped and sent through
the safety controller at the simulation rate. The report records both rates.
Use `--policy-hz` only when the training dataset metadata is unavailable or an
explicit override is needed.

For failure analysis, `--trace-actions` writes one JSONL row per simulation
step with requested and executed actions, the safety outcome and detail, robot
end-effector pose, and cube pose. Aggregate command outcome counts are included
in each episode report.

The adapter must verify the resolved framework action horizon after loading,
and evaluation reports that effective value. Pinned LeRobot 0.6.1 requires
explicit config CLI overrides; arbitrary loading kwargs can be ignored. A real
checkpoint test must prove each policy call predicts a fresh chunk when horizon
one is requested. See the [loader diagnosis](../reports/paired-camera-diagnosis.md).

### Metric counter precision

Pinned LeRobot formats counters with rounded K/M/B suffixes. The single-process
Compose training adapter records exact update/sample counts from the configured
logging schedule and checkpoint start, including resume. Loss and timing scalars
retain upstream display precision. Historic text logs remain immutable; use
`scripts/recover-training-metrics.py` to write separate corrected JSONL and
provenance bound to the original log hash, dataset and source revision. A running
run cannot be recovered, and existing outputs are never overwritten.

### Optional inference ensemble

`sim evaluate --temporal-ensemble-coeff` explicitly enables pinned ACT temporal
ensembling. Reports must record the effective coefficient and action horizon.
Every call predicts a fresh chunk; episode reset and rejected camera input clear
all prediction history. Default None preserves baseline inference and the frozen
training recipe. An inference change needs a predeclared diagnostic protocol and
a passing candidate still needs untouched final seeds. See ADR 031.
