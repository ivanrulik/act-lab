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

