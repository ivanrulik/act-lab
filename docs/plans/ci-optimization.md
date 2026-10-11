# CI optimization research and staged plan

Date: 2026-10-10. Baseline: GitHub run 38106912264, commit f9f580b.
Stages 1–2 are implemented on `codex/ci-build-cache`; hosted cold/warm qualification
is pending. Later stages remain proposed. Test selection is unchanged.

## Measured costs

| Job | Image build | Tests / evidence |
|---|---:|---|
| container | 45 s | default tests 40 s |
| training-cpu | 140 s | training tests 13 s |
| ros2-contracts | 56 s | small DDS contract suite |
| crisp-feasibility | 337 s | tests 27 s; evidence 26 s |
| ros2-simulation | 355 s | tests 190 s; repeated motion evidence 162 s |
| ros2-observability | 411 s | tests 25 s |
| tooling | 425 s | loaded controller evidence 263 s |
| ros2-recording | 434 s | tests 44 s; failed before full capture |
| ros2-recording-conversion | not run | skipped after recording failure |

The eight image builds consumed 2,203 seconds (36.7 runner minutes) in parallel.
This is compute time, not elapsed workflow duration. Five jobs independently
build the CRISP-derived chain. The workflow has no external build cache or
concurrency cancellation. The default Dockerfile copies source/tests before pip
installation, so ordinary edits invalidate dependency installation too.

The earlier run 38104044726 passed recording after about 11.5 minutes, then
started a separate CPU image build for conversion. This dependency is necessary
for capture artifacts, but the CPU build does not need to wait for recording.

## Recommended order

1. **Cancel superseded PR runs.** Use a workflow/PR concurrency group and
   conditional cancel-in-progress for PRs. Keep main qualification runs intact.
   [GitHub concurrency](https://docs.github.com/en/actions/how-tos/write-workflows/choose-when-workflows-run/control-workflow-concurrency).
2. **Add persistent BuildKit caches.** Use Docker's maintained Buildx/Bake actions
   with Compose as the build definition. Start with per-image cache scopes and
   `mode=max` to retain intermediate build stages. Measure restore/export time and
   eviction before deciding between GitHub cache and registry storage. Do not
   claim a warm-cache improvement from a cold run.
   [Docker cache integration](https://docs.docker.com/build/ci/github-actions/cache/),
   [cache scopes](https://docs.docker.com/build/cache/backends/gha/),
   [Bake from Compose](https://docs.docker.com/build/bake/compose-file/).
3. **Separate dependency layers from project installation.** Install constrained
   Python dependencies from dependency metadata before copying changing source,
   tests and notebooks; install ACT Lab afterward without resolving dependencies
   again. Preserve generated Protobuf packaging and CPU/GPU separation. Keep
   runs/data excluded from build context. Ordinary source changes should not
   reinstall PyTorch or rebuild pinned upstream CRISP.
   [Docker layer ordering](https://docs.docker.com/build/cache/optimize/).
4. **Remove repeated image builds and evidence execution.** Prototype a shared
   ROS build job that exports exact-commit images for separate isolated test jobs;
   compare image transfer cost with independent cached builds before choosing.
   Prepare the CPU conversion image alongside recording, then let conversion
   consume both outputs. Reuse the full `run_motion` report already produced by
   pytest instead of executing that same full qualification again. Retain a small
   CLI invocation check and all safety assertions.
   [Docker image sharing](https://docs.docker.com/build/ci/github-actions/share-image-jobs/).
5. **Introduce conservative change routing last.** Changes to shared domain,
   application, safety, models, dependencies or CI trigger the full suite. Any
   reduced PR suite needs explicit dependency mapping and an always-running
   aggregate required check; failures and unexpected skips must fail that gate.
   Do not filter an entire required workflow by paths: GitHub warns its checks
   can remain pending and block merging. Preserve full main qualification.
   [GitHub workflow filters](https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-syntax).

## Validation stages

- Establish baseline build, test, image-transfer and artifact costs from job steps.
- Verify Bake's printed targets preserve Compose's dependency contexts and tags.
- Run cold and warm CI on the same source; then change only Python source and
  verify pinned dependency layers are cache hits while current source is tested.
- Change constraints/generated interfaces to prove appropriate invalidation.
- Inject a failing test and verify the merge gate fails; test superseded PR
  cancellation and numeric UID/GID 1001 conversion.
- Retain every ROS safety suite, full RGB capture and ROS-free fingerprint
  equivalence during the initial cache rollout. Compare outcomes and artifacts.

Do not set a promised wall-time target yet. Caching can eliminate much of the
measured build cost; full RGB acquisition and loaded dynamics remain real work.
Changing those acceptance workloads requires separate reviewed scope.

## Latest failure

Run 38106912264 passed seven jobs. Recording failed its new pause regression at
an exact three-sample assertion; conversion was skipped. All pause/state/gripper
and consecutive 20 ms assertions passed. Uploaded quality evidence shows four
valid samples, one preserved rejected attempt and no sample-gap error. The count
must match observed DDS attempts plus the terminal disabled sample, rather than
assuming every initial attempt arrives without recovery. Keep original quality
and fault assertions intact.

## Initial implementation

Run 38108031631 on 4815f42 passed all nine jobs before caching changes. The
separate cache branch keeps that recording PR intact. ADR 023 adds a composite
Buildx/Bake action using Compose build definitions and per-service GitHub cache
scopes. Only the selected target exports; dependency targets import its cache.
Exports time out after two minutes without failing successful test builds.
Existing test/evidence commands remain unchanged. Superseded PR runs cancel;
main runs retain distinct concurrency groups. No cache storage/billing setting
is changed, and automatic Docker build-record uploads are disabled.

Local validation passed: dev build, Ruff, strict mypy (71 files), doctor and
default pytest (333 passed, four expected optional skips). Actionlint and Compose
configuration checks passed. Actual Bake builds loaded the CI, ROS-contract and
recording targets. Three dedicated ROS-contract tests passed; the 13 recording
tests passed. An initial combined suite incorrectly ran contract provenance
checks in the recording image; those checks passed in the dedicated image.
The runtime override removes build definitions so test startup cannot rebuild
unloaded parent targets. Hosted cold/warm performance remains unqualified.
