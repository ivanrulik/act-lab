# ADR 023: Compose-derived CI builds with an external cache

- Status: accepted design; hosted cold/warm qualification pending
- Date: 2026-10-10
- Extends ADR 001

## Context

GitHub's ephemeral hosted runners repeatedly rebuild the pinned ROS/CRISP chain.
Run 38106912264 spent 36.7 runner minutes on eight parallel image builds. Compose
must remain the authoritative build/runtime interface. Caching must not change
test selection, current-source validation, pinned dependencies or motion safety.

## Decision

Use a reusable local composite action with pinned Docker Buildx and Bake actions.
Bake reads `compose.yaml` and the small `compose.ci.yaml` override, which defines
the CI target and expected local image names. Build definitions and service build
contexts remain in Compose. Load the selected image into Docker for the existing
test commands; dependent service targets remain internal cache-only outputs.
For test execution, `compose.ci-runtime.yaml` removes service build definitions
and specifies the loaded image tags. This prevents Compose rebuilding unloaded
build dependencies. Bake receives its build files explicitly; local workflows
continue to use the normal Compose definitions.

Each selected service has one GitHub cache scope. All targets in its dependency
graph import that scope; only the selected target exports its build graph with
`mode=max`. This avoids competing dependency exports overwriting the same scope.
CPU training and recording conversion reuse the `train-cpu` scope. Cache export
has a two-minute timeout and is optional; build/test failures remain fatal.
Keep cache storage settings unchanged, using the included repository allowance.
No paid expansion, registry upload or extra automatic build-record artifact is
enabled. Cache eviction may cause cold builds and must be measured.

Cancel obsolete PR runs using a workflow/PR concurrency group. Main runs use
unique run IDs so main qualification is retained. Keep all nine jobs, test
commands, full RGB evidence and recording/conversion ordering unchanged.

## Dependencies and verification

`docker/setup-buildx-action` v4 and `docker/bake-action` v7 provide the isolated
builder, Compose target graph, GitHub runtime cache authentication and local image
loading. Exact action commits are recorded in the composite action. No domain or
runtime Python dependencies are added. See
[Docker cache integration](https://docs.docker.com/build/ci/github-actions/cache/)
and [Bake from Compose](https://docs.docker.com/build/bake/compose-file/).

Verify all resolved target tags and service dependency contexts, validate workflow
syntax, and run the normal required dev checks. Hosted CI must pass once cold and
once warm on identical source. Compare build restore/export times and cache usage;
success alone does not establish an improvement. A cache hit never skips tests.
Dependency-layer reordering, shared-image distribution, duplicate evidence
removal and change routing remain separate stages in the optimization plan.
