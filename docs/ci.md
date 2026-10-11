# CI builds and caching

CI builds the same Compose definitions used locally, with `compose.ci.yaml`
providing stable image tags and the CI-only target. A pinned Buildx/Bake action
imports optional GitHub Actions caches. Every acceptance job still runs its tests.
The CI runtime override removes build definitions when running loaded images,
preventing Compose from rebuilding unloaded parent service images. Bake receives
its build files explicitly and does not use the runtime override.

Inspect the graph locally:

```bash
docker buildx bake -f compose.yaml -f compose.ci.yaml ros2-recording --print
```

Build and test without GitHub cache credentials:

```bash
docker buildx bake -f compose.yaml -f compose.ci.yaml ros2-recording \
  --set ros2-recording.output=type=docker
docker compose -f compose.yaml -f compose.ci-runtime.yaml --profile ros2-recording \
  run --rm ros2-recording python -m pytest tests/ros2_recording
```

Each selected service has a cache scope. Its dependencies import that scope;
only the selected target exports it, including intermediate layers. Export is
optional and bounded to two minutes. Changed build inputs invalidate affected
layers. Cache loss causes rebuilds; it does not bypass tests.

Paid cache settings remain unchanged. Start within GitHub's included 10 GB
allowance; eviction can reduce cache hits. Automatic Docker build-record uploads
are disabled, while existing acceptance artifacts remain enabled.

Superseded PR runs are canceled. Main qualification runs remain independent.
To qualify performance, complete a cold run and rerun the same commit, then
compare build durations, cache hits and cache usage. Local builds cannot establish
hosted-cache performance. Dependency-layer restructuring, repeated evidence
reduction and change routing remain later stages.

See [the optimization plan](plans/ci-optimization.md) and
[ADR 023](adr/023-compose-ci-build-cache.md).
