# ADR 027: Explicit curated Hugging Face releases

- Status: accepted
- Date: 2026-10-13
- Extends: ADR 002, ADR 011 and ADR 012

## Context

Local artifact accumulation is expensive and does not itself produce a useful
public learning release. A curated validated dataset plus one evaluated inference
checkpoint is much smaller and more reusable than full experiment directories.
ADR 012 keeps training local and disables implicit Hub publication.

## Decision

Add a separate, opt-in `release` CLI and `hub-release` Compose profile. Stage
selected, fingerprinted artifacts and publication cards; verify per-file hashes;
publish only the reviewed inventory; record Hub commit receipts; and verify a
new commit-pinned download. Training remains local with publication disabled.
The Hub SDK loads lazily inside the outer adapter, reusing the pinned training
dependency. Domain types and default simulation require no Hub authentication.

Separate frozen payloads from cards/evidence to preserve existing fingerprints.
Do not reencode videos, alter episode splits, remove required normalization
processors, or upload optimizer state by default. Preserve source provenance,
including dirty training Git state and original versus inspected seed results.
The first recipe releases the legacy educational two-camera policy; articulated
tool learning qualification remains separate.

## Consequences

Publication is explicit and defaults private. Credentials stay outside Git and
mount read-only; OAuth refresh happens on the host. Model and dataset repos have
separate commits and receipts. Failed publication never deletes local artifacts.
Existing conflicting repository inventories/releases are refused. Downloads
verify complete file sets and content identities before use. Storage quotas,
license cards and public visibility are release choices, not training settings.

Current load smoke does not establish hardware use, bit-identical retraining,
a new holdout result or independent newcomer acceptance. Archive buckets and
automatic retention are deferred.
