# ADR 034: Public presentation editions of frozen releases

- Status: accepted
- Date: 2026-10-25
- Supersedes: ADR 027's same-repository changed-inventory rejection only for
  explicitly requested presentation editions; ADR 033's private-only choice for
  the paired release after the user's public-publication request

## Context

The paired release has passed upload/download and clean-source integration
checks. The user now wants the model and dataset publicly usable as a project
showcase, with clear attribution, results, limitations and a short demonstration.
Changing the current card must not silently invalidate a pinned release.

## Decision

Keep all original commits and receipts. A separate explicit `release promote`
operation verifies a new local package, loads the baseline inventory at the
receipt's immutable commit, and permits only README/reproduction-instruction
edits and preview MP4/JPEG/PNG additions under `media/` (20 MiB maximum).
Payload, inference settings, evidence, notices and release identity cannot
change; no files may be removed. Changed bytes require a new checksummed
manifest and Hub commit. A different head/inventory is refused. Commits use the
expected parent SHA; retry is permitted only when the head's manifest already
matches the verified presentation edition.

Visibility changes to public only after the presentation commit is confirmed.
Write a separate public receipt recording its predecessor. Original private
receipts retain their historical meaning; those old commits become accessible
when their repository is public. Normal `release publish` remains strict and
private by default. No API automatically promotes during training or staging.

The preview samples original recorded paired RGB at 25 Hz, encodes a 10.16 s
side-by-side MP4 and labels it a consumed-seed ACT demonstration at 1x simulation
time. It is a presentation derivative, not a replacement raw record or new
qualification. No renderer, dependency, safety path or weights change.

## Consequences

Visitors can inspect and download without authentication, with model, dataset
and GitHub cross-links. New public commits require anonymous byte verification;
old commit-pinned downloads remain valid. Public visibility does not establish
new evaluation, hardware safety or independent newcomer acceptance. Legacy
educational repositories remain outside this promotion.
