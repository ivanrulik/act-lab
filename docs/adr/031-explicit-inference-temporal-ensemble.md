# ADR 031: Explicit optional ACT inference temporal ensembling

- Status: accepted
- Date: 2026-10-16
- Extends: ADR 012
- Supersedes: disabled temporal ensembling only for explicitly requested inference diagnostics

## Context

The corrected adapter enforces fresh horizon-one inference. The requested-intent
5,000-update model still times out on all 20 diagnostic seeds despite improved
motion and sub-period p95 decision time. The pinned framework provides an online
ensemble of overlapping predictions, with a fresh chunk predicted on every call.
The original recipe disables this option; its default remains reproducible.

## Decision

Add explicit `sim evaluate --temporal-ensemble-coeff` and a typed optional adapter
parameter. Validate finite coefficients and horizon one. Resolve the coefficient
through explicit framework config overrides, verify it, and report its effective
value. Clear ensemble history on episode reset and rejected camera inputs. Keep the default None, training recipe,
model weights, rates, camera checks and shared application safety unchanged.

Use the pinned implementation without modifying its algorithm. Older overlapping
predictions contribute to a new inference decision; no transport timestamp or
camera freshness check is bypassed. The [bounded experiment](../plans/act-temporal-ensemble-study.md)
uses coefficient 0.01 once on diagnostic seeds, not a sweep or final qualification.
No new dependency, physical claim or automatic hardware authorization follows.

## Consequences

Ensembling changes inference behavior and must be explicit in evidence. Passing
unit/loading tests is not proof of task success. Any successful candidate still
requires the existing unused-final-cohort and safety review gates. None remains
the baseline configuration; the incomplete/failed earlier experiments remain
reported.
