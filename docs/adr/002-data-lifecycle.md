# ADR 002: Separate raw MCAP logs from training datasets

- Status: accepted
- Date: 2026-08-29

## Context

Robotics acquisition requires timestamped diagnostics and provenance, while ACT
training needs synchronized tensors/video, normalization, and episode indexing.

## Decision

Record raw sessions as versioned Protobuf messages in MCAP. Validate and convert
selected episodes deterministically into LeRobotDataset. Preserve rejected raw
episodes and record conversion lineage.

## Consequences

There is an explicit conversion stage and some storage duplication. In return,
raw evidence stays inspectable and training data becomes reproducible.

