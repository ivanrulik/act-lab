# ADR 001: Docker Compose is the authoritative environment

- Status: accepted
- Date: 2026-08-29

## Context

Robotics and ML stacks combine native libraries, graphics, video, and optional
CUDA dependencies. Host installations drift and make experiments difficult to
reproduce.

## Decision

Use reusable multi-stage Docker images and Docker Compose as the supported
development and execution interface. Keep webcam, display, and GPU access in
opt-in profiles. Run unprivileged and mount only required paths/devices.

## Consequences

CI and local execution share an environment. Interactive desktop support needs
documented host adapters, and Docker Desktop webcam limitations may require a
future host camera bridge. Native Python remains a diagnostic fallback.

