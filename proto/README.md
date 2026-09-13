# Recording schemas

`act_lab/v1/episode.proto` defines raw MCAP channels. Compatible changes add new
field numbers; never reuse numbers or change meanings. Breaking changes require
a new package/profile version and migration decision.

The build hook invokes pinned protoc to generate `act_lab_episode.desc`, installed
with the package and embedded in each MCAP schema. Generated artifacts are not
committed. Rebuild the Docker image after schema edits. See
[recording](../docs/recording.md) and ADR 010 for lifecycle and clock semantics.
