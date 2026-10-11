# PR 14 implementation and qualification stages

Branch: `feature/pr14-ros-mcap-equivalence`, base `5e2116a`.
ADR 021 and `contracts/ros2-recording.md` define scope and safety limits.

1. **ROS-free contracts:** atomic sample codecs, source time, full reports, RGB,
   provenance/lifecycle, contiguous sequences, reset segmentation and interruption.
   Gate: unit tests require no generated interfaces or ROS imports.
2. **Required transport/storage:** generated CDR, separate publisher/recorder DDS,
   rosbag2 MCAP with CRCs, bounded ACKs, durable finalization and staged import.
   Gate: missing dependencies fail; gap, replay, loss, wrong episode and reset
   cases cannot publish training-ready output.
3. **Actual simulation acquisition:** local shared safety/recording path and stepped
   real CRISP scratch snapshots. Gate: all declared original cameras align with
   post-command state; renderer stays outside physics; final disabled hold recorded.
4. **Learning equivalence:** complete seeded expert acquisition; compare decoded
   local/ROS episodes, quality decisions and deterministic resampling. Run the
   public manifest/conversion CLI in the ROS-free CPU image. Gate: logical
   dataset fingerprints match, source lineage verifies and ROS is never imported.
5. **Delivery:** Compose build, Ruff, mypy, pytest, doctor, required ROS-contract and
   recording suites, CPU learning regression, CI artifact handoff, documentation,
   safety matrix and draft PR. Interactive Foxglove acceptance is separately
   reported; the earlier flange appearance follow-up remains deferred.

This PR qualifies stepped acquisition. Continuous paced recording and physical
sensor synchronization require a new buffering/drop/backpressure design and
independent hardware safety qualification. Full CRISP pick/place success and
crash-prefix repair tooling remain follow-ups.
