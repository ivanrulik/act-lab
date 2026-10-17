# Qualified paired ACT release

Date: 2026-10-17. Release candidate: articulated UR5e/2F-85 with paired wrist and
fixed policy RGB. The distinct private repositories preserve the historical
educational release. No raw acquisition archive, optimizer state or intermediate
checkpoint is included.

## Inventory and identities

- Dataset payload: **662,097,467 bytes**, 40 episodes / 11,624 frames, with the
  original 30 training / 10 reserved episode split.
- Inference payload: **206,738,459 bytes**, eight unchanged files.
- Combined staged inventory including cards, notices, reproduction files,
  evidence and two replays: approximately **1.015 GB**, below the 1.1 GB gate.
- Logical dataset fingerprint:
  `49e3f6f6fa7f016bef373cf4050b88f8b6c81905106332cdde70813291a2be04`.
- Storage fingerprint:
  `baace059af172bc39a3df8c4a06d7066493dc61c82681a5b845da07860ce6d52`.
- Checkpoint aggregate SHA-256:
  `4525c1dc57eb840ffe2090317813a3998e9fdad0f2ccc8a569a7d3c386986d6d`.

`inference.json` binds the original qualification to horizon one, 25 Hz policy
decisions and ensemble 0.01; shared safety remains 50 Hz. The serialized training
defaults are deliberately not substituted. Simulation config, model/camera
contract, completed training and exact episode indices are checked. All hashes
are inventory-verified before upload and after download/use.

## Qualification and illustrations

The original frozen model passed 18/20 diagnostics and **90/100 fresh final
tasks**, Wilson 95% interval 82.56–94.48%. Its ten timeouts and 1,073
collision-stop commands remain in the packaged per-episode evaluation. This
release does not rerun or enlarge that qualification cohort.

Two additional Foxglove examples illustrate consumed seeds: 23000 succeeds in
10.16 simulated seconds; 23003 times out at 15 seconds. Reruns preserved the
original step counts and safety outcomes, with maximum compared end-effector
coordinate differences of 0.440 mm and 1.613 mm respectively. They are actual
new camera captures, not bit-identical original traces or expert training data.
The separately retained seed-23001 demonstration is omitted from publication.

The included viewer derivatives add standard RawImage topics to original
canonical messages. Canonical raw logs are retained locally; viewer derivatives
are not accepted by the raw-training validator as canonical episodes.

## Validation and publication

Publication/download receipts and clean-source results are recorded below after
completion. The staged CUDA integration check passed two consumed seeds using
the packaged model/config, loaded all 30 training episode indices and checked
state/action/image shapes. Dataset and checkpoint hashes were unchanged after
execution, and ROS was not imported.

The existing optional pinned Hub SDK is reused. No dependency, domain contract,
default training behavior or ROS requirement was added. UR5e and Robotiq source
notices are preserved separately from the project's Apache-2.0 release metadata.

## Remaining acceptance

Independent newcomer completion, public visibility, full raw-to-derived
reconstruction from a remote archive, bit-identical retraining, wrist-only
qualification, broader generalization and learned ROS/live/hardware execution
remain separate. These release checks qualify artifact transfer and integration,
not a new statistical result or a physical safety claim.
