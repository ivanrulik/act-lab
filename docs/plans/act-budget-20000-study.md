# Requested-intent ACT: conditional 20,000-update continuation

Status: active; official CUDA resume started from 10,000 with explicit
project_action_columns=true after the three isolated GPU A/B pairs passed exact
model/optimizer/RNG comparison and source-checkpoint integrity checks.
Protocol was predeclared before this gate completed.

The 10,000-step ensemble model completed 8/20 diagnostics, versus 3/20 at 5,000:
two previous successes retained, six gained, one lost. Its 95% Wilson interval
is 21.88–61.34%; these adaptive diagnostic results are not final qualification.
Sampled median XYZ errors fell from 18.46/18.54 mm to 6.06/8.09 mm train/reserved.
Collision-stop commands rose from 827 to 1,469 across eight seeds; all guards
remain active. The 18/20 gate failed and no final seeds were consumed.

The reductions in sampled error and observed task gains justify one additional
bounded optimization hypothesis. They do not prove a larger budget will qualify
or that collision outcomes will improve. No loss weighting, VAE, camera, model,
state/action representation, gains, thresholds or safety changes are bundled.

1. Snapshot the completed 10,000-step manifest, source identity and all inference
   hashes. Preserve 5,000 and 10,000 checkpoints and every prior failed result.
2. Require exact loading equivalence and completed three-repeat isolated GPU
   A/B. Only then explicitly enable `project_action_columns` as the validated
   execution optimization; record its provenance and the ADR 032 exception.
3. Resume the official pipeline from 10,000 to **20,000 total updates** with
   optimizer/RNG/data-order state intact. Retain seed 1000, batch 8, learning rate,
   architecture, dataset/splits, cameras, normalizers, device and save/log cadence.
   Save intermediate 15,000 as scheduled; evaluate only predeclared final 20,000.
4. Evaluate seeds 21000–21019 with coefficient 0.01, actual horizon one, fresh
   prediction, 25 Hz decisions, 50 Hz safety and 15-second task budget. Report
   every failure, grasp/drop and collision-stop command, latencies and error probe.
5. Require >=18/20 before freezing a candidate for an audited unused 100-seed
   final cohort (candidate 23000–23099), target >=90/100 with Wilson intervals.
   A failed gate requires diagnosis, not automatic extension or final-seed reuse.

Reuse all data. Combined run artifacts must stay below the 5 GiB soft gate.
Record per-attempt source Git/container/configuration in a sidecar, preserving
initial and completed manifests. No hardware use, publication or merge follows.

Runtime record: resume source `421b787`; initial completed manifest and actual
source/image/configuration sidecar are retained. The preflight projected peak
including shared raw data and planned checkpoints was 4,863,388,660 bytes, below
5 GiB. Early sustained throughput is approximately 9.1 updates/s; task outcome
remains pending.
