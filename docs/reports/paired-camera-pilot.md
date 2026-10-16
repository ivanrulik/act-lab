# Paired-camera ACT pilot — not qualified at 5,000 updates

Date: 2026-10-16. Scope: articulated simulated UR5e/2F85, wrist + fixed policy
RGB, shared application safety. [Frozen plan](../plans/paired-camera-pilot.md).
[Machine-readable evidence](paired-camera-pilot-metrics.json) contains every
per-seed outcome, confidence interval, completion time, event, configuration,
source/calibration hash, dataset identity, split and checkpoint digest.

## Interpretation correction

Subsequent [diagnosis](paired-camera-diagnosis.md) found the loader silently
retained a 100-action queue despite reporting one. The historical results below
measure that unintended execution mode. Corrected one-step feedback scored
0/20 diagnostic successes and stalled before grasping. Treat the 33/100 and its
latency as historical measurements, not qualification of the declared protocol.

## Decision

**Do not release this checkpoint as a qualified policy.** ACT completed 33/100
final tasks, below the frozen 90/100 target. The bounded assessment completed
successfully and made no automatic longer run or publication. It does not
establish that the camera pair is unsuitable or explain the failures causally.
This is one candidate, with no measured camera superiority or noninferiority.

| Cohort | Expert | ACT | ACT 95% Wilson interval |
| --- | ---: | ---: | ---: |
| Diagnostic 21000–21019 | 20/20 | 6/20 | 14.5–51.9% |
| Final 22000–22099 | 100/100 | 33/100 | 24.6–42.7% |

Both cohorts used the same frozen checkpoint, CUDA, 25 Hz policy / 50 Hz
environment, and `SafeCartesianRobot.command(Action)`. No settings were tuned
between diagnostics and final evaluation. Final seeds are now consumed and must
not be presented as untouched evidence for a subsequent tuned model.

## Failure and safety evidence

All 67 final ACT failures timed out at 15 simulation seconds: 58 had no recorded
grasp and nine recorded a grasp without completing placement. The model recorded
42 grasps and zero drops overall. Mean successful completion was 12.944 s
(median and individual times are in the JSON); expert mean was 11.533 s.

Application safety rejected 2,439 commands: 2,168 collision stops and 271 IK
failures. Another 69,168 commands were limited by workspace/rate handling.
These are command decisions, not counts of executed unsafe motion. The expert
had zero rejected commands. No root cause is established by these counts alone;
reaching/closing and the post-grasp approach warrant diagnostic inspection.

## Data, runtime and cost

- Collection 20000–20039: 40/40 expert successes, all quality-valid MCAP.
- Frozen hash split: 30 training / 10 reserved episodes using fraction 0.2,
  split seed 0; the hash assignment does not promise exactly eight reserved.
- Dataset: 11,645 frames at 25 Hz; **663,634,399 bytes** (633 MiB).
- Training: seed 1000, batch 8, 5,000 updates, unchanged ACT architecture,
  deterministic cuDNN, no AMP; **42 min 51 s** including command startup.
- Model inference payload: **206,738,252 bytes**. Full training directory with
  resumable state: 619,942,101 bytes. Raw acquisition: 1,385,027,581 bytes.
- All pilot artifacts: approximately **2.49 GiB**, below the 5 GiB soft gate.
- Final RGB + decision latency: p50 **2.508 ms**, p95 **3.650 ms**, p99
  **8.382 ms**, maximum **291.759 ms**, 35,814 samples. Includes cold calls;
  this offline wall-time measurement is not a real-time guarantee.
- RTX 4060 Laptop GPU, CUDA 12.8, Torch 2.11.0+cu128, LeRobot 0.6.1,
  Python 3.12.14. Acquisition and evaluation used NVIDIA rendering consistently.

The initial acquisition was stopped early to reserve 1 GiB shared memory for
four prefetched data workers. Its small artifacts were retained separately.
The complete subsequent cohort and training were run once, without a training
restart. Training source revision was `73b8f35aaabdd6a5b98eefb115b2c48771735476`;
an unrelated untracked editor log marked Git dirty. Later source commits changed
documentation only. No large data or checkpoint is committed.

## Renderer limit

The paired gate passed all ten audit seeds on both Mesa software rendering and
NVIDIA hardware rendering. Wrist failed the earlier Mesa gate but passed the
NVIDIA gate with matching source/model/calibration identities. Some target
counts changed from zero to 20–23 pixels, crossing the frozen 16-pixel threshold.
The underlying rasterization cause remains unproven. See the corrected
[visibility report](wrist-camera-study.md). Wrist-only feasibility remains open;
this paired pilot cannot settle the leanest camera choice.

## Validation and next gate

Dev build, Ruff, mypy (81 files), doctor, Compose and diff checks passed. Full dev
suite: 417 passed, six expected optional skips. Standalone image runner suite:
13 passed. All nine hosted jobs passed on implementation commit `83a8386`.
Documentation/evidence additions are checked separately on the subsequent head.

Next work should inspect the existing diagnostic failures and predeclare a
single changed factor (for example, training budget) with a fresh final holdout.
A training/data change needs a new frozen protocol. Do not tune against the
consumed final cohort, weaken safety or publish this model as qualified.
