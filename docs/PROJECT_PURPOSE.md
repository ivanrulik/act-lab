# Project purpose and alternatives

ACT Lab is an inspectable UR5e robot-learning lab where demonstrations, safety
decisions, dataset changes, and policy failures can be traced and reproduced.
Its contribution is a focused integration and its evidence: existing tools
provide simulation, learning algorithms, recording formats, and visualization.

## Who it serves

Learners and engineers who want to understand the complete imitation-learning
workflow and investigate the boundary between data, policy, and robot control:

```text
teleoperate -> record MCAP -> validate/convert -> train ACT -> evaluate
```

Useful questions include: Was the original image fresh? Which intent was
requested and executed? Why was an episode rejected? Which recordings produced
this dataset and checkpoint? What happened when the producer disappeared? Did
a policy change improve task success under the same evaluation conditions?

## What makes this integration useful

- **Traceable data:** immutable raw acquisition, explicit quality decisions,
  deterministic conversion, episode-level splits, and dataset fingerprints.
- **Comparable experiments:** the expert and ACT use shared application safety;
  reports retain matched seeds, confidence intervals, configuration, and failures.
- **Inspectable faults:** optional ROS simulation exercises freshness, clock
  pause/reset, process loss, recording overload, and recovery. Foxglove exposes
  original images and diagnostic evidence.
- **Replaceable boundaries:** framework-independent domain contracts keep local
  simulation and learning independent of optional ROS adapters.

See [architecture](ARCHITECTURE.md), [data pipeline](data-pipeline.md),
[evaluation evidence](policy-target-feasibility.md), and
[CRISP task/capture qualification](reports/crisp-task-qualification.md).

## Existing tools and when to choose them

The comparison below was reviewed on 2026-10-12 using project documentation.
Recommendations are judgments about fit, not comparative performance results.
Competing stacks were not installed or benchmarked for this review.

| Project | Documented overlap | Consider it when |
| --- | --- | --- |
| [LeRobot](https://github.com/huggingface/lerobot) and [LeLab](https://github.com/huggingface/leLab) | Collection, datasets, policies, evaluation; browser calibration, teleoperation, training, replay, and inference | You want the standard LeRobot workflow or supported robot integration |
| [robosuite](https://github.com/ARISE-Initiative/robosuite) and [robomimic](https://robomimic.github.io/docs/datasets/overview.html) | MuJoCo manipulation tasks/controllers and demonstration-based training/evaluation | You want established manipulation benchmarks and algorithm comparisons |
| [ManiSkill](https://github.com/mani-skill/ManiSkill) | GPU parallel simulation/data collection, diverse tasks, and learning baselines | You prioritize parallel experiments and simulation dataset scale |
| [Isaac Lab Mimic](https://isaac-sim.github.io/IsaacLab/v2.3.1/source/overview/imitation-learning/teleop_imitation.html) | Teleoperation, replay, demonstration expansion, training, and evaluation | You want the NVIDIA simulation and synthetic demonstration workflow |
| [UR5e-LeRobot](https://github.com/Yechen-Fan/UR5e-LeRobot) | Documented physical UR5e collection, multiple teleoperators, training, and evaluation examples | You are assessing an existing LeRobot integration for physical UR hardware |
| [so101-ros2-mujoco](https://github.com/MuRain37/so101-ros2-mujoco) | UR5e/Robotiq simulation, wrist RGB, ROS Jazzy, MCAP, LeRobot conversion, and policy inference | You want a closely related ROS simulation workspace with multiple tasks and policies |

The last project has particularly close overlap and documents runtime validation
and reset isolation as well. The component combination alone does not establish
ACT Lab's distinct value. This review does not establish that other projects lack
equivalent data, safety, or evaluation checks. Physical integration descriptions
in another repository do not establish suitability for a particular robot setup.

## Evidence and current limits

Earlier local ACT experiments have measured task-success results, with limits
documented in [the coverage experiment](policy-coverage-experiment.md) and
[target-feasibility regression](policy-target-feasibility.md). Those results
must remain associated with their original model and configuration.

For the current articulated gripper/wrist-camera preset, the
[training smoke](reports/gripper-wrist-camera.md) establishes data plumbing,
checkpoint compatibility, and inference execution. A full learned-policy
training/evaluation campaign remains separate work. The current CRISP showcase
qualifies scripted expert execution and acquisition on inspected seeds; it does
not establish learned-policy generalization or physical robot readiness.

Educational usefulness also needs an independent newcomer trial. Maintainer
validation does not measure how easily someone unfamiliar with the repository
can complete the workflow. A reusable dataset, compatible checkpoint, held-out
report, and short walkthrough would make that value easier to verify; these
belong in the [roadmap's reproducible learning release](ROADMAP.md).

## Scope discipline

Prefer changes that improve traceability, reproducibility, failure diagnosis,
or the teaching workflow. Reuse upstream policies and tools where they satisfy
the need. Additional robots, policies, tasks, or interfaces should have a concrete
use case and evidence requirement before expanding the platform.

The [roadmap](ROADMAP.md) controls delivery boundaries. This positioning note
does not change accepted architecture decisions or authorize hardware use.
