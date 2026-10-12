# ADR 026: Model-specific loaded watchdog hold

Status: Proposed; local and hosted qualification pending.
Date: 2026-10-12
Partially supersedes: ADR 016's scene-stiffness hold and ADR 018's damping for
`ur5e_2f85_d405_v1` only. The legacy educational model retains ADR 018.

## Context

[Main CI run 38301550098](https://github.com/ivanrulik/act-lab/actions/runs/38301550098)
failed the complete-task recording test after PR #26 merged. The task finished
and its recording passed quality checks, but one integrated tick reached
4.059721 rad/s² joint acceleration against the unchanged 4.0 rad/s² ceiling.
The other 28 recording tests passed.

At simulation time 74.022 seconds, a gateway wall timeout transferred a moving,
loaded robot into hold. Task effort was immediately zero. ADR 018's 30/6
Nm s/rad hold damping braked this articulated transit state too abruptly.
Scheduling selected the takeover state; hold dynamics caused the violation.

The saved eight loaded fault tests also contain hold acceleration maxima of
6.09–6.61 rad/s². Their assertions checked retained grip, wrist drift, cube drop
and recovery, but omitted every-tick acceleration. Those passing tests did not
establish compliance with all measured motion bounds.

Reconstructing the preceding six-joint positions/velocities, measured aperture,
and cube center reproduces the task spike without ROS or wall scheduling.
Passive finger joints are reconstructed from measured aperture; cube rotation,
velocity and solver state are not replayed. These are deterministic regression
fixtures, not exact reproduction of all CI dynamics.

## Decision

Use model-specific simulated hold parameters:

| Model | Position stiffness relative to scene | Proximal damping | Wrist damping |
| --- | --- | --- | --- |
| Legacy educational UR5e | 1.0 | 30 Nm s/rad | 6 Nm s/rad |
| Articulated 2F-85/wrist preset | 0.5 | 10 Nm s/rad | 2 Nm s/rad |

Keep exclusive hold ownership and existing actuator force ceilings. Reset the
scene actuator parameters before applying each hold so scaling never compounds.
Retain original velocity, continuing contact dynamics, accepted gripper aperture,
immediate task-effort inhibition, and fresh-command recovery. The 4 rad/s²
acceleration limit and all freshness/quality gates remain unchanged. Local
Cartesian driver actuation is unchanged.

A shared model-specific parameter function configures the plant and runtime
provenance. Unsupported model IDs fail explicitly rather than inheriting tuning.

The nine reconstructed articulated takeovers peak at 3.610195 rad/s² joint
acceleration. Maximum wrist drift is 1.135077 mm during their one-second holds;
maximum final cube drop is 0.733664 mm. Every tick retains pad/cube contact.
These values describe the reconstructed fixtures, not the original full states.

## Rejected candidates

25/5 damping with original stiffness fixes the complete-task reconstructed
spike but leaves loaded fault holds above 4 rad/s². Reducing only damping to
15/3 or 10/2 also fails some loaded reconstructions because position stiffness
continues braking the moving arm. Retain these findings rather than weakening
acceleration assertions.

Applying the selected articulated tuning to the legacy model slows its existing
loaded-hold settling: terminal joint speed is 0.01544 rad/s at 0.5 seconds,
above its 0.01 rad/s acceptance criterion. Model-specific selection preserves
the already qualified legacy tuning. It does not assume tools and inertias
share a suitable hold configuration.

## Safety review and acceptance

| Case | Required evidence |
| --- | --- |
| Nine reconstructed articulated faults | Every hold tick meets joint/translation/angular acceleration bounds; <2 mm wrist drift, <1 cm cube drop, pad contact and accepted aperture retained, task effort zero |
| Legacy moving timeout and loaded settling | Existing deterministic takeover and loaded-hold regressions pass |
| Startup and unsafe candidate | Existing effort-plant hold/ownership regressions pass |
| Complete articulated task | Actual DDS/CRISP capture completes, releases cube, passes quality and every-tick physics checks |
| Eight loaded process/clock/acquisition faults | Retained grip/hold and recovery checks plus new whole-trace physics assertions pass |
| Moving ROS adapter | Existing ROS simulation suite and measured smoke evidence pass |

Run the required dev build, Ruff, mypy, pytest and doctor, plus the full ROS
recording and moving-simulation checks. Hosted CI must validate the branch.
The failed main artifacts remain under ignored `runs/main-ci-38301550098/`.
Local parameter comparisons and rejected/qualified candidate evidence remain
under ignored `runs/crisp-hold-fix/`.

## Limits

These fixtures qualify observed simulated stopping states and existing regression
cases. Reduced stiffness/damping does not prove all robot states or payloads stop
within these limits. Broader stopping-state coverage and physical stop/hold remain
separate work; no hardware claim follows. The prior frozen task/capture campaign
remains evidence for its recorded revision and is not rerun by this correction.
