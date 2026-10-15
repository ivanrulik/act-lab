# ADR 028: Explicit camera selection and gated wrist study

- Status: accepted
- Date: 2026-10-15
- Extends: ADR 011, ADR 019 and ADR 027

## Context

The current articulated robot has three RGB views, but no full qualified ACT
campaign. Extra views cost acquisition, storage and visual processing. A smaller
camera set needs measurable evidence. Source review supports experimentation:
[original ACT simulation configs](https://github.com/tonyzhaozh/act/blob/742c753c0d4a5d87076c8f69e5628c79a8cc5488/constants.py)
use one overhead view;
[UMI's task](https://github.com/real-stanford/universal_manipulation_interface/blob/d095ba9590df789df5189eea5ee7e431689038a6/diffusion_policy/config/task/umi.yaml)
uses wrist RGB plus proprioception with a different policy/optical design;
[a published wrist-only ACT](https://huggingface.co/TakuyaHiraoka/act_so101_pick_diverse_objects)
discloses field-of-view limitations without statistical success metrics; and
[the comparable UR5e/MuJoCo task](https://github.com/MuRain37/so101-ros2-mujoco/blob/6f63e78ccaac928c55e2dbe515866c3a013e599d/src/mujoco_sim/mujoco_sim/tasks/ur5e_red_cube_to_target.py)
declares fixed and wrist views. These do not settle our camera choice.

## Decision

Add explicit wrist-only and wrist+fixed presets. Conversion can project a named
camera subset after validating original MCAP selection, quality and calibrated
source identities. Model identities and selected camera profiles remain strict;
the articulated model must retain wrist. Each derived logical fingerprint
includes selected camera IDs/bytes. Preserve raw provenance, episode order,
selection manifest, numeric samples and splits. Keep the original source camera
contracts alongside the projected one. Dropped-view calibration differences can
be irrelevant to a selected subset; selected-view/model differences still fail.

Run a bounded privileged geom-segmentation audit before collecting new training
data. The frozen study plan defines visibility thresholds, disjoint seeds,
matched training/evaluation gates and artifact budgets. Segmentation, cube/tray
identity and expert phase labels are diagnostics only; no generic observation or
policy receives them. Rendering must not advance physics or change dynamics.

Keep the existing strict evaluation comparator. A separate camera-study
comparator permits only the declared camera difference, verifies matched data,
training and execution settings, and reports quality/cost gates separately.
Evaluation adds bounded wall-time samples around RGB capture plus policy.act;
these do not alter source timestamps, safety checks or control clocks.

## Consequences

A conservative visibility no-go stops the training workflow without pretending
to establish policy success. Missing dependencies or broken assertions are
execution failures. A recurrent/actively observing policy might tolerate unseen
targets, so an audit failure is not proof of general wrist-only impossibility.
No camera movement, wider lens, new dependency, ROS path, hardware motion or
weaker safety limit is introduced. Different optics, active search or a changed
observation gate require an explicit subsequent experiment.
