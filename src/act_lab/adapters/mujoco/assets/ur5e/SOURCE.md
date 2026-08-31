# Vendored UR5e model

- Source: `google-deepmind/mujoco_menagerie/universal_robots_ur5e`
- Revision: `da76818e269b82289eba39808e2fb91d679d6994`
- License: BSD-3-Clause; preserved in `LICENSE`
- Upstream `ur5e.xml` SHA-256:
  `ffd0a7b336d3c2d27c35a15cc0d4aaa95196769aca190ac8d6522d3318c345a0`

ACT Lab modifies the vendored XML to set the fixed physics timestep and attach
the simple parallel gripper required by roadmap PR 2. `act_lab_scene.xml` and
the gripper are ACT Lab additions. The mesh files are unchanged from upstream.
