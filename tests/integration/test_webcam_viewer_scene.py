from __future__ import annotations

from types import SimpleNamespace

import mujoco

from act_lab.adapters.mediapipe.session import _update_command_scene
from act_lab.adapters.mediapipe.viewer_ui import ViewerHud


def test_command_target_and_error_arrow_populate_mujoco_scene() -> None:
    model = mujoco.MjModel.from_xml_string("<mujoco/>")
    scene = mujoco.MjvScene(model, maxgeom=3)
    handle = SimpleNamespace(user_scn=scene)
    hud = ViewerHud(
        banner="ACTIVE | APPLIED",
        lines=(),
        target_xyz_m=(0.4, 0.1, 0.6),
        actual_xyz_m=(0.3, 0.1, 0.6),
        executed_target_xyz_m=(0.4, 0.1, 0.6),
        target_rgba=(0.1, 0.85, 1.0, 0.85),
        show_target=True,
    )

    _update_command_scene(mujoco, handle, hud)

    assert scene.ngeom == 2
    assert scene.geoms[0].type == mujoco.mjtGeom.mjGEOM_SPHERE
    assert scene.geoms[1].type == mujoco.mjtGeom.mjGEOM_ARROW
