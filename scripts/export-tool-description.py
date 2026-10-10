"""Generate the visualization and rigid six-arm-DOF URDFs in the optional ROS image."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import mujoco

from act_lab.adapters.mujoco.environment import MujocoUR5eEnvironment
from act_lab.adapters.mujoco.tool_description import compose_tool_urdf


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, default=Path("/opt/crisp/ur5e.urdf"))
    parser.add_argument("--output", type=Path, default=Path("runs/tool-description"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with MujocoUR5eEnvironment.from_config_file(
        Path("configs/sim/ur5e_2f85_d405.toml")
    ) as env:
        env.reset(0)
        base = args.base.read_text()
        visual = compose_tool_urdf(base, env._model)
        # The controller has six arm DOFs; freeze the nominal cube-grasp aperture.
        env.gripper.set_kinematic(env._data, 0.05 / 0.085)
        mujoco.mj_forward(env._model, env._data)
        rigid = compose_tool_urdf(base, env._model, rigid_data=env._data)
        (args.output / "ur5e_2f85_d405.urdf").write_text(visual)
        (args.output / "ur5e_2f85_d405_rigid.urdf").write_text(rigid)
        (args.output / "provenance.json").write_text(
            json.dumps(
                dict(
                    ur_description_revision="6b639c2efd9f4f12da858a72cf1a9e40da365ed8",
                    base_sha256=hashlib.sha256(base.encode()).hexdigest(),
                    visual_sha256=hashlib.sha256(visual.encode()).hexdigest(),
                    rigid_sha256=hashlib.sha256(rigid.encode()).hexdigest(),
                    rigid_nominal_aperture_m=0.05,
                    articulated_control_dynamics=False,
                    model=env.model_identity,
                ),
                indent=2,
            )
            + "\n"
        )


if __name__ == "__main__":
    main()
