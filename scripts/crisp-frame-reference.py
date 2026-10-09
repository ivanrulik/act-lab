"""Generate stationary MJCF FK reference; no physics steps or ROS dependencies."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from act_lab.adapters.ros2.contracts import JOINT_NAMES
from act_lab.adapters.ros2.crisp_trace import HOME_JOINTS


def generate_reference() -> dict:
    root = Path("src/act_lab/adapters/mujoco/assets/ur5e")
    model = mujoco.MjModel.from_xml_path(str(root / "act_lab_scene.xml"))
    data = mujoco.MjData(model)
    fixtures = dict(
        home=list(HOME_JOINTS),
        nearby=[q + (-0.02 if i % 2 else 0.02) for i, q in enumerate(HOME_JOINTS)],
        singular=[q if i != 4 else 1e-6 for i, q in enumerate(HOME_JOINTS)],
    )
    for i in range(6):
        fixtures[f"joint_{i}_perturbation"] = [
            q + (0.15 if j == i else 0.0) for j, q in enumerate(HOME_JOINTS)
        ]
    rows = []
    for name, joints in fixtures.items():
        mujoco.mj_resetData(model, data)
        for joint, value in zip(JOINT_NAMES, joints, strict=True):
            data.joint(joint).qpos[0] = value
        mujoco.mj_forward(model, data)
        rows.append(
            dict(
                fixture=name,
                joints=joints,
                attachment_position=data.site("attachment_site").xpos.tolist(),
                attachment_rotation=data.site("attachment_site")
                .xmat.reshape(3, 3)
                .tolist(),
                tip_position=data.site("end_effector").xpos.tolist(),
                tip_rotation=data.site("end_effector").xmat.reshape(3, 3).tolist(),
            )
        )
    result = dict(
        schema_version=1,
        integrated_motion=False,
        mujoco_version=mujoco.__version__,
        model_hashes={
            name: hashlib.sha256((root / name).read_bytes()).hexdigest()
            for name in ("ur5e.xml", "act_lab_scene.xml")
        },
        joint_names=list(JOINT_NAMES),
        fixtures=rows,
    )
    return result


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    result = generate_reference()
    path = Path("tests/fixtures/crisp/kinematics.json")
    if args.check:
        expected = json.loads(path.read_text())
        # Runtime version is provenance; geometry allows only roundoff differences.
        result["mujoco_version"] = expected["mujoco_version"]
        for actual, recorded in zip(
            result["fixtures"], expected["fixtures"], strict=True
        ):
            for key in (
                "joints",
                "attachment_position",
                "attachment_rotation",
                "tip_position",
                "tip_rotation",
            ):
                if not np.allclose(actual[key], recorded[key], rtol=0.0, atol=1e-12):
                    raise SystemExit("stationary MJCF FK differs; regenerate it")
                actual[key] = recorded[key]
        if result != expected:
            raise SystemExit("stationary MJCF reference metadata differs")
    else:
        path.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(path)


if __name__ == "__main__":
    main()
