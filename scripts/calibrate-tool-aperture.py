"""Generate the unloaded 2F-85 aperture/actuator lookup in an isolated fixed tool."""

from __future__ import annotations

import copy
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "src/act_lab/adapters/mujoco/assets"


def generate() -> dict:
    source = ASSETS / "ur5e/ur5e_2f85.xml"
    tree = ET.parse(source).getroot()
    compiler = tree.find("compiler")
    assert compiler is not None
    compiler.set("meshdir", str(source.parent))
    world = tree.find("worldbody")
    mount = tree.find(".//body[@name='gripper_mount']")
    assert world is not None and mount is not None
    isolated = copy.deepcopy(mount)
    world.clear()
    world.append(isolated)
    actuators = tree.find("actuator")
    assert actuators is not None
    for actuator in list(actuators):
        if actuator.get("name") != "gripper":
            actuators.remove(actuator)
    model = mujoco.MjModel.from_xml_string(ET.tostring(tree).decode())
    data = mujoco.MjData(model)
    names = tuple(model.joint(j).name for j in range(model.njnt))
    samples = []
    for control in np.linspace(0, 255, 21):
        data.actuator("gripper").ctrl[0] = control
        for _ in range(2000):
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        gap = data.site("rq_right_inner_pad").xpos - data.site("rq_left_inner_pad").xpos
        axis = data.body("rq_base").xmat.reshape(3, 3)[:, 1]
        samples.append(
            dict(
                control=float(control),
                aperture_m=float(np.dot(gap, axis)),
                joints={name: float(data.joint(name).qpos[0]) for name in names},
            )
        )
    return dict(
        schema_version=1,
        mujoco_version=mujoco.__version__,
        method="Fixed unloaded tool; 2000 ticks/control; production tool parameters.",
        assembly_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        nominal_aperture_m=0.085,
        samples=samples,
    )


def main() -> None:
    target = ASSETS / "robotiq_2f85/aperture-calibration.json"
    target.write_text(json.dumps(generate(), indent=2, allow_nan=False) + "\n")
    print(target)


if __name__ == "__main__":
    main()
