"""Compose pinned UR5e/2F-85 assets and a primitive wrist camera, deterministically."""

from __future__ import annotations

import copy
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "src/act_lab/adapters/mujoco/assets"


def assemble() -> dict[Path, bytes]:
    robot = ET.parse(ASSETS / "ur5e/ur5e.xml").getroot()
    robot.set("model", "ACT Lab UR5e 2F85 D405 RGB")
    compiler = robot.find("compiler")
    assert compiler is not None
    compiler.set("meshdir", ".")
    option = robot.find("option")
    assert option is not None
    option.set("cone", "elliptic")
    option.set("impratio", "10")
    for mesh in robot.findall("asset/mesh"):
        filename = mesh.attrib["file"]
        mesh.set("name", Path(filename).stem)
        mesh.set("file", "assets/" + filename)
    wrist = robot.find(".//body[@name='wrist_3_link']")
    assert wrist is not None
    palm = wrist.find("body[@name='gripper_palm']")
    assert palm is not None
    wrist.remove(palm)
    for tag in ("equality", "keyframe"):
        element = robot.find(tag)
        assert element is not None
        robot.remove(element)
    actuators = robot.find("actuator")
    assert actuators is not None
    gripper = actuators.find("position[@name='gripper']")
    assert gripper is not None
    actuators.remove(gripper)

    source = ET.parse(ASSETS / "robotiq_2f85/2f85.xml").getroot()
    references = {
        "name",
        "class",
        "childclass",
        "mesh",
        "material",
        "body1",
        "body2",
        "joint",
        "joint1",
        "joint2",
        "tendon",
    }
    for mesh in source.findall("asset/mesh"):
        mesh.set("name", Path(mesh.attrib["file"]).stem)
        mesh.set("file", "../robotiq_2f85/assets/" + mesh.attrib["file"])
    for element in source.iter():
        for attribute in references & element.attrib.keys():
            element.set(attribute, "rq_" + element.attrib[attribute])
    defaults = robot.find("default")
    assert defaults is not None
    for child in source.findall("default/default"):
        defaults.append(copy.deepcopy(child))
    asset = robot.find("asset")
    assert asset is not None
    for child in source.findall("asset/*"):
        asset.append(copy.deepcopy(child))

    mount = ET.SubElement(
        wrist, "body", name="gripper_mount", pos="0 0.1 0", quat="-1 1 0 0"
    )
    root = source.find("worldbody/body")
    assert root is not None
    mount.append(copy.deepcopy(root))
    for body in mount.iter("body"):
        body.set("gravcomp", "1")
    pinch = mount.find(".//site[@name='rq_pinch']")
    assert pinch is not None
    # TCP orientation is the flange orientation; the upstream base rotates Z.
    pinch.set("name", "end_effector")
    pinch.set("quat", "1 0 0 1")
    for side in ("left", "right"):
        pad = mount.find(f".//body[@name='rq_{side}_pad']")
        assert pad is not None
        ET.SubElement(
            pad,
            "site",
            name=f"rq_{side}_inner_pad",
            pos="0 -0.0066 0.01875",
            size="0.001",
            group="5",
        )
    camera = ET.SubElement(
        mount,
        "body",
        name="wrist_camera_link",
        pos="0 0.065 0.025",
        quat="0.9781476007338057 0.20791169081775934 0 0",
        gravcomp="1",
    )
    ET.SubElement(
        camera,
        "geom",
        name="wrist_camera_housing",
        type="box",
        size="0.021 0.021 0.0115",
        mass="0.06",
        rgba="0.2 0.22 0.24 1",
    )
    ET.SubElement(
        camera,
        "geom",
        name="wrist_camera_bracket",
        type="box",
        pos="0 -0.024 -0.013",
        size="0.008 0.024 0.002",
        mass="0.04",
        rgba="0.3 0.3 0.3 1",
    )
    optical = ET.SubElement(
        camera, "body", name="wrist_camera_optical_frame", pos="0 0 0.0115"
    )
    ET.SubElement(optical, "site", name="wrist_optical", size="0.001", group="5")
    # MuJoCo looks down -Z with +Y up; optical frames use +Z forward, +Y down.
    ET.SubElement(optical, "camera", name="wrist", quat="0 1 0 0", fovy="58")

    for tag in ("contact", "tendon", "equality"):
        element = source.find(tag)
        assert element is not None
        robot.append(copy.deepcopy(element))
    for actuator in source.findall("actuator/*"):
        actuator.set("name", "gripper")
        actuators.append(copy.deepcopy(actuator))
    scene = ET.parse(ASSETS / "ur5e/act_lab_scene.xml").getroot()
    scene.set("model", "ACT Lab UR5e 2F85 D405 cube-to-tray task")
    include = scene.find("include")
    assert include is not None
    include.set("file", "ur5e_2f85.xml")
    contact = scene.find("contact")
    assert contact is not None
    scene.remove(contact)  # Old educational finger exclusions do not apply.
    result = {}
    for name, tree in (("ur5e_2f85.xml", robot), ("act_lab_2f85_scene.xml", scene)):
        ET.indent(tree, space="  ")
        result[ASSETS / "ur5e" / name] = ET.tostring(tree, encoding="utf-8") + b"\n"
    return result


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    for path, content in assemble().items():
        if args.check:
            if not path.exists() or path.read_bytes() != content:
                raise SystemExit(f"generated assembly differs: {path.name}")
        else:
            path.write_bytes(content)
    source = ASSETS / "robotiq_2f85"
    manifest = dict(
        schema_version=1,
        repository="https://github.com/google-deepmind/mujoco_menagerie",
        revision="0059d4335f8156206f63a35662313385f7ad6d74",
        source_directory="robotiq_2f85",
        license="BSD-2-Clause",
        files={
            str(path.relative_to(source)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(source.rglob("*"))
            if path.is_file()
            and path.name not in {"manifest.json", "aperture-calibration.json"}
        },
        modifications=[
            "Upstream files retained byte-for-byte; assembly is generated separately.",
            "Prefix gripper names/classes; flange attachment; retain source contacts.",
            "Preserve scene body gravity compensation for gripper and camera.",
            "Add grasp TCP, inner-pad sites and original primitive camera/bracket.",
            "58 degree vertical FOV; RGB aspect ratio defines horizontal FOV.",
        ],
    )
    content = (json.dumps(manifest, indent=2) + "\n").encode()
    target = source / "manifest.json"
    if args.check:
        if target.read_bytes() != content:
            raise SystemExit("source asset manifest differs")
    else:
        target.write_bytes(content)


if __name__ == "__main__":
    main()
