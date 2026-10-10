"""Compose a URDF tool tree from the same compiled geometry as the simulator."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import Any

import mujoco  # type: ignore[import-untyped]
import numpy as np
from numpy.typing import NDArray


def rotation(quaternion: Any) -> NDArray[np.float64]:
    value = np.empty(9)
    mujoco.mju_quat2Mat(value, np.asarray(quaternion))
    return value.reshape(3, 3)


def numbers(values: Any) -> str:
    return " ".join(format(float(value), ".17g") for value in values)


def rpy(matrix: NDArray[np.float64]) -> tuple[float, float, float]:
    pitch = math.atan2(-matrix[2, 0], math.hypot(matrix[0, 0], matrix[1, 0]))
    if abs(math.cos(pitch)) < 1e-10:
        return (math.atan2(-matrix[1, 2], matrix[1, 1]), pitch, 0.0)
    return (
        math.atan2(matrix[2, 1], matrix[2, 2]),
        pitch,
        math.atan2(matrix[1, 0], matrix[0, 0]),
    )


def origin(
    element: ET.Element, position: Any, orientation: NDArray[np.float64]
) -> None:
    ET.SubElement(
        element, "origin", xyz=numbers(position), rpy=numbers(rpy(orientation))
    )


def compose_tool_urdf(base_urdf: str, model: Any, *, rigid_data: Any = None) -> str:
    """Append a tool0 tree; optionally freeze linkage for CRISP's six DOFs.

    Visualization uses each measured linkage angle, without inventing mimic laws.
    The rigid control variant captures the same masses at an explicit nominal
    linkage pose; it is an approximation, not articulated inverse dynamics.
    """
    tree = ET.fromstring(
        base_urdf, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    )
    if tree.find("link[@name='tool0']") is None:
        raise ValueError("base URDF must provide tool0")
    root_id = int(model.body("gripper_mount").id)
    bodies = {root_id}
    for body in range(root_id + 1, model.nbody):
        if int(model.body_parentid[body]) in bodies:
            bodies.add(body)
    pivots: dict[int, NDArray[np.float64]] = {}
    for body in sorted(bodies):
        if model.body_jntnum[body] > 1:
            raise ValueError("URDF tool export requires at most one joint per body")
        joint_id = int(model.body_jntadr[body])
        moving = joint_id >= 0 and rigid_data is None
        pivot = model.jnt_pos[joint_id].copy() if moving else np.zeros(3)
        pivots[body] = pivot
        name = model.body(body).name
        link = ET.SubElement(tree, "link", name=name)
        if model.body_mass[body] > 0:
            inertial = ET.SubElement(link, "inertial")
            # URDF diagonalization is unnecessary; retain full body-frame tensor.
            origin(inertial, model.body_ipos[body] - pivot, np.eye(3))
            ET.SubElement(
                inertial, "mass", value=format(float(model.body_mass[body]), ".17g")
            )
            axes = rotation(model.body_iquat[body])
            inertia = axes @ np.diag(model.body_inertia[body]) @ axes.T
            ET.SubElement(
                inertial,
                "inertia",
                {
                    key: format(float(inertia[i, j]), ".17g")
                    for key, i, j in (
                        ("ixx", 0, 0),
                        ("ixy", 0, 1),
                        ("ixz", 0, 2),
                        ("iyy", 1, 1),
                        ("iyz", 1, 2),
                        ("izz", 2, 2),
                    )
                },
            )
        for geom_id in range(
            int(model.body_geomadr[body]),
            int(model.body_geomadr[body] + model.body_geomnum[body]),
        ):
            visual = (
                not model.geom_contype[geom_id] and not model.geom_conaffinity[geom_id]
            )
            element = ET.SubElement(link, "visual" if visual else "collision")
            orientation = rotation(model.geom_quat[geom_id])
            position = model.geom_pos[geom_id].copy()
            geometry = ET.SubElement(element, "geometry")
            kind = int(model.geom_type[geom_id])
            if kind == mujoco.mjtGeom.mjGEOM_MESH:
                mesh = int(model.geom_dataid[geom_id])
                # Undo MuJoCo's mesh centering/principal-axis preprocessing.
                orientation = orientation @ rotation(model.mesh_quat[mesh]).T
                position -= orientation @ model.mesh_pos[mesh]
                mesh_name = model.mesh(mesh).name.removeprefix("rq_")
                ET.SubElement(
                    geometry,
                    "mesh",
                    filename=f"package://act_lab_tool_assets/{mesh_name}.stl",
                    scale=numbers(model.mesh_scale[mesh]),
                )
            elif kind == mujoco.mjtGeom.mjGEOM_BOX:
                ET.SubElement(
                    geometry, "box", size=numbers(2 * model.geom_size[geom_id])
                )
            else:
                raise ValueError(f"unsupported tool geometry: {kind}")
            origin(element, position - pivot, orientation)
            if visual:
                material = ET.SubElement(element, "material", name=f"{name}_{geom_id}")
                material_id = int(model.geom_matid[geom_id])
                rgba = (
                    model.mat_rgba[material_id]
                    if material_id >= 0
                    else model.geom_rgba[geom_id]
                )
                ET.SubElement(material, "color", rgba=numbers(rgba))
        parent = int(model.body_parentid[body])
        if body == root_id:
            parent_name = "tool0"
            position, orientation = np.zeros(3), np.eye(3)
        elif rigid_data is not None:
            parent_name = model.body(parent).name
            orientation = rigid_data.xmat[parent].reshape(3, 3).T @ rigid_data.xmat[
                body
            ].reshape(3, 3)
            position = rigid_data.xmat[parent].reshape(3, 3).T @ (
                rigid_data.xpos[body] - rigid_data.xpos[parent]
            )
        else:
            parent_name = model.body(parent).name
            orientation = rotation(model.body_quat[body])
            position = model.body_pos[body] + orientation @ pivot - pivots[parent]
        joint = ET.SubElement(
            tree,
            "joint",
            name=model.joint(joint_id).name if moving else f"{name}_fixed",
            type="revolute" if moving else "fixed",
        )
        ET.SubElement(joint, "parent", link=parent_name)
        ET.SubElement(joint, "child", link=name)
        origin(joint, position, orientation)
        if moving:
            if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError("tool joints must be hinges")
            ET.SubElement(joint, "axis", xyz=numbers(model.jnt_axis[joint_id]))
            lower, upper = model.jnt_range[joint_id]
            ET.SubElement(
                joint,
                "limit",
                lower=str(lower),
                upper=str(upper),
                effort="5",
                velocity="2",
            )
    ET.SubElement(tree, "link", name="grasp_tcp")
    site = int(model.site("end_effector").id)
    body = int(model.site_bodyid[site])
    joint = ET.SubElement(tree, "joint", name="grasp_tcp_fixed", type="fixed")
    ET.SubElement(joint, "parent", link=model.body(body).name)
    ET.SubElement(joint, "child", link="grasp_tcp")
    origin(joint, model.site_pos[site] - pivots[body], rotation(model.site_quat[site]))
    ET.indent(tree, space="  ")
    return ET.tostring(tree, encoding="unicode") + "\n"
