"""Actual articulated tool, contact grasp and calibrated headless camera checks."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np
import pytest

from act_lab.adapters.mujoco.cartesian_driver import MujocoCartesianDriver
from act_lab.adapters.mujoco.environment import ActuatorTargets
from act_lab.adapters.mujoco.tool_description import compose_tool_urdf
from act_lab.application import SafeCartesianRobot
from act_lab.application.scripted_expert import ExpertPhase, ScriptedPickPlaceExpert
from act_lab.domain import Action

CONFIG = Path("configs/sim/ur5e_2f85_d405.toml")


@pytest.fixture
def driver():
    value = MujocoCartesianDriver.from_config_file(CONFIG)
    value.reset(0)
    yield value
    value.close()


def test_named_binding_and_monotonic_measured_aperture(driver):
    env = driver.environment
    assert env.binding.model_id == "ur5e_2f85_d405_v1"
    assert env._model.nu == 13  # Six position, six velocity, one tendon actuator.
    assert len(env.gripper.joint_names) == 8
    measured = []
    controls = []
    for aperture in (0, 0.25, 0.5, 0.75, 1):
        env.gripper.set_kinematic(env._data, aperture)
        mujoco.mj_forward(env._model, env._data)
        measured.append(env.gripper.measured(env._data))
        controls.append(env.gripper.control(aperture))
    assert all(a < b for a, b in zip(measured[:-1], measured[1:], strict=True))
    assert all(a > b for a, b in zip(controls[:-1], controls[1:], strict=True))
    assert np.allclose(measured, (0, 0.25, 0.5, 0.75, 1), atol=0.012)
    assert env.binding.tool_translation_m == (0, 0, 0.1558)
    for value in (float("nan"), -0.01, 1.01):
        with pytest.raises(ValueError):
            env.gripper.control(value)


def test_urdf_tree_contains_articulated_linkage_and_camera(driver):
    base = '<robot name="fixture"><link name="tool0"/></robot>'
    tree = ET.fromstring(compose_tool_urdf(base, driver._model))
    links = {link.attrib["name"] for link in tree.findall("link")}
    parents = {}
    for joint in tree.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        assert parent in links and child in links
        assert child not in parents
        parents[child] = parent
        assert joint.find("mimic") is None
    assert set(parents) == links - {"tool0"}
    for link in links - {"tool0"}:
        ancestors = set()
        while link != "tool0":
            assert link not in ancestors
            ancestors.add(link)
            link = parents[link]
    assert {"wrist_camera_link", "wrist_camera_optical_frame", "grasp_tcp"} <= links
    assert len(tree.findall("joint[@type='revolute']")) == 8
    fixed = ET.fromstring(
        compose_tool_urdf(base, driver._model, rigid_data=driver._data)
    )
    assert not fixed.findall("joint[@type='revolute']")


def test_wrist_rgb_calibration_projection_and_no_physics_advance(driver):
    env = driver.environment
    profile = env.camera_calibration("wrist")
    assert profile["extrinsics"]["parent_frame"] == "gripper_mount"
    assert np.allclose(
        profile["extrinsics"]["position_xyz_m"], [0, 0.060322528, 0.035505774]
    )
    assert np.allclose(
        profile["extrinsics"]["quaternion_wxyz"],
        [math.cos(math.radians(12)), math.sin(math.radians(12)), 0, 0],
    )
    before = env.observe()
    qpos = env._data.qpos.copy()
    image = env.render("wrist")
    assert image.shape == (240, 320, 3) and image.dtype == np.uint8
    assert env.observe() == before and np.array_equal(qpos, env._data.qpos)
    assert image.std() > 10
    pose = env.camera_world_pose("wrist")
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, np.asarray(pose.quaternion_wxyz))
    point = rotation.reshape(3, 3).T @ (
        np.asarray(env.cube_position_xyz_m) - np.asarray(pose.position_xyz_m)
    )
    k = np.asarray(profile["k"]).reshape(3, 3)
    pixel = k @ point / point[2]
    assert point[2] > 0 and 0 < pixel[0] < 320 and 0 < pixel[1] < 240
    x, y = round(pixel[0]), round(pixel[1])
    region = image[max(0, y - 8) : y + 9, max(0, x - 8) : x + 9].astype(float)
    assert np.any((region[:, :, 0] > region[:, :, 1] * 1.5) & (region[:, :, 0] > 70))
    env.step(ActuatorTargets(env.neutral_targets.joint_positions_rad, 0.5))
    assert env.camera_calibration("wrist") == profile


def prepare_grasp(driver):
    robot = SafeCartesianRobot(driver, driver.limits)
    observation = robot.reset(0)
    expert = ScriptedPickPlaceExpert(
        driver.environment, driver.simulation_config.expert
    )
    expert.reset(observation)
    for _ in range(500):
        robot.command(expert.poll(observation))
        expert.record_command_report(robot.last_report)
        observation = robot.observe()
        if expert.phase is ExpertPhase.TRANSIT:
            return robot, observation
    pytest.fail(f"fixture failed to lift: {expert.phase}, {expert.failure_reason}")


@pytest.mark.parametrize("fault", ["disabled", "stale", "future", "frame"])
def test_contact_lift_and_two_second_fault_hold_preserve_grasp(driver, fault):
    robot, observation = prepare_grasp(driver)
    env = driver.environment
    cube = env.cube_position_xyz_m
    assert cube[2] >= driver.simulation_config.cube_center_z_m + 0.05
    accepted = robot.last_report.executed_action.gripper_position
    assert observation.robot.gripper_position > 0.5  # The cube blocks closed intent.
    # A close, loaded grasp must retain a useful image, not only a home view.
    image = env.render("wrist").astype(float)
    red = (image[:, :, 0] > image[:, :, 1] * 1.5) & (image[:, :, 0] > 70)
    rows, columns = np.where(red)
    assert len(rows) > 200
    assert rows.min() > 5 and rows.max() < 235
    assert columns.min() > 5 and columns.max() < 315
    before_joints = env.gripper.joints(env._data)
    assert driver.collision_free(observation.robot.joint_positions_rad, 0)
    assert env.gripper.joints(driver._scratch) == before_joints
    for _ in range(100):
        pose = observation.robot.end_effector_pose
        if fault == "frame":
            pose = replace(pose, frame_id="camera")
        stamp = observation.timestamp_ns
        if fault == "stale":
            stamp -= 100_000_000
        elif fault == "future":
            stamp += 1
        robot.command(
            Action(
                stamp,
                pose,
                1.0,
                fault != "disabled",
            )
        )
        observation = robot.observe()
        assert env._data.actuator("gripper").ctrl[0] == env.gripper.control(accepted)
        assert math.dist(cube, env.cube_position_xyz_m) < 0.005
    assert env.physics_ticks > 1000


def test_new_model_reset_repeats_state_and_identity(driver):
    first = driver.reset(17)
    identity = driver.environment.model_identity
    driver.environment.step()
    assert driver.reset(17) == first
    assert driver.environment.model_identity == identity
    assert identity["model_id"] == "ur5e_2f85_d405_v1"
