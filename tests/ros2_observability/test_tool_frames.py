"""Independent Pinocchio FK checks for every measured tool joint and the camera."""

from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin

from act_lab.adapters.mujoco.environment import MujocoUR5eEnvironment
from act_lab.adapters.mujoco.tool_description import compose_tool_urdf
from act_lab.adapters.ros2.contracts import JOINT_NAMES


def test_articulated_urdf_matches_actual_tool_fk_and_rigid_model_has_six_dofs():
    base = Path("/opt/crisp/ur5e.urdf").read_text()
    with MujocoUR5eEnvironment.from_config_file(
        Path("configs/sim/ur5e_2f85_d405.toml")
    ) as env:
        env.reset(0)
        model = pin.buildModelFromXML(compose_tool_urdf(base, env._model))
        data = model.createData()
        assert model.nq == 14
        names = (*JOINT_NAMES, *env.gripper.joint_names)
        for aperture in (0, 0.25, 0.5, 0.75, 1, None):
            if aperture is None:
                from act_lab.adapters.mujoco.cartesian_driver import (
                    MujocoCartesianDriver,
                )
                from act_lab.application import (
                    SafeCartesianRobot,
                    ScriptedPickPlaceExpert,
                )
                from act_lab.application.scripted_expert import ExpertPhase

                driver = MujocoCartesianDriver(env, env._config)
                robot = SafeCartesianRobot(driver, driver.limits)
                observation = robot.reset(0)
                expert = ScriptedPickPlaceExpert(env, env._config.expert)
                expert.reset(observation)
                for _ in range(750):
                    robot.command(expert.poll(observation))
                    observation = robot.observe()
                    if expert.phase is ExpertPhase.TRANSIT:
                        break
                assert expert.phase is ExpertPhase.TRANSIT
                assert env.gripper.measured(env._data) > 0.5
            else:
                env.gripper.set_kinematic(env._data, aperture)
                mujoco.mj_forward(env._model, env._data)
            q = pin.neutral(model)
            for name in names:
                q[model.joints[model.getJointId(name)].idx_q] = env._data.joint(
                    name
                ).qpos[0]
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            actual_mount = data.oMf[model.getFrameId("gripper_mount")]
            source_mount = pin.SE3(
                env._data.body("gripper_mount").xmat.reshape(3, 3),
                env._data.body("gripper_mount").xpos,
            )
            for body in range(env._model.nbody):
                name = env._model.body(body).name
                frame = model.getFrameId(name)
                if not (name.startswith("rq_") or name.startswith("wrist_camera")):
                    continue
                expected = pin.SE3(
                    env._data.xmat[body].reshape(3, 3), env._data.xpos[body]
                )
                joint = int(env._model.body_jntadr[body])
                if joint >= 0:
                    expected.translation += (
                        expected.rotation @ env._model.jnt_pos[joint]
                    )
                expected = source_mount.inverse() * expected
                observed = actual_mount.inverse() * data.oMf[frame]
                assert (
                    np.linalg.norm(expected.translation - observed.translation) < 1e-9
                )
                assert (
                    np.linalg.norm(pin.log3(expected.rotation @ observed.rotation.T))
                    < 1e-9
                ), name
        rigid = pin.buildModelFromXML(
            compose_tool_urdf(base, env._model, rigid_data=env._data)
        )
        assert rigid.nq == rigid.nv == 6
