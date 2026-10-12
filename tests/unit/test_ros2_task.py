"""Task producer contracts require neither ROS nor generated interfaces."""

import copy
import json
from array import array
from dataclasses import asdict, replace
from uuid import uuid4

import pytest

from act_lab.adapters.ros2.contracts import encode_report, encode_state
from act_lab.adapters.ros2.demonstrator import HOME
from act_lab.adapters.ros2.recording_demo import _expert_task
from act_lab.adapters.ros2.runtime import message_fields
from act_lab.adapters.ros2.task import ExpertTaskSource, GripperRamp, task_state
from act_lab.adapters.ros2.task_qualification import distribution, physics_summary
from act_lab.domain import (
    Action,
    CommandOutcome,
    CommandReport,
    Observation,
    RobotState,
)

STATE = RobotState(0, (0.0,) * 6, (0.0,) * 6, HOME, 0.0)


def test_approach_route_uses_measured_waypoints_without_renewing_intent():
    from act_lab.adapters.ros2.task import ApproachRoute

    pose = replace(HOME, position_xyz_m=(0.52, -0.19, 0.62))
    state = replace(STATE, end_effector_pose=pose)
    observation = Observation(0, state, ())
    action = Action(19, replace(pose, position_xyz_m=(0.52, -0.19, 0.455)), 1.0, True)
    route = ApproachRoute(0.46, 0.56)
    inward = route.apply("approach_cube", action, observation)
    assert inward.target_pose.position_xyz_m == (0.46, -0.19, 0.62)
    assert inward.timestamp_ns == 19
    assert route.apply("approach_cube", action, observation) == inward
    measured = replace(state, end_effector_pose=inward.target_pose)
    lower = route.apply("approach_cube", action, Observation(0, measured, ()))
    assert lower.target_pose.position_xyz_m == (0.46, -0.19, 0.56)
    measured = replace(state, end_effector_pose=lower.target_pose)
    assert route.apply("approach_cube", action, Observation(0, measured, ())) == action
    assert route.stage == "final"
    disabled = replace(action, enabled=False)
    assert route.apply("approach_cube", disabled, observation) == disabled
    assert route.stage == "inactive"
    assert ApproachRoute().apply("approach_cube", action, observation) == action
    assert route.apply("close_dwell", action, observation) == action


def test_approach_route_rejects_incomplete_or_nonfinite_configuration():
    from act_lab.adapters.ros2.task import ApproachRoute

    with pytest.raises(ValueError, match="both"):
        ApproachRoute(0.46)
    with pytest.raises(ValueError, match="finite"):
        ApproachRoute(float("nan"), 0.56)


def snapshot():
    return dict(
        episode=str(uuid4()),
        task_timestamp_ns=STATE.timestamp_ns,
        cube_gripper_contacts=0,
        state=encode_state(STATE, str(uuid4())),
        task=dict(
            cube_pose=asdict(STATE.end_effector_pose),
            desired_cube_pose=asdict(STATE.end_effector_pose),
            success=False,
            terminal=False,
            reason=None,
        ),
    )


def test_task_snapshot_is_separate_and_requires_coherent_time():
    value = snapshot()
    source = ExpertTaskSource()
    observation = Observation(STATE.timestamp_ns, STATE, ())
    source.update(value, observation)
    assert source.task_state().cube_pose == STATE.end_effector_pose
    assert not hasattr(observation, "task")
    with pytest.raises(ValueError, match="timestamps"):
        source.update(value, replace(observation, timestamp_ns=STATE.timestamp_ns + 1))


@pytest.mark.parametrize("change", ["nan", "frame", "flag", "terminal", "episode"])
def test_malformed_task_snapshot(change):
    value = snapshot()
    if change == "nan":
        value["task"]["cube_pose"]["position_xyz_m"] = [float("nan"), 0.0, 0.0]
    elif change == "frame":
        value["task"]["cube_pose"]["frame_id"] = "camera"
    elif change == "flag":
        value["task"]["terminal"] = 1
    elif change == "terminal":
        value["task"]["success"] = True
    else:
        value["episode"] = "not-a-uuid"
    with pytest.raises(ValueError):
        task_state(value, STATE.timestamp_ns)


def test_gripper_ramp_uses_source_time_and_preserves_intent_identity():
    observation = Observation(
        STATE.timestamp_ns, replace(STATE, gripper_position=1.0), ()
    )
    ramp = GripperRamp(observation)
    action = Action(
        STATE.timestamp_ns + 100_000_000, STATE.end_effector_pose, 0.0, True
    )
    approved = ramp.apply(action)
    assert approved.gripper_position == pytest.approx(0.99)
    assert approved.timestamp_ns == action.timestamp_ns
    assert ramp.apply(action) == approved  # reuse does not progress the ramp
    disabled = replace(action, enabled=False, gripper_position=0.3)
    assert ramp.apply(disabled) == disabled
    with pytest.raises(ValueError, match="clock reversed"):
        ramp.apply(replace(action, timestamp_ns=STATE.timestamp_ns))


def test_opening_hold_keeps_measured_pose_without_refreshing_intent():
    from act_lab.adapters.ros2.task import OpeningPoseHold

    opening = OpeningPoseHold()
    observation = Observation(0, STATE, ())
    action = Action(17, replace(HOME, position_xyz_m=(0.5, 0.1, 0.4)), 1.0, True)
    approved = opening.apply("open_dwell", action, observation)
    assert approved.target_pose == HOME
    assert approved.timestamp_ns == 17
    moved = replace(STATE, end_effector_pose=action.target_pose)
    assert (
        opening.apply(
            "open_dwell", replace(action, timestamp_ns=18), Observation(1, moved, ())
        ).target_pose
        == HOME
    )
    disabled = replace(action, enabled=False)
    assert opening.apply("open_dwell", disabled, observation) == disabled
    assert opening.apply("open_dwell", action, Observation(1, moved, ())) == action


class Fields:
    def __init__(self, **values):
        self.__dict__.update(values)

    def get_fields_and_field_types(self):
        return {name: "unused" for name in self.__dict__}


def test_primitive_conversion_preserves_rgb_without_integer_expansion():
    pixels = array("B", [0, 255, 128])
    message = Fields(images=[Fields(data=pixels, width=1)], q=array("d", [1.0, 2.0]))
    fields = message_fields(message)
    assert fields["images"][0]["data"] == pixels.tobytes()
    assert fields["q"] == [1.0, 2.0]
    assert fields["images"][0]["width"] == 1


@pytest.mark.parametrize("result", ["success", "invalid", "timeout"])
def test_expert_uses_actual_task_and_preserves_source_timestamp(
    tmp_path, monkeypatch, result
):
    from pathlib import Path

    from act_lab.adapters.mujoco.config import SimulationConfig

    monkeypatch.setattr(
        "act_lab.adapters.ros2.recording_demo.time.sleep", lambda _: None
    )
    settings = SimulationConfig.load(Path("configs/sim/ur5e_2f85_d405.toml"))
    value = snapshot()
    value["state"] = encode_state(STATE, value["episode"])

    class Session:
        output = tmp_path

        def __init__(self):
            self.value = copy.deepcopy(value)
            self.calls = 0

        def rpc(self, request):
            return self.value

        def expert_trace_path(self):
            return self.output / "expert.jsonl"

        def command(self, pose, gripper, enabled, *, stamp):
            assert stamp == self.value["task_timestamp_ns"]
            self.calls += 1
            state = replace(
                STATE,
                timestamp_ns=stamp + 20_000_000,
                gripper_position=1.0 if result == "success" else 0.0,
            )
            action = Action(stamp, pose, gripper, enabled)
            outcome = (
                CommandOutcome.INVALID
                if result == "invalid"
                else CommandOutcome.APPLIED
            )
            report = CommandReport(outcome, action, action, state, "test")
            self.value.update(
                state=encode_state(state, value["episode"]),
                report=encode_report(report, value["episode"], self.calls),
                mode="ENABLED",
                reason="authorized",
                task_timestamp_ns=state.timestamp_ns,
            )
            if result == "success":
                self.value["task"].update(success=True, terminal=True, reason="success")
            return self.value

    session = Session()
    task = _expert_task(session, Path("configs/sim/ur5e_2f85_d405.toml"), 2)
    assert task["success"] == (result == "success")
    assert (
        task["reason"]
        == {"success": "success", "invalid": "invalid", "timeout": "step_limit"}[result]
    )
    assert (tmp_path / "expert.jsonl").exists()
    assert settings.episode_steps == 750


def test_physics_summary_rejects_nonzero_task_effort_in_hold(tmp_path):
    row = dict(
        state=asdict(STATE), mode="FAULT_HOLD", reason="loss", task_effort_nm=[0.1] * 6
    )
    path = tmp_path / "physics.json"
    path.write_text(json.dumps([row]))
    summary = physics_summary(path)
    assert not summary["zero_task_effort_in_hold"]
    assert distribution([1, 2, 100])["maximum"] == 100
    with pytest.raises(ValueError, match="empty"):
        distribution([])


def test_transport_retries_are_identical_and_do_not_advance_sequence():
    from act_lab.adapters.ros2.recording_contracts import PacketReceipt

    receipts = PacketReceipt()
    episode = str(uuid4())
    assert receipts.accept(episode, 1, b"source packet")
    assert not receipts.accept(episode, 1, b"source packet")
    assert receipts.sequence == 1
    assert receipts.duplicates == 1
    with pytest.raises(ValueError, match="conflicting"):
        receipts.accept(episode, 1, b"modified timestamp")
    with pytest.raises(ValueError, match="conflicting"):
        receipts.accept(str(uuid4()), 1, b"source packet")
    assert receipts.accept(episode, 2, b"next packet")
    with pytest.raises(ValueError, match="gap/replay"):
        receipts.accept(episode, 1, b"source packet")


def test_gateway_uses_configured_control_period(tmp_path):
    from pathlib import Path

    from act_lab.adapters.mujoco import MujocoCartesianDriver
    from act_lab.adapters.ros2.simulation_gateway import RosSimulationDriver

    config = Path("configs/sim/ur5e_2f85_d405_crisp_task_bench.toml")
    with MujocoCartesianDriver.from_config_file(config) as local:
        observation = local.reset(0)

        class Runtime:
            config_path = config
            requests = []
            snapshot = dict(
                state=encode_state(observation.robot, str(uuid4())),
                qpos=local.environment._data.qpos.tolist(),
                qvel=local.environment._data.qvel.tolist(),
                environment_steps=0,
            )

            def rpc(self, request):
                self.requests.append(request)

        runtime = Runtime()
        driver = RosSimulationDriver(runtime)
        try:
            driver.step(observation.robot.joint_positions_rad, (0.0,) * 6, 0.5)
            assert driver.control_period_s == 0.04
            assert runtime.requests[-1]["ticks"] == 20
        finally:
            driver.close()


def test_settled_but_pinched_cube_is_not_complete_task():
    value = snapshot()
    value["task"].update(success=True, terminal=True, reason="success")
    value["cube_gripper_contacts"] = 2
    observation = Observation(
        STATE.timestamp_ns, replace(STATE, gripper_position=0.6), ()
    )
    source = ExpertTaskSource(require_release=True)
    source.update(value, observation)
    assert source.owner_task_state().success
    assert not source.task_state().terminal
    assert not source.released
    value["cube_gripper_contacts"] = 0
    source.update(
        value, replace(observation, robot=replace(STATE, gripper_position=1.0))
    )
    assert source.task_state().success and source.released
    del value["cube_gripper_contacts"]
    source.update(
        value, replace(observation, robot=replace(STATE, gripper_position=1.0))
    )
    assert not source.released  # absent contact evidence fails closed
    value["task"].update(success=False, reason="timeout")
    source.update(value, observation)
    assert source.task_state().terminal  # failure is never masked for opening


def test_loaded_transit_lookahead_preserves_vertical_and_intent_identity():
    from act_lab.adapters.ros2.task import loaded_transit_action

    pose = replace(HOME, position_xyz_m=(0.7, 0.4, 0.55))
    action = Action(STATE.timestamp_ns, pose, 0.2, True)
    observation = Observation(STATE.timestamp_ns, STATE, ())
    shaped = loaded_transit_action(action, observation, 0.01)
    import math

    assert math.dist(
        shaped.target_pose.position_xyz_m[:2], HOME.position_xyz_m[:2]
    ) == pytest.approx(0.01)
    assert shaped.target_pose.position_xyz_m[2] == 0.55
    assert (shaped.timestamp_ns, shaped.gripper_position, shaped.enabled) == (
        action.timestamp_ns,
        0.2,
        True,
    )


def test_descent_preserves_intent_identity_and_limits_downward_lookahead():
    from act_lab.adapters.ros2.task import descent_action

    observation = Observation(0, STATE, ())
    target = replace(HOME, position_xyz_m=(0.5, 0.2, 0.44))
    action = Action(0, target, 0.3, True)
    bounded = descent_action(action, observation, 0.0012)
    import math

    assert (
        math.dist(bounded.target_pose.position_xyz_m[:2], HOME.position_xyz_m[:2])
        <= 0.0006 + 1e-12
    )
    assert bounded.target_pose.position_xyz_m[2] == max(
        0.44, HOME.position_xyz_m[2] - 0.0012
    )
    assert bounded.timestamp_ns == action.timestamp_ns
    assert bounded.target_pose.quaternion_wxyz == action.target_pose.quaternion_wxyz
    assert bounded.gripper_position == action.gripper_position
    assert bounded.enabled == action.enabled
    upward = replace(action, target_pose=replace(HOME, position_xyz_m=(0.5, 0.2, 0.9)))
    assert descent_action(upward, observation, 0.0012) == upward
    with pytest.raises(ValueError):
        descent_action(action, observation, float("nan"))
