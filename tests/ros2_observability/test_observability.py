"""Required optional runtime tests: dependencies must fail, never skip."""

import json
import multiprocessing as mp
import time
from pathlib import Path

from act_lab.adapters.ros2.observability import ObservationChannel
from act_lab.adapters.ros2.observation_demo import run_observability
from act_lab.adapters.ros2.simulation import snapshot_state
from act_lab.adapters.ros2.simulation_smoke import MotionSession


def test_generated_messages_roundtrip():
    from act_lab_interfaces.msg import ExecutionEvent, ExecutionTelemetry
    from rclpy.serialization import deserialize_message, serialize_message

    for cls in (ExecutionEvent, ExecutionTelemetry):
        message = cls()
        message.schema_version = 1
        message.episode_id = "test-episode"
        message.reason = "explicit fault"
        assert deserialize_message(serialize_message(message), cls) == message


def test_live_bridge_and_fault_demo(tmp_path):
    result = run_observability(tmp_path, duration=4, smoke=True)
    assert result["bridge"]["telemetry_delivered"]
    assert result["cases"][-1]["reset"]
    assert json.loads((tmp_path / "observer.json").read_text())["samples"] > 0
    rows = json.loads((tmp_path / "physics.json").read_text())
    assert any(r["mode"] == "ENABLED" for r in rows)
    assert any(r["reason"] == "gateway_wall_timeout" for r in rows)
    assert rows[-1]["mode"] == "SHUTDOWN_HOLD"


def _busy_reader(channel, ready):
    channel.lock.acquire()
    ready.set()
    time.sleep(60)


def test_dead_busy_observer_cannot_change_owner_hold(tmp_path):
    channel = ObservationChannel()
    context = mp.get_context("spawn")
    ready = context.Event()
    reader = context.Process(target=_busy_reader, args=(channel, ready))
    reader.start()
    assert ready.wait(5)
    reader.kill()
    reader.join(5)
    session = MotionSession(tmp_path, observation_channel=channel)
    try:
        session.motion_command(snapshot_state(session.value).end_effector_pose)
        time.sleep(0.12)
        value = session.rpc(dict(kind="advance", ticks=60))
        assert value["mode"] == "FAULT_HOLD"
        assert value["reason"] == "gateway_wall_timeout"
        assert channel.dropped.value > 0
    finally:
        session.close()
    assert Path(tmp_path, "physics.json").exists()


def test_model_and_authority_dds_delivery_reset_and_stale(tmp_path):
    import xml.etree.ElementTree as ET

    import numpy as np
    import pinocchio as pin
    import rclpy
    from act_lab_interfaces.msg import ExecutionTelemetry
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import MarkerArray

    from act_lab.adapters.ros2.observation_runtime import (
        ViewerProcesses,
        observer_process,
    )

    channel = ObservationChannel()
    context = mp.get_context("spawn")
    stop = context.Event()
    observer = context.Process(
        target=observer_process, args=(channel, stop, str(tmp_path))
    )
    views = ViewerProcesses(tmp_path, bridge=False)
    observer.start()
    session = MotionSession(tmp_path, observation_channel=channel)
    node = session.node
    best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    static = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    seen = {}
    transforms = {}
    subscriptions = []

    def receive_tf(message):
        transforms.update({t.child_frame_id: t for t in message.transforms})

    for cls, topic, qos in [
        (JointState, "/joint_states", best),
        (ExecutionTelemetry, "/act_lab/view/telemetry", best),
        (DiagnosticArray, "/act_lab/view/health", best),
        (MarkerArray, "/act_lab/view/poses", best),
        (String, "/act_lab/view/robot_description", static),
    ]:
        subscriptions.append(
            node.create_subscription(
                cls, topic, lambda msg, name=topic: seen.update({name: msg}), qos
            )
        )
    for topic, qos in [("/tf", QoSProfile(depth=100)), ("/tf_static", static)]:
        subscriptions.append(
            node.create_subscription(TFMessage, topic, receive_tf, qos)
        )

    def wait_for(predicate, timeout=15):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError(
            f"bounded DDS wait expired: {list(seen)}, {list(transforms)}"
        )

    try:
        session.motion_command(snapshot_state(session.value).end_effector_pose)
        wait_for(
            lambda: (
                len(seen) == 5
                and all(
                    "ur_model/" + f in transforms
                    for f in (
                        "tool0",
                        "base_link",
                        "base_link_inertia",
                        "shoulder_link",
                        "upper_arm_link",
                        "forearm_link",
                        "wrist_1_link",
                        "wrist_2_link",
                        "wrist_3_link",
                    )
                )
                and "act_lab_scene_tip" in transforms
                and "ur_model/world" in transforms
            )
        )
        joints = seen["/joint_states"]
        assert len(joints.name) == len(joints.position) == len(joints.effort) == 6
        model = pin.buildModelFromUrdf("/opt/crisp/ur5e.urdf")
        data = model.createData()
        pin.framesForwardKinematics(model, data, np.asarray(joints.position))
        # Mesh package URIs resolve inside the pinned installation.
        urdf = ET.fromstring(seen["/act_lab/view/robot_description"].data)
        for mesh in urdf.findall(".//mesh"):
            uri = mesh.attrib["filename"]
            assert uri.startswith("package://ur_description/")
            assert Path(
                "/opt/crisp/install/ur_description/share/ur_description",
                uri.removeprefix("package://ur_description/"),
            ).is_file()
        root = transforms["ur_model/world"]
        assert root.header.frame_id == "world"
        assert root.transform.rotation.z == 1.0
        tip = transforms["act_lab_scene_tip"]
        assert tip.header.frame_id == "world"
        assert seen["/act_lab/view/telemetry"].frame_residual_m <= 0.002

        def world_transform(child):
            if child == "world":
                return np.eye(4)
            item = transforms[child]
            q = item.transform.rotation
            p = item.transform.translation
            local = pin.SE3(
                pin.Quaternion(np.array([q.x, q.y, q.z, q.w])).matrix(),
                np.array([p.x, p.y, p.z]),
            ).homogeneous
            return world_transform(item.header.frame_id) @ local

        expected = (
            np.diag([-1.0, -1.0, 1.0, 1.0])
            @ data.oMf[model.getFrameId("tool0")].homogeneous
        )
        actual = world_transform("ur_model/tool0")
        assert np.linalg.norm(actual[:3, 3] - expected[:3, 3]) <= 0.002
        assert np.linalg.norm(pin.log3(actual[:3, :3].T @ expected[:3, :3])) <= 0.02
        old_episode = seen["/act_lab/view/telemetry"].episode_id
        # Force the nonblocking reset handoff to lose its first offer. The
        # idle stepped owner must retry the original snapshot after release.
        channel.lock.acquire()
        try:
            session.rpc(dict(kind="reset", seed=0))
        finally:
            channel.lock.release()
        wait_for(lambda: seen["/act_lab/view/telemetry"].episode_id != old_episode)
        assert not seen["/act_lab/view/telemetry"].source_valid
        # Freeze the authority handoff independently of DDS and its steady heartbeat.
        channel.lock.acquire()
        try:
            wait_for(lambda: seen["/act_lab/view/health"].status[0].level == bytes([2]))
            wait_for(
                lambda: any(
                    "STALE" in m.text for m in seen["/act_lab/view/poses"].markers
                )
            )
            assert all(
                m.id not in (1, 2, 3) for m in seen["/act_lab/view/poses"].markers
            )
        finally:
            channel.lock.release()
        session.motion_command(snapshot_state(session.value).end_effector_pose)
    finally:
        session.close()
        stop.set()
        observer.join(5)
        if observer.is_alive():
            observer.kill()
            observer.join(5)
        views.close()
