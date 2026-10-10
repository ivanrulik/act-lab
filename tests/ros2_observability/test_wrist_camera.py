"""Required generated camera serialization and separate-process DDS delivery."""

import json
import multiprocessing as mp
import time
from pathlib import Path
from uuid import uuid4

import rclpy
from act_lab_interfaces.msg import CameraSample
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message

from act_lab.adapters.mujoco.environment import MujocoUR5eEnvironment
from act_lab.adapters.ros2.camera_runtime import camera_process
from act_lab.adapters.ros2.observability import ObservationChannel
from act_lab.adapters.ros2.observation_runtime import observer_process


def test_generated_wrist_frames_use_captured_state_and_episode(tmp_path):
    config = Path("configs/sim/ur5e_2f85_d405.toml")
    channel = ObservationChannel(camera=True)
    context = mp.get_context("spawn")
    stop = context.Event()
    worker = context.Process(
        target=camera_process, args=(channel.camera, stop, str(tmp_path), config)
    )
    observer_stop = context.Event()
    observer = context.Process(
        target=observer_process, args=(channel, observer_stop, str(tmp_path))
    )
    rclpy.init()
    node = rclpy.create_node("act_lab_camera_contract_test")
    samples = []
    node.create_subscription(
        CameraSample,
        "/act_lab/view/wrist/sample",
        samples.append,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    health = []
    node.create_subscription(
        DiagnosticArray,
        "/act_lab/view/health",
        health.append,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    observer.start()
    worker.start()
    try:
        with MujocoUR5eEnvironment.from_config_file(config) as env:
            env.reset(0)
            episode = str(uuid4())
            sequence = 0
            end = time.monotonic() + 12
            while time.monotonic() < end and len(samples) < 3:
                sequence += 1
                env.step()
                channel.camera.offer(
                    dict(
                        schema_version=1,
                        model_id=env.binding.model_id,
                        model_sha256=env.model_identity["sha256"],
                        episode=episode,
                        capture_sequence=sequence,
                        physics_tick=env.physics_ticks,
                        domain_ns=env.observe().timestamp_ns,
                        capture_steady_ns=time.monotonic_ns(),
                        qpos=env._data.qpos.tolist(),
                        qvel=env._data.qvel.tolist(),
                    )
                )
                rclpy.spin_once(node, timeout_sec=0.04)
            assert len(samples) >= 3, "bounded DDS discovery/capture wait expired"
            latest = samples[-1]
            assert latest.episode_id == episode
            assert latest.physics_tick * 2_000_000 == (
                latest.header.stamp.sec * 1_000_000_000
                + latest.header.stamp.nanosec
                - 1_000_000_000
            )
            assert latest.image.header == latest.header == latest.calibration.header
            assert latest.image.encoding == "rgb8"
            assert (latest.image.width, latest.image.height, latest.image.step) == (
                320,
                240,
                960,
            )
            assert len(latest.image.data) == 320 * 240 * 3
            assert latest.model_sha256 == env.model_identity["sha256"]
            assert (
                latest.calibration_sha256 == env.camera_calibration("wrist")["sha256"]
            )
            decoded = deserialize_message(serialize_message(latest), CameraSample)
            assert decoded == latest
            stop.set()
            worker.join(timeout=5)
            assert worker.exitcode == 0
            delivered = channel.camera.last_delivery_ns.value
            assert delivered > 0
            health.clear()
            deadline = time.monotonic() + 3
            diagnosed = False
            while time.monotonic() < deadline and not diagnosed:
                rclpy.spin_once(node, timeout_sec=0.05)
                diagnosed = any(
                    status.name == "act_lab_wrist_camera"
                    and status.level == bytes([2])
                    and any(
                        value.key == "capture_age_ns"
                        and int(value.value) >= 100_000_000
                        for value in status.values
                    )
                    for message in health
                    for status in message.status
                )
            assert diagnosed, "independent observer missed renderer loss"
            assert observer.is_alive()
            assert channel.camera.last_delivery_ns.value == delivered
    finally:
        stop.set()
        worker.join(timeout=5)
        if worker.is_alive():
            worker.terminate()
            worker.join(timeout=5)
        observer_stop.set()
        observer.join(timeout=5)
        if observer.is_alive():
            observer.terminate()
            observer.join(timeout=5)
        node.destroy_node()
        rclpy.try_shutdown()
    assert worker.exitcode == 0
    evidence = json.loads((tmp_path / "camera.json").read_text())
    assert evidence["frames"] >= 3
    assert evidence["renderer_pid"] == worker.pid
    assert evidence["physics_owner_rendering"] is False
