"""Optional ROS telemetry process and model/bridge supervision."""

from __future__ import annotations

import importlib
import json
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Any

from act_lab.adapters.ros2.observability import ObserverFreshness
from act_lab.adapters.ros2.observation_view import (
    VIEW_TOPICS,
    joint_fields,
    marker_fields,
    scene_tf,
)
from act_lab.adapters.ros2.simulation import CONFIG


def model_parameters(config_path: Path = CONFIG) -> dict[str, Any]:
    # Original root is preserved under frame_prefix; scene-to-UR mapping is Rz(pi).
    description = Path("/opt/crisp/ur5e.urdf").read_text()
    from act_lab.adapters.mujoco.config import SimulationConfig

    config = SimulationConfig.load(config_path)
    if config.model_id != "ur5e_educational_v1":
        from act_lab.adapters.mujoco.environment import MujocoUR5eEnvironment
        from act_lab.adapters.mujoco.tool_description import compose_tool_urdf

        with MujocoUR5eEnvironment(config) as env:
            description = compose_tool_urdf(description, env._model)
    return {
        "robot_description": description,
        "frame_prefix": "ur_model/",
        "use_sim_time": True,
        "publish_frequency": 50.0,
    }


def observer_process(channel: Any, stop: Any, output: str) -> None:
    import rclpy  # type: ignore[import-not-found]
    from rclpy.qos import (  # type: ignore[import-not-found]
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    set_message_fields = importlib.import_module(
        "rosidl_runtime_py.set_message"
    ).set_message_fields

    rclpy.init()
    node = rclpy.create_node(
        "act_lab_observer",
        parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)],
    )
    best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    reliable = QoSProfile(depth=64)
    static = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    specs = {
        "joint": ("sensor_msgs", "JointState", best),
        "telemetry": ("act_lab_interfaces", "ExecutionTelemetry", best),
        "events": ("act_lab_interfaces", "ExecutionEvent", reliable),
        "poses": ("visualization_msgs", "MarkerArray", best),
        "health": ("diagnostic_msgs", "DiagnosticArray", best),
        "tf": ("tf2_msgs", "TFMessage", QoSProfile(depth=100)),
        "static": ("tf2_msgs", "TFMessage", static),
    }
    publishers, types = {}, {}
    for name, (package, message, qos) in specs.items():
        types[name] = getattr(importlib.import_module(package + ".msg"), message)
        publishers[name] = node.create_publisher(
            types[name],
            {"tf": "/tf", "static": "/tf_static"}.get(name, VIEW_TOPICS.get(name, "")),
            qos,
        )

    def publish(name: str, fields: dict[str, Any]) -> None:
        message = types[name]()
        set_message_fields(message, fields)
        publishers[name].publish(message)

    def publish_static() -> None:
        publish(
            "static",
            dict(
                transforms=[
                    dict(
                        header=dict(frame_id="world", stamp=dict(sec=1, nanosec=0)),
                        child_frame_id="ur_model/world",
                        transform=dict(
                            translation=dict(x=0.0, y=0.0, z=0.0),
                            rotation=dict(x=0.0, y=0.0, z=1.0, w=0.0),
                        ),
                    )
                ]
            ),
        )

    publish_static()
    freshness = ObserverFreshness()
    current = None
    last_sample = -1
    last_event = 0
    received = 0
    lost_events = 0
    next_health = 0
    stale_heartbeats = 0
    try:
        while not stop.is_set():
            now = time.monotonic_ns()
            packet = channel.read()
            if packet is not None and packet["sample_id"] != last_sample:
                reset = freshness.receive(packet, now)
                if reset:
                    publish_static()
                current = packet
                last_sample = packet["sample_id"]
                received += 1
                publish("telemetry", packet["telemetry"])
                publish("joint", joint_fields(packet))
                publish("poses", marker_fields(packet))
                publish("tf", scene_tf(packet))
                for event in packet["events"]:
                    if event["event_id"] > last_event:
                        lost_events += max(0, event["event_id"] - last_event - 1)
                        last_event = event["event_id"]
                        publish("events", event)
            if now >= next_health:
                stale = freshness.stale(now)
                stale_heartbeats += int(stale)
                if current is not None and stale:
                    publish("poses", marker_fields(current, stale=True))
                camera_health = []
                if channel.camera is not None:
                    captured = channel.camera.last_delivery_ns.value
                    camera_stale = not captured or now - captured >= 100_000_000
                    camera_health.append(
                        dict(
                            level=bytes([2 if camera_stale else 0]),
                            name="act_lab_wrist_camera",
                            hardware_id="simulation",
                            message="STALE / renderer or source unavailable"
                            if camera_stale
                            else "live camera capture",
                            values=[
                                dict(
                                    key="capture_age_ns",
                                    value=str(now - captured if captured else -1),
                                )
                            ],
                        )
                    )
                publish(
                    "health",
                    dict(
                        header=current["telemetry"]["header"]
                        if current
                        else dict(frame_id="world"),
                        status=[
                            dict(
                                level=bytes([2 if stale else 0]),
                                name="act_lab_observer",
                                message="STALE / no authority snapshot"
                                if stale
                                else "live authority snapshot",
                                hardware_id="simulation",
                                values=[
                                    dict(
                                        key="observer_receipt_age_ns",
                                        value=str(
                                            now - freshness.received_ns
                                            if freshness.received_ns
                                            else -1
                                        ),
                                    ),
                                    dict(
                                        key="handoff_drops",
                                        value=str(channel.dropped.value),
                                    ),
                                    dict(
                                        key="events_lost_by_observer",
                                        value=str(lost_events),
                                    ),
                                ],
                            ),
                            *camera_health,
                        ],
                    ),
                )
                next_health = now + 50_000_000
            rclpy.spin_once(node, timeout_sec=0.002)
    finally:
        Path(output).mkdir(parents=True, exist_ok=True)
        Path(output, "observer.json").write_text(
            json.dumps(
                dict(
                    schema_version=1,
                    samples=received,
                    stale_heartbeats=stale_heartbeats,
                    events=last_event,
                    lost_events=lost_events,
                    handoff_drops=channel.dropped.value,
                ),
                indent=2,
            )
            + "\n"
        )
        node.destroy_node()
        rclpy.try_shutdown()


class ViewerProcesses:
    def __init__(
        self, output: Path, bridge: bool = True, config_path: Path = CONFIG
    ) -> None:
        import yaml  # type: ignore[import-untyped]

        output.mkdir(parents=True, exist_ok=True)
        self.processes: list[Any] = []
        self.files: list[Any] = []
        model_file = output / "view-model.yaml"
        model_file.write_text(
            yaml.safe_dump(
                {
                    "/act_lab/view/model": {
                        "ros__parameters": model_parameters(config_path)
                    }
                }
            )
        )
        commands = [
            [
                "ros2",
                "run",
                "robot_state_publisher",
                "robot_state_publisher",
                "--ros-args",
                "-r",
                "__ns:=/act_lab/view",
                "-r",
                "__node:=model",
                "-r",
                "joint_states:=/joint_states",
                "--params-file",
                str(model_file),
            ],
            [
                "ros2",
                "run",
                "foxglove_bridge",
                "foxglove_bridge",
                "--ros-args",
                "-r",
                "__ns:=/act_lab/view",
                "--params-file",
                "configs/ros2/foxglove/bridge.yaml",
            ],
        ]
        if not bridge:
            commands = commands[:1]
        try:
            for index, command in enumerate(commands):
                log = (output / f"viewer-{index}.log").open("w")
                self.files.append(log)
                self.processes.append(
                    subprocess.Popen(
                        command, stdout=log, stderr=log, start_new_session=True
                    )
                )
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        for process in self.processes:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for process in self.processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        for file in self.files:
            file.close()
