"""Separate scratch-model renderer and read-only camera publisher."""

from __future__ import annotations

import json
import time
from array import array
from pathlib import Path
from typing import Any

from act_lab.adapters.ros2.camera_contracts import CameraSource, can_publish_frame


def camera_process(channel: Any, stop: Any, output: str, config_path: Path) -> None:
    import mujoco  # type: ignore[import-untyped]
    import rclpy  # type: ignore[import-not-found]
    from act_lab_interfaces.msg import CameraSample  # type: ignore[import-not-found]
    from rclpy.parameter import Parameter  # type: ignore[import-not-found]
    from rclpy.qos import (  # type: ignore[import-not-found]
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import CameraInfo, Image  # type: ignore[import-not-found]
    from tf2_msgs.msg import TFMessage  # type: ignore[import-not-found]

    from act_lab.adapters.mujoco.environment import MujocoUR5eEnvironment
    from act_lab.adapters.ros2.contracts import encode_pose, encode_stamp

    rclpy.init()
    node = rclpy.create_node(
        "act_lab_wrist_camera",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    root = "/act_lab/view/wrist/"
    image_pub = node.create_publisher(Image, root + "image_raw", qos)
    info_pub = node.create_publisher(CameraInfo, root + "camera_info", qos)
    sample_pub = node.create_publisher(CameraSample, root + "sample", qos)
    tf_pub = node.create_publisher(TFMessage, "/tf", QoSProfile(depth=100))
    from rosidl_runtime_py.set_message import (  # type: ignore[import-not-found]
        set_message_fields,
    )

    frames = drops = 0
    durations: list[float] = []
    profile: dict[str, Any] = {}
    identity: dict[str, object] = {}
    started_at = time.monotonic()
    env = None
    try:
        env = MujocoUR5eEnvironment.from_config_file(config_path)
        env.reset(0)
        profile = dict[str, Any](env.camera_calibration("wrist"))
        identity = env.model_identity
        source = CameraSource(
            str(env.model_identity["sha256"]), env._model.nq, env._model.nv
        )
        while not stop.is_set():
            now = time.monotonic_ns()
            packet = channel.read()
            if packet is not None and source.offer(packet, now):
                started = time.monotonic()
                env._data.qpos[:] = packet["qpos"]
                env._data.qvel[:] = packet["qvel"]
                env._data.time = packet["domain_ns"] / 1e9
                env._physics_ticks = packet["physics_tick"]
                mujoco.mj_forward(env._model, env._data)
                pixels = env.render("wrist")
                if not can_publish_frame(packet, channel.read(), time.monotonic_ns()):
                    drops += 1
                    continue
                image = Image()
                stamp = encode_stamp(packet["domain_ns"])
                set_message_fields(
                    image,
                    dict(
                        header=dict(stamp=stamp, frame_id=profile["frame_id"]),
                        height=profile["height"],
                        width=profile["width"],
                        encoding="rgb8",
                        is_bigendian=0,
                        step=int(profile["width"]) * 3,
                    ),
                )
                image.data = array("B", pixels.tobytes())
                info = CameraInfo()
                set_message_fields(
                    info,
                    dict(
                        header=dict(stamp=stamp, frame_id=profile["frame_id"]),
                        height=profile["height"],
                        width=profile["width"],
                        distortion_model=profile["distortion_model"],
                        d=profile["d"],
                        k=profile["k"],
                        r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                        p=[
                            profile["k"][0],
                            0.0,
                            profile["k"][2],
                            0.0,
                            0.0,
                            profile["k"][4],
                            profile["k"][5],
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            0.0,
                        ],
                    ),
                )
                sample = CameraSample()
                sample.header = image.header
                sample.schema_version = 1
                sample.episode_id = packet["episode"]
                sample.capture_sequence = packet["capture_sequence"]
                sample.physics_tick = packet["physics_tick"]
                sample.model_sha256 = packet["model_sha256"]
                sample.calibration_sha256 = profile["sha256"]
                sample.image, sample.calibration = image, info
                tf = TFMessage()
                pose = encode_pose(env.camera_world_pose("wrist"))
                set_message_fields(
                    tf,
                    dict(
                        transforms=[
                            dict(
                                header=dict(stamp=stamp, frame_id="world"),
                                child_frame_id=profile["frame_id"],
                                transform=dict(
                                    translation=pose["position"],
                                    rotation=pose["orientation"],
                                ),
                            )
                        ]
                    ),
                )
                # Serialization also consumes capture age; check immediately before DDS.
                if not can_publish_frame(packet, channel.read(), time.monotonic_ns()):
                    drops += 1
                    continue
                tf_pub.publish(tf)
                image_pub.publish(image)
                info_pub.publish(info)
                sample_pub.publish(sample)
                frames += 1
                channel.last_delivery_ns.value = packet["capture_steady_ns"]
                durations.append(time.monotonic() - started)
            rclpy.spin_once(node, timeout_sec=0.002)
    finally:
        Path(output, "camera.json").write_text(
            json.dumps(
                dict(
                    schema_version=1,
                    frames=frames,
                    discarded_frames=drops,
                    renderer_pid=__import__("os").getpid(),
                    render_publish_max_s=max(durations, default=0),
                    render_publish_mean_s=sum(durations) / len(durations)
                    if durations
                    else 0,
                    physics_owner_rendering=False,
                    model=identity,
                    calibration=profile,
                    wall_duration_s=time.monotonic() - started_at,
                    delivered_hz=frames / max(0.001, time.monotonic() - started_at),
                ),
                indent=2,
            )
            + "\n"
        )
        if env:
            env.close()
        node.destroy_node()
        rclpy.try_shutdown()
