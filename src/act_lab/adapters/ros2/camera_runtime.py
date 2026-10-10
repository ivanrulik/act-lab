"""Separate scratch-model renderer and read-only camera publisher."""

from __future__ import annotations

import json
import time
from array import array
from importlib import import_module
from pathlib import Path
from typing import Any

from act_lab.adapters.ros2.camera_contracts import CameraSource, can_publish_frame
from act_lab.adapters.ros2.camera_rendering import (
    encode_jpeg,
    render_settings,
    verify_renderer,
)


def camera_process(channel: Any, stop: Any, output: str, config_path: Path) -> None:
    import mujoco  # type: ignore[import-untyped]
    import rclpy  # type: ignore[import-not-found]
    from act_lab_interfaces.msg import CameraSample  # type: ignore[import-not-found]
    from rclpy.parameter import Parameter  # type: ignore[import-not-found]
    from rclpy.qos import (  # type: ignore[import-not-found]
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import (  # type: ignore[import-not-found]
        CameraInfo,
        CompressedImage,
        Image,
    )
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
    preview_pub = node.create_publisher(
        CompressedImage, root + "image_raw/compressed", qos
    )
    info_pub = node.create_publisher(CameraInfo, root + "camera_info", qos)
    sample_pub = node.create_publisher(CameraSample, root + "sample", qos)
    tf_pub = node.create_publisher(TFMessage, "/tf", QoSProfile(depth=100))
    from rosidl_runtime_py.set_message import (  # type: ignore[import-not-found]
        set_message_fields,
    )

    frames = drops = 0
    durations: list[float] = []
    renders: list[float] = []
    encodes: list[float] = []
    ages: list[int] = []
    delivery_times: list[int] = []
    jpeg_sizes: list[int] = []
    settings = render_settings()
    gl_identity: dict[str, str] = {}
    profile: dict[str, Any] = {}
    identity: dict[str, object] = {}
    started_at = time.monotonic()
    env = None
    try:
        env = MujocoUR5eEnvironment.from_config_file(config_path)
        env.reset(0)
        profile = dict[str, Any](env.camera_calibration("wrist"))
        identity = env.model_identity
        # Viewer-only quality; the acquisition model and its files stay unchanged.
        env._model.vis.quality.offsamples = settings["offsamples"]
        env.render("wrist")  # Initialize the reusable EGL context before inspection.
        gl_identity = verify_renderer(settings)
        renderer = env._renderer
        assert renderer is not None
        source = CameraSource(
            str(env.model_identity["sha256"]), env._model.nq, env._model.nv
        )
        period_ns = 1_000_000_000 // settings["target_hz"]
        next_render_ns = time.monotonic_ns()
        while not stop.is_set():
            now = time.monotonic_ns()
            packet = channel.read()
            if (
                now >= next_render_ns
                and packet is not None
                and source.offer(packet, now)
            ):
                next_render_ns = max(next_render_ns + period_ns, now + 1)
                started = time.monotonic()
                env._data.qpos[:] = packet["qpos"]
                env._data.qvel[:] = packet["qvel"]
                env._data.time = packet["domain_ns"] / 1e9
                env._physics_ticks = packet["physics_tick"]
                mujoco.mj_forward(env._model, env._data)
                render_started = time.monotonic()
                renderer.update_scene(env._data, camera="wrist")
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = settings[
                    "shadows"
                ]
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = (
                    settings["reflections"]
                )
                pixels = renderer.render()
                renders.append(time.monotonic() - render_started)
                if not can_publish_frame(packet, channel.read(), time.monotonic_ns()):
                    drops += 1
                    continue
                encode_started = time.monotonic()
                jpeg = encode_jpeg(pixels, settings["jpeg_quality"])
                encodes.append(time.monotonic() - encode_started)
                jpeg_sizes.append(len(jpeg))
                preview = CompressedImage()
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
                preview.header = image.header
                preview.format = "rgb8; jpeg compressed bgr8"
                preview.data = array("B", jpeg)
                raw_count = image_pub.get_subscription_count()
                sample_count = sample_pub.get_subscription_count()
                raw_requested = raw_count > 0 or sample_count > 0
                if raw_requested:
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
                preview_pub.publish(preview)
                if raw_count > 0:
                    image_pub.publish(image)
                info_pub.publish(info)
                if sample_count > 0:
                    sample_pub.publish(sample)
                delivered_at = time.monotonic_ns()
                ages.append(delivered_at - packet["capture_steady_ns"])
                delivery_times.append(delivered_at)
                frames += 1
                channel.last_delivery_ns.value = packet["capture_steady_ns"]
                durations.append(time.monotonic() - started)
            rclpy.spin_once(node, timeout_sec=0.002)
    finally:
        Path(output, "camera.json").write_text(
            json.dumps(
                dict(
                    schema_version=2,
                    renderer=gl_identity,
                    render_profile=settings,
                    transport="jpeg",
                    jpeg_codec=dict(
                        name="Pillow", version=import_module("PIL").__version__
                    ),
                    render_mean_s=sum(renders) / len(renders) if renders else 0,
                    encode_mean_s=sum(encodes) / len(encodes) if encodes else 0,
                    capture_age_max_ns=max(ages, default=0),
                    capture_age_p95_ns=sorted(ages)[int((len(ages) - 1) * 0.95)]
                    if ages
                    else 0,
                    jpeg_mean_bytes=sum(jpeg_sizes) / len(jpeg_sizes)
                    if jpeg_sizes
                    else 0,
                    active_delivery_hz=(len(delivery_times) - 1)
                    * 1e9
                    / (delivery_times[-1] - delivery_times[0])
                    if len(delivery_times) > 1
                    else 0,
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
