"""Lazy ROS access and generated-message factories, isolated from the core."""

from __future__ import annotations

import importlib
from typing import Any

from act_lab.adapters.ros2.contracts import TOPICS, WATCHDOG_NS

INSTALL_HINT = (
    "ROS 2 contracts require the isolated image: "
    "docker compose --profile ros2-contracts run --rm ros2-contracts"
)


def require_ros() -> None:
    try:
        for name in (
            "rclpy",
            "act_lab_interfaces.msg",
            "rosgraph_msgs.msg",
            "rosidl_runtime_py.convert",
            "rosidl_runtime_py.set_message",
        ):
            importlib.import_module(name)
    except ImportError as error:
        raise RuntimeError(INSTALL_HINT) from error


def make_message(kind: str, fields: dict[str, Any]) -> Any:
    require_ros()
    module = importlib.import_module(
        "rosgraph_msgs.msg" if kind == "Clock" else "act_lab_interfaces.msg"
    )
    message = getattr(module, kind)()
    setter = importlib.import_module("rosidl_runtime_py.set_message")
    setter.set_message_fields(message, fields)
    return message


def message_fields(message: Any) -> dict[str, Any]:
    converter = importlib.import_module("rosidl_runtime_py.convert")
    return dict(converter.message_to_ordereddict(message))


def qos_profiles() -> dict[str, Any]:
    require_ros()
    qos = importlib.import_module("rclpy.qos")
    duration = importlib.import_module("rclpy.duration").Duration

    def profile(reliable: bool, depth: int) -> Any:
        return qos.QoSProfile(
            history=qos.HistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=qos.ReliabilityPolicy.RELIABLE
            if reliable
            else qos.ReliabilityPolicy.BEST_EFFORT,
            durability=qos.DurabilityPolicy.VOLATILE,
        )

    profiles = {
        "command": profile(True, 1),
        "state": profile(False, 1),
        "report": profile(True, 10),
        "clock": profile(False, 1),
    }
    profiles["command"].deadline = duration(nanoseconds=WATCHDOG_NS)
    profiles["command"].lifespan = duration(nanoseconds=WATCHDOG_NS)
    assert set(profiles) == set(TOPICS)
    return profiles
