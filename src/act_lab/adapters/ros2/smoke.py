"""Two-process localhost DDS contract demonstration with injected safety clocks."""

from __future__ import annotations

import hashlib
import importlib
import json
import multiprocessing
import os
import platform
import time
from collections.abc import Callable
from dataclasses import replace
from multiprocessing.connection import Connection
from pathlib import Path
from typing import Any
from uuid import uuid4

from act_lab.adapters.ros2.contracts import (
    CLOCK_OFFSET_NS,
    JOINT_NAMES,
    TOPICS,
    CommandEnvelope,
    decode_command,
    decode_report,
    decode_state,
    encode_command,
    encode_report,
    encode_stamp,
    encode_state,
)
from act_lab.adapters.ros2.demonstrator import HOME, make_robot
from act_lab.adapters.ros2.inbox import CommandInbox
from act_lab.adapters.ros2.runtime import (
    make_message,
    message_fields,
    qos_profiles,
    require_ros,
)
from act_lab.domain import Action, CommandOutcome


def _wait(rclpy: Any, node: Any, ready: Callable[[], bool], label: str) -> None:
    deadline = time.monotonic() + 15.0
    while not ready():
        if time.monotonic() >= deadline:
            raise RuntimeError(f"bounded DDS wait expired: {label}")
        rclpy.spin_once(node, timeout_sec=0.01)


def _producer(connection: Connection) -> None:
    """Separate ROS process; never has a robot or driver."""
    rclpy = importlib.import_module("rclpy")
    rclpy.init()
    node = rclpy.create_node("act_lab_contract_producer")
    parameter = importlib.import_module("rclpy.parameter").Parameter
    node.set_parameters([parameter("use_sim_time", value=True)])
    messages = importlib.import_module("act_lab_interfaces.msg")
    profiles = qos_profiles()
    publisher = node.create_publisher(
        messages.CartesianCommand, TOPICS["command"], profiles["command"]
    )
    node.create_subscription(
        messages.RobotState,
        TOPICS["state"],
        lambda msg: connection.send(("state", message_fields(msg))),
        profiles["state"],
    )
    node.create_subscription(
        messages.CommandReport,
        TOPICS["report"],
        lambda msg: connection.send(("report", message_fields(msg))),
        profiles["report"],
    )
    try:
        _wait(
            rclpy,
            node,
            lambda: publisher.get_subscription_count() > 0,
            "command subscriber discovery",
        )
        connection.send(("ready", os.getpid()))
        running = True
        while running:
            rclpy.spin_once(node, timeout_sec=0.01)
            while connection.poll():
                kind, fields = connection.recv()
                if kind == "stop":
                    running = False
                    break
                publisher.publish(make_message("CartesianCommand", fields))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        connection.close()


class _DDS:
    """Consumer callbacks store intent; caller/control thread owns execution."""

    def __init__(self) -> None:
        require_ros()
        self.rclpy = importlib.import_module("rclpy")
        self.rclpy.init()
        self.node = self.rclpy.create_node("act_lab_contract_consumer")
        parameter = importlib.import_module("rclpy.parameter").Parameter
        self.node.set_parameters([parameter("use_sim_time", value=True)])
        self.inbox = CommandInbox()
        self.steady = 0
        self.received = 0
        self.last_sequence = 0
        self.rows: list[tuple[str, Any]] = []
        self.profiles = qos_profiles()
        messages = importlib.import_module("act_lab_interfaces.msg")
        self.node.create_subscription(
            messages.CartesianCommand,
            TOPICS["command"],
            self._receive,
            self.profiles["command"],
        )
        self.state = self.node.create_publisher(
            messages.RobotState, TOPICS["state"], self.profiles["state"]
        )
        self.report = self.node.create_publisher(
            messages.CommandReport, TOPICS["report"], self.profiles["report"]
        )
        clock_type = importlib.import_module("rosgraph_msgs.msg").Clock
        self.clock = self.node.create_publisher(
            clock_type, TOPICS["clock"], self.profiles["clock"]
        )
        context = multiprocessing.get_context("spawn")
        self.connection, child_connection = context.Pipe()
        self.child = context.Process(target=_producer, args=(child_connection,))
        self.child.start()
        child_connection.close()

    def _receive(self, message: Any) -> None:
        self.received += 1
        try:
            envelope = decode_command(message_fields(message))
            self.last_sequence = envelope.sequence
            self.inbox.offer(envelope, self.steady)
        except (ValueError, KeyError, TypeError, OverflowError):
            self.inbox.reject()

    def drain(self) -> bool:
        while self.connection.poll():
            self.rows.append(self.connection.recv())
        return bool(self.rows)

    def ready(self) -> None:
        def ready() -> bool:
            self.drain()
            return (
                any(kind == "ready" for kind, _ in self.rows)
                and self.state.get_subscription_count() > 0
                and self.report.get_subscription_count() > 0
                and self.clock.get_subscription_count() >= 2
            )

        _wait(self.rclpy, self.node, ready, "producer/state/report/clock discovery")

    def tick(self, domain_ns: int, episode: str, steady: int) -> None:
        self.steady = steady
        self.inbox.update_clock(domain_ns + CLOCK_OFFSET_NS, episode, steady)
        message = make_message("Clock", {"clock": encode_stamp(domain_ns)})
        self.clock.publish(message)

        def observed() -> bool:
            if self.node.get_clock().now().nanoseconds == domain_ns + CLOCK_OFFSET_NS:
                return True
            self.clock.publish(message)  # Best effort may legitimately lose a packet.
            return False

        _wait(self.rclpy, self.node, observed, "explicit simulation clock delivery")

    def send(self, envelope: CommandEnvelope) -> None:
        self.send_fields(encode_command(envelope))

    def send_fields(self, fields: dict[str, Any]) -> None:
        before = self.received
        self.connection.send(("command", fields))
        _wait(self.rclpy, self.node, lambda: self.received > before, "command delivery")

    def publish_result(self, report: Any, episode: str) -> None:
        state_fields = encode_state(report.resulting_state, episode)
        report_fields = encode_report(report, episode, self.last_sequence)
        self.rows.clear()
        self.state.publish(make_message("RobotState", state_fields))
        self.report.publish(make_message("CommandReport", report_fields))

        def received() -> bool:
            self.drain()
            kinds = {kind for kind, _ in self.rows}
            if "state" not in kinds:
                self.state.publish(make_message("RobotState", state_fields))
            return kinds >= {"state", "report"}

        _wait(self.rclpy, self.node, received, "state and report delivery")
        assert any(
            decode_state(fields)[0] == report.resulting_state
            for kind, fields in self.rows
            if kind == "state"
        )
        assert any(
            json.dumps(
                encode_report(decode_report(fields)[0], episode, self.last_sequence),
                sort_keys=True,
            )
            == json.dumps(report_fields, sort_keys=True)
            for kind, fields in self.rows
            if kind == "report"
        )

    def stop_producer(self) -> None:
        if self.child.is_alive():
            self.connection.send(("stop", None))
            self.child.join(timeout=5)
        if self.child.is_alive():
            self.child.terminate()
            self.child.join(timeout=5)
        if self.child.exitcode != 0:
            raise RuntimeError(f"DDS producer exited with {self.child.exitcode}")

    def close(self) -> None:
        try:
            self.stop_producer()
        finally:
            self.connection.close()
            self.node.destroy_node()
            self.rclpy.shutdown()


def run_smoke() -> dict[str, Any]:
    """Assert real DDS conversions and faults, without wall-time safety thresholds."""
    robot, driver = make_robot()
    dds = _DDS()
    cases: list[dict[str, Any]] = []
    try:
        dds.ready()
        episode = str(uuid4())
        steady = 0
        sequence = 0

        def tick(delta: int = 1_000_000) -> None:
            nonlocal steady
            steady += delta
            dds.tick(robot.observe().timestamp_ns, episode, steady)

        def send(
            *,
            frame: str = "world",
            timestamp: int | None = None,
            episode_override: str | None = None,
            seq: int | None = None,
            gripper: float = 0.6,
        ) -> None:
            nonlocal sequence
            sequence += 1
            dds.send(
                CommandEnvelope(
                    episode_override or episode,
                    sequence if seq is None else seq,
                    Action(
                        robot.observe().timestamp_ns
                        if timestamp is None
                        else timestamp,
                        replace(HOME, frame_id=frame),
                        gripper,
                        True,
                    ),
                )
            )

        def consume(
            name: str,
            expected: CommandOutcome,
            reason: str | None = None,
            *,
            publish: bool = True,
        ) -> None:
            action = dds.inbox.poll(robot.observe(), steady)
            robot.command(action)
            report = robot.last_command_report
            assert report is not None
            assert report.outcome == expected, (name, report)
            if reason is not None:
                assert dds.inbox.last_rejection_reason == reason, name
            cases.append(
                {
                    "case": name,
                    "outcome": report.outcome.value,
                    "transport_reason": dds.inbox.last_rejection_reason,
                    "source_timestamp_ns": report.requested_action.timestamp_ns,
                    "gripper_position": report.resulting_state.gripper_position,
                }
            )
            if publish:
                dds.publish_result(report, episode)

        tick()
        consume("startup_missing", CommandOutcome.DISABLED)
        tick()
        send()
        consume("fresh", CommandOutcome.LIMITED)
        tick()
        consume("repeated_original_stamp", CommandOutcome.LIMITED)
        assert cases[-1]["source_timestamp_ns"] == cases[-2]["source_timestamp_ns"]
        tick()
        send(frame="camera")
        consume("unsupported_frame", CommandOutcome.INVALID)
        tick()
        send(gripper=float("nan"))
        # Canonical JSON comparison preserves NaN audit intent during round trips.
        consume("invalid_numeric", CommandOutcome.INVALID)
        tick()
        send(timestamp=robot.observe().timestamp_ns + 1)
        consume("future", CommandOutcome.INVALID, "future_timestamp")
        tick()
        send(seq=sequence)
        consume("duplicate", CommandOutcome.DISABLED, "out_of_order")
        tick()
        send(seq=0)
        consume("out_of_order", CommandOutcome.DISABLED, "out_of_order")
        tick()
        send(episode_override=str(uuid4()))
        consume("wrong_episode", CommandOutcome.DISABLED, "wrong_episode")
        tick()
        malformed = encode_command(
            CommandEnvelope(
                episode,
                sequence + 1,
                Action(robot.observe().timestamp_ns, HOME, 0.6, True),
            )
        )
        malformed["episode_id"] = "not-a-uuid"
        dds.send_fields(malformed)
        consume("malformed_envelope", CommandOutcome.DISABLED, "malformed_envelope")
        tick()
        send(timestamp=robot.observe().timestamp_ns - 100_000_000)
        consume("exact_stale_boundary", CommandOutcome.STALE, "stale_command")
        tick()
        send()
        # Keep receiving newer sequences while simulation time is paused.
        steady += 100_000_000
        dds.tick(robot.observe().timestamp_ns, episode, steady)
        send()
        consume("paused_clock", CommandOutcome.DISABLED, "clock_paused")
        tick()
        consume("resume_without_fresh", CommandOutcome.DISABLED)
        tick()
        send()
        consume("resume_fresh", CommandOutcome.LIMITED)
        tick()
        send()
        driver.state = replace(
            driver.state, timestamp_ns=driver.state.timestamp_ns + 100_000_000
        )
        tick()
        consume("forward_jump", CommandOutcome.STALE, "stale_command")
        # Backward clock while retaining UUID latches until a new epoch.
        steady += 1_000_000
        dds.tick(0, episode, steady)
        consume("backward_jump", CommandOutcome.DISABLED, "clock_reversed")
        tick()
        send()
        consume("same_epoch_after_reset", CommandOutcome.DISABLED, "clock_not_ready")
        old_episode = episode
        episode = str(uuid4())
        robot.reset(0)
        tick()
        send(episode_override=old_episode)
        consume("old_epoch_replay", CommandOutcome.DISABLED, "wrong_episode")
        tick()
        send()
        consume("new_epoch_recovery", CommandOutcome.LIMITED)
        tick()
        send()
        dds.stop_producer()
        # Advance ROS time only 20ms but receipt steady age exactly100ms;
        # progress was <100ms ago, distinguishing the two watchdogs.
        driver.state = replace(
            driver.state, timestamp_ns=driver.state.timestamp_ns + 20_000_000
        )
        steady += 99_000_000
        dds.tick(robot.observe().timestamp_ns, episode, steady)
        steady += 1_000_000
        consume(
            "publisher_loss", CommandOutcome.DISABLED, "receipt_timeout", publish=False
        )
        assert cases[-1]["gripper_position"] == cases[-2]["gripper_position"]
        dds.inbox.reject("shutdown")
        tick()
        consume("shutdown", CommandOutcome.DISABLED, "shutdown", publish=False)
        packages = Path("/opt/act-lab-ros/packages.txt")
        return {
            "schema_version": 1,
            "status": "ok",
            "cases": cases,
            "dds_commands_received": dds.received,
            "conversion_checks": {
                "domain_zero_ros_stamp": encode_stamp(0),
                "canonical_joint_names": list(JOINT_NAMES),
                "command_state_report_dds_roundtrips": True,
                "quaternion_conversion": "XYZW wire to WXYZ domain; no normalization",
            },
            "provenance": {
                "python": platform.python_version(),
                "base_image_digest": (
                    "sha256:066420e07f60aa18262f2479981def87ebc"
                    "fcec42eefb0c0c57c4a46098348ca"
                ),
                "episode_id": episode,
                "ros_distro": os.environ.get("ROS_DISTRO"),
                "rmw": dds.rclpy.get_rmw_implementation_identifier(),
                "consumer_pid": os.getpid(),
                "producer_pid": dds.child.pid,
                "driver": "deterministic_stationary_fake",
                "package_manifest_sha256": hashlib.sha256(
                    packages.read_bytes()
                ).hexdigest(),
            },
            "qos": {
                key: {
                    "depth": value.depth,
                    "reliability": value.reliability.name,
                    "durability": value.durability.name,
                    "deadline_ns": value.deadline.nanoseconds,
                    "lifespan_ns": value.lifespan.nanoseconds,
                }
                for key, value in dds.profiles.items()
            },
        }
    finally:
        try:
            dds.inbox.reject("shutdown")
            observed = robot.observe()
            robot.command(
                Action(
                    observed.timestamp_ns,
                    observed.robot.end_effector_pose,
                    observed.robot.gripper_position,
                    False,
                )
            )
        finally:
            dds.close()
