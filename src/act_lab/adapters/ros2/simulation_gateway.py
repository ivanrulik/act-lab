"""Optional shared-safety driver and separate-process DDS command gateway."""

from __future__ import annotations

import time
from dataclasses import asdict
from multiprocessing.connection import Connection
from pathlib import Path
from typing import Any

import mujoco  # type: ignore[import-untyped]
import numpy as np

from act_lab.adapters.mujoco.cartesian_driver import MujocoCartesianDriver
from act_lab.adapters.mujoco.config import SimulationConfig
from act_lab.adapters.mujoco.environment import MujocoUR5eEnvironment
from act_lab.adapters.ros2.contracts import (
    TOPICS,
    CommandEnvelope,
    decode_command,
    encode_report,
)
from act_lab.adapters.ros2.inbox import CommandInbox
from act_lab.adapters.ros2.simulation import (
    CONFIG,
    SimulationProcess,
    diagnostic_json,
    receive,
    send,
    snapshot_state,
)
from act_lab.application.cartesian_control import JointVector, SafeCartesianRobot
from act_lab.domain import Action, Observation, RobotState


class RosSimulationDriver(MujocoCartesianDriver):
    """Scratch state follows physics snapshots; only the remote owner integrates."""

    def __init__(self, runtime: SimulationProcess) -> None:
        config = SimulationConfig.load(runtime.config_path)
        super().__init__(MujocoUR5eEnvironment(config), config)
        self.runtime = runtime
        self.intent: Action | None = None
        self.sequence = 0
        self.receipt_ns = 0
        self.view_intent: dict[str, Any] | None = None
        self.acquisition_active = False
        self._sync()

    def _sync(self) -> None:
        value = self.runtime.snapshot
        self._data.qpos[:] = value["qpos"]
        self._data.qvel[:] = value["qvel"]
        state = snapshot_state(value)
        self.environment._physics_ticks = state.timestamp_ns // 2_000_000  # noqa: SLF001
        self.environment._environment_steps = state.timestamp_ns // 20_000_000  # noqa: SLF001
        mujoco.mj_forward(self._model, self._data)

    def reset(self, seed: int) -> Observation:
        self.runtime.rpc(dict(kind="reset", seed=seed))
        self._sync()
        return self.observe()

    def observe(self) -> Observation:
        state = snapshot_state(self.runtime.snapshot)
        return Observation(state.timestamp_ns, state, self.simulation_config.cameras)

    def set_execution(self, intent: Action | None) -> None:
        self.intent = intent
        if self.view_intent is not None and intent is not None:
            from act_lab.adapters.ros2.contracts import encode_pose

            self.view_intent["approved"] = encode_pose(intent.target_pose)

    def step(
        self, joints: JointVector, joint_velocity: JointVector, gripper: float
    ) -> RobotState:
        authorization = None
        if self.intent is not None:
            self._copy_state_to_scratch()
            self._scratch.qpos[self._qpos_indices] = joints
            mujoco.mj_forward(self._model, self._scratch)
            q = np.empty(4)
            mujoco.mju_mat2Quat(q, self._scratch.site_xmat[self._site_id])
            authorization = dict(
                episode=self.runtime.snapshot["episode"],
                sequence=self.sequence,
                source_ns=self.intent.timestamp_ns,
                receipt_ns=self.receipt_ns,
                frame=self.intent.target_pose.frame_id,
                position=self._scratch.site_xpos[self._site_id].tolist(),
                quaternion_wxyz=q.tolist(),
                gripper=gripper,
            )
        self.runtime.rpc(
            dict(
                kind="execute",
                authorization=authorization,
                ticks=10,
                observation=self.view_intent,
                hold_after=self.acquisition_active,
            )
        )
        self._sync()
        return self.observe().robot


def gateway(
    connection: Connection,
    output: str,
    seed: int,
    paced: bool = False,
    observation_channel: Any = None,
    config_path: Path = CONFIG,
) -> None:
    import rclpy  # type: ignore[import-not-found]
    from rclpy.parameter import Parameter  # type: ignore[import-not-found]

    from act_lab.adapters.ros2.runtime import make_message, message_fields, qos_profiles

    rclpy.init()
    node = rclpy.create_node(
        "act_lab_command_gateway",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    runtime = SimulationProcess(
        Path(output), seed, paced, observation_channel, config_path
    )
    driver = RosSimulationDriver(runtime)
    robot = SafeCartesianRobot(driver, driver.limits)
    robot.reset(seed)
    inbox = CommandInbox()
    inbox.update_clock(
        driver.observe().timestamp_ns + 1_000_000_000,
        runtime.snapshot["episode"],
        runtime.snapshot["steady_ns"],
    )
    latest: CommandEnvelope | None = None
    receipt = 0
    wall_receipt = 0
    arrivals = 0
    recording_manager: Any = None
    recording_sink: Any = None
    recording_sequence = 0

    def callback(message: Any) -> None:
        nonlocal latest, receipt, wall_receipt, arrivals
        arrivals += 1
        wall_receipt = time.monotonic_ns()
        receipt = runtime.snapshot["steady_ns"]
        try:
            latest = decode_command(message_fields(message))
            inbox.offer(latest, receipt)
        except (ValueError, KeyError, TypeError):
            inbox.reject("malformed_envelope")

    qos = qos_profiles()
    message_type = type(make_message("CartesianCommand", {}))
    node.create_subscription(message_type, TOPICS["command"], callback, qos["command"])
    report_pub = node.create_publisher(
        type(make_message("CommandReport", {})), TOPICS["report"], qos["report"]
    )
    try:
        send(
            connection, dict(**runtime.snapshot, gateway_pid=__import__("os").getpid())
        )
        while True:
            request = receive(connection)
            kind = request["kind"]
            if kind == "record_start":
                if paced:
                    raise ValueError(
                        "synchronized acquisition requires stepped simulation"
                    )
                if recording_manager is not None:
                    raise ValueError("recording already active")
                import argparse

                from act_lab.adapters.mcap.session import recording_robot
                from act_lab.adapters.ros2.recording import RosEpisodeSink

                recording_sink = RosEpisodeSink(
                    node,
                    episode=lambda: runtime.snapshot["episode"],
                    sequence_start=recording_sequence,
                    command_sequence=lambda: driver.sequence,
                )
                args = argparse.Namespace(
                    record_dir=Path(output) / "acquisition",
                    outcome="failure",
                    reason="bounded_ros_demo",
                    operator="",
                )
                recording_manager = recording_robot(
                    driver,
                    args,
                    "ros-crisp",
                    request["seed"],
                    sink_factory=recording_sink.for_directory,
                )
                robot = recording_manager.__enter__()
                driver.acquisition_active = True
                robot.reset(request["seed"])
                inbox = CommandInbox()
                latest = None
                inbox.update_clock(
                    driver.observe().timestamp_ns + 1_000_000_000,
                    runtime.snapshot["episode"],
                    runtime.snapshot["steady_ns"],
                )
                send(connection, runtime.snapshot)
                continue
            if kind == "record_stop":
                if recording_manager is None:
                    raise ValueError("no active recording")
                observation = robot.observe()
                robot.command(
                    Action(
                        observation.timestamp_ns,
                        observation.robot.end_effector_pose,
                        observation.robot.gripper_position,
                        False,
                    )
                )
                recording_manager.__exit__(None, None, None)
                recording_sequence = recording_sink.sequence
                recording_manager = None
                driver.acquisition_active = False
                robot = SafeCartesianRobot(driver, driver.limits)
                send(connection, runtime.snapshot)
                continue
            if kind == "shutdown":
                if recording_manager is not None:
                    recording_manager.__exit__(
                        RuntimeError,
                        RuntimeError("shutdown interrupted acquisition"),
                        None,
                    )
                runtime.close()
                send(connection, dict(closed=True))
                break
            if kind == "snapshot":
                runtime.rpc(request)
                driver._sync()
                send(connection, runtime.snapshot)
                continue
            if kind in {
                "kill_controller",
                "restart_controller",
                "prepare_controller",
                "clock",
                "reset",
                "advance",
                "forward_jump",
            }:
                if kind == "reset" and recording_manager is not None:
                    raise ValueError("finish the recorded attempt before resetting")
                if kind == "reset":
                    # Reset application motion history together with authority,
                    # otherwise a new episode inherits old limiter velocities.
                    robot.reset(request["seed"])
                else:
                    runtime.rpc(request)
                    driver._sync()
                if kind == "reset":
                    inbox = CommandInbox()
                    latest = None
                    inbox.update_clock(
                        driver.observe().timestamp_ns + 1_000_000_000,
                        runtime.snapshot["episode"],
                        runtime.snapshot["steady_ns"],
                    )
                send(connection, runtime.snapshot)
                continue
            if kind != "poll":
                raise ValueError("unknown gateway request")
            if paced:
                runtime.rpc(dict(kind="snapshot"))
                driver._sync()
            # Consume authority progress before receiving a fresh command. A
            # long pause may clear old intent, never the newly offered sequence.
            inbox.update_clock(
                driver.observe().timestamp_ns + 1_000_000_000,
                runtime.snapshot["episode"],
                runtime.snapshot["steady_ns"],
            )
            if request.get("wait_delivery", True):
                # Prepare the executor before the producer creates a message
                # with a 100 ms DDS lifespan, including its first publication.
                if request.get("delivery_barrier"):
                    rclpy.spin_once(node, timeout_sec=0.0)
                before = arrivals
                if request.get("delivery_barrier"):
                    send(connection, dict(ready_delivery=True))
                end = time.monotonic() + 0.1
                while arrivals == before:
                    if time.monotonic() >= end:
                        inbox.reject("delivery_timeout")
                        break
                    rclpy.spin_once(node, timeout_sec=0.005)
            else:
                rclpy.spin_once(node, timeout_sec=0.0)
            observation = driver.observe()
            steady = runtime.snapshot["steady_ns"]
            inbox.update_clock(
                observation.timestamp_ns + 1_000_000_000,
                runtime.snapshot["episode"],
                steady,
            )
            driver.sequence = latest.sequence if latest else 0
            driver.receipt_ns = receipt
            driver.view_intent = None
            if latest is not None and observation_channel is not None:
                from act_lab.adapters.ros2.contracts import encode_pose

                driver.view_intent = dict(
                    sequence=latest.sequence,
                    source_episode_id=latest.episode_id,
                    requested_frame=latest.action.target_pose.frame_id,
                    source_ns=latest.action.timestamp_ns,
                    wall_receipt_ns=wall_receipt,
                    requested=diagnostic_json(encode_pose(latest.action.target_pose)),
                    approved=None,
                )
            state = robot.command(inbox.poll(observation, steady))
            report = robot.last_report
            assert report is not None
            fields = encode_report(report, runtime.snapshot["episode"], driver.sequence)
            report_pub.publish(make_message("CommandReport", fields))
            if recording_manager is not None:
                import json

                recording_sink.diagnostics(
                    state.timestamp_ns,
                    time.monotonic_ns(),
                    json.dumps(
                        dict(
                            transport_rejection=inbox.last_rejection_reason,
                            source_episode_id=latest.episode_id if latest else None,
                            source_sequence=latest.sequence if latest else None,
                            source_timestamp_ns=latest.action.timestamp_ns
                            if latest
                            else None,
                            mode=runtime.snapshot["mode"],
                            reason=runtime.snapshot["reason"],
                        ),
                        sort_keys=True,
                    ),
                    "{}",
                )
            send(
                connection,
                dict(
                    **runtime.snapshot,
                    report=diagnostic_json(fields),
                    transport_rejection=inbox.last_rejection_reason,
                    measured=asdict(state),
                ),
            )
    except BaseException as error:
        try:
            send(connection, dict(error=str(error)))
        except (BrokenPipeError, EOFError):
            pass
        raise
    finally:
        if recording_manager is not None:
            recording_sink.interrupt()
        if runtime.process.is_alive():
            runtime.close()
        driver.close()
        node.destroy_node()
        rclpy.try_shutdown()
        connection.close()
