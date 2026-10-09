"""Optional moving simulation process boundary. No ROS imports at module import."""

from __future__ import annotations

import json
import math
import multiprocessing as mp
import os
import select
import shutil
import subprocess
import time
from dataclasses import asdict
from multiprocessing.connection import Connection
from pathlib import Path
from typing import Any
from uuid import UUID, uuid4

from act_lab.adapters.ros2.contracts import JOINT_NAMES, decode_state, encode_state
from act_lab.adapters.ros2.simulation_guard import Authorization, SimulationGuard
from act_lab.domain import Pose

MAX_PACKET = 262144
CONTROLLER = (
    "/opt/simulation/install/act_lab_mujoco_system/lib/"
    "act_lab_mujoco_system/act_lab_controller"
)
CONFIG = Path("configs/sim/ur5e_pick_place.toml")


def diagnostic_json(value: Any) -> Any:
    """Represent rejected nonfinite numbers in artifacts, never in authorization."""
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {key: diagnostic_json(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [diagnostic_json(item) for item in value]
    return value


def send(connection: Connection, value: dict[str, Any]) -> None:
    data = json.dumps({**value, "schema_version": 1}, allow_nan=False).encode()
    if len(data) > MAX_PACKET:
        raise ValueError("simulation packet too large")
    connection.send_bytes(data)


def receive(connection: Connection, timeout: float = 20.0) -> dict[str, Any]:
    if not connection.poll(timeout):
        raise RuntimeError("bounded simulation IPC timeout")
    value = json.loads(connection.recv_bytes(MAX_PACKET))
    if not isinstance(value, dict) or value.get("schema_version") != 1:
        raise ValueError("simulation packet must be a version 1 object")
    return value


class ControllerProcess:
    def __init__(self, state: Any, output: Path) -> None:
        self.log = (output / "controller.log").open("a")
        self.process = subprocess.Popen(
            [CONTROLLER],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=self.log,
            bufsize=0,
        )
        self.buffer = b""
        self.request(dict(q=state.joint_positions_rad, joints=JOINT_NAMES))

    def request(self, value: dict[str, Any], on_wait: Any = None) -> dict[str, Any]:
        assert self.process.stdin and self.process.stdout
        self.process.stdin.write(
            json.dumps({**value, "schema_version": 1}, allow_nan=False).encode() + b"\n"
        )
        self.process.stdin.flush()
        end = time.monotonic() + (
            20.0 if "joints" in value or value.get("recover") else 0.12
        )
        while time.monotonic() < end:
            while b"\n" in self.buffer:
                line, self.buffer = self.buffer.split(b"\n", 1)
                try:
                    row = json.loads(line)
                except ValueError:
                    continue
                if isinstance(row, dict) and ("ready" in row or "effort" in row):
                    if row.get("schema_version") != 1:
                        raise RuntimeError("invalid controller protocol version")
                    return row
            if on_wait:
                on_wait()
            if select.select([self.process.stdout], [], [], 0.002)[0]:
                data = os.read(self.process.stdout.fileno(), 65536)
                if not data:
                    raise RuntimeError("controller process disappeared")
                self.buffer += data
                if len(self.buffer) > MAX_PACKET:
                    raise RuntimeError("controller packet/log overflow")
        raise RuntimeError("bounded controller response timeout")

    def close(self) -> None:
        if self.process.poll() is None:
            try:
                assert self.process.stdin is not None
                self.process.stdin.write(b'{"schema_version":1,"shutdown":true}\n')
                self.process.stdin.close()
                self.process.wait(timeout=2)
            except (BrokenPipeError, subprocess.TimeoutExpired):
                self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.log.close()


def physics_owner(
    connection: Connection, output: str, seed: int, paced: bool = False
) -> None:
    """The only process with live MuJoCo access; controller death leaves it running."""
    from act_lab.adapters.mujoco.config import SimulationConfig
    from act_lab.adapters.mujoco.effort import MujocoEffortPlant

    folder = Path(output)
    folder.mkdir(parents=True, exist_ok=True)
    plant = MujocoEffortPlant(SimulationConfig.load(CONFIG))
    plant.reset(seed)
    episode = uuid4()
    guard = SimulationGuard(episode)
    steady = 0
    last_authorization_wall = time.monotonic_ns()
    controller: ControllerProcess | None = None
    rows: list[dict[str, Any]] = []
    import rclpy  # type: ignore[import-not-found]
    from rclpy.parameter import Parameter  # type: ignore[import-not-found]

    from act_lab.adapters.ros2.contracts import TOPICS, encode_stamp
    from act_lab.adapters.ros2.runtime import make_message, qos_profiles

    rclpy.init()
    node = rclpy.create_node(
        "act_lab_physics_authority",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    qos = qos_profiles()
    state_pub = node.create_publisher(
        type(
            make_message(
                "RobotState", encode_state(plant.observe().robot, str(episode))
            )
        ),
        TOPICS["state"],
        qos["state"],
    )
    clock_pub = node.create_publisher(
        type(make_message("Clock", {"clock": encode_stamp(0)})),
        TOPICS["clock"],
        qos["clock"],
    )

    def publish() -> None:
        clock_pub.publish(
            make_message("Clock", {"clock": encode_stamp(plant.observe().timestamp_ns)})
        )
        if plant.environment.physics_ticks % 10 == 0:
            state_pub.publish(
                make_message(
                    "RobotState", encode_state(plant.observe().robot, str(episode))
                )
            )

    import numpy as np
    import pinocchio as pin  # type: ignore[import-not-found]

    ur_model = pin.buildModelFromUrdf("/opt/crisp/ur5e.urdf")
    ur_data = ur_model.createData()
    tool_id = ur_model.getFrameId("tool0")
    residual_m = 0.0
    residual_rad = 0.0
    generation_recovered = -1
    target: dict[str, Any] | None = None

    def snapshot() -> dict[str, Any]:
        return dict(
            steady_ns=steady,
            episode=str(episode),
            state=encode_state(plant.observe().robot, str(episode)),
            qpos=plant._data.qpos.tolist(),
            qvel=plant._data.qvel.tolist(),
            mode=plant.mode,
            reason=plant.reason,
            controller_pid=controller.process.pid if controller else None,
            physics_pid=os.getpid(),
        )

    def request(recover: bool = False) -> dict[str, Any]:
        state = plant.observe().robot
        value = dict(
            q=state.joint_positions_rad,
            dq=state.joint_velocities_rad_s,
            domain_ns=state.timestamp_ns,
            episode=str(episode),
            generation=guard.generation,
            tick=plant.environment.physics_ticks,
            recover=recover,
        )
        if target is not None:
            nonlocal residual_m, residual_rad
            import mujoco  # type: ignore[import-untyped]

            scene = state.end_effector_pose
            rotation = np.empty(9)
            mujoco.mju_quat2Mat(rotation, np.asarray(scene.quaternion_wxyz))
            a = np.diag([-1.0, -1.0, 1.0])
            r = a @ rotation.reshape(3, 3)
            p = a @ np.asarray(scene.position_xyz_m) - r @ np.array([0.0, 0.0, 0.11])
            mapped = pin.SE3(r, p)
            pin.forwardKinematics(
                ur_model, ur_data, np.asarray(state.joint_positions_rad)
            )
            pin.updateFramePlacements(ur_model, ur_data)
            actual = ur_data.oMf[tool_id]
            residual_m = float(np.linalg.norm(actual.translation - mapped.translation))
            residual_rad = float(
                np.linalg.norm(pin.log3(actual.rotation @ mapped.rotation.T))
            )
            if residual_m > 0.002 or residual_rad > 0.02:
                raise RuntimeError("moving frame residual exceeds reviewed bounds")
            raw_rotation = np.empty(9)
            mujoco.mju_quat2Mat(raw_rotation, np.asarray(target["quaternion_wxyz"]))
            desired = pin.SE3(
                raw_rotation.reshape(3, 3), np.asarray(target["position"])
            )
            corrected = actual * mapped.inverse() * desired
            quaternion = np.empty(4)
            mujoco.mju_mat2Quat(quaternion, np.asarray(corrected.rotation).reshape(9))
            value["target"] = dict(
                position=corrected.translation.tolist(),
                quaternion_wxyz=quaternion.tolist(),
                source_ns=target["source_ns"],
            )
        return value

    def inhibit(reason: str) -> None:
        guard.fault(reason)
        plant.hold(reason)

    try:
        publish()
        controller = ControllerProcess(plant.observe().robot, folder)
        # Lifecycle/DDS preparation happens under startup hold, before authorization.
        controller.request(request(True), publish)
        send(connection, snapshot())
        command: dict[str, Any]
        while True:
            if not connection.poll(0.0 if paced else 0.002):
                # Both schedulers share authorization and actuator state machines.
                if (
                    plant.mode == "ENABLED"
                    and time.monotonic_ns() - last_authorization_wall >= 100_000_000
                ):
                    inhibit("gateway_wall_timeout")
                if paced:
                    command = dict(kind="advance", ticks=1, internal=True)
                else:
                    if plant.reason == "gateway_wall_timeout":
                        rows.append(asdict(plant.tick()))
                        publish()
                        steady += 2_000_000
                    continue
            else:
                command = receive(connection)
            kind = command.get("kind")
            if kind == "shutdown":
                plant.hold("shutdown", shutdown=True)
                for _ in range(250):
                    rows.append(asdict(plant.tick()))
                send(connection, snapshot())
                break
            if kind == "snapshot":
                send(connection, snapshot())
                continue
            if kind == "kill_controller":
                controller.process.kill()
                controller.process.wait(timeout=5)
                send(connection, snapshot())
                continue
            if kind == "restart_controller":
                plant.hold("controller_restart")
                guard.fault("controller_restart")
                controller.close()
                controller = ControllerProcess(plant.observe().robot, folder)
                controller.request(request(True), publish)
                send(connection, snapshot())
                continue
            if kind == "reset":
                plant.reset(command["seed"])
                episode = uuid4()
                steady += 1
                guard.clock(0, episode, steady)
                target = None
                generation_recovered = -1
                send(connection, snapshot())
                continue
            if kind == "forward_jump":
                ticks = command["ticks"]
                if type(ticks) is not int or ticks <= 0:
                    raise ValueError("forward jump requires positive integer ticks")
                plant.environment._physics_ticks += ticks
                guard.clock(plant.observe().timestamp_ns, episode, steady)
                if not guard.allowed(steady):
                    plant.hold(guard.reason)
                publish()
                send(connection, snapshot())
                continue
            if kind == "clock":
                steady = command["steady_ns"]
                guard.clock(command["domain_ns"], UUID(command["episode"]), steady)
                if not guard.allowed(steady):
                    plant.hold(guard.reason)
                send(connection, snapshot())
                continue
            if kind not in {"execute", "advance"}:
                raise ValueError("unsupported simulation request")
            intent = command.get("authorization")
            guard.clock(plant.observe().timestamp_ns, episode, steady)
            if kind == "advance":
                pass
            elif intent is None:
                inhibit(command.get("reason", "disabled"))
            else:
                pose = Pose(
                    intent["frame"],
                    tuple(intent["position"]),
                    tuple(intent["quaternion_wxyz"]),
                )
                authorization = Authorization(
                    UUID(intent["episode"]),
                    intent["sequence"],
                    intent["source_ns"],
                    intent["receipt_ns"],
                    pose,
                    intent["gripper"],
                )
                if guard.authorize(authorization, steady):
                    # Fixed adapter transform: Rz(pi) and tip-to-tool Z offset.
                    import mujoco
                    import numpy as np

                    rotation = np.empty(9)
                    mujoco.mju_quat2Mat(rotation, np.asarray(pose.quaternion_wxyz))
                    a = np.diag([-1.0, -1.0, 1.0])
                    tool_rotation = a @ rotation.reshape(3, 3)
                    tool_position = a @ np.asarray(
                        pose.position_xyz_m
                    ) - tool_rotation @ np.array([0.0, 0.0, 0.11])
                    quaternion = np.empty(4)
                    mujoco.mju_mat2Quat(quaternion, tool_rotation.reshape(9))
                    target = dict(
                        position=tool_position.tolist(),
                        quaternion_wxyz=quaternion.tolist(),
                        source_ns=authorization.source_ns,
                    )
                    last_authorization_wall = time.monotonic_ns()
                else:
                    plant.hold(guard.reason)
            output_wall = [time.monotonic_ns()]

            def watchdog_wait(lease: list[int] = output_wall) -> None:
                nonlocal steady
                if time.monotonic_ns() - lease[0] >= 100_000_000:
                    inhibit("controller_wall_timeout")
                    rows.append(asdict(plant.tick()))
                    publish()
                    steady += 2_000_000

            for _ in range(command.get("ticks", 10)):
                tick_deadline = time.monotonic() + 0.002
                raw_effort: Any = None
                if (
                    guard.mode in {"ENABLED", "RECOVERING"}
                    and time.monotonic_ns() - last_authorization_wall >= 100_000_000
                ):
                    inhibit("gateway_wall_timeout")
                state = plant.observe().robot
                guard.clock(state.timestamp_ns, episode, steady)
                if guard.mode == "RECOVERING":
                    try:
                        assert controller is not None
                        response = controller.request(request(True), watchdog_wait)
                        if (
                            plant.reason != "controller_wall_timeout"
                            and response["generation"] == guard.generation
                        ):
                            generation_recovered = guard.generation
                            guard.reactivated(generation_recovered, steady)
                            output_wall[0] = time.monotonic_ns()
                    except (RuntimeError, BrokenPipeError):
                        inhibit("controller_lost")
                if guard.allowed(steady):
                    try:
                        assert controller is not None
                        response = controller.request(request(), watchdog_wait)
                        effort = tuple(response["effort"])
                        raw_effort = diagnostic_json(effort)
                        if guard.output(
                            UUID(response["episode"]),
                            response["generation"],
                            response["tick"],
                            effort,
                            steady,
                        ):
                            # Valid output advances the wall lease, just as it
                            # advances the injected steady-clock output lease.
                            output_wall[0] = time.monotonic_ns()
                            if plant.mode != "ENABLED":
                                plant.enable(guard.gripper)
                            else:
                                plant.accepted_gripper = guard.gripper
                            sample = plant.tick(effort)
                            if sample.mode != "ENABLED":
                                guard.fault(sample.reason)
                        else:
                            plant.hold(guard.reason)
                            sample = plant.tick()
                    except (
                        RuntimeError,
                        BrokenPipeError,
                        ValueError,
                        TypeError,
                        KeyError,
                    ):
                        inhibit("controller_lost")
                        sample = plant.tick()
                else:
                    plant.hold(guard.reason)
                    sample = plant.tick()
                rows.append(
                    dict(
                        **asdict(sample),
                        episode=str(episode),
                        steady_ns=steady,
                        frame_residual_m=residual_m,
                        frame_residual_rad=residual_rad,
                        generation=guard.generation,
                        source_timestamp_ns=guard._authorization.source_ns
                        if guard._authorization
                        else None,
                        command_sequence=guard._authorization.sequence
                        if guard._authorization
                        else None,
                        receipt_steady_ns=guard._authorization.receipt_ns
                        if guard._authorization
                        else None,
                        raw_effort_nm=raw_effort,
                    )
                )
                publish()
                steady += 2_000_000
                if paced:
                    remaining = tick_deadline - time.monotonic()
                    rows[-1]["paced_budget_overrun"] = remaining < 0
                    if remaining > 0:
                        time.sleep(remaining)
            if command.get("internal"):
                continue
            send(
                connection,
                dict(
                    **snapshot(),
                    generation=guard.generation,
                    recovered_generation=generation_recovered,
                    guard_reason=guard.reason,
                ),
            )
    except EOFError:
        # Gateway process death cannot execute its cleanup. The owner can.
        plant.hold("gateway_disconnected")
        for _ in range(250):
            rows.append(asdict(plant.tick()))
    except BaseException as error:
        plant.hold("owner_error")
        for _ in range(250):
            rows.append(asdict(plant.tick()))
        try:
            send(connection, dict(error=str(error)))
        except (BrokenPipeError, EOFError):
            pass
        raise
    finally:
        (folder / "physics.json").write_text(
            json.dumps(rows, indent=2, allow_nan=False) + "\n"
        )
        if controller:
            controller.close()
        node.destroy_node()
        rclpy.shutdown()
        plant.close()
        connection.close()


class SimulationProcess:
    def __init__(self, output: Path, seed: int = 0, paced: bool = False) -> None:
        if not Path(CONTROLLER).exists() or shutil.which("ros2") is None:
            raise RuntimeError(
                "ROS simulation requires: docker compose --profile ros2-simulation "
                "run --build --rm ros2-simulation"
            )
        context = mp.get_context("spawn")
        self.connection, child = context.Pipe()
        self.process = context.Process(
            target=physics_owner, args=(child, str(output), seed, paced)
        )
        self.process.start()
        child.close()
        try:
            self.snapshot = receive(self.connection)
            if "error" in self.snapshot:
                raise RuntimeError(self.snapshot["error"])
        except BaseException:
            self.process.terminate()
            self.process.join(timeout=5)
            self.connection.close()
            raise

    def rpc(self, command: dict[str, Any]) -> dict[str, Any]:
        send(self.connection, command)
        value = receive(self.connection)
        if "error" in value:
            raise RuntimeError(value["error"])
        self.snapshot = value
        return value

    def close(self) -> None:
        if self.process.is_alive():
            try:
                self.rpc(dict(kind="shutdown"))
                self.process.join(timeout=5)
            except (RuntimeError, EOFError, BrokenPipeError):
                self.process.terminate()
                self.process.join(timeout=5)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=5)
            raise RuntimeError("physics shutdown timeout")
        self.connection.close()


def snapshot_state(value: dict[str, Any]) -> Any:
    return decode_state(value["state"])[0]
