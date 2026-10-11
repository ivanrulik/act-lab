"""Executable ROS/CRISP motion evidence, with a separate DDS command gateway."""

from __future__ import annotations

import json
import math
import multiprocessing as mp
import os
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

from act_lab.adapters.ros2.contracts import (
    TOPICS,
    CommandEnvelope,
    encode_command,
)
from act_lab.adapters.ros2.runtime import make_message, qos_profiles, require_ros
from act_lab.adapters.ros2.simulation import (
    CONFIG,
    CONTROLLER,
    diagnostic_json,
    receive,
    send,
    snapshot_state,
)
from act_lab.domain import Action, Pose


class MotionSession:
    def __init__(
        self,
        output: Path,
        seed: int = 0,
        paced: bool = False,
        observation_channel: Any = None,
        config_path: Path = CONFIG,
    ) -> None:
        try:
            require_ros()
        except RuntimeError as error:
            raise RuntimeError(
                "ROS simulation requires docker compose --profile ros2-simulation "
                "run --build --rm ros2-simulation"
            ) from error
        if not Path(CONTROLLER).exists():
            raise RuntimeError(
                "Run docker compose --profile ros2-simulation "
                "run --build --rm ros2-simulation"
            )
        import rclpy  # type: ignore[import-not-found]
        from rclpy.parameter import Parameter  # type: ignore[import-not-found]

        from act_lab.adapters.ros2.simulation_gateway import gateway

        self.paced = paced
        self.config_path = config_path
        self.output = output
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node(
            "act_lab_motion_producer",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.pub = self.node.create_publisher(
            type(make_message("CartesianCommand", {})),
            TOPICS["command"],
            qos_profiles()["command"],
        )
        context = mp.get_context("spawn")
        self.connection, child = context.Pipe()
        self.process = context.Process(
            target=gateway,
            args=(child, str(output), seed, paced, observation_channel, config_path),
        )
        self.process.start()
        child.close()
        try:
            self.value = receive(self.connection)
            if "error" in self.value:
                raise RuntimeError(self.value["error"])
        except BaseException:
            self.process.terminate()
            self.process.join(timeout=5)
            self.connection.close()
            self.node.destroy_node()
            rclpy.shutdown()
            raise
        self.telemetry = {"state": 0, "report": 0, "clock": 0}
        for name, message in (
            ("state", "RobotState"),
            ("report", "CommandReport"),
            ("clock", "Clock"),
        ):

            def seen(value: Any, key: str = name) -> None:
                self.telemetry[key] += 1

            self.node.create_subscription(
                type(make_message(message, {})),
                TOPICS[name],
                seen,
                qos_profiles()[name],
            )
        self.sequence = 0
        self.trace: list[dict[str, Any]] = []
        end = time.monotonic() + 15
        while self.pub.get_subscription_count() < 1:
            if time.monotonic() >= end:
                self.close()
                raise RuntimeError("bounded command publisher discovery timeout")
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def rpc(self, request: dict[str, Any]) -> dict[str, Any]:
        send(self.connection, request)
        value = receive(self.connection)
        if "error" in value:
            raise RuntimeError(value["error"])
        self.value = value
        return value

    def command(
        self,
        pose: Pose,
        gripper: float = 0.0,
        enabled: bool = True,
        *,
        stamp: int | None = None,
        replay: bool = False,
        episode: str | None = None,
    ) -> dict[str, Any]:
        # The independent owner may integrate hold while this producer is
        # descheduled, including in stepped mode. Sample current authority
        # before creating a new intent; never restamp an existing command.
        self.rpc(dict(kind="snapshot"))
        state = snapshot_state(self.value)
        if not replay:
            self.sequence += 1
        action = Action(
            state.timestamp_ns if stamp is None else stamp, pose, gripper, enabled
        )
        envelope = CommandEnvelope(
            episode or self.value["episode"], self.sequence, action
        )
        # Ask the consuming gateway to wait for this DDS message before polling.
        send(
            self.connection,
            dict(kind="poll", wait_delivery=True, delivery_barrier=True),
        )
        ready = receive(self.connection)
        if ready.get("ready_delivery") is not True:
            raise RuntimeError("gateway did not establish DDS delivery barrier")
        self.pub.publish(make_message("CartesianCommand", encode_command(envelope)))
        value = receive(self.connection)
        if "error" in value:
            raise RuntimeError(value["error"])
        self.value = value
        self.rclpy.spin_once(self.node, timeout_sec=0.0)
        self.trace.append(
            dict(
                sequence=self.sequence,
                episode=envelope.episode_id,
                transport_rejection=value.get("transport_rejection"),
                source_timestamp_ns=action.timestamp_ns,
                requested=diagnostic_json(pose.position_xyz_m),
                quaternion_wxyz=pose.quaternion_wxyz,
                enabled=enabled,
                gripper=gripper,
                measured=snapshot_state(value).end_effector_pose.position_xyz_m,
                outcome=value["report"]["outcome"],
                mode=value["mode"],
                reason=value["reason"],
            )
        )
        return value

    def motion_command(
        self, pose: Pose, gripper: float = 0.0, enabled: bool = True
    ) -> dict[str, Any]:
        """Issue fresh intents through bounded recovery after recorded wall faults."""
        if not enabled:
            return self.command(pose, gripper, False)
        deadline = time.monotonic() + 15
        while True:
            value = self.command(pose, gripper)
            if value["mode"] == "ENABLED" or (
                value["reason"] == "acquisition_pause"
                and value.get("acquisition_execution_mode") == "ENABLED"
            ):
                return value
            missing_delivery = value.get("transport_rejection") == "delivery_timeout"
            execution_reason = value.get(
                "acquisition_execution_reason", value["reason"]
            )
            if not missing_delivery and execution_reason not in {
                "controller_wall_timeout",
                "gateway_wall_timeout",
                "clock_paused",
                "stale_source",
            }:
                raise RuntimeError(f"nominal motion rejected: {execution_reason}")
            self.trace[-1]["wall_fault_recovery"] = not missing_delivery
            self.trace[-1]["transport_loss_recovery"] = missing_delivery
            if time.monotonic() >= deadline:
                raise RuntimeError("bounded nominal fresh-command recovery timeout")
            self.rpc(dict(kind="prepare_controller"))

    def close(self) -> None:
        if getattr(self, "closed", False):
            return
        self.closed = True
        try:
            if self.process.is_alive():
                try:
                    send(self.connection, dict(kind="shutdown"))
                    reply = receive(self.connection, timeout=75.0)
                    if reply.get("closed") is not True:
                        raise RuntimeError(
                            reply.get("error", "gateway shutdown failed")
                        )
                    self.process.join(timeout=5)
                except (RuntimeError, EOFError, OSError):
                    self.process.terminate()
                    self.process.join(timeout=5)
                    if self.process.is_alive():
                        self.process.kill()
                        self.process.join(timeout=5)
                    raise
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=5)
                raise RuntimeError("gateway shutdown timeout")
            if self.process.exitcode != 0:
                raise RuntimeError(f"gateway exited with code {self.process.exitcode}")
        finally:
            self.connection.close()
            self.node.destroy_node()
            self.rclpy.try_shutdown()


def producer_loss(session: MotionSession) -> dict[str, Any]:
    """Stop producing intent; whichever independent lease expires first wins."""
    if session.value["mode"] != "ENABLED":
        raise RuntimeError("producer-loss fixture requires enabled intent")
    source_ns = session.trace[-1]["source_timestamp_ns"]
    sequence = session.sequence
    value = session.rpc(dict(kind="advance", ticks=250))
    age_ns = snapshot_state(value).timestamp_ns - source_ns
    if (
        value["mode"] != "FAULT_HOLD"
        or value["reason"]
        not in {"stale_source", "gateway_wall_timeout", "controller_wall_timeout"}
        or age_ns < 100_000_000
        or session.sequence != sequence
    ):
        raise RuntimeError("producer disappearance did not inhibit expired intent")
    return dict(
        case="producer_loss",
        reason=value["reason"],
        source_timestamp_ns=source_ns,
        final_source_age_ns=age_ns,
        command_sequence=sequence,
    )


def run_motion(
    output: Path, *, paced: bool = False, config_path: Path = CONFIG
) -> dict[str, Any]:
    output.mkdir(parents=True, exist_ok=True)
    session = MotionSession(output, paced=paced, config_path=config_path)
    from act_lab.adapters.mujoco import SimulationConfig

    config = SimulationConfig.load(config_path)
    controller_config = Path(
        "configs/ros2/crisp-tooling.yaml"
        if config.model_id == "ur5e_2f85_d405_v1"
        else "configs/ros2/crisp-simulation.yaml"
    )
    cases: list[dict[str, Any]] = []
    try:
        initial = snapshot_state(session.value)
        target = replace(
            initial.end_effector_pose,
            position_xyz_m=(
                initial.end_effector_pose.position_xyz_m[0] - 0.01,
                *initial.end_effector_pose.position_xyz_m[1:],
            ),
        )
        # The same measured home, source periods and intents are used locally.
        from act_lab.adapters.mujoco.cartesian_driver import MujocoCartesianDriver
        from act_lab.application import SafeCartesianRobot

        local_driver = MujocoCartesianDriver.from_config_file(config_path)
        local_robot = SafeCartesianRobot(local_driver, local_driver.limits)
        local_robot.reset(0)
        fixtures = (
            ("home", (0.0, 0.0, 0.0)),
            ("nearby_x", (0.005, 0.0, 0.005)),
            ("nearby_y", (0.0, 0.005, 0.005)),
        )
        sweep = (
            (name, offset, axis, direction)
            for name, offset in fixtures
            for axis, direction in ((0, -1), (0, 1), (1, -1), (1, 1), (2, -1), (2, 1))
        )
        first_trajectory: list[dict[str, Any]] = []
        first_trace_end = 0
        for fixture, offset, axis, direction in sweep:
            position = [
                v + delta
                for v, delta in zip(
                    initial.end_effector_pose.position_xyz_m, offset, strict=True
                )
            ]
            position[axis] += direction * 0.01
            target = replace(initial.end_effector_pose, position_xyz_m=tuple(position))
            motion_start_ns = snapshot_state(session.value).timestamp_ns
            for _ in range(250):
                value = session.motion_command(target)
                if fixture == "home" and axis == 0 and direction == -1:
                    first_trajectory.append(session.trace[-1])
                    first_trace_end = len(session.trace)
                local_robot.command(
                    Action(local_robot.observe().timestamp_ns, target, 0.0, True)
                )
                local_state = local_robot.observe().robot
                assert local_robot.last_report is not None
                session.trace[-1]["local"] = dict(
                    timestamp_ns=local_state.timestamp_ns,
                    measured_pose=local_state.end_effector_pose.position_xyz_m,
                    measured_quaternion_wxyz=local_state.end_effector_pose.quaternion_wxyz,
                    joint_positions_rad=local_state.joint_positions_rad,
                    gripper=local_state.gripper_position,
                    outcome=local_robot.last_report.outcome.value,
                )
                if value["mode"] != "ENABLED":
                    raise RuntimeError(
                        f"motion inhibited on axis {axis}: {value['reason']}"
                    )
            state = snapshot_state(session.value)
            error = math.dist(
                target.position_xyz_m, state.end_effector_pose.position_xyz_m
            )
            baseline = math.dist(
                target.position_xyz_m,
                local_robot.observe().robot.end_effector_pose.position_xyz_m,
            )
            if error > 0.002 or baseline > 0.002:
                raise RuntimeError(
                    f"motion convergence failed: ROS={error:.6f}, "
                    f"local={baseline:.6f} m"
                )
            cases.append(
                dict(
                    case="translation",
                    axis=axis,
                    direction=direction,
                    error_m=error,
                    local_error_m=baseline,
                    starting_fixture=fixture,
                    fixture_offset_xyz_m=offset,
                    settling_window_s=(state.timestamp_ns - motion_start_ns) / 1e9,
                    enabled_updates=250,
                )
            )
            if axis == 2 and direction == 1:
                orientation, angular_error = orientation_trial(
                    session, target, local_robot
                )
                cases.append(
                    dict(case="orientation", fixture=fixture, error_rad=angular_error)
                )
        local_driver.close()
        for gripper in (0.8, 0.0):
            # Allow the configured aperture rate plus physical linkage settling.
            # The articulated preset deliberately closes slower than the slides.
            gripper_updates = max(
                30,
                math.ceil(
                    (0.8 / config.control.max_gripper_velocity_s + 0.4)
                    * config.environment_hz
                ),
            )
            for _ in range(gripper_updates):
                session.motion_command(orientation, gripper=gripper)
            if abs(snapshot_state(session.value).gripper_position - gripper) > 0.02:
                raise RuntimeError("atomic gripper command failed")
        cases.append(dict(case="gripper_open_close", status="passed"))
        state = snapshot_state(session.value)
        for case, kwargs in (
            ("disabled", dict(enabled=False, gripper=1.0)),
            ("future", dict(stamp=state.timestamp_ns + 1_000_000_000)),
            ("stale", dict(stamp=state.timestamp_ns - 100_000_000)),
            ("invalid_frame", dict(pose=replace(target, frame_id="camera"))),
            (
                "invalid_numeric",
                dict(pose=replace(target, position_xyz_m=(float("nan"), 0.0, 0.6))),
            ),
            (
                "invalid_quaternion",
                dict(pose=replace(target, quaternion_wxyz=(2.0, 0.0, 0.0, 0.0))),
            ),
            ("replay", dict(replay=True)),
            ("wrong_episode", dict(episode="00000000-0000-0000-0000-000000000002")),
        ):
            value = session.command(kwargs.pop("pose", target), **kwargs)
            if value.get("transport_rejection") == "delivery_timeout":
                raise RuntimeError(f"fault case missing DDS delivery: {case}")
            if value["mode"] == "ENABLED":
                raise RuntimeError(f"fault failed to transfer ownership: {case}")
            cases.append(
                dict(case=case, mode=value["mode"], outcome=value["report"]["outcome"])
            )
            session.motion_command(target)
        session.motion_command(target)
        episode_before_jump = session.value["episode"]
        value = session.rpc(dict(kind="forward_jump", ticks=100))
        if value["mode"] != "FAULT_HOLD" or value["episode"] != episode_before_jump:
            raise RuntimeError(
                "forward jump did not age source within the same episode"
            )
        cases.append(dict(case="clock_forward_jump", mode=value["mode"]))
        session.motion_command(target)
        session.motion_command(target)
        cases.append(producer_loss(session))
        session.motion_command(target)
        value = session.rpc(
            dict(
                kind="clock",
                domain_ns=snapshot_state(session.value).timestamp_ns,
                steady_ns=session.value["steady_ns"] + 100_000_000,
                episode=session.value["episode"],
            )
        )
        if value["mode"] != "FAULT_HOLD":
            raise RuntimeError("paused clock did not hold")
        cases.append(dict(case="clock_pause", mode=value["mode"]))
        session.motion_command(target)
        session.motion_command(target)
        if session.value["mode"] != "ENABLED":
            raise RuntimeError("fresh sequence after resumed clock did not recover")
        session.rpc(
            dict(
                kind="clock",
                domain_ns=0,
                steady_ns=session.value["steady_ns"] + 1,
                episode=session.value["episode"],
            )
        )
        value = session.command(target)
        if value["mode"] == "ENABLED":
            raise RuntimeError("backward clock recovered without a new episode")
        cases.append(dict(case="clock_reversal", mode=value["mode"]))
        session.rpc(dict(kind="reset", seed=0))
        session.sequence = 0
        target = snapshot_state(session.value).end_effector_pose
        session.motion_command(target)
        session.rpc(dict(kind="kill_controller"))
        value = session.command(target)
        if value["mode"] != "FAULT_HOLD":
            raise RuntimeError("controller death did not hold")
        cases.append(dict(case="controller_loss", mode=value["mode"]))
        session.rpc(dict(kind="restart_controller"))
        recovered = session.motion_command(target)
        if recovered["mode"] != "ENABLED":
            raise RuntimeError(
                "fresh authorization did not recover restarted controller"
            )
        cases.append(dict(case="controller_recovery", mode=recovered["mode"]))
        before_episode = session.value["episode"]
        session.rpc(dict(kind="reset", seed=0))
        if session.value["episode"] == before_episode:
            raise RuntimeError("reset failed to change episode")
        session.sequence = 0
        session.motion_command(snapshot_state(session.value).end_effector_pose)
        cases.append(dict(case="reset_recovery", mode=session.value["mode"]))
        repeated_start = len(session.trace)
        repeated_target = replace(
            initial.end_effector_pose,
            position_xyz_m=(
                initial.end_effector_pose.position_xyz_m[0] - 0.01,
                *initial.end_effector_pose.position_xyz_m[1:],
            ),
        )
        session.rpc(dict(kind="reset", seed=0))
        session.sequence = 0
        repeated_trajectory: list[dict[str, Any]] = []
        for _ in range(250):
            session.motion_command(repeated_target)
            repeated_trajectory.append(session.trace[-1])
        repeat_error = max(
            math.dist(a["measured"], b["measured"])
            for a, b in zip(first_trajectory, repeated_trajectory, strict=True)
        )
        # Wall watchdog events are external inputs even in stepped mode. Keep
        # the physical comparison tolerance authoritative; report bitwise
        # repeatability separately rather than treating faulted traces as
        # identical clock inputs.
        if repeat_error > 0.002:
            raise RuntimeError(f"same-seed trajectory diverged: {repeat_error:.12f} m")
        cases.append(
            dict(
                case="repeatability",
                maximum_difference_m=repeat_error,
                deterministic_stepped=not paced,
                bitwise_repeat=repeat_error <= 1e-9,
                comparison_tolerance_m=0.002,
                first_trace_holds=[
                    dict(sequence=row["sequence"], reason=row["reason"])
                    for row in session.trace[:first_trace_end]
                    if row["mode"] != "ENABLED"
                ],
                repeated_trace_holds=[
                    dict(sequence=row["sequence"], reason=row["reason"])
                    for row in session.trace[repeated_start:]
                    if row["mode"] != "ENABLED"
                ],
            )
        )
        cases.append(grasp_lift_hold(session))
        for _ in range(10):
            session.rclpy.spin_once(session.node, timeout_sec=0.005)
        if not all(session.telemetry.values()):
            raise RuntimeError(f"missing generated DDS telemetry: {session.telemetry}")
        import hashlib
        import platform
        import sys

        controller_pid = session.value["controller_pid"]
        physics_pid = session.value["physics_pid"]
        session.close()
        physics = json.loads((output / "physics.json").read_text())
        max_task = max(abs(t) for row in physics for t in row["task_effort_nm"])
        max_total = max(abs(t) for row in physics for t in row["total_effort_nm"])
        maxima = {
            key: max(row[key] for row in physics)
            for key in (
                "cartesian_speed_m_s",
                "cartesian_acceleration_m_s2",
                "angular_speed_rad_s",
                "angular_acceleration_rad_s2",
                "maximum_joint_acceleration_rad_s2",
            )
        }
        maxima["maximum_joint_speed_rad_s"] = max(
            abs(v) for row in physics for v in row["state"]["joint_velocities_rad_s"]
        )
        for key, bound in (
            ("maximum_joint_speed_rad_s", 1.0),
            ("cartesian_speed_m_s", 0.25),
            ("cartesian_acceleration_m_s2", 1.0),
            ("angular_speed_rad_s", 1.0),
            ("angular_acceleration_rad_s2", 4.0),
            ("maximum_joint_acceleration_rad_s2", 4.0),
        ):
            if maxima[key] > bound + 1e-9:
                raise RuntimeError(
                    f"measured motion limit exceeded: {key}={maxima[key]}"
                )
        for row in physics:
            if row["mode"] != "ENABLED" and any(row["task_effort_nm"]):
                raise RuntimeError("task effort present during hold")
            if row["mode"] == "ENABLED" and any(row["servo_effort_nm"]):
                raise RuntimeError("competing servo during enabled effort")
            for t, bound in zip(
                row["total_effort_nm"], (150, 150, 150, 28, 28, 28), strict=True
            ):
                if abs(t) > bound + 1e-9:
                    raise RuntimeError("total actuator ceiling exceeded")
        if max_task > 5.0 + 1e-9:
            raise RuntimeError("task effort ceiling exceeded")
        report = dict(
            schema_version=1,
            scheduler="paced" if paced else "stepped",
            status="completed",
            cases=cases,
            trace=session.trace,
            producer_pid=os.getpid(),
            gateway_pid=session.process.pid,
            controller_pid=controller_pid,
            physics_pid=physics_pid,
            physics_ticks=len(physics),
            resolved_actuation=dict(
                physics_period_s=0.002,
                application_period_s=0.02,
                task_ceiling_nm=5.0,
                task_slew_nm_s=100.0,
                hold_damping_nm_s_rad=[30.0, 30.0, 30.0, 6.0, 6.0, 6.0],
                hold_ceiling_nm=[150.0, 150.0, 150.0, 28.0, 28.0, 28.0],
                watchdog_ns=100_000_000,
            ),
            measured_maxima=maxima,
            maximum_task_effort_nm=max_task,
            maximum_total_actuator_effort_nm=max_total,
            telemetry=session.telemetry,
            provenance=dict(
                python=sys.version,
                machine=platform.machine(),
                platform=platform.platform(),
                ros_distro=os.environ.get("ROS_DISTRO"),
                rmw=os.environ.get("RMW_IMPLEMENTATION"),
                configuration_sha256=hashlib.sha256(
                    config_path.read_bytes()
                ).hexdigest(),
                scene_hashes={
                    str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in sorted(
                        Path("src/act_lab/adapters/mujoco/assets").rglob("*.xml")
                    )
                },
                controller_configuration_sha256=hashlib.sha256(
                    controller_config.read_bytes()
                ).hexdigest(),
                source_hashes={
                    label: {
                        str(path.relative_to(root)): hashlib.sha256(
                            path.read_bytes()
                        ).hexdigest()
                        for path in sorted(root.rglob(pattern))
                    }
                    for label, root, pattern in (
                        ("application", Path("src/act_lab"), "*.py"),
                        (
                            "compiled_controller_source",
                            Path("/opt/simulation/src/act_lab_mujoco_system"),
                            "*.*",
                        ),
                    )
                },
                upstream_crisp_commit="0279dc8f1196dab6a2b2cd5af853dd3d4dc3a969",
                upstream_description_commit="6b639c2efd9f4f12da858a72cf1a9e40da365ed8",
                compiler=Path("/opt/crisp/compiler.txt").read_text(),
                build_flags="Release; ENABLE_NATIVE_OPTIMIZATION=OFF",
                installed_packages=Path("/opt/simulation/packages.txt").read_text(),
                python_packages=Path("/opt/simulation/python-packages.txt").read_text(),
                resolved_controller_configuration=controller_config.read_text(),
                ur_model_sha256=hashlib.sha256(
                    Path("/opt/crisp/ur5e.urdf").read_bytes()
                ).hexdigest(),
            ),
            hardware_approved=False,
            limitations=["stepped simulation; not hardware realtime evidence"],
        )
        (output / "report.json").write_text(
            json.dumps(report, indent=2, allow_nan=False) + "\n"
        )
        return report
    except BaseException as error:
        (output / "failure.json").write_text(
            json.dumps(
                dict(
                    status="failed", error=str(error), cases=cases, trace=session.trace
                ),
                indent=2,
                allow_nan=False,
            )
            + "\n"
        )
        raise
    finally:
        session.close()


def grasp_lift_hold(session: MotionSession) -> dict[str, Any]:
    """A privileged scripted source uses snapshots; all motion still travels DDS."""
    import mujoco  # type: ignore[import-untyped]

    from act_lab.adapters.mujoco.config import SimulationConfig
    from act_lab.adapters.mujoco.environment import MujocoUR5eEnvironment
    from act_lab.application.scripted_expert import ExpertPhase, ScriptedPickPlaceExpert
    from act_lab.domain import Observation

    session.rpc(dict(kind="reset", seed=0))
    session.sequence = 0
    config = SimulationConfig.load(session.config_path)
    environment = MujocoUR5eEnvironment(config)
    environment.reset(0)
    expert = ScriptedPickPlaceExpert(environment, config.expert)
    state = snapshot_state(session.value)
    expert.reset(Observation(state.timestamp_ns, state, config.cameras))
    source_gripper = state.gripper_position
    try:
        for _step in range(2000):
            environment._data.qpos[:] = session.value["qpos"]  # noqa: SLF001
            environment._data.qvel[:] = session.value["qvel"]  # noqa: SLF001
            mujoco.mj_forward(environment._model, environment._data)  # noqa: SLF001
            state = snapshot_state(session.value)
            action = expert.poll(Observation(state.timestamp_ns, state, config.cameras))
            if environment.binding.adaptive_gripper:
                # Qualified contact approach for the effort-controlled tool.
                # Faster inputs remain subject to the independent physics guard.
                source_gripper += min(
                    max(
                        action.gripper_position - source_gripper,
                        -0.1 / config.environment_hz,
                    ),
                    0.1 / config.environment_hz,
                )
            else:
                source_gripper = action.gripper_position
            value = session.motion_command(
                action.target_pose, source_gripper, action.enabled
            )
            if value["mode"] != "ENABLED":
                raise RuntimeError(
                    f"grasp motion inhibited at {expert.phase}: {value['reason']}"
                )
            if expert.phase is ExpertPhase.TRANSIT:
                break
        else:
            raise RuntimeError(f"CRISP grasp/lift did not complete: {expert.phase}")
        cube_address = int(environment._model.jnt_qposadr[environment._cube_joint_id])  # noqa: SLF001
        cube_z = session.value["qpos"][cube_address + 2]
        if cube_z < config.cube_center_z_m + 0.04:
            raise RuntimeError("grasp/lift did not lift the cube")
        before = snapshot_state(session.value).end_effector_pose
        aperture = session.value["report"]["executed_command"]["gripper_position"]
        session.command(before, gripper=1.0, enabled=False)
        hold_ticks = 1000 if environment.binding.adaptive_gripper else 250
        session.rpc(dict(kind="advance", ticks=hold_ticks))
        after = snapshot_state(session.value).end_effector_pose
        drift = math.dist(before.position_xyz_m, after.position_xyz_m)
        if drift > 0.002 or session.value["qpos"][cube_address + 2] < cube_z - 0.01:
            raise RuntimeError(f"CRISP loaded hold failed: {drift:.6f} m")
        session.motion_command(after, gripper=aperture)
        if session.value["mode"] != "ENABLED":
            raise RuntimeError("loaded fresh-command recovery failed")
        result = dict(
            case="grasp_lift_hold_resume",
            application_steps=_step + 1,
            cube_lift_m=cube_z - config.cube_center_z_m,
            hold_drift_m=drift,
            accepted_gripper=aperture,
            hold_duration_s=hold_ticks / config.physics_hz,
            source_gripper_rate_s=0.1 if environment.binding.adaptive_gripper else None,
        )
        (session.output / "loaded-hold.json").write_text(
            json.dumps(result, indent=2) + "\n"
        )
        return result
    finally:
        environment.close()


def orientation_trial(
    session: MotionSession, target: Pose, local_robot: Any = None
) -> tuple[Pose, float]:
    from act_lab.application.cartesian_control import (
        _apply_rotation,
        _quaternion_error_vector,
    )

    orientation = replace(
        target,
        quaternion_wxyz=_apply_rotation(target.quaternion_wxyz, (0.0, 0.0, 0.01)),
    )
    initial = snapshot_state(session.value).end_effector_pose
    for _ in range(250):
        result = session.motion_command(orientation)
        if result["mode"] != "ENABLED":
            raise RuntimeError("orientation motion unexpectedly inhibited")
        if local_robot is not None:
            local_robot.command(
                Action(local_robot.observe().timestamp_ns, orientation, 0.0, True)
            )
            state = local_robot.observe().robot
            session.trace[-1]["local"] = dict(
                timestamp_ns=state.timestamp_ns,
                measured_pose=state.end_effector_pose.position_xyz_m,
                measured_quaternion_wxyz=state.end_effector_pose.quaternion_wxyz,
                gripper=state.gripper_position,
                outcome=local_robot.last_report.outcome.value,
            )
    measured = snapshot_state(session.value).end_effector_pose
    error = math.sqrt(
        sum(
            v * v
            for v in _quaternion_error_vector(
                orientation.quaternion_wxyz, measured.quaternion_wxyz
            )
        )
    )
    moved = math.sqrt(
        sum(
            v * v
            for v in _quaternion_error_vector(
                measured.quaternion_wxyz, initial.quaternion_wxyz
            )
        )
    )
    if error > 0.02 or moved < 0.001:
        raise RuntimeError("orientation convergence failed")
    return orientation, error
