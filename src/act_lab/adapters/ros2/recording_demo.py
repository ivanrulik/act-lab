"""Stepped or bounded paced capture, with canonical transport equivalence."""

from __future__ import annotations

import argparse
import hashlib
import json
import multiprocessing as mp
import os
import platform
import time
import tomllib
from importlib.metadata import version
from pathlib import Path
from typing import Any

from act_lab.adapters.ros2.recording import (
    RosEpisodeSink,
    import_bag,
    record_process,
    require_recording,
)

CONFIG = Path("configs/sim/ur5e_2f85_d405.toml")


class Recorder:
    def __init__(self, output: Path, drop_ack_once: int | None = None) -> None:
        context = mp.get_context("spawn")
        self.connection, child = context.Pipe()
        self.process = context.Process(
            target=record_process, args=(child, str(output), drop_ack_once)
        )
        self.process.start()
        child.close()
        if not self.connection.poll(30):
            self.process.terminate()
            self.process.join(5)
            raise RuntimeError("recorder startup timeout")
        try:
            self.ready = self.connection.recv()
        except EOFError as error:
            self.process.join(5)
            self.connection.close()
            raise RuntimeError(
                "recorder failed during startup; inspect its traceback"
            ) from error
        if not self.ready.get("ready"):
            raise RuntimeError(str(self.ready))

    def close(self, finalize: bool = True) -> dict[str, Any]:
        try:
            self.connection.send(dict(finalize=finalize))
            if not self.connection.poll(30):
                raise RuntimeError("recorder finalization timeout")
            result: dict[str, Any] = self.connection.recv()
            self.process.join(5)
            if self.process.exitcode != 0 or "error" in result:
                raise RuntimeError(str(result))
            return result
        finally:
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(5)
            self.connection.close()


def _local(output: Path, config: Path, seed: int, steps: int) -> dict[str, Any]:
    import rclpy  # type: ignore[import-not-found]

    from act_lab.adapters.mcap.recording import McapEpisodeSink
    from act_lab.adapters.mcap.session import recording_robot
    from act_lab.adapters.mujoco import MujocoCartesianDriver
    from act_lab.application import ScriptedPickPlaceExpert

    rclpy.init()
    from rclpy.parameter import Parameter  # type: ignore[import-not-found]

    node = rclpy.create_node(
        "act_lab_acquisition_producer",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    args = argparse.Namespace(
        record_dir=output / "local", outcome="auto", reason="", operator=""
    )
    sink = RosEpisodeSink(
        node, lambda episode: McapEpisodeSink(output / "local", episode)
    )
    try:
        with (
            MujocoCartesianDriver.from_config_file(config) as driver,
            recording_robot(
                driver, args, "expert", seed, sink_factory=lambda _: sink
            ) as robot,
        ):
            observation = robot.reset(seed)
            expert = ScriptedPickPlaceExpert(
                driver.environment, driver.simulation_config.expert
            )
            expert.reset(observation)
            for _ in range(steps or driver.simulation_config.episode_steps):
                robot.command(expert.poll(observation))
                assert robot.last_report is not None
                expert.record_command_report(robot.last_report)
                observation = robot.observe()
                if driver.environment.task_state().terminal or expert.failure_reason:
                    robot.command(expert.poll(observation))
                    break
        return dict(
            episode_id=sink.episode_id,
            producer_pid=os.getpid(),
            controller="local",
            packets=sink.sequence,
        )
    finally:
        sink.interrupt()
        node.destroy_node()
        rclpy.try_shutdown()


def _expert_task(
    session: Any,
    config: Path,
    steps: int,
    source_hz: float = 25.0,
    *,
    stop_phase: str | None = None,
) -> dict[str, Any]:
    from dataclasses import asdict

    from act_lab.adapters.mujoco.config import SimulationConfig
    from act_lab.adapters.ros2.contracts import decode_report
    from act_lab.adapters.ros2.simulation import snapshot_state
    from act_lab.adapters.ros2.task import (
        ApproachRoute,
        ExpertTaskSource,
        GripperRamp,
        OpeningPoseHold,
        descent_action,
        loaded_transit_action,
    )
    from act_lab.application.scripted_expert import ScriptedPickPlaceExpert
    from act_lab.domain import Observation

    settings = SimulationConfig.load(config)
    source_settings = tomllib.loads(config.read_text()).get("ros2", {})
    lookahead = source_settings.get("transit_lookahead_m", 0.01)
    source = ExpertTaskSource(require_release=True)

    def observe() -> Observation:
        state = snapshot_state(session.value)
        value = Observation(state.timestamp_ns, state, settings.cameras)
        source.update(session.value, value)
        return value

    observation = observe()
    initial_ns = observation.timestamp_ns
    started = time.monotonic()
    expert = ScriptedPickPlaceExpert(source, settings.expert)
    expert.reset(observation)
    ramp = GripperRamp(observation, source_settings.get("gripper_rate_s", 0.1))
    opening = OpeningPoseHold()
    approach_route = ApproachRoute(
        source_settings.get("approach_route_inward_x_m"),
        source_settings.get("approach_route_transition_z_m"),
    )
    recovery_deadline: float | None = None
    count = 0
    emitted = 0
    with session.expert_trace_path().open("w", buffering=1024 * 1024) as trace:
        for count in range(steps or settings.episode_steps):
            iteration_start = time.monotonic()
            session.rpc(dict(kind="snapshot"))
            snapshot_wall = time.monotonic()
            observation = observe()
            task = source.task_state()
            if task.terminal or expert.failure_reason:
                break
            expert_action = expert.poll(observation)
            action = expert_action
            action = approach_route.apply(expert.phase.value, action, observation)
            if expert.phase.value == "transit":
                action = loaded_transit_action(action, observation, lookahead)
            if expert.phase.value in {"descend", "lower", "open_dwell"}:
                action = descent_action(
                    action,
                    observation,
                    source_settings.get("descent_lookahead_m", 0.0012),
                )
            if settings.model_id == "ur5e_2f85_d405_v1":
                action = ramp.apply(action)
            if source_settings.get("hold_pose_while_opening", False):
                action = opening.apply(expert.phase.value, action, observation)
            command_wall = time.monotonic()
            value = session.command(
                action.target_pose,
                action.gripper_position,
                action.enabled,
                stamp=action.timestamp_ns,
            )
            report, episode, sequence = decode_report(value["report"])
            observation = observe()
            response_wall = time.monotonic()
            trace.write(
                json.dumps(
                    dict(
                        step=count,
                        phase=expert.phase.value,
                        expert_action=asdict(expert_action),
                        action=asdict(action),
                        report=asdict(report),
                        task=asdict(source.owner_task_state()),
                        released=source.released,
                        cube_gripper_contacts=session.value.get(
                            "cube_gripper_contacts"
                        ),
                        episode=episode,
                        sequence=sequence,
                        mode=value["mode"],
                        reason=value["reason"],
                        transport_rejection=value.get("transport_rejection"),
                        approach_route_stage=approach_route.stage,
                        producer_timing_ns=getattr(session, "trace", [{}])[-1].get(
                            "producer_timing_ns", {}
                        ),
                        gateway_timing_ns=getattr(session, "trace", [{}])[-1].get(
                            "gateway_timing_ns", {}
                        ),
                        iteration_timing_s=dict(
                            snapshot=snapshot_wall - iteration_start,
                            policy=command_wall - snapshot_wall,
                            command=response_wall - command_wall,
                        ),
                    ),
                    allow_nan=False,
                )
                + "\n"
            )
            emitted += 1
            recoverable = value["mode"] != "ENABLED" and (
                value["reason"]
                in {
                    "controller_wall_timeout",
                    "gateway_wall_timeout",
                    "clock_paused",
                    "stale_source",
                }
                or value.get("transport_rejection") == "delivery_timeout"
            )
            if recoverable:
                recovery_deadline = recovery_deadline or time.monotonic() + 15
                if time.monotonic() >= recovery_deadline:
                    raise RuntimeError("bounded expert readiness recovery timeout")
                session.rpc(dict(kind="prepare_controller"))
            else:
                recovery_deadline = None
                expert.record_command_report(report)
                if expert.failure_reason:
                    break
                if action.enabled and value["mode"] != "ENABLED":
                    raise RuntimeError(f"expert motion rejected: {value['reason']}")
            if stop_phase is not None and expert.phase.value == stop_phase:
                break
            remaining = iteration_start + 1 / source_hz - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
        task = source.task_state()
        return dict(
            success=task.success and source.released,
            released=source.released,
            terminal=task.terminal,
            reason=task.reason or expert.failure_reason or "step_limit",
            phase=expert.phase.value,
            commands=emitted,
            simulation_duration_s=(observation.timestamp_ns - initial_ns) / 1e9,
            wall_duration_s=time.monotonic() - started,
            source_gripper_rate_s=ramp.rate,
            source_hz=source_hz,
            transit_lookahead_m=lookahead,
            final_task=asdict(task),
        )


def _crisp(
    output: Path,
    config: Path,
    seed: int,
    steps: int,
    paced: bool = False,
    capture_profile: str = "full",
    full_task: bool = False,
    source_hz: float = 25.0,
) -> dict[str, Any]:
    from act_lab.adapters.ros2.simulation import snapshot_state
    from act_lab.adapters.ros2.simulation_smoke import MotionSession

    session = MotionSession(
        output / "motion", seed=seed, paced=paced, config_path=config
    )
    failed = False
    try:
        session.rpc(
            dict(
                kind="record_start",
                seed=seed,
                capture_profile=capture_profile,
                full_task=full_task,
            )
        )
        initial = snapshot_state(session.value)
        from dataclasses import replace

        target = replace(
            initial.end_effector_pose,
            position_xyz_m=(
                initial.end_effector_pose.position_xyz_m[0] - 0.01,
                *initial.end_effector_pose.position_xyz_m[1:],
            ),
        )
        task_result = None
        if full_task:
            if not paced:
                raise ValueError("full CRISP task requires paced acquisition")
            task_result = _expert_task(session, config, steps, source_hz)
        else:
            for _ in range(steps or 20):
                session.motion_command(target)
        session.rpc(
            dict(
                kind="record_stop",
                reason=(task_result["reason"] if task_result else "bounded_ros_demo"),
            )
        )
        return dict(
            episode_id=session.value["episode"],
            producer_pid=session.value["gateway_pid"]
            if "gateway_pid" in session.value
            else session.process.pid,
            controller="crisp",
            physics_pid=session.value["physics_pid"],
            scheduler="paced" if paced else "stepped",
            capture=session.value.get("capture"),
            task=task_result,
            controller_configuration=session.value["controller_configuration"],
        )
    except BaseException:
        failed = True
        raise
    finally:
        if failed:
            try:
                session.close()
            except (RuntimeError, OSError, EOFError):
                pass  # Preserve the original acquisition error after owner cleanup.
        else:
            session.close()


def run_recording(
    output: Path,
    *,
    config: Path = CONFIG,
    seed: int = 0,
    steps: int = 0,
    controller: str = "local",
    smoke: bool = False,
    paced: bool = False,
    capture_profile: str = "full",
    full_task: bool = False,
    source_hz: float = 25.0,
) -> dict[str, Any]:
    if not 0 < source_hz <= 50:
        raise ValueError("source cadence must be in (0, 50] Hz")
    if full_task and (controller != "crisp" or not paced):
        raise ValueError("full task requires --controller crisp --paced")
    require_recording()
    if output.exists():
        raise FileExistsError(str(output))
    if steps < 0 or controller not in {"local", "crisp"}:
        raise ValueError("invalid recording configuration")
    if paced and controller != "crisp":
        raise ValueError("paced acquisition requires --controller crisp")
    if capture_profile not in {"full", "reduced-effects"}:
        raise ValueError("unknown capture profile")
    if capture_profile != "full" and not paced:
        raise ValueError("capture profile selection requires paced acquisition")
    output.mkdir(parents=True)
    recorder = Recorder(output / "rosbag")
    try:
        acquisition = (
            _local(output, config, seed, steps)
            if controller == "local"
            else _crisp(
                output,
                config,
                seed,
                steps,
                paced,
                capture_profile,
                full_task,
                source_hz,
            )
        )
        storage = recorder.close()
    except BaseException as error:
        (output / "failure.json").write_text(
            json.dumps(
                dict(
                    status="failed",
                    reason=str(error),
                    seed=seed,
                    controller=controller,
                    full_task=full_task,
                    paced=paced,
                ),
                indent=2,
            )
            + "\n"
        )
        if recorder.process.is_alive():
            try:
                recorder.close(False)
            except (RuntimeError, OSError, EOFError):
                pass  # Preserve the acquisition error; partial evidence remains.
        raise
    imported = import_bag(output / "rosbag" / "bag", output / "imported")
    if storage["pid"] == acquisition["producer_pid"]:
        raise RuntimeError("DDS recorder and producer must be separate processes")
    cases: list[dict[str, Any]] = [
        dict(name="two_process_dds_and_rosbag2_mcap", passed=True),
        dict(name="canonical_import", passed=True, episodes=imported["cases"]),
    ]
    if controller == "local" or paced:
        from act_lab.adapters.mcap.reading import read_episode
        from act_lab.application.dataset import resample_episode, validate_episode

        episode = acquisition["episode_id"]
        local_directory = (
            output / "motion" / "acquisition" if paced else output / "local"
        )
        local = read_episode(local_directory / f"{episode}.mcap")
        ros = read_episode(output / "imported" / f"{episode}.mcap")
        if (
            local != ros
            or validate_episode(local) != validate_episode(ros)
            or resample_episode(local, 25) != resample_episode(ros, 25)
        ):
            raise RuntimeError("local/ROS recording equivalence failed")
        cases.append(
            dict(
                name="exact_local_ros_equivalence",
                passed=True,
                samples=len(local.samples),
                original_rgb=True,
            )
        )
    if smoke:
        second = output / "second-import"
        again = import_bag(output / "rosbag" / "bag", second)
        if imported != again:
            raise RuntimeError("import is not deterministic")
        cases.append(dict(name="deterministic_repeat_import", passed=True))
    if full_task:
        from act_lab.adapters.ros2.task_qualification import physics_summary

        physics = physics_summary(output / "motion" / "physics.json")
        cases.extend(
            [
                dict(
                    name="complete_released_task",
                    passed=acquisition["task"]["success"]
                    and acquisition["task"]["released"],
                ),
                dict(
                    name="complete_task_quality",
                    passed=all(case["quality"]["valid"] for case in imported["cases"]),
                ),
                dict(
                    name="measured_motion_limits",
                    passed=not physics["violations"]
                    and physics["zero_task_effort_in_hold"],
                    violations=physics["violations"],
                ),
            ]
        )
    result = dict(
        schema_version=1,
        status="passed" if all(case["passed"] for case in cases) else "failed",
        cases=cases,
        acquisition=acquisition,
        storage=storage,
        import_report=imported,
        provenance=dict(
            python=platform.python_version(),
            machine=platform.machine(),
            rmw=os.environ.get("RMW_IMPLEMENTATION"),
            ros_distro=os.environ.get("ROS_DISTRO"),
            simulation_config=config.name,
            builtin_transports=os.environ.get("FASTDDS_BUILTIN_TRANSPORTS", "DEFAULT"),
            publication_mode=os.environ.get("RMW_FASTRTPS_PUBLICATION_MODE"),
            real_time_claim=False,
            dependencies={
                name: version(name)
                for name in ("mcap", "protobuf", "mujoco", "lz4", "zstandard")
            },
            transport_profile_sha256=hashlib.sha256(
                Path(os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"]).read_bytes()
            ).hexdigest()
            if os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE")
            else None,
            controller_configuration=acquisition.get("controller_configuration"),
            runtime_packages_sha256=hashlib.sha256(
                Path("/opt/recording-packages.txt").read_bytes()
            ).hexdigest(),
        ),
    )
    (output / "report.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    return result
