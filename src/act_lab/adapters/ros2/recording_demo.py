"""Live stepped capture and transport equivalence against the canonical writer."""

from __future__ import annotations

import argparse
import hashlib
import json
import multiprocessing as mp
import os
import platform
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
    def __init__(self, output: Path) -> None:
        context = mp.get_context("spawn")
        self.connection, child = context.Pipe()
        self.process = context.Process(target=record_process, args=(child, str(output)))
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


def _crisp(output: Path, config: Path, seed: int, steps: int) -> dict[str, Any]:
    from act_lab.adapters.ros2.simulation import snapshot_state
    from act_lab.adapters.ros2.simulation_smoke import MotionSession

    session = MotionSession(
        output / "motion", seed=seed, paced=False, config_path=config
    )
    try:
        session.rpc(dict(kind="record_start", seed=seed))
        initial = snapshot_state(session.value)
        from dataclasses import replace

        target = replace(
            initial.end_effector_pose,
            position_xyz_m=(
                initial.end_effector_pose.position_xyz_m[0] - 0.01,
                *initial.end_effector_pose.position_xyz_m[1:],
            ),
        )
        for _ in range(steps or 20):
            session.motion_command(target)
        session.rpc(dict(kind="record_stop"))
        return dict(
            episode_id=session.value["episode"],
            producer_pid=session.value["gateway_pid"]
            if "gateway_pid" in session.value
            else session.process.pid,
            controller="crisp",
            physics_pid=session.value["physics_pid"],
        )
    finally:
        session.close()


def run_recording(
    output: Path,
    *,
    config: Path = CONFIG,
    seed: int = 0,
    steps: int = 0,
    controller: str = "local",
    smoke: bool = False,
) -> dict[str, Any]:
    require_recording()
    if output.exists():
        raise FileExistsError(str(output))
    if steps < 0 or controller not in {"local", "crisp"}:
        raise ValueError("invalid recording configuration")
    output.mkdir(parents=True)
    recorder = Recorder(output / "rosbag")
    try:
        acquisition = (_local if controller == "local" else _crisp)(
            output, config, seed, steps
        )
        storage = recorder.close()
    except BaseException:
        if recorder.process.is_alive():
            recorder.close(False)
        raise
    imported = import_bag(output / "rosbag" / "bag", output / "imported")
    if storage["pid"] == acquisition["producer_pid"]:
        raise RuntimeError("DDS recorder and producer must be separate processes")
    cases: list[dict[str, Any]] = [
        dict(name="two_process_dds_and_rosbag2_mcap", passed=True),
        dict(name="canonical_import", passed=True, episodes=imported["cases"]),
    ]
    if controller == "local":
        from act_lab.adapters.mcap.reading import read_episode
        from act_lab.application.dataset import resample_episode, validate_episode

        episode = acquisition["episode_id"]
        local = read_episode(output / "local" / f"{episode}.mcap")
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
    result = dict(
        schema_version=1,
        status="passed",
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
            real_time_claim=False,
            dependencies={
                name: version(name)
                for name in ("mcap", "protobuf", "mujoco", "lz4", "zstandard")
            },
            runtime_packages_sha256=hashlib.sha256(
                Path("/opt/recording-packages.txt").read_bytes()
            ).hexdigest(),
        ),
    )
    (output / "report.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    return result
