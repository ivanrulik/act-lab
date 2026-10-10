"""Supervised, paced read-only observability demo using existing motion authority."""

from __future__ import annotations

import hashlib
import json
import math
import multiprocessing as mp
import os
import platform
import subprocess
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

from act_lab.adapters.ros2.observability import ObservationChannel, read_report_snapshot
from act_lab.adapters.ros2.observation_runtime import ViewerProcesses, observer_process
from act_lab.adapters.ros2.simulation import CONFIG, snapshot_state
from act_lab.adapters.ros2.simulation_smoke import MotionSession


def run_observability(
    output: Path,
    duration: float = 30.0,
    seed: int = 0,
    paced: bool = True,
    smoke: bool = False,
    config_path: Path = CONFIG,
) -> dict[str, Any]:
    if not 2.0 <= duration <= 3600.0 or not math.isfinite(duration):
        raise ValueError(
            "observability duration must be finite, between 2 and 3600 seconds"
        )
    if not Path("/opt/observability-packages.txt").exists():
        raise RuntimeError(
            "Use docker compose --profile ros2-observability "
            "run --build --rm ros2-observability"
        )
    output.mkdir(parents=True, exist_ok=True)
    from act_lab.adapters.mujoco.config import SimulationConfig

    config = SimulationConfig.load(config_path)
    channel = ObservationChannel(camera=config.model_id != "ur5e_educational_v1")
    context = mp.get_context("spawn")
    stop = context.Event()
    observer = context.Process(
        target=observer_process, args=(channel, stop, str(output))
    )
    views = ViewerProcesses(output, config_path=config_path)
    session = None
    events: list[dict[str, Any]] = []
    observer.start()
    camera = None
    if channel.camera is not None:
        from act_lab.adapters.ros2.camera_runtime import camera_process

        camera = context.Process(
            target=camera_process, args=(channel.camera, stop, str(output), config_path)
        )
        camera.start()
    try:
        session = MotionSession(output, seed, paced, channel, config_path)
        if channel.camera is not None and not smoke:
            from act_lab.adapters.ros2.simulation_smoke import grasp_lift_hold

            events.append(grasp_lift_hold(session))
        initial = snapshot_state(session.value).end_effector_pose
        start = time.monotonic()
        for phase in range(4):
            events.append(dict(phase=phase, wall_elapsed_s=time.monotonic() - start))
            if phase == 3:
                session.rpc(dict(kind="reset", seed=seed))
                session.sequence = 0
                initial = snapshot_state(session.value).end_effector_pose
                events[-1]["reset"] = True
            phase_start = time.monotonic()
            while time.monotonic() - phase_start < duration / 4:
                elapsed = time.monotonic() - phase_start
                if phase == 1:
                    # The producer really stops: the independent owner installs hold.
                    time.sleep(0.02)
                    session.rpc(dict(kind="snapshot"))
                else:
                    offset = -0.01 * min(1.0, elapsed / 2.0) if phase == 0 else 0.0
                    target = replace(
                        initial,
                        position_xyz_m=(
                            initial.position_xyz_m[0] + offset,
                            *initial.position_xyz_m[1:],
                        ),
                    )
                    session.motion_command(
                        target, gripper=(0.0 if channel.camera is not None else 0.2)
                    )
                if not observer.is_alive():
                    raise RuntimeError("observer process exited")
                if camera is not None and not camera.is_alive():
                    raise RuntimeError(
                        "camera renderer exited; inspect camera evidence"
                    )
                if any(process.poll() is not None for process in views.processes):
                    raise RuntimeError(
                        "viewer infrastructure exited; inspect viewer logs"
                    )
        final = read_report_snapshot(channel)
        if channel.camera is not None and not channel.camera.last_delivery_ns.value:
            raise RuntimeError("wrist camera produced no current frames")
        if smoke:
            from act_lab.adapters.ros2.observation_protocol import bridge_probe

            probe = bridge_probe(tool_assets=channel.camera is not None)
        else:
            probe = None
        result = dict(
            provenance=dict(
                repository_revision=subprocess.check_output(
                    ["git", "rev-parse", "HEAD"], text=True
                ).strip(),
                source_sha256={
                    str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in Path("src/act_lab/adapters/ros2").glob("*.py")
                },
                ros_distro=os.environ.get("ROS_DISTRO"),
                rmw=os.environ.get("RMW_IMPLEMENTATION"),
                machine=platform.uname()._asdict(),
                packages=Path("/opt/observability-packages.txt").read_text(),
                model_sha256=hashlib.sha256(
                    Path("/opt/crisp/ur5e.urdf").read_bytes()
                ).hexdigest(),
                description_commit="6b639c2efd9f4f12da858a72cf1a9e40da365ed8",
                crisp_commit="0279dc8f1196dab6a2b2cd5af853dd3d4dc3a969",
                config_sha256={
                    str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in [
                        Path("configs/ros2/foxglove/bridge.yaml"),
                        Path("configs/ros2/foxglove/act-lab.json"),
                    ]
                },
            ),
            schema_version=1,
            status="completed",
            cases=events,
            final_telemetry=final["telemetry"],
            bridge=probe,
            observer_pid=observer.pid,
            renderer_pid=camera.pid if camera else None,
            model_id=config.model_id,
            physics_pid=session.value["physics_pid"],
            paced=paced,
            seed=seed,
            duration_s=duration,
            actual_duration_s=time.monotonic() - start,
        )
        (output / "report.json").write_text(json.dumps(result, indent=2) + "\n")
        return result
    finally:
        try:
            if session:
                session.close()
        finally:
            stop.set()
            for child in (observer, camera):
                if child is None:
                    continue
                child.join(timeout=5)
                if child.is_alive():
                    child.terminate()
                    child.join(timeout=5)
            views.close()
