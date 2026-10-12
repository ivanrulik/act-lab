"""Optional fixed-cohort and sustained CRISP acquisition evidence runner."""

from __future__ import annotations

import hashlib
import json
import math
import os
import platform
import resource
import statistics
import time
import tomllib
from collections import Counter
from dataclasses import asdict
from pathlib import Path
from typing import Any

from act_lab.application.evaluation import wilson_interval

TASK_CONFIG = Path("configs/sim/ur5e_2f85_d405_crisp_task.toml")


def distribution(values: list[int | float]) -> dict[str, float | int]:
    if not values:
        raise ValueError("timing evidence is empty")
    ordered = sorted(values)
    return dict(
        samples=len(values),
        median=float(statistics.median(ordered)),
        p95=float(ordered[math.ceil(len(ordered) * 0.95) - 1]),
        p99=float(ordered[math.ceil(len(ordered) * 0.99) - 1]),
        maximum=float(ordered[-1]),
    )


def physics_summary(path: Path) -> dict[str, Any]:
    rows = json.loads(path.read_text())
    if not rows:
        raise RuntimeError("missing physics evidence")
    maxima = {}
    bounds = {
        "cartesian_speed_m_s": 0.25,
        "cartesian_acceleration_m_s2": 1.0,
        "angular_speed_rad_s": 1.0,
        "angular_acceleration_rad_s2": 4.0,
        "maximum_joint_acceleration_rad_s2": 4.0,
    }
    for field in bounds:
        maxima[field] = max(abs(r.get(field, 0.0)) for r in rows)
    maxima["joint_speed_rad_s"] = max(
        abs(v) for r in rows for v in r["state"]["joint_velocities_rad_s"]
    )
    maxima["task_effort_nm"] = max(abs(v) for r in rows for v in r["task_effort_nm"])
    bounds.update(joint_speed_rad_s=1.0, task_effort_nm=5.0)
    violations = {k: v for k, v in maxima.items() if v > bounds[k] + 1e-6}
    return dict(
        ticks=len(rows),
        maxima=maxima,
        bounds=bounds,
        violations=violations,
        modes=dict(Counter(r["mode"] for r in rows)),
        reasons=dict(Counter(r["reason"] for r in rows)),
        paced_budget_overruns=sum(r.get("paced_budget_overrun", False) for r in rows),
        zero_task_effort_in_hold=all(
            not any(r["task_effort_nm"]) for r in rows if r["mode"] != "ENABLED"
        ),
        sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    )


def capture_summary(capture: dict[str, Any]) -> dict[str, Any]:
    return dict(
        samples=capture["samples"],
        renderer=capture["renderer"],
        capture_profile=capture["capture_profile"],
        peak_pending=capture["peak_pending"],
        receipt_to_ack_ns=distribution(capture["capture_ages_ns"]),
        stages_ns=capture["capture_stage_summary_ns"],
        reference_mirror=capture.get("reference_mirror", {}),
    )


def expert_summary(path: Path) -> dict[str, Any]:
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    if not rows:
        raise RuntimeError("missing expert task evidence")
    transitions = []
    previous = None
    for row in rows:
        if row["phase"] != previous:
            transitions.append(
                dict(phase=row["phase"], source_ns=row["action"]["timestamp_ns"])
            )
            previous = row["phase"]
    return dict(
        commands=len(rows),
        phase_transitions=transitions,
        phases=dict(Counter(row["phase"] for row in rows)),
        approach_route_stages=dict(
            Counter(row.get("approach_route_stage", "inactive") for row in rows)
        ),
        modes=dict(Counter(row["mode"] for row in rows)),
        reasons=dict(Counter(row["reason"] for row in rows)),
        report_outcomes=dict(Counter(row["report"]["outcome"] for row in rows)),
        maximum_cube_height_m=max(
            row["task"]["cube_pose"]["position_xyz_m"][2] for row in rows
        ),
        contact_observed=any(row["cube_gripper_contacts"] > 0 for row in rows),
        release_observed=any(row["released"] for row in rows),
        final_released=rows[-1]["released"],
        producer_timings_ns={
            stage: distribution([row["producer_timing_ns"][stage] for row in rows])
            for stage in rows[0].get("producer_timing_ns", {})
        },
        gateway_timings_ns={
            stage: distribution([row["gateway_timing_ns"][stage] for row in rows])
            for stage in rows[0].get("gateway_timing_ns", {})
        },
        sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    )


def local_attempt(config: Path, seed: int) -> dict[str, Any]:
    from act_lab.adapters.mujoco import MujocoCartesianDriver
    from act_lab.adapters.ros2.task import (
        ApproachRoute,
        ExpertTaskSource,
        GripperRamp,
        OpeningPoseHold,
        descent_action,
        loaded_transit_action,
    )
    from act_lab.application import SafeCartesianRobot, ScriptedPickPlaceExpert

    with MujocoCartesianDriver.from_config_file(config) as driver:
        robot = SafeCartesianRobot(driver, driver.limits)
        observation = robot.reset(seed)
        source = ExpertTaskSource(require_release=True)

        def update_source() -> None:
            contacts = sum(
                driver._is_pad_cube_contact(int(c.geom1), int(c.geom2))
                for c in driver._data.contact[: driver._data.ncon]
            )
            source.update_task(driver.environment.task_state(), observation, contacts)

        update_source()
        expert = ScriptedPickPlaceExpert(source, driver.simulation_config.expert)
        expert.reset(observation)
        source_settings = tomllib.loads(config.read_text()).get("ros2", {})
        ramp = GripperRamp(observation, source_settings.get("gripper_rate_s", 0.1))
        opening = OpeningPoseHold()
        approach_route = ApproachRoute(
            source_settings.get("approach_route_inward_x_m"),
            source_settings.get("approach_route_transition_z_m"),
        )
        phases: Counter[str] = Counter()
        outcomes: Counter[str] = Counter()
        for _ in range(driver.simulation_config.episode_steps):
            if source.task_state().terminal or expert.failure_reason:
                break
            action = expert.poll(observation)
            action = approach_route.apply(expert.phase.value, action, observation)
            if expert.phase.value == "transit":
                action = loaded_transit_action(
                    action,
                    observation,
                    source_settings.get("transit_lookahead_m", 0.01),
                )
            action = ramp.apply(action)
            if expert.phase.value in {"descend", "lower", "open_dwell"}:
                action = descent_action(
                    action,
                    observation,
                    source_settings.get("descent_lookahead_m", 0.0012),
                )
            if source_settings.get("hold_pose_while_opening", False):
                action = opening.apply(expert.phase.value, action, observation)
            robot.command(action)
            assert robot.last_report is not None
            expert.record_command_report(robot.last_report)
            outcomes[robot.last_report.outcome.value] += 1
            phases[expert.phase.value] += 1
            observation = robot.observe()
            update_source()
        task = source.task_state()
        return dict(
            seed=seed,
            success=task.success and source.released,
            released=source.released,
            reason=task.reason or expert.failure_reason or "step_limit",
            simulation_duration_s=observation.timestamp_ns / 1e9,
            phases=dict(phases),
            outcomes=dict(outcomes),
            source_gripper_rate_s=ramp.rate,
            task=asdict(task),
        )


def _equivalence(output: Path, episode: str) -> dict[str, Any]:
    from act_lab.adapters.mcap.reading import read_episode
    from act_lab.application.dataset import resample_episode, validate_episode

    local = read_episode(output / "motion" / "acquisition" / f"{episode}.mcap")
    imported = read_episode(output / "imported" / f"{episode}.mcap")
    if (
        local != imported
        or validate_episode(local) != validate_episode(imported)
        or resample_episode(local, 25) != resample_episode(imported, 25)
    ):
        raise RuntimeError("sustained canonical equivalence failed")
    stamps = [s.timestamp_ns for s in local.samples]
    return dict(
        episode_id=episode,
        samples=len(stamps),
        gaps_ns=distribution([b - a for a, b in zip(stamps, stamps[1:], strict=False)]),
        quality=asdict(validate_episode(imported)),
        exact_equivalence=True,
    )


def sustained_run(
    output: Path,
    config: Path,
    seeds: list[int],
    duration_s: float,
    source_hz: float,
    capture_profile: str,
) -> dict[str, Any]:
    from act_lab.adapters.ros2.recording import import_bag
    from act_lab.adapters.ros2.recording_demo import Recorder, _expert_task
    from act_lab.adapters.ros2.simulation_smoke import MotionSession

    output.mkdir()
    recorder = Recorder(output / "rosbag")
    session = None
    attempts: list[dict[str, Any]] = []
    active_wall = 0.0
    try:
        session = MotionSession(output / "motion", paced=True, config_path=config)
        while active_wall < duration_s:
            seed = seeds[len(attempts) % len(seeds)]
            session.rpc(
                dict(
                    kind="record_start",
                    seed=seed,
                    full_task=True,
                    capture_profile=capture_profile,
                )
            )
            task = _expert_task(session, config, 0, source_hz)
            episode = session.value["episode"]
            session.rpc(dict(kind="record_stop"))
            capture = capture_summary(session.value["capture"])
            trace = session.output / "expert.jsonl"
            trace.rename(session.output / f"expert-{episode}.jsonl")
            active_wall += task["wall_duration_s"]
            attempts.append(
                dict(
                    seed=seed,
                    episode_id=episode,
                    task=task,
                    capture=capture,
                    expert=expert_summary(session.output / f"expert-{episode}.jsonl"),
                )
            )
            (output / "progress.json").write_text(
                json.dumps(
                    dict(
                        active_wall_s=active_wall,
                        target_s=duration_s,
                        attempts=attempts,
                    ),
                    indent=2,
                )
                + "\n"
            )
            if not task["success"]:
                raise RuntimeError(f"sustained task failed: {task['reason']}")
        session.close()
        storage = recorder.close()
        imported = import_bag(output / "rosbag" / "bag", output / "imported")
        for attempt in attempts:
            attempt["data"] = _equivalence(output, attempt["episode_id"])
        physics = physics_summary(output / "motion" / "physics.json")
        return dict(
            attempts=attempts,
            active_wall_s=active_wall,
            storage=storage,
            import_report=imported,
            physics=physics,
            passed=(
                not physics["violations"]
                and physics["zero_task_effort_in_hold"]
                and all(
                    attempt["data"]["quality"]["valid"]
                    and attempt["data"]["quality"]["training_eligible"]
                    and attempt["data"]["exact_equivalence"]
                    for attempt in attempts
                )
            ),
        )
    finally:
        if session is not None and not getattr(session, "closed", False):
            try:
                session.close()
            except (RuntimeError, OSError, EOFError):
                pass
        if recorder.process.is_alive():
            try:
                recorder.close(False)
            except (RuntimeError, OSError, EOFError):
                pass  # The primary fault and partial evidence remain authoritative.


def run_qualification(
    output: Path,
    *,
    config: Path = TASK_CONFIG,
    seeds: list[int] | None = None,
    duration_s: float = 0.0,
    repeats: int = 3,
    source_hz: float = 25.0,
    capture_profile: str = "full",
) -> dict[str, Any]:
    from act_lab.adapters.mujoco.config import SimulationConfig
    from act_lab.adapters.ros2.recording import require_recording
    from act_lab.adapters.ros2.recording_demo import run_recording

    require_recording()
    seeds = list(range(20)) if seeds is None else seeds
    if (
        not seeds
        or len(set(seeds)) != len(seeds)
        or any(type(seed) is not int or seed < 0 for seed in seeds)
        or not math.isfinite(duration_s)
        or duration_s < 0
        or repeats < 1
        or not math.isfinite(source_hz)
        or not 0 < source_hz <= 50
    ):
        raise ValueError("invalid qualification configuration")
    if output.exists():
        raise FileExistsError(str(output))
    settings = SimulationConfig.load(config)
    output.mkdir(parents=True)
    source_paths = sorted(Path("src/act_lab").rglob("*.py")) + [
        config,
        Path("configs/ros2/fastdds-recording.xml"),
        Path("configs/ros2/crisp-task-qualification.yaml"),
    ]
    source_files = {
        str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in source_paths
    }
    result: dict[str, Any] = dict(
        schema_version=1,
        status="running",
        cases=[],
        seeds=seeds,
        resolved_configuration=asdict(settings),
        source_configuration=tomllib.loads(config.read_text()).get("ros2", {}),
        source_hz=source_hz,
        capture_profile=capture_profile,
        local=[],
        crisp=[],
        sustained=[],
        provenance=dict(
            python=platform.python_version(),
            machine=platform.machine(),
            ros=os.environ.get("ROS_DISTRO"),
            rmw=os.environ.get("RMW_IMPLEMENTATION"),
            config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),
            source_files=source_files,
            source_sha256=hashlib.sha256(
                json.dumps(source_files, sort_keys=True).encode()
            ).hexdigest(),
            hardware_qualified=False,
        ),
    )
    started = time.monotonic()

    def save() -> None:
        result["wall_duration_s"] = time.monotonic() - started
        result["runner_max_rss_kib"] = resource.getrusage(
            resource.RUSAGE_SELF
        ).ru_maxrss
        temporary = output / "report.json.partial"
        temporary.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        temporary.replace(output / "report.json")

    save()
    try:
        for seed in seeds:
            result["local"].append(local_attempt(config, seed))
            save()
        for seed in seeds:
            attempt_output = output / f"seed-{seed:05d}"
            try:
                attempt = run_recording(
                    attempt_output,
                    config=config,
                    seed=seed,
                    controller="crisp",
                    paced=True,
                    full_task=True,
                    source_hz=source_hz,
                    capture_profile=capture_profile,
                )
                physics = physics_summary(attempt_output / "motion" / "physics.json")
                result["crisp"].append(
                    dict(
                        seed=seed,
                        task=attempt["acquisition"]["task"],
                        physics=physics,
                        expert=expert_summary(
                            attempt_output / "motion" / "expert.jsonl"
                        ),
                        capture=capture_summary(attempt["acquisition"]["capture"]),
                        episode_id=attempt["acquisition"]["episode_id"],
                        data=_equivalence(
                            attempt_output, attempt["acquisition"]["episode_id"]
                        ),
                    )
                )
            except (RuntimeError, OSError, ValueError, EOFError) as error:
                result["crisp"].append(dict(seed=seed, error=str(error), success=False))
            save()
        successes = sum(
            row.get("task", {}).get("success", False) for row in result["crisp"]
        )
        result["successes"] = successes
        result["success_rate_ci_95"] = wilson_interval(successes, len(seeds))
        cohort_passed = (
            successes / len(seeds) >= 0.9
            and all(
                "error" not in row
                and not row["physics"]["violations"]
                and row["physics"]["zero_task_effort_in_hold"]
                and row["data"]["quality"]["valid"]
                for row in result["crisp"]
            )
            and all(row["success"] for row in result["local"])
        )
        result["cases"].append(dict(name="fixed_cohort", passed=cohort_passed))
        if duration_s and cohort_passed:
            for repeat in range(repeats):
                sustained = sustained_run(
                    output / f"sustained-{repeat}",
                    config,
                    seeds,
                    duration_s,
                    source_hz,
                    capture_profile,
                )
                result["sustained"].append(sustained)
                result["cases"].append(
                    dict(name=f"sustained-{repeat}", passed=sustained["passed"])
                )
                save()
        result["full_gpu_sustained_gate"] = (
            capture_profile == "full"
            and os.environ.get("ACT_LAB_RENDERER") == "nvidia"
            and duration_s >= 600
            and repeats >= 3
            and len(result["sustained"]) >= 3
            and all(row["passed"] for row in result["sustained"])
        )
        result["scope"] = "cohort_and_capture"
        result["full_qualification"] = False
        result["remaining_gates"] = [
            "loaded_fault_matrix",
            "derived_dataset",
            "manual_replay",
        ]
        if seeds != list(range(20)):
            result["remaining_gates"].append("frozen_twenty_seed_cohort")
        if not result["full_gpu_sustained_gate"]:
            result["remaining_gates"].append("three_600_second_full_gpu_runs")
        result["status"] = (
            "passed" if all(case["passed"] for case in result["cases"]) else "failed"
        )
    except BaseException as error:
        result.update(status="failed", error=str(error))
        save()
        raise
    save()
    return result
