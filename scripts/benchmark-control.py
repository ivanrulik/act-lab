"""Reproducible timing client of the production MuJoCo safe-control APIs."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import subprocess
import time
from collections import Counter
from dataclasses import asdict, replace
from datetime import UTC, datetime
from pathlib import Path

import mujoco
import numpy as np

from act_lab.adapters.mujoco import MujocoCartesianDriver, MujocoUR5eEnvironment
from act_lab.adapters.mujoco.config import SimulationConfig
from act_lab.application import SafeCartesianRobot, ScriptedPickPlaceExpert
from act_lab.domain import Action, Pose


def distribution(values: list[float], budget_ms: float) -> dict[str, object]:
    return {
        "samples": len(values),
        "median_ms": float(np.median(values)),
        "p95_ms": float(np.percentile(values, 95)),
        "p99_ms": float(np.percentile(values, 99)),
        "max_ms": max(values),
        "over_budget": sum(value > budget_ms for value in values),
    }


def benchmark(
    config: SimulationConfig,
    fixture: dict,
    samples: int,
    warmup: int,
    repeats: int,
    paced_cycles: int,
) -> dict[str, object]:
    near = Pose("world", tuple(fixture["position"]), tuple(fixture["quaternion"]))
    far = Pose("world", (0.8, 0.6, 1.0), (1.0, 0.0, 0.0, 0.0))
    budget_ms = 1000 * (1 / config.environment_hz)
    cases = {}
    with MujocoCartesianDriver(
        MujocoUR5eEnvironment(config), config, profile=True
    ) as driver:

        def reset(boundary: bool) -> SafeCartesianRobot:
            driver.reset(fixture["seed"])
            if boundary:
                for address, value in zip(
                    driver._qpos_addresses, fixture["joints"], strict=True
                ):  # noqa: SLF001
                    driver._data.qpos[address] = value  # noqa: SLF001
                mujoco.mj_forward(driver._model, driver._data)  # noqa: SLF001
            return SafeCartesianRobot(driver, config.control)

        for name in (
            "ik_home",
            "ik_near_boundary",
            "ik_unreachable",
            "command_motion",
            "command_boundary",
            "command_unreachable",
            "command_disabled",
            "command_stale",
            "command_contact_approach",
        ):
            rows = []
            batches = []
            for repeat in range(repeats):
                robot = reset(
                    name
                    in {"ik_near_boundary", "command_boundary", "command_unreachable"}
                )
                expert = ScriptedPickPlaceExpert(driver.environment, config.expert)
                expert.reset(robot.observe())
                batch = []
                for index in range(warmup + samples):
                    if name not in {
                        "command_motion",
                        "command_contact_approach",
                        "command_unreachable",
                    }:
                        robot = reset(
                            name
                            in {
                                "ik_near_boundary",
                                "command_boundary",
                                "command_unreachable",
                            }
                        )
                    elif driver.environment.task_state().terminal:
                        robot = reset(False)
                        expert.reset(robot.observe())
                    observation = robot.observe()
                    driver.clear_profile()
                    driver.last_ik_diagnostics = None
                    started = time.perf_counter_ns()
                    if name.startswith("ik_"):
                        target = {
                            "ik_home": observation.robot.end_effector_pose,
                            "ik_near_boundary": near,
                            "ik_unreachable": far,
                        }[name]
                        outcome = (
                            "converged" if driver.solve_ik(target) else "ik_failure"
                        )
                    else:
                        if name == "command_motion":
                            action = expert.poll(observation)
                        else:
                            target = near if name == "command_boundary" else far
                            if name == "command_contact_approach":
                                target = Pose(
                                    "world",
                                    (0.58, 0.23, 0.44),
                                    observation.robot.end_effector_pose.quaternion_wxyz,
                                )
                            action = Action(
                                observation.timestamp_ns
                                - (
                                    config.control.watchdog_timeout_ns
                                    if name == "command_stale"
                                    else 0
                                ),
                                target,
                                1.0,
                                enabled=name != "command_disabled",
                            )
                        robot.command(action)
                        robot.observe()
                        report = robot.last_report
                        assert report is not None
                        outcome = report.outcome.value
                        if name == "command_motion":
                            expert.record_command_report(report)
                    elapsed_ms = (time.perf_counter_ns() - started) / 1e6
                    if index >= warmup:
                        row = {
                            "repeat": repeat,
                            "duration_ms": elapsed_ms,
                            "outcome": outcome,
                            "phase_ms": {
                                k: v / 1e6 for k, v in driver.profile_totals_ns.items()
                            },
                            "ik": asdict(driver.last_ik_diagnostics)
                            if driver.last_ik_diagnostics
                            else None,
                        }
                        rows.append(row)
                        batch.append(elapsed_ms)
                batches.append(distribution(batch, budget_ms))
            cases[name] = {
                "summary": distribution([r["duration_ms"] for r in rows], budget_ms),
                "outcomes": dict(Counter(r["outcome"] for r in rows)),
                "batches": batches,
                "rows": rows,
            }

        # Production-rate synthetic source: infeasible requests alternate with
        # disabled input. This measures scheduling and next-cycle hold behavior
        # without claiming display or physical-camera validation.
        robot = reset(True)
        cycles = []
        paced_rows = []
        holds = []
        intervals = []
        outcomes = Counter()
        previous_cycle = None
        overrun_streak = maximum_overrun_streak = 0
        for index in range(paced_cycles):
            cycle = time.perf_counter_ns()
            if previous_cycle is not None:
                intervals.append((cycle - previous_cycle) / 1e6)
            previous_cycle = cycle
            observation = robot.observe()
            enabled = index % 11 != 10
            action = Action(observation.timestamp_ns, far, 1.0, enabled)
            driver.clear_profile()
            driver.last_ik_diagnostics = None
            command_started = time.perf_counter_ns()
            robot.command(action)
            observation = robot.observe()
            elapsed_ms = (time.perf_counter_ns() - cycle) / 1e6
            cycles.append(elapsed_ms)
            paced_rows.append(
                {
                    "index": index,
                    "enabled": enabled,
                    "duration_ms": elapsed_ms,
                    "outcome": robot.last_report.outcome.value,
                    "phase_ms": {
                        k: v / 1e6 for k, v in driver.profile_totals_ns.items()
                    },
                    "ik": asdict(driver.last_ik_diagnostics)
                    if driver.last_ik_diagnostics
                    else None,
                }
            )
            assert robot.last_report is not None
            outcomes[robot.last_report.outcome.value] += 1
            overrun_streak = overrun_streak + 1 if elapsed_ms > budget_ms else 0
            maximum_overrun_streak = max(maximum_overrun_streak, overrun_streak)
            if not enabled:
                assert robot.last_report is not None
                assert robot.last_report.outcome.value == "disabled"
                holds.append((time.perf_counter_ns() - command_started) / 1e6)
            remaining = driver.control_period_s - (time.perf_counter_ns() - cycle) / 1e9
            if remaining > 0:
                time.sleep(remaining)
        paced = {
            "cycle_work": distribution(cycles, budget_ms) if cycles else None,
            "cycle_intervals": distribution(intervals, budget_ms)
            if intervals
            else None,
            "maximum_consecutive_work_overruns": maximum_overrun_streak,
            "outcomes": dict(outcomes),
            "rows": paced_rows,
            "intervals_ms": intervals,
            "disabled_to_hold": distribution(holds, 100) if holds else None,
            "includes": "synthetic input, safe command, observation",
            "excludes": "display/camera",
        }
    return {"config": asdict(config), "cases": cases, "paced": paced}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=Path("configs/sim/ur5e_pick_place.toml")
    )
    parser.add_argument(
        "--fixture", type=Path, default=Path("tests/fixtures/control/ik_boundary.json")
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--image-id", required=True)
    parser.add_argument("--samples", type=int, default=300)
    parser.add_argument("--warmup", type=int, default=20)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--paced-cycles", type=int, default=500)
    parser.add_argument(
        "--variants",
        nargs="+",
        choices=("original", "current", "cap100"),
        default=["original", "current", "cap100"],
    )
    args = parser.parse_args()
    if (
        args.samples <= 0
        or args.repeats <= 0
        or args.warmup < 0
        or args.paced_cycles < 0
    ):
        parser.error(
            "samples/repeats must be positive; warmup/paced-cycles nonnegative"
        )
    if args.output.exists():
        parser.error("output exists; keep benchmark reports immutable")
    base = SimulationConfig.load(args.config)
    variants = {
        "original": replace(
            base,
            control=replace(
                base.control, ik_max_iterations=50, ik_position_tolerance_m=0.0001
            ),
        ),
        "current": base,
        "cap100": replace(base, control=replace(base.control, ik_max_iterations=100)),
    }
    report = {
        "schema_version": 1,
        "created_at_utc": datetime.now(UTC).isoformat(),
        "git_revision": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True
        ).strip(),
        "git_dirty": bool(
            subprocess.check_output(["git", "status", "--porcelain"], text=True).strip()
        ),
        "image_id": args.image_id,
        "source_sha256": {
            str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (
                Path(__file__),
                args.config,
                args.fixture,
                Path("src/act_lab/adapters/mujoco/cartesian_driver.py"),
                Path("src/act_lab/application/cartesian_control.py"),
            )
        },
        "python": platform.python_version(),
        "mujoco": mujoco.__version__,
        "numpy": np.__version__,
        "cpu": platform.machine(),
        "cpu_affinity_count": len(os.sched_getaffinity(0)),
        "thread_environment": {
            k: os.environ.get(k)
            for k in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS")
        },
        "blas": np.show_config(mode="dicts")["Build Dependencies"]["blas"],
        "fixture": json.loads(args.fixture.read_text()),
        "sample_settings": {
            k: getattr(args, k)
            for k in ("samples", "warmup", "repeats", "paced_cycles")
        },
        "timing_note": "observational wall time; step includes collision checks",
        "variants": {},
    }
    for name in args.variants:
        result = benchmark(
            variants[name],
            report["fixture"],
            args.samples,
            args.warmup,
            args.repeats,
            args.paced_cycles,
        )
        report["variants"][name] = result
        print(
            name,
            {k: round(v["summary"]["p99_ms"], 2) for k, v in result["cases"].items()},
            flush=True,
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(f"benchmark report: {args.output}")


if __name__ == "__main__":
    main()
