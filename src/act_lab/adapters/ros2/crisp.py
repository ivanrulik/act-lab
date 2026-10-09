"""Optional pinned-controller assessment orchestration (no ROS at import time)."""

from __future__ import annotations

import hashlib
import json
import math
import os
import platform
import shutil
import signal
import subprocess
import time
import tomllib
from dataclasses import asdict
from pathlib import Path
from typing import Any

from act_lab.adapters.ros2.crisp_trace import HOME_JOINTS, generate_trace, write_trace
from act_lab.adapters.ros2.demonstrator import DemonstratorLimits
from act_lab.domain import Pose

CRISP_REVISION = "0279dc8f1196dab6a2b2cd5af853dd3d4dc3a969"
DESCRIPTION_REVISION = "6b639c2efd9f4f12da858a72cf1a9e40da365ed8"
DRIVER_REVISION = "f6997bd4a0eee8d58198b696caf034009ab4de1e"
MODES = ("impedance", "operational_space")
BASE_IMAGE = (
    "ros:jazzy-ros-base-noble@sha256:"
    "066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca"
)


def decision(modes: dict[str, Any], *, enforce_timing: bool = True) -> dict[str, Any]:
    qualified = []
    for mode in MODES:
        evidence = modes[mode]
        runs = [r for r in evidence["timing"] if r["rate_hz"] == 500]
        if len(runs) != 3 or any(
            r["samples"] != 10_000 or r["warmup"] != 1_000 for r in runs
        ):
            raise RuntimeError("incomplete 500 Hz benchmark evidence")
        timing_pass = all(r["p99_ns"] < 2_000_000 for r in runs)
        if evidence["functional_pass"] and (timing_pass or not enforce_timing):
            qualified.append(mode)
    return dict(
        outcome=("conditional_go" if qualified else "no_go")
        if enforce_timing
        else "ci_evidence_only",
        selected_mode=qualified[0] if qualified and enforce_timing else None,
        functional_candidates=qualified,
        timing_policy="local_p99_below_2ms" if enforce_timing else "ci_record_only",
        hardware_approved=False,
    )


def _write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def _process(
    executable: str, args: list[str], output: Path, name: str, *, unsafe: bool = False
) -> Any:
    start = time.monotonic()
    stdout_path = output / f"{name}.stdout.log"
    with (
        stdout_path.open("w") as stdout_file,
        (output / f"{name}.log").open("w") as stderr_file,
    ):
        process = subprocess.Popen(
            [executable, *args],
            stdout=stdout_file,
            stderr=stderr_file,
            text=True,
            start_new_session=True,
            env={**os.environ, "RCUTILS_LOGGING_USE_STDOUT": "0"},
        )
        try:
            process.wait(timeout=90)
        except subprocess.TimeoutExpired as error:
            raise RuntimeError(f"bench timeout: {name}") from error
        finally:
            # A native crash must not leave its DDS publisher alive or keep
            # inherited evidence pipes open. Never kill unrelated processes.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()
    stdout = stdout_path.read_text()
    if process.returncode:
        if unsafe and process.returncode in (
            -signal.SIGABRT,
            -signal.SIGSEGV,
            -signal.SIGFPE,
        ):
            result = dict(
                isolated_exit_code=process.returncode,
                elapsed_s=time.monotonic() - start,
            )
            _write(output / f"{name}.json", result)
            return result
        raise RuntimeError(f"bench failed: {name}; see {output / (name + '.log')}")
    try:
        result = json.loads(stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError(f"invalid bench evidence: {name}") from error
    _write(output / f"{name}.json", result)
    return result


def _git(path: str) -> str:
    return subprocess.check_output(
        ["git", "-c", f"safe.directory={path}", "-C", path, "rev-parse", "HEAD"],
        text=True,
    ).strip()


def provenance() -> dict[str, Any]:
    roots = dict(
        crisp="/opt/crisp/src/crisp_controllers",
        description="/opt/crisp/src/ur_description",
        driver="/opt/ur-driver-audit",
    )
    commits = {key: _git(path) for key, path in roots.items()}
    if commits != dict(
        crisp=CRISP_REVISION, description=DESCRIPTION_REVISION, driver=DRIVER_REVISION
    ):
        raise RuntimeError("upstream revision mismatch")
    for name, path in roots.items():
        if subprocess.check_output(
            [
                "git",
                "-c",
                f"safe.directory={path}",
                "-C",
                path,
                "status",
                "--porcelain",
            ],
            text=True,
        ):
            raise RuntimeError(f"modified pinned source: {name}")
    package = Path(roots["crisp"]) / "package.xml"
    license_text = (Path(roots["crisp"]) / "LICENSE.md").read_text()
    source_notices = {}
    for notice_path in Path(roots["crisp"]).rglob("*"):
        if notice_path.suffix in (".hpp", ".cpp", ".h"):
            content = notice_path.read_text()
            if "Licensed under" in content or "Copyright" in content:
                source_notices[str(notice_path.relative_to(roots["crisp"]))] = (
                    content.splitlines()[:35]
                )
    if "MIT" not in license_text or "Apache" not in package.read_text():
        raise RuntimeError("license metadata changed")
    simulation_config = tomllib.loads(
        Path("configs/sim/ur5e_pick_place.toml").read_text()
    )
    if tuple(simulation_config["robot"]["home_joint_positions_rad"]) != HOME_JOINTS:
        raise RuntimeError("bench home differs from repository configuration")
    audit_path = Path(roots["driver"]) / "ur_robot_driver/src/hardware_interface.cpp"
    audit_lines = audit_path.read_text().splitlines()
    audit = {
        f"hardware_interface.cpp:{start}-{end}": audit_lines[start - 1 : end]
        for start, end in (
            (170, 213),
            (370, 382),
            (578, 585),
            (798, 808),
            (909, 917),
            (1020, 1026),
            (1270, 1345),
            (1390, 1410),
            (1465, 1470),
            (1520, 1528),
        )
    }
    return dict(
        driver_audit=audit,
        application_limits=asdict(DemonstratorLimits()),
        repository_simulation_config=simulation_config,
        implementation_hashes={
            "runner_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "trace_sha256": hashlib.sha256(
                Path(__file__).with_name("crisp_trace.py").read_bytes()
            ).hexdigest(),
            "bench_sha256": hashlib.sha256(
                Path(str(shutil.which("crisp_bench"))).read_bytes()
            ).hexdigest(),
        },
        commits=commits,
        base_image=BASE_IMAGE,
        model_sha256=hashlib.sha256(
            Path("/opt/crisp/ur5e.urdf").read_bytes()
        ).hexdigest(),
        model=dict(
            root="world", end_effector="tool0", prefix="", integrated_motion=False
        ),
        compiler=Path("/opt/crisp/compiler.txt").read_text(),
        dependencies=Path("/opt/crisp/packages.txt").read_text().splitlines(),
        cmake_cache=Path("/opt/crisp/build/crisp_controllers/CMakeCache.txt")
        .read_text()
        .splitlines(),
        build_flags=dict(
            build_type="Release",
            native_optimization=False,
            precompiled_headers=True,
            parallel_level=1,
        ),
        ros_distro=os.environ.get("ROS_DISTRO"),
        rmw=os.environ.get("RMW_IMPLEMENTATION"),
        machine=dict(
            platform=platform.platform(),
            python=platform.python_version(),
            cpu_count=os.cpu_count(),
            cpuinfo=Path("/proc/cpuinfo").read_text().splitlines()[:30],
        ),
        licensing=dict(
            license_file=license_text,
            package_declaration=package.read_text(),
            source_notices=source_notices,
            consistent_metadata=False,
        ),
    )


def qualification_inputs() -> tuple[dict[str, Any], Path]:
    """Reject changed qualification assumptions or stale FK evidence before ROS."""
    profile = json.loads(Path("configs/ros2/crisp-feasibility.json").read_text())
    expected_filters = dict(
        target_pose=0.1, q=0.0, dq=0.0, q_ref=0.0, output_torque=1.0
    )
    if profile.get("schema_version") != 2 or profile.get("filters") != expected_filters:
        raise RuntimeError("unsupported CRISP qualification profile")
    binding = profile["frame_binding"]
    if (
        binding["scene_from_ur_world_quaternion_wxyz"] != [0.0, 0.0, 0.0, 1.0]
        or binding["tool0_to_gripper_tip_translation_m"] != [0.0, 0.0, 0.11]
        or binding["translation_tolerance_m"] != 0.002
        or binding["rotation_tolerance_rad"] != 0.02
    ):
        raise RuntimeError("unreviewed CRISP frame binding")
    reference_path = Path("tests/fixtures/crisp/kinematics.json")
    reference = json.loads(reference_path.read_text())
    from act_lab.adapters.ros2.contracts import JOINT_NAMES

    if (
        reference["schema_version"] != 1
        or reference["integrated_motion"] is not False
        or reference["joint_names"] != list(JOINT_NAMES)
        or len(reference["fixtures"]) != 9
        or set(reference["model_hashes"]) != {"ur5e.xml", "act_lab_scene.xml"}
    ):
        raise RuntimeError("incomplete MJCF FK reference")
    for row in reference["fixtures"]:
        if len(row["joints"]) != 6 or not all(math.isfinite(q) for q in row["joints"]):
            raise RuntimeError("invalid MJCF joint fixture")
    model_root = Path("src/act_lab/adapters/mujoco/assets/ur5e")
    for name, digest in reference["model_hashes"].items():
        if hashlib.sha256((model_root / name).read_bytes()).hexdigest() != digest:
            raise RuntimeError(
                "stale MJCF FK reference; regenerate with "
                "scripts/crisp-frame-reference.py"
            )
    return profile, reference_path


def run_feasibility(output: Path, *, enforce_timing: bool = True) -> dict[str, Any]:
    executable = shutil.which("crisp_bench")
    if executable is None or os.environ.get("ROS_DISTRO") != "jazzy":
        raise RuntimeError(
            "CRISP feasibility requires the isolated Jazzy bench. Run: "
            "docker compose --profile crisp-feasibility run --build --rm "
            "crisp-feasibility act-lab ros2 crisp-feasibility "
            "--output runs/crisp-feasibility --json"
        )
    output.mkdir(parents=True, exist_ok=True)
    # A failed rerun must not leave a previous successful report in this directory.
    (output / "report.json").unlink(missing_ok=True)
    profile, reference_path = qualification_inputs()
    recorded = provenance()
    recorded["qualification_profile"] = profile
    _write(output / "provenance.json", recorded)
    description = _process(executable, ["describe", "impedance"], output, "resolved")
    frames = _process(
        executable,
        ["frames", "impedance", str(reference_path)],
        output,
        "frame-binding",
    )
    measured = description["scene_pose"]
    pose = Pose(
        "world", tuple(measured["position"]), tuple(measured["quaternion_wxyz"])
    )
    events, approvals = generate_trace(pose)
    trace = output / "authorization.jsonl"
    write_trace(trace, events)
    _write(output / "approvals.json", approvals)
    modes: dict[str, Any] = {}
    for mode in MODES:
        _process(executable, ["describe", mode], output, f"resolved-{mode}")
        native: dict[str, Any] = {}
        for fixture in ("home", "nearby", "singular"):
            native[fixture] = _process(
                executable,
                ["native", mode, fixture, "baseline"],
                output,
                f"{mode}-{fixture}",
            )
        for variant in (
            "invalid_joint",
            "invalid_frame",
            "nonfinite",
            "non_unit",
            "gravity",
            "moving",
            "friction",
            "coriolis",
            "smoothing",
            "rotation",
            "responsive_probe",
            "historical",
            "feedback",
        ):
            native[variant] = _process(
                executable,
                ["native", mode, "home", variant],
                output,
                f"{mode}-{variant}",
                unsafe=variant in ("nonfinite", "non_unit"),
            )
        guarded = _process(
            executable, ["guarded", mode, str(trace)], output, f"{mode}-guarded"
        )
        expected_cases = [e["case"] for e in events if e["kind"] == "step"]
        if [row["case"] for row in guarded] != expected_cases:
            raise RuntimeError("incomplete guard evidence")
        gravity = native["gravity"]
        gravity_error = max(
            abs(a - b)
            for a, b in zip(gravity["effort"], gravity["gravity_expected"], strict=True)
        )
        if gravity_error > 1e-6:
            raise RuntimeError(f"gravity convention mismatch: {mode} {gravity_error}")
        compensation_errors = {}
        for variant in ("friction", "coriolis"):
            delta = [
                a - b
                for a, b in zip(
                    native[variant]["effort"], native["moving"]["effort"], strict=True
                )
            ]
            error = max(
                abs(a - b)
                for a, b in zip(
                    delta, native[variant]["expected_compensation"], strict=True
                )
            )
            compensation_errors[variant] = error
            if error > 1e-6:
                raise RuntimeError(f"{variant} convention mismatch: {mode} {error}")
        for fixture in ("home", "nearby", "singular"):
            row = native[fixture]
            if row["producer_pid"] == row["consumer_pid"]:
                raise RuntimeError("publisher/consumer must be separate processes")
        timing = []
        for rate in (250, 500, 1000):
            for repeat in range(3):
                fixture = "home"
                result = _process(
                    executable,
                    ["benchmark", mode, str(rate), fixture],
                    output,
                    f"{mode}-{rate}-{repeat}",
                )
                result["repeat"] = repeat
                timing.append(result)
        numeric_pass = all(native[f]["finite"] for f in ("home", "nearby", "singular"))
        translation_response = any(
            abs(v) > 1e-5 for v in native["home"]["fresh_effort"]
        )
        rotation_response = any(
            abs(v) > 1e-5 for v in native["rotation"]["fresh_effort"]
        )
        pose_errors = {}
        for fixture in ("home", "nearby", "singular", "rotation"):
            row = native[fixture]
            pose_errors[fixture] = max(
                abs(a - b)
                for a, b in zip(
                    row["fresh_effort"], row["expected_effort"], strict=True
                )
            )
        feedback = native["feedback"]
        feedback_errors = {
            kind: max(
                abs(a - b)
                for a, b in zip(
                    feedback[kind + "_effort"],
                    feedback[kind + "_expected"],
                    strict=True,
                )
            )
            for kind in ("position", "velocity")
        }
        numeric_pass = (
            numeric_pass
            and max(pose_errors.values()) < 1e-6
            and max(feedback_errors.values()) < 1e-6
        )
        feedback_response = all(
            any(abs(v) > 1e-5 for v in feedback[kind + "_effort"])
            for kind in ("position", "velocity")
        )
        functional_pass = (
            numeric_pass
            and translation_response
            and rotation_response
            and feedback_response
            and frames["pass"]
        )
        # Native nonfinite/stamp/frame gaps remain findings; the independent
        # guarded fault assertions are mandatory and fail execution on mismatch.
        modes[mode] = dict(
            native=native,
            guarded=guarded,
            timing=timing,
            functional_pass=functional_pass,
            numeric_pass=numeric_pass,
            pose_effort_errors_nm=pose_errors,
            feedback_errors_nm=feedback_errors,
            feedback_response=feedback_response,
            translation_response=translation_response,
            rotation_response=rotation_response,
            gravity_max_error_nm=gravity_error,
            compensation_errors_nm=compensation_errors,
        )
    report = dict(
        schema_version=2,
        frame_binding=frames,
        status="completed",
        decision=decision(modes, enforce_timing=enforce_timing),
        modes=modes,
        provenance=recorded,
        limitations=[
            "stationary mock interfaces, no torque integration",
            "zero mock effort is not a physical hold",
            "timing is local throughput, not hardware real-time evidence",
        ],
        pr13_requirements=[
            "production guard and independent watchdog",
            "validated physical stop/hold and recovery",
            "moving simulation dynamics and compensation",
            "UR driver/version/effort ownership validation",
        ],
    )
    _write(output / "report.json", report)
    return report
