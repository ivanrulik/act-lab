"""ROS-free trace generation, decision criteria and lazy import contract."""

from __future__ import annotations

import copy
import subprocess
import sys
from pathlib import Path

import pytest

from act_lab.adapters.ros2.crisp import _process, decision, run_feasibility
from act_lab.adapters.ros2.crisp_trace import (
    generate_trace,
    validate_trace,
    write_trace,
)
from act_lab.adapters.ros2.demonstrator import HOME


def test_trace_boundaries_and_gripper(tmp_path: Path) -> None:
    events, approvals = generate_trace(HOME)
    steps = {e["case"]: e for e in events if e["kind"] == "step"}
    for prefix in ("source", "receipt", "pause"):
        assert steps[prefix + "_before_boundary"]["expected_enabled"]
    assert not steps["source_exact_boundary"]["expected_enabled"]
    assert not steps["receipt_exact_boundary_gateway_loss"]["expected_enabled"]
    assert not steps["pause_exact_boundary"]["expected_enabled"]
    assert not steps["resume_requires_fresh"]["expected_enabled"]
    assert steps["pause_recovery_reactivated"]["expected_enabled"]
    assert steps["new_episode_recovery"]["expected_enabled"]
    assert not steps["same_episode_reset_rejected"]["expected_enabled"]
    rejected = [a for a in approvals if a.get("case", "").startswith("application_")]
    assert len(rejected) == 4
    assert {a["outcome"] for a in rejected} == {"invalid", "stale"}
    accepted = approvals[-2]["measured_gripper"]
    assert approvals[-1]["retained_gripper"] == accepted
    assert approvals[0]["command"]["source_timestamp_ns"] == 0
    write_trace(tmp_path / "trace.jsonl", events)
    assert len((tmp_path / "trace.jsonl").read_text().splitlines()) == len(events)


@pytest.mark.parametrize(
    "mutation", ["steady", "shape", "sequence", "numeric", "duplicate"]
)
def test_reject_malformed_trace(mutation: str) -> None:
    events, _ = generate_trace(HOME)
    events = copy.deepcopy(events)
    authorization = next(e for e in events if e["kind"] == "authorize")
    if mutation == "steady":
        events[-1]["steady_ns"] = -1
    elif mutation == "shape":
        authorization["command"]["position"] = [1]
    elif mutation == "sequence":
        authorization["command"]["sequence"] = 2**64
    elif mutation == "numeric":
        authorization["command"]["position"][0] = float("nan")
    else:
        repeated = copy.deepcopy(next(e for e in events if e["kind"] == "step"))
        repeated["steady_ns"] = events[-1]["steady_ns"]
        events.append(repeated)
    with pytest.raises(ValueError):
        validate_trace(events)


def evidence(p99: int = 1_999_999, functional: bool = True) -> dict:
    return dict(
        functional_pass=functional,
        timing=[
            dict(rate_hz=500, samples=10_000, warmup=1_000, p99_ns=p99)
            for _ in range(3)
        ],
    )


def test_decision_prioritizes_impedance_and_exact_timing_boundary() -> None:
    both = dict(impedance=evidence(), operational_space=evidence())
    assert decision(both)["selected_mode"] == "impedance"
    both["impedance"] = evidence(2_000_000)
    assert decision(both)["selected_mode"] == "operational_space"
    both["operational_space"] = evidence(functional=False)
    assert decision(both)["outcome"] == "no_go"
    assert not decision(both)["hardware_approved"]
    assert decision(both, enforce_timing=False)["outcome"] == "ci_evidence_only"
    both["impedance"]["timing"].pop()
    with pytest.raises(RuntimeError, match="incomplete"):
        decision(both)


def test_missing_bench_is_actionable(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("PATH", "")
    with pytest.raises(
        RuntimeError, match="docker compose --profile crisp-feasibility"
    ):
        run_feasibility(tmp_path)


def test_import_isolation() -> None:
    code = """
import sys
import act_lab.domain
import act_lab.application
import act_lab.adapters.ros2.crisp
blocked = {'rclpy', 'pinocchio', 'mujoco', 'torch'}
assert not any(k.split('.')[0] in blocked for k in sys.modules)
"""
    subprocess.run([sys.executable, "-c", code], check=True)


def test_native_unsafe_case_does_not_mask_harness_failure(tmp_path: Path) -> None:
    with pytest.raises(RuntimeError, match="bench failed"):
        _process(
            sys.executable,
            ["-c", "raise SystemExit(1)"],
            tmp_path,
            "broken",
            unsafe=True,
        )
    code = "import json; print(json.dumps(dict(complete=True)))"
    result = _process(sys.executable, ["-c", code], tmp_path, "valid")
    assert result == {"complete": True}
