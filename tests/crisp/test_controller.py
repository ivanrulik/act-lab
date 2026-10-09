"""Required real-plugin tests. Missing ROS/bench packages fail, never skip."""

from __future__ import annotations

import json

import pytest

from act_lab.adapters.ros2.crisp import run_feasibility


@pytest.fixture(scope="module")
def report(tmp_path_factory: pytest.TempPathFactory) -> dict:
    return run_feasibility(tmp_path_factory.mktemp("crisp"), enforce_timing=False)


@pytest.mark.parametrize("mode", ["impedance", "operational_space"])
def test_actual_plugin_lifecycle_interfaces_and_guard(report: dict, mode: str) -> None:
    evidence = report["modes"][mode]
    assert evidence["numeric_pass"]
    # The requested filter=1 baseline can correctly produce a completed no-go.
    assert not evidence["functional_pass"]
    assert not evidence["translation_response"]
    assert not evidence["rotation_response"]
    rows = {r["case"]: r for r in evidence["guarded"]}
    for case in (
        "startup_missing",
        "source_exact_boundary",
        "receipt_exact_boundary_gateway_loss",
        "pause_exact_boundary",
        "resume_requires_fresh",
        "backward_reset",
        "same_episode_reset_rejected",
        "replay",
        "wrong_episode",
        "future_timestamp",
        "invalid_frame",
        "invalid_numeric",
        "malformed_shape",
        "invalid_sequence",
        "disabled",
        "nonfinite_raw_effort",
        "shutdown_immediate_zero",
    ):
        assert not rows[case]["enabled"]
        assert rows[case]["guarded_effort"] == [0.0] * 6
    for case in (
        "approved",
        "pause_recovery_reactivated",
        "new_episode_recovery",
        "numeric_recovery",
    ):
        assert rows[case]["enabled"]
        assert rows[case]["lifecycle_reactivated"]
    assert rows["ceiling_slew_29"]["guarded_effort"] == [5.0] * 6
    assert evidence["gravity_max_error_nm"] < 1e-6
    assert all(v < 1e-6 for v in evidence["compensation_errors_nm"].values())
    for variant in ("invalid_joint", "invalid_frame"):
        assert (
            evidence["native"][variant].get("configuration_rejected")
            or evidence["native"][variant]["isolated_exit_code"] != 0
        )


@pytest.mark.parametrize("mode", ["impedance", "operational_space"])
def test_dds_native_gaps_are_visible(report: dict, mode: str) -> None:
    row = report["modes"][mode]["native"]["responsive_probe"]
    assert row["producer_pid"] != row["consumer_pid"]
    assert any(abs(v) > 1e-5 for v in row["fresh_effort"])
    for case in (
        "stale_effort",
        "future_effort",
        "unsupported_frame_effort",
        "publisher_loss_effort",
        "deactivated_effort_buffer",
    ):
        assert row[case] == pytest.approx(row["fresh_effort"], abs=1e-6)


def test_complete_provenance_and_benchmarks(report: dict) -> None:
    assert report["status"] == "completed"
    assert report["provenance"]["rmw"] == "rmw_fastrtps_cpp"
    assert len(report["provenance"]["model_sha256"]) == 64
    assert not report["provenance"]["licensing"]["consistent_metadata"]
    for evidence in report["modes"].values():
        assert len(evidence["timing"]) == 9
        assert all(
            r["samples"] == 10_000 and r["warmup"] == 1_000 for r in evidence["timing"]
        )
    json.dumps(report, allow_nan=False)
