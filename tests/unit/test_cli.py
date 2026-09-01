import json

from act_lab.cli import main


def test_doctor_reports_supported_runtime(capsys: object) -> None:
    assert main(["doctor", "--json"]) == 0
    captured = capsys.readouterr()  # type: ignore[attr-defined]
    report = json.loads(captured.out)
    assert report["status"] == "ok"
    assert report["python_supported"] is True


def test_sim_rollout_rejects_non_positive_steps(capsys: object) -> None:
    assert main(["sim", "rollout", "--steps", "0"]) == 2
    captured = capsys.readouterr()  # type: ignore[attr-defined]
    assert "steps must be positive" in captured.err


def test_control_smoke_reports_safe_outcomes(capsys: object) -> None:
    assert main(["sim", "control-smoke", "--seed", "3", "--steps", "3", "--json"]) == 0
    captured = capsys.readouterr()  # type: ignore[attr-defined]
    report = json.loads(captured.out)
    assert report["status"] == "ok"
    assert report["environment_steps"] == 3
    assert sum(report["command_outcomes"].values()) == 3
