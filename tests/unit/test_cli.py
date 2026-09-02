import json
from pathlib import Path

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


def test_expert_validates_arguments(capsys: object) -> None:
    assert main(["sim", "expert", "--episodes", "0"]) == 2
    captured = capsys.readouterr()  # type: ignore[attr-defined]
    assert "episodes must be positive" in captured.err

    assert main(["sim", "expert", "--min-success-rate", "1.1"]) == 2
    captured = capsys.readouterr()  # type: ignore[attr-defined]
    assert "min-success-rate must be in [0, 1]" in captured.err


def test_expert_json_schema_and_exit_codes(capsys: object) -> None:
    arguments = [
        "sim",
        "expert",
        "--seed-start",
        "0",
        "--episodes",
        "1",
        "--min-success-rate",
        "1",
        "--json",
    ]
    assert main(arguments) == 0
    report = json.loads(capsys.readouterr().out)  # type: ignore[attr-defined]
    assert report["success_rate"] == 1.0
    assert report["threshold_passed"] is True
    episode = report["episodes"][0]
    assert set(episode) == {
        "final_phase",
        "result",
        "safety_outcomes",
        "seed",
        "steps",
        "terminal_reason",
    }

    arguments[arguments.index("0")] = "10"
    assert main(arguments) == 1
    failed = json.loads(capsys.readouterr().out)  # type: ignore[attr-defined]
    assert failed["threshold_passed"] is False


def test_compose_has_authoritative_keyboard_and_expert_commands() -> None:
    compose = Path("compose.yaml").read_text()
    assert "keyboard-teleop:" in compose
    assert "act-lab sim keyboard-teleop" in compose
    assert "expert:" in compose
    assert "act-lab sim expert --seed-start 0 --episodes 20" in compose
