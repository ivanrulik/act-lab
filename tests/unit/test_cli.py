import json

from act_lab.cli import main


def test_doctor_reports_supported_runtime(capsys: object) -> None:
    assert main(["doctor", "--json"]) == 0
    captured = capsys.readouterr()  # type: ignore[attr-defined]
    report = json.loads(captured.out)
    assert report["status"] == "ok"
    assert report["python_supported"] is True

