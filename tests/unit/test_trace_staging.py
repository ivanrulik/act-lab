"""Staged diagnostics preserve ordered bytes and refuse false finalization."""

import pytest

from act_lab.adapters.ros2.trace_staging import DiagnosticFile, staging_root


@pytest.mark.parametrize("complete", [True, False])
def test_staged_trace_retains_exact_bytes_at_commit(tmp_path, complete):
    staging = tmp_path / "staging"
    staging.mkdir()
    target = tmp_path / "trace.json"
    trace = DiagnosticFile(target, staging)
    trace.partial.write_bytes(b"[1,2,3]\n")
    assert not target.exists() and not trace.output_partial.exists()
    trace.commit(complete=complete)
    result = target if complete else trace.output_partial
    assert result.read_bytes() == b"[1,2,3]\n"
    assert not list(staging.iterdir())


def test_requested_staging_does_not_fall_back_to_disk(tmp_path, monkeypatch):
    monkeypatch.setenv("ACT_LAB_REFERENCE_DIR", str(tmp_path))
    with pytest.raises(RuntimeError, match="tmpfs"):
        staging_root()


def test_failed_copy_remains_partial_and_cannot_publish(tmp_path, monkeypatch):
    staging = tmp_path / "staging"
    staging.mkdir()
    target = tmp_path / "trace.json"
    trace = DiagnosticFile(target, staging)
    trace.partial.write_bytes(b"[1,2,3]\n")

    def failed(*args, **kwargs):
        raise OSError("disk full")

    monkeypatch.setattr("shutil.copyfileobj", failed)
    with pytest.raises(OSError, match="disk full"):
        trace.commit(complete=True)
    assert not target.exists() and trace.output_partial.exists()
    assert trace.partial.read_bytes() == b"[1,2,3]\n"
