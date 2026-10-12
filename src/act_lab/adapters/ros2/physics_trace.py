"""Bounded-memory, atomically finalized physics evidence; no ROS dependency."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from act_lab.adapters.ros2.trace_staging import DiagnosticFile, staging_root


class PhysicsTrace:
    """Retain one editable tick while streaming prior ticks in exact order.

    The pending tick allows the owner to record its complete wall budget after
    append. Earlier step objects are released instead of accumulating in the
    control process for a whole sustained run. The final JSON array becomes
    visible only after close; interrupted prefixes remain explicitly partial.
    """

    def __init__(self, folder: Path) -> None:
        self.file = DiagnosticFile(folder / "physics.json", staging_root())
        self.partial = self.file.partial
        self.final = self.file.final
        self.stream = self.partial.open("w", buffering=1024 * 1024)
        self.stream.write("[")
        self.pending: dict[str, Any] | None = None
        self.count = 0
        self.closed = False

    def append(self, row: dict[str, Any]) -> None:
        if self.closed:
            raise RuntimeError("physics evidence is already finalized")
        self._write_pending()
        self.pending = row

    def __getitem__(self, index: int) -> dict[str, Any]:
        if index != -1 or self.pending is None:
            raise IndexError("only the current physics tick remains in memory")
        return self.pending

    def _write_pending(self) -> None:
        if self.pending is None:
            return
        # Validate before changing the prefix. Nonfinite evidence cannot be
        # published as a completed trace.
        encoded = json.dumps(self.pending, allow_nan=False, separators=(",", ":"))
        if self.count:
            self.stream.write(",")
        self.stream.write(encoded)
        self.pending = None
        self.count += 1

    def close(self) -> None:
        if self.closed:
            return
        try:
            self._write_pending()
            self.stream.write("]\n")
        except BaseException:
            self.stream.close()
            self.file.commit(complete=False)
            raise
        finally:
            self.stream.close()
            self.closed = True
        self.file.commit(complete=True)
