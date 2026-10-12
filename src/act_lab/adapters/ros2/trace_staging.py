"""Optional bounded Compose staging for diagnostic evidence, never authority."""

from __future__ import annotations

import os
import shutil
import tempfile
from pathlib import Path


def staging_root() -> Path | None:
    value = os.environ.get("ACT_LAB_REFERENCE_DIR")
    if not value:
        return None
    root = Path(value)
    mounted = any(
        fields[1] == str(root) and fields[2] == "tmpfs"
        for line in Path("/proc/mounts").read_text().splitlines()
        if len(fields := line.split()) >= 3
    )
    if not mounted:
        raise RuntimeError("recording evidence staging requires the Compose tmpfs")
    return root


class DiagnosticFile:
    """Write a bounded staged prefix; persist it only after motion is disabled."""

    def __init__(self, final: Path, root: Path | None = None) -> None:
        self.final = final
        self.output_partial = final.with_name(final.name + ".partial")
        self.staging = (
            tempfile.TemporaryDirectory(prefix="trace-", dir=root)
            if root is not None
            else None
        )
        self.partial = (
            Path(self.staging.name) / self.output_partial.name
            if self.staging is not None
            else self.output_partial
        )

    def commit(self, *, complete: bool) -> None:
        if self.staging is not None:
            with (
                self.partial.open("rb") as reader,
                self.output_partial.open("xb") as writer,
            ):
                shutil.copyfileobj(reader, writer, length=1024 * 1024)
                writer.flush()
                os.fsync(writer.fileno())
        if complete:
            os.link(self.output_partial, self.final)
            self.output_partial.unlink()
        fd = os.open(self.final.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
        if self.staging is not None:
            self.staging.cleanup()
