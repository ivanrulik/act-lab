"""Primitive camera-source freshness checks, independent of ROS and rendering."""

from __future__ import annotations

import math
from typing import Any
from uuid import UUID

STALE_NS = 100_000_000


class CameraSource:
    def __init__(self, model_sha256: str, nq: int, nv: int) -> None:
        self.model_sha256 = model_sha256
        self.nq, self.nv = nq, nv
        self.current: dict[str, Any] | None = None
        self.reason = "missing camera source"
        self.valid = False
        self._retired: set[str] = set()

    def offer(self, packet: dict[str, Any], steady_now_ns: int) -> bool:
        try:
            if type(packet["episode"]) is not str:
                raise ValueError("invalid camera episode")
            UUID(packet["episode"])
            if (
                packet["schema_version"] != 1
                or packet["model_sha256"] != self.model_sha256
            ):
                raise ValueError("camera model/schema mismatch")
            for key in (
                "capture_sequence",
                "physics_tick",
                "domain_ns",
                "capture_steady_ns",
            ):
                if type(packet[key]) is not int or packet[key] < 0:
                    raise ValueError("invalid camera source identity")
            for key, size in (("qpos", self.nq), ("qvel", self.nv)):
                if len(packet[key]) != size or not all(
                    type(v) in (int, float) and math.isfinite(v) for v in packet[key]
                ):
                    raise ValueError("invalid camera state")
            age = steady_now_ns - packet["capture_steady_ns"]
            if age < 0 or age >= STALE_NS:
                raise ValueError("camera source future/stale")
            if packet["episode"] in self._retired:
                raise ValueError("retired camera episode")
            if self.current:
                old = self.current
                if packet["episode"] == old["episode"]:
                    if packet["capture_sequence"] <= old["capture_sequence"]:
                        self.reason = "duplicate/replayed camera source"
                        return False
                    if packet["domain_ns"] <= old["domain_ns"]:
                        if packet["domain_ns"] < old["domain_ns"]:
                            self._retired.add(old["episode"])
                        raise ValueError(
                            "camera clock did not progress; reset requires new episode"
                        )
                else:
                    self._retired.add(old["episode"])
            self.current = dict(packet)
            self.valid = True
            self.reason = "live camera source"
            return True
        except (KeyError, TypeError, ValueError) as error:
            self.valid = False
            self.reason = str(error)
            return False

    def stale(self, steady_now_ns: int) -> bool:
        return (
            not self.valid
            or self.current is None
            or not (0 <= steady_now_ns - self.current["capture_steady_ns"] < STALE_NS)
        )


def can_publish_frame(
    captured: dict[str, Any], latest: dict[str, Any] | None, steady_now_ns: int
) -> bool:
    """Discard stale work and reset-crossing work without changing capture identity."""
    return bool(
        latest
        and captured["episode"] == latest["episode"]
        and captured["model_sha256"] == latest["model_sha256"]
        and 0 <= steady_now_ns - captured["capture_steady_ns"] < STALE_NS
    )
