"""Small timestamp-aware 1 Euro filter for interactive hand input."""

from __future__ import annotations

import math


class OneEuroFilter:
    def __init__(
        self, min_cutoff_hz: float, beta: float, derivative_cutoff_hz: float
    ) -> None:
        self._min_cutoff_hz = min_cutoff_hz
        self._beta = beta
        self._derivative_cutoff_hz = derivative_cutoff_hz
        self.reset()

    def reset(self) -> None:
        self._timestamp_ns: int | None = None
        self._value: float | None = None
        self._raw_value: float | None = None
        self._derivative = 0.0

    def filter(self, value: float, timestamp_ns: int) -> float:
        if self._timestamp_ns is None or self._value is None:
            self._timestamp_ns = timestamp_ns
            self._value = value
            self._raw_value = value
            return value
        dt = (timestamp_ns - self._timestamp_ns) / 1_000_000_000
        if dt <= 0.0:
            return self._value
        assert self._raw_value is not None
        raw_derivative = (value - self._raw_value) / dt
        derivative_alpha = _alpha(self._derivative_cutoff_hz, dt)
        self._derivative += derivative_alpha * (raw_derivative - self._derivative)
        cutoff = self._min_cutoff_hz + self._beta * abs(self._derivative)
        value_alpha = _alpha(cutoff, dt)
        self._value += value_alpha * (value - self._value)
        self._raw_value = value
        self._timestamp_ns = timestamp_ns
        return self._value


def _alpha(cutoff_hz: float, dt: float) -> float:
    tau = 1.0 / (2.0 * math.pi * cutoff_hz)
    return 1.0 / (1.0 + tau / dt)
