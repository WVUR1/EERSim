"""Opt-in, bounded wall-time diagnostics owned by one simulation thread.

Set EERSIM_PROFILE=1 before creating a Session/World or spawning PhysX. Timings
are inclusive: parent and child spans overlap and must not be added together.
Counts, totals, means and maxima cover all completed spans since the last reset;
percentiles describe the most recent ``sample_limit`` completions per name.
No random sampling, simulation-clock reads, or physics calls are performed.
"""
from __future__ import annotations

from collections import deque
import math
from numbers import Real
import os
import time


class _NullSpan:
    __slots__ = ()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False


_NULL_SPAN = _NullSpan()


class _Span:
    __slots__ = ("recorder", "name", "started", "epoch")

    def __init__(self, recorder, name):
        self.recorder, self.name = recorder, name

    def __enter__(self):
        self.epoch = self.recorder._epoch
        self.started = self.recorder._clock()
        return self

    def __exit__(self, *_):
        duration = (self.recorder._clock() - self.started) / 1_000_000_000
        if self.epoch == self.recorder._epoch:
            self.recorder._record(self.name, duration)
        return False


class PerformanceRecorder:
    """Bounded single-thread recorder; disabled operations never read a clock.

    ``snapshot(reset=True)`` returns the previous interval, then clears timing
    samples and counters. Latest gauges persist, so scene inventory remains
    available after excluding setup. Spans crossing a reset are discarded.
    An enabled recorder can be attached as ``owner.performance`` explicitly.
    """

    def __init__(self, enabled: bool | None = None, sample_limit: int = 2048,
                 *, max_names: int = 128, clock=None):
        for name, value in (("sample_limit", sample_limit), ("max_names", max_names)):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        self.enabled = (os.environ.get("EERSIM_PROFILE", "").strip().lower() in
                        {"1", "true", "yes", "on"}) if enabled is None else bool(enabled)
        self.sample_limit, self.max_names = sample_limit, max_names
        self._clock = clock or time.perf_counter_ns
        self._spans, self._counters, self._gauges = {}, {}, {}
        self._epoch = self._dropped = 0

    def span(self, name: str):
        return _Span(self, name) if self.enabled else _NULL_SPAN

    def _admit(self, collection, name):
        if not isinstance(name, str) or not name:
            raise ValueError("A performance metric name must be a nonempty string")
        if name not in collection and len(collection) >= self.max_names:
            self._dropped += 1
            return False
        return True

    def _record(self, name, duration):
        if not math.isfinite(duration) or duration < 0:
            raise ValueError("Performance clock must be finite and monotonic")
        if not self._admit(self._spans, name):
            return
        if name not in self._spans:
            self._spans[name] = [0, 0., 0., deque(maxlen=self.sample_limit)]
        entry = self._spans[name]
        entry[0] += 1
        entry[1] += duration
        entry[2] = max(entry[2], duration)
        entry[3].append(duration)

    @staticmethod
    def _number(value):
        if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
            raise ValueError("A performance counter/gauge must be a finite number")
        return int(value) if isinstance(value, int) else float(value)

    def increment(self, name: str, value=1):
        if self.enabled and self._admit(self._counters, name):
            self._counters[name] = self._number(self._counters.get(name, 0) + self._number(value))

    def gauge(self, name: str, value):
        if self.enabled and self._admit(self._gauges, name):
            self._gauges[name] = self._number(value)

    @staticmethod
    def _percentile(values, quantile):
        position = (len(values) - 1) * quantile
        lower = int(position)
        upper = min(lower + 1, len(values) - 1)
        return values[lower] + (values[upper] - values[lower]) * (position - lower)

    def snapshot(self, reset: bool = False) -> dict:
        spans = {}
        for name, (count, total, maximum, samples) in self._spans.items():
            values = sorted(samples)
            spans[name] = {"count": count, "total": total, "mean": total / count,
                           "p50": self._percentile(values, .5), "p95": self._percentile(values, .95),
                           "max": maximum, "sample_count": len(values)}
        result = {"enabled": self.enabled, "unit": "seconds", "sample_limit": self.sample_limit,
                  "sample_policy": "most recent completions per stage", "timing_scope": "inclusive; nested spans overlap",
                  "max_names_per_collection": self.max_names, "dropped_metrics": self._dropped,
                  "spans": spans, "counters": dict(self._counters), "gauges": dict(self._gauges)}
        if reset:
            self._spans.clear()
            self._counters.clear()
            self._dropped = 0
            self._epoch += 1
        return result
