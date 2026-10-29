"""Bounded diagnostic timing. Values never participate in control or QC."""

import math
import threading
import time
from collections import deque


def performance_config(value=None):
    defaults = {"enabled": True, "window_samples": 512, "max_series": 128}
    if value is None:
        return defaults
    if not isinstance(value, dict) or set(value) - set(defaults):
        raise ValueError("invalid performance config")
    c = {**defaults, **value}
    if type(c["enabled"]) is not bool:
        raise ValueError("performance.enabled must be boolean")
    for key, maximum in [("window_samples", 8192), ("max_series", 512)]:
        if type(c[key]) is not int or not 1 <= c[key] <= maximum:
            raise ValueError("invalid performance " + key)
    return c


class Performance:
    def __init__(self, config=None):
        self.config = performance_config(config)
        self.lock = threading.Lock()
        self.series = {}
        self.rejected = 0

    def observe(self, name, value):
        if not self.config["enabled"]:
            return
        if not math.isfinite(value) or value < 0:
            return
        with self.lock:
            if name not in self.series:
                if len(self.series) >= self.config["max_series"]:
                    self.rejected += 1
                    return
                self.series[name] = {
                    "values": deque(maxlen=self.config["window_samples"]),
                    "count": 0,
                    "sum": 0.0,
                    "max": 0.0,
                }
            s = self.series[name]
            s["values"].append(value)
            s["count"] += 1
            s["sum"] += value
            s["max"] = max(s["max"], value)

    def elapsed(self, name, start_ns):
        self.observe(name, (time.monotonic_ns() - start_ns) / 1e6)

    def snapshot(self):
        with self.lock:
            copied = {
                k: (list(s["values"]), s["count"], s["sum"], s["max"])
                for k, s in self.series.items()
            }
            rejected = self.rejected
        result = {}
        for name, (values, count, total, maximum) in copied.items():
            values.sort()
            result[name] = {
                "count": count,
                "window_count": len(values),
                "mean_ms": total / count,
                "max_ms": maximum,
                "window_p50_ms": values[math.ceil(len(values) * 0.5) - 1],
                "window_p95_ms": values[math.ceil(len(values) * 0.95) - 1],
            }
        return {
            "schema_version": "performance-snapshot/1",
            "host_monotonic_ns": time.monotonic_ns(),
            "enabled": self.config["enabled"],
            "window_samples": self.config["window_samples"],
            "series": result,
            "rejected_series_samples": rejected,
            "clock": "durations in host monotonic; source gaps use consecutive source stamps only",
        }
