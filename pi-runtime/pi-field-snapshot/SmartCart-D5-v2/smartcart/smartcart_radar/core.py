"""Dependency-free scan validation, geometry and latest-value state.

ROS axes: x forward, y left, z up. No interpolation, scan fusion, or control.
All non-finite returns become JSON null; their angular indices are retained.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Any

SCHEMA = "smartcart.scan.v1"
MAX_RAYS = 100_000
STALE_SECONDS = 1.5


def finite(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def required_number(data: dict, key: str) -> float:
    value = finite(data.get(key))
    if value is None:
        raise ValueError(f"{key}: a finite number is required")
    return value


def normalise_scan(data: dict, *, kind: str = "replay", label: str = "File replay") -> dict:
    """Validate a LaserScan-like object. Never synthesize missing bearings.

    A trailing '...' is a truncation marker, not a no-return ray. A marker in
    the middle is rejected: indices after it cannot be recovered reliably.
    """
    if not isinstance(data, dict):
        raise ValueError("A scan must be a JSON object")
    if isinstance(data.get("scan"), dict):
        data = data["scan"]
    amin = required_number(data, "angle_min")
    amax = required_number(data, "angle_max")
    step = required_number(data, "angle_increment")
    rmin = required_number(data, "range_min")
    rmax = required_number(data, "range_max")
    if step == 0 or (amax - amin) / step < 0:
        raise ValueError("Inconsistent angle_min / angle_max / angle_increment")
    expected = int(round((amax - amin) / step)) + 1
    if not 1 <= expected <= MAX_RAYS or abs(amax - amin) > 2 * math.pi + 0.1:
        raise ValueError("Invalid or excessive angular geometry")
    if not 0 <= rmin < rmax <= 100_000:
        raise ValueError("Invalid range_min / range_max")
    ranges = data.get("ranges")
    if not isinstance(ranges, (list, tuple)) or not 1 <= len(ranges) <= MAX_RAYS:
        raise ValueError("ranges must be a nonempty, bounded array")

    values: list[float | None] = []
    trunc = bool(data.get("truncated", False))
    for index, value in enumerate(ranges):
        if value == "...":
            if index != len(ranges) - 1:
                raise ValueError("Only a trailing '...' can be interpreted without guessing indices")
            trunc = True
            break
        # Preserve null/NaN/Inf locations, but reject arbitrary text.
        if value is not None and not isinstance(value, (int, float)):
            raise ValueError(f"ranges[{index}] is not numeric or null")
        values.append(finite(value))
    if not values:
        raise ValueError("No numeric ray positions were provided")
    trunc = trunc or len(values) < expected
    intensity_raw = data.get("intensities", [])
    if not isinstance(intensity_raw, (list, tuple)) or len(intensity_raw) > MAX_RAYS:
        raise ValueError("intensities must be a bounded array")
    intensities = []
    for value in intensity_raw:
        if value == "...":
            break
        intensities.append(finite(value))
    header = data.get("header") or {}
    if not isinstance(header, dict):
        raise ValueError("header must be an object")
    stamp = header.get("stamp") or {}
    if not isinstance(stamp, dict):
        raise ValueError("header.stamp must be an object")
    warnings = []
    if trunc:
        warnings.append("TRUNCATED_SCAN")
    if len(values) != expected:
        warnings.append("RAY_COUNT_MISMATCH")
    if intensities and len(intensities) != len(values):
        warnings.append("INTENSITY_COUNT_MISMATCH")
    below = sum(v is not None and v < rmin for v in values)
    above = sum(v is not None and v > rmax for v in values)
    no_return = sum(v is None for v in values)
    valid = len(values) - below - above - no_return
    if below:
        warnings.append("BELOW_SENSOR_MINIMUM")
    if not valid:
        warnings.append("NO_VALID_RETURNS")
    previous_source = data.get("source") or {}
    original_kind = (previous_source.get("original_kind") or previous_source.get("kind")) if isinstance(previous_source, dict) else None
    scan_time = max(0.0, finite(data.get("scan_time")) or 0.0)
    return {
        "schema": SCHEMA,
        "source": {"kind": kind, "label": str(label)[:240], "original_kind": original_kind},
        "header": {
            "frame_id": str(header.get("frame_id", "laser"))[:240],
            "stamp": {"sec": int(finite(stamp.get("sec")) or 0),
                      "nanosec": int(finite(stamp.get("nanosec")) or 0)},
        },
        "angle_min": amin, "angle_max": amax, "angle_increment": step,
        "time_increment": max(0.0, finite(data.get("time_increment")) or 0.0),
        "scan_time": scan_time, "range_min": rmin, "range_max": rmax,
        "ranges": values, "intensities": intensities, "truncated": trunc,
        "warnings": warnings,
        "stats": {
            "provided_rays": len(values), "expected_rays": expected,
            "valid_sensor_rays": valid, "no_return_rays": no_return,
            "below_min_rays": below, "above_max_rays": above,
            "metadata_scan_hz": 1.0 / scan_time if scan_time else None,
            "provided_span_deg": abs(step) * max(0, len(values) - 1) * 180 / math.pi,
        },
    }


def project_scan(scan: dict, radius: float = 5.0, yaw_deg: float = 0.0,
                 offset_x: float = 0.0, offset_y: float = 0.0) -> dict:
    """Apply a fixed planar laser-to-cart transform, then cart-centered crop.

    Sector minima are distances from the cart origin, NOT footprint clearance.
    No-return and discarded readings mean unknown, not free space.
    """
    if not all(math.isfinite(v) for v in (radius, yaw_deg, offset_x, offset_y)) or radius <= 0:
        raise ValueError("Invalid projection settings")
    yaw = math.radians(yaw_deg)
    points = []
    sectors = {name: {"min_m": None, "count": 0} for name in ("front", "left", "right", "rear")}
    outside = 0
    for i, r in enumerate(scan["ranges"]):
        if r is None or not math.isfinite(r) or r < scan["range_min"] or r > scan["range_max"]:
            continue
        a = scan["angle_min"] + i * scan["angle_increment"] + yaw
        x = offset_x + r * math.cos(a)
        y = offset_y + r * math.sin(a)
        d = math.hypot(x, y)
        if d > radius:
            outside += 1
            continue
        bearing = math.degrees(math.atan2(y, x))
        sector = ("front" if -45 <= bearing < 45 else
                  "left" if 45 <= bearing < 135 else
                  "right" if -135 <= bearing < -45 else "rear")
        s = sectors[sector]
        s["count"] += 1
        s["min_m"] = d if s["min_m"] is None else min(s["min_m"], d)
        points.append({"index": i, "x": x, "y": y, "distance": d, "range": r, "bearing_deg": bearing})
    return {"points": points, "sectors": sectors, "outside_view": outside}


class ScanStore:
    """Bounded latest-value mailbox. Producer cadence is independent of UI cadence."""
    def __init__(self, source_mode: str = "ros", stale_seconds: float = STALE_SECONDS):
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.source_mode = source_mode
        self.stale_seconds = stale_seconds
        self.latest = None
        self.last_received = None
        self.arrivals = deque(maxlen=300)
        self.sequence = 0
        self.error = None
        self.ended = False
        self.rejected = 0

    def publish(self, scan: dict, *, now: float | None = None) -> None:
        mono = time.monotonic() if now is None else now
        with self.lock:
            self.sequence += 1
            # Scan dictionaries are not changed after publication.
            self.latest = {**scan, "sequence": self.sequence, "received_unix": time.time()}
            self.last_received = mono
            self.arrivals.append(mono)
            self.error = None
            self.ended = False

    def set_error(self, message: str, *, rejected: bool = False) -> None:
        with self.lock:
            self.error = str(message)[:500]
            self.rejected += int(rejected)

    def finish(self) -> None:
        with self.lock:
            self.ended = True

    def snapshot(self, *, now: float | None = None) -> dict:
        mono = time.monotonic() if now is None else now
        with self.lock:
            age = None if self.last_received is None else max(0, mono - self.last_received)
            samples = [v for v in self.arrivals if mono - v <= 3.0]
            hz = ((len(samples) - 1) / (samples[-1] - samples[0])
                  if len(samples) >= 2 and samples[-1] > samples[0] else 0.0)
            if self.error:
                state = "error"
            elif self.ended:
                state = "ended"
            elif age is None:
                state = "waiting"
            elif age > self.stale_seconds:
                state = "stale"
            else:
                state = {"ros": "live", "demo": "synthetic", "replay": "replay"}[self.source_mode]
            return {"schema": "smartcart.envelope.v1", "state": state,
                    "source_mode": self.source_mode, "server_unix": time.time(),
                    "age_ms": None if age is None else round(age * 1000, 1),
                    "receive_hz": round(hz, 2), "stale_after_ms": self.stale_seconds * 1000,
                    "rejected_scans": self.rejected, "error": self.error, "scan": self.latest}
