"""JSON/JSONL and a strict subset of ROS echo block-YAML. No YAML dependency."""
from __future__ import annotations
import json
import math
from pathlib import Path
from typing import Iterator
from .core import normalise_scan

MAX_FILE_BYTES = 100 * 1024 * 1024


def _number(text: str):
    value = text.strip().strip("'\"")
    if value == "...":
        return "..."
    if value.lower() in (".inf", "+.inf", "-.inf", ".nan", "inf", "-inf", "nan", "null", "~"):
        return None
    try:
        num = float(value)
    except ValueError as exc:
        raise ValueError(f"Unsupported scalar in echo file: {text!r}") from exc
    return num if math.isfinite(num) else None


def parse_echo(text: str) -> dict:
    """Parse one default block-style `ros2 topic echo ... --once` document.

    Not a general YAML parser. Flow-style YAML, tags, aliases and arbitrary
    nested objects are intentionally unsupported. Data is never evaluated.
    """
    result = {"header": {"stamp": {}}, "ranges": [], "intensities": []}
    section = None
    seen_doc = False
    scalar_fields = {"angle_min", "angle_max", "angle_increment", "time_increment", "scan_time", "range_min", "range_max"}
    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        if s == "---":
            if seen_doc:
                break
            continue
        if s == "...":
            break
        seen_doc = True
        if s.startswith("- "):
            if section not in ("ranges", "intensities"):
                raise ValueError("Unexpected sequence outside ranges / intensities")
            result[section].append(_number(s[2:]))
            continue
        if ":" not in s:
            raise ValueError("Use a default block-style ROS echo file, not terminal prompts")
        key, value = (part.strip() for part in s.split(":", 1))
        if key in ("ranges", "intensities"):
            section = key
            if value == "[]":
                result[key] = []
            elif value:
                raise ValueError("Flow-style arrays are unsupported; omit --flow-style")
        else:
            section = None
            if key in scalar_fields:
                result[key] = _number(value)
            elif key in ("sec", "nanosec"):
                result["header"]["stamp"][key] = _number(value)
            elif key == "frame_id":
                result["header"]["frame_id"] = value.strip("'\"")
            elif key not in ("header", "stamp"):
                raise ValueError(f"Unexpected echo field: {key}")
    return result


def read_frames(path: str | Path) -> Iterator[dict]:
    file = Path(path)
    if file.stat().st_size > MAX_FILE_BYTES:
        raise ValueError("Replay file exceeds 100 MiB; split the recording first")
    suffix = file.suffix.lower()
    if suffix in (".jsonl", ".ndjson"):
        with file.open(encoding="utf-8-sig") as handle:
            for line_number, line in enumerate(handle, 1):
                if not line.strip():
                    continue
                try:
                    data = json.loads(line)
                    yield normalise_scan(data, kind="replay", label=file.name)
                except (ValueError, TypeError) as exc:
                    raise ValueError(f"{file.name}:{line_number}: {exc}") from exc
    else:
        text = file.read_text(encoding="utf-8-sig")
        data = json.loads(text) if text.lstrip().startswith("{") else parse_echo(text)
        yield normalise_scan(data, kind="replay", label=file.name)


def write_scan(handle, scan: dict) -> None:
    handle.write(json.dumps(scan, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n")
