"""원시 UWB·LiDAR 이벤트를 순서와 간격을 보존해 재생하는 제한된 입력 계약."""
from __future__ import annotations
import json
import math
import os
from pathlib import Path
import stat

from smartcart_uwb.replay import MAX_FILE_BYTES, MAX_RECORDS, REPLAY_CLOCK_ORIGIN_NS, MAX_AT_NS
from smartcart_uwb.config import _unique_object, _reject_constant
from .observation import SCAN_FIELDS

CAPTURE_SCHEMA = "smartcart.target_capture.v1"
REPLAY_SCHEMA = "smartcart.target_replay_event.v1"
MAX_LINE_BYTES = 512 * 1024
MAX_RAYS = 8192
EVENTS = {"uwb_bytes", "uwb_disconnect", "uwb_reconnect", "lidar_scan",
          "lidar_disconnect", "lidar_reconnect", "tick"}


def _integer(value, maximum=MAX_AT_NS):
    if type(value) is not int or not 0 <= value <= maximum:
        raise ValueError("재생 시각은 허용 범위의 음수가 아닌 정수여야 합니다")
    return value


def validate_record(record):
    if type(record) is not dict:
        raise ValueError("추적 캡처 객체가 필요합니다")
    event = record.get("event")
    if not isinstance(event, str) or event not in EVENTS:
        raise ValueError("지원하지 않는 추적 이벤트입니다")
    fields = {"schema", "event", "at_ns", "source"}
    if event.startswith("uwb_"):
        fields.add("sensor_id")
    if event == "uwb_bytes":
        fields.add("data_hex")
    if event == "lidar_scan":
        fields.add("scan")
    if set(record) != fields or record["schema"] != CAPTURE_SCHEMA:
        raise ValueError("추적 캡처 필드·스키마가 잘못되었습니다")
    _integer(record["at_ns"])
    if record["source"] not in ("synthetic", "recorded"):
        raise ValueError("합성 또는 기록 출처만 허용합니다")
    if event.startswith("uwb_") and record["sensor_id"] not in ("uwb_left", "uwb_right"):
        raise ValueError("좌우 UWB 센서 식별자가 필요합니다")
    if event == "uwb_bytes":
        value = record["data_hex"]
        if (type(value) is not str or not 2 <= len(value) <= 2048 or len(value) % 2
                or any(char not in "0123456789abcdefABCDEF" for char in value)):
            raise ValueError("1~1024바이트의 연속된 16진수 자료가 필요합니다")
    if event == "lidar_scan":
        scan = record["scan"]
        if type(scan) is not dict or set(scan) != SCAN_FIELDS:
            raise ValueError("LiDAR 원시 스캔 필드가 잘못되었습니다")
        ranges = scan["ranges"]
        if type(ranges) not in (list, tuple) or not 1 <= len(ranges) <= MAX_RAYS:
            raise ValueError("LiDAR 광선 수 상한을 넘었습니다")
        for value in (scan[key] for key in SCAN_FIELDS - {"ranges"}):
            if type(value) not in (int, float) or not -100000 <= value <= 100000 or not math.isfinite(value):
                raise ValueError("LiDAR 수치가 허용 범위를 벗어났습니다")
        for value in ranges:
            if value is not None and (type(value) not in (int, float)
                    or not -100000 <= value <= 100000 or not math.isfinite(value)):
                raise ValueError("LiDAR 반사 거리의 형식·범위가 잘못되었습니다")
    return dict(record)


def read_records(path):
    path = Path(path)
    if not stat.S_ISREG(path.stat().st_mode):
        raise ValueError("추적 입력은 일반 파일이어야 합니다")
    with path.open("rb") as handle:
        info = os.fstat(handle.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size > MAX_FILE_BYTES:
            raise ValueError("추적 입력 파일 종류·크기 상한이 잘못되었습니다")
        total = count = 0
        while True:
            line = handle.readline(MAX_LINE_BYTES + 1)
            if not line:
                break
            count += 1
            total += len(line)
            if len(line) > MAX_LINE_BYTES or total > MAX_FILE_BYTES or count > MAX_RECORDS:
                raise ValueError("추적 캡처 줄·파일·레코드 상한을 넘었습니다")
            try:
                payload = json.loads(line.decode("utf-8"), object_pairs_hook=_unique_object,
                                     parse_constant=_reject_constant)
            except (UnicodeError, json.JSONDecodeError, RecursionError) as exc:
                raise ValueError("올바른 UTF-8 추적 캡처가 필요합니다") from exc
            yield validate_record(payload)
        if not count:
            raise ValueError("추적 캡처가 비어 있습니다")


def track_records(records, pipeline, *, clock_origin_ns=REPLAY_CLOCK_ORIGIN_NS):
    try:
        _integer(clock_origin_ns, REPLAY_CLOCK_ORIGIN_NS)
        if not clock_origin_ns or getattr(pipeline, "source_mode", None) != "replay":
            raise ValueError("재생 전용 파이프라인과 양의 시계 원점이 필요합니다")
        previous = -1
        count = 0
        original_source = None
        for count, raw in enumerate(records, 1):
            if count > MAX_RECORDS:
                raise ValueError("추적 레코드 수 상한을 넘었습니다")
            record = validate_record(raw)
            if record["at_ns"] < previous:
                raise ValueError("추적 캡처 시각이 역행했습니다")
            if original_source is not None and record["source"] != original_source:
                raise ValueError("합성과 실측 출처를 혼합할 수 없습니다")
            original_source = record["source"]
            previous = record["at_ns"]
            now_ns = clock_origin_ns + record["at_ns"]
            result = pipeline.process(record, now_ns)
            yield {**result, "schema": REPLAY_SCHEMA, "event_index": count - 1, "at_ns": record["at_ns"],
                   "replay_now_ns": now_ns, "clock_origin_ns": clock_origin_ns,
                   "source_mode": "replay", "original_source": original_source,
                   "drive_authorized": False}
        if not count:
            raise ValueError("추적 재생 이벤트가 없습니다")
    except (ValueError, TypeError, OSError, RuntimeError):
        pipeline.reset("replay_input_error")
        raise


def track_file(path, pipeline, **kwargs):
    yield from track_records(read_records(path), pipeline, **kwargs)
