"""명시적으로 지정한 원시 캡처를 실제 시간 간격대로 평가하는 읽기 전용 재생기.

벽시계나 재생 속도로 입력 시각을 바꾸지 않는다. 출력은 항상 재생 자료이며
구동 허가가 아니다. 파일 전체와 전체 결과를 메모리에 모으지 않는다.
"""
from __future__ import annotations

import json
import stat
from pathlib import Path
from typing import Iterable, Iterator

CAPTURE_SCHEMA = "smartcart.uwb_capture.v1"
REPLAY_SCHEMA = "smartcart.uwb_replay_event.v1"
MAX_LINE_BYTES = 16 * 1024
MAX_FILE_BYTES = 100 * 1024 * 1024
MAX_RECORDS = 100_000
MAX_CHUNK_BYTES = 1024
# 0을 미수신 시각으로 쓰는 구현과도 충돌하지 않는 명시적인 재생 시계 원점이다.
REPLAY_CLOCK_ORIGIN_NS = 1_000_000_000_000
MAX_AT_NS = (1 << 63) - 1 - REPLAY_CLOCK_ORIGIN_NS
SENSOR_IDS = frozenset(("uwb_left", "uwb_right"))
EVENTS = frozenset(("bytes", "disconnect", "reconnect", "tick"))
_COMMON_KEYS = frozenset(("schema", "sensor_id", "event", "at_ns", "source"))
_HEX = frozenset("0123456789abcdefABCDEF")


class ReplayError(ValueError):
    """입력 내용을 그대로 노출하지 않는 캡처 검증 오류."""


def _integer(value, name: str, maximum: int = MAX_AT_NS) -> int:
    if type(value) is not int or not 0 <= value <= maximum:
        raise ReplayError(f"{name}: 허용 범위의 음이 아닌 정수가 필요합니다")
    return value


def validate_record(record: dict) -> dict:
    """미지 필드·암묵적 형변환·실행 가능한 경로를 받지 않는 고정 스키마."""
    if not isinstance(record, dict):
        raise ReplayError("레코드는 JSON 객체여야 합니다")
    event = record.get("event")
    if not isinstance(event, str) or event not in EVENTS:
        raise ReplayError("event: 지원하지 않는 이벤트입니다")
    required = {"schema", "event", "at_ns", "source"}
    allowed = _COMMON_KEYS | ({"data_hex"} if event == "bytes" else set())
    if event != "tick":
        required.add("sensor_id")
    if event == "bytes":
        required.add("data_hex")
    if not required <= record.keys() or not record.keys() <= allowed:
        raise ReplayError("필수 필드가 없거나 허용하지 않는 필드가 있습니다")
    if record["schema"] != CAPTURE_SCHEMA:
        raise ReplayError("schema: 캡처 v1만 지원합니다")
    if record["source"] not in ("synthetic", "recorded"):
        raise ReplayError("source: synthetic 또는 recorded만 지원합니다")
    _integer(record["at_ns"], "at_ns")
    if "sensor_id" in record:
        sensor_id = record["sensor_id"]
        if not isinstance(sensor_id, str) or sensor_id not in SENSOR_IDS:
            raise ReplayError("sensor_id: 좌측 또는 우측 센서만 지원합니다")
    if event == "bytes":
        data = record["data_hex"]
        if (not isinstance(data, str) or not 2 <= len(data) <= MAX_CHUNK_BYTES * 2
                or len(data) % 2 or any(char not in _HEX for char in data)):
            raise ReplayError("data_hex: 1~1024바이트의 연속된 16진수 문자열이 필요합니다")
    return dict(record)


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ReplayError("중복 JSON 필드는 허용하지 않습니다")
        result[key] = value
    return result


def _constant(_value):
    raise ReplayError("비유한 JSON 숫자는 허용하지 않습니다")


def read_records(path: str | Path) -> Iterator[dict]:
    """일반 파일만 열고 줄·전체 바이트·레코드 수를 스트리밍 중에도 제한한다."""
    path = Path(path)
    if not stat.S_ISREG(path.stat().st_mode):
        raise ReplayError("캡처 입력은 일반 파일이어야 합니다")
    with path.open("rb") as handle:
        import os
        info = os.fstat(handle.fileno())
        if not stat.S_ISREG(info.st_mode):
            raise ReplayError("캡처 입력은 일반 파일이어야 합니다")
        if info.st_size > MAX_FILE_BYTES:
            raise ReplayError("캡처 파일은 100 MiB 이하여야 합니다")
        total = 0
        count = 0
        while True:
            line = handle.readline(MAX_LINE_BYTES + 1)
            if not line:
                break
            total += len(line)
            if len(line) > MAX_LINE_BYTES or total > MAX_FILE_BYTES:
                raise ReplayError("캡처 줄 또는 파일 크기 상한을 초과했습니다")
            if not line.strip():
                raise ReplayError("빈 캡처 줄은 허용하지 않습니다")
            count += 1
            if count > MAX_RECORDS:
                raise ReplayError("캡처 레코드는 100000개 이하여야 합니다")
            try:
                record = json.loads(line.decode("utf-8"), object_pairs_hook=_object,
                                    parse_constant=_constant)
                yield validate_record(record)
            except (UnicodeError, json.JSONDecodeError, RecursionError) as exc:
                raise ReplayError(f"레코드 {count}: 올바른 UTF-8 JSON 객체가 아닙니다") from exc
        if not count:
            raise ReplayError("캡처 파일에 레코드가 없습니다")


def replay_records(records: Iterable[dict], collector, *,
                   clock_origin_ns: int = REPLAY_CLOCK_ORIGIN_NS) -> Iterator[dict]:
    """각 입력 이벤트의 관측·상태를 반환한다. 과거 간격을 압축하지 않는다.

    source_rx_ns는 원본 캡처의 at_ns이다. 이것은 BU04 하드웨어 측정 시각이
    아니며, now_ns는 고정 원점을 더한 가상 수신 시각이다. ROS 발행은 없다.
    """
    _integer(clock_origin_ns, "clock_origin_ns", REPLAY_CLOCK_ORIGIN_NS)
    if not clock_origin_ns:
        raise ReplayError("재생 시계 원점은 0보다 커야 합니다")
    if getattr(collector, "source_mode", None) != "replay":
        raise ReplayError("재생 전용 collector가 필요합니다")
    previous = -1
    count = 0
    for count, raw in enumerate(records, 1):
        if count > MAX_RECORDS:
            raise ReplayError("캡처 레코드는 100000개 이하여야 합니다")
        record = validate_record(raw)
        at_ns = record["at_ns"]
        if at_ns < previous:
            raise ReplayError(f"레코드 {count}: 전역 at_ns가 역행했습니다")
        previous = at_ns
        now_ns = clock_origin_ns + at_ns
        event = record["event"]
        observations = []
        if event == "bytes":
            observations = collector.feed(record["sensor_id"], bytes.fromhex(record["data_hex"]),
                                          now_ns=now_ns, source_rx_ns=at_ns)
        elif event == "disconnect":
            collector.disconnected(record["sensor_id"], reason="capture_disconnect")
        elif event == "reconnect":
            collector.reconnect(record["sensor_id"])
        snapshot = collector.snapshot(now_ns)
        # 재생 출력을 다른 경로가 실측이나 구동 허가로 오인하지 못하게 고정한다.
        snapshot = {**snapshot, "source_mode": "replay", "drive_authorized": False}
        observations = [{**item, "source_mode": "replay", "drive_authorized": False}
                        for item in observations]
        yield {"schema": REPLAY_SCHEMA, "event_index": count - 1, "at_ns": at_ns,
               "replay_now_ns": now_ns, "clock_origin_ns": clock_origin_ns,
               "source_mode": "replay", "original_source": record["source"],
               "drive_authorized": False, "observations": observations, "snapshot": snapshot}
    if not count:
        raise ReplayError("재생 레코드가 없습니다")


def replay_file(path: str | Path, collector, **kwargs) -> Iterator[dict]:
    """파일 입력과 결과를 모두 지연 평가하여 전체 결과 누적을 피한다."""
    yield from replay_records(read_records(path), collector, **kwargs)
