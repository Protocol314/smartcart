"""원시 프레임 재생과 동일한 상태 입력으로 좌우 위치 추정을 검증한다."""
from __future__ import annotations

from typing import Iterable, Iterator

from .replay import REPLAY_CLOCK_ORIGIN_NS, read_records, replay_records, validate_record

ESTIMATION_REPLAY_SCHEMA = "smartcart.uwb_estimation_replay_event.v1"


def estimate_records(records: Iterable[dict], collector, estimator, *,
                     clock_origin_ns: int = REPLAY_CLOCK_ORIGIN_NS) -> Iterator[dict]:
    """수집·추정 모두 재생 전용이다. 합성과 실측을 한 위치로 섞지 않는다.

    매 이벤트의 전체 snapshot을 전달하므로 disconnect/tick에서도 이전
    위치가 즉시 무효화된다. 각 원시 레코드와 결과를 하나씩 처리한다.
    """
    def checked_records():
        original_source = None
        for raw in records:
            record = validate_record(raw)
            if original_source is None:
                original_source = record["source"]
            elif original_source != record["source"]:
                raise ValueError("위치 재생에서 합성과 실측 출처를 혼합할 수 없습니다")
            yield record

    try:
        if (getattr(collector, "source_mode", None) != "replay"
                or getattr(estimator, "source_mode", None) != "replay"):
            raise ValueError("위치 재생에는 재생 전용 수집기와 추정기가 필요합니다")
        if getattr(collector, "config", None) != getattr(estimator, "config", None):
            raise ValueError("수집기와 추정기의 장착·태그 설정이 일치해야 합니다")
        for event in replay_records(checked_records(), collector, clock_origin_ns=clock_origin_ns):
            estimate = estimator.update(event["snapshot"], event["replay_now_ns"])
            yield {**event, "schema": ESTIMATION_REPLAY_SCHEMA, "estimate": estimate,
                   "drive_authorized": False}
    except (ValueError, OSError, RuntimeError):
        estimator.reset("replay_input_error")
        raise


def estimate_file(path, collector, estimator, **kwargs) -> Iterator[dict]:
    yield from estimate_records(read_records(path), collector, estimator, **kwargs)
