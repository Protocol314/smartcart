"""센서별 파서와 순번을 격리하며 불량 입력은 이전 관측을 즉시 무효화한다."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import re
from threading import RLock
from typing import Callable
from uuid import uuid4

from .config import CollectorConfig, SENSOR_IDS, validate_distinct_ports
from .observation import MAX_MONOTONIC_NS, make_observation, timestamp_ns, validate_frame

MAX_FEED_BYTES = 65536
STATUS_SCHEMA = "smartcart.dual_uwb_status.v1"


@dataclass
class _SensorState:
    parser: object
    connected: bool = True
    stream_epoch: int = 0
    sample_sequence: int = 0
    device_sequence: int | None = None
    last_rx_ns: int | None = None
    last_valid_rx_ns: int | None = None
    latest: dict | None = None
    last_error: str | None = None


class DualCollector:
    """위치 융합 없이 최신 관측만 유지한다. 재연결에서만 스트림 epoch가 증가한다."""

    def __init__(self, config: CollectorConfig, parser_factory: Callable | None = None,
                 source_mode: str = "live", session_id: str | None = None):
        if not isinstance(config, CollectorConfig) or set(config.sensors) != set(SENSOR_IDS):
            raise ValueError("검증된 좌우 수집기 설정이 필요합니다")
        validate_distinct_ports(config)
        if source_mode not in ("live", "replay"):
            raise ValueError("source_mode는 live 또는 replay여야 합니다")
        session_id = str(uuid4()) if session_id is None else session_id
        if not isinstance(session_id, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", session_id):
            raise ValueError("세션 식별자의 형식 또는 길이가 잘못되었습니다")
        if parser_factory is None:
            from .native import NativeParser
            parser_factory = NativeParser
        self.config = config
        self.source_mode = source_mode
        self.adapter_session = session_id
        self._lock = RLock()
        self._closed = False
        self._states: dict[str, _SensorState] = {}
        try:
            for sensor_id in SENSOR_IDS:
                self._states[sensor_id] = _SensorState(parser_factory())
        except Exception:
            for state in self._states.values():
                state.parser.close()
            raise

    def __enter__(self):
        self._require_open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("수집기가 닫혔습니다")

    def _state(self, sensor_id: str) -> _SensorState:
        self._require_open()
        if sensor_id not in self._states:
            raise ValueError("알 수 없는 센서 식별자입니다")
        return self._states[sensor_id]

    @staticmethod
    def _invalidate(state: _SensorState, reason: str) -> None:
        state.last_error = reason
        if state.latest is not None:
            state.latest = {**state.latest, "valid": False, "invalid_reason": reason,
                            "drive_authorized": False}

    def _observation(self, sensor_id: str, state: _SensorState, frame: dict | None,
                     now_ns: int, source_rx_ns: int | None, reason: str | None = None) -> dict:
        state.sample_sequence += 1
        # 무효 입력은 마지막 정상 관측의 만료 시각을 연장하지 않는다.
        deadline = (state.last_valid_rx_ns + self.config.timeout_ms * 1_000_000
                    if state.last_valid_rx_ns is not None else 0)
        observation = make_observation(
            sensor=self.config.sensors[sensor_id], frame=frame,
            allowed_tag_address=self.config.allowed_tag_address,
            logical_target_id=self.config.logical_target_id,
            sample_sequence=state.sample_sequence, stream_epoch=state.stream_epoch,
            adapter_session=self.adapter_session, source_mode=self.source_mode,
            rx_ns=now_ns, source_rx_ns=source_rx_ns, valid_until_ns=deadline,
            invalid_reason=reason,
        )
        if observation["valid"]:
            state.last_valid_rx_ns = now_ns
            observation["valid_until_monotonic_ns"] = now_ns + self.config.timeout_ms * 1_000_000
        state.latest = observation
        state.last_error = observation["invalid_reason"]
        return observation

    def feed(self, sensor_id: str, data: bytes, now_ns: int,
             source_rx_ns: int | None = None) -> list[dict]:
        with self._lock:
            state = self._state(sensor_id)
            try:
                timestamp_ns(now_ns)
                if now_ns > MAX_MONOTONIC_NS - self.config.timeout_ms * 1_000_000:
                    raise ValueError("수신 시각에 만료 기간을 더할 수 없습니다")
                if source_rx_ns is not None:
                    timestamp_ns(source_rx_ns, "source_rx_ns")
                if not isinstance(data, bytes):
                    raise TypeError("수신 데이터는 bytes여야 합니다")
                if len(data) > MAX_FEED_BYTES:
                    raise ValueError("한 번에 처리할 수신 데이터 크기를 넘었습니다")
            except (TypeError, ValueError):
                self._invalidate(state, "invalid_input")
                state.parser.reset()
                raise
            if state.last_rx_ns is not None and now_ns < state.last_rx_ns:
                state.parser.reset()
                observation = self._observation(sensor_id, state, None, state.last_rx_ns,
                                                source_rx_ns, "non_monotonic_time")
                return [deepcopy(observation)]
            state.last_rx_ns = now_ns
            if not state.connected:
                return [deepcopy(self._observation(sensor_id, state, None, now_ns,
                                                  source_rx_ns, "disconnected"))]
            try:
                events = state.parser.feed_events(data)
            except Exception:
                self._invalidate(state, "parser_error")
                raise
            observations = []
            for event in events:
                if not isinstance(event, dict) or event.get("kind") != "frame":
                    reason = "bad_frame"
                    if isinstance(event, dict) and event.get("kind") == "bad_frame":
                        code = event.get("reason")
                        if code in ("argument", "size", "header", "length", "footer", "checksum", "unknown"):
                            reason = "bad_frame:" + code
                    observations.append(self._observation(sensor_id, state, None, now_ns, source_rx_ns, reason))
                    continue
                try:
                    frame = validate_frame(event.get("frame"))
                except ValueError:
                    observations.append(self._observation(sensor_id, state, None, now_ns,
                                                          source_rx_ns, "invalid_frame"))
                    continue
                reason = None
                eligible = (frame["tag_address"] == self.config.allowed_tag_address
                            and 5 <= frame["distance_cm"] <= 2000
                            and -180 <= frame["angle_deg"] <= 180)
                if eligible and state.device_sequence is not None:
                    delta = (frame["sequence"] - state.device_sequence) & 255
                    if delta == 0:
                        reason = "duplicate_sequence"
                    elif delta >= 128:
                        reason = "out_of_order_sequence"
                if eligible and reason is None:
                    state.device_sequence = frame["sequence"]
                observations.append(self._observation(sensor_id, state, frame, now_ns, source_rx_ns, reason))
            return deepcopy(observations)

    def disconnected(self, sensor_id: str, reason: str = "disconnected") -> None:
        with self._lock:
            state = self._state(sensor_id)
            if not isinstance(reason, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", reason):
                raise ValueError("연결 중단 원인의 형식 또는 길이가 잘못되었습니다")
            state.connected = False
            self._invalidate(state, reason)
            state.parser.reset()

    def invalidate(self, sensor_id: str, reason: str) -> None:
        """발행 경로 오류도 즉시 무효화하되 다음 새 관측의 수신은 허용한다."""
        with self._lock:
            state = self._state(sensor_id)
            if not isinstance(reason, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", reason):
                raise ValueError("무효화 원인의 형식 또는 길이가 잘못되었습니다")
            self._invalidate(state, reason)

    def reconnect(self, sensor_id: str) -> None:
        with self._lock:
            state = self._state(sensor_id)
            state.parser.reset()
            state.connected = True
            state.stream_epoch += 1
            state.device_sequence = None
            state.last_valid_rx_ns = None
            state.latest = None
            state.last_error = None
            # 8비트 순번만으로 장치 재부팅을 판별하지 않는다. 명시적 재연결만 기준으로 삼는다.

    def snapshot(self, now_ns: int) -> dict:
        timestamp_ns(now_ns)
        with self._lock:
            self._require_open()
            sensors = {}
            for sensor_id, state in self._states.items():
                latest = deepcopy(state.latest)
                error = state.last_error
                if not state.connected:
                    status = "error"
                elif latest is None:
                    status = "error" if error else "waiting"
                elif not latest["valid"]:
                    status = "stale" if latest["invalid_reason"] == "stale" else "error"
                elif now_ns < latest["rx_monotonic_ns"]:
                    status = "error"
                    latest.update(valid=False, invalid_reason="non_monotonic_time")
                    state.latest = deepcopy(latest)
                    state.last_error = "non_monotonic_time"
                elif now_ns >= latest["valid_until_monotonic_ns"]:
                    status = "stale"
                    latest.update(valid=False, invalid_reason="stale")
                    state.latest = deepcopy(latest)
                    state.last_error = "stale"
                else:
                    status = "live"
                sensors[sensor_id] = {
                    "state": status, "connected": state.connected,
                    "stream_epoch": state.stream_epoch, "latest": latest,
                    "last_error": error or (latest["invalid_reason"] if latest is not None else None),
                }
            return {"schema": STATUS_SCHEMA, "source_mode": self.source_mode,
                    "adapter_session": self.adapter_session, "drive_authorized": False,
                    "sensors": sensors}

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            for state in self._states.values():
                self._invalidate(state, "closed")
                state.parser.close()
