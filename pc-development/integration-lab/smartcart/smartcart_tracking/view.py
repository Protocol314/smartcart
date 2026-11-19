"""코어 시각을 브라우저용 남은 시간으로 바꾸는 제한된 최신 표시 상태."""
from copy import deepcopy
import json
import math
import re
from threading import Event, RLock
import time

from smartcart_uwb.config import CollectorConfig
from smartcart_uwb.observation import timestamp_ns
from .config import TrackingConfig
from .tracker import TargetTracker, TRACK_SCHEMA

VIEW_SCHEMA = "smartcart.tracking_view.v1"
MAX_VIEW_BYTES = 256 * 1024
_STATES = {"waiting", "acquiring", "tracking", "coasting", "ambiguous", "lost", "error"}
_TRACK_FIELDS = {"schema", "frame_id", "state", "valid", "invalid_reason", "track_id",
                 "position_xy_m", "velocity_xy_mps", "predicted_position_xy_m", "error_radius_m",
                 "reference_monotonic_ns", "valid_until_monotonic_ns", "confirmation_count",
                 "candidate_count", "track_sequence", "source_mode", "provisional", "drive_authorized",
                 "inputs", "associated_cluster_id", "association_distance_m", "association_gate_m"}


def _code(value, optional=False):
    if optional and value is None:
        return
    if type(value) is not str or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", value):
        raise ValueError("표시 상태 코드가 잘못되었습니다")


def _number(value, low=0, high=2000):
    if type(value) not in (int, float) or not low <= value <= high or not math.isfinite(value):
        raise ValueError("표시 수치가 잘못되었습니다")
    return float(value)


def _xy(value):
    if type(value) not in (tuple, list) or len(value) != 2:
        raise ValueError("평면 표시 좌표가 필요합니다")
    return [_number(v, -100, 100) for v in value]


class TrackingStore:
    """기본 기록·외부 송신 없이 최신 결과만 보관하고 만료된 마커를 제거한다."""
    def __init__(self, mounting=None, settings=None, *, mode="off", stop=None, heartbeat_timeout_ms=750):
        if mode not in ("off", "ros", "replay"):
            raise ValueError("지원하지 않는 추적 표시 모드입니다")
        if type(heartbeat_timeout_ms) is not int or not 100 <= heartbeat_timeout_ms <= 2000:
            raise ValueError("표시 수신 제한이 잘못되었습니다")
        if mode != "off" and (not isinstance(mounting, CollectorConfig) or not isinstance(settings, TrackingConfig)):
            raise ValueError("추적 표시에는 검증된 장착·추적 설정이 필요합니다")
        self.mode = mode
        self.source_mode = {"off": "off", "ros": "live", "replay": "replay"}[mode]
        self.config, self.settings = mounting, settings
        self.stop = Event() if stop is None else stop
        self._validator = None if mode == "off" else TargetTracker(mounting, settings, self.source_mode)
        self._lock = RLock()
        self._timeout_ns = heartbeat_timeout_ms * 1_000_000
        self._sequence = 0
        self._result = self._wall_ns = self._eval_ns = None
        self._rate = 1.0
        self._state = "off" if mode == "off" else "waiting"
        self._reason = None
        self._replay = None

    def _validate(self, result, now_ns):
        if (type(result) is not dict or not {"uwb_estimate", "lidar_clusters", "track"} <= result.keys()
                or set(result) - {"uwb_estimate", "lidar_clusters", "track", "source_mode",
                                  "drive_authorized", "uwb_observation_count"}
                or result.get("source_mode") != self.source_mode or result.get("drive_authorized") is not False):
            raise ValueError("추적 표시 출처·필드가 일치하지 않습니다")
        # 코어의 관측 검증을 재사용하되 추적 상태나 확인 횟수는 변경하지 않는다.
        lidar = self._validator._lidar(result["lidar_clusters"], now_ns)
        uwb = self._validator._uwb(result["uwb_estimate"], now_ns)
        track = result["track"]
        if (type(track) is not dict or set(track) != _TRACK_FIELDS or track.get("schema") != TRACK_SCHEMA
                or track.get("source_mode") != self.source_mode or track.get("frame_id") != "base_link"
                or track.get("drive_authorized") is not False or track.get("provisional") is not True
                or type(track.get("valid")) is not bool or track.get("state") not in _STATES):
            raise ValueError("유효한 읽기 전용 추적 결과가 필요합니다")
        _code(track.get("invalid_reason"), optional=track["valid"])
        for name in ("track_sequence", "confirmation_count", "candidate_count"):
            value = track.get(name)
            if type(value) is not int or not 0 <= value <= (1 << 53) - 1:
                raise ValueError("추적 표시 순번이 잘못되었습니다")
        if track.get("track_id") is not None and (type(track["track_id"]) is not int or not 1 <= track["track_id"] < (1 << 53)):
            raise ValueError("추적 표시 식별자가 잘못되었습니다")
        if track.get("reference_monotonic_ns") is not None:
            timestamp_ns(track["reference_monotonic_ns"])
            if track["reference_monotonic_ns"] > now_ns:
                raise ValueError("추적 표시 시각이 미래입니다")
        timestamp_ns(track.get("valid_until_monotonic_ns"))
        if track["valid"]:
            if (track["state"] != "tracking" or track["invalid_reason"] is not None
                    or not track["reference_monotonic_ns"] < track["valid_until_monotonic_ns"]):
                raise ValueError("추적 표시 수명이 잘못되었습니다")
            _xy(track["position_xy_m"])
            _xy(track["velocity_xy_mps"])
            _number(track["error_radius_m"])
        elif (track.get("position_xy_m") is not None or track.get("velocity_xy_mps") is not None
              or track["valid_until_monotonic_ns"] != 0):
            raise ValueError("무효 추적에 측정 위치가 남아 있습니다")
        if track.get("predicted_position_xy_m") is not None:
            _xy(track["predicted_position_xy_m"])
        inputs = track["inputs"]
        if type(inputs) is not dict or set(inputs) - {"lidar", "uwb"}:
            raise ValueError("추적 입력 요약이 잘못되었습니다")
        for item in inputs.values():
            if type(item) is not dict or set(item) != {"adapter_session", "sequence", "reference_monotonic_ns"}:
                raise ValueError("추적 입력 요약 필드가 잘못되었습니다")
            _code(item["adapter_session"], optional=True)
            timestamp_ns(item["sequence"])
            if item["reference_monotonic_ns"] is not None:
                timestamp_ns(item["reference_monotonic_ns"])
        if track["valid"]:
            identifier = track["associated_cluster_id"]
            if type(identifier) is not int or not 0 <= identifier < self.settings.max_clusters:
                raise ValueError("선택 군집 식별자가 잘못되었습니다")
            distance = _number(track["association_distance_m"], 0, 100)
            gate = _number(track["association_gate_m"], 0, self.settings.max_association_radius_m)
            if distance > gate + 1e-8:
                raise ValueError("선택 군집이 연관 범위를 벗어났습니다")
            # 각 입력의 형식뿐 아니라 선택 결과와 입력 사이의 계약도 검사한다.
            selected = next((item for item in lidar.value["clusters"]
                             if item["cluster_id"] == identifier), None)
            expected_inputs = {name: {"adapter_session": seen.context[0], "sequence": seen.sequence,
                                     "reference_monotonic_ns": seen.rx_ns}
                               for name, seen in (("lidar", lidar), ("uwb", uwb))}
            if (not lidar.value["valid"] or not uwb.value["valid"] or selected is None
                    or not now_ns < track["valid_until_monotonic_ns"] <= min(
                        lidar.value["valid_until_monotonic_ns"], uwb.value["valid_until_monotonic_ns"])
                    or track["reference_monotonic_ns"] != lidar.rx_ns
                    or inputs != expected_inputs or track["track_id"] is None
                    or track["candidate_count"] != 1
                    or track["confirmation_count"] < self.settings.confirmation_hits
                    or track["predicted_position_xy_m"] is not None
                    or list(track["position_xy_m"]) != list(selected["centroid_xy_m"])):
                raise ValueError("선택 대상과 입력 관측이 일치하지 않습니다")
            actual_distance = math.dist(selected["centroid_xy_m"], uwb.value["position_xy_m"])
            if abs(distance - actual_distance) > 1e-8:
                raise ValueError("선택 대상의 연관 거리가 일치하지 않습니다")
        elif any(track[key] is not None for key in ("associated_cluster_id", "association_distance_m", "association_gate_m")):
            raise ValueError("무효 추적에 선택 군집이 남아 있습니다")
        encoded = json.dumps(result, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        if len(encoded.encode("utf-8")) > MAX_VIEW_BYTES:
            raise ValueError("추적 표시 결과 크기 상한을 넘었습니다")
        return deepcopy({name: result[name] for name in ("uwb_estimate", "lidar_clusters", "track")})

    def begin(self, *, replay=None):
        """재생 반복도 새 구간으로 시작하며 표시 순번은 계속 증가한다."""
        with self._lock:
            self._result = self._wall_ns = self._eval_ns = None
            self._state = "off" if self.mode == "off" else "waiting"
            self._reason = None
            self._replay = deepcopy(replay)
            self._sequence += 1

    def publish(self, result, *, now_ns, clock_rate=1.0, wall_ns=None, replay=None):
        with self._lock:
            try:
                if self.mode == "off":
                    raise ValueError("꺼진 추적 표시에는 입력을 넣을 수 없습니다")
                timestamp_ns(now_ns)
                wall_ns = time.monotonic_ns() if wall_ns is None else timestamp_ns(wall_ns)
                rate = _number(clock_rate, .05, 4)
                if self.mode == "ros" and rate != 1:
                    raise ValueError("실측 시계의 속도를 바꿀 수 없습니다")
                if ((self._eval_ns is not None and now_ns < self._eval_ns)
                        or (self._wall_ns is not None and wall_ns < self._wall_ns)):
                    raise ValueError("표시 시계가 역행했습니다")
                value = self._validate(result, now_ns)
                self._result, self._eval_ns, self._wall_ns = value, now_ns, wall_ns
                self._rate, self._state, self._reason = rate, "running", None
                if replay is not None:
                    self._replay = deepcopy(replay)
                self._sequence += 1
            except Exception:
                self.set_error("invalid_tracking_result")
                raise

    def set_error(self, reason="tracking_error"):
        with self._lock:
            _code(reason)
            self._state, self._reason, self._result = "error", reason, None
            self._sequence += 1

    def finish(self):
        with self._lock:
            self._state, self._reason = "ended", "replay_ended"
            self._sequence += 1

    def snapshot(self, now=None, now_ns=None):
        with self._lock:
            if now is not None and now_ns is not None:
                raise ValueError("표시 시각은 한 단위로만 지정합니다")
            wall = (int(_number(now, 0, 1e10) * 1e9) if now is not None else
                    time.monotonic_ns() if now_ns is None else timestamp_ns(now_ns))
            elapsed = max(0, wall - self._wall_ns) if self._wall_ns is not None else 0
            if self._wall_ns is not None and wall < self._wall_ns:
                self.set_error("non_monotonic_view_time")
            state = self._state
            if state == "running" and elapsed >= self._timeout_ns:
                # 통신이 멎은 과거 결과는 같은 벽시각을 다시 넣어도 재활성화하지 않는다.
                self._state, self._reason = "stale", "tracking_transport_stale"
                state = "stale"
            evaluation = (self._eval_ns or 0) + int(elapsed * self._rate)
            active = state == "running"
            value = deepcopy(self._result)
            transport_ms = max(0, (self._timeout_ns - elapsed) / 1e6) if active else 0

            def remaining(deadline):
                if not active or type(deadline) is not int:
                    return 0.0
                return min(transport_ms, max(0, (deadline - evaluation) / self._rate / 1e6))

            track_ms = cluster_ms = prediction_ms = uwb_ms = 0.0
            if value is not None:
                track, clusters, uwb = (value[name] for name in ("track", "lidar_clusters", "uwb_estimate"))
                track_ms = remaining(track["valid_until_monotonic_ns"]) if track["valid"] else 0
                cluster_ms = remaining(clusters["valid_until_monotonic_ns"]) if clusters["valid"] else 0
                uwb_ms = remaining(uwb["valid_until_monotonic_ns"]) if uwb["valid"] else 0
                reference = track["reference_monotonic_ns"]
                if track["predicted_position_xy_m"] is not None and reference is not None:
                    prediction_ms = min(cluster_ms, uwb_ms, remaining(
                        reference + self.settings.occlusion_timeout_ms * 1_000_000))
                if not track_ms:
                    track.update(valid=False, position_xy_m=None, velocity_xy_mps=None, error_radius_m=None,
                                 valid_until_monotonic_ns=0, associated_cluster_id=None,
                                 association_distance_m=None, association_gate_m=None)
                    if track["state"] == "tracking":
                        track.update(state="lost", invalid_reason="display_stale", track_id=None)
                if not prediction_ms:
                    track["predicted_position_xy_m"] = None
                if track_ms:
                    track["error_radius_m"] += self.settings.max_relative_speed_mps * elapsed * self._rate / 1e9
                if not active:
                    track.update(state="lost", invalid_reason=self._reason or "display_stale", track_id=None)
                if not cluster_ms:
                    clusters.update(valid=False, invalid_reason="display_stale", clusters=[], valid_until_monotonic_ns=0)
                if not uwb_ms:
                    uwb_reason = 'display_stale' if uwb['valid'] else uwb['invalid_reason']
                    for name in ("position_xy_m", "tag_height_m", "error_radius_m", "error_at_reference_m",
                                 "estimate_age_ms", "pair_residual_m", "rx_skew_ns", "reference_monotonic_ns", "alignment_method"):
                        uwb[name] = None
                    uwb.update(valid=False, invalid_reason=uwb_reason, valid_until_monotonic_ns=0)
            transform = None if self.config is None else {
                "yaw_deg": self.config.lidar.yaw_deg, "translation_m": list(self.config.lidar.translation_m)}
            return {"schema": VIEW_SCHEMA, "mode": self.mode, "source_mode": self.source_mode,
                    "state": state, "sequence": self._sequence, "age_ms": None if self._wall_ns is None else elapsed / 1e6,
                    "remaining_ms": track_ms, "cluster_remaining_ms": cluster_ms,
                    "prediction_remaining_ms": prediction_ms, "uwb_remaining_ms": uwb_ms,
                    "valid": bool(value is not None and value["track"]["valid"]),
                    "frame_id": "base_link", "drive_authorized": False, "provisional": True,
                    "result": value, "lidar_transform": transform, "replay": deepcopy(self._replay),
                    "error": self._reason}
