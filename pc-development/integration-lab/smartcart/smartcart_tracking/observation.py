"""LiDAR 스캔의 같은 호스트 시각·세션·수명을 관리하는 장치 없는 수집 상태."""
from copy import deepcopy
import math
import re
from threading import RLock
from uuid import uuid4

from smartcart_uwb.config import CollectorConfig
from smartcart_uwb.observation import MAX_MONOTONIC_NS, timestamp_ns
from .config import TrackingConfig

LIDAR_OBSERVATION_SCHEMA = "smartcart.lidar_observation.v1"
SCAN_FIELDS = {"angle_min", "angle_max", "angle_increment", "range_min", "range_max",
               "ranges", "scan_time", "time_increment"}


class LidarCollector:
    """최신 스캔 한 개만 유지한다. 실물 장치·ROS·파일을 직접 열지 않는다."""
    def __init__(self, mounting: CollectorConfig, settings: TrackingConfig, *,
                 source_mode="replay", session_id=None):
        if not isinstance(mounting, CollectorConfig) or not isinstance(settings, TrackingConfig):
            raise ValueError("검증된 LiDAR 장착·추적 설정이 필요합니다")
        if source_mode not in ("live", "replay"):
            raise ValueError("지원하지 않는 LiDAR 출처입니다")
        session_id = str(uuid4()) if session_id is None else session_id
        if not isinstance(session_id, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", session_id):
            raise ValueError("LiDAR 세션 형식이 잘못되었습니다")
        self.mounting, self.settings = mounting, settings
        self.source_mode, self.adapter_session = source_mode, session_id
        self._lock = RLock()
        self._connected = True
        self._epoch = self._sequence = 0
        self._now_ns = None
        self._latest = None
        self._reason = "waiting_for_scan"

    def _empty(self, reason):
        return {"schema": LIDAR_OBSERVATION_SCHEMA, "sensor_id": "lidar_center",
                "frame_id": self.mounting.lidar.frame_id, "adapter_session": self.adapter_session,
                "stream_epoch": self._epoch, "sample_sequence": self._sequence,
                "source_mode": self.source_mode, "timestamp_source": "HOST_RX",
                "rx_monotonic_ns": 0, "valid_until_monotonic_ns": 0,
                "valid": False, "invalid_reason": reason, "scan": None}

    def invalidate(self, reason="lidar_error"):
        with self._lock:
            if not isinstance(reason, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", reason):
                raise ValueError("LiDAR 오류 코드 형식이 잘못되었습니다")
            self._reason = reason
            if self._latest is not None:
                self._latest.update(valid=False, invalid_reason=reason, scan=None,
                                    valid_until_monotonic_ns=0)

    def _time(self, now_ns):
        try:
            timestamp_ns(now_ns)
        except ValueError:
            self.invalidate("invalid_timestamp")
            raise
        if self._now_ns is not None and now_ns < self._now_ns:
            self.invalidate("non_monotonic_time")
            return False
        self._now_ns = now_ns
        return True

    def feed(self, scan, now_ns):
        with self._lock:
            try:
                if not self._time(now_ns):
                    return self.snapshot(self._now_ns)
                if now_ns > MAX_MONOTONIC_NS - self.settings.lidar_timeout_ms * 1_000_000:
                    raise ValueError("LiDAR 시각에 수명을 더할 수 없습니다")
                self._sequence += 1
                if not self._connected:
                    self.invalidate("lidar_disconnected")
                    return self.snapshot(now_ns)
                if (type(scan) is not dict or set(scan) != SCAN_FIELDS
                        or type(scan["ranges"]) not in (list, tuple)
                        or not 1 <= len(scan["ranges"]) <= self.settings.max_rays):
                    self._latest = self._empty("invalid_scan")
                    return deepcopy(self._latest)
                scalars_ok = all(type(scan[name]) in (int, float) and -100000 <= scan[name] <= 100000
                                 and math.isfinite(scan[name]) for name in SCAN_FIELDS - {"ranges"})
                rays_ok = all(value is None or type(value) is float
                              or (type(value) is int and -100000 <= value <= 100000)
                              for value in scan["ranges"])
                if not scalars_ok or not rays_ok:
                    self._latest = self._empty("invalid_scan")
                    return deepcopy(self._latest)
                # 수치·기하는 클러스터 계층에서 검증한다. 복사 전에 배열 크기를 제한한다.
                observation = self._empty(None)
                observation.update(rx_monotonic_ns=now_ns, valid=True, scan=deepcopy(scan),
                                   valid_until_monotonic_ns=now_ns + self.settings.lidar_timeout_ms * 1_000_000)
                self._latest, self._reason = observation, None
                return deepcopy(observation)
            except (ValueError, TypeError, OverflowError):
                self.invalidate("invalid_lidar_input")
                raise

    def disconnected(self):
        with self._lock:
            self._connected = False
            self.invalidate("lidar_disconnected")

    def reconnect(self):
        with self._lock:
            self._connected = True
            self._epoch += 1
            self._latest = None
            self._reason = "waiting_for_scan"

    def snapshot(self, now_ns):
        with self._lock:
            self._time(now_ns)
            if self._latest is None:
                return self._empty(self._reason)
            if self._latest["valid"] and now_ns >= self._latest["valid_until_monotonic_ns"]:
                self.invalidate("lidar_stale")
            return deepcopy(self._latest)
