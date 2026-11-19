"""위치·원시 광선을 보관하지 않는 읽기 전용 현장 관측 집계."""
from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass, field
import math
from threading import RLock

from smartcart_tracking.clusters import _scan
from smartcart_tracking.config import TrackingConfig
from smartcart_uwb.config import CollectorConfig, SENSOR_IDS
from smartcart_uwb.estimation_config import EstimationConfig
from smartcart_uwb.estimator import TagEstimator
from smartcart_uwb.observation import timestamp_ns

SENSORS = (*SENSOR_IDS, "lidar_center")
REPORT_SCHEMA = "smartcart.field_monitor.v1"
_ADAPTER_ERRORS = {"status_json", "tracking_json", "scan_message", "ros_runtime", "ros_interrupted"}


class _Window:
    def __init__(self, size):
        self.values = deque(maxlen=size)
        self.count = 0
        self.maximum = None

    def add(self, value):
        self.values.append(float(value))
        self.count += 1
        self.maximum = value if self.maximum is None else max(value, self.maximum)

    def report(self):
        ordered = sorted(self.values)
        def percentile(fraction):
            return ordered[max(0, math.ceil(len(ordered) * fraction) - 1)] if ordered else None
        return {"unit": "ms", "count": self.count, "window_count": len(ordered),
                "window_capacity": self.values.maxlen, "p50": percentile(.5), "p95": percentile(.95),
                "max": ordered[-1] if ordered else None, "max_observed": self.maximum}


@dataclass
class _Sensor:
    state: str = "waiting"
    rx_ns: int | None = None
    deadline_ns: int = 0
    sample: tuple | None = None
    fingerprint: tuple | None = None
    device_sequence: int | None = None
    valid: bool = False
    counts: Counter = field(default_factory=Counter)
    manual_phase: str = "untested"
    manual_cycles: int = 0
    last_sample_sequence: int | None = None
    last_epoch: int | None = None


class FieldMonitor:
    """타임스탬프의 같은 호스트 전제는 검사하지만 물리 동기화·단절 원인은 추정하지 않는다."""
    def __init__(self, mounting: CollectorConfig, uwb_settings: EstimationConfig,
                 settings: TrackingConfig, *, window_size=2048, source_mode="live"):
        if (not isinstance(mounting, CollectorConfig) or not isinstance(uwb_settings, EstimationConfig)
                or not isinstance(settings, TrackingConfig)):
            raise ValueError("검증된 장착·UWB·추적 설정이 필요합니다")
        if type(window_size) is not int or not 16 <= window_size <= 8192:
            raise ValueError("통계 창은 16~8192개여야 합니다")
        if source_mode not in ("live", "replay"):
            raise ValueError("관측 출처는 live 또는 replay여야 합니다")
        self.mounting, self.uwb_settings, self.settings = mounting, uwb_settings, settings
        self.source_mode = source_mode
        self._lock = RLock()
        self._started_ns = self._now_ns = None
        self._sensors = {name: _Sensor() for name in SENSORS}
        self._counts = Counter()
        self._session = None
        self._retired_sessions = set()
        self._last_pair = None
        self._tracking_latest = None
        # update/기하 계산은 호출하지 않는다. 기존의 엄격한 전체 관측 스키마 검증만 재사용한다.
        self._validator = TagEstimator(mounting, uwb_settings, source_mode=source_mode)
        names = ["pair_rx_skew", "lidar_pair_rx_skew", "lidar_scan_duration"]
        names += [name + suffix for name in SENSORS for suffix in ("_interval", "_age")]
        self._windows = {name: _Window(window_size) for name in names}

    def begin(self, now_ns):
        with self._lock:
            timestamp_ns(now_ns)
            if self._started_ns is None:
                self._started_ns = self._now_ns = now_ns
            return self._started_ns == now_ns

    def _advance(self, now_ns):
        try:
            timestamp_ns(now_ns)
            if self._now_ns is not None and now_ns < self._now_ns:
                raise ValueError()
        except ValueError:
            self._counts["invalid_monitor_time"] += 1
            for name in SENSORS:
                self._fault(name, "invalid_monitor_time")
            return False
        if self._started_ns is None:
            self.begin(now_ns)
        self._now_ns = now_ns
        self._expire(now_ns)
        return True

    def _expire(self, now_ns):
        for name, sensor in self._sensors.items():
            if sensor.state == "fresh" and now_ns >= sensor.deadline_ns:
                self._state(name, "stale")
        if self._tracking_latest is not None and now_ns >= self._tracking_latest["valid_until_monotonic_ns"]:
            self._tracking_latest["valid"] = False

    def _state(self, name, state):
        sensor = self._sensors[name]
        previous = sensor.state
        if previous == state:
            return
        sensor.state = state
        sensor.counts["state_transitions"] += 1
        if state == "disconnected":
            sensor.counts["explicit_disconnects"] += 1
        if previous == "fresh" and state != "fresh":
            sensor.counts["freshness_losses"] += 1
            if state == "stale":
                sensor.counts["stale_transitions"] += 1
            if sensor.manual_phase == "awaiting_outage" and state in ("stale", "disconnected"):
                sensor.manual_phase = "awaiting_replug"
        if previous in ("stale", "disconnected", "invalid") and state == "fresh":
            sensor.counts["recoveries"] += 1
            if sensor.manual_phase == "awaiting_recovery":
                sensor.manual_cycles += 1
                sensor.manual_phase = "complete"

    def _fault(self, name, reason):
        self._sensors[name].counts[reason] += 1
        self._sensors[name].valid = False
        self._state(name, "invalid")

    def note_error(self, code, now_ns):
        """ROS 경계의 형식 오류도 기존 신선함을 유지하지 않는다."""
        if code not in _ADAPTER_ERRORS:
            raise ValueError("지원하지 않는 어댑터 오류 코드입니다")
        with self._lock:
            if not self._advance(now_ns):
                return False
            self._counts[code] += 1
            names = (SENSORS if code in ("ros_runtime", "ros_interrupted") else SENSOR_IDS if code == "status_json"
                     else ("lidar_center",) if code == "scan_message" else ())
            for name in names:
                self._fault(name, "adapter_input_errors")
            if code == "tracking_json":
                self._tracking_latest = None
            return False

    def _context(self, session):
        if session != self._session:
            if session in self._retired_sessions:
                self._counts["retired_session"] += 1
                return False
            if self._session is not None:
                if len(self._retired_sessions) >= 32:
                    self._counts["session_history_limit"] += 1
                    return False
                self._retired_sessions.add(self._session)
                self._counts["session_changes"] += 1
            self._session = session
            for name in SENSOR_IDS:
                sensor = self._sensors[name]
                sensor.sample = sensor.fingerprint = sensor.device_sequence = None
                sensor.last_epoch = sensor.last_sample_sequence = None
        return True

    def _uwb(self, name, state, session, now_ns):
        sensor = self._sensors[name]
        epoch, observation = state["stream_epoch"], state["latest"]
        if sensor.last_epoch is not None and epoch < sensor.last_epoch:
            self._fault(name, "epoch_regressions")
            return
        if sensor.last_epoch is not None and epoch > sensor.last_epoch:
            sensor.counts["epoch_changes"] += 1
            sensor.sample = sensor.fingerprint = sensor.device_sequence = None
        sensor.last_epoch = epoch
        if observation is None:
            sensor.valid, sensor.deadline_ns = False, 0
            self._state(name, "disconnected" if not state["connected"] else "waiting")
            return True
        sample, rx = observation["sample_sequence"], observation["rx_monotonic_ns"]
        key = (session, epoch, sample)
        fingerprint = tuple(observation[field] for field in (
            "rx_monotonic_ns", "valid_until_monotonic_ns", "device_sequence", "id_valid", "measurement_valid"))
        new = key != sensor.sample
        if not new:
            sensor.counts["repeated_sample_heartbeats"] += 1
            if sensor.fingerprint != fingerprint or (not sensor.valid and observation["valid"]):
                self._fault(name, "sample_reuse")
                return
        else:
            if sensor.sample is not None and sample <= sensor.last_sample_sequence:
                self._fault(name, "sample_regressions")
                return
            if sensor.rx_ns is not None and rx < sensor.rx_ns:
                self._fault(name, "rx_regressions")
                return
            device = observation["device_sequence"]
            if observation["valid"] and device is not None and sensor.device_sequence is not None:
                delta = (device - sensor.device_sequence) & 255
                if not 0 < delta < 128:
                    self._fault(name, "device_sequence_duplicates" if delta == 0 else "device_sequence_regressions")
                    return
            if sensor.rx_ns is not None and rx > sensor.rx_ns:
                self._windows[name + "_interval"].add((rx - sensor.rx_ns) / 1e6)
            self._windows[name + "_age"].add((now_ns - rx) / 1e6)
            if now_ns - rx > self.uwb_settings.max_host_rx_delay_ms * 1e6:
                sensor.counts["host_rx_delay_assumption_exceeded"] += 1
            sensor.counts["new_samples"] += 1
            sensor.sample, sensor.fingerprint = key, fingerprint
            sensor.last_sample_sequence, sensor.rx_ns = sample, rx
            if observation["valid"]:
                sensor.device_sequence = device
            reason = observation["invalid_reason"] or ""
            if reason == "id_mismatch":
                sensor.counts["id_mismatches"] += 1
            elif reason == "bad_frame:checksum":
                sensor.counts["checksum_failures"] += 1
            elif reason == "duplicate_sequence":
                sensor.counts["device_sequence_duplicates"] += 1
            elif reason == "out_of_order_sequence":
                sensor.counts["device_sequence_regressions"] += 1
            elif not observation["valid"]:
                sensor.counts["other_invalid_samples"] += 1
        sensor.valid = observation["valid"]
        sensor.deadline_ns = min(observation["valid_until_monotonic_ns"],
            rx + int(self.uwb_settings.max_observation_age_ms * 1e6))
        if not state["connected"]:
            self._state(name, "disconnected")
        elif not observation["valid"]:
            self._state(name, "stale" if observation["invalid_reason"] == "stale" else "invalid")
        else:
            self._state(name, "fresh" if now_ns < sensor.deadline_ns else "stale")
        return True

    def _pair(self):
        left, right = (self._sensors[name] for name in SENSOR_IDS)
        if left.state == right.state == "fresh":
            key = (left.sample, right.sample)
            if key != self._last_pair:
                self._last_pair = key
                skew = abs(left.rx_ns - right.rx_ns) / 1e6
                self._windows["pair_rx_skew"].add(skew)
                if skew > self.uwb_settings.max_pair_skew_ms:
                    self._counts["pair_skew_exceeded"] += 1

    def observe_status(self, snapshot, now_ns):
        with self._lock:
            if not self._advance(now_ns):
                return False
            self._counts["status_messages"] += 1
            try:
                session, states = self._validator._validate_snapshot(snapshot, now_ns)
            except (ValueError, TypeError, OverflowError) as error:
                self._counts["invalid_status_messages"] += 1
                reason = getattr(error, "reason", None)
                if reason in ("invalid_timestamp", "source_mode_mismatch"):
                    self._counts[reason] += 1
                for name in SENSOR_IDS:
                    self._fault(name, "invalid_metadata")
                return False
            if not self._context(session):
                for name in SENSOR_IDS:
                    self._fault(name, "invalid_session")
                return False
            accepted = [self._uwb(name, states[name], session, now_ns) for name in SENSOR_IDS]
            self._pair()
            return all(accepted)

    def observe_scan(self, scan, now_ns, *, frame_id):
        with self._lock:
            if not self._advance(now_ns):
                return False
            self._counts["scan_messages"] += 1
            if frame_id != self.mounting.lidar.frame_id:
                self._fault("lidar_center", "frame_mismatches")
                return False
            try:
                # 정규화는 한 호출 안에서만 사용하며 광선·점·군집을 멤버나 보고서에 복사하지 않는다.
                _normalised, duration = _scan(scan, self.settings)
            except (ValueError, TypeError, OverflowError) as error:
                self._fault("lidar_center", "scan_duration_errors" if getattr(error, "reason", None) == "scan_duration"
                            else "invalid_scans")
                return False
            sensor = self._sensors["lidar_center"]
            if sensor.rx_ns is not None and now_ns > sensor.rx_ns:
                self._windows["lidar_center_interval"].add((now_ns - sensor.rx_ns) / 1e6)
            self._windows["lidar_scan_duration"].add(duration * 1000)
            if scan["scan_time"] == scan["time_increment"] == 0:
                sensor.counts["zero_duration_used_configured_bound"] += 1
            self._windows["lidar_center_age"].add(0)
            sensor.counts["new_samples"] += 1
            sensor.rx_ns, sensor.valid = now_ns, True
            sensor.last_sample_sequence = sensor.counts["new_samples"]
            sensor.deadline_ns = now_ns + self.settings.lidar_timeout_ms * 1_000_000
            self._state("lidar_center", "fresh")
            left, right = (self._sensors[name] for name in SENSOR_IDS)
            if left.state == right.state == "fresh":
                skew = abs(now_ns - max(left.rx_ns, right.rx_ns)) / 1e6
                self._windows["lidar_pair_rx_skew"].add(skew)
                if skew > self.settings.max_sensor_skew_ms:
                    self._counts["lidar_pair_skew_exceeded"] += 1
            return True

    def observe_tracking_status(self, value, now_ns):
        """위치·대상 ID는 읽거나 복사하지 않고 상태·수명·출처만 투영한다."""
        with self._lock:
            if not self._advance(now_ns):
                return False
            self._counts["tracking_messages"] += 1
            try:
                if (type(value) is not dict or set(value) != {"uwb_estimate", "lidar_clusters", "track", "source_mode", "drive_authorized"}
                        or value.get("source_mode") != self.source_mode
                        or value.get("drive_authorized") is not False or type(value.get("track")) is not dict):
                    raise ValueError()
                for name, schema in (("uwb_estimate", "smartcart.uwb_tag_estimate.v1"),
                                     ("lidar_clusters", "smartcart.lidar_clusters.v1")):
                    item = value[name]
                    if (type(item) is not dict or len(item) > 64 or item.get("schema") != schema
                            or item.get("source_mode") != self.source_mode or item.get("drive_authorized") is not False):
                        raise ValueError()
                track = value["track"]
                if (len(track) > 64 or track.get("schema") != "smartcart.target_track.v1" or track.get("source_mode") != self.source_mode
                        or track.get("drive_authorized") is not False or type(track.get("valid")) is not bool
                        or track.get("state") not in {"waiting", "acquiring", "tracking", "coasting", "ambiguous", "lost", "error"}):
                    raise ValueError()
                deadline = timestamp_ns(track.get("valid_until_monotonic_ns"))
                reference = track.get("reference_monotonic_ns")
                if reference is not None and timestamp_ns(reference) > now_ns:
                    raise ValueError()
                if track["valid"] and (track["state"] != "tracking" or reference is None or not reference <= now_ns < deadline):
                    raise ValueError()
                if deadline > now_ns + max(self.mounting.timeout_ms, self.settings.lidar_timeout_ms) * 1_000_000:
                    raise ValueError()
                self._tracking_latest = {"state": track["state"], "valid": track["valid"],
                                         "rx_monotonic_ns": now_ns, "valid_until_monotonic_ns": deadline}
                return True
            except (ValueError, TypeError):
                self._tracking_latest = None
                self._counts["invalid_tracking_messages"] += 1
                return False

    def tick(self, now_ns):
        with self._lock:
            if not self._advance(now_ns):
                return False
            return True

    def mark(self, sensor_id, action, now_ns):
        """수동 선언은 신선→분리 선언→관측 손실→재연결 선언→관측 복구 순서로만 완료한다."""
        if sensor_id not in SENSORS or action not in ("unplug", "replug"):
            raise ValueError("센서 이름과 unplug/replug 수동 선언이 필요합니다")
        with self._lock:
            if not self._advance(now_ns):
                return False
            self.tick(now_ns)
            sensor = self._sensors[sensor_id]
            sensor.counts["manual_" + action + "_declarations"] += 1
            if action == "unplug" and sensor.state == "fresh" and sensor.manual_phase in ("untested", "complete"):
                sensor.manual_phase = "awaiting_outage"
                return True
            if action == "replug" and sensor.manual_phase == "awaiting_replug" and sensor.state != "fresh":
                sensor.manual_phase = "awaiting_recovery"
                return True
            sensor.counts["manual_order_errors"] += 1
            return False

    def report(self, now_ns=None):
        with self._lock:
            if now_ns is not None:
                self.tick(now_ns)
            manual = {name: {"source": "operator_declaration", "phase": sensor.manual_phase,
                            "status": "완료" if sensor.manual_cycles else "미시험" if sensor.manual_phase == "untested" else "미완료",
                            "completed_cycles": sensor.manual_cycles}
                      for name, sensor in self._sensors.items()}
            complete = (self.source_mode == "live" and all(sensor.manual_cycles and sensor.manual_phase == "complete" and sensor.state == "fresh"
                                                         for sensor in self._sensors.values()))
            statistics = {name + "_ms": window.report() for name, window in self._windows.items()}
            sensors = {}
            for name, sensor in self._sensors.items():
                interval = statistics[name + "_interval_ms"]["p50"]
                sensors[name] = {"state": sensor.state, "counts": dict(sensor.counts),
                    "latest_metadata": {"rx_monotonic_ns": sensor.rx_ns, "valid_until_monotonic_ns": sensor.deadline_ns,
                                        "sample_sequence": sensor.last_sample_sequence, "stream_epoch": sensor.last_epoch},
                    "median_arrival_rate_hz": 1000 / interval if interval else None}
            return {"schema": REPORT_SCHEMA, "source_mode": self.source_mode, "drive_authorized": False,
                    "purpose": "read_only_field_observation", "completion": bool(complete),
                    "physical_acceptance": False, "clock_basis": "same_host_HOST_RX_required_not_hardware_synchronised",
                    "started_monotonic_ns": self._started_ns, "last_monotonic_ns": self._now_ns,
                    "duration_ms": None if self._started_ns is None else (self._now_ns - self._started_ns) / 1e6,
                    "counts": dict(self._counts), "sensors": sensors, "statistics": statistics,
                    "manual_unplug_tests": manual, "tracking_latest_metadata": None if self._tracking_latest is None else dict(self._tracking_latest),
                    "thresholds": {"uwb_timeout_ms": self.mounting.timeout_ms,
                        "max_pair_skew_ms": self.uwb_settings.max_pair_skew_ms,
                        "max_observation_age_ms": self.uwb_settings.max_observation_age_ms,
                        "uwb_max_host_rx_delay_ms": self.uwb_settings.max_host_rx_delay_ms,
                        "lidar_timeout_ms": self.settings.lidar_timeout_ms,
                        "max_sensor_skew_ms": self.settings.max_sensor_skew_ms,
                        "max_scan_duration_ms": self.settings.max_scan_duration_ms},
                    "limits": ["수동 선언은 물리 분리의 독립적인 전기 검증이 아닙니다",
                               "신선도 손실만으로 USB 분리·RF 장애·프로세스 중단을 구별하지 않습니다",
                               "p50/p95/max는 유한 최근 창, max_observed는 실행 전체 최대입니다",
                               "위치·각도·거리·대상 ID·원시 광선을 보관하지 않습니다"]}
