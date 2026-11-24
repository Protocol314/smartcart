"""왼쪽 BU04 단독 관측을 임시 위치로 투영한다. 모터 구동 권한은 없다."""
from copy import deepcopy
from threading import RLock

from .estimator import TagEstimator, _SnapshotError, _integer
from .geometry import GeometryError, project_observation
from .observation import MAX_MONOTONIC_NS


class SingleLeftEstimator:
    """원래 수신 시각을 보존하고 오류 뒤에는 새로운 관측을 요구한다."""

    def __init__(self, config, settings, source_mode="replay"):
        self.validator = TagEstimator(config, settings, source_mode=source_mode)
        self.config, self.settings, self.source_mode = config, settings, source_mode
        self._lock = RLock()
        self._now = self._session = self._epoch = self._last = None
        self._retired = set()
        self._barrier = 0

    def _invalid(self, reason):
        return {"schema": "smartcart.single_uwb_estimate.v1", "valid": False,
                "invalid_reason": reason, "position_xy_m": None, "error_radius_m": None,
                "valid_until_monotonic_ns": 0, "sensor_id": "uwb_left",
                "frame_id": "base_link", "reference_monotonic_ns": None,
                "sample_sequence": self._last[0] if self._last is not None else 0, "stream_epoch": self._epoch, "adapter_session": self._session,
                "estimate_age_ms": None,
                "source_mode": self.source_mode, "provisional": True, "drive_authorized": False}

    def reset(self, reason="reset"):
        with self._lock:
            if self._last is not None:
                self._barrier = max(self._barrier, self._last[0])
            return self._invalid(reason)

    def update(self, snapshot, now_ns):
        with self._lock:
            try:
                _integer(now_ns, maximum=MAX_MONOTONIC_NS)
                if self._now is not None and now_ns < self._now:
                    return self.reset("non_monotonic_time")
                self._now = now_ns
                session, states = self.validator._validate_snapshot(snapshot, now_ns)
                state = states["uwb_left"]
                epoch = state["stream_epoch"]
                if self._session is not None and session != self._session:
                    if session in self._retired or len(self._retired) >= 32:
                        return self.reset("retired_session")
                    self._retired.add(self._session)
                    self._epoch = self._last = None
                    self._barrier = 0
                if self._epoch is not None and epoch < self._epoch:
                    return self.reset("epoch_regression")
                if self._epoch is not None and epoch > self._epoch:
                    self._last = None
                    self._barrier = 0
                self._session, self._epoch = session, epoch
                observation = state["latest"]
                if observation is None or not state["connected"] or not observation["valid"]:
                    if observation is not None:
                        self._barrier = max(self._barrier, observation["sample_sequence"])
                    return self.reset(state["last_error"] or "waiting_for_left")
                seq, rx = observation["sample_sequence"], observation["rx_monotonic_ns"]
                fingerprint = tuple((k, observation[k]) for k in sorted(observation))
                if self._last is not None:
                    previous_seq, previous_rx, previous_fingerprint, previous_device = self._last
                    if seq < previous_seq or rx < previous_rx:
                        return self.reset("sample_regression")
                    if seq == previous_seq and fingerprint != previous_fingerprint:
                        return self.reset("sample_reuse")
                    if seq > previous_seq:
                        delta = (observation["device_sequence"] - previous_device) & 255
                        if not 1 <= delta <= 127:
                            return self.reset("device_sequence_regression")
                self._last = (seq, rx, fingerprint, observation["device_sequence"])
                deadline = min(observation["valid_until_monotonic_ns"],
                               rx + int(self.settings.max_observation_age_ms * 1_000_000))
                if now_ns >= deadline:
                    return self.reset("observation_stale")
                if seq <= self._barrier:
                    return self._invalid("awaiting_fresh_left")
                geometry = project_observation(observation, self.config.sensors["uwb_left"], self.settings)
                age_s = (now_ns - rx) / 1e9
                error = max(self.settings.position_error_floor_m, geometry["error_radius_m"])
                error += self.settings.max_relative_speed_mps * (age_s + self.settings.max_host_rx_delay_ms / 1000)
                return {"schema": "smartcart.single_uwb_estimate.v1", "valid": True,
                        "invalid_reason": None, "position_xy_m": deepcopy(geometry["position_xy_m"]),
                        "error_radius_m": error, "valid_until_monotonic_ns": deadline,
                        "reference_monotonic_ns": rx, "sample_sequence": seq,
                        "stream_epoch": epoch, "adapter_session": session, "estimate_age_ms": age_s * 1000,
                        "sensor_id": "uwb_left", "frame_id": "base_link",
                        "source_mode": self.source_mode, "provisional": True, "drive_authorized": False}
            except GeometryError as error:
                # 기하 조건 오류는 매 평가마다 원인을 보존한다. 잘못된 위치는 저장하지 않는다.
                return self._invalid(error.reason)
            except (ValueError, TypeError, OverflowError, _SnapshotError):
                return self.reset("invalid_snapshot")
