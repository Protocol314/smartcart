"""같은 호스트의 최신 전체 UWB 상태와 LiDAR 스캔만 연결하는 실시간 코디네이터."""

from copy import deepcopy
from threading import RLock

from smartcart_uwb.config import CollectorConfig
from smartcart_uwb.estimation_config import EstimationConfig
from smartcart_uwb.estimator import TagEstimator
from smartcart_uwb.single_estimator import SingleLeftEstimator
from smartcart_uwb.observation import timestamp_ns
from .clusters import cluster_scan
from .config import TrackingConfig
from .observation import LidarCollector
from .tracker import TargetTracker

_REJECTED_STATUS = {"invalid_snapshot", "invalid_timestamp", "source_mode_mismatch", "sample_regression",
                    "sample_reuse", "device_sequence_regression", "epoch_regression", "retired_session",
                    "session_limit", "estimation_error", "non_monotonic_time"}


class LiveTrackingRuntime:
    """입력 수신 자체로 원래 관측 만료를 연장하지 않으며 장치·ROS·파일을 열지 않는다."""

    def __init__(self, mounting: CollectorConfig, uwb_settings: EstimationConfig,
                 settings: TrackingConfig, source_mode="live", single_left=False):
        if source_mode != "live":
            raise ValueError("실시간 코디네이터에는 live 출처만 허용합니다")
        if (not isinstance(mounting, CollectorConfig) or not isinstance(uwb_settings, EstimationConfig)
                or not isinstance(settings, TrackingConfig)):
            raise ValueError("검증된 장착·위치·추적 설정이 필요합니다")
        self.config, self.settings, self.uwb_settings = mounting, settings, uwb_settings
        self.source_mode = source_mode
        self.estimator = (SingleLeftEstimator if single_left else TagEstimator)(mounting, uwb_settings, source_mode="live")
        self.lidar = LidarCollector(mounting, settings, source_mode="live")
        self.tracker = TargetTracker(mounting, settings, source_mode="live")
        self._lock = RLock()
        self._now = None
        self._status = None
        self._estimate = self.estimator.reset("waiting_for_pair")
        self._cluster_key = self._clusters = None

    def _time(self, now_ns):
        try:
            timestamp_ns(now_ns)
            if self._now is not None and now_ns < self._now:
                raise ValueError("호스트 시각이 역행했습니다")
        except ValueError:
            self.invalidate_status("invalid_timestamp")
            self.lidar.invalidate("invalid_timestamp")
            raise
        self._now = now_ns

    def invalidate_status(self, reason="invalid_uwb_status"):
        """JSON 경계의 오류도 이전 UWB 추정과 추적을 즉시 폐기한다."""
        with self._lock:
            self._status = None
            self._estimate = self.estimator.reset(reason)
            self.tracker.reset(reason)

    def invalidate_scan(self, reason="invalid_lidar_scan"):
        with self._lock:
            self.lidar.invalidate(reason)
            self._cluster_key = self._clusters = None
            self.tracker.reset(reason)

    def consume_status(self, snapshot, now_ns):
        with self._lock:
            self._time(now_ns)
            estimate = self.estimator.update(snapshot, now_ns)
            if estimate["invalid_reason"] in _REJECTED_STATUS:
                self.invalidate_status("invalid_uwb_status")
            else:
                # 성공 또는 정상 스키마의 명시적 센서 오류만 유한 상태로 보관한다.
                self._status = deepcopy(snapshot)
                self._estimate = estimate
            return self._snapshot(now_ns)

    def consume_scan(self, scan, now_ns):
        with self._lock:
            self._time(now_ns)
            try:
                self.lidar.feed(scan, now_ns)
            except (ValueError, TypeError, OverflowError):
                self.invalidate_scan("invalid_lidar_scan")
            return self._snapshot(now_ns)

    def _snapshot(self, now_ns):
        if self._status is not None:
            self._estimate = self.estimator.update(self._status, now_ns)
        observation = self.lidar.snapshot(now_ns)
        key = tuple(observation[name] for name in ("stream_epoch", "sample_sequence", "valid", "invalid_reason"))
        if key != self._cluster_key:
            self._clusters = cluster_scan(observation, self.config, self.settings)
            self._cluster_key = key
        track = self.tracker.update(self._clusters, self._estimate, now_ns)
        return deepcopy({"uwb_estimate": self._estimate, "lidar_clusters": self._clusters,
                         "track": track, "source_mode": "live", "drive_authorized": False})

    def snapshot(self, now_ns):
        with self._lock:
            self._time(now_ns)
            return self._snapshot(now_ns)
