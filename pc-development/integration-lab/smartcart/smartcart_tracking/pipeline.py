"""원시 좌우 UWB와 중앙 LiDAR를 한 가상 시계로 처리하는 재생 파이프라인."""
from copy import deepcopy
from threading import RLock

from smartcart_uwb.collector import DualCollector
from smartcart_uwb.config import CollectorConfig
from smartcart_uwb.estimation_config import EstimationConfig
from smartcart_uwb.estimator import TagEstimator
from .clusters import cluster_scan
from .config import TrackingConfig
from .observation import LidarCollector
from .tracker import TargetTracker


class TrackingPipeline:
    """재생 입력을 실측으로 바꾸지 않는다. 출력·기록·장치 제어는 호출자 책임이다."""
    def __init__(self, mounting: CollectorConfig, uwb_settings: EstimationConfig,
                 tracking_settings: TrackingConfig, *, source_mode="replay", session_id=None,
                 parser_factory=None):
        if source_mode != "replay":
            raise ValueError("원시 캡처 파이프라인은 재생 전용입니다")
        if (not isinstance(mounting, CollectorConfig) or not isinstance(uwb_settings, EstimationConfig)
                or not isinstance(tracking_settings, TrackingConfig)):
            raise ValueError("검증된 센서·위치·추적 설정이 필요합니다")
        self.config, self.settings = mounting, tracking_settings
        self.source_mode = source_mode
        self._lock = RLock()
        self._closed = False
        self._original_source = None
        self._at_ns = None
        self._cluster_key = self._clusters = None
        self.uwb = DualCollector(mounting, parser_factory=parser_factory,
                                 source_mode="replay", session_id=session_id)
        try:
            self.estimator = TagEstimator(mounting, uwb_settings, source_mode="replay")
            self.lidar = LidarCollector(mounting, tracking_settings, source_mode="replay",
                                        session_id=self.uwb.adapter_session)
            self.tracker = TargetTracker(mounting, tracking_settings, source_mode="replay")
        except Exception:
            self.uwb.close()
            raise

    def reset(self, reason="pipeline_reset"):
        with self._lock:
            self.estimator.reset(reason)
            self.lidar.invalidate(reason)
            self.tracker.reset(reason)
            self._cluster_key = self._clusters = None

    def process(self, record, now_ns):
        from .replay import validate_record
        with self._lock:
            if self._closed:
                raise RuntimeError("추적 파이프라인이 닫혔습니다")
            try:
                record = validate_record(record)
                if self._original_source is not None and record["source"] != self._original_source:
                    raise ValueError("합성과 실측 캡처를 섞을 수 없습니다")
                if self._at_ns is not None and record["at_ns"] < self._at_ns:
                    raise ValueError("캡처 시간이 역행했습니다")
                self._original_source, self._at_ns = record["source"], record["at_ns"]
                event = record["event"]
                count = 0
                if event == "uwb_bytes":
                    count = len(self.uwb.feed(record["sensor_id"], bytes.fromhex(record["data_hex"]),
                                              now_ns=now_ns, source_rx_ns=record["at_ns"]))
                elif event == "uwb_disconnect":
                    self.uwb.disconnected(record["sensor_id"], "capture_disconnect")
                elif event == "uwb_reconnect":
                    self.uwb.reconnect(record["sensor_id"])
                elif event == "lidar_scan":
                    self.lidar.feed(record["scan"], now_ns)
                elif event == "lidar_disconnect":
                    self.lidar.disconnected()
                elif event == "lidar_reconnect":
                    self.lidar.reconnect()
                return self._snapshot(now_ns, count)
            except Exception:
                self.reset("pipeline_input_error")
                raise

    def _snapshot(self, now_ns, count=0):
        uwb_estimate = self.estimator.update(self.uwb.snapshot(now_ns), now_ns)
        lidar_observation = self.lidar.snapshot(now_ns)
        key = tuple(lidar_observation[name] for name in (
            "stream_epoch", "sample_sequence", "valid", "invalid_reason"))
        if key != self._cluster_key:
            self._clusters = cluster_scan(lidar_observation, self.config, self.settings)
            self._cluster_key = key
        track = self.tracker.update(self._clusters, uwb_estimate, now_ns)
        return deepcopy({"uwb_estimate": uwb_estimate, "lidar_clusters": self._clusters,
                         "track": track, "uwb_observation_count": count,
                         "source_mode": "replay", "drive_authorized": False})

    def refresh(self, now_ns):
        """새 측정이나 캡처 레코드 없이 같은 시계에서 수명만 재평가한다."""
        with self._lock:
            if self._closed:
                raise RuntimeError("추적 파이프라인이 닫혔습니다")
            try:
                return self._snapshot(now_ns)
            except Exception:
                self.reset("pipeline_refresh_error")
                raise

    def close(self):
        with self._lock:
            if not self._closed:
                self.reset("pipeline_closed")
                self._closed = True
                self.uwb.close()

    def __enter__(self):
        if self._closed:
            raise RuntimeError("추적 파이프라인이 닫혔습니다")
        return self

    def __exit__(self, *_):
        self.close()
