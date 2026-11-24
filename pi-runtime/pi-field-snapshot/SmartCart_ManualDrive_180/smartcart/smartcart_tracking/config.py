"""기존 장착·UWB 설정과 분리된 클러스터·대상 연관·추적 가정."""
from dataclasses import asdict, dataclass
import json
from pathlib import Path

from smartcart_uwb.config import MAX_CONFIG_BYTES, _boolean, _integer, _number, _object
from smartcart_uwb.config import _reject_constant, _unique_object

TRACKING_CONFIG_SCHEMA = "smartcart.target_tracking_config.v1"


@dataclass(frozen=True)
class TrackingConfig:
    schema: str = TRACKING_CONFIG_SCHEMA
    cluster_gap_base_m: float = .10
    cluster_gap_scale: float = 1.5
    min_cluster_points: int = 3
    max_cluster_points: int = 600
    min_cluster_width_m: float = .04
    max_cluster_width_m: float = .90
    max_clusters: int = 128
    max_rays: int = 8192
    max_range_m: float = 10.0
    lidar_timeout_ms: int = 300
    max_scan_duration_ms: float = 200.0
    lidar_range_error_m: float = .03
    mount_translation_error_m: float = .03
    mount_yaw_error_deg: float = 3.0
    max_host_rx_delay_ms: float = 50.0
    max_relative_speed_mps: float = 3.0
    max_sensor_skew_ms: float = 100.0
    max_uwb_error_m: float = 1.2
    tag_cluster_offset_m: float = .45
    max_association_radius_m: float = 2.0
    continuity_slack_m: float = .25
    confirmation_hits: int = 3
    occlusion_timeout_ms: int = 400
    velocity_alpha: float = .5
    assumptions_verified: bool = False

    def __post_init__(self):
        if self.schema != TRACKING_CONFIG_SCHEMA:
            raise ValueError("지원하지 않는 대상 추적 설정입니다")
        numbers = {
            "cluster_gap_base_m": (.01, 1), "cluster_gap_scale": (0, 5),
            "min_cluster_width_m": (.005, 1), "max_cluster_width_m": (.01, 3),
            "max_range_m": (.5, 30), "max_scan_duration_ms": (1, 500),
            "lidar_range_error_m": (.001, 1), "mount_translation_error_m": (.001, 1),
            "mount_yaw_error_deg": (0, 45), "max_host_rx_delay_ms": (0, 1000),
            "max_relative_speed_mps": (.1, 10), "max_sensor_skew_ms": (1, 500),
            "max_uwb_error_m": (.01, 3), "tag_cluster_offset_m": (0, 1),
            "max_association_radius_m": (.05, 4), "continuity_slack_m": (.01, 2),
            "velocity_alpha": (.01, 1),
        }
        integers = {"min_cluster_points": (2, 1000), "max_cluster_points": (2, 8192),
                    "max_clusters": (1, 256), "max_rays": (8, 8192),
                    "lidar_timeout_ms": (20, 2000), "confirmation_hits": (2, 10),
                    "occlusion_timeout_ms": (20, 2000)}
        for name, (minimum, maximum) in numbers.items():
            object.__setattr__(self, name, _number(getattr(self, name), name, minimum, maximum))
        for name, (minimum, maximum) in integers.items():
            _integer(getattr(self, name), name, minimum, maximum)
        _boolean(self.assumptions_verified, "assumptions_verified")
        if (self.min_cluster_points > self.max_cluster_points or self.max_cluster_points > self.max_rays
                or self.min_cluster_width_m > self.max_cluster_width_m):
            raise ValueError("클러스터 최소·최대 범위가 일치하지 않습니다")


def config_from_dict(payload: object) -> TrackingConfig:
    return TrackingConfig(**_object(payload, set(TrackingConfig.__dataclass_fields__), "tracking_config"))


def config_dict(settings: TrackingConfig) -> dict:
    if not isinstance(settings, TrackingConfig):
        raise ValueError("검증된 추적 설정이 필요합니다")
    return asdict(settings)


def load_tracking_config(path: str | Path) -> TrackingConfig:
    path = Path(path)
    if not path.is_file() or path.stat().st_size > MAX_CONFIG_BYTES:
        raise ValueError("추적 설정은 64 KiB 이내의 일반 파일이어야 합니다")
    with path.open("rb") as handle:
        raw = handle.read(MAX_CONFIG_BYTES + 1)
    if len(raw) > MAX_CONFIG_BYTES:
        raise ValueError("추적 설정 크기 상한을 넘었습니다")
    try:
        payload = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_object,
                             parse_constant=_reject_constant)
    except (UnicodeError, json.JSONDecodeError, RecursionError) as exc:
        raise ValueError("올바른 UTF-8 추적 설정이 필요합니다") from exc
    return config_from_dict(payload)
