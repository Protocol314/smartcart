"""LiDAR 전체 스캔을 보수적으로 평가한다. 장착 검수 전에는 이동 허가를 만들지 않는다."""
from dataclasses import dataclass
import math

from .control import _time


@dataclass(frozen=True)
class ClearanceConfig:
    """반경은 LiDAR 중심 기준으로 차체 회전 영역과 정지거리를 모두 포함해야 한다."""
    stop_radius_m: float
    geometry_verified: bool = False
    max_age_ms: int = 300
    frame_id: str = 'lidar_scan'

    def __post_init__(self):
        if (type(self.stop_radius_m) not in (int, float) or not math.isfinite(self.stop_radius_m)
                or not .1 <= self.stop_radius_m <= 10):
            raise ValueError('정지 반경이 잘못되었습니다')
        if type(self.geometry_verified) is not bool or type(self.max_age_ms) is not int or not 50 <= self.max_age_ms <= 1000:
            raise ValueError('장착 검수 또는 수신 수명이 잘못되었습니다')
        if type(self.frame_id) is not str or not self.frame_id or len(self.frame_id) > 128:
            raise ValueError('LiDAR 프레임이 잘못되었습니다')


def evaluate_clearance(observation, now_ns, config):
    """미관측·범위 밖을 장애물 없음으로 취급하지 않는다. 센서/구동 승인과 별개다."""
    if not isinstance(config, ClearanceConfig):
        raise ValueError('장애물 판정 설정이 필요합니다')
    now = _time(now_ns)
    def blocked(reason):
        return {'valid': False, 'clear': False, 'reason': reason, 'valid_until_ns': 0}
    if not isinstance(observation, dict) or observation.get('schema') != 'smartcart.lidar_observation.v1':
        return blocked('missing_scan')
    if (observation.get('valid') is not True or observation.get('source_mode') not in ('live', 'replay')
            or observation.get('sensor_id') != 'lidar_center' or observation.get('frame_id') != config.frame_id
            or observation.get('timestamp_source') != 'HOST_RX'
            or type(observation.get('sample_sequence')) is not int or observation['sample_sequence'] < 1):
        return blocked('invalid_scan')
    try:
        rx, deadline = _time(observation.get('rx_monotonic_ns')), _time(observation.get('valid_until_monotonic_ns'))
        if rx > now or now >= deadline or now - rx >= config.max_age_ms * 1_000_000:
            return blocked('stale_scan')
        scan = observation.get('scan')
        if not isinstance(scan, dict):
            return blocked('invalid_scan')
        values = scan.get('ranges')
        if type(values) not in (list, tuple) or not 3 <= len(values) <= 4096:
            return blocked('invalid_scan')
        start, end, step, lower, upper = (scan.get(k) for k in
                                          ('angle_min', 'angle_max', 'angle_increment', 'range_min', 'range_max'))
        if any(type(x) not in (int, float) or not math.isfinite(x) for x in (start, end, step, lower, upper)):
            return blocked('invalid_scan')
        if not 0 < step <= .1 or not 0 <= lower <= config.stop_radius_m <= upper or lower >= upper:
            return blocked('invalid_scan')
        span = (len(values) - 1) * step
        if (not 2 * math.pi - 1.5 * step <= span <= 2 * math.pi + step
                or abs((end - start) - span) > 1.5 * step):
            return blocked('incomplete_coverage')
        unknown = False
        for value in values:
            if type(value) not in (int, float) or not math.isfinite(value) or not lower <= value <= upper:
                unknown = True
            elif value <= config.stop_radius_m:
                return blocked('obstacle')
        if unknown:
            return blocked('unobserved_rays')
        if not config.geometry_verified:
            return blocked('geometry_unverified')
        return {'valid': True, 'clear': True, 'reason': None,
                'valid_until_ns': min(deadline, rx + config.max_age_ms * 1_000_000)}
    except (TypeError, ValueError, OverflowError):
        return blocked('invalid_scan')
