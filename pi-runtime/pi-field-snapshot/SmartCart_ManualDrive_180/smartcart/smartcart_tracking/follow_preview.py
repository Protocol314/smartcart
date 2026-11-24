"""합성 추적 결과를 정규화된 가상 추종량으로 변환한다. 실제 모터에는 연결하지 않는다."""
from dataclasses import dataclass
import math

NS = 1_000_000_000


def _number(value, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError('가상 추종 수치 범위가 잘못되었습니다')
    return float(value)


def _time(value):
    if type(value) is not int or not 0 <= value < (1 << 63):
        raise ValueError('가상 추종 시각이 잘못되었습니다')
    return value


@dataclass(frozen=True)
class FollowPreviewConfig:
    """모든 값은 합성 검사용 예시이며 실물 속도·정지거리 설정이 아니다."""
    follow_distance_m: float = 1.5
    stop_distance_m: float = 1.0
    distance_gain: float = .4
    angle_gain: float = .6
    pivot_angle_deg: float = 35.0
    max_forward_fraction: float = .35
    max_turn_fraction: float = .35
    max_tracking_error_m: float = .5
    max_observation_age_ms: int = 300

    def __post_init__(self):
        for field, low, high in (('follow_distance_m', .2, 5), ('stop_distance_m', .1, 4),
                                 ('distance_gain', .01, 2), ('angle_gain', .01, 2),
                                 ('pivot_angle_deg', 5, 90), ('max_forward_fraction', .01, 1),
                                 ('max_turn_fraction', .01, 1), ('max_tracking_error_m', .01, 2)):
            _number(getattr(self, field), low, high)
        if self.stop_distance_m >= self.follow_distance_m:
            raise ValueError('정지 거리는 추종 거리보다 작아야 합니다')
        if type(self.max_observation_age_ms) is not int or not 50 <= self.max_observation_age_ms <= 1000:
            raise ValueError('가상 관측 만료 설정이 잘못되었습니다')


def _result(state, reason, forward=0.0, turn=0.0):
    return {'schema': 'smartcart.follow_preview.v1', 'state': state, 'reason': reason,
            'preview_forward_fraction': forward, 'preview_turn_fraction': turn,
            'synthetic_only': True, 'drive_authorized': False}


def simulate(track, clearance, now_ns, config=None):
    """재생 추적과 명시적 합성 장애물 상태만 받아 화면 밖 가상 수치를 반환한다.

    실제 주행 승인·GPIO·속도 피드백·안전 정지거리 계산을 제공하지 않는다.
    clearance는 synthetic, valid, forward_clear, rotation_clear, observed_ns,
    valid_until_ns를 갖는 합성 시험 입력이다.
    """
    settings = FollowPreviewConfig() if config is None else config
    if not isinstance(settings, FollowPreviewConfig):
        raise ValueError('가상 추종 설정이 필요합니다')
    try:
        now = _time(now_ns)
        if not isinstance(track, dict) or not isinstance(clearance, dict):
            return _result('blocked', 'missing_input')
        if (track.get('schema') != 'smartcart.target_track.v1' or track.get('source_mode') != 'replay' or
                track.get('provisional') is not True or track.get('drive_authorized') is not False):
            return _result('blocked', 'synthetic_tracking_required')
        if (track.get('valid') is not True or track.get('state') != 'tracking' or
                type(track.get('confirmation_count')) is not int or not 3 <= track['confirmation_count'] <= 10 or
                type(track.get('candidate_count')) is not int or track['candidate_count'] != 1 or
                type(track.get('associated_cluster_id')) is not int or track['associated_cluster_id'] < 0):
            return _result('blocked', 'tracking_unavailable')
        reference, deadline = _time(track.get('reference_monotonic_ns')), _time(track.get('valid_until_monotonic_ns'))
        if reference > now or now >= deadline or now - reference >= settings.max_observation_age_ms * 1_000_000:
            return _result('blocked', 'tracking_stale')
        if _number(track.get('error_radius_m'), 0, 100) > settings.max_tracking_error_m:
            return _result('blocked', 'tracking_uncertain')
        point = track.get('position_xy_m')
        if not isinstance(point, (tuple, list)) or len(point) != 2:
            return _result('blocked', 'invalid_position')
        x, y = (_number(value, -100, 100) for value in point)
        if (clearance.get('synthetic') is not True or clearance.get('valid') is not True or
                type(clearance.get('forward_clear')) is not bool or
                type(clearance.get('rotation_clear')) is not bool):
            return _result('blocked', 'synthetic_clearance_required')
        observed, clear_until = _time(clearance.get('observed_ns')), _time(clearance.get('valid_until_ns'))
        if observed > now or now >= clear_until or now - observed >= settings.max_observation_age_ms * 1_000_000:
            return _result('blocked', 'clearance_stale')
        distance, angle = math.hypot(x, y), math.atan2(y, x)
        if distance <= settings.stop_distance_m:
            return _result('stopped', 'inside_stop_distance')
        if abs(math.degrees(angle)) >= settings.pivot_angle_deg:
            if not clearance['rotation_clear']:
                return _result('blocked', 'rotation_obstacle')
            return _result('preview_pivot', 'target_off_axis',
                           turn=math.copysign(settings.max_turn_fraction, angle))
        forward = min(settings.max_forward_fraction, max(0.0, (distance - settings.follow_distance_m) * settings.distance_gain))
        turn = max(-settings.max_turn_fraction, min(settings.max_turn_fraction, angle * settings.angle_gain))
        if forward and not clearance['forward_clear']:
            return _result('blocked', 'forward_obstacle')
        if turn and not clearance['rotation_clear']:
            return _result('blocked', 'rotation_obstacle')
        return _result('preview_follow' if forward or turn else 'stopped', 'synthetic_only', forward, turn)
    except (TypeError, ValueError, OverflowError):
        return _result('blocked', 'invalid_input')
