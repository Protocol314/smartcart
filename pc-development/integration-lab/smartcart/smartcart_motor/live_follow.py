"""검증된 실측 상태만 모터 서비스에 전달하는 연결부. 자체적으로 무장하거나 GPIO를 열지 않는다."""
from dataclasses import dataclass, replace
import math

from .control import SafetyFrame, _number, _time
from .lidar_clearance import ClearanceConfig, evaluate_clearance
from .service import MotorService


@dataclass(frozen=True)
class LiveFollowConfig:
    """거리는 현장 검수 전까지 지정하지 않는다. 비율은 실측 속도가 아니다."""
    follow_distance_m: float
    stop_distance_m: float
    max_error_m: float
    max_forward: float = .2
    max_turn: float = .2
    pivot_angle_deg: float = 35.0

    def __post_init__(self):
        for name, low, high in (('follow_distance_m', .2, 5), ('stop_distance_m', .1, 4),
                                ('max_error_m', .01, 2), ('max_forward', .01, .5),
                                ('max_turn', .01, .5), ('pivot_angle_deg', 5, 90)):
            _number(getattr(self, name), name, low, high)
        if self.stop_distance_m >= self.follow_distance_m:
            raise ValueError('정지 거리는 추종 거리보다 작아야 합니다')


class LiveFollow:
    """호출자는 단일 스레드에서 최신 입력을 공급한다. 서비스 자체의 주기 타이머도 필요하다."""
    def __init__(self, service, follow, clearance):
        if not isinstance(service, MotorService) or not isinstance(follow, LiveFollowConfig) or not isinstance(clearance, ClearanceConfig):
            raise ValueError('모터 서비스와 검증 설정이 필요합니다')
        self.service, self.follow, self.clearance = service, follow, clearance
        self.last_sequence = None

    def update(self, track, lidar_observation, safety, now_ns):
        """정지 사유가 있으면 안전 상태를 폐기한다. 무장은 별도 명시적 요청이다."""
        def block(reason):
            self.service.invalidate()
            return {'accepted': False, 'reason': reason}
        try:
            now = _time(now_ns)
            if not isinstance(safety, SafetyFrame) or not safety.permits(now, self.service.controller.config.safety_timeout_ms):
                return block('safety_unavailable')
            if safety.control_mode != 'follow':
                # 수동용 상태는 추적 유효를 보증하지 않으므로 추종에 쓰지 않는다.
                return block('safety_mode_mismatch')
            if (not isinstance(track, dict) or track.get('schema') != 'smartcart.target_track.v1'
                    or track.get('source_mode') != 'live' or track.get('frame_id') != 'base_link'
                    or track.get('valid') is not True or track.get('state') != 'tracking'
                    or track.get('provisional') is not False or track.get('drive_authorized') is not True
                    or type(track.get('confirmation_count')) is not int or track['confirmation_count'] < 3
                    or type(track.get('candidate_count')) is not int or track['candidate_count'] != 1
                    or type(track.get('associated_cluster_id')) is not int or track['associated_cluster_id'] < 0):
                return block('tracking_unavailable')
            reference = _time(track.get('reference_monotonic_ns'))
            deadline = _time(track.get('valid_until_monotonic_ns'))
            sequence = track.get('track_sequence')
            if (reference > now or now >= deadline or now - reference >= 300_000_000
                    or type(sequence) is not int or not 0 <= sequence < (1 << 53)):
                return block('tracking_stale')
            if self.last_sequence is not None and sequence < self.last_sequence:
                return block('tracking_sequence')
            error = _number(track.get('error_radius_m'), '추적 오차', 0, 100)
            point = track.get('position_xy_m')
            if error > self.follow.max_error_m or type(point) not in (list, tuple) or len(point) != 2:
                return block('tracking_uncertain')
            x, y = (_number(v, '추적 위치', -100, 100) for v in point)
            if not isinstance(lidar_observation, dict) or lidar_observation.get('source_mode') != 'live':
                return block('lidar_not_live')
            clear = evaluate_clearance(lidar_observation, now, self.clearance)
            if not clear['valid'] or not clear['clear']:
                return block(clear['reason'])
            limited = replace(safety, tracking_until_ns=min(safety.tracking_until_ns, deadline),
                              clearance_until_ns=min(safety.clearance_until_ns, clear['valid_until_ns']))
            if not limited.permits(now, self.service.controller.config.safety_timeout_ms):
                return block('safety_expired')
            # 같은 추적 표본은 명령·안전 상태의 수명을 늘리지 않는다.
            # 다만 위에서 새 LiDAR 및 안전 상태 오류는 먼저 차단했다.
            if sequence == self.last_sequence:
                return {'accepted': False, 'reason': 'no_new_track'}
            distance, angle = math.hypot(x, y), math.atan2(y, x)
            forward = turn = 0.0
            if distance > self.follow.stop_distance_m:
                if abs(math.degrees(angle)) >= self.follow.pivot_angle_deg:
                    turn = math.copysign(self.follow.max_turn, angle)
                else:
                    forward = min(self.follow.max_forward, max(0.0, (distance - self.follow.follow_distance_m) * .2))
                    turn = max(-self.follow.max_turn, min(self.follow.max_turn, angle * .3))
            self.service.update_safety(limited)
            self.service.submit(forward, turn, sequence)
            self.last_sequence = sequence
            return {'accepted': True, 'reason': None}
        except (ValueError, TypeError, OverflowError):
            return block('invalid_input')
        except Exception:
            self.service.invalidate()
            raise
