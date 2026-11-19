"""검증된 좌우 관측의 최신 수신 상태만 웹에 표시한다. 위치·구동 허가는 만들지 않는다."""
from copy import deepcopy
import math
from threading import RLock
import time

from smartcart_tracking.runtime import _REJECTED_STATUS
from smartcart_uwb.estimator import TagEstimator
from smartcart_uwb.observation import timestamp_ns

SCHEMA = 'smartcart.dual_uwb_view.v1'
ROLES = ('uwb_left', 'uwb_right')


def _bounded(value, minimum, maximum):
    return (type(value) in (int, float) and math.isfinite(value)
            and minimum <= value <= maximum)


class DualUwbStore:
    """기존 전체 상태 검증을 재사용하고 식별자·위치·과거 기록은 웹에 추가하지 않는다."""
    def __init__(self, mounting, settings, *, stop, timeout_ms=750, single_left=False):
        if type(timeout_ms) is not int or not 100 <= timeout_ms <= 2000:
            raise ValueError('좌우 수신 표시 제한은 100~2000ms 정수여야 합니다')
        self.stop = stop
        self._observation_timeout_ns = min(mounting.timeout_ms, 1000) * 1_000_000
        if single_left:
            from smartcart_uwb.single_estimator import SingleLeftEstimator
        self.single_left = single_left
        self.validator = (SingleLeftEstimator if single_left else TagEstimator)(mounting, settings, source_mode='live')
        self._lock = RLock()
        self._timeout_ns = timeout_ms * 1_000_000
        self._status = self._received_ns = self._now_ns = None
        self._state, self._reason, self._sequence = 'waiting', None, 0

    def set_error(self, reason='invalid_dual_status'):
        with self._lock:
            self._status = None
            self._state, self._reason = 'error', reason
            self.validator.reset('invalid_uwb_status')
            self._sequence += 1

    def consume_status(self, status, now_ns):
        with self._lock:
            now_ns = timestamp_ns(now_ns)
            result = self.validator.update(status, now_ns)
            if result['invalid_reason'] in _REJECTED_STATUS:
                self.set_error()
                return
            self._status = deepcopy(status)
            self._received_ns = self._now_ns = now_ns
            self._state, self._reason = 'running', None
            self._sequence += 1

    def snapshot(self, *, now=None, now_ns=None):
        with self._lock:
            current = (int(now * 1e9) if now is not None else
                       time.monotonic_ns() if now_ns is None else timestamp_ns(now_ns))
            timestamp_ns(current)
            if self._now_ns is not None and current < self._now_ns:
                self.set_error('non_monotonic_view_time')
            self._now_ns = current
            age = None if self._received_ns is None else max(0, current - self._received_ns)
            if self._state == 'running' and age >= self._timeout_ns:
                self._state, self._reason = 'stale', 'dual_transport_stale'
                self._status = None
            sensors = {}
            for role in ROLES:
                item = (self._status or {}).get('sensors', {}).get(role)
                latest = item['latest'] if item else None
                sample_age = None if latest is None else max(0, (current - latest['rx_monotonic_ns']) / 1e6)
                deadline = min(latest['valid_until_monotonic_ns'],
                               latest['rx_monotonic_ns'] + self._observation_timeout_ns) if latest else 0
                if latest is not None and not latest['valid']:
                    # 새 무효 프레임의 원시값은 짧게 표시하되 유효 관측의 만료는 바꾸지 않는다.
                    deadline = latest['rx_monotonic_ns'] + self._observation_timeout_ns
                reason = latest['invalid_reason'] if latest else self._reason
                state = self._state if self._state != 'running' else 'waiting'
                if item is not None and not item['connected']:
                    state, reason = 'disconnected', item['last_error'] or 'disconnected'
                elif latest is not None:
                    if current >= deadline or current < latest['rx_monotonic_ns']:
                        state, reason = 'stale', 'stale'
                    else:
                        state = 'live' if latest['valid'] else 'invalid'
                # 체크섬 오류·재연결·단절의 이전 값은 현재 측정으로 표시하지 않는다.
                displayable = (latest is not None and state in ('live', 'invalid')
                               and reason in (None, 'invalid_measurement', 'id_mismatch'))
                distance = latest['range_m'] if displayable else None
                angle = latest['raw_angle_deg'] if displayable else None
                sensors[role] = {
                    'state': state, 'connected': bool(item and item['connected']),
                    'valid': state == 'live', 'invalid_reason': reason,
                    'measurement_valid': bool(displayable and latest['measurement_valid']),
                    'id_valid': bool(displayable and latest['id_valid']),
                    'range_m': distance if _bounded(distance, .05, 20) else None,
                    'raw_angle_deg': angle if _bounded(angle, -180, 180) else None,
                    'range_out_of_bounds': bool(displayable and distance is not None and not _bounded(distance, .05, 20)),
                    'angle_out_of_bounds': bool(displayable and angle is not None and not _bounded(angle, -180, 180)),
                    'age_ms': sample_age,
                    'remaining_ms': max(0, (deadline - current) / 1e6) if displayable else 0,
                    'sample_sequence': latest['sample_sequence'] if latest else None,
                }
            active = ('uwb_left',) if self.single_left else ROLES
            return {'schema': SCHEMA, 'source_mode': 'live', 'state': self._state,
                    'profile': 'single_left' if self.single_left else 'dual',
                    'sequence': self._sequence, 'age_ms': None if age is None else age / 1e6,
                    'remaining_ms': max(0, (self._timeout_ns - (age or 0)) / 1e6) if self._state == 'running' else 0,
                    'valid': all(sensors[role]['valid'] for role in active),
                    'received': self._state == 'running' and all(sensors[role]['age_ms'] is not None
                        and sensors[role]['state'] in ('live', 'invalid') for role in active),
                    'sensors': sensors, 'drive_authorized': False}
