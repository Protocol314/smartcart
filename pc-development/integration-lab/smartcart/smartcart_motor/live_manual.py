"""검증된 로컬 안전 상태와 LiDAR를 통과한 수동 명령만 모터 서비스에 전달한다.

네트워크·GPIO·무장 처리는 포함하지 않는다. 호출자는 별도의 독립 주기에서
MotorService를 실행하고, 검증된 안전 상태를 원래 수명과 함께 제공해야 한다.
"""
from dataclasses import replace

from .control import SafetyFrame, _time, mix
from .lidar_clearance import ClearanceConfig, evaluate_clearance
from .service import MotorService


class LiveManual:
    """센서 장애와 재생·중복 명령에 대해 모터 안전 상태를 폐기한다."""

    def __init__(self, service, clearance):
        if not isinstance(service, MotorService) or not isinstance(clearance, ClearanceConfig):
            raise ValueError('모터 서비스와 검증 설정이 필요합니다')
        self.service = service
        self.clearance = clearance
        self.last_sequence = None

    def stop(self):
        """서비스의 다음 안전 주기에서 정지한다. 하드웨어 비상정지가 아니다."""
        self.service.invalidate()
        return {'accepted': False, 'reason': 'manual_stop'}

    def update(self, forward, turn, sequence, lidar_observation, safety, now_ns):
        def block(reason):
            self.service.invalidate()
            return {'accepted': False, 'reason': reason}

        try:
            now = _time(now_ns)
            mix(forward, turn)
            if type(sequence) is not int or not 0 <= sequence < (1 << 53):
                return block('invalid_sequence')
            if self.last_sequence is not None and sequence <= self.last_sequence:
                return block('repeated_or_reordered_command')
            if not isinstance(safety, SafetyFrame) or not safety.permits(
                    now, self.service.controller.config.safety_timeout_ms):
                return block('safety_unavailable')
            if not isinstance(lidar_observation, dict) or lidar_observation.get('source_mode') != 'live':
                return block('lidar_not_live')
            clear = evaluate_clearance(lidar_observation, now, self.clearance)
            if not clear['valid'] or not clear['clear']:
                return block(clear['reason'])
            limited = replace(
                safety,
                lidar_until_ns=min(safety.lidar_until_ns, clear['valid_until_ns']),
                clearance_until_ns=min(safety.clearance_until_ns, clear['valid_until_ns']),
            )
            if not limited.permits(now, self.service.controller.config.safety_timeout_ms):
                return block('safety_expired')
            self.service.update_safety(limited)
            self.service.submit(forward, turn, sequence)
            self.last_sequence = sequence
            return {'accepted': True, 'reason': None}
        except (TypeError, ValueError, OverflowError):
            return block('invalid_input')
        except Exception:
            self.service.invalidate()
            raise
