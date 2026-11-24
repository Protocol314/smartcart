"""합성 추종 결과를 좌우 가상 듀티로 연결한다. 실제 GPIO나 모터 서비스는 열지 않는다."""
from smartcart_motor.control import mix
from .follow_preview import FollowPreviewConfig, simulate


class ReplayMotorBridge:
    """재생 전용 계산. 입력 고장 뒤에는 명시적으로 다시 시작해야 한다."""

    def __init__(self, config=None, max_duty=17.0):
        self.config = FollowPreviewConfig() if config is None else config
        if not isinstance(self.config, FollowPreviewConfig):
            raise ValueError('합성 추종 설정이 필요합니다')
        if type(max_duty) not in (int, float) or not 0 < max_duty <= 30:
            raise ValueError('가상 최대 듀티가 잘못되었습니다')
        self.max_duty = float(max_duty)
        self.latched = False
        self.latch_reason = None

    def reset(self):
        """합성 시험의 재시작 요청. 실제 장치의 구동 승인이 아니다."""
        self.latched = False
        self.latch_reason = None

    def evaluate(self, track, clearance, now_ns):
        result = simulate(track, clearance, now_ns, self.config)
        if result['state'] == 'blocked' and not self.latched:
            self.latched = True
            self.latch_reason = result['reason']
        forward = result['preview_forward_fraction']
        turn = result['preview_turn_fraction']
        if self.latched:
            forward = turn = 0.0
        left, right = mix(forward, turn)
        return {'schema': 'smartcart.replay_motor_preview.v1',
                'state': 'latched' if self.latched else result['state'],
                'reason': self.latch_reason if self.latched else result['reason'],
                'left_preview_duty': left * self.max_duty,
                'right_preview_duty': right * self.max_duty,
                'synthetic_only': True, 'drive_authorized': False}
