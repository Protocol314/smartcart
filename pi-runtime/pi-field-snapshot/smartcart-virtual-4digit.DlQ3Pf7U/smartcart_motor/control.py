"""입력 수명과 안전 잠금을 지키는 차동 구동. 실제 속도 피드백은 포함하지 않는다."""
from dataclasses import dataclass
import math

NS = 1_000_000_000


def _number(value, name, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(name + ' 범위가 잘못되었습니다')
    return float(value)


def _time(value):
    if type(value) is not int or not 0 <= value < (1 << 63):
        raise ValueError('단조 시각은 음수가 아닌 63비트 정수여야 합니다')
    return value


def _flag(value, name):
    if type(value) is not bool:
        raise ValueError(name + '은 참 또는 거짓이어야 합니다')
    return value


def mix(forward, turn):
    """전진 양수, 좌회전 양수. 좌·우 비율을 보존하며 명령을 [-1, 1]로 제한한다."""
    forward = _number(forward, '전진 명령', -1, 1)
    turn = _number(turn, '회전 명령', -1, 1)
    left, right = forward - turn, forward + turn
    scale = max(1.0, abs(left), abs(right))
    return left / scale, right / scale


@dataclass(frozen=True)
class MotorConfig:
    """전진 시 각 물리 드라이버가 RPWM을 사용하는지 실물 확인 전에는 None이다."""
    left_forward_rpwm: bool | None = None
    right_forward_rpwm: bool | None = None
    polarity_verified: bool = False
    max_duty: float = 17.0
    slew_per_second: float = 20.0
    reverse_deadtime_ms: int = 500
    command_timeout_ms: int = 300
    safety_timeout_ms: int = 300

    def __post_init__(self):
        for name in ('left_forward_rpwm', 'right_forward_rpwm'):
            value = getattr(self, name)
            if value is not None:
                _flag(value, name)
        _flag(self.polarity_verified, '전진 극성 실물 확인')
        _number(self.max_duty, '최대 PWM 듀티', .1, 30)
        _number(self.slew_per_second, '듀티 변화율', 1, 100)
        for name, low, high in (('reverse_deadtime_ms', 200, 5000),
                                ('command_timeout_ms', 50, 1000), ('safety_timeout_ms', 50, 1000)):
            value = getattr(self, name)
            if type(value) is not int or not low <= value <= high:
                raise ValueError(name + ' 범위가 잘못되었습니다')

    @property
    def polarity_known(self):
        return (self.polarity_verified and self.left_forward_rpwm is not None
                and self.right_forward_rpwm is not None)


@dataclass(frozen=True)
class SafetyFrame:
    """독립 안전 계층이 만든 실측 상태. 각 기한은 해당 센서의 원래 수명이다."""
    source_mode: str
    drive_authorized: bool
    provisional: bool
    estop_feedback_valid: bool
    estop_released: bool
    uwb_left_valid: bool
    uwb_right_valid: bool
    lidar_valid: bool
    tracking_valid: bool
    clearance_valid: bool
    clearance_ok: bool
    speed_limit: float
    observed_ns: int
    uwb_left_until_ns: int
    uwb_right_until_ns: int
    lidar_until_ns: int
    tracking_until_ns: int
    clearance_until_ns: int
    uwb_profile: str = 'dual'

    def __post_init__(self):
        if self.source_mode not in ('live', 'replay'):
            raise ValueError('안전 상태 출처가 잘못되었습니다')
        if self.uwb_profile not in ('dual', 'single_left'):
            raise ValueError('UWB 안전 구성은 dual 또는 single_left여야 합니다')
        for name in ('drive_authorized', 'provisional', 'estop_feedback_valid', 'estop_released',
                     'uwb_left_valid', 'uwb_right_valid', 'lidar_valid', 'tracking_valid',
                     'clearance_valid', 'clearance_ok'):
            _flag(getattr(self, name), name)
        _number(self.speed_limit, '안전 속도 한도', 0, 1)
        for name in ('observed_ns', 'uwb_left_until_ns', 'uwb_right_until_ns',
                     'lidar_until_ns', 'tracking_until_ns', 'clearance_until_ns'):
            _time(getattr(self, name))

    def permits(self, now_ns, timeout_ms):
        now_ns = _time(now_ns)
        if type(timeout_ms) is not int or not 50 <= timeout_ms <= 1000:
            raise ValueError('안전 상태 유효 시간이 잘못되었습니다')
        flags = (self.drive_authorized, not self.provisional, self.estop_feedback_valid,
                 self.estop_released, self.uwb_left_valid,
                 self.lidar_valid, self.tracking_valid, self.clearance_valid, self.clearance_ok)
        deadlines = (self.uwb_left_until_ns, self.lidar_until_ns,
                     self.tracking_until_ns, self.clearance_until_ns)
        if self.uwb_profile == 'dual':
            flags += (self.uwb_right_valid,)
            deadlines += (self.uwb_right_until_ns,)
        return (self.source_mode == 'live' and all(flags) and self.speed_limit > 0 and self.observed_ns <= now_ns
                and now_ns - self.observed_ns < timeout_ms * 1_000_000
                and all(now_ns < deadline <= self.observed_ns + 2 * NS for deadline in deadlines))


class MotorController:
    """명시적 재무장·원래 센서 수명·명령 타임아웃을 모두 요구한다.

    호출자는 독립 타이머에서 step을 주기적으로 실행해야 한다. Python이 멈추거나 전원이
    끊길 때의 출력 차단은 별도 하드웨어가 담당한다. 이 클래스는 GPIO를 직접 열지 않는다.
    """
    def __init__(self, config, backend):
        if not isinstance(config, MotorConfig):
            raise ValueError('검증된 모터 설정이 필요합니다')
        if hasattr(backend, 'config') and backend.config != config:
            raise ValueError('제어기와 GPIO 설정이 일치해야 합니다')
        self.config, self.backend = config, backend
        self.armed = False
        self.reason = 'disarmed'
        self._now = self._last_clock = self._deadline = self._sequence = None
        self._target = (0.0, 0.0)
        self._current = [0.0, 0.0]
        self._last_sign = [0, 0]
        self._zero_since = [None, None]

    def _clock(self, now_ns):
        now_ns = _time(now_ns)
        if self._last_clock is not None and now_ns < self._last_clock:
            raise ValueError('모터 제어 시계가 역행했습니다')
        self._last_clock = now_ns
        return now_ns

    def _safe(self, safety, now_ns):
        return isinstance(safety, SafetyFrame) and safety.permits(now_ns, self.config.safety_timeout_ms)

    def disarm(self, reason='disarmed', *, now_ns=None):
        if now_ns is not None:
            _time(now_ns)
            if self._last_clock is not None:
                now_ns = max(now_ns, self._last_clock)
        self.armed, self.reason = False, reason
        self._target, self._deadline = (0.0, 0.0), None
        for index, duty in enumerate(self._current):
            if duty:
                self._last_sign[index] = 1 if duty > 0 else -1
                self._zero_since[index] = now_ns
        self._current = [0.0, 0.0]
        self.backend.stop()

    def arm(self, safety, now_ns):
        try:
            now_ns = self._clock(now_ns)
            if not self.config.polarity_known or not self._safe(safety, now_ns):
                self.disarm('interlock', now_ns=now_ns)
                return False
            if self.armed:
                raise ValueError('이미 활성화된 모터는 재무장할 수 없습니다')
            self.backend.start(safety, now_ns)
            self.armed, self.reason = True, 'waiting_command'
            self._now, self._sequence = now_ns, None
            self._deadline = now_ns + self.config.command_timeout_ms * 1_000_000
            return True
        except Exception:
            self.disarm('arm_error', now_ns=now_ns if type(now_ns) is int and 0 <= now_ns < (1 << 63) else None)
            raise

    def submit(self, forward, turn, sequence, safety, now_ns, *, issued_ns=None):
        try:
            now_ns = self._clock(now_ns)
            issued_ns = now_ns if issued_ns is None else _time(issued_ns)
            if not self.armed or not self._safe(safety, now_ns):
                self.disarm('interlock', now_ns=now_ns)
                return False
            command_deadline = issued_ns + self.config.command_timeout_ms * 1_000_000
            if (self._deadline is None or now_ns >= self._deadline or issued_ns > now_ns
                    or now_ns >= command_deadline):
                self.disarm('command_timeout', now_ns=now_ns)
                return False
            if type(sequence) is not int or not 0 <= sequence < (1 << 53):
                raise ValueError('명령 순번이 잘못되었습니다')
            if self._sequence is not None and sequence <= self._sequence:
                raise ValueError('반복되거나 역행한 명령을 거부합니다')
            values = mix(forward, turn)
            self._target = tuple(item * self.config.max_duty for item in values)
            self._sequence = sequence
            self._deadline = command_deadline
            return True
        except Exception:
            self.disarm('invalid_command', now_ns=now_ns if type(now_ns) is int and 0 <= now_ns < (1 << 63) else None)
            raise

    def _wheel(self, index, target, now_ns, delta):
        current = self._current[index]
        target_sign = 1 if target > 0 else -1 if target < 0 else 0
        current_sign = 1 if current > 0 else -1 if current < 0 else 0
        # 반대 방향 요청은 먼저 0까지 줄이고, 0 도달 시각부터 정지 구간을 둔다.
        if current_sign and target_sign != current_sign:
            value = max(0.0, abs(current) - delta) * current_sign
            if value == 0:
                self._last_sign[index], self._zero_since[index] = current_sign, now_ns
            return value
        if not current_sign and target_sign and self._last_sign[index] not in (0, target_sign):
            zero_since = self._zero_since[index]
            if zero_since is None or now_ns - zero_since < self.config.reverse_deadtime_ms * 1_000_000:
                return 0.0
        if target_sign == 0:
            return 0.0
        value = min(abs(target), abs(current) + delta) * target_sign if abs(current) < abs(target) else max(abs(target), abs(current) - delta) * target_sign
        if value:
            self._last_sign[index] = target_sign
        return value

    def step(self, safety, now_ns):
        try:
            now_ns = self._clock(now_ns)
            if not self.armed:
                return (0.0, 0.0)
            if not self._safe(safety, now_ns):
                self.disarm('interlock', now_ns=now_ns)
                return (0.0, 0.0)
            if self._deadline is None or now_ns >= self._deadline:
                self.disarm('command_timeout', now_ns=now_ns)
                return (0.0, 0.0)
            elapsed = min(now_ns - self._now, 100_000_000) / NS
            delta = self.config.slew_per_second * elapsed
            limit = self.config.max_duty * safety.speed_limit
            output = tuple(max(-limit, min(limit, self._wheel(index, target * safety.speed_limit, now_ns, delta)))
                           for index, target in enumerate(self._target))
            self.backend.write(*output, safety, now_ns)
            self._current, self._now, self.reason = list(output), now_ns, 'active'
            return output
        except Exception:
            self.disarm('control_error', now_ns=now_ns if type(now_ns) is int and 0 <= now_ns < (1 << 63) else None)
            raise

    def close(self, now_ns=None):
        try:
            self.disarm('closed', now_ns=now_ns)
        finally:
            self.backend.close()
