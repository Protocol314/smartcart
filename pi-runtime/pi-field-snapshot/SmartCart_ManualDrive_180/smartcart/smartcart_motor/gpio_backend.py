"""IBT-2 GPIO 출력 계층. 외부에서 안전 상태를 검증한 MotorController만 사용한다."""
from .control import MotorConfig, SafetyFrame, _number
from smartcart_runner.supervisor import InstanceLock

LEFT = {'ren': 23, 'len': 24, 'rpwm': 12, 'lpwm': 13}
RIGHT = {'ren': 16, 'len': 26, 'rpwm': 18, 'lpwm': 19}
PINS = tuple(LEFT.values()) + tuple(RIGHT.values())


class MotorLock(InstanceLock):
    """기존 소유권·경로 검사를 재사용해 새 모터 제어기 간 중복 실행을 막는다."""
    def __init__(self):
        super().__init__(8765)
        self.path = self.directory / 'motor.lock'


class GpioBackend:
    """기본 생성은 장치를 열지 않는다. 안전 확인 후 start가 LOW부터 초기화한다."""
    def __init__(self, config, *, gpio=None, ownership=None):
        if not isinstance(config, MotorConfig):
            raise ValueError('검증된 모터 설정이 필요합니다')
        self.config, self.gpio = config, gpio
        self._ownership = MotorLock() if ownership is None else ownership
        self._owns = False
        self._configured, self._pwm = [], {}
        self._ready = self._fault = False
        self._signs = [0, 0]

    def _check_safety(self, safety, now_ns):
        if not isinstance(safety, SafetyFrame) or not safety.permits(now_ns, self.config.safety_timeout_ms):
            self.stop()
            raise ValueError('유효한 안전 상태가 없어 GPIO 출력을 잠갔습니다')

    def start(self, safety, now_ns):
        self._check_safety(safety, now_ns)
        if self._fault:
            raise RuntimeError('GPIO 오류 이후 새 제어기를 준비해야 합니다')
        if self._ready:
            return
        if not self.config.polarity_known:
            raise ValueError('좌우 전진 극성을 실물에서 확인해야 합니다')
        try:
            self._ownership.__enter__()
            self._owns = True
            if self.gpio is None:
                import RPi.GPIO as gpio
                self.gpio = gpio
            self.gpio.setmode(self.gpio.BCM)
            for pin in PINS:
                self.gpio.setup(pin, self.gpio.OUT, initial=self.gpio.LOW)
                self._configured.append(pin)
            for pin in (LEFT['rpwm'], LEFT['lpwm'], RIGHT['rpwm'], RIGHT['lpwm']):
                self._pwm[pin] = self.gpio.PWM(pin, 1000)
                self._pwm[pin].start(0)
            self._ready = True
        except Exception:
            self._fault = True
            self._quiet()
            self._release()
            self._release_ownership()
            raise

    def _quiet(self):
        """일부 호출이 실패해도 나머지 PWM과 활성 핀을 모두 낮추려고 시도한다."""
        errors = []
        for pwm in self._pwm.values():
            try:
                pwm.ChangeDutyCycle(0)
            except Exception as error:
                errors.append(error)
        if self.gpio is not None:
            for pin in (LEFT['ren'], LEFT['len'], RIGHT['ren'], RIGHT['len']):
                if pin in self._configured:
                    try:
                        self.gpio.output(pin, self.gpio.LOW)
                    except Exception as error:
                        errors.append(error)
        self._signs = [0, 0]
        return errors

    def stop(self):
        errors = self._quiet()
        if errors:
            self._fault = True
            raise RuntimeError('GPIO 정지 출력에 실패했습니다. 독립 전원 차단이 필요합니다') from errors[0]

    def _release(self):
        errors = []
        for pwm in self._pwm.values():
            try:
                pwm.stop()
            except Exception as error:
                errors.append(error)
        if self.gpio is not None and self._configured:
            try:
                self.gpio.cleanup(list(self._configured))
            except Exception as error:
                errors.append(error)
        self._configured, self._pwm, self._ready = [], {}, False
        return errors

    def _release_ownership(self):
        if self._owns:
            self._ownership.__exit__(None, None, None)
            self._owns = False

    def write(self, left, right, safety, now_ns):
        if not self._ready or self._fault:
            raise RuntimeError('안전 조건을 통과한 GPIO 초기화가 필요합니다')
        try:
            self._check_safety(safety, now_ns)
            values = (_number(left, '왼쪽 듀티', -self.config.max_duty, self.config.max_duty),
                      _number(right, '오른쪽 듀티', -self.config.max_duty, self.config.max_duty))
            if any(abs(value) > self.config.max_duty * safety.speed_limit for value in values):
                raise ValueError('안전 속도 한도를 초과했습니다')
            signs = [1 if value > 0 else -1 if value < 0 else 0 for value in values]
            if any(old and new and old != new for old, new in zip(self._signs, signs)):
                raise ValueError('방향 전환 전에 해당 바퀴 출력을 0으로 내려야 합니다')
            for index, (pins, forward_rpwm, value) in enumerate((
                    (LEFT, self.config.left_forward_rpwm, values[0]),
                    (RIGHT, self.config.right_forward_rpwm, values[1]))):
                if value == 0:
                    self._pwm[pins['rpwm']].ChangeDutyCycle(0)
                    self._pwm[pins['lpwm']].ChangeDutyCycle(0)
                    self.gpio.output(pins['ren'], self.gpio.LOW)
                    self.gpio.output(pins['len'], self.gpio.LOW)
                else:
                    use_rpwm = forward_rpwm if value > 0 else not forward_rpwm
                    active = pins['rpwm'] if use_rpwm else pins['lpwm']
                    inactive = pins['lpwm'] if use_rpwm else pins['rpwm']
                    self._pwm[inactive].ChangeDutyCycle(0)
                    self._pwm[active].ChangeDutyCycle(abs(value))
                    self.gpio.output(pins['ren'], self.gpio.HIGH)
                    self.gpio.output(pins['len'], self.gpio.HIGH)
                self._signs[index] = signs[index]
        except Exception:
            self._fault = True
            self._quiet()
            raise

    def close(self):
        try:
            errors = self._quiet()
            errors.extend(self._release())
        finally:
            self._release_ownership()
        if errors:
            self._fault = True
            raise RuntimeError('GPIO 종료에 실패했습니다. 독립 전원 차단이 필요합니다') from errors[0]
