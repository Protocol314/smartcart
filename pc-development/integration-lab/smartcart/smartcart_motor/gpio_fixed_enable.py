"""5V 고정 EN 배선을 위한 GPIO 출력. 별도 안전 상태 없이는 시작하지 않는다.

EN을 GPIO로 낮출 수 없으므로 PWM 0을 시도할 뿐이다. 전원 차단·재시작
방지와 비상정지 피드백은 독립 하드웨어가 맡아야 한다. 이 모듈을 import해도
GPIO를 열지 않는다.
"""
from .control import _number
from .gpio_backend import GpioBackend

LEFT_PWM = {'rpwm': 17, 'lpwm': 27}
RIGHT_PWM = {'rpwm': 23, 'lpwm': 24}
PWM_PINS = (17, 27, 23, 24)


class FixedEnableGpioBackend(GpioBackend):
    """현장 배선에 대한 출력부. 외부 전원 차단의 독립 검수가 선행돼야 한다."""

    def __init__(self, config, *, cutoff_verified=False, gpio=None, ownership=None):
        super().__init__(config, gpio=gpio, ownership=ownership)
        if type(cutoff_verified) is not bool:
            raise ValueError('외부 차단 경로 검수 값이 잘못되었습니다')
        self.cutoff_verified = cutoff_verified

    def start(self, safety, now_ns):
        self._check_safety(safety, now_ns)
        if not self.cutoff_verified:
            raise ValueError('독립 전원 차단 및 피드백 검수가 필요합니다')
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
            for pin in PWM_PINS:
                self.gpio.setup(pin, self.gpio.OUT, initial=self.gpio.LOW)
                self._configured.append(pin)
            for pin in PWM_PINS:
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
        errors = []
        for pwm in self._pwm.values():
            try:
                pwm.ChangeDutyCycle(0)
            except Exception as error:
                errors.append(error)
        self._signs = [0, 0]
        return errors

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
                    (LEFT_PWM, self.config.left_forward_rpwm, values[0]),
                    (RIGHT_PWM, self.config.right_forward_rpwm, values[1]))):
                use_rpwm = forward_rpwm if value > 0 else not forward_rpwm
                active = pins['rpwm'] if use_rpwm else pins['lpwm']
                inactive = pins['lpwm'] if use_rpwm else pins['rpwm']
                self._pwm[inactive].ChangeDutyCycle(0)
                self._pwm[active].ChangeDutyCycle(abs(value))
                self._signs[index] = signs[index]
        except Exception:
            self._fault = True
            self._quiet()
            raise
