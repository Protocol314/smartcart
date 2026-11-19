"""Manual-only drive path for the reported Pi 4 / fixed-5V EN wiring.

No synthetic SafetyFrame is produced. Sensor tracking/automatic driving is not
connected here. PWM zero is a software request, not motor-power isolation.
"""
from __future__ import annotations

import math
import threading
import time

PINS = (17, 27, 23, 24)
MAX_PWM = 180
COMMAND_TIMEOUT = 0.35
LOOP_TIMEOUT = 0.20
REVERSE_WAIT = 0.50


def number(value, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError("명령 숫자 범위가 잘못되었습니다")
    return float(value)


class Ramp:
    def __init__(self):
        self.value = [0.0, 0.0]
        self.last_sign = [0, 0]
        self.zero_at = -math.inf

    def stop(self, now):
        if any(self.value):
            self.zero_at = now
        self.value = [0.0, 0.0]

    def targets(self, forward, turn, pwm):
        if abs(forward) < 0.08:
            if abs(turn) < 0.08:
                return [0.0, 0.0]
            # Positive turn is left: left reverses, right advances.
            # Keep pivot below the existing reverse limit (PWM 70).
            pivot = min(pwm, 70) * turn
            return [-pivot, pivot]
        base = min(pwm, 70 if forward < 0 else MAX_PWM)
        turning = abs(turn) >= 0.08
        if turning:
            base = min(base, 80)
        base *= abs(forward)
        signed = math.copysign(base, forward)
        # Straight deceleration first; apply differential steering below PWM 80.
        yaw = turn * base * 0.75 if turning and max(map(abs, self.value)) <= 80.001 else 0
        return [max(-base, min(base, signed - yaw)),
                max(-base, min(base, signed + yaw))]

    def step(self, forward, turn, pwm, dt, now):
        target = self.targets(forward, turn, pwm)
        reversing = any(target[i] * self.last_sign[i] < 0 for i in (0, 1))
        if reversing and (any(self.value) or now - self.zero_at < REVERSE_WAIT):
            target = [0.0, 0.0]
        delta = [target[i] - self.value[i] for i in (0, 1)]
        increasing = any(abs(target[i]) > abs(self.value[i]) for i in (0, 1))
        rate = 30.0 if increasing else 90.0
        largest = max(map(abs, delta))
        factor = min(1.0, rate * dt / largest) if largest else 1.0
        moving = any(self.value)
        self.value = [self.value[i] + delta[i] * factor for i in (0, 1)]
        self.value = [0.0 if abs(v) < 1e-6 else v for v in self.value]
        if moving and not any(self.value):
            self.zero_at = now
        for i, value in enumerate(self.value):
            if value:
                self.last_sign[i] = 1 if value > 0 else -1
        return tuple(round(v) for v in self.value)


class MockBackend:
    hardware = False

    def __init__(self):
        self.values = (0, 0)

    def open(self):
        self.stop()

    def write(self, left, right):
        self.values = (left, right)

    def stop(self):
        self.values = (0, 0)

    def close(self):
        self.stop()


class RPiBackend:
    """Only BCM 17/27/23/24. BCM22 and all EN pins are never opened."""
    hardware = True

    def __init__(self, directions=(-1, -1), gpio=None, ownership=None):
        self.directions = tuple(directions)
        if len(self.directions) != 2 or any(v not in (-1, 1) for v in self.directions):
            raise ValueError("모터 방향은 각각 -1 또는 1이어야 합니다")
        self.gpio = gpio
        self.ownership = ownership
        self.owns = False
        self.pwm = {}
        self.configured = []
        self.signs = [0, 0]

    def open(self):
        if self.ownership is None:
            # Share the existing project's motor lock.
            from .gpio_backend import MotorLock
            self.ownership = MotorLock()
        self.ownership.__enter__()
        self.owns = True
        try:
            if self.gpio is None:
                import RPi.GPIO as gpio
                self.gpio = gpio
            self.gpio.setmode(self.gpio.BCM)
            for pin in PINS:
                self.gpio.setup(pin, self.gpio.OUT, initial=self.gpio.LOW)
                self.configured.append(pin)
                pwm = self.gpio.PWM(pin, 1000)
                self.pwm[pin] = pwm
                pwm.start(0)
        except Exception:
            self.close()
            raise

    def stop(self):
        errors = []
        for pwm in self.pwm.values():
            try:
                pwm.ChangeDutyCycle(0)
            except Exception as exc:
                errors.append(exc)
        self.signs = [0, 0]
        if errors:
            raise RuntimeError("PWM 정지 실패: 물리 전원 차단이 필요합니다") from errors[0]

    def write(self, left, right):
        values = [number(v, -MAX_PWM, MAX_PWM) * d
                  for v, d in zip((left, right), self.directions)]
        signs = [1 if v > 0 else -1 if v < 0 else 0 for v in values]
        if any(a and b and a != b for a, b in zip(self.signs, signs)):
            self.stop()
            raise RuntimeError("0 출력 없이 방향이 전환되었습니다")
        try:
            for index, value in enumerate(values):
                rpwm, lpwm = PINS[index*2:index*2+2]
                active, inactive = (rpwm, lpwm) if value >= 0 else (lpwm, rpwm)
                self.pwm[inactive].ChangeDutyCycle(0)
                self.pwm[active].ChangeDutyCycle(abs(value) * 100 / 255)
            self.signs = signs
        except Exception:
            self.stop()
            raise

    def close(self):
        errors = []
        try:
            self.stop()
        except Exception as exc:
            errors.append(exc)
        for pwm in self.pwm.values():
            try:
                pwm.stop()
            except Exception as exc:
                errors.append(exc)
        if self.gpio is not None and self.configured:
            try:
                self.gpio.cleanup(self.configured)
            except Exception as exc:
                errors.append(exc)
        self.pwm.clear()
        self.configured.clear()
        if self.owns:
            self.ownership.__exit__(None, None, None)
            self.owns = False
        if errors:
            raise RuntimeError("GPIO 종료 실패: 물리 전원 차단이 필요합니다") from errors[0]


class ManualDrive:
    """A dedicated watchdog thread; HTTP/SSE cannot block its normal deadline checks."""
    def __init__(self, backend, *, clock=time.monotonic):
        self.backend = backend
        self.clock = clock
        self.lock = threading.Lock()
        self.quit = threading.Event()
        self.ramp = Ramp()
        self.thread = None
        self.owner = None
        self.armed = False
        self.reason = "disarmed"
        self.fault = None
        self.epoch = 0
        self.sequence = -1
        self.last_command = -math.inf
        self.previous_tick = None
        self.forward = self.turn = 0.0
        self.pwm = 50.0
        self.output = (0, 0)

    def start(self):
        self.backend.open()
        self.thread = threading.Thread(target=self._run, name="cart-pwm-watchdog", daemon=True)
        self.thread.start()

    def acquire(self, owner):
        with self.lock:
            if self.owner is not None or self.fault:
                return False
            self.owner = owner
            self.sequence = -1
            self._stop("connected", self.clock())
            return True

    def release(self, owner):
        with self.lock:
            if self.owner == owner:
                self._stop("disconnected", self.clock())
                self.owner = None

    def _stop(self, reason, now):
        self.armed = False
        self.reason = reason
        self.epoch += 1
        self.forward = self.turn = 0.0
        self.ramp.stop(now)
        self.output = (0, 0)
        try:
            self.backend.stop()
        except Exception as exc:
            self.fault = str(exc)
            self.reason = "gpio_fault"

    def _expire(self, now):
        if self.armed and now - self.last_command >= COMMAND_TIMEOUT:
            self._stop("command_timeout", now)

    def command(self, owner, data):
        now = self.clock()
        with self.lock:
            if owner != self.owner:
                raise ValueError("조종 세션이 아닙니다")
            self._expire(now)
            if not isinstance(data, dict):
                self._stop("invalid_command", now)
                raise ValueError("명령은 JSON 객체여야 합니다")
            kind = data.get("type")
            if kind == "stop":
                self._stop("software_stop", now)
                return
            try:
                stamp = number(data["stamp"], 0, now)
                if now - stamp >= COMMAND_TIMEOUT:
                    raise ValueError("오래된 명령입니다")
                sequence = data["seq"]
                if type(sequence) is not int or not 0 <= sequence < 2**53 or sequence <= self.sequence:
                    raise ValueError("명령 순번이 잘못되었습니다")
                if type(data["epoch"]) is not int or data["epoch"] != self.epoch:
                    raise ValueError("정지 이후 이전 명령을 거부합니다")
                f = number(data["forward"], -1, 1)
                t = number(data["turn"], -1, 1)
                pwm = number(data["pwm"], 0, MAX_PWM)
                if kind not in ("arm", "drive"):
                    raise ValueError("지원하지 않는 명령입니다")
                if self.fault:
                    raise ValueError("GPIO 오류 이후 서버를 다시 시작해야 합니다")
                self.sequence = sequence
                if kind == "arm":
                    if self.armed or f or t or any(self.output):
                        raise ValueError("방향 버튼을 놓고 활성화하십시오")
                    self.armed, self.reason = True, "armed"
                    self.last_command = stamp
                    self.forward = self.turn = 0.0
                    self.pwm = pwm
                elif self.armed:
                    self.forward, self.turn, self.pwm = f, t, pwm
                    self.last_command = stamp
            except (KeyError, TypeError, ValueError):
                self._stop("invalid_command", now)
                raise ValueError("명령 오류 또는 만료: 다시 연결·활성화하십시오") from None

    def tick(self, now=None):
        now = self.clock() if now is None else now
        with self.lock:
            dt = 0.02 if self.previous_tick is None else now - self.previous_tick
            self.previous_tick = now
            if self.armed and (dt < 0 or dt > LOOP_TIMEOUT):
                self._stop("loop_delay", now)
            self._expire(now)
            if self.armed:
                try:
                    self.output = self.ramp.step(
                        self.forward, self.turn, self.pwm, min(dt, 0.04), now)
                    self.backend.write(*self.output)
                except Exception as exc:
                    self.fault = str(exc)
                    self._stop("gpio_fault", now)

    def _run(self):
        while not self.quit.is_set():
            self.tick()
            self.quit.wait(0.02)

    def snapshot(self):
        with self.lock:
            left, right = self.output
            rpm = [round(v / 255 * 160, 1) for v in self.output]
            speed = abs(sum(rpm) / 2) * math.pi * 0.1524 * 60 / 1000
            return {
                "schema": "smartcart.manual.v1", "hardware": self.backend.hardware,
                "armed": self.armed, "reason": self.reason, "fault": self.fault,
                "epoch": self.epoch, "ack_seq": self.sequence,
                "stamp": self.clock(), "pwm": [left, right], "selected_pwm": self.pwm,
                "estimated_rpm": rpm, "estimated_kmh": round(speed, 2),
                "measured_rpm": None, "sensor_interlock": False,
                "en_controlled": False,
            }

    def close(self):
        self.quit.set()
        if self.thread:
            self.thread.join(timeout=1)
        with self.lock:
            self._stop("closed", self.clock())
            self.backend.close()
