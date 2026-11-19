"""GPIO 루프백의 핀 사용·실패 시 정리를 가상 장치로 검증한다."""
import importlib.util
from pathlib import Path
import unittest

source = Path(__file__).resolve().parents[1] / 'scripts/gpio_loopback_check.py'
spec = importlib.util.spec_from_file_location('gpio_loopback_check', source)
loopback = importlib.util.module_from_spec(spec)
spec.loader.exec_module(loopback)


class FakeGPIO:
    BCM, IN, OUT, PUD_DOWN, LOW, HIGH = 11, 1, 2, 3, 0, 1
    def __init__(self, *, failure=False):
        self.value = 0
        self.pins = []
        self.cleaned = []
        self.failure = failure
    def setmode(self, mode):
        self.mode = mode
    def setup(self, pin, mode, **kwargs):
        self.pins.append(pin)
    def output(self, pin, value):
        self.value = value
    def input(self, pin):
        if self.failure:
            raise OSError('가상 입력 오류')
        return self.value
    def cleanup(self, pins):
        self.cleaned = list(pins)


class LoopbackTests(unittest.TestCase):
    def test_output_input_and_scoped_cleanup(self):
        gpio = FakeGPIO()
        self.assertEqual(loopback.check(gpio, sleep=lambda _: None), [(0, 0), (1, 1), (0, 0)])
        self.assertEqual(gpio.pins, [6, 5])
        self.assertEqual(gpio.cleaned, [6, 5])
        self.assertEqual(gpio.value, 0)

    def test_input_error_still_sets_low_and_cleans_up(self):
        gpio = FakeGPIO(failure=True)
        with self.assertRaises(OSError):
            loopback.check(gpio, sleep=lambda _: None)
        self.assertEqual(gpio.value, 0)
        self.assertEqual(gpio.cleaned, [6, 5])


if __name__ == '__main__':
    unittest.main()
