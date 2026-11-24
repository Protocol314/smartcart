"""현재 보고된 PWM 핀의 가상 출력 검사. 실제 핀은 열지 않는다."""
from dataclasses import replace
import unittest

from smartcart_motor.control import MotorConfig
from smartcart_motor.gpio_fixed_enable import FixedEnableGpioBackend
from test_motor_control import FakeGPIO, SECOND, safety


class Ownership:
    def __enter__(self):
        return self
    def __exit__(self, *_):
        pass


class FixedEnableTests(unittest.TestCase):
    def setUp(self):
        self.gpio = FakeGPIO()
        self.config = MotorConfig(left_forward_rpwm=True, right_forward_rpwm=True,
                                  polarity_verified=True)

    def backend(self, verified=False, config=None):
        return FixedEnableGpioBackend(config or self.config, cutoff_verified=verified,
                                      gpio=self.gpio, ownership=Ownership())

    def test_default_and_current_missing_feedback_do_not_open_gpio(self):
        backend = self.backend()
        with self.assertRaises(ValueError):
            backend.start(safety(), SECOND)
        self.assertEqual(self.gpio.events, [])
        backend = self.backend(True)
        with self.assertRaises(ValueError):
            backend.start(safety(estop_feedback_valid=False), SECOND)
        self.assertEqual(self.gpio.events, [])
        backend = self.backend(True, MotorConfig())
        with self.assertRaises(ValueError):
            backend.start(safety(), SECOND)
        self.assertEqual(self.gpio.events, [])

    def test_same_forward_channel_and_both_turns(self):
        cases = ((6, 6, {17, 23}), (-6, -6, {27, 24}),
                 (-6, 6, {27, 23}), (6, -6, {17, 24}))
        for left, right, expected in cases:
            with self.subTest(left=left, right=right):
                self.setUp()
                backend = self.backend(True)
                backend.start(safety(), SECOND)
                backend.write(left, right, safety(), SECOND)
                self.assertEqual({pin for pin, duty in self.gpio.duty.items() if duty}, expected)
                self.assertEqual(set(self.gpio.out), {17, 27, 23, 24})
                backend.stop()
                self.assertTrue(all(value == 0 for value in self.gpio.duty.values()))
                backend.close()

    def test_reverse_without_zero_and_sensor_error_zero_all_outputs(self):
        backend = self.backend(True)
        backend.start(safety(), SECOND)
        backend.write(6, 6, safety(), SECOND)
        with self.assertRaises(ValueError):
            backend.write(-6, 6, safety(), SECOND)
        self.assertTrue(all(value == 0 for value in self.gpio.duty.values()))
        backend.close()
        self.setUp()
        backend = self.backend(True)
        backend.start(safety(), SECOND)
        backend.write(6, 6, safety(), SECOND)
        with self.assertRaises(ValueError):
            backend.write(6, 6, safety(lidar_valid=False), SECOND)
        self.assertTrue(all(value == 0 for value in self.gpio.duty.values()))
        backend.close()

    def test_fault_on_one_pwm_attempts_zero_for_others(self):
        backend = self.backend(True)
        backend.start(safety(), SECOND)
        self.gpio.fail_pin = 23
        with self.assertRaises(OSError):
            backend.write(6, 6, safety(), SECOND)
        self.assertTrue(all(value == 0 for value in self.gpio.duty.values()))
        with self.assertRaises(RuntimeError):
            backend.start(safety(), SECOND)
        backend.close()


if __name__ == '__main__':
    unittest.main()
