"""실제 장치 없이 좌우 방향·출력 순서·안전 잠금·고장을 검증한다."""
from dataclasses import replace
import subprocess
import sys
import threading
import time
from pathlib import Path
import unittest

from smartcart_motor.control import MotorConfig, MotorController, SafetyFrame, mix
from smartcart_motor.gpio_backend import GpioBackend, MotorLock, LEFT, RIGHT
from smartcart_motor.service import MotorService

MS = 1_000_000
SECOND = 1_000_000_000
ROOT = Path(__file__).resolve().parents[1]


def safety(now=SECOND, **overrides):
    values = dict(source_mode='live', drive_authorized=True, provisional=False,
                  estop_feedback_valid=True, estop_released=True,
                  uwb_left_valid=True, uwb_right_valid=True, lidar_valid=True, tracking_valid=True,
                  clearance_valid=True, clearance_ok=True, speed_limit=1.0,
                  observed_ns=now, uwb_left_until_ns=now + 500 * MS,
                  uwb_right_until_ns=now + 500 * MS, lidar_until_ns=now + 300 * MS,
                  tracking_until_ns=now + 300 * MS, clearance_until_ns=now + 300 * MS)
    values.update(overrides)
    return SafetyFrame(**values)


def known_config(**changes):
    return replace(MotorConfig(left_forward_rpwm=True, right_forward_rpwm=False,
                               polarity_verified=True), **changes)


class FakeBackend:
    def __init__(self):
        self.starts = self.stops = self.closes = 0
        self.writes = []
    def start(self, safety, now_ns):
        self.starts += 1
    def write(self, left, right, safety, now_ns):
        self.writes.append((left, right))
    def stop(self):
        self.stops += 1
        self.writes.append((0.0, 0.0))
    def close(self):
        self.closes += 1


class MotorControlTests(unittest.TestCase):
    def setUp(self):
        self.backend = FakeBackend()
        self.controller = MotorController(known_config(), self.backend)

    def test_differential_mix_covers_straight_and_both_tank_turns(self):
        self.assertEqual(mix(1, 0), (1, 1))
        self.assertEqual(mix(-1, 0), (-1, -1))
        self.assertEqual(mix(0, 1), (-1, 1))
        self.assertEqual(mix(0, -1), (1, -1))
        self.assertEqual(mix(.8, .8), (0, 1))
        self.assertEqual(mix(0, 0), (0, 0))

    def test_invalid_numeric_values_and_configuration_are_rejected(self):
        for forward, turn in ((True, 0), (float('nan'), 0), (0, float('inf')), (1.1, 0)):
            with self.subTest(forward=forward, turn=turn), self.assertRaises(ValueError):
                mix(forward, turn)
        for config in ({'max_duty': 100}, {'max_duty': True}, {'reverse_deadtime_ms': 0},
                       {'command_timeout_ms': 0}, {'left_forward_rpwm': 1}, {'polarity_verified': 1}):
            with self.subTest(config=config), self.assertRaises(ValueError):
                MotorConfig(**config)

    def test_unknown_or_unverified_polarity_never_opens_backend(self):
        for config in (MotorConfig(), MotorConfig(left_forward_rpwm=True, right_forward_rpwm=False)):
            backend = FakeBackend()
            controller = MotorController(config, backend)
            self.assertFalse(controller.arm(safety(), SECOND))
            self.assertEqual(backend.starts, 0)
            self.assertFalse(controller.armed)

    def test_current_read_only_tracking_output_cannot_arm(self):
        current = safety(drive_authorized=False, provisional=True, estop_feedback_valid=False)
        self.assertFalse(self.controller.arm(current, SECOND))
        self.assertEqual(self.backend.starts, 0)
        self.assertFalse(self.controller.armed)

    def test_explicit_single_left_profile_never_fabricates_right_status(self):
        single = safety(uwb_profile='single_left', uwb_right_valid=False, uwb_right_until_ns=0)
        self.assertTrue(single.permits(SECOND, 300))
        self.assertFalse(single.uwb_right_valid)
        self.assertFalse(replace(single, uwb_left_valid=False).permits(SECOND, 300))
        self.assertFalse(replace(single, estop_feedback_valid=False).permits(SECOND, 300))
        self.assertFalse(replace(single, drive_authorized=False, provisional=True).permits(SECOND, 300))
        self.assertFalse(replace(single, uwb_profile='dual').permits(SECOND, 300))
        with self.assertRaises(ValueError):
            replace(single, uwb_profile='unknown')

    def test_any_fault_or_expiry_immediately_disarms_and_requires_rearm(self):
        for changes in ({'drive_authorized': False}, {'provisional': True}, {'source_mode': 'replay'},
                        {'estop_feedback_valid': False}, {'estop_released': False},
                        {'uwb_left_valid': False}, {'uwb_right_valid': False},
                        {'lidar_valid': False}, {'tracking_valid': False}, {'clearance_valid': False},
                        {'clearance_ok': False}, {'speed_limit': 0},
                        {'uwb_left_until_ns': SECOND + 100 * MS},
                        {'uwb_right_until_ns': SECOND + 100 * MS},
                        {'lidar_until_ns': SECOND + 100 * MS},
                        {'tracking_until_ns': SECOND + 100 * MS},
                        {'clearance_until_ns': SECOND + 100 * MS}):
            with self.subTest(changes=changes):
                backend = FakeBackend()
                controller = MotorController(known_config(), backend)
                self.assertTrue(controller.arm(safety(), SECOND))
                self.assertTrue(controller.submit(1, 0, 1, safety(), SECOND))
                self.assertEqual(controller.step(safety(SECOND + 50 * MS), SECOND + 50 * MS), (1, 1))
                bad = safety(SECOND + 100 * MS, **changes)
                self.assertEqual(controller.step(bad, SECOND + 100 * MS), (0, 0))
                self.assertFalse(controller.armed)
                self.assertEqual(backend.writes[-1], (0, 0))
                self.assertFalse(controller.submit(1, 0, 2, safety(SECOND + 150 * MS), SECOND + 150 * MS))
                self.assertTrue(controller.arm(safety(SECOND + 160 * MS), SECOND + 160 * MS))

    def test_unchanged_safety_or_command_expires_without_autorestart(self):
        self.assertTrue(self.controller.arm(safety(), SECOND))
        self.assertTrue(self.controller.submit(1, 0, 1, safety(), SECOND))
        self.assertEqual(self.controller.step(safety(SECOND + 100 * MS), SECOND + 100 * MS), (2, 2))
        self.assertEqual(self.controller.step(safety(SECOND + 300 * MS), SECOND + 300 * MS), (0, 0))
        self.assertEqual(self.controller.reason, 'command_timeout')
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.controller.step(safety(SECOND + 310 * MS), SECOND + 310 * MS), (0, 0))

    def test_repeated_status_never_extends_observation_lifetime(self):
        frame = safety(uwb_left_until_ns=SECOND + 200 * MS)
        self.assertTrue(self.controller.arm(frame, SECOND))
        self.assertTrue(self.controller.submit(1, 0, 1, frame, SECOND))
        self.assertTrue(self.controller.submit(1, 0, 2, frame, SECOND + 100 * MS))
        self.assertEqual(self.controller.step(frame, SECOND + 200 * MS), (0, 0))
        self.assertEqual(self.controller.reason, 'interlock')

    def test_duplicate_or_invalid_command_immediately_stops(self):
        self.assertTrue(self.controller.arm(safety(), SECOND))
        self.assertTrue(self.controller.submit(1, 0, 1, safety(), SECOND))
        self.controller.step(safety(SECOND + 100 * MS), SECOND + 100 * MS)
        with self.assertRaises(ValueError):
            self.controller.submit(1, 0, 1, safety(SECOND + 110 * MS), SECOND + 110 * MS)
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.backend.writes[-1], (0, 0))

    def test_clock_regression_stops_even_after_a_newer_command(self):
        self.assertTrue(self.controller.arm(safety(), SECOND))
        self.assertTrue(self.controller.submit(1, 0, 1, safety(SECOND + 50 * MS), SECOND + 50 * MS))
        with self.assertRaises(ValueError):
            self.controller.step(safety(SECOND + 20 * MS), SECOND + 20 * MS)
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.backend.writes[-1], (0, 0))

    def test_ramp_and_reverse_wait_while_keepalive_continues(self):
        c = self.controller
        self.assertTrue(c.arm(safety(), SECOND))
        self.assertTrue(c.submit(1, 0, 1, safety(), SECOND))
        self.assertEqual(c.step(safety(SECOND + 100 * MS), SECOND + 100 * MS), (2, 2))
        self.assertTrue(c.submit(-1, 0, 2, safety(SECOND + 100 * MS), SECOND + 100 * MS))
        self.assertEqual(c.step(safety(SECOND + 200 * MS), SECOND + 200 * MS), (0, 0))
        for index, ms in enumerate((300, 400, 500, 600), start=3):
            now = SECOND + ms * MS
            self.assertTrue(c.submit(-1, 0, index, safety(now), now))
            self.assertEqual(c.step(safety(now), now), (0, 0))
        now = SECOND + 700 * MS
        self.assertTrue(c.submit(-1, 0, 7, safety(now), now))
        self.assertEqual(c.step(safety(now), now), (-2, -2))

    def test_safety_failure_does_not_wait_for_ramp(self):
        self.assertTrue(self.controller.arm(safety(), SECOND))
        self.assertTrue(self.controller.submit(1, 0, 1, safety(), SECOND))
        self.controller.step(safety(SECOND + 100 * MS), SECOND + 100 * MS)
        bad = safety(SECOND + 110 * MS, lidar_valid=False)
        self.assertEqual(self.controller.step(bad, SECOND + 110 * MS), (0, 0))
        self.assertEqual(self.backend.writes[-1], (0, 0))

    def test_safety_speed_limit_reduces_output_without_waiting_for_ramp(self):
        self.assertTrue(self.controller.arm(safety(), SECOND))
        self.assertTrue(self.controller.submit(1, 0, 1, safety(), SECOND))
        self.assertEqual(self.controller.step(safety(SECOND + 100 * MS), SECOND + 100 * MS), (2, 2))
        restricted = safety(SECOND + 150 * MS, speed_limit=.05)
        output = self.controller.step(restricted, SECOND + 150 * MS)
        self.assertAlmostEqual(output[0], .85)
        self.assertAlmostEqual(output[1], .85)
        self.assertEqual(self.backend.writes[-1], output)

    def test_example_runs_without_gpio(self):
        result = subprocess.run([sys.executable, '-X', 'utf8', '-m', 'smartcart_motor'], cwd=ROOT,
                                capture_output=True, text=True, encoding='utf-8', timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('GPIO 미개방', result.stdout)
        self.assertIn('제자리 좌회전: 차체 기준 왼쪽 -1.0, 오른쪽 +1.0', result.stdout)


class FakeGPIO:
    BCM, OUT, LOW, HIGH = 11, 1, 0, 1
    def __init__(self):
        self.out = {}
        self.duty = {}
        self.events = []
        self.fail_pin = None
    def setmode(self, mode):
        self.events.append(('mode', mode))
    def setup(self, pin, mode, initial):
        self.out[pin] = initial
    def output(self, pin, value):
        self.out[pin] = value
    def cleanup(self, pins):
        self.events.append(('cleanup', tuple(pins)))
    def PWM(self, pin, frequency):
        owner = self
        class FakePWM:
            def start(self, value):
                owner.duty[pin] = value
            def ChangeDutyCycle(self, value):
                if pin == owner.fail_pin and value > 0:
                    raise OSError('가상 PWM 오류')
                owner.duty[pin] = value
                for one, other in ((12, 13), (18, 19)):
                    if owner.duty.get(one, 0) and owner.duty.get(other, 0):
                        raise AssertionError('양 방향 PWM 동시 활성')
            def stop(self):
                owner.events.append(('pwm_stop', pin))
        if frequency != 1000:
            raise ValueError('PWM 주파수 불일치')
        return FakePWM()


class GpioBackendTests(unittest.TestCase):
    def setUp(self):
        self.gpio = FakeGPIO()
        class Ownership:
            def __enter__(self):
                return self
            def __exit__(self, *_args):
                pass
        self.backend = GpioBackend(known_config(), gpio=self.gpio, ownership=Ownership())

    def test_backend_rechecks_interlocks_and_speed_limit(self):
        with self.assertRaises(ValueError):
            self.backend.start(safety(drive_authorized=False), SECOND)
        self.assertEqual(self.gpio.events, [])
        self.backend.start(safety(), SECOND)
        self.backend.write(6, 6, safety(), SECOND)
        with self.assertRaises(ValueError):
            self.backend.write(6, 6, safety(lidar_valid=False), SECOND)
        self.assertTrue(all(value == 0 for value in self.gpio.duty.values()))
        self.assertTrue(all(self.gpio.out[pin] == 0 for pin in (23, 24, 16, 26)))

    def test_mismatched_controller_and_backend_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            MotorController(known_config(max_duty=10), self.backend)

    def test_no_gpio_access_until_start_and_stop_disables_both_drivers(self):
        self.assertEqual(self.gpio.events, [])
        self.backend.start(safety(), SECOND)
        self.assertTrue(all(value == 0 for value in self.gpio.out.values()))
        self.backend.write(6, 6, safety(), SECOND)
        self.assertEqual((self.gpio.duty[12], self.gpio.duty[19]), (6, 6))
        self.assertEqual((self.gpio.duty[13], self.gpio.duty[18]), (0, 0))
        self.backend.stop()
        self.assertTrue(all(value == 0 for value in self.gpio.duty.values()))
        self.assertTrue(all(self.gpio.out[pin] == 0 for pin in (23, 24, 16, 26)))
        self.backend.close()
        self.assertEqual(self.gpio.events[-1][0], 'cleanup')

    def test_opposite_polarity_configuration_uses_same_driver_input_for_tank_turn(self):
        self.backend.start(safety(), SECOND)
        self.backend.write(-6, 6, safety(), SECOND)
        self.assertEqual((self.gpio.duty[13], self.gpio.duty[19]), (6, 6))
        self.assertEqual((self.gpio.duty[12], self.gpio.duty[18]), (0, 0))
        self.backend.stop()
        self.backend.write(6, -6, safety(), SECOND)
        self.assertEqual((self.gpio.duty[12], self.gpio.duty[18]), (6, 6))

    def test_matching_forward_channels_map_straight_and_both_tank_turns(self):
        """좌우 전진 채널이 같은 배선에서 명령부터 가상 GPIO 출력까지 확인한다."""
        class Ownership:
            def __enter__(self):
                return self
            def __exit__(self, *_args):
                pass

        # 참은 양쪽 RPWM이 전진, 거짓은 양쪽 LPWM이 전진인 가상 설정이다.
        expected_by_config = {
            True: ((1, 0, {12, 18}), (-1, 0, {13, 19}),
                   (0, 1, {13, 18}), (0, -1, {12, 19})),
            False: ((1, 0, {13, 19}), (-1, 0, {12, 18}),
                    (0, 1, {12, 19}), (0, -1, {13, 18})),
        }
        for forward_rpwm, cases in expected_by_config.items():
            for forward, turn, expected in cases:
                with self.subTest(forward_rpwm=forward_rpwm, forward=forward, turn=turn):
                    config = MotorConfig(left_forward_rpwm=forward_rpwm,
                                         right_forward_rpwm=forward_rpwm,
                                         polarity_verified=True)
                    gpio = FakeGPIO()
                    backend = GpioBackend(config, gpio=gpio, ownership=Ownership())
                    controller = MotorController(config, backend)
                    try:
                        self.assertTrue(controller.arm(safety(), SECOND))
                        self.assertTrue(controller.submit(forward, turn, 1, safety(), SECOND))
                        now = SECOND + 100 * MS
                        controller.step(safety(now), now)
                        active = {pin for pin, duty in gpio.duty.items() if duty > 0}
                        self.assertEqual(active, expected)
                        self.assertTrue(all(gpio.duty[pin] == 2 for pin in expected))
                    finally:
                        controller.close()

    def test_direction_change_requires_zero_before_opposite_pwm(self):
        self.backend.start(safety(), SECOND)
        self.backend.write(5, 5, safety(), SECOND)
        with self.assertRaises(ValueError):
            self.backend.write(-5, 5, safety(), SECOND)
        self.assertTrue(all(value == 0 for value in self.gpio.duty.values()))
        self.assertTrue(all(self.gpio.out[pin] == 0 for pin in (23, 24, 16, 26)))

    def test_gpio_exception_attempts_to_disable_both_motors_and_latches_fault(self):
        self.backend.start(safety(), SECOND)
        self.gpio.fail_pin = RIGHT['lpwm']
        with self.assertRaises(OSError):
            self.backend.write(6, 6, safety(), SECOND)
        self.assertTrue(all(value == 0 for value in self.gpio.duty.values()))
        self.assertTrue(all(self.gpio.out[pin] == 0 for pin in (23, 24, 16, 26)))
        with self.assertRaises(RuntimeError):
            self.backend.start(safety(), SECOND)

    def test_controller_and_gpio_combination_stops_both_when_uwb_becomes_invalid(self):
        controller = MotorController(known_config(), self.backend)
        self.assertFalse(controller.arm(safety(drive_authorized=False), SECOND))
        self.assertEqual(self.gpio.events, [])
        self.assertTrue(controller.arm(safety(), SECOND + 10 * MS))
        self.assertTrue(controller.submit(1, 0, 1, safety(SECOND + 10 * MS), SECOND + 10 * MS))
        self.assertEqual(controller.step(safety(SECOND + 110 * MS), SECOND + 110 * MS), (2, 2))
        self.assertEqual((self.gpio.duty[12], self.gpio.duty[19]), (2, 2))
        controller.step(safety(SECOND + 120 * MS, uwb_left_valid=False), SECOND + 120 * MS)
        self.assertTrue(all(value == 0 for value in self.gpio.duty.values()))
        self.assertTrue(all(self.gpio.out[pin] == 0 for pin in (23, 24, 16, 26)))
        self.assertFalse(controller.armed)
        controller.close()


class MotorServiceTests(unittest.TestCase):
    def setUp(self):
        self.now = SECOND
        self.backend = FakeBackend()
        self.controller = MotorController(known_config(), self.backend)
        self.service = MotorService(self.controller, clock=lambda: self.now)

    def test_no_safety_no_arm_and_no_gpio(self):
        self.service.request_arm()
        self.service.submit(1, 0, 1)
        self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.assertEqual(self.backend.starts, 0)
        self.assertFalse(self.controller.armed)

    def test_local_mailbox_and_periodic_evaluation_stop_when_command_times_out(self):
        self.service.update_safety(safety(self.now))
        self.service.request_arm()
        self.service.submit(1, 0, 1)
        self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.now += 100 * MS
        self.service.update_safety(safety(self.now))
        self.assertEqual(self.service.cycle(self.now), (2, 2))
        self.now += 200 * MS
        self.service.update_safety(safety(self.now))
        self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.controller.reason, 'command_timeout')

    def test_stale_command_in_mailbox_does_not_get_a_new_deadline(self):
        self.service.update_safety(safety(self.now))
        self.service.request_arm()
        self.service.submit(1, 0, 1)
        self.now += 301 * MS
        self.service.update_safety(safety(self.now))
        self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.assertFalse(self.controller.armed)

    def test_invalid_or_replayed_mailbox_command_latches_stop(self):
        self.service.update_safety(safety(self.now))
        self.service.request_arm()
        self.service.submit(1, 0, 1)
        self.service.cycle(self.now)
        self.now += 100 * MS
        self.service.update_safety(safety(self.now))
        self.service.cycle(self.now)
        with self.assertRaises(ValueError):
            self.service.submit(1, 0, 1)
        self.now += 20 * MS
        self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.backend.writes[-1], (0, 0))

    def test_invalid_safety_clears_previous_and_stops(self):
        self.service.update_safety(safety(self.now))
        self.service.request_arm()
        self.service.submit(1, 0, 1)
        self.service.cycle(self.now)
        self.now += 100 * MS
        self.service.update_safety(safety(self.now))
        self.service.cycle(self.now)
        self.service.update_safety({'valid': True})
        self.now += 20 * MS
        self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.assertFalse(self.controller.armed)

    def test_invalid_service_clock_cannot_leave_a_moving_output(self):
        self.service.update_safety(safety(self.now))
        self.service.request_arm()
        self.service.submit(1, 0, 1)
        self.service.cycle(self.now)
        self.now += 100 * MS
        self.service.update_safety(safety(self.now))
        self.service.cycle(self.now)
        with self.assertRaises(ValueError):
            self.service.cycle(-1)
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.backend.writes[-1], (0, 0))

    def test_real_timer_thread_stops_output_when_heartbeats_stop(self):
        backend = FakeBackend()
        controller = MotorController(known_config(), backend)
        service = MotorService(controller)
        now = time.monotonic_ns()
        service.update_safety(safety(now))
        service.request_arm()
        service.submit(1, 0, 1)
        worker = threading.Thread(target=service.run, daemon=True)
        worker.start()
        try:
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline and not any(left > 0 and right > 0 for left, right in backend.writes):
                time.sleep(.01)
            self.assertTrue(any(left > 0 and right > 0 for left, right in backend.writes))
            while time.monotonic() < deadline and controller.armed:
                time.sleep(.01)
            self.assertFalse(controller.armed)
            self.assertEqual(backend.writes[-1], (0, 0))
        finally:
            service.stop.set()
            worker.join(timeout=2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(backend.closes, 1)

    def test_second_motor_owner_is_rejected(self):
        with MotorLock():
            with self.assertRaises(RuntimeError):
                with MotorLock():
                    pass
