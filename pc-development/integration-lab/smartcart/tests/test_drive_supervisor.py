"""수동·추종 모드 전환, 정지 우선순위, 래치·재무장의 가상 통합 검사.

가상 GPIO와 합성 실측 형식 입력만 사용한다. 실물 모터·센서·비상정지 검수가 아니다.
'VIRTUAL-TEST-ONLY' 검수 기록은 시험용이며 실제 현장 검수 결과가 아니다.
"""
from dataclasses import replace
import threading
import unittest

from smartcart_motor.control import MotorConfig, MotorController
from smartcart_motor.gpio_fixed_enable import FixedEnableGpioBackend
from smartcart_motor.lidar_clearance import ClearanceConfig
from smartcart_motor.live_follow import LiveFollow, LiveFollowConfig
from smartcart_motor.safety_provider import (DriveAcceptance, ESTOP_UNAVAILABLE, EstopReading,
                                             build_safety_frame)
from smartcart_motor.service import MotorService
from smartcart_motor.supervisor import DriveSupervisor
from test_gpio_fixed_enable import Ownership
from test_live_follow_integration import scan, track
from test_motor_control import FakeBackend, FakeGPIO, known_config, safety, SECOND, MS

VERIFIED = DriveAcceptance(record_id='VIRTUAL-TEST-ONLY', forward_polarity=True,
                           independent_cutoff=True, estop_feedback=True, lidar_geometry=True,
                           uwb_model=True, follow_trial=True)
FOLLOW = LiveFollowConfig(follow_distance_m=1.2, stop_distance_m=.8, max_error_m=.5)
CLEAR = ClearanceConfig(.8, geometry_verified=True)


def uwb_status(now, left=None, right=None, **changes):
    def observation(sensor_id):
        value = {'schema': 'smartcart.uwb_observation.v1', 'sensor_id': sensor_id,
                 'source_mode': 'live', 'valid': True, 'measurement_valid': True,
                 'id_valid': True, 'logical_target_id': 1, 'rx_monotonic_ns': now,
                 'valid_until_monotonic_ns': now + 300 * MS}
        return value
    latest = observation('uwb_left')
    latest.update(changes)
    sensors = {'uwb_left': left or {'state': 'live', 'connected': True, 'stream_epoch': 0,
                                    'latest': latest, 'last_error': None},
               'uwb_right': right or {'state': 'error', 'connected': False, 'stream_epoch': 0,
                                      'latest': None, 'last_error': 'disconnected'}}
    return {'schema': 'smartcart.dual_uwb_status.v1', 'source_mode': 'live',
            'adapter_session': 'virtual', 'drive_authorized': False, 'sensors': sensors}


def estop(now, released=True):
    return EstopReading(True, released, now + 200 * MS)


class SupervisorTests(unittest.TestCase):
    def setUp(self):
        self.now = SECOND
        self.seq = 0
        self.track_seq = 0
        self.backend = FakeBackend()
        self.controller = MotorController(known_config(), self.backend)
        self.sup = self.make()

    def make(self, controller=None, acceptance=VERIFIED, clearance=CLEAR, **options):
        return DriveSupervisor(controller or self.controller, acceptance=acceptance,
                               clearance=clearance, follow=FOLLOW, clock=lambda: self.now, **options)

    def tick(self, ms=20):
        self.now += ms * MS

    def feed(self, sup=None, new_track=True, **changes):
        if new_track:
            self.track_seq += 1
        values = dict(lidar_observation=scan(self.now), uwb_status=uwb_status(self.now),
                      estop=estop(self.now), track=track(self.now, self.track_seq))
        values.update(changes)
        return (sup or self.sup).update_inputs(now_ns=self.now, **values)

    def drive(self, forward=1., turn=0.):
        self.seq += 1
        return self.sup.manual_command(forward, turn, self.seq, self.now)

    def start(self, mode='manual'):
        self.sup.set_mode(mode, self.now)
        self.assertEqual(self.feed()['blocking_reasons'], [])
        self.assertTrue(self.sup.request_arm(self.now)['accepted'])
        self.sup.cycle(self.now)
        self.assertTrue(self.sup.status(self.now)['active'])
        self.tick()
        self.feed()
        if mode == 'manual':
            self.assertTrue(self.drive()['accepted'])
        output = self.sup.cycle(self.now)
        self.assertGreater(output[0], 0)
        self.assertGreater(output[1], 0)

    def assert_stopped(self, reason):
        status = self.sup.status(self.now)
        self.assertFalse(status['active'])
        self.assertEqual(status['latched_reason'], reason)
        self.assertFalse(self.controller.armed)
        # 다음 제어 주기를 기다리지 않고 즉시 PWM 0을 요청한다.
        self.assertEqual(self.backend.writes[-1], (0.0, 0.0))
        self.assertEqual(self.sup.cycle(self.now), (0.0, 0.0))

    # 현재 현장 상태 ----------------------------------------------------------
    def test_current_field_state_stays_locked_with_priority_reasons(self):
        backend = FakeBackend()
        controller = MotorController(MotorConfig(), backend)
        sup = self.make(controller, acceptance=DriveAcceptance(), clearance=ClearanceConfig(.8))
        observed = {**track(self.now), 'provisional': True, 'drive_authorized': False,
                    'state': 'ambiguous', 'valid': False, 'candidate_count': 9}
        sup.set_mode('manual', self.now)
        status = self.feed(sup, estop=ESTOP_UNAVAILABLE, track=observed)
        self.assertEqual(status['blocking_reasons'], [
            'estop_feedback_unavailable', 'unverified_forward_polarity',
            'unverified_independent_cutoff', 'unverified_estop_feedback',
            'unverified_lidar_geometry', 'clearance_geometry_unverified'])
        self.assertEqual(sup.request_arm(self.now)['reason'], 'estop_feedback_unavailable')
        self.assertEqual(sup.manual_command(1., 0., 1, self.now)['reason'], 'latched')
        self.assertEqual(sup.cycle(self.now), (0.0, 0.0))
        sup.set_mode('follow', self.now)
        reasons = self.feed(sup, estop=ESTOP_UNAVAILABLE, track=observed)['blocking_reasons']
        self.assertIn('unverified_uwb_model', reasons)
        self.assertEqual(reasons[-1], 'tracking_ambiguous')
        self.assertEqual(sup.request_arm(self.now)['accepted'], False)
        self.assertEqual(backend.starts, 0)
        self.assertEqual(sup.status(self.now)['measured_rpm'], None)

    def test_fixed_enable_output_is_never_started_in_current_field_state(self):
        gpio = FakeGPIO()
        config = MotorConfig()
        controller = MotorController(config, FixedEnableGpioBackend(config, gpio=gpio, ownership=Ownership()))
        sup = self.make(controller, acceptance=DriveAcceptance(), clearance=ClearanceConfig(.8))
        sup.set_mode('manual', self.now)
        self.feed(sup, estop=ESTOP_UNAVAILABLE)
        self.assertFalse(sup.request_arm(self.now)['accepted'])
        sup.cycle(self.now)
        self.assertTrue(sup.status(self.now)['hardware_output'])
        self.assertEqual(gpio.events, [])

    def test_verification_flags_without_acceptance_record_are_rejected(self):
        with self.assertRaises(ValueError):
            DriveAcceptance(forward_polarity=True)
        with self.assertRaises(ValueError):
            self.make(MotorController(known_config(), FakeBackend()), acceptance=DriveAcceptance(),
                      clearance=ClearanceConfig(.8))
        with self.assertRaises(ValueError):
            self.make(MotorController(MotorConfig(), FakeBackend()), acceptance=DriveAcceptance())
        config = MotorConfig()
        backend = FixedEnableGpioBackend(config, cutoff_verified=True, gpio=FakeGPIO(), ownership=Ownership())
        with self.assertRaises(ValueError):
            self.make(MotorController(config, backend), acceptance=DriveAcceptance(),
                      clearance=ClearanceConfig(.8))
        with self.assertRaises(ValueError):
            build_safety_frame('manual', lidar_observation=scan(), uwb_status=uwb_status(SECOND),
                               track=None, estop=estop(SECOND), acceptance=DriveAcceptance(),
                               clearance=CLEAR, now_ns=SECOND)

    # 수동 ------------------------------------------------------------------
    def test_manual_drive_and_tank_turns_with_virtual_verified_inputs(self):
        self.start('manual')
        # 중립 유지 신호로 감속·역전 대기(500ms)를 지난 뒤 제자리 좌회전한다.
        for _ in range(30):
            self.tick(); self.feed(); self.drive(0., 0.); self.sup.cycle(self.now)
        for _ in range(10):
            self.tick(); self.feed(); self.drive(0., 1.)
            left, right = self.sup.cycle(self.now)
        self.assertLess(left, 0)
        self.assertGreater(right, 0)
        self.assertFalse(self.sup.status(self.now)['hardware_output'])

    def test_obstacle_latches_and_recovery_needs_neutral_explicit_rearm(self):
        self.start()
        self.tick()
        blocked = scan(self.now)
        blocked['scan']['ranges'][90] = .3
        self.assertEqual(self.feed(lidar_observation=blocked)['blocking_reasons'], ['obstacle'])
        self.assert_stopped('obstacle')
        self.tick(); self.feed()
        self.assertEqual(self.drive()['reason'], 'latched')
        self.assertEqual(self.sup.request_arm(self.now)['reason'], 'controls_not_neutral')
        self.drive(0., 0.)
        self.assertTrue(self.sup.request_arm(self.now)['accepted'])
        self.sup.cycle(self.now)
        self.assertTrue(self.sup.status(self.now)['active'])

    def test_sensor_faults_identity_and_estop_stop_immediately(self):
        cases = (
            (dict(lidar_observation={**scan(SECOND), 'valid': False}), 'lidar_invalid'),
            (dict(lidar_observation={**scan(SECOND), 'source_mode': 'replay'}), 'lidar_not_live'),
            (dict(lidar_observation=None), 'lidar_missing'),
            (dict(uwb_status=None), 'uwb_status_missing'),
            (dict(uwb_status={'connected': False}), 'uwb_left_disconnected'),
            (dict(uwb_status={'id_valid': False, 'logical_target_id': None}), 'uwb_left_id_mismatch'),
            (dict(uwb_status={'measurement_valid': False}), 'uwb_left_invalid'),
            (dict(uwb_status={'rx_monotonic_ns': 0, 'valid_until_monotonic_ns': 1}), 'uwb_left_stale'),
            (dict(estop='engaged'), 'estop_engaged'),
            (dict(estop=ESTOP_UNAVAILABLE), 'estop_feedback_unavailable'),
            (dict(lidar_observation='unobserved'), 'clearance_unobserved_rays'),
        )
        for change, reason in cases:
            with self.subTest(reason=reason):
                self.setUp(); self.start(); self.tick()
                change = dict(change)
                if 'lidar_observation' in change and isinstance(change['lidar_observation'], dict):
                    change['lidar_observation'].update(rx_monotonic_ns=self.now,
                                                       valid_until_monotonic_ns=self.now + 200 * MS)
                if change.get('lidar_observation') == 'unobserved':
                    change['lidar_observation'] = scan(self.now)
                    change['lidar_observation']['scan']['ranges'][10] = float('inf')
                if isinstance(change.get('uwb_status'), dict):
                    value = change['uwb_status']
                    if 'connected' in value:
                        left = {'state': 'error', 'connected': False, 'stream_epoch': 1,
                                'latest': None, 'last_error': 'disconnected'}
                        change['uwb_status'] = uwb_status(self.now, left=left)
                    else:
                        change['uwb_status'] = uwb_status(self.now, **value)
                if change.get('estop') == 'engaged':
                    change['estop'] = estop(self.now, released=False)
                self.assertEqual(self.feed(**change)['blocking_reasons'][0], reason)
                self.assert_stopped(reason)

    def test_fault_and_stop_cannot_be_erased_by_same_cycle_recovery(self):
        self.start(); self.tick()
        self.feed(lidar_observation={**scan(self.now), 'valid': False})
        self.feed(new_track=False)
        self.assertEqual(self.drive()['reason'], 'latched')
        self.assert_stopped('lidar_invalid')
        self.setUp(); self.start(); self.tick()
        self.sup.stop(now_ns=self.now)
        self.feed()
        self.assertEqual(self.drive()['reason'], 'latched')
        self.assert_stopped('operator_stop')

    def test_command_expiry_and_input_feed_loss_latch(self):
        self.start()
        for _ in range(15):
            self.tick(); self.feed(); self.sup.cycle(self.now)
        self.assert_stopped('controller_command_timeout')
        self.setUp(); self.start()
        self.tick(320)
        self.sup.cycle(self.now)
        self.assert_stopped('inputs_expired')

    def test_invalid_and_reordered_manual_commands_stop(self):
        self.start(); self.tick(); self.feed()
        self.assertEqual(self.sup.manual_command(float('nan'), 0., 99, self.now)['reason'], 'invalid_command')
        self.assert_stopped('invalid_command')
        self.setUp(); self.start(); self.tick(); self.feed()
        self.assertFalse(self.sup.manual_command(1., 0., self.seq, self.now)['accepted'])
        self.assert_stopped('manual_repeated_or_reordered_command')

    # 모드 전환과 추종 ---------------------------------------------------------
    def test_mode_change_stops_and_requires_rearm_in_new_mode(self):
        self.start('manual')
        self.sup.set_mode('follow', self.now)
        self.assert_stopped('mode_change')
        self.assertEqual(self.sup.request_arm(self.now)['reason'], 'awaiting_inputs')
        self.tick()
        self.assertEqual(self.feed()['blocking_reasons'], [])
        self.assertTrue(self.sup.request_arm(self.now)['accepted'])
        self.sup.cycle(self.now)
        self.tick(); self.feed()
        left, right = self.sup.cycle(self.now)
        self.assertGreater(left, 0)
        self.assertGreater(right, 0)
        self.sup.set_mode('locked', self.now)
        self.assertEqual(self.sup.request_arm(self.now)['reason'], 'mode_locked')
        self.assertEqual(self.feed()['blocking_reasons'], ['mode_locked'])

    def test_follow_tracking_errors_stop(self):
        cases = (({'candidate_count': 2}, 'tracking_ambiguous'),
                 ({'state': 'lost', 'valid': False}, 'tracking_lost'),
                 ({'provisional': True}, 'tracking_provisional'),
                 ({'drive_authorized': False}, 'tracking_not_authorized'),
                 ({'valid_until_monotonic_ns': 1, 'reference_monotonic_ns': 0}, 'tracking_stale'),
                 ({'error_radius_m': 5.}, 'follow_tracking_uncertain'))
        for change, reason in cases:
            with self.subTest(reason=reason):
                self.setUp(); self.start('follow'); self.tick()
                self.track_seq += 1
                self.feed(new_track=False, track={**track(self.now, self.track_seq), **change})
                self.assert_stopped(reason)

    def test_same_track_sample_does_not_extend_follow_command(self):
        self.start('follow')
        for _ in range(16):
            self.tick(); self.feed(new_track=False); self.sup.cycle(self.now)
        self.assert_stopped('controller_command_timeout')

    def test_manual_input_during_follow_stops_but_neutral_keepalive_does_not(self):
        self.start('follow')
        self.assertEqual(self.sup.manual_command(0., 0., 1, self.now)['reason'], 'mode_mismatch')
        self.assertTrue(self.sup.status(self.now)['active'])
        self.assertEqual(self.sup.manual_command(1., 0., 2, self.now)['reason'], 'mode_mismatch')
        self.assert_stopped('mode_mismatch')

    def test_single_left_ignores_disconnected_right_but_dual_requires_it(self):
        self.start('manual')
        dual = self.make(MotorController(known_config(), FakeBackend()), uwb_profile='dual')
        dual.set_mode('manual', self.now)
        self.assertEqual(self.feed(dual)['blocking_reasons'], ['uwb_right_disconnected'])

    def test_manual_frame_skips_tracking_and_follow_rejects_manual_frame(self):
        manual = safety(SECOND, control_mode='manual', tracking_valid=False, tracking_until_ns=0)
        self.assertTrue(manual.permits(SECOND, 300))
        self.assertFalse(replace(manual, control_mode='follow').permits(SECOND, 300))
        self.assertFalse(safety(SECOND, control_mode='manual', estop_feedback_valid=False).permits(SECOND, 300))
        with self.assertRaises(ValueError):
            safety(control_mode='auto')
        service = MotorService(MotorController(known_config(), FakeBackend()), clock=lambda: SECOND)
        follow = LiveFollow(service, FOLLOW, CLEAR)
        self.assertEqual(follow.update(track(SECOND), scan(SECOND), manual, SECOND)['reason'],
                         'safety_mode_mismatch')

    def test_run_loop_shuts_down_and_closes_output(self):
        stop = threading.Event()
        worker = threading.Thread(target=self.sup.run, args=(stop,), daemon=True)
        worker.start()
        stop.set()
        worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(self.backend.closes, 1)
        self.assertEqual(self.sup.status(self.now)['last_stop_reason'], 'shutdown')


if __name__ == '__main__':
    unittest.main()
