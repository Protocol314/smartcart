"""수동 입력과 LiDAR·안전 잠금의 가상 통합 검사. 실물 GPIO는 열지 않는다."""
import unittest

from smartcart_motor.control import MotorController
from smartcart_motor.lidar_clearance import ClearanceConfig
from smartcart_motor.live_manual import LiveManual
from smartcart_motor.service import MotorService
from test_live_follow_integration import scan
from test_motor_control import FakeBackend, known_config, safety, SECOND, MS


class LiveManualTests(unittest.TestCase):
    def setUp(self):
        self.now = SECOND
        self.backend = FakeBackend()
        self.controller = MotorController(known_config(), self.backend)
        self.service = MotorService(self.controller, clock=lambda: self.now)
        self.manual = LiveManual(self.service, ClearanceConfig(.8, geometry_verified=True))

    def update(self, sequence, forward=1, turn=0, observation=None, frame=None):
        return self.manual.update(forward, turn, sequence,
                                  scan(self.now) if observation is None else observation,
                                  safety(self.now) if frame is None else frame, self.now)

    def start(self):
        self.service.request_arm()
        self.assertTrue(self.update(1)['accepted'])
        self.service.cycle(self.now)
        self.now += 50 * MS
        self.assertTrue(self.update(2)['accepted'])
        self.assertGreater(self.service.cycle(self.now)[0], 0)

    def test_manual_obstacle_stops_and_requires_explicit_rearm(self):
        self.start()
        self.now += 20 * MS
        obs = scan(self.now)
        obs['scan']['ranges'][40] = .2
        self.assertEqual(self.update(3, observation=obs)['reason'], 'obstacle')
        self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.assertFalse(self.controller.armed)
        self.now += 20 * MS
        self.update(4)
        self.assertEqual(self.service.cycle(self.now), (0, 0))

    def test_fault_cannot_be_erased_by_recovery_before_control_cycle(self):
        self.start()
        self.now += 20 * MS
        bad = safety(self.now, uwb_left_valid=False)
        self.assertEqual(self.update(3, frame=bad)['reason'], 'safety_unavailable')
        self.assertTrue(self.update(4)['accepted'])
        self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.assertFalse(self.controller.armed)

    def test_stop_cannot_be_erased_before_control_cycle(self):
        self.start()
        self.manual.stop()
        self.now += 10 * MS
        self.update(3)
        self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.assertFalse(self.controller.armed)

    def test_sensor_error_estop_unverified_track_and_duplicate_stop(self):
        for change in ({'lidar_valid': False}, {'tracking_valid': False},
                       {'uwb_left_valid': False}, {'uwb_right_valid': False},
                       {'estop_feedback_valid': False}, {'drive_authorized': False},
                       {'provisional': True}):
            with self.subTest(change=change):
                self.setUp(); self.start(); self.now += 10 * MS
                self.assertFalse(self.update(3, frame=safety(self.now, **change))['accepted'])
                self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.setUp(); self.start(); self.now += 10 * MS
        self.assertEqual(self.update(2)['reason'], 'repeated_or_reordered_command')
        self.assertEqual(self.service.cycle(self.now), (0, 0))

    def test_missing_scan_and_invalid_input_stop(self):
        for obs in ({}, {'source_mode': 'replay'}, {**scan(), 'valid': False}):
            with self.subTest(obs=obs):
                self.setUp(); self.start(); self.now += 10 * MS
                self.assertFalse(self.update(3, observation=obs)['accepted'])
                self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.setUp(); self.start(); self.now += 10 * MS
        self.assertEqual(self.update(3, forward=float('nan'))['reason'], 'invalid_input')
        self.assertEqual(self.service.cycle(self.now), (0, 0))

    def test_command_and_sensor_deadlines_expire(self):
        self.start()
        self.now += 201 * MS
        self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.assertFalse(self.controller.armed)

    def test_single_left_profile_keeps_right_sensor_disconnected(self):
        frame = safety(self.now, uwb_profile='single_left', uwb_right_valid=False, uwb_right_until_ns=0)
        self.service.request_arm()
        self.assertTrue(self.update(1, frame=frame)['accepted'])
        self.service.cycle(self.now)
        self.now += 50 * MS
        frame = safety(self.now, uwb_profile='single_left', uwb_right_valid=False, uwb_right_until_ns=0)
        self.assertTrue(self.update(2, frame=frame)['accepted'])
        self.assertGreater(self.service.cycle(self.now)[0], 0)
        self.now += 10 * MS
        current = safety(self.now, uwb_profile='single_left', uwb_right_valid=False,
                         uwb_right_until_ns=0, estop_feedback_valid=False,
                         drive_authorized=False, provisional=True)
        self.assertEqual(self.update(3, frame=current)['reason'], 'safety_unavailable')
        self.assertEqual(self.service.cycle(self.now), (0, 0))


if __name__ == '__main__':
    unittest.main()
