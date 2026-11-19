"""합성 추적에서 좌우 가상 듀티와 고장 잠금을 검증한다."""
import unittest

from smartcart_tracking.follow_motor_replay import ReplayMotorBridge
from test_tracking_follow_preview import NOW, track, clearance


class ReplayMotorBridgeTests(unittest.TestCase):
    def test_forward_and_tank_turn_are_virtual(self):
        bridge = ReplayMotorBridge()
        result = bridge.evaluate(track(3), clearance(), NOW)
        self.assertGreater(result['left_preview_duty'], 0)
        self.assertEqual(result['left_preview_duty'], result['right_preview_duty'])
        self.assertFalse(result['drive_authorized'])
        self.assertTrue(result['synthetic_only'])
        for y, expected in ((2, (-1, 1)), (-2, (1, -1))):
            result = bridge.evaluate(track(.5, y), clearance(), NOW)
            self.assertEqual((result['left_preview_duty'] > 0, result['right_preview_duty'] > 0),
                             (expected[0] > 0, expected[1] > 0))

    def test_obstacle_or_lost_track_latches_zero_until_reset(self):
        for changed_track, changed_clearance in (({**track(), 'valid': False}, clearance()),
                                                  (track(), {**clearance(), 'forward_clear': False}),
                                                  (track(), {**clearance(), 'valid_until_ns': NOW})):
            bridge = ReplayMotorBridge()
            self.assertGreater(bridge.evaluate(track(3), clearance(), NOW)['left_preview_duty'], 0)
            result = bridge.evaluate(changed_track, changed_clearance, NOW)
            self.assertEqual((result['left_preview_duty'], result['right_preview_duty']), (0, 0))
            latched = bridge.evaluate(track(3), clearance(), NOW)
            self.assertEqual(latched['state'], 'latched')
            self.assertEqual(latched['reason'], result['reason'])
            bridge.reset()
            self.assertGreater(bridge.evaluate(track(3), clearance(), NOW)['left_preview_duty'], 0)

    def test_rotation_obstacle_and_expired_track_block(self):
        bridge = ReplayMotorBridge()
        result = bridge.evaluate(track(.5, 2), {**clearance(), 'rotation_clear': False}, NOW)
        self.assertEqual(result['reason'], 'rotation_obstacle')
        self.assertEqual((result['left_preview_duty'], result['right_preview_duty']), (0, 0))
        bridge.reset()
        result = bridge.evaluate({**track(), 'valid_until_monotonic_ns': NOW}, clearance(), NOW)
        self.assertEqual(result['reason'], 'tracking_stale')
        self.assertEqual((result['left_preview_duty'], result['right_preview_duty']), (0, 0))

    def test_live_input_is_rejected(self):
        result = ReplayMotorBridge().evaluate({**track(), 'source_mode': 'live'}, clearance(), NOW)
        self.assertEqual(result['state'], 'latched')
        self.assertEqual((result['left_preview_duty'], result['right_preview_duty']), (0, 0))


if __name__ == '__main__':
    unittest.main()
