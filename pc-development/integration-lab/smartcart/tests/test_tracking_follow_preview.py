"""가상 추종량과 실패 시 0 출력을 검사한다. GPIO나 실제 센서는 열지 않는다."""
import math
import unittest

from smartcart_tracking.follow_preview import FollowPreviewConfig, simulate

NOW = 1_000_000_000


def track(x=2.0, y=0.0):
    return {'schema': 'smartcart.target_track.v1', 'source_mode': 'replay', 'provisional': True,
            'drive_authorized': False, 'valid': True, 'state': 'tracking', 'confirmation_count': 3,
            'candidate_count': 1, 'associated_cluster_id': 0, 'position_xy_m': [x, y],
            'reference_monotonic_ns': NOW - 10_000_000, 'valid_until_monotonic_ns': NOW + 100_000_000,
            'error_radius_m': .2}


def clearance():
    return {'synthetic': True, 'valid': True, 'forward_clear': True, 'rotation_clear': True,
            'observed_ns': NOW - 10_000_000, 'valid_until_ns': NOW + 100_000_000}


class FollowPreviewTests(unittest.TestCase):
    def test_distance_scales_forward_without_claiming_speed_or_drive(self):
        near = simulate(track(1.6), clearance(), NOW)
        far = simulate(track(3.0), clearance(), NOW)
        self.assertGreater(near['preview_forward_fraction'], 0)
        self.assertGreater(far['preview_forward_fraction'], near['preview_forward_fraction'])
        self.assertLessEqual(far['preview_forward_fraction'], .35)
        self.assertTrue(far['synthetic_only'])
        self.assertFalse(far['drive_authorized'])

    def test_stop_and_both_pivot_directions(self):
        stopped = simulate(track(.9), clearance(), NOW)
        self.assertEqual(stopped['state'], 'stopped')
        left, right = simulate(track(.5, 2), clearance(), NOW), simulate(track(.5, -2), clearance(), NOW)
        self.assertEqual(left['state'], 'preview_pivot')
        self.assertEqual(left['preview_forward_fraction'], 0)
        self.assertGreater(left['preview_turn_fraction'], 0)
        self.assertLess(right['preview_turn_fraction'], 0)

    def test_obstacles_require_the_relevant_synthetic_clearance(self):
        front = clearance(); front['forward_clear'] = False
        self.assertEqual(simulate(track(), front, NOW)['reason'], 'forward_obstacle')
        turn = clearance(); turn['rotation_clear'] = False
        self.assertEqual(simulate(track(.5, 2), turn, NOW)['reason'], 'rotation_obstacle')

    def test_live_or_unconfirmed_tracking_never_generates_preview_motion(self):
        for changes in ({'source_mode': 'live'}, {'drive_authorized': True}, {'provisional': False},
                        {'valid': False}, {'state': 'ambiguous'}, {'confirmation_count': 2},
                        {'candidate_count': 2}, {'candidate_count': True}, {'associated_cluster_id': -1},
                        {'error_radius_m': 1.0}):
            item = track(); item.update(changes)
            result = simulate(item, clearance(), NOW)
            self.assertEqual(result['preview_forward_fraction'], 0, changes)
            self.assertEqual(result['preview_turn_fraction'], 0, changes)

    def test_stale_missing_and_malformed_inputs_block(self):
        stale = track(); stale['valid_until_monotonic_ns'] = NOW
        for t, c in ((stale, clearance()), (None, clearance()), (track(), None),
                     (track(), {**clearance(), 'synthetic': False}),
                     (track(), {**clearance(), 'valid_until_ns': NOW}),
                     ({**track(), 'position_xy_m': [math.nan, 0]}, clearance())):
            result = simulate(t, c, NOW)
            self.assertEqual(result['state'], 'blocked')
            self.assertEqual(result['preview_forward_fraction'], 0)
            self.assertEqual(result['preview_turn_fraction'], 0)

    def test_bad_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            FollowPreviewConfig(stop_distance_m=2, follow_distance_m=1)
        with self.assertRaises(ValueError):
            FollowPreviewConfig(max_forward_fraction=math.nan)


if __name__ == '__main__':
    unittest.main()
