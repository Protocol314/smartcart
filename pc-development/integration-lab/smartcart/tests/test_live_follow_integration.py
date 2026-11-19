"""실제 포트를 열지 않고 LiDAR·추적·모터 서비스의 고장 차단을 검증한다."""
import math
import unittest

from smartcart_motor.control import MotorController
from smartcart_motor.lidar_clearance import ClearanceConfig, evaluate_clearance
from smartcart_motor.live_follow import LiveFollow, LiveFollowConfig
from smartcart_motor.service import MotorService
from test_motor_control import FakeBackend, known_config, safety, SECOND, MS


def scan(now=SECOND):
    return {'schema': 'smartcart.lidar_observation.v1', 'sensor_id': 'lidar_center',
            'frame_id': 'lidar_scan', 'timestamp_source': 'HOST_RX', 'sample_sequence': 1,
            'valid': True, 'source_mode': 'live', 'rx_monotonic_ns': now,
            'valid_until_monotonic_ns': now + 200 * MS,
            'scan': {'angle_min': -math.pi, 'angle_max': math.pi - 2 * math.pi / 360,
                     'angle_increment': 2 * math.pi / 360, 'range_min': .05,
                     'range_max': 10., 'ranges': [2.] * 360}}


def track(now=SECOND, sequence=1):
    return {'schema': 'smartcart.target_track.v1', 'source_mode': 'live', 'frame_id': 'base_link',
            'valid': True, 'state': 'tracking', 'provisional': False, 'drive_authorized': True,
            'confirmation_count': 3, 'candidate_count': 1, 'associated_cluster_id': 1,
            'reference_monotonic_ns': now, 'valid_until_monotonic_ns': now + 200 * MS,
            'track_sequence': sequence, 'error_radius_m': .1, 'position_xy_m': [2., 0.]}


class ClearanceTests(unittest.TestCase):
    def setUp(self):
        self.cfg = ClearanceConfig(.8, geometry_verified=True)

    def test_obstacle_any_direction_and_unverified_geometry(self):
        for index in (0, 90, 180, 270):
            obs = scan(); obs['scan']['ranges'][index] = .3
            self.assertEqual(evaluate_clearance(obs, SECOND, self.cfg)['reason'], 'obstacle')
        self.assertEqual(evaluate_clearance(scan(), SECOND, ClearanceConfig(.8))['reason'], 'geometry_unverified')
        self.assertTrue(evaluate_clearance(scan(), SECOND, self.cfg)['clear'])

    def test_missing_rays_and_stale_scan_are_blocked(self):
        for value in (None, float('nan'), float('inf'), 0., 11.):
            obs = scan(); obs['scan']['ranges'][10] = value
            self.assertFalse(evaluate_clearance(obs, SECOND, self.cfg)['clear'])
        obs = scan(); obs['scan']['ranges'] = obs['scan']['ranges'][:180]
        self.assertEqual(evaluate_clearance(obs, SECOND, self.cfg)['reason'], 'incomplete_coverage')
        self.assertEqual(evaluate_clearance(scan(), SECOND + 200 * MS, self.cfg)['reason'], 'stale_scan')
        self.assertFalse(evaluate_clearance(None, SECOND, self.cfg)['clear'])
        bad_frame = {**scan(), 'frame_id': 'other'}
        self.assertFalse(evaluate_clearance(bad_frame, SECOND, self.cfg)['clear'])
        obs = scan(); obs['scan']['angle_max'] = 0
        self.assertEqual(evaluate_clearance(obs, SECOND, self.cfg)['reason'], 'incomplete_coverage')
        obs = scan(); obs['scan']['range_min'] = 1.
        self.assertEqual(evaluate_clearance(obs, SECOND, self.cfg)['reason'], 'invalid_scan')


class LiveIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.now = SECOND
        self.backend = FakeBackend()
        self.controller = MotorController(known_config(), self.backend)
        self.service = MotorService(self.controller, clock=lambda: self.now)
        self.bridge = LiveFollow(self.service, LiveFollowConfig(1.5, 1., .5), ClearanceConfig(.8, True))

    def start(self):
        self.service.request_arm()
        self.assertTrue(self.bridge.update(track(), scan(), safety(), self.now)['accepted'])
        self.service.cycle(self.now)
        self.now += 50 * MS
        self.assertTrue(self.bridge.update(track(self.now, 2), scan(self.now), safety(self.now), self.now)['accepted'])
        self.assertGreater(self.service.cycle(self.now)[0], 0)

    def test_obstacle_stops_and_requires_rearm(self):
        self.start()
        self.now += 20 * MS
        obs = scan(self.now); obs['scan']['ranges'][90] = .2
        self.assertEqual(self.bridge.update(track(self.now, 3), obs, safety(self.now), self.now)['reason'], 'obstacle')
        self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.backend.writes[-1], (0, 0))
        self.now += 20 * MS
        self.bridge.update(track(self.now, 4), scan(self.now), safety(self.now), self.now)
        self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.assertFalse(self.controller.armed)

    def test_sensor_faults_and_current_provisional_track_stop(self):
        for change in ({'uwb_left_valid': False}, {'uwb_right_valid': False}, {'lidar_valid': False},
                       {'estop_feedback_valid': False}, {'tracking_valid': False},
                       {'uwb_left_until_ns': SECOND + 60 * MS}, {'uwb_right_until_ns': SECOND + 60 * MS}):
            with self.subTest(change=change):
                self.setUp(); self.start(); self.now += 20 * MS
                self.assertFalse(self.bridge.update(track(self.now, 3), scan(self.now), safety(self.now, **change), self.now)['accepted'])
                self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.setUp(); self.start(); self.now += 20 * MS
        current = {**track(self.now, 3), 'drive_authorized': False, 'provisional': True}
        self.assertFalse(self.bridge.update(current, scan(self.now), safety(self.now), self.now)['accepted'])
        self.assertEqual(self.service.cycle(self.now), (0, 0))

    def test_input_stops_even_if_updates_stop(self):
        self.start()
        self.now += 201 * MS
        self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.assertFalse(self.controller.armed)

    def test_transient_obstacle_is_latched_until_control_cycle(self):
        self.start()
        self.now += 20 * MS
        obs = scan(self.now)
        obs['scan']['ranges'][0] = .2
        self.assertEqual(self.bridge.update(track(self.now, 3), obs, safety(self.now), self.now)['reason'], 'obstacle')
        self.assertTrue(self.bridge.update(track(self.now, 4), scan(self.now), safety(self.now), self.now)['accepted'])
        self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.assertFalse(self.controller.armed)

    def test_replay_missing_scan_and_duplicate_track_never_keep_running(self):
        for changed_track, changed_scan in (({**track(SECOND + 70 * MS, 3), 'source_mode': 'replay'}, scan(SECOND + 70 * MS)),
                                             (track(SECOND + 70 * MS, 3), None)):
            with self.subTest(track=changed_track.get('source_mode'), missing=changed_scan is None):
                self.setUp(); self.start(); self.now += 20 * MS
                self.assertFalse(self.bridge.update(changed_track, changed_scan, safety(self.now), self.now)['accepted'])
                self.assertEqual(self.service.cycle(self.now), (0, 0))

    def test_repeated_track_does_not_refresh_deadline_and_obstacle_still_stops(self):
        self.start(); self.now += 20 * MS
        repeated = track(SECOND + 50 * MS, 2)
        self.assertEqual(self.bridge.update(repeated, scan(self.now), safety(self.now), self.now)['reason'], 'no_new_track')
        self.now = SECOND + 251 * MS
        self.assertEqual(self.service.cycle(self.now), (0, 0))
        self.setUp(); self.start(); self.now += 20 * MS
        obs = scan(self.now); obs['scan']['ranges'][0] = .2
        self.assertEqual(self.bridge.update(repeated, obs, safety(self.now), self.now)['reason'], 'obstacle')
        self.assertEqual(self.service.cycle(self.now), (0, 0))


if __name__ == '__main__':
    unittest.main()
