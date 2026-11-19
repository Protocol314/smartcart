"""좌우 원시 표시의 출처·범위·만료와 정상 위치 판정의 분리를 검사한다."""
from copy import deepcopy
import json
import threading
import unittest

from smartcart_sensing.dual_uwb import DualUwbStore
from smartcart_sensing.service import SensingStore
from smartcart_sensing.uwb import UwbStore
from smartcart_radar.core import ScanStore
from smartcart_uwb.estimation_config import EstimationConfig
from test_tracking_runtime import mounting_config, status, SECOND, MS
from test_sensing_service import simulated_ros_scan


class DualUwbViewTests(unittest.TestCase):
    def setUp(self):
        self.config = mounting_config()
        self.stop = threading.Event()
        self.store = DualUwbStore(self.config, EstimationConfig(), stop=self.stop)

    def test_unknown_position_model_still_displays_valid_raw_observations(self):
        self.store.consume_status(status(self.config), SECOND)
        result = self.store.snapshot(now_ns=SECOND + MS)
        self.assertTrue(result['valid'])
        self.assertTrue(result['received'])
        self.assertEqual(result['sensors']['uwb_left']['range_m'], 2)
        self.assertEqual(result['sensors']['uwb_left']['raw_angle_deg'], 0)
        self.assertFalse(result['drive_authorized'])
        for forbidden in ('tag_address', 'position_xy_m', 'identity', 'port'):
            self.assertNotIn(forbidden, json.dumps(result))

    def test_invalid_range_is_not_shown_as_a_normal_distance(self):
        value = status(self.config)
        left = value['sensors']['uwb_left']
        left['latest'].update(range_m=4294967295 / 100, measurement_valid=False,
                              valid=False, invalid_reason='invalid_measurement', valid_until_monotonic_ns=0)
        left.update(state='error', last_error='invalid_measurement')
        self.store.consume_status(value, SECOND)
        result = self.store.snapshot(now_ns=SECOND + MS)
        self.assertEqual(result['sensors']['uwb_left']['state'], 'invalid')
        self.assertIsNone(result['sensors']['uwb_left']['range_m'])
        self.assertTrue(result['sensors']['uwb_left']['range_out_of_bounds'])
        self.assertTrue(result['sensors']['uwb_right']['valid'])
        self.assertTrue(result['received'])
        self.assertFalse(result['valid'])

    def test_parse_error_never_reuses_previous_distance(self):
        value = status(self.config)
        value['sensors']['uwb_left']['latest'].update(valid=False, invalid_reason='checksum')
        value['sensors']['uwb_left'].update(state='error', last_error='checksum')
        self.store.consume_status(value, SECOND)
        result = self.store.snapshot(now_ns=SECOND)
        self.assertIsNone(result['sensors']['uwb_left']['range_m'])
        self.assertIsNone(result['sensors']['uwb_left']['raw_angle_deg'])

    def test_transport_expiry_and_clock_regression_clear_values(self):
        self.store.consume_status(status(self.config), SECOND)
        self.assertEqual(self.store.snapshot(now_ns=SECOND + 751 * MS)['state'], 'stale')
        self.assertIsNone(self.store.snapshot(now_ns=SECOND + 752 * MS)['sensors']['uwb_left']['range_m'])
        self.assertEqual(self.store.snapshot(now_ns=SECOND)['state'], 'error')

    def test_repeated_old_status_never_renews_observation_lifetime(self):
        value = status(self.config)
        self.store.consume_status(value, SECOND)
        self.store.consume_status(value, SECOND + 400 * MS)
        result = self.store.snapshot(now_ns=SECOND + 501 * MS)
        self.assertEqual(result['sensors']['uwb_left']['state'], 'stale')
        self.assertIsNone(result['sensors']['uwb_left']['range_m'])

    def test_malformed_or_replay_input_clears_previous_values(self):
        for key, replacement in (('source_mode', 'replay'), ('drive_authorized', True), ('schema', 'bad')):
            with self.subTest(key=key):
                self.setUp()
                self.store.consume_status(status(self.config), SECOND)
                value = status(self.config, 2, SECOND + MS)
                value[key] = replacement
                self.store.consume_status(value, SECOND + MS)
                result = self.store.snapshot(now_ns=SECOND + MS)
                self.assertEqual(result['state'], 'error')
                self.assertIsNone(result['sensors']['uwb_left']['range_m'])

    def test_integrated_status_uses_dual_inputs_when_legacy_channel_is_off(self):
        lidar = ScanStore('ros')
        uwb = UwbStore('off', stop=lidar.stop)
        dual = DualUwbStore(self.config, EstimationConfig(), stop=lidar.stop)
        service = SensingStore(lidar, uwb, 'ros', dual_uwb=dual)
        lidar.publish(simulated_ros_scan(), now=1)
        dual.consume_status(status(self.config), SECOND)
        result = service.snapshot(now=1.01)
        self.assertEqual(result['uwb']['state'], 'off')
        self.assertTrue(result['sensing']['uwb_valid'])
        self.assertTrue(result['sensing']['ready'])
        self.assertEqual(result['sensing']['uwb_backend'], 'dual')
        self.assertTrue(result['dual_uwb']['valid'])


if __name__ == '__main__':
    unittest.main()
