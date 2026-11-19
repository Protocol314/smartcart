"""웹 표본의 중복 제거와 두 가정의 기하 계산을 합성값으로 검증한다."""
import importlib.util
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('observe_dual_geometry', ROOT / 'scripts/observe_dual_geometry.py')
monitor_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor_module)


def payload(left_seq=1, right_seq=1):
    distance = math.hypot(2, .225)
    angle = math.degrees(math.atan2(.225, 2))
    base = {'state': 'live', 'valid': True, 'connected': True, 'measurement_valid': True,
            'id_valid': True, 'range_m': distance, 'raw_angle_deg': angle, 'age_ms': 20, 'remaining_ms': 200}
    return {'schema': 'smartcart.dual_uwb_view.v1', 'source_mode': 'live', 'state': 'running',
            'drive_authorized': False, 'sensors': {
                'uwb_left': {**base, 'sample_sequence': left_seq},
                'uwb_right': {**base, 'raw_angle_deg': -angle, 'sample_sequence': right_seq}}}


class GeometryMonitorTests(unittest.TestCase):
    def test_center_geometry_distinguishes_hypotheses_without_changing_validity(self):
        monitor = monitor_module.GeometryMonitor()
        monitor.observe(payload())
        result = monitor.summary(15, 0)
        self.assertEqual(result['paired_unique_samples'], 1)
        self.assertLess(result['pair_residual_m']['current']['median'], .001)
        self.assertGreater(result['pair_residual_m']['reversed_sign_hypothesis']['median'], .8)
        self.assertFalse(result['drive_authorized'])

    def test_duplicate_and_one_sided_updates_are_not_counted_as_new_pairs(self):
        monitor = monitor_module.GeometryMonitor()
        for item in (payload(), payload(), payload(2, 1), payload(2, 2)):
            monitor.observe(item)
        result = monitor.summary(15, 0)
        self.assertEqual(result['paired_unique_samples'], 2)
        self.assertEqual(result['sensors']['uwb_left']['unique_live_observations'], 2)
        self.assertEqual(result['sensors']['uwb_right']['unique_live_observations'], 2)
        self.assertEqual(result['successful_polls'], 4)

    def test_invalid_stale_malformed_and_skew_not_fused(self):
        monitor = monitor_module.GeometryMonitor()
        item = payload(); item['sensors']['uwb_left']['valid'] = False; monitor.observe(item)
        item = payload(); item['sensors']['uwb_right']['age_ms'] = 200; monitor.observe(item)
        item = payload(); item['sensors']['uwb_left']['range_m'] = float('nan'); monitor.observe(item)
        item = payload(); item['source_mode'] = 'replay'; monitor.observe(item)
        self.assertEqual(monitor.summary(15, 0)['paired_unique_samples'], 0)
        self.assertEqual(monitor.summary(15, 0)['malformed_polls'], 1)

    def test_new_sequences_after_reset_can_be_counted(self):
        monitor = monitor_module.GeometryMonitor()
        monitor.observe(payload(10, 10)); monitor.observe(payload(1, 1))
        result = monitor.summary(15, 0)
        self.assertEqual(result['paired_unique_samples'], 2)
        self.assertEqual(result['sensors']['uwb_left']['sequence_resets'], 1)

    def test_single_sensor_is_sufficient_only_for_that_side(self):
        monitor = monitor_module.GeometryMonitor()
        item = payload()
        item['sensors']['uwb_left'] = {'state': 'disconnected'}
        monitor.observe(item)
        self.assertTrue(monitor_module.has_required_input(monitor, 'right'))
        self.assertFalse(monitor_module.has_required_input(monitor, 'left'))
        self.assertFalse(monitor_module.has_required_input(monitor, 'both'))
        self.assertEqual(monitor.summary(15, 0)['paired_unique_samples'], 0)
        with self.assertRaises(ValueError):
            monitor_module.has_required_input(monitor, 'unknown')


if __name__ == '__main__':
    unittest.main()
