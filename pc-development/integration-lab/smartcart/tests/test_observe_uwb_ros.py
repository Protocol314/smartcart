"""실제 ROS·장치를 열지 않고 구독 집계와 무효 사유 처리를 검증한다."""
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace
import json
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = spec_from_file_location('observe_uwb_ros', ROOT / 'scripts/observe_uwb_ros.py')
monitor = module_from_spec(SPEC)
SPEC.loader.exec_module(monitor)


def observation(sequence=1, *, role='uwb_left', valid=True, reason='', session='A', mode='live'):
    return SimpleNamespace(sensor_id=role, source_mode=mode, sample_sequence=sequence,
                           adapter_session=session, valid=valid, invalid_reason=reason)


class ObserveUwbRosTests(unittest.TestCase):
    def test_valid_invalid_sequence_gap_and_duplicate(self):
        counter = monitor.CounterState()
        counter.observe('uwb_left', observation(1))
        counter.observe('uwb_left', observation(3, valid=False, reason='bad_frame:checksum'))
        counter.observe('uwb_left', observation(3))
        counter.observe('uwb_left', observation(1, session='B'))
        row = counter.summary(10, True)['sensors']['uwb_left']
        self.assertEqual((row['observations_received'], row['valid_observations'],
                          row['invalid_observations'], row['valid_observation_pct']), (3, 2, 1, 66.67))
        self.assertEqual(row['invalid_reasons'], {'bad_frame:checksum': 1})
        self.assertEqual(row['missing_observation_sequence_estimate'], 1)
        self.assertEqual(row['repeated_or_reordered'], 1)
        self.assertEqual(row['session_changes'], 1)

    def test_status_and_invalid_data_are_bounded_and_redacted(self):
        counter = monitor.CounterState()
        counter.observe('uwb_left', observation(1, valid=False, reason='private_person_123'))
        counter.observe('uwb_left', observation(2, role='uwb_right'))
        counter.observe('uwb_left', observation(3, mode='replay'))
        status = {'source_mode': 'live', 'sensors': {
            'uwb_left': {'state': 'live', 'connected': True},
            'uwb_right': {'state': 'stale', 'connected': True}}}
        counter.status(json.dumps(status))
        counter.status('{' + 'x' * monitor.MAX_STATUS_BYTES)
        status['sensors']['uwb_left']['state'] = []
        counter.status(json.dumps(status))
        result = counter.summary(100, True)
        left, right = result['sensors']['uwb_left'], result['sensors']['uwb_right']
        self.assertEqual(left['invalid_reasons'], {'other': 1})
        self.assertEqual(left['status_counts'], {'live': 1, 'unknown': 1})
        self.assertEqual(right['status_stale_pct'], 100.0)
        self.assertEqual(result['malformed_status'], 1)
        self.assertEqual(result['ignored_observations'], 2)
        self.assertNotIn('private_person_123', json.dumps(result))
        self.assertFalse(result['port_opened'])
        self.assertFalse(result['drive_authorized'])

    def test_zero_observations_and_duration_validation(self):
        summary = monitor.CounterState().summary(0, False)
        self.assertIsNone(summary['sensors']['uwb_left']['valid_observation_pct'])
        for bad in ('NaN', 'inf', '-1', '601', 'not-a-number'):
            with self.subTest(value=bad), self.assertRaises(Exception):
                monitor.seconds(bad)
        self.assertEqual(monitor.seconds('100'), 100.0)


if __name__ == '__main__':
    unittest.main()
