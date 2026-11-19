import json
import math
import unittest
from smartcart_radar.core import normalise_scan, project_scan, ScanStore


def make_scan(ranges=None, **overrides):
    values = ranges if ranges is not None else [1, 1, 1, 1, 1]
    data = dict(angle_min=-math.pi, angle_max=math.pi, angle_increment=math.pi / 2,
                range_min=0.05, range_max=16, scan_time=0.1, ranges=values)
    data.update(overrides)
    return normalise_scan(data)


class CoreTests(unittest.TestCase):
    def test_expected_count(self):
        self.assertEqual(make_scan()['stats']['expected_rays'], 5)

    def test_finite_json(self):
        scan = make_scan([float('inf'), float('nan'), -float('inf'), None, 1])
        self.assertEqual(scan['ranges'], [None, None, None, None, 1])
        json.dumps(scan, allow_nan=False)

    def test_filter_below_above(self):
        scan = make_scan([.01, .05, 16, 17, None])
        self.assertEqual(scan['stats']['below_min_rays'], 1)
        self.assertEqual(scan['stats']['above_max_rays'], 1)
        self.assertEqual(scan['stats']['valid_sensor_rays'], 2)

    def test_sensor_minimum_float_not_loosened(self):
        scan = make_scan([0.05], angle_min=0, angle_max=0, range_min=0.05000000074505806)
        self.assertEqual(scan['stats']['valid_sensor_rays'], 0)

    def test_no_return_is_not_a_boundary_point(self):
        scan = make_scan([None] * 5)
        self.assertEqual(project_scan(scan)['points'], [])

    def test_truncation_preserves_angles(self):
        scan = make_scan([1, 2, '...'])
        self.assertTrue(scan['truncated'])
        points = project_scan(scan)['points']
        self.assertAlmostEqual(points[0]['x'], -1)
        self.assertAlmostEqual(points[1]['y'], -2)
        self.assertEqual(len(scan['ranges']), 2)

    def test_middle_marker_rejected(self):
        with self.assertRaises(ValueError):
            make_scan([1, '...', 2])

    def test_short_array_without_marker(self):
        self.assertTrue(make_scan([1])['truncated'])

    def test_arbitrary_array_text_rejected(self):
        with self.assertRaises(ValueError):
            make_scan(['bad'])

    def test_zero_increment_rejected(self):
        with self.assertRaises(ValueError):
            make_scan(angle_increment=0)

    def test_inconsistent_direction_rejected(self):
        with self.assertRaises(ValueError):
            make_scan(angle_increment=-.1)

    def test_negative_increment_supported(self):
        scan = make_scan(angle_min=math.pi, angle_max=-math.pi, angle_increment=-math.pi / 2)
        self.assertEqual(scan['stats']['expected_rays'], 5)

    def test_huge_array_geometry_rejected(self):
        with self.assertRaises(ValueError):
            make_scan(angle_increment=1e-10)

    def test_nonfinite_metadata_rejected(self):
        with self.assertRaises(ValueError):
            make_scan(angle_min=float('nan'))

    def test_front_cart_axes(self):
        scan = make_scan([2], angle_min=0, angle_max=0)
        point = project_scan(scan)['points'][0]
        self.assertAlmostEqual(point['x'], 2)
        self.assertAlmostEqual(point['y'], 0)
        self.assertEqual(project_scan(scan)['sectors']['front']['count'], 1)

    def test_left_is_positive_y(self):
        scan = make_scan([2], angle_min=math.pi/2, angle_max=math.pi/2)
        point = project_scan(scan)['points'][0]
        self.assertAlmostEqual(point['y'], 2)
        self.assertEqual(project_scan(scan)['sectors']['left']['count'], 1)

    def test_yaw_counterclockwise(self):
        scan = make_scan([2], angle_min=0, angle_max=0)
        point = project_scan(scan, yaw_deg=90)['points'][0]
        self.assertAlmostEqual(point['x'], 0)
        self.assertAlmostEqual(point['y'], 2)

    def test_translation(self):
        scan = make_scan([2], angle_min=0, angle_max=0)
        point = project_scan(scan, offset_x=.4, offset_y=.2)['points'][0]
        self.assertAlmostEqual(point['x'], 2.4)
        self.assertAlmostEqual(point['y'], .2)

    def test_crop_after_cart_transform(self):
        scan = make_scan([4.8], angle_min=0, angle_max=0)
        self.assertEqual(len(project_scan(scan, offset_x=.3)['points']), 0)
        self.assertEqual(scan['ranges'], [4.8])

    def test_large_range_can_enter_cart_crop(self):
        scan = make_scan([5.1], angle_min=math.pi, angle_max=math.pi)
        self.assertEqual(len(project_scan(scan, offset_x=.2)['points']), 1)

    def test_projection_does_not_mutate_raw_data(self):
        scan = make_scan([7] * 5)
        raw = json.dumps(scan)
        project_scan(scan, radius=5)
        self.assertEqual(json.dumps(scan), raw)

    def test_invalid_projection(self):
        with self.assertRaises(ValueError):
            project_scan(make_scan(), radius=0)

    def test_unknown_sector_is_null(self):
        result = project_scan(make_scan([None] * 5))
        self.assertIsNone(result['sectors']['front']['min_m'])

    def test_capture_wrapper(self):
        scan = make_scan()
        self.assertEqual(normalise_scan({'scan': scan})['ranges'], scan['ranges'])

    def test_intensity_mismatch_warning(self):
        self.assertIn('INTENSITY_COUNT_MISMATCH', make_scan(intensities=[1])['warnings'])

    def test_original_source_recorded(self):
        scan = make_scan(source={'kind': 'ros'})
        self.assertEqual(scan['source']['original_kind'], 'ros')


class StoreTests(unittest.TestCase):
    def test_waiting(self):
        self.assertEqual(ScanStore().snapshot()['state'], 'waiting')

    def test_live_then_stale(self):
        store = ScanStore(); store.publish(make_scan(), now=10)
        self.assertEqual(store.snapshot(now=11)['state'], 'live')
        self.assertEqual(store.snapshot(now=11.6)['state'], 'stale')

    def test_monotonic_not_ros_stamp(self):
        store = ScanStore(); store.publish(make_scan(header={'stamp': {'sec': 1}}), now=10)
        self.assertEqual(store.snapshot(now=10.1)['state'], 'live')

    def test_latest_not_accumulated(self):
        store = ScanStore()
        for i in range(25): store.publish(make_scan([i+1] * 5), now=i*.1)
        snapshot = store.snapshot(now=2.4)
        self.assertEqual(snapshot['scan']['sequence'], 25)
        self.assertEqual(snapshot['scan']['ranges'], [25]*5)
        self.assertAlmostEqual(snapshot['receive_hz'], 10)

    def test_rate_decays(self):
        store = ScanStore()
        for i in range(10): store.publish(make_scan(), now=i*.1)
        self.assertEqual(store.snapshot(now=10)['receive_hz'], 0)

    def test_errors_not_hidden(self):
        store = ScanStore(); store.set_error('bad', rejected=True)
        self.assertEqual(store.snapshot()['state'], 'error')
        self.assertEqual(store.snapshot()['rejected_scans'], 1)

    def test_recovery_on_valid_frame(self):
        store = ScanStore(); store.set_error('bad'); store.publish(make_scan())
        self.assertEqual(store.snapshot()['state'], 'live')

    def test_demo_never_live(self):
        store = ScanStore('demo'); store.publish(make_scan())
        self.assertEqual(store.snapshot()['state'], 'synthetic')

    def test_replay_end(self):
        store = ScanStore('replay'); store.publish(make_scan()); store.finish()
        self.assertEqual(store.snapshot()['state'], 'ended')
