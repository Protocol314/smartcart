"""LiDAR 연속점·경계 연결·원래 인덱스·시간 오차와 상한을 합성 스캔으로 검사한다."""
from copy import deepcopy
from dataclasses import replace
import math
from pathlib import Path
import unittest

from smartcart_tracking.clusters import CLUSTERS_SCHEMA, OBSERVATION_SCHEMA, cluster_scan
from smartcart_tracking.config import TrackingConfig
from smartcart_uwb.config import load_config


EXAMPLE = Path(__file__).resolve().parents[1] / "config" / "dual_uwb.d5.example.json"


def scan(ranges, step=.02, angle_min=0, scan_time=.02, time_increment=0):
    return {"angle_min": angle_min, "angle_max": angle_min + (len(ranges) - 1) * step,
            "angle_increment": step, "range_min": .05, "range_max": 16,
            "ranges": list(ranges), "scan_time": scan_time, "time_increment": time_increment}


def observation(raw, config):
    return {"schema": OBSERVATION_SCHEMA, "sensor_id": "lidar_center", "frame_id": config.lidar.frame_id,
            "adapter_session": "test-lidar", "stream_epoch": 0, "sample_sequence": 1,
            "source_mode": "replay", "timestamp_source": "HOST_RX", "rx_monotonic_ns": 1_000_000_000,
            "valid_until_monotonic_ns": 1_300_000_000, "valid": True, "invalid_reason": None, "scan": raw}


class ClusterTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(EXAMPLE)
        self.config = replace(self.config, lidar=replace(self.config.lidar, translation_m=(0, 0, .2)))
        self.settings = TrackingConfig()

    def run_scan(self, raw, settings=None, config=None):
        config = self.config if config is None else config
        return cluster_scan(observation(raw, config), config, self.settings if settings is None else settings)

    def test_features_and_exact_output_contract(self):
        result = self.run_scan(scan([2, 2, 2]))
        self.assertTrue(result["valid"])
        self.assertEqual(result["schema"], CLUSTERS_SCHEMA)
        self.assertEqual(set(result), {"schema", "sensor_id", "frame_id", "input_frame_id", "adapter_session",
            "stream_epoch", "sample_sequence", "source_mode", "timestamp_source", "rx_monotonic_ns",
            "valid_until_monotonic_ns", "valid", "invalid_reason", "clusters", "drive_authorized", "provisional"})
        feature = result["clusters"][0]
        self.assertEqual(set(feature), {"cluster_id", "centroid_xy_m", "radius_m", "width_m", "point_count", "first_ray", "last_ray", "error_radius_m"})
        self.assertEqual((feature["point_count"], feature["first_ray"], feature["last_ray"]), (3, 0, 2))
        self.assertGreater(feature["width_m"], .04)
        self.assertFalse(result["drive_authorized"])
        self.assertTrue(result["provisional"])

    def test_mount_yaw_translation_and_negative_step(self):
        raw = scan([2, 2, 2], angle_min=.02, step=-.02)
        plain = self.run_scan(raw)["clusters"][0]["centroid_xy_m"]
        config = replace(self.config, lidar=replace(self.config.lidar, translation_m=(.4, -.2, .2), yaw_deg=90))
        rotated = self.run_scan(raw, config=config)["clusters"][0]["centroid_xy_m"]
        self.assertAlmostEqual(rotated[0], .4 - plain[1])
        self.assertAlmostEqual(rotated[1], -.2 + plain[0])

    def test_null_nonfinite_and_invalid_ranges_break_original_indices(self):
        for missing in (None, float("nan"), float("inf"), -float("inf"), .01, 17, 11):
            with self.subTest(missing=missing):
                result = self.run_scan(scan([2, 2, 2, missing, 2, 2, 2]))
                self.assertTrue(result["valid"])
                self.assertEqual([(g["first_ray"], g["last_ray"]) for g in result["clusters"]], [(0, 2), (4, 6)])

    def test_range_jump_breaks_contiguous_rays(self):
        result = self.run_scan(scan([1.5, 1.5, 1.5, 3, 3, 3]))
        self.assertEqual([g["point_count"] for g in result["clusters"]], [3, 3])

    def test_noise_inside_adjacency_gate_stays_one_group(self):
        result = self.run_scan(scan([2, 2.02, 1.98, 2.01, 2]))
        self.assertTrue(result["valid"])
        self.assertEqual(len(result["clusters"]), 1)
        self.assertEqual(result["clusters"][0]["point_count"], 5)

    def test_full_circle_exclusive_endpoint_wrap_merge(self):
        ranges = [1.5, 1.5] + [None] * 32 + [1.5, 1.5]
        result = self.run_scan(scan(ranges, step=2 * math.pi / 36, angle_min=-math.pi))
        self.assertEqual(len(result["clusters"]), 1)
        feature = result["clusters"][0]
        self.assertEqual((feature["first_ray"], feature["last_ray"], feature["point_count"]), (34, 1, 4))

    def test_full_circle_inclusive_endpoint_wrap_merge(self):
        ranges = [1.5, 1.5] + [None] * 33 + [1.5, 1.5]
        result = self.run_scan(scan(ranges, step=2 * math.pi / 36, angle_min=-math.pi))
        self.assertTrue(result["valid"])
        self.assertEqual(len(result["clusters"]), 1)
        self.assertEqual(result["clusters"][0]["point_count"], 4)

    def test_partial_scan_or_invalid_boundary_never_wraps(self):
        partial = [1.5, 1.5] + [None] * 31 + [1.5, 1.5]
        full_invalid = [None, 1.5] + [None] * 32 + [1.5, 1.5]
        for values in (partial, full_invalid):
            with self.subTest(count=len(values)):
                result = self.run_scan(scan(values, step=2 * math.pi / 36, angle_min=-math.pi))
                self.assertTrue(result["valid"])
                self.assertEqual(result["clusters"], [])

    def test_geometrically_distant_circle_boundaries_never_merge(self):
        values = [1, 1] + [None] * 32 + [3, 3]
        result = self.run_scan(scan(values, step=2 * math.pi / 36, angle_min=-math.pi))
        self.assertTrue(result["valid"])
        self.assertEqual(result["clusters"], [])

    def test_wall_and_tiny_or_too_many_points_filtered(self):
        angles = [-math.pi / 4 + i * math.pi / 198 for i in range(100)]
        wall = scan([2 / math.cos(a) for a in angles], angle_min=angles[0], step=math.pi / 198)
        self.assertEqual(self.run_scan(wall)["clusters"], [])
        self.assertEqual(self.run_scan(scan([.1, .1, .1], step=.001))["clusters"], [])
        limited = replace(self.settings, max_cluster_points=4)
        self.assertEqual(self.run_scan(scan([2] * 5), limited)["clusters"], [])
        self.assertEqual(self.run_scan(scan([2, 2]))["clusters"], [])

    def test_cluster_count_overflow_invalidates_whole_frame(self):
        result = self.run_scan(scan([2, 2, 2, None, 2, 2, 2]), replace(self.settings, max_clusters=1))
        self.assertFalse(result["valid"])
        self.assertEqual(result["invalid_reason"], "cluster_overflow")
        self.assertEqual(result["clusters"], [])

    def test_malformed_range_types_or_truncated_geometry_rejected(self):
        cases = [scan([2, True, 2]), scan([2, "2", 2]), scan([2, "...", 2]),
                 scan([2, 2, 2]) | {"angle_max": .06}, scan([2, 2, 2]) | {"truncated": True},
                 scan([2, 2, 2]) | {"header": {}}, scan([2, 10 ** 1000, 2])]
        for raw in cases:
            with self.subTest(raw=str(raw)[:60]):
                result = self.run_scan(raw)
                self.assertFalse(result["valid"])
                self.assertEqual(result["clusters"], [])

    def test_excessive_input_and_nonfinite_metadata_fail_closed(self):
        cases = [scan([2] * 8193, step=.0001), scan([2, 2, 2]) | {"angle_increment": 0},
                 scan([2, 2, 2]) | {"angle_min": float("nan")},
                 scan([2, 2, 2]) | {"range_min": True}, scan([2, 2, 2]) | {"range_max": "16"}]
        for raw in cases:
            result = self.run_scan(raw)
            self.assertFalse(result["valid"])
            self.assertEqual(result["clusters"], [])

    def test_duration_bound_per_ray_span_and_zero_fallback_error(self):
        for raw in (scan([2, 2, 2], scan_time=.20001), scan([2, 2, 2], time_increment=.10001),
                    scan([2, 2, 2], scan_time=-.01), scan([2, 2, 2], time_increment=-.01)):
            result = self.run_scan(raw)
            self.assertFalse(result["valid"])
        result = self.run_scan(scan([2, 2, 2], scan_time=0, time_increment=0))
        expected = .03 + .03 + 4 * math.sin(math.radians(3) / 2) + 3 * (.2 + .05)
        self.assertAlmostEqual(result["clusters"][0]["error_radius_m"], expected)
        self.assertGreater(result["clusters"][0]["error_radius_m"], self.run_scan(scan([2, 2, 2]))["clusters"][0]["error_radius_m"])

    def test_empty_valid_scan_and_waiting_observation(self):
        result = self.run_scan(scan([None] * 20))
        self.assertTrue(result["valid"])
        self.assertEqual(result["clusters"], [])
        incoming = observation(None, self.config) | {"valid": False, "invalid_reason": "waiting_for_scan",
                                                    "sample_sequence": 0, "rx_monotonic_ns": 0, "valid_until_monotonic_ns": 0}
        result = cluster_scan(incoming, self.config, self.settings)
        self.assertFalse(result["valid"])
        self.assertEqual(result["clusters"], [])
        self.assertEqual(result["invalid_reason"], "waiting_for_scan")
        self.assertEqual(result["adapter_session"], incoming["adapter_session"])

    def test_strict_observation_metadata_and_no_raw_scan_in_output(self):
        incoming = observation(scan([2, 2, 2]), self.config)
        for key, value in (("schema", "other"), ("sensor_id", "uwb_left"), ("frame_id", "other"),
                           ("adapter_session", "bad\nvalue"), ("stream_epoch", True),
                           ("sample_sequence", 0), ("source_mode", "synthetic"),
                           ("timestamp_source", "DEVICE"), ("rx_monotonic_ns", -1),
                           ("valid_until_monotonic_ns", 1_300_000_001), ("valid", 1),
                           ("invalid_reason", "unexpected")):
            with self.subTest(key=key):
                result = cluster_scan(incoming | {key: value}, self.config, self.settings)
                self.assertFalse(result["valid"])
                self.assertEqual(result["clusters"], [])
                self.assertNotIn("scan", result)
        self.assertFalse(cluster_scan(incoming | {"extra": 1}, self.config, self.settings)["valid"])

    def test_input_immutability_radius_encloses_measured_points(self):
        raw = scan([2, 2.03, 1.98, 2])
        incoming = observation(raw, self.config)
        before = deepcopy(incoming)
        result = cluster_scan(incoming, self.config, self.settings)
        feature = result["clusters"][0]
        cx, cy = feature["centroid_xy_m"]
        for index, distance in enumerate(raw["ranges"]):
            angle = raw["angle_min"] + index * raw["angle_increment"]
            self.assertLessEqual(math.hypot(distance * math.cos(angle) - cx,
                                           distance * math.sin(angle) - cy), feature["radius_m"] + 1e-12)
        self.assertEqual(incoming, before)

    def test_invalid_outer_inputs_never_leak_exception(self):
        for incoming in (None, [], "invalid", {}, {"ranges": []}):
            result = cluster_scan(incoming, self.config, self.settings)
            self.assertFalse(result["valid"])
            self.assertEqual(result["clusters"], [])
        self.assertEqual(cluster_scan({}, None, self.settings)["invalid_reason"], "invalid_settings")


if __name__ == "__main__":
    unittest.main()
