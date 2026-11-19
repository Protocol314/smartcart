"""센서 손실과 출처를 독립적으로 판정하는 통합 상태 회귀 검사."""
import math
import threading
import unittest

from smartcart_radar.core import ScanStore, normalise_scan
from smartcart_radar.sources import synthetic_scan
from smartcart_sensing.ros import diagnostic_data, lidar_diagnostic_data
from smartcart_sensing.service import SensingStore
from smartcart_sensing.uwb import UwbStore, synthetic_line


def simulated_ros_scan(ranges=None):
    """실물 연결 없는 ROS 계약용 가상 자료다. 물리 센서 정상의 근거가 아니다."""
    return normalise_scan({
        "header": {"frame_id": "synthetic_test_laser"},
        "angle_min": -math.pi, "angle_max": math.pi, "angle_increment": math.pi,
        "range_min": .05, "range_max": 16, "ranges": [1, 2, 3] if ranges is None else ranges,
    }, kind="ros", label="가상 ROS 계약 테스트")


class SensingStoreTests(unittest.TestCase):
    def stores(self, mode="ros"):
        lidar = ScanStore(mode)
        uwb = UwbStore("demo" if mode == "demo" else "serial", stop=lidar.stop)
        return lidar, uwb, SensingStore(lidar, uwb, mode)

    def test_waiting_never_ready(self):
        _, _, store = self.stores()
        self.assertEqual(store.snapshot(now=10)["sensing"]["state"], "waiting")
        self.assertFalse(store.snapshot(now=10)["sensing"]["ready"])

    def test_independent_staleness_and_recovery(self):
        lidar, uwb, store = self.stores()
        lidar.publish(simulated_ros_scan(), now=10)
        uwb.publish_line(synthetic_line(1, 1), now=10)
        self.assertTrue(store.snapshot(now=10.2)["sensing"]["ready"])
        stale = store.snapshot(now=10.6)
        self.assertEqual(stale["state"], "live")
        self.assertEqual(stale["uwb"]["state"], "stale")
        self.assertIsNone(stale["uwb"]["measurement"]["position"])
        self.assertFalse(stale["sensing"]["ready"])
        uwb.publish_line(synthetic_line(2, 2), now=11.6)
        lidar_stale = store.snapshot(now=11.7)
        self.assertEqual(lidar_stale["state"], "stale")
        self.assertEqual(lidar_stale["uwb"]["state"], "live")
        self.assertFalse(lidar_stale["sensing"]["ready"])
        lidar.publish(simulated_ros_scan(), now=11.7)
        self.assertTrue(store.snapshot(now=11.8)["sensing"]["ready"])

    def test_bad_identity_never_ready(self):
        lidar, uwb, store = self.stores()
        lidar.publish(simulated_ros_scan(), now=10)
        row = synthetic_line(1, 1).split(",")
        row[5] = "1"
        uwb.publish_line(",".join(row), now=10)
        snapshot = store.snapshot(now=10.1)
        self.assertFalse(snapshot["sensing"]["ready"])
        self.assertIsNone(snapshot["uwb"]["measurement"]["position"])

    def test_rejected_input_hides_previous_position(self):
        lidar, uwb, store = self.stores()
        lidar.publish(simulated_ros_scan(), now=10)
        uwb.publish_line(synthetic_line(1, 1), now=10)
        uwb.publish_line("C_KEY_DIAG_V3,broken", now=10.1)
        snapshot = store.snapshot(now=10.2)
        self.assertFalse(snapshot["sensing"]["ready"])
        self.assertEqual(snapshot["uwb"]["state"], "error")
        self.assertIsNone(snapshot["uwb"]["measurement"]["position"])

    def test_demo_is_never_operational_ready(self):
        lidar, uwb, store = self.stores("demo")
        lidar.publish(synthetic_scan(0), now=10)
        uwb.publish_line(synthetic_line(1, 1), now=10)
        snapshot = store.snapshot(now=10.1)
        self.assertEqual(snapshot["sensing"]["state"], "synthetic")
        self.assertFalse(snapshot["sensing"]["ready"])
        self.assertTrue(snapshot["uwb"]["synthetic"])

    def test_no_valid_lidar_returns_never_ready(self):
        for ranges in ([None, None, None], [0, .01, 20]):
            with self.subTest(ranges=ranges):
                lidar, uwb, store = self.stores()
                lidar.publish(simulated_ros_scan(ranges), now=10)
                uwb.publish_line(synthetic_line(1, 1), now=10)
                snapshot = store.snapshot(now=10.1)
                self.assertEqual(snapshot["state"], "live")
                self.assertFalse(snapshot["sensing"]["lidar_valid"])
                self.assertTrue(snapshot["sensing"]["uwb_valid"])
                self.assertFalse(snapshot["sensing"]["ready"])
                self.assertEqual(snapshot["sensing"]["state"], "degraded")
                self.assertEqual(lidar_diagnostic_data(snapshot)["level"], 2)

    def test_explicit_synthetic_scan_in_ros_store_is_never_real(self):
        lidar, uwb, store = self.stores()
        lidar.publish(synthetic_scan(0), now=10)
        uwb.publish_line(synthetic_line(1, 1), now=10)
        snapshot = store.snapshot(now=10.1)
        self.assertEqual(snapshot["state"], "live")
        self.assertEqual(snapshot["source_mode"], "ros")
        self.assertTrue(snapshot["sensing"]["synthetic"])
        self.assertFalse(snapshot["sensing"]["lidar_valid"])
        self.assertFalse(snapshot["sensing"]["ready"])
        self.assertEqual(snapshot["sensing"]["state"], "synthetic")

    def test_original_synthetic_provenance_cannot_become_ready(self):
        lidar, uwb, store = self.stores()
        scan = simulated_ros_scan()
        scan["source"]["original_kind"] = "synthetic"
        lidar.publish(scan, now=10)
        uwb.publish_line(synthetic_line(1, 1), now=10)
        snapshot = store.snapshot(now=10.1)
        self.assertTrue(snapshot["sensing"]["synthetic"])
        self.assertFalse(snapshot["sensing"]["ready"])
        self.assertEqual(snapshot["sensing"]["state"], "synthetic")

    def test_gui_validity_matches_ros_diagnostics(self):
        lidar, uwb, store = self.stores()
        lidar.publish(simulated_ros_scan(), now=10)
        uwb.publish_line(synthetic_line(1, 1), now=10)
        for now in (10.1, 10.6, 11.6):
            with self.subTest(now=now):
                snapshot = store.snapshot(now=now)
                self.assertEqual(snapshot["sensing"]["lidar_valid"],
                                 lidar_diagnostic_data(snapshot)["level"] == 0)
                self.assertEqual(snapshot["sensing"]["uwb_valid"],
                                 diagnostic_data(snapshot["uwb"])["level"] == 0)

    def test_shared_lifecycle_is_required(self):
        with self.assertRaises(ValueError):
            SensingStore(ScanStore(), UwbStore(stop=threading.Event()))
