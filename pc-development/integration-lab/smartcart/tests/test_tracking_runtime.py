"""ROS·장치 없이 같은 호스트 상태와 타이머의 원래 수명 보존을 검사한다."""

from copy import deepcopy
from pathlib import Path
import unittest

from smartcart_tracking.config import TrackingConfig
from smartcart_tracking.runtime import LiveTrackingRuntime
from smartcart_uwb.collector import STATUS_SCHEMA
from smartcart_uwb.config import load_config
from smartcart_uwb.estimation_config import EstimationConfig
from smartcart_uwb.observation import make_observation

ROOT = Path(__file__).resolve().parents[1]
SECOND = 1_000_000_000
MS = 1_000_000


def mounting_config():
    return load_config(ROOT / "config" / "dual_uwb.d5.example.json")


def status(config, sequence=1, at=SECOND):
    sensors = {}
    for sensor_id in ("uwb_left", "uwb_right"):
        observation = make_observation(
            sensor=config.sensors[sensor_id], frame={"sequence": sequence & 255, "tag_address": 0x6E19,
                "angle_deg": 0, "distance_cm": 200, "first_path_power": 0, "rx_level": 0},
            allowed_tag_address=config.allowed_tag_address, logical_target_id=config.logical_target_id,
            sample_sequence=sequence, stream_epoch=0, adapter_session="runtime-live", source_mode="live",
            rx_ns=at, source_rx_ns=None, valid_until_ns=at + config.timeout_ms * MS)
        sensors[sensor_id] = {"state": "live", "connected": True, "stream_epoch": 0,
                              "latest": observation, "last_error": None}
    return {"schema": STATUS_SCHEMA, "adapter_session": "runtime-live", "source_mode": "live",
            "drive_authorized": False, "sensors": sensors}


def raw_scan():
    return {"angle_min": -.08, "angle_max": .08, "angle_increment": .04,
            "range_min": .05, "range_max": 10, "ranges": [2.0] * 5,
            "scan_time": .05, "time_increment": .01}


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.mounting = mounting_config()
        self.settings = TrackingConfig()
        self.runtime = LiveTrackingRuntime(self.mounting, EstimationConfig(range_model="horizontal"), self.settings)

    def cycle(self, sequence):
        at = SECOND + (sequence - 1) * 40 * MS
        self.runtime.consume_status(status(self.mounting, sequence, at), at)
        return self.runtime.consume_scan(raw_scan(), at)

    def acquire(self):
        for sequence in (1, 2, 3):
            result = self.cycle(sequence)
        self.assertTrue(result["track"]["valid"])
        return result

    def test_startup_has_valid_schemas_and_waiting_without_native_parser_or_ros(self):
        result = self.runtime.snapshot(SECOND)
        self.assertEqual(set(result), {"uwb_estimate", "lidar_clusters", "track", "source_mode", "drive_authorized"})
        self.assertEqual(result["source_mode"], "live")
        self.assertEqual(result["lidar_clusters"]["sample_sequence"], 0)
        self.assertEqual(result["track"]["state"], "waiting")
        self.assertFalse(result["track"]["valid"])
        self.assertFalse(result["drive_authorized"])
        with self.assertRaises(ValueError):
            LiveTrackingRuntime(self.mounting, EstimationConfig(), self.settings, source_mode="replay")

    def test_independent_status_scan_inputs_reach_three_pair_tracking(self):
        result = self.acquire()
        self.assertEqual(result["track"]["confirmation_count"], 3)
        self.assertEqual(result["lidar_clusters"]["source_mode"], "live")
        self.assertTrue(result["track"]["provisional"])
        self.assertFalse(result["track"]["drive_authorized"])

    def test_repeated_old_status_does_not_renew_uwb_or_track_ttl(self):
        first = self.acquire()
        original = status(self.mounting, 3, SECOND + 80 * MS)
        result = self.runtime.consume_status(original, SECOND + 100 * MS)
        self.assertEqual(result["uwb_estimate"]["valid_until_monotonic_ns"], first["uwb_estimate"]["valid_until_monotonic_ns"])
        expired = self.runtime.consume_status(original, SECOND + 280 * MS)
        self.assertFalse(expired["uwb_estimate"]["valid"])
        self.assertIsNone(expired["track"]["position_xy_m"])

    def test_timer_expires_inputs_without_any_new_message(self):
        self.acquire()
        result = self.runtime.snapshot(SECOND + 500 * MS)
        self.assertFalse(result["uwb_estimate"]["valid"])
        self.assertFalse(result["lidar_clusters"]["valid"])
        self.assertFalse(result["track"]["valid"])
        self.assertIsNone(result["track"]["predicted_position_xy_m"])

    def test_malformed_status_immediately_clears_track_and_requires_new_inputs(self):
        first = self.acquire()
        result = self.runtime.consume_status({"unexpected": True}, SECOND + 90 * MS)
        self.assertIsNone(result["track"]["position_xy_m"])
        self.assertIsNone(result["track"]["track_id"])
        for sequence in (4, 5, 6):
            result = self.cycle(sequence)
        self.assertTrue(result["track"]["valid"])
        self.assertNotEqual(first["track"]["track_id"], result["track"]["track_id"])

    def test_disconnection_in_full_status_is_not_lost_by_reading_only_latest_observation(self):
        self.acquire()
        value = status(self.mounting, 3, SECOND + 80 * MS)
        state = value["sensors"]["uwb_right"]
        state.update(state="error", connected=False, last_error="disconnected")
        state["latest"].update(valid=False, invalid_reason="disconnected")
        result = self.runtime.consume_status(value, SECOND + 90 * MS)
        self.assertFalse(result["uwb_estimate"]["valid"])
        self.assertIsNone(result["track"]["track_id"])
        self.assertIsNone(result["track"]["position_xy_m"])

    def test_invalid_scan_and_clock_rollback_do_not_leave_previous_position_visible(self):
        self.acquire()
        result = self.runtime.consume_scan({"ranges": []}, SECOND + 90 * MS)
        self.assertFalse(result["lidar_clusters"]["valid"])
        self.assertIsNone(result["track"]["position_xy_m"])
        with self.assertRaises(ValueError):
            self.runtime.snapshot(SECOND)
        result = self.runtime.snapshot(SECOND + 91 * MS)
        self.assertIsNone(result["track"]["position_xy_m"])

    def test_status_boundary_invalidation_cannot_be_reversed_by_timer(self):
        self.acquire()
        self.runtime.invalidate_status("invalid_status_json")
        result = self.runtime.snapshot(SECOND + 90 * MS)
        self.assertFalse(result["uwb_estimate"]["valid"])
        self.assertIsNone(result["track"]["track_id"])

    def test_copying_latest_results_protects_internal_state(self):
        result = self.acquire()
        result["track"]["valid"] = False
        result["lidar_clusters"]["clusters"].clear()
        again = self.runtime.snapshot(SECOND + 81 * MS)
        self.assertTrue(again["track"]["valid"])
        self.assertEqual(len(again["lidar_clusters"]["clusters"]), 1)

    def test_actual_tracking_store_accepts_initial_and_tracking_results(self):
        from smartcart_tracking.view import TrackingStore
        store = TrackingStore(self.mounting, self.settings, mode="ros")
        store.publish(self.runtime.snapshot(SECOND), now_ns=SECOND, wall_ns=SECOND)
        self.assertEqual(store.snapshot(now_ns=SECOND)["state"], "running")
        result = self.acquire()
        at = SECOND + 80 * MS
        store.publish(result, now_ns=at, wall_ns=at)
        self.assertTrue(store.snapshot(now_ns=at)["valid"])
        self.assertFalse(store.snapshot(now_ns=at + 300 * MS)["valid"])


if __name__ == "__main__":
    unittest.main()
