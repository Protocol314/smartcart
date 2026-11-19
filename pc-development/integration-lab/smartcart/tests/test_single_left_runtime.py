"""오른쪽이 없는 단일 실측 경로의 LiDAR 연관과 만료를 가상으로 확인한다."""
import unittest
from threading import Event

from test_tracking_runtime import mounting_config, status, raw_scan, SECOND, MS
from smartcart_tracking.runtime import LiveTrackingRuntime
from smartcart_tracking.config import TrackingConfig
from smartcart_tracking.view import TrackingStore
from smartcart_sensing.dual_uwb import DualUwbStore
from smartcart_uwb.estimation_config import EstimationConfig
from smartcart_tracking.ros_runtime import _result


class SingleLeftRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.mounting = mounting_config()
        self.uwb = EstimationConfig(range_model="horizontal")
        self.settings = TrackingConfig()
        self.runtime = LiveTrackingRuntime(self.mounting, self.uwb, self.settings, single_left=True)

    def cycle(self, seq):
        at = SECOND + (seq - 1) * 40 * MS
        sample = status(self.mounting, seq, at)
        sample["sensors"]["uwb_right"] = {"state": "error", "connected": False,
            "stream_epoch": 0, "latest": None, "last_error": "disabled_single_left"}
        self.runtime.consume_status(sample, at)
        return self.runtime.consume_scan(raw_scan(), at), sample, at

    def test_acquire_display_and_expire(self):
        store = TrackingStore(self.mounting, self.settings, mode="ros")
        initial = self.runtime.snapshot(SECOND)
        store.publish(initial, now_ns=SECOND, wall_ns=SECOND)
        self.assertFalse(store.snapshot(now_ns=SECOND)["valid"])
        for seq in range(1, 4):
            result, sample, now = self.cycle(seq)
        self.assertTrue(result["uwb_estimate"]["valid"], result)
        self.assertTrue(result["track"]["valid"], result)
        self.assertFalse(result["track"]["drive_authorized"])
        self.assertEqual(_result(result, "live")["uwb_estimate"]["sensor_id"], "uwb_left")
        store.publish(result, now_ns=now, wall_ns=now)
        self.assertTrue(store.snapshot(now_ns=now)["valid"])
        ui = DualUwbStore(self.mounting, self.uwb, stop=Event(), single_left=True)
        ui.consume_status(sample, now)
        self.assertTrue(ui.snapshot(now_ns=now)["valid"])
        expired = self.runtime.snapshot(now + 300 * MS)
        self.assertFalse(expired["track"]["valid"])
        self.assertIsNone(expired["track"]["position_xy_m"])
        self.assertFalse(store.snapshot(now_ns=now + 300 * MS)["valid"])

    def test_left_disconnect_invalidates_target(self):
        for seq in range(1, 4):
            result, sample, now = self.cycle(seq)
        self.assertTrue(result["track"]["valid"])
        left = sample["sensors"]["uwb_left"]
        left.update(state="error", connected=False, last_error="disconnected")
        left["latest"].update(valid=False, invalid_reason="disconnected")
        result = self.runtime.consume_status(sample, now + MS)
        self.assertFalse(result["track"]["valid"])
        self.assertIsNone(result["track"]["position_xy_m"])
        for seq in range(4, 9):
            result, _, _ = self.cycle(seq)
        self.assertTrue(result["track"]["valid"], result)

    def test_invalid_lidar_and_unknown_range_do_not_authorize(self):
        for seq in range(1, 4):
            result, sample, now = self.cycle(seq)
        result = self.runtime.consume_scan({"ranges": []}, now + MS)
        self.assertFalse(result["track"]["valid"])
        other = LiveTrackingRuntime(self.mounting, EstimationConfig(), self.settings, single_left=True)
        result = other.consume_status(sample, now)
        self.assertFalse(result["uwb_estimate"]["valid"])
        self.assertFalse(result["drive_authorized"])
