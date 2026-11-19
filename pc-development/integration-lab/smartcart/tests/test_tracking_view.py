"""실제 원시 캡처 결과로 표시 경계·벽시계 수명·상태 전이를 검증한다."""
from copy import deepcopy
from pathlib import Path
import unittest

from smartcart_tracking.config import load_tracking_config
from smartcart_tracking.pipeline import TrackingPipeline
from smartcart_tracking.replay import track_file
from smartcart_tracking.view import TrackingStore
from smartcart_uwb.config import load_config
from smartcart_uwb.estimation_config import load_estimation_config
from smartcart_uwb.native import NativeParser

ROOT = Path(__file__).resolve().parents[1]
MS = 1_000_000
WALL = 10_000_000_000


class TrackingViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mounting = load_config(ROOT / "config" / "dual_uwb.d5.example.json")
        cls.uwb_settings = load_estimation_config(ROOT / "config" / "uwb_estimation.synthetic.json")
        cls.settings = load_tracking_config(ROOT / "config" / "target_tracking.synthetic.json")
        try:
            parser = NativeParser()
        except OSError as error:
            raise unittest.SkipTest("미리 빌드한 UWB 네이티브 라이브러리가 필요합니다") from error
        parser.close()
        cls.valid = cls.coasting = cls.invalid_uwb = cls.invalid_clusters = None
        with TrackingPipeline(cls.mounting, cls.uwb_settings, cls.settings, session_id="view-fixture") as pipeline:
            for event in track_file(ROOT / "samples" / "target_tracking.synthetic.jsonl", pipeline):
                result = {key: event[key] for key in ("uwb_estimate", "lidar_clusters", "track")}
                result.update(source_mode="replay", drive_authorized=False)
                if cls.invalid_uwb is None and not result["uwb_estimate"]["valid"]:
                    cls.invalid_uwb = deepcopy(result["uwb_estimate"])
                if cls.invalid_clusters is None and not result["lidar_clusters"]["valid"]:
                    cls.invalid_clusters = deepcopy(result["lidar_clusters"])
                if cls.valid is None and result["track"]["valid"]:
                    cls.valid, cls.at = deepcopy(result), event["replay_now_ns"]
                if (cls.coasting is None and result["track"]["state"] == "coasting"
                        and result["track"]["predicted_position_xy_m"] is not None):
                    cls.coasting, cls.coast_at = deepcopy(result), event["replay_now_ns"]
        if any(value is None for value in (cls.valid, cls.coasting, cls.invalid_uwb, cls.invalid_clusters)):
            raise AssertionError("표시 검사의 실제 합성 체크포인트가 없습니다")

    def setUp(self):
        self.store = self.new_store()

    def new_store(self, **kwargs):
        return TrackingStore(self.mounting, self.settings, mode="replay", **kwargs)

    def publish(self, result=None, *, rate=1):
        self.store.publish(deepcopy(self.valid if result is None else result), now_ns=self.at,
                           wall_ns=WALL, clock_rate=rate)
        return self.store.snapshot(now_ns=WALL)

    def assert_rejected(self, result):
        with self.assertRaises(ValueError):
            self.publish(result)
        value = self.store.snapshot(now_ns=WALL)
        self.assertEqual(value["state"], "error")
        self.assertFalse(value["valid"])
        self.assertIsNone(value["result"])

    def test_off_and_initial_waiting_have_no_result_or_drive_permission(self):
        for store, state, source in ((TrackingStore(), "off", "off"), (self.store, "waiting", "replay")):
            value = store.snapshot(now_ns=WALL)
            self.assertEqual(value["state"], state)
            self.assertEqual(value["source_mode"], source)
            self.assertIsNone(value["result"])
            self.assertEqual(value["remaining_ms"], 0)
            self.assertFalse(value["valid"])
            self.assertFalse(value["drive_authorized"])

    def test_actual_valid_result_preserves_selected_cluster_and_source(self):
        value = self.publish()
        self.assertTrue(value["valid"])
        self.assertEqual(value["state"], "running")
        self.assertEqual(value["source_mode"], "replay")
        self.assertTrue(value["provisional"])
        track = value["result"]["track"]
        identifiers = {item["cluster_id"] for item in value["result"]["lidar_clusters"]["clusters"]}
        self.assertIn(track["associated_cluster_id"], identifiers)
        self.assertEqual(value["lidar_transform"]["translation_m"], list(self.mounting.lidar.translation_m))

    def test_valid_track_with_invalid_uwb_is_rejected(self):
        value = deepcopy(self.valid)
        value["uwb_estimate"] = deepcopy(self.invalid_uwb)
        self.assert_rejected(value)

    def test_valid_track_with_invalid_clusters_is_rejected(self):
        value = deepcopy(self.valid)
        value["lidar_clusters"] = deepcopy(self.invalid_clusters)
        self.assert_rejected(value)

    def test_selected_cluster_must_exist_in_the_same_frame(self):
        value = deepcopy(self.valid)
        selected = value["track"]["associated_cluster_id"]
        value["lidar_clusters"]["clusters"] = [item for item in value["lidar_clusters"]["clusters"]
                                                  if item["cluster_id"] != selected]
        self.assert_rejected(value)

    def test_track_deadline_may_not_exceed_uwb_deadline(self):
        value = deepcopy(self.valid)
        value["track"]["valid_until_monotonic_ns"] = value["uwb_estimate"]["valid_until_monotonic_ns"] + 1
        self.assert_rejected(value)

    def test_track_deadline_may_not_exceed_cluster_deadline(self):
        value = deepcopy(self.valid)
        value["lidar_clusters"]["valid_until_monotonic_ns"] = value["track"]["valid_until_monotonic_ns"] - 1
        self.assert_rejected(value)

    def test_track_reference_cannot_be_in_the_future(self):
        value = deepcopy(self.valid)
        value["track"]["reference_monotonic_ns"] = self.at + 1
        self.assert_rejected(value)

    def test_track_reference_must_match_the_selected_scan(self):
        value = deepcopy(self.valid)
        value["track"]["reference_monotonic_ns"] -= 1
        self.assert_rejected(value)

    def test_lidar_input_sequence_and_session_must_match(self):
        for key, replacement in (("sequence", self.valid["track"]["inputs"]["lidar"]["sequence"] + 1),
                                 ("adapter_session", "different-session")):
            with self.subTest(key=key):
                self.store = self.new_store()
                value = deepcopy(self.valid)
                value["track"]["inputs"]["lidar"][key] = replacement
                self.assert_rejected(value)

    def test_uwb_input_reference_must_match_the_actual_estimate(self):
        value = deepcopy(self.valid)
        value["track"]["inputs"]["uwb"]["reference_monotonic_ns"] += 1
        self.assert_rejected(value)

    def test_live_input_cannot_enter_replay_view(self):
        value = deepcopy(self.valid)
        value["uwb_estimate"]["source_mode"] = "live"
        self.assert_rejected(value)

    def test_replay_speed_scales_ttl_but_never_extends_transport_deadline(self):
        delta = self.valid["track"]["valid_until_monotonic_ns"] - self.at
        self.assertGreater(delta, 0)
        for rate in (.05, .25, 1, 4):
            with self.subTest(rate=rate):
                self.store = self.new_store()
                value = self.publish(rate=rate)
                expected_ms = min(750, delta / rate / MS)
                self.assertAlmostEqual(value["remaining_ms"], expected_ms, places=6)
                expired = self.store.snapshot(now_ns=WALL + int(expected_ms * MS) + 1)
                self.assertFalse(expired["valid"])
                self.assertIsNone(expired["result"]["track"]["position_xy_m"])

    def test_transport_timeout_is_sticky_even_if_an_earlier_read_time_is_reused(self):
        self.publish()
        stale = self.store.snapshot(now_ns=WALL + 751 * MS)
        self.assertEqual(stale["state"], "stale")
        earlier = self.store.snapshot(now_ns=WALL + MS)
        self.assertEqual(earlier["state"], "stale")
        self.assertFalse(earlier["valid"])
        self.assertEqual(earlier["result"]["lidar_clusters"]["clusters"], [])
        self.assertIsNone(earlier["result"]["track"]["predicted_position_xy_m"])

    def test_expired_uwb_and_clusters_are_removed_from_display_payload(self):
        self.publish()
        value = self.store.snapshot(now_ns=WALL + 500 * MS)
        self.assertFalse(value["valid"])
        self.assertFalse(value["result"]["uwb_estimate"]["valid"])
        self.assertIsNone(value["result"]["uwb_estimate"]["position_xy_m"])
        self.assertFalse(value["result"]["lidar_clusters"]["valid"])
        self.assertEqual(value["result"]["lidar_clusters"]["clusters"], [])

    def test_finish_clears_all_measurement_and_prediction_markers_immediately(self):
        self.publish()
        self.store.finish()
        value = self.store.snapshot(now_ns=WALL)
        self.assertEqual(value["state"], "ended")
        self.assertFalse(value["valid"])
        self.assertEqual(value["remaining_ms"], 0)
        self.assertEqual(value["cluster_remaining_ms"], 0)
        self.assertEqual(value["prediction_remaining_ms"], 0)
        self.assertIsNone(value["result"]["track"]["track_id"])
        self.assertIsNone(value["result"]["track"]["position_xy_m"])

    def test_error_and_begin_start_a_new_empty_lifecycle_without_resetting_sequence(self):
        initial = self.publish()
        self.store.set_error("fixture_error")
        error = self.store.snapshot(now_ns=WALL)
        self.assertEqual(error["state"], "error")
        self.assertIsNone(error["result"])
        self.store.begin(replay={"iteration": 2, "speed": .25, "event_index": 0})
        waiting = self.store.snapshot(now_ns=WALL)
        self.assertEqual(waiting["state"], "waiting")
        self.assertIsNone(waiting["result"])
        self.assertGreater(waiting["sequence"], initial["sequence"])
        self.assertEqual(waiting["replay"]["iteration"], 2)
        self.assertTrue(self.publish()["valid"])

    def test_publish_clock_rollback_clears_previously_valid_display(self):
        for kwargs in ({"now_ns": self.at - 1, "wall_ns": WALL + 1},
                       {"now_ns": self.at + 1, "wall_ns": WALL - 1}):
            with self.subTest(kwargs=kwargs):
                self.store = self.new_store()
                self.publish()
                with self.assertRaises(ValueError):
                    self.store.publish(deepcopy(self.valid), **kwargs)
                self.assertIsNone(self.store.snapshot(now_ns=WALL)["result"])

    def test_returned_mutable_data_and_original_input_do_not_change_stored_result(self):
        original = deepcopy(self.valid)
        self.store.publish(original, now_ns=self.at, wall_ns=WALL)
        original["track"]["position_xy_m"][0] = 88
        value = self.store.snapshot(now_ns=WALL)
        value["result"]["lidar_clusters"]["clusters"].clear()
        again = self.store.snapshot(now_ns=WALL)
        self.assertEqual(again["result"]["track"]["position_xy_m"], self.valid["track"]["position_xy_m"])
        self.assertTrue(again["result"]["lidar_clusters"]["clusters"])

    def test_coasting_prediction_is_separate_and_expires_at_earliest_input_limit(self):
        self.store.publish(deepcopy(self.coasting), now_ns=self.coast_at, wall_ns=WALL)
        value = self.store.snapshot(now_ns=WALL)
        self.assertFalse(value["valid"])
        self.assertIsNone(value["result"]["track"]["position_xy_m"])
        self.assertIsNotNone(value["result"]["track"]["predicted_position_xy_m"])
        self.assertGreater(value["prediction_remaining_ms"], 0)
        deadline = min(self.coasting["uwb_estimate"]["valid_until_monotonic_ns"],
                       self.coasting["lidar_clusters"]["valid_until_monotonic_ns"],
                       self.coasting["track"]["reference_monotonic_ns"] + self.settings.occlusion_timeout_ms * MS)
        expired = self.store.snapshot(now_ns=WALL + deadline - self.coast_at + 1)
        self.assertIsNone(expired["result"]["track"]["predicted_position_xy_m"])
        self.assertEqual(expired["prediction_remaining_ms"], 0)


if __name__ == "__main__":
    unittest.main()
