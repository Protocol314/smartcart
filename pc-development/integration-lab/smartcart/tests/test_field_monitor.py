"""장치 없는 관측 집계·수명·연속성·수동 선언과 위치 비보관을 확인한다."""
from copy import deepcopy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from smartcart_field.monitor import FieldMonitor
from smartcart_tracking.config import TrackingConfig
from smartcart_uwb.config import load_config
from smartcart_uwb.estimation_config import EstimationConfig
from smartcart_uwb.observation import make_observation

ROOT = Path(__file__).resolve().parents[1]
MS = 1_000_000
SECOND = 1_000_000_000


def status(mounting, sequence=1, at=SECOND, *, epoch=0, session="field-test", source="live", right_at=None, tag=0x6E19):
    sensors = {}
    for name in ("uwb_left", "uwb_right"):
        rx = at if name == "uwb_left" or right_at is None else right_at
        frame = {"sequence": sequence & 255, "tag_address": tag, "angle_deg": 0,
                 "distance_cm": 200, "first_path_power": -90, "rx_level": -80}
        value = make_observation(sensor=mounting.sensors[name], frame=frame,
            allowed_tag_address=mounting.allowed_tag_address, logical_target_id=mounting.logical_target_id,
            sample_sequence=sequence, stream_epoch=epoch, adapter_session=session, source_mode=source,
            rx_ns=rx, source_rx_ns=None, valid_until_ns=rx + mounting.timeout_ms * MS)
        sensors[name] = {"state": "live" if value["valid"] else "error", "connected": True,
                         "stream_epoch": epoch, "latest": value, "last_error": value["invalid_reason"]}
    return {"schema": "smartcart.dual_uwb_status.v1", "adapter_session": session,
            "source_mode": source, "drive_authorized": False, "sensors": sensors}


def scan():
    return {"angle_min": -.08, "angle_max": .08, "angle_increment": .04,
            "range_min": .05, "range_max": 10, "ranges": [2.0] * 5,
            "scan_time": .02, "time_increment": .005}


def invalid(value, name, reason, *, connected=True):
    value = deepcopy(value)
    sensor = value["sensors"][name]
    sensor.update(state="stale" if reason == "stale" else "error", connected=connected, last_error=reason)
    sensor["latest"].update(valid=False, invalid_reason=reason)
    return value


class FieldMonitorTests(unittest.TestCase):
    def setUp(self):
        self.mounting = load_config(ROOT / "config/dual_uwb.d5.example.json")
        self.uwb = EstimationConfig()
        self.settings = TrackingConfig()
        self.monitor = FieldMonitor(self.mounting, self.uwb, self.settings, window_size=16)

    def observe_all(self, sequence=1, at=SECOND, source="live"):
        self.assertTrue(self.monitor.observe_status(status(self.mounting, sequence, at, source=source), at))
        self.assertTrue(self.monitor.observe_scan(scan(), at, frame_id=self.mounting.lidar.frame_id))

    def sensor(self, name="uwb_left"):
        return self.monitor.report()["sensors"][name]

    def manual_cycle(self, source="live"):
        self.observe_all(source=source)
        for name in ("uwb_left", "uwb_right", "lidar_center"):
            self.assertTrue(self.monitor.mark(name, "unplug", SECOND + 10 * MS))
        self.monitor.tick(SECOND + 301 * MS)
        for name in ("uwb_left", "uwb_right", "lidar_center"):
            self.assertTrue(self.monitor.mark(name, "replug", SECOND + 301 * MS))
        self.observe_all(2, SECOND + 310 * MS, source=source)

    def test_initial_report_is_untested_without_geometry_calculation(self):
        with patch("smartcart_uwb.estimator.TagEstimator.update", side_effect=AssertionError("금지된 위치 계산")):
            self.observe_all()
        report = self.monitor.report()
        self.assertFalse(report["completion"])
        self.assertFalse(report["physical_acceptance"])
        self.assertTrue(all(item["status"] == "미시험" for item in report["manual_unplug_tests"].values()))
        self.assertEqual(report["thresholds"]["uwb_timeout_ms"], self.mounting.timeout_ms)
        self.assertEqual(report["thresholds"]["max_observation_age_ms"], self.uwb.max_observation_age_ms)
        self.assertIsNone(self.monitor._validator._current)

    def test_heartbeat_does_not_add_samples_pair_statistics_or_refresh_deadline(self):
        value = status(self.mounting)
        self.monitor.observe_status(value, SECOND)
        self.monitor.observe_status(value, SECOND + 100 * MS)
        report = self.monitor.report()
        self.assertEqual(self.sensor()["counts"]["new_samples"], 1)
        self.assertEqual(self.sensor()["counts"]["repeated_sample_heartbeats"], 1)
        self.assertEqual(report["statistics"]["pair_rx_skew_ms"]["count"], 1)
        self.monitor.tick(SECOND + 200 * MS)
        self.assertEqual(self.sensor()["state"], "stale")

    def test_pair_skew_lidar_skew_and_arrival_rate_use_host_receive_times(self):
        self.monitor.observe_status(status(self.mounting, right_at=SECOND + 50 * MS), SECOND + 50 * MS)
        self.monitor.observe_scan(scan(), SECOND + 160 * MS, frame_id=self.mounting.lidar.frame_id)
        report = self.monitor.report()
        self.assertEqual(report["statistics"]["pair_rx_skew_ms"]["p95"], 50)
        self.assertEqual(report["statistics"]["lidar_pair_rx_skew_ms"]["p95"], 110)
        self.assertEqual(report["counts"]["pair_skew_exceeded"], 1)
        self.assertEqual(report["counts"]["lidar_pair_skew_exceeded"], 1)

    def test_bounded_rolling_percentiles_keep_total_count_and_overall_maximum(self):
        for index in range(50):
            self.monitor.observe_scan(scan(), SECOND + index * 20 * MS, frame_id=self.mounting.lidar.frame_id)
        report = self.monitor.report()
        value = report["statistics"]["lidar_center_interval_ms"]
        self.assertEqual((value["count"], value["window_count"], value["window_capacity"]), (49, 16, 16))
        self.assertEqual((value["p50"], value["p95"], value["max_observed"]), (20, 20, 20))
        self.assertEqual(self.sensor("lidar_center")["median_arrival_rate_hz"], 50)

    def test_stale_recovery_is_observed_but_not_a_manual_unplug_test(self):
        self.observe_all()
        self.monitor.tick(SECOND + 301 * MS)
        self.observe_all(2, SECOND + 310 * MS)
        for name in ("uwb_left", "uwb_right", "lidar_center"):
            counts = self.sensor(name)["counts"]
            self.assertEqual(counts["stale_transitions"], 1)
            self.assertEqual(counts["recoveries"], 1)
        self.assertFalse(self.monitor.report()["completion"])

    def test_manual_order_with_observed_loss_and_recovery_completes_only_observation(self):
        self.manual_cycle()
        report = self.monitor.report()
        self.assertTrue(report["completion"])
        self.assertFalse(report["physical_acceptance"])
        self.assertTrue(all(item["source"] == "operator_declaration" and item["completed_cycles"] == 1
                            for item in report["manual_unplug_tests"].values()))
        self.assertTrue(self.monitor.mark("uwb_left", "unplug", SECOND + 320 * MS))
        self.assertFalse(self.monitor.report()["completion"])

    def test_manual_marks_without_observed_outage_never_complete(self):
        self.observe_all()
        self.assertTrue(self.monitor.mark("uwb_left", "unplug", SECOND + MS))
        self.assertFalse(self.monitor.mark("uwb_left", "replug", SECOND + 2 * MS))
        self.assertFalse(self.monitor.report()["completion"])
        self.assertEqual(self.monitor.report()["manual_unplug_tests"]["uwb_left"]["completed_cycles"], 0)
        with self.assertRaises(ValueError):
            self.monitor.mark("motor", "unplug", SECOND + 3 * MS)

    def test_replay_can_exercise_manual_order_but_cannot_claim_completion(self):
        self.monitor = FieldMonitor(self.mounting, self.uwb, self.settings, source_mode="replay")
        self.manual_cycle(source="replay")
        report = self.monitor.report()
        self.assertFalse(report["completion"])
        self.assertEqual(report["manual_unplug_tests"]["uwb_left"]["completed_cycles"], 1)

    def test_explicit_disconnect_is_distinct_from_silence_and_recovers(self):
        value = status(self.mounting)
        self.monitor.observe_status(value, SECOND)
        self.monitor.observe_status(invalid(value, "uwb_left", "disconnected", connected=False), SECOND + MS)
        self.assertEqual(self.sensor()["state"], "disconnected")
        self.assertEqual(self.sensor()["counts"]["explicit_disconnects"], 1)
        self.monitor.observe_status(status(self.mounting, 2, SECOND + 2 * MS), SECOND + 2 * MS)
        self.assertEqual(self.sensor()["counts"]["recoveries"], 1)

    def test_sample_and_epoch_regression_and_changed_same_sample_fail_closed(self):
        for modifier, counter in ((lambda v: v, "sample_regressions"),
                                  (lambda v: v, "epoch_regressions"),
                                  (lambda v: v, "sample_reuse")):
            with self.subTest(counter=counter):
                monitor = FieldMonitor(self.mounting, self.uwb, self.settings)
                first = status(self.mounting, 3, epoch=2)
                monitor.observe_status(first, SECOND)
                bad = status(self.mounting, 2 if counter == "sample_regressions" else 3,
                             SECOND + MS, epoch=1 if counter == "epoch_regressions" else 2)
                monitor.observe_status(modifier(bad), SECOND + MS)
                sensor = monitor.report()["sensors"]["uwb_left"]
                self.assertEqual(sensor["state"], "invalid")
                self.assertEqual(sensor["counts"][counter], 1)

    def test_new_sample_with_duplicate_or_reversed_device_sequence_is_rejected(self):
        for device, counter in ((1, "device_sequence_duplicates"), (0, "device_sequence_regressions")):
            with self.subTest(device=device):
                monitor = FieldMonitor(self.mounting, self.uwb, self.settings)
                monitor.observe_status(status(self.mounting), SECOND)
                value = status(self.mounting, 2, SECOND + MS)
                value["sensors"]["uwb_left"]["latest"]["device_sequence"] = device
                monitor.observe_status(value, SECOND + MS)
                self.assertEqual(monitor.report()["sensors"]["uwb_left"]["counts"][counter], 1)

    def test_epoch_change_allows_device_sequence_restart(self):
        self.monitor.observe_status(status(self.mounting, 200), SECOND)
        self.monitor.observe_status(status(self.mounting, 1, SECOND + MS, epoch=1), SECOND + MS)
        self.assertEqual(self.sensor()["state"], "fresh")
        self.assertEqual(self.sensor()["counts"]["epoch_changes"], 1)

    def test_retired_session_and_bounded_session_history(self):
        self.monitor.observe_status(status(self.mounting, session="first"), SECOND)
        self.monitor.observe_status(status(self.mounting, at=SECOND + MS, session="second"), SECOND + MS)
        self.assertFalse(self.monitor.observe_status(status(self.mounting, at=SECOND + 2 * MS, session="first"), SECOND + 2 * MS))
        self.assertEqual(self.monitor.report()["counts"]["retired_session"], 1)
        for index in range(40):
            self.monitor.observe_status(status(self.mounting, at=SECOND + (index + 3) * MS,
                                               session="next-" + str(index)), SECOND + (index + 3) * MS)
        self.assertEqual(len(self.monitor._retired_sessions), 32)
        self.assertGreater(self.monitor.report()["counts"]["session_history_limit"], 0)

    def test_wrong_id_and_checksum_are_counted_per_new_sample(self):
        self.monitor.observe_status(status(self.mounting, tag=1), SECOND)
        self.assertEqual(self.sensor()["counts"]["id_mismatches"], 1)
        value = invalid(status(self.mounting, 2, SECOND + MS), "uwb_left", "bad_frame:checksum")
        self.monitor.observe_status(value, SECOND + MS)
        self.monitor.observe_status(value, SECOND + 2 * MS)
        self.assertEqual(self.sensor()["counts"]["checksum_failures"], 1)

    def test_source_clock_future_timestamp_and_overlong_ttl_rejected(self):
        values = []
        wrong_source = status(self.mounting, source="replay")
        values.append(wrong_source)
        wrong_clock = status(self.mounting)
        wrong_clock["sensors"]["uwb_left"]["latest"]["timestamp_source"] = "DEVICE"
        values.append(wrong_clock)
        values.append(status(self.mounting, at=SECOND + MS))
        overlong = status(self.mounting)
        overlong["sensors"]["uwb_left"]["latest"]["valid_until_monotonic_ns"] += MS
        values.append(overlong)
        for value in values:
            with self.subTest(value=value["source_mode"]):
                self.assertFalse(self.monitor.observe_status(value, SECOND))
                self.assertEqual(self.sensor()["state"], "invalid")

    def test_monitor_time_regression_invalidates_every_sensor(self):
        self.observe_all()
        self.assertFalse(self.monitor.tick(SECOND - 1))
        self.assertTrue(all(item["state"] == "invalid" for item in self.monitor.report()["sensors"].values()))
        self.assertEqual(self.monitor.report()["counts"]["invalid_monitor_time"], 1)

    def test_scan_frame_duration_geometry_and_ray_bounds(self):
        self.assertFalse(self.monitor.observe_scan(scan(), SECOND, frame_id="wrong"))
        self.assertEqual(self.sensor("lidar_center")["counts"]["frame_mismatches"], 1)
        for mutation in (lambda value: value.update(scan_time=.5),
                         lambda value: value.update(ranges=[1.] * 8193),
                         lambda value: value["ranges"].append(2.),
                         lambda value: value["ranges"].__setitem__(0, True)):
            value = scan()
            mutation(value)
            self.assertFalse(self.monitor.observe_scan(value, SECOND, frame_id=self.mounting.lidar.frame_id))
        self.assertEqual(self.sensor("lidar_center")["counts"]["scan_duration_errors"], 1)

    def test_zero_scan_duration_is_reported_as_configured_assumption(self):
        value = scan()
        value.update(scan_time=0., time_increment=0.)
        self.assertTrue(self.monitor.observe_scan(value, SECOND, frame_id=self.mounting.lidar.frame_id))
        self.assertEqual(self.monitor.report()["statistics"]["lidar_scan_duration_ms"]["max"], self.settings.max_scan_duration_ms)
        self.assertEqual(self.sensor("lidar_center")["counts"]["zero_duration_used_configured_bound"], 1)

    def test_tracking_projection_and_report_never_retain_raw_positions_or_ids(self):
        self.observe_all()
        value = {"source_mode": "live", "drive_authorized": False, "track": {
            "schema": "smartcart.target_track.v1", "state": "tracking", "valid": True,
            "source_mode": "live", "drive_authorized": False, "reference_monotonic_ns": SECOND,
            "valid_until_monotonic_ns": SECOND + 50 * MS,
            "position_xy_m": [123.456789, 987.654321], "track_id": "SECRET-TARGET"}}
        for name, schema in (("uwb_estimate", "smartcart.uwb_tag_estimate.v1"), ("lidar_clusters", "smartcart.lidar_clusters.v1")):
            value[name] = {"schema": schema, "source_mode": "live", "drive_authorized": False}
        self.assertTrue(self.monitor.observe_tracking_status(value, SECOND))
        encoded = json.dumps(self.monitor.report())
        for forbidden in ("position_xy_m", "track_id", "tag_address", "ranges", "SECRET-TARGET", "123.456789"):
            self.assertNotIn(forbidden, encoded)
        self.monitor.tick(SECOND + 50 * MS)
        self.assertFalse(self.monitor.report()["tracking_latest_metadata"]["valid"])

    def test_bad_tracking_and_adapter_error_clear_previous_metadata(self):
        value = {"source_mode": "replay", "drive_authorized": False, "track": {}}
        self.assertFalse(self.monitor.observe_tracking_status(value, SECOND))
        self.assertIsNone(self.monitor.report()["tracking_latest_metadata"])
        self.observe_all()
        self.monitor.note_error("status_json", SECOND + MS)
        self.assertEqual(self.sensor()["state"], "invalid")
        with self.assertRaises(ValueError):
            self.monitor.note_error("unbounded-user-text", SECOND + MS)

    def test_host_delay_is_an_observed_assumption_exceedance_not_fake_clock_rejection(self):
        monitor = FieldMonitor(self.mounting, EstimationConfig(max_host_rx_delay_ms=0), self.settings)
        self.assertTrue(monitor.observe_status(status(self.mounting), SECOND + MS))
        sensor = monitor.report()["sensors"]["uwb_left"]
        self.assertEqual(sensor["state"], "fresh")
        self.assertEqual(sensor["counts"]["host_rx_delay_assumption_exceeded"], 1)

    def test_runtime_error_and_clock_fault_cannot_reactivate_an_old_sample(self):
        self.observe_all()
        self.monitor.note_error("ros_runtime", SECOND + MS)
        self.assertTrue(all(item["state"] == "invalid" for item in self.monitor.report()["sensors"].values()))
        self.assertFalse(self.monitor.observe_status(status(self.mounting), SECOND + 2 * MS))
        self.assertEqual(self.sensor()["state"], "invalid")

    def test_expiration_is_checked_before_a_scan_arrival_without_an_extra_tick(self):
        self.monitor.observe_status(status(self.mounting), SECOND)
        self.monitor.observe_scan(scan(), SECOND + 250 * MS, frame_id=self.mounting.lidar.frame_id)
        report = self.monitor.report()
        self.assertEqual(self.sensor()["state"], "stale")
        self.assertEqual(report["statistics"]["lidar_pair_rx_skew_ms"]["count"], 0)


if __name__ == "__main__":
    unittest.main()
