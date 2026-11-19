"""추적 설정·수신 수명·캡처 상한과 실패 후 상태 폐기를 장치 없이 검사한다."""
from dataclasses import replace
from itertools import repeat
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from smartcart_tracking.config import (TrackingConfig, config_dict, config_from_dict,
                                       load_tracking_config)
from smartcart_tracking.observation import LidarCollector
from smartcart_tracking.replay import CAPTURE_SCHEMA, read_records, track_records, validate_record
from smartcart_uwb.config import load_config

ROOT = Path(__file__).resolve().parents[1]
NS = 1_000_000_000


def scan():
    return {"angle_min": -.1, "angle_max": .1, "angle_increment": .1,
            "range_min": .05, "range_max": 10, "ranges": [2, 2, 2],
            "scan_time": .02, "time_increment": .01}


def event(kind="tick", at=0, **changes):
    return {"schema": CAPTURE_SCHEMA, "event": kind, "at_ns": at,
            "source": "synthetic", **changes}


class TrackingConfigTests(unittest.TestCase):
    def test_example_roundtrip_has_no_control_authority(self):
        settings = load_tracking_config(ROOT / "config" / "target_tracking.example.json")
        self.assertEqual(settings, TrackingConfig())
        self.assertFalse(settings.assumptions_verified)
        self.assertEqual(config_from_dict(config_dict(settings)), settings)

    def test_reject_unknown_missing_fields_and_wrong_schema(self):
        values = config_dict(TrackingConfig())
        for item in ({**values, "motor_port": "forbidden"}, {**values, "schema": "v2"},
                     {key: value for key, value in values.items() if key != "max_rays"}):
            with self.subTest(item=item), self.assertRaises(ValueError):
                config_from_dict(item)

    def test_numeric_types_bounds_and_cross_field_relations(self):
        values = config_dict(TrackingConfig())
        for name in set(values) - {"schema", "assumptions_verified"}:
            for bad in (True, float("inf"), float("nan"), "1", -10, 10**1000):
                with self.subTest(name=name, bad=str(bad)[:8]), self.assertRaises(ValueError):
                    replace(TrackingConfig(), **{name: bad})
        for change in ({"min_cluster_points": 700}, {"min_cluster_width_m": 1},
                       {"max_rays": 10}, {"confirmation_hits": 1}, {"assumptions_verified": 1}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                TrackingConfig(**change)

    def test_config_file_limits_and_duplicate_keys(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            for bad in (b'{}', b'{"schema":1,"schema":2}', b'\xff', b' ' * 65537,
                        b'[' * 2000 + b']' * 2000):
                path.write_bytes(bad)
                with self.subTest(size=len(bad)), self.assertRaises(ValueError):
                    load_tracking_config(path)


class LidarObservationTests(unittest.TestCase):
    def setUp(self):
        self.mounting = load_config(ROOT / "config" / "dual_uwb.d5.example.json")
        self.settings = TrackingConfig()
        self.collector = LidarCollector(self.mounting, self.settings, session_id="contracts")

    def test_wait_receive_deadline_and_no_stale_resurrection(self):
        self.assertFalse(self.collector.snapshot(NS)["valid"])
        data = self.collector.feed(scan(), NS)
        self.assertTrue(data["valid"])
        deadline = data["valid_until_monotonic_ns"]
        self.assertTrue(self.collector.snapshot(deadline - 1)["valid"])
        self.assertFalse(self.collector.snapshot(deadline)["valid"])
        self.assertFalse(self.collector.snapshot(deadline + 1)["valid"])
        self.assertTrue(self.collector.feed(scan(), deadline + 2)["valid"])

    def test_disconnect_requires_explicit_reconnect_and_new_scan(self):
        first = self.collector.feed(scan(), NS)
        self.collector.disconnected()
        self.assertFalse(self.collector.feed(scan(), NS + 1)["valid"])
        self.collector.reconnect()
        self.assertFalse(self.collector.snapshot(NS + 2)["valid"])
        fresh = self.collector.feed(scan(), NS + 3)
        self.assertGreater(fresh["stream_epoch"], first["stream_epoch"])
        self.assertGreater(fresh["sample_sequence"], first["sample_sequence"])
        self.assertTrue(fresh["valid"])

    def test_clock_regression_and_invalid_time_discard_old_scan(self):
        self.collector.feed(scan(), NS)
        self.assertFalse(self.collector.snapshot(NS - 1)["valid"])
        self.collector.feed(scan(), NS + 1)
        with self.assertRaises(ValueError):
            self.collector.snapshot(True)
        self.assertFalse(self.collector.snapshot(NS + 2)["valid"])

    def test_malformed_or_oversized_scan_cannot_keep_old_measurement(self):
        for index, bad in enumerate((None, {}, scan() | {"ranges": [0] * 8193},
                    scan() | {"ranges": [[0] * 1000]}, scan() | {"scan_time": "private input"},
                    scan() | {"ranges": [True]})):
            with self.subTest(bad=str(bad)[:15]):
                now = NS + index * 10
                self.assertTrue(self.collector.feed(scan(), now)["valid"])
                self.assertFalse(self.collector.feed(bad, now + 1)["valid"])
                self.assertIsNone(self.collector.snapshot(now + 2)["scan"])

    def test_input_and_output_do_not_alias_latest_scan(self):
        payload = scan()
        result = self.collector.feed(payload, NS)
        payload["ranges"][0] = 99
        result["scan"]["ranges"][1] = 99
        self.assertEqual(self.collector.snapshot(NS + 1)["scan"]["ranges"], [2, 2, 2])


class FakePipeline:
    source_mode = "replay"

    def __init__(self):
        self.resets = []

    def process(self, record, now_ns):
        return {"now_ns": now_ns, "source_mode": "replay", "drive_authorized": True}

    def reset(self, reason):
        self.resets.append(reason)


class TrackingCaptureTests(unittest.TestCase):
    def test_only_exact_schema_source_event_and_sensor_are_accepted(self):
        for bad in (None, {}, event(source="live"), event(command="motor"), event(kind="run"),
                    event(at=True), event(at=-1), event(kind="uwb_disconnect", sensor_id="lidar")):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                validate_record(bad)
        self.assertEqual(validate_record(event()), event())

    def test_hex_and_scan_input_are_bounded_and_strictly_typed(self):
        for value in ("", "00 01", "0", "ff" * 1025, True):
            with self.subTest(value=str(value)[:10]), self.assertRaises(ValueError):
                validate_record(event("uwb_bytes", sensor_id="uwb_left", data_hex=value))
        for bad in (scan() | {"ranges": [True]}, scan() | {"ranges": [math.inf]},
                    scan() | {"ranges": [0] * 8193}, scan() | {"angle_min": None}):
            with self.subTest(bad=str(bad)[:20]), self.assertRaises(ValueError):
                validate_record(event("lidar_scan", scan=bad))
        self.assertEqual(validate_record(event("lidar_scan", scan=scan()))["scan"], scan())

    def test_file_json_duplicates_nonfinite_utf8_line_limits_and_empty(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.jsonl"
            for bad in (b'', b'\n', b'\xff', b'{"event":1,"event":2}', b'{"at_ns":NaN}',
                        b'x' * (512 * 1024 + 1)):
                path.write_bytes(bad)
                with self.subTest(size=len(bad)), self.assertRaises(ValueError):
                    list(read_records(path))

    def test_time_gaps_equal_times_and_drive_false_are_preserved(self):
        pipeline = FakePipeline()
        result = list(track_records([event(at=0), event(at=0), event(at=1_000_000_000)], pipeline))
        self.assertEqual(result[-1]["replay_now_ns"] - result[0]["replay_now_ns"], NS)
        self.assertTrue(all(item["drive_authorized"] is False for item in result))

    def test_every_input_failure_resets_previous_tracking_state(self):
        for records in ([], [event(at=2), event(at=1)], [event(), event(source="recorded")], [{}]):
            with self.subTest(records=records):
                pipeline = FakePipeline()
                with self.assertRaises(ValueError):
                    list(track_records(records, pipeline))
                self.assertEqual(pipeline.resets, ["replay_input_error"])
        pipeline = FakePipeline()
        pipeline.source_mode = "live"
        with self.assertRaises(ValueError):
            list(track_records([event()], pipeline))
        self.assertEqual(pipeline.resets, ["replay_input_error"])

    def test_record_count_does_not_accumulate_unbounded_input(self):
        pipeline = FakePipeline()
        with patch("smartcart_tracking.replay.MAX_RECORDS", 2), self.assertRaises(ValueError):
            list(track_records(repeat(event(), 3), pipeline))


if __name__ == "__main__":
    unittest.main()
