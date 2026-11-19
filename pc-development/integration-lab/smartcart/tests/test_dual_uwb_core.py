"""장치 연결 없이 좌우 설정·관측·오류 우선순위와 세션 경계를 검증한다."""

from dataclasses import FrozenInstanceError, replace
import json
import math
from pathlib import Path
import tempfile
from concurrent.futures import ThreadPoolExecutor
import unittest

from smartcart_uwb.collector import DualCollector
from smartcart_uwb.config import config_from_dict, load_config

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "config" / "dual_uwb.d5.example.json"


def config_payload():
    return json.loads(EXAMPLE.read_text(encoding="utf-8"))


def frame(sequence=1, tag=0x6E19, distance_cm=123, angle_deg=30):
    return {"sequence": sequence, "tag_address": tag, "distance_cm": distance_cm,
            "angle_deg": angle_deg, "user_command": 0xC000,
            "first_path_power": -91, "rx_level": -83,
            "acceleration_x": 0, "acceleration_y": 0, "acceleration_z": 0}


def event(sequence=1, **kwargs):
    return {"kind": "frame", "frame": frame(sequence, **kwargs)}


class FakeParser:
    def __init__(self):
        self.events = []
        self.resets = 0
        self.closed = False

    def feed_events(self, data):
        events, self.events = self.events, []
        return events

    def reset(self):
        self.events = []
        self.resets += 1

    def close(self):
        self.closed = True


class ConfigTests(unittest.TestCase):
    def test_example_mounting_is_cad_proposal_and_not_drive_authority(self):
        config = load_config(EXAMPLE)
        self.assertEqual(set(config.sensors), {"uwb_left", "uwb_right"})
        self.assertEqual(config.sensors["uwb_left"].translation_m, (.756, .244, .286))
        self.assertEqual(config.lidar.translation_m, (.817, 0, .2678))
        self.assertFalse(config.drive_authorized)
        self.assertFalse(config.sensors["uwb_left"].mounting_verified)
        self.assertIsNone(config.sensors["uwb_right"].port)
        with self.assertRaises(TypeError):
            config.sensors["uwb_left"] = config.sensors["uwb_right"]
        with self.assertRaises(FrozenInstanceError):
            config.timeout_ms = 10

    def test_direct_dataclass_replacement_is_validated_and_nested_mapping_is_frozen(self):
        config = load_config(EXAMPLE)
        with self.assertRaises(ValueError):
            replace(config, timeout_ms=True)
        sensors = dict(config.sensors)
        copy = replace(config, sensors=sensors)
        sensors.clear()
        self.assertEqual(len(copy.sensors), 2)
        with self.assertRaises(ValueError):
            replace(config.sensors["uwb_left"], translation_m=(float("nan"), 0, 0))

    def test_unknown_top_sensor_mounting_and_metadata_fields_rejected(self):
        for target in ((), ("sensors", "uwb_left"), ("lidar",), ("metadata",)):
            with self.subTest(target=target):
                payload = config_payload()
                item = payload
                for key in target:
                    item = item[key]
                item["drive_authorized"] = True
                with self.assertRaises(ValueError):
                    config_from_dict(payload)

    def test_missing_fields_and_wrong_sensor_names_rejected(self):
        payload = config_payload()
        del payload["sensors"]["uwb_right"]
        with self.assertRaises(ValueError):
            config_from_dict(payload)
        payload = config_payload()
        del payload["timeout_ms"]
        with self.assertRaises(ValueError):
            config_from_dict(payload)

    def test_nonfinite_and_boolean_numeric_values_rejected(self):
        for value in (True, float("nan"), float("inf"), 11, -(10**1000)):
            with self.subTest(value=str(value)[:20]):
                payload = config_payload()
                payload["sensors"]["uwb_left"]["translation_m"][0] = value
                with self.assertRaises(ValueError):
                    config_from_dict(payload)
        for key, value in (("timeout_ms", True), ("timeout_ms", 0), ("timeout_ms", 5001),
                           ("logical_target_id", 16), ("allowed_tag_address", False)):
            payload = config_payload()
            payload[key] = value
            with self.assertRaises(ValueError):
                config_from_dict(payload)

    def test_duplicate_com_aliases_and_frame_ids_rejected(self):
        for left, right in (("COM25", "com25"), ("COM25", "\\\\.\\COM25")):
            payload = config_payload()
            payload["sensors"]["uwb_left"]["port"] = left
            payload["sensors"]["uwb_right"]["port"] = right
            with self.assertRaises(ValueError):
                config_from_dict(payload)
        payload = config_payload()
        payload["sensors"]["uwb_left"]["frame_id"] = "uwb_right"
        with self.assertRaises(ValueError):
            config_from_dict(payload)

    def test_realpath_port_alias_rejected(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as temp:
            target = Path(temp) / "device"
            target.touch()
            payload = config_payload()
            payload["sensors"]["uwb_left"]["port"] = str(target)
            payload["sensors"]["uwb_right"]["port"] = str(target.parent / "." / target.name)
            with self.assertRaises(ValueError):
                config_from_dict(payload)

    def test_no_implicit_ports_or_uncorroborated_identity(self):
        payload = config_payload()
        payload["sensors"]["uwb_left"]["port"] = "ttyUSB0"
        with self.assertRaises(ValueError):
            config_from_dict(payload)
        payload = config_payload()
        payload["sensors"]["uwb_left"]["device_identity_verified"] = True
        with self.assertRaises(ValueError):
            config_from_dict(payload)

    def test_calibration_cannot_silently_enable_frozen_door_lock_profile(self):
        payload = config_payload()
        payload["sensors"]["uwb_left"]["calibration_profile"] = "final_v2"
        with self.assertRaises(ValueError):
            config_from_dict(payload)

    def test_duplicate_json_keys_nonfinite_and_large_file_rejected(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as temp:
            path = Path(temp) / "config.json"
            for text in ('{"schema":"a","schema":"b"}', '{"number":NaN}', " " * 65537):
                path.write_text(text, encoding="utf-8")
                with self.assertRaises(ValueError):
                    load_config(path)


class CollectorTests(unittest.TestCase):
    def setUp(self):
        self.parsers = []
        def factory():
            parser = FakeParser()
            self.parsers.append(parser)
            return parser
        self.collector = DualCollector(load_config(EXAMPLE), parser_factory=factory, session_id="core-tests")
        self.addCleanup(self.collector.close)

    def feed(self, sensor="uwb_left", now=1_000_000_000, events=None):
        self.parsers[0 if sensor == "uwb_left" else 1].events = events or [event()]
        return self.collector.feed(sensor, b"sample", now)

    def latest(self, sensor="uwb_left", now=1_000_000_000):
        return self.collector.snapshot(now)["sensors"][sensor]

    def test_raw_observation_does_not_claim_horizontal_position_or_drive(self):
        observation = self.feed()[0]
        self.assertEqual(observation["schema"], "smartcart.uwb_observation.v1")
        self.assertTrue(observation["valid"])
        self.assertEqual(observation["range_m"], 1.23)
        self.assertAlmostEqual(observation["bearing_rad"], -math.pi / 6)
        self.assertEqual(observation["logical_target_id"], 0)
        self.assertIsNone(observation["device_timestamp_ns"])
        self.assertEqual(observation["timestamp_source"], "HOST_RX")
        self.assertFalse(observation["drive_authorized"])
        self.assertNotIn("position", observation)
        self.assertNotIn("x", observation)

    def test_left_right_parsers_sequences_and_faults_are_isolated(self):
        self.feed(events=[event(15)])
        self.feed("uwb_right", events=[event(1)])
        self.feed(now=1_100_000_000, events=[event(15)])
        snapshot = self.collector.snapshot(1_100_000_000)
        self.assertEqual(snapshot["sensors"]["uwb_left"]["state"], "error")
        self.assertEqual(snapshot["sensors"]["uwb_right"]["state"], "live")
        self.assertFalse(snapshot["drive_authorized"])

    def test_duplicate_and_old_sequence_invalidate_without_renewing_freshness(self):
        first = self.feed(events=[event(10)])[0]
        duplicate = self.feed(now=1_100_000_000, events=[event(10)])[0]
        self.assertEqual(duplicate["invalid_reason"], "duplicate_sequence")
        self.assertFalse(self.latest(now=1_100_000_000)["latest"]["valid"])
        self.assertEqual(first["valid_until_monotonic_ns"], duplicate["valid_until_monotonic_ns"])
        old = self.feed(now=1_200_000_000, events=[event(9)])[0]
        self.assertEqual(old["invalid_reason"], "out_of_order_sequence")
        recovery = self.feed(now=1_300_000_000, events=[event(11)])[0]
        self.assertTrue(recovery["valid"])

    def test_wraparound_and_half_range_boundary(self):
        for sequence in (254, 255, 0, 127):
            self.assertTrue(self.feed(events=[event(sequence)])[0]["valid"])
        result = self.feed(events=[event(255)])[0]
        self.assertEqual(result["invalid_reason"], "out_of_order_sequence")

    def test_checksum_error_and_wrong_id_clear_previous_valid_immediately(self):
        first = self.feed()[0]
        bad = self.feed(now=1_100_000_000, events=[{"kind": "bad_frame", "reason": "checksum"}])[0]
        self.assertFalse(bad["valid"])
        self.assertEqual(bad["valid_until_monotonic_ns"], first["valid_until_monotonic_ns"])
        self.assertEqual(self.latest(now=1_100_000_000)["state"], "error")
        wrong = self.feed(now=1_200_000_000, events=[event(2, tag=1)])[0]
        self.assertFalse(wrong["id_valid"])
        self.assertIsNone(wrong["logical_target_id"])
        self.assertEqual(wrong["invalid_reason"], "id_mismatch")
        self.assertTrue(self.feed(now=1_300_000_000, events=[event(3)])[0]["valid"])

    def test_wrong_tag_and_invalid_measurement_do_not_poison_sequence_reference(self):
        self.feed(events=[event(1)])
        wrong = self.feed(now=1_100_000_000, events=[event(200, tag=1)])[0]
        self.assertEqual(wrong["invalid_reason"], "id_mismatch")
        recovery = self.feed(now=1_200_000_000, events=[event(2)])[0]
        self.assertTrue(recovery["valid"])
        self.feed(now=1_300_000_000, events=[event(100, distance_cm=0)])
        self.assertTrue(self.feed(now=1_400_000_000, events=[event(3)])[0]["valid"])

    def test_ordered_error_between_valid_frames_is_observable(self):
        outputs = self.feed(events=[event(1), {"kind": "bad_frame", "reason": "checksum"}, event(2)])
        self.assertEqual([o["valid"] for o in outputs], [True, False, True])
        self.assertEqual([o["sample_sequence"] for o in outputs], [1, 2, 3])
        self.assertEqual(self.latest()["state"], "live")

    def test_final_error_in_chunk_does_not_leave_valid_snapshot(self):
        self.feed(events=[event(1), {"kind": "bad_frame", "reason": "footer"}])
        self.assertEqual(self.latest()["state"], "error")
        self.assertFalse(self.latest()["latest"]["valid"])

    def test_invalid_measurements_and_malformed_native_values_are_visible(self):
        for value in (0, 4, 2001):
            result = self.feed(events=[event(value, distance_cm=value)])[0]
            self.assertFalse(result["measurement_valid"])
        malformed = event(6)
        malformed["frame"]["sequence"] = True
        result = self.feed(events=[malformed])[0]
        self.assertEqual(result["invalid_reason"], "invalid_frame")

    def test_expiry_at_boundary_is_sticky_until_a_new_sample(self):
        self.feed()
        self.assertEqual(self.latest(now=1_499_999_999)["state"], "live")
        stale = self.latest(now=1_500_000_000)
        self.assertEqual(stale["state"], "stale")
        self.assertFalse(stale["latest"]["valid"])
        self.assertEqual(stale["latest"]["invalid_reason"], "stale")
        self.assertEqual(self.latest(now=1_400_000_000)["state"], "stale")
        self.assertTrue(self.feed(now=1_600_000_000, events=[event(2)])[0]["valid"])

    def test_reconnect_only_increments_epoch_and_keeps_host_sequence(self):
        first = self.feed(events=[event(200)])[0]
        self.collector.disconnected("uwb_left")
        self.assertEqual(self.latest()["stream_epoch"], 0)
        self.assertFalse(self.latest()["latest"]["valid"])
        self.collector.reconnect("uwb_left")
        self.assertEqual(self.latest()["state"], "waiting")
        after = self.feed(now=1_100_000_000, events=[event(1)])[0]
        self.assertEqual(after["stream_epoch"], 1)
        self.assertEqual(after["sample_sequence"], first["sample_sequence"] + 1)
        self.assertTrue(after["valid"])

    def test_disconnect_cannot_be_undone_by_receiving_bytes(self):
        self.feed()
        self.collector.disconnected("uwb_left")
        result = self.feed(now=1_100_000_000, events=[event(2)])[0]
        self.assertEqual(result["invalid_reason"], "disconnected")
        self.assertEqual(self.latest(now=1_100_000_000)["state"], "error")

    def test_host_clock_regression_is_rejected_and_can_recover(self):
        first = self.feed()[0]
        result = self.feed(now=999_999_999, events=[event(2)])[0]
        self.assertEqual(result["invalid_reason"], "non_monotonic_time")
        self.assertEqual(result["valid_until_monotonic_ns"], first["valid_until_monotonic_ns"])
        self.assertTrue(self.feed(now=1_100_000_000, events=[event(2)])[0]["valid"])

    def test_invalid_feed_arguments_invalidate_previous_observation(self):
        for data, now in ((b"x", True), (b"x" * 65537, 1_100_000_000), ("x", 1_100_000_000)):
            self.feed(events=[event()])
            with self.assertRaises((TypeError, ValueError)):
                self.collector.feed("uwb_left", data, now)
            self.assertEqual(self.latest()["state"], "error")

    def test_returned_observations_and_snapshots_cannot_mutate_internal_state(self):
        result = self.feed()[0]
        result["valid"] = False
        result["drive_authorized"] = True
        snapshot = self.collector.snapshot(1_000_000_000)
        snapshot["sensors"]["uwb_left"]["latest"]["valid"] = False
        self.assertTrue(self.latest()["latest"]["valid"])
        self.assertFalse(self.latest()["latest"]["drive_authorized"])

    def test_replay_time_metadata_does_not_replace_current_host_clock(self):
        with DualCollector(load_config(EXAMPLE), parser_factory=FakeParser,
                           source_mode="replay", session_id="replay") as collector:
            collector._states["uwb_left"].parser.events = [event()]
            result = collector.feed("uwb_left", b"x", 10_000, source_rx_ns=123)[0]
            self.assertEqual(result["source_mode"], "replay")
            self.assertEqual(result["rx_monotonic_ns"], 10_000)
            self.assertEqual(result["source_rx_monotonic_ns"], 123)
            self.assertFalse(result["drive_authorized"])

    def test_concurrent_sensor_feeds_preserve_independent_state(self):
        def worker(sensor_id, parser):
            for sequence in range(100):
                parser.events = [event(sequence)]
                result = self.collector.feed(sensor_id, b"x", 1_000_000_000 + sequence)
                self.assertTrue(result[0]["valid"])
        with ThreadPoolExecutor(max_workers=2) as pool:
            futures = [pool.submit(worker, sensor_id, parser)
                       for sensor_id, parser in zip(("uwb_left", "uwb_right"), self.parsers)]
            for future in futures:
                future.result()
        snapshot = self.collector.snapshot(1_000_000_100)
        for state in snapshot["sensors"].values():
            self.assertEqual(state["latest"]["sample_sequence"], 100)

    def test_close_is_idempotent_and_closes_both_parsers(self):
        self.collector.close()
        self.collector.close()
        self.assertTrue(all(parser.closed for parser in self.parsers))
        with self.assertRaises(RuntimeError):
            self.collector.feed("uwb_left", b"x", 0)


class NativeCollectorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from smartcart_uwb.native import NativeParser
        try:
            with NativeParser():
                pass
        except (FileNotFoundError, OSError) as error:
            raise unittest.SkipTest("명시적으로 빌드한 C 파서가 없습니다") from error

    def test_real_c_protocol_sample_is_preserved_through_dual_collector(self):
        raw = bytes.fromhex("2a 1b 0c 19 6e eb ff ff ff 39 00 00 00 00 c0 "
                            "00 00 00 00 00 00 00 00 00 00 00 00 00 00 96 23")
        with DualCollector(load_config(EXAMPLE), session_id="native-core-test") as collector:
            self.assertEqual(collector.feed("uwb_left", raw[:10], 1_000_000_000), [])
            result = collector.feed("uwb_left", raw[10:], 1_010_000_000)[0]
            self.assertTrue(result["valid"])
            self.assertEqual(result["device_sequence"], 12)
            self.assertEqual(result["raw_angle_deg"], -21)
            self.assertAlmostEqual(result["range_m"], .57)
            self.assertAlmostEqual(result["bearing_rad"], math.radians(21))
            self.assertEqual(collector.snapshot(1_010_000_000)["sensors"]["uwb_right"]["state"], "waiting")

    def test_real_c_checksum_failure_invalidates_existing_snapshot(self):
        raw = bytes.fromhex("2a 1b 0c 19 6e eb ff ff ff 39 00 00 00 00 c0 "
                            "00 00 00 00 00 00 00 00 00 00 00 00 00 00 96 23")
        damaged = bytearray(raw)
        damaged[29] ^= 1
        with DualCollector(load_config(EXAMPLE), session_id="native-error-test") as collector:
            collector.feed("uwb_left", raw, 1_000_000_000)
            result = collector.feed("uwb_left", bytes(damaged), 1_010_000_000)
            self.assertTrue(any(item["invalid_reason"] == "bad_frame:checksum" for item in result))
            self.assertFalse(collector.snapshot(1_010_000_000)["sensors"]["uwb_left"]["latest"]["valid"])


if __name__ == "__main__":
    unittest.main()
