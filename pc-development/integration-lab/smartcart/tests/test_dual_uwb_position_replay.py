"""합성 기준점의 원시 31바이트부터 위치 추정·CLI까지 검증한다.

높이는 지면이 아닌 base_link 원점 기준이다. 합성 좌표를 실측 검수값으로
취급하지 않으며 실제 장치·ROS·모터를 시작하지 않는다.
"""
from __future__ import annotations

import contextlib
import io
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from smartcart_uwb.collector import DualCollector
from smartcart_uwb.config import config_from_dict as mounting_from_dict, load_config
from smartcart_uwb.estimation_config import EstimationConfig, load_estimation_config
from smartcart_uwb.native import NativeParser
from smartcart_uwb.replay import CAPTURE_SCHEMA, read_records

ROOT = Path(__file__).resolve().parents[1]
CAPTURE = ROOT / "samples" / "dual_uwb.position.synthetic.jsonl"
TRUTH = ROOT / "samples" / "dual_uwb.position.truth.json"
SETTINGS = ROOT / "config" / "uwb_estimation.synthetic.json"
MOUNTING = ROOT / "config" / "dual_uwb.d5.example.json"
TAG_HEIGHT_M = .9


def position_frame(sensor, point_xy, sequence, *, tag_address=0x6E19,
                   angle_bias_deg=0.0, distance_override_m=None, horizontal=False):
    """공간 거리 cm와 센서 로컬 오른쪽 양수 정수 각도로 양자화한다."""
    dx = point_xy[0] - sensor.translation_m[0]
    dy = point_xy[1] - sensor.translation_m[1]
    dz = 0.0 if horizontal else TAG_HEIGHT_M - sensor.translation_m[2]
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)
    if distance_override_m is not None:
        distance = distance_override_m
    bearing = math.atan2(dy, dx) - math.radians(sensor.yaw_deg)
    bearing = math.atan2(math.sin(bearing), math.cos(bearing))
    raw_angle = -math.degrees(bearing) + angle_bias_deg
    raw_angle = (raw_angle + 180) % 360 - 180
    angle = int(round(raw_angle))
    distance_cm = int(math.floor(distance * 100 + .5))
    payload = struct.pack("<BHiIHii3h", sequence, tag_address, angle,
                          distance_cm, 0xC000, 0, 0, 0, 0, 0)
    checksum = 0
    for value in payload:
        checksum ^= value
    return b"\x2a\x1b" + payload + bytes((checksum, 0x23))


def build_position_fixture(config):
    """동일한 설정에서 커밋된 합성 자료를 재생성하는 순수 테스트 도우미."""
    records, checkpoints = [], []
    cursor = 0
    sequences = {"uwb_left": 254, "uwb_right": 254}

    def add(event, *, sensor_id=None, data=None, at_ns=None):
        nonlocal cursor
        moment = cursor if at_ns is None else at_ns
        item = {"schema": CAPTURE_SCHEMA, "event": event,
                "at_ns": moment, "source": "synthetic"}
        if sensor_id is not None:
            item["sensor_id"] = sensor_id
        if data is not None:
            item["data_hex"] = data.hex()
        records.append(item)
        cursor = moment + 1_000_000

    def checkpoint(label, valid, point=None):
        checkpoints.append({"label": label, "event_index": len(records) - 1,
                            "expected_valid": valid, "position_xy_m": point,
                            "max_error_m": .08 if valid else None})

    def reset():
        for sensor_id in ("uwb_left", "uwb_right"):
            add("disconnect", sensor_id=sensor_id)
            add("reconnect", sensor_id=sensor_id)
            sequences[sensor_id] = 254

    def sample(sensor_id, point, *, at_ns=None, **kwargs):
        data = position_frame(config.sensors[sensor_id], point, sequences[sensor_id], **kwargs)
        sequences[sensor_id] = (sequences[sensor_id] + 1) & 255
        add("bytes", sensor_id=sensor_id, data=data, at_ns=at_ns)

    def pair(point, **right_kwargs):
        start = cursor
        sample("uwb_left", point)
        sample("uwb_right", point, at_ns=start + 5_000_000, **right_kwargs)

    def warm(point):
        for _ in range(3):
            pair(point)

    for label, point in (("front_2m", [2.0, 0.0]), ("front_left_3m", [3.0, 1.0]),
                         ("front_right_5m", [5.0, -1.0]), ("left", [0.0, 2.0]),
                         ("right", [0.0, -2.0]), ("rear", [-2.0, 0.0])):
        reset(); warm(point); checkpoint(label, True, point)

    point = [2.5, .2]
    reset(); warm(point)
    start = cursor
    sample("uwb_left", point)
    sample("uwb_right", point, at_ns=start + 100_000_000)
    checkpoint("asynchronous_pair", False)
    warm(point); checkpoint("skew_recovery", True, point)

    pair(point, angle_bias_deg=90)
    checkpoint("one_sensor_outlier", False)
    warm(point); checkpoint("outlier_recovery", True, point)

    pair(point, tag_address=0x1234)
    checkpoint("wrong_tag", False)
    warm(point); checkpoint("id_recovery", True, point)

    data = bytearray(position_frame(config.sensors["uwb_right"], point, sequences["uwb_right"]))
    data[29] ^= 1
    add("bytes", sensor_id="uwb_right", data=bytes(data))
    checkpoint("bad_checksum", False)
    warm(point); checkpoint("checksum_recovery", True, point)

    add("tick", at_ns=cursor + 250_000_000)
    checkpoint("estimator_age_timeout", False)
    add("tick", at_ns=cursor + 600_000_000)
    checkpoint("collector_timeout", False)

    add("disconnect", sensor_id="uwb_left")
    checkpoint("disconnected", False)
    reset(); warm(point); checkpoint("reconnect_recovery", True, point)

    pair(point, distance_override_m=.1)
    checkpoint("impossible_slant_height", False)
    reset(); warm(point); checkpoint("final_recovery", True, point)
    return records, checkpoints


def moving_capture(config, samples=100):
    """각 센서의 서로 다른 수신 시각에 맞춘 가상의 저속 이동 태그다.

    USB 지연·자차 운동은 없는 합성 가정이다. X 속도 .36m/s와 Y 속도
    최대 .12m/s이므로 설정 max_relative_speed_mps=3보다 충분히 느리다.
    """
    records = []

    def truth_at(at_ns):
        seconds = max(0, (at_ns - 10_000_000) / 1_000_000_000)
        return [2.0 + .36 * seconds, .4 * math.sin(.3 * seconds)]

    for index, sensor_id in enumerate(("uwb_left", "uwb_right")):
        for event_index, event in enumerate(("disconnect", "reconnect")):
            records.append({"schema": CAPTURE_SCHEMA, "event": event,
                            "sensor_id": sensor_id, "source": "synthetic",
                            "at_ns": (index * 2 + event_index) * 1_000_000})
    for index in range(samples):
        left_time = 10_000_000 + index * 80_000_000
        skew = (5 + index % 26) * 1_000_000
        for sensor_id, at_ns in (("uwb_left", left_time), ("uwb_right", left_time + skew)):
            data = position_frame(config.sensors[sensor_id], truth_at(at_ns), (254 + index) & 255)
            records.append({"schema": CAPTURE_SCHEMA, "event": "bytes", "sensor_id": sensor_id,
                            "source": "synthetic", "at_ns": at_ns, "data_hex": data.hex()})
    return records, truth_at


class PositionFixtureTests(unittest.TestCase):
    def test_synthetic_fixture_reproducible_with_integer_protocol_quantization(self):
        config = load_config(MOUNTING)
        records, checkpoints = build_position_fixture(config)
        self.assertEqual(records, list(read_records(CAPTURE)))
        truth = json.loads(TRUTH.read_text(encoding="utf-8"))
        self.assertEqual(checkpoints, truth["checkpoints"])
        self.assertEqual(truth["source"], "synthetic")
        self.assertEqual(truth["tag_height_base_link_m"], TAG_HEIGHT_M)
        for item in records:
            if item["event"] != "bytes":
                continue
            data = bytes.fromhex(item["data_hex"])
            self.assertEqual(len(data), 31)
            self.assertEqual(data[:2], b"\x2a\x1b")
            self.assertEqual(data[-1], 0x23)

    def test_explicit_model_and_unverified_assumptions_are_preserved(self):
        settings = load_estimation_config(SETTINGS)
        self.assertEqual(settings.range_model, "slant_known_height")
        self.assertEqual(settings.tag_height_m, TAG_HEIGHT_M)
        self.assertFalse(settings.assumptions_verified)
        self.assertEqual(EstimationConfig().range_model, "unknown")


class PositionReplayEndToEndTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from smartcart_uwb.estimator import TagEstimator
        from smartcart_uwb.estimation_replay import estimate_file, estimate_records
        cls.estimator_type = TagEstimator
        cls.estimate_file = staticmethod(estimate_file)
        cls.estimate_records = staticmethod(estimate_records)
        cls.mounting = load_config(MOUNTING)
        cls.settings = load_estimation_config(SETTINGS)
        try:
            NativeParser().close()
        except (OSError, RuntimeError) as exc:
            raise unittest.SkipTest("명시적으로 빌드한 C 원시 파서가 필요합니다") from exc

    def components(self, *, mounting=None, settings=None):
        mounting = self.mounting if mounting is None else mounting
        settings = self.settings if settings is None else settings
        collector = DualCollector(mounting, source_mode="replay", session_id="synthetic-position-test")
        self.addCleanup(collector.close)
        estimator = self.estimator_type(mounting, settings, source_mode="replay")
        return collector, estimator

    def test_all_truth_checkpoints_from_native_bytes_to_base_link(self):
        collector, estimator = self.components()
        events = list(self.estimate_file(CAPTURE, collector, estimator))
        truth = json.loads(TRUTH.read_text(encoding="utf-8"))
        for point in truth["checkpoints"]:
            with self.subTest(label=point["label"]):
                estimate = events[point["event_index"]]["estimate"]
                self.assertEqual(estimate["valid"], point["expected_valid"])
                self.assertFalse(estimate["drive_authorized"])
                if point["expected_valid"]:
                    self.assertEqual(estimate["frame_id"], "base_link")
                    actual = estimate["position_xy_m"]
                    error = math.dist(actual, point["position_xy_m"])
                    self.assertLessEqual(error, point["max_error_m"])
                    self.assertLessEqual(error, estimate["error_radius_m"])
                    self.assertTrue(estimate["provisional"])
                else:
                    self.assertIsNone(estimate["position_xy_m"])
                    self.assertEqual(estimate["valid_until_monotonic_ns"], 0)
        self.assertTrue(all(event["estimate"]["source_mode"] == "replay" for event in events))
        json.dumps(events, allow_nan=False)

    def test_yaw_and_nonzero_rear_axle_origin_are_applied(self):
        payload = json.loads(MOUNTING.read_text(encoding="utf-8"))
        payload["sensors"]["uwb_left"]["yaw_deg"] = 30.0
        payload["sensors"]["uwb_right"]["yaw_deg"] = -45.0
        mounting = mounting_from_dict(payload)
        records, checkpoints = build_position_fixture(mounting)
        collector, estimator = self.components(mounting=mounting)
        events = list(self.estimate_records(records, collector, estimator))
        for point in checkpoints:
            if not point["expected_valid"]:
                continue
            with self.subTest(label=point["label"]):
                estimate = events[point["event_index"]]["estimate"]
                self.assertTrue(estimate["valid"])
                self.assertLessEqual(math.dist(estimate["position_xy_m"], point["position_xy_m"]), .08)

    def test_moving_tag_with_5_to_30ms_skew_stays_inside_error_radius(self):
        from smartcart_uwb.replay import REPLAY_CLOCK_ORIGIN_NS
        records, truth_at = moving_capture(self.mounting)
        collector, estimator = self.components()
        created = set()
        skews = set()
        for event in self.estimate_records(records, collector, estimator):
            estimate = event["estimate"]
            if not estimate["valid"]:
                self.assertIsNone(estimate["position_xy_m"])
                continue
            sequence = estimate["estimate_sequence"]
            if sequence in created:
                continue
            created.add(sequence)
            pair = estimate["samples"]
            latest_rx = max(pair[key]["rx_monotonic_ns"] for key in ("uwb_left", "uwb_right"))
            self.assertEqual(estimate["reference_monotonic_ns"], latest_rx)
            actual = truth_at(latest_rx - REPLAY_CLOCK_ORIGIN_NS)
            self.assertLessEqual(math.dist(actual, estimate["position_xy_m"]), estimate["error_radius_m"])
            self.assertFalse(estimate["drive_authorized"])
            skews.add(estimate["rx_skew_ns"])
        self.assertGreaterEqual(len(created), 95)
        self.assertGreaterEqual(len(skews), 20)

    def test_wall_clock_speed_cannot_change_replay_output(self):
        records, _ = moving_capture(self.mounting, samples=12)
        results = []
        for host_clock in (10, 9_000_000_000_000):
            collector, estimator = self.components()
            with patch("time.monotonic_ns", return_value=host_clock):
                results.append(list(self.estimate_records(records, collector, estimator)))
        self.assertEqual(results[0], results[1])

    def test_unknown_range_model_never_returns_position(self):
        collector, estimator = self.components(settings=EstimationConfig())
        events = list(self.estimate_file(CAPTURE, collector, estimator))
        self.assertTrue(all(not event["estimate"]["valid"] for event in events))
        self.assertTrue(all(event["estimate"]["position_xy_m"] is None for event in events))

    def test_repeated_tick_does_not_extend_estimate_lifetime(self):
        records, checkpoints = build_position_fixture(self.mounting)
        end = checkpoints[0]["event_index"]
        initial = records[:end + 1]
        last_ns = initial[-1]["at_ns"]
        for delta in (10_000_000, 20_000_000, 250_000_000):
            initial.append({"schema": CAPTURE_SCHEMA, "event": "tick", "source": "synthetic",
                            "at_ns": last_ns + delta})
        collector, estimator = self.components()
        events = list(self.estimate_records(initial, collector, estimator))
        first, held, expired = events[end]["estimate"], events[end + 1]["estimate"], events[-1]["estimate"]
        self.assertTrue(first["valid"])
        self.assertEqual(first["estimate_sequence"], held["estimate_sequence"])
        self.assertEqual(first["valid_until_monotonic_ns"], held["valid_until_monotonic_ns"])
        self.assertFalse(expired["valid"])
        self.assertIsNone(expired["position_xy_m"])

    def test_one_side_updates_cannot_refresh_a_pair(self):
        records, checkpoints = build_position_fixture(self.mounting)
        end = checkpoints[0]["event_index"]
        initial = records[:end + 1]
        last_ns = initial[-1]["at_ns"]
        for sequence, delta in ((1, 20_000_000), (2, 30_000_000)):
            initial.append({"schema": CAPTURE_SCHEMA, "event": "bytes", "source": "synthetic",
                            "sensor_id": "uwb_left", "at_ns": last_ns + delta,
                            "data_hex": position_frame(self.mounting.sensors["uwb_left"], [2, 0], sequence).hex()})
        collector, estimator = self.components()
        events = list(self.estimate_records(initial, collector, estimator))
        first = events[end]["estimate"]
        for event in events[end + 1:]:
            estimate = event["estimate"]
            self.assertEqual(first["estimate_sequence"], estimate["estimate_sequence"])
            self.assertEqual(first["valid_until_monotonic_ns"], estimate["valid_until_monotonic_ns"])

    def test_recorded_capture_stays_replay_and_mixed_sources_rejected(self):
        records, checkpoints = build_position_fixture(self.mounting)
        initial = records[:checkpoints[0]["event_index"] + 1]
        recorded = [{**item, "source": "recorded"} for item in initial]
        collector, estimator = self.components()
        events = list(self.estimate_records(recorded, collector, estimator))
        self.assertTrue(events[-1]["estimate"]["valid"])
        for event in events:
            self.assertEqual(event["original_source"], "recorded")
            self.assertEqual(event["estimate"]["source_mode"], "replay")
            self.assertFalse(event["estimate"]["drive_authorized"])
        collector, estimator = self.components()
        mixed = [initial[0], {**initial[1], "source": "recorded"}]
        with self.assertRaises(ValueError):
            list(self.estimate_records(mixed, collector, estimator))

    def test_cli_summary_does_not_print_tag_or_position(self):
        from smartcart_uwb.estimation_cli import main
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            status = main(["--config", str(MOUNTING), "--estimation-config", str(SETTINGS),
                           "--input", str(CAPTURE)])
        self.assertEqual(status, 0)
        summary = json.loads(stdout.getvalue())
        self.assertEqual(summary["source_mode"], "replay")
        self.assertFalse(summary["drive_authorized"])
        for field in ("tag_address", "position_xy_m", "bearing_rad", "range_m"):
            self.assertNotIn(f'"{field}":', stdout.getvalue())

    def test_cli_output_is_opt_in_and_refuses_existing_file(self):
        from smartcart_uwb.estimation_cli import main
        with tempfile.TemporaryDirectory(dir=ROOT) as folder:
            output = Path(folder) / "position.jsonl"
            arguments = ["--config", str(MOUNTING), "--estimation-config", str(SETTINGS),
                         "--input", str(CAPTURE), "--output", str(output)]
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(arguments), 0)
            data = output.read_bytes()
            self.assertGreater(len(data), 0)
            for line in data.splitlines():
                item = json.loads(line)
                self.assertFalse(item["estimate"]["drive_authorized"])
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertNotEqual(main(arguments), 0)
            self.assertEqual(output.read_bytes(), data)

    def test_cli_invalid_input_does_not_echo_private_fields(self):
        from smartcart_uwb.estimation_cli import main
        with tempfile.TemporaryDirectory(dir=ROOT) as folder:
            path = Path(folder) / "invalid.jsonl"
            sentinel = "PRIVATE_TAG_POSITION_MUST_NOT_LEAK"
            path.write_text(json.dumps({"tag_address": sentinel, "position_xy_m": [123, 456]}),
                            encoding="utf-8")
            stdout, stderr = io.StringIO(), io.StringIO()
            with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                status = main(["--config", str(MOUNTING), "--estimation-config", str(SETTINGS),
                               "--input", str(path)])
            self.assertNotEqual(status, 0)
            self.assertEqual(stdout.getvalue(), "")
            self.assertNotIn(sentinel, stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
