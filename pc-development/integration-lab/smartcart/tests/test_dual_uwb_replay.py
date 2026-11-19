"""오프라인 캡처의 검증·시간·출처·좌우 독립성 회귀 검사. 실물 포트를 열지 않는다."""
from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from smartcart_uwb import replay, replay_cli

ROOT = Path(__file__).resolve().parents[1]
REAL_FRAME = bytes.fromhex("2a 1b 0c 19 6e eb ff ff ff 39 00 00 00 00 c0 "
                           "00 00 00 00 00 00 00 00 00 00 00 00 00 00 96 23")


def record(event="bytes", at_ns=0, sensor_id="uwb_left", **changes):
    result = {"schema": replay.CAPTURE_SCHEMA, "event": event, "at_ns": at_ns,
              "source": "synthetic"}
    if sensor_id is not None:
        result["sensor_id"] = sensor_id
    if event == "bytes":
        result["data_hex"] = REAL_FRAME.hex()
    result.update(changes)
    return result


def frame(sequence, *, tag=0x6E19, angle=-21, distance=57):
    """변형 회귀 프레임이며 실물 측정값으로 표시하지 않는다."""
    data = bytearray(REAL_FRAME)
    struct.pack_into("<BHiI", data, 2, sequence, tag, angle, distance)
    data[29] = 0
    for byte in data[2:29]:
        data[29] ^= byte
    return bytes(data)


class FakeCollector:
    """시계 전달 계약만 검증하는 장치 없는 대역이다."""
    def __init__(self, source_mode="replay"):
        self.source_mode = source_mode
        self.calls = []
        self.closed = False

    def feed(self, sensor_id, data, *, now_ns, source_rx_ns):
        self.calls.append(("bytes", sensor_id, data, now_ns, source_rx_ns))
        return [{"sensor_id": sensor_id, "tag_address": "TAG_MUST_NOT_LEAK"}]

    def disconnected(self, sensor_id, reason):
        self.calls.append(("disconnect", sensor_id, reason))

    def reconnect(self, sensor_id):
        self.calls.append(("reconnect", sensor_id))

    def snapshot(self, now_ns):
        self.calls.append(("snapshot", now_ns))
        return {"now_ns": now_ns, "source_mode": self.source_mode, "drive_authorized": True}

    def close(self):
        self.closed = True


class CaptureValidationTests(unittest.TestCase):
    def test_only_exact_capture_schema(self):
        for value in (None, [], {}, record(schema="smartcart.uwb_capture.v2")):
            with self.subTest(value=value), self.assertRaises(replay.ReplayError):
                replay.validate_record(value)

    def test_unknown_fields_cannot_be_paths_or_control(self):
        for key in ("path", "command", "drive_authorized", "at_seconds"):
            with self.subTest(key=key), self.assertRaises(replay.ReplayError):
                replay.validate_record(record(**{key: "ignored"}))

    def test_integer_clock_bounds_reject_bool_float_negative_and_huge(self):
        for value in (True, 0.0, -1, float("nan"), float("inf"), replay.MAX_AT_NS + 1):
            with self.subTest(value=value), self.assertRaises(replay.ReplayError):
                replay.validate_record(record(at_ns=value))

    def test_invalid_sensor_and_event(self):
        for value in (record(sensor_id="unknown"), record(sensor_id=[]), record(event="run"),
                      record(event=[]), record(sensor_id=None)):
            with self.subTest(value=value), self.assertRaises(replay.ReplayError):
                replay.validate_record(value)

    def test_tick_can_omit_sensor_but_no_bytes(self):
        self.assertEqual(replay.validate_record(record("tick", sensor_id=None))["event"], "tick")
        with self.assertRaises(replay.ReplayError):
            replay.validate_record(record("tick", data_hex="00"))

    def test_hex_size_format_and_types(self):
        for value in ("", "0", "GG", "00 01", "00\n", "00" * 1025, 123, None):
            with self.subTest(value=value), self.assertRaises(replay.ReplayError):
                replay.validate_record(record(data_hex=value))
        self.assertEqual(len(replay.validate_record(record(data_hex="aB" * 1024))["data_hex"]), 2048)

    def test_missing_source_and_unapproved_source(self):
        value = record()
        del value["source"]
        for item in (value, record(source="live"), record(source={})):
            with self.subTest(item=item), self.assertRaises(replay.ReplayError):
                replay.validate_record(item)

    def test_replay_clock_preserves_gaps_and_original_time(self):
        collector = FakeCollector()
        outputs = list(replay.replay_records([record(at_ns=5), record(at_ns=9_000_000_005)], collector))
        calls = [item for item in collector.calls if item[0] == "bytes"]
        self.assertEqual(calls[1][3] - calls[0][3], 9_000_000_000)
        self.assertEqual([item[4] for item in calls], [5, 9_000_000_005])
        self.assertEqual(outputs[0]["replay_now_ns"], replay.REPLAY_CLOCK_ORIGIN_NS + 5)

    def test_equal_times_allowed_but_global_reverse_rejected(self):
        collector = FakeCollector()
        events = [record(at_ns=10), record(at_ns=10, sensor_id="uwb_right"),
                  record(at_ns=9, sensor_id="uwb_right")]
        stream = replay.replay_records(events, collector)
        next(stream); next(stream)
        with self.assertRaises(replay.ReplayError):
            next(stream)

    def test_recorded_data_cannot_claim_live_or_authorize_drive(self):
        output = next(replay.replay_records([record(source="recorded")], FakeCollector()))
        self.assertEqual(output["original_source"], "recorded")
        for value in (output, output["snapshot"], output["observations"][0]):
            self.assertEqual(value["source_mode"], "replay")
            self.assertIs(value["drive_authorized"], False)

    def test_live_collector_rejected_before_feed(self):
        collector = FakeCollector("live")
        with self.assertRaises(replay.ReplayError):
            next(replay.replay_records([record()], collector))
        self.assertFalse(collector.calls)

    def test_disconnect_reconnect_tick_forwarded_in_order(self):
        collector = FakeCollector()
        outputs = list(replay.replay_records([record("disconnect"), record("reconnect", 1),
                                             record("tick", 2, sensor_id=None)], collector))
        self.assertEqual([item[0] for item in collector.calls],
                         ["disconnect", "snapshot", "reconnect", "snapshot", "snapshot"])
        self.assertTrue(all(not item["observations"] for item in outputs))

    def test_lazy_iterator_does_not_read_ahead(self):
        def events():
            yield record()
            raise AssertionError("아직 다음 레코드를 읽으면 안 됩니다")
        stream = replay.replay_records(events(), FakeCollector())
        self.assertEqual(next(stream)["event_index"], 0)

    def test_record_count_is_bounded(self):
        with patch.object(replay, "MAX_RECORDS", 2), self.assertRaises(replay.ReplayError):
            list(replay.replay_records([record(), record(), record()], FakeCollector()))

    def test_empty_stream_and_invalid_clock_origin_rejected(self):
        with self.assertRaises(replay.ReplayError):
            list(replay.replay_records([], FakeCollector()))
        for value in (0, -1, True, 1.5, replay.REPLAY_CLOCK_ORIGIN_NS + 1):
            with self.subTest(value=value), self.assertRaises(replay.ReplayError):
                list(replay.replay_records([record()], FakeCollector(), clock_origin_ns=value))


class CaptureFileTests(unittest.TestCase):
    def setUp(self):
        # 허용된 작업 폴더 안에서만 임시 파일을 만들고 검사 후 삭제한다.
        self.temp = tempfile.TemporaryDirectory(dir=ROOT)
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "capture.jsonl"

    def write(self, content):
        self.path.write_bytes(content)
        return self.path

    def test_file_streaming_and_real_frame_bytes(self):
        path = self.write((json.dumps(record()) + "\n").encode())
        collector = FakeCollector()
        self.assertEqual(len(list(replay.replay_file(path, collector))), 1)
        self.assertEqual(collector.calls[0][2], REAL_FRAME)

    def test_invalid_utf8_json_and_nonfinite(self):
        values = (b"\xff\n", b"{\n", b"[]\n", b"\n", b"{}\n", b'{"at_ns":NaN}\n')
        for content in values:
            with self.subTest(content=content), self.assertRaises(replay.ReplayError):
                list(replay.read_records(self.write(content)))

    def test_duplicate_json_key_rejected(self):
        with self.assertRaises(replay.ReplayError):
            list(replay.read_records(self.write(b'{"event":"tick","event":"bytes"}\n')))

    def test_long_line_and_file_limit(self):
        with self.assertRaises(replay.ReplayError):
            list(replay.read_records(self.write(b" " * (replay.MAX_LINE_BYTES + 1))))
        self.write((json.dumps(record()) + "\n").encode())
        with patch.object(replay, "MAX_FILE_BYTES", 10), self.assertRaises(replay.ReplayError):
            list(replay.read_records(self.path))

    def test_file_count_bound_and_empty_file(self):
        self.write(((json.dumps(record()) + "\n") * 3).encode())
        with patch.object(replay, "MAX_RECORDS", 2), self.assertRaises(replay.ReplayError):
            list(replay.read_records(self.path))
        with self.assertRaises(replay.ReplayError):
            list(replay.read_records(self.write(b"")))

    def test_cli_default_summary_omits_tag_and_coordinates(self):
        self.write((json.dumps(record()) + "\n").encode())
        collector = FakeCollector()
        modules = {
            "smartcart_uwb.config": types.SimpleNamespace(load_config=lambda path: object()),
            "smartcart_uwb.collector": types.SimpleNamespace(DualCollector=lambda *args, **kwargs: collector),
            "smartcart_uwb.native": types.SimpleNamespace(NativeParser=lambda **kwargs: object()),
        }
        stdout = io.StringIO()
        with patch.dict(sys.modules, modules), contextlib.redirect_stdout(stdout):
            result = replay_cli.main(["--input", str(self.path), "--summary"])
        self.assertEqual(result, 0)
        summary = json.loads(stdout.getvalue())
        self.assertEqual(summary["records"], 1)
        self.assertNotIn("TAG_MUST_NOT_LEAK", stdout.getvalue())
        self.assertNotIn("tag_address", stdout.getvalue())
        self.assertNotIn("position", stdout.getvalue())
        self.assertTrue(collector.closed)
        self.assertEqual([item.name for item in Path(self.temp.name).iterdir()], ["capture.jsonl"])

    def test_cli_explicit_output_is_exclusive(self):
        self.write((json.dumps(record()) + "\n").encode())
        output = Path(self.temp.name) / "result.jsonl"
        output.write_text("기존 결과", encoding="utf-8")
        collector = FakeCollector()
        modules = {
            "smartcart_uwb.config": types.SimpleNamespace(load_config=lambda path: object()),
            "smartcart_uwb.collector": types.SimpleNamespace(DualCollector=lambda *args, **kwargs: collector),
            "smartcart_uwb.native": types.SimpleNamespace(NativeParser=lambda **kwargs: object()),
        }
        with patch.dict(sys.modules, modules), contextlib.redirect_stderr(io.StringIO()):
            result = replay_cli.main(["--input", str(self.path), "--output", str(output)])
        self.assertEqual(result, 2)
        self.assertEqual(output.read_text(encoding="utf-8"), "기존 결과")
        self.assertTrue(collector.closed)


class NativeReplaySemanticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from smartcart_uwb.config import load_config
        from smartcart_uwb.collector import DualCollector
        from smartcart_uwb.native import NativeParser
        cls.config = load_config(ROOT / "config" / "dual_uwb.d5.example.json")
        try:
            probe = NativeParser()
            probe.close()
        except (OSError, RuntimeError, FileNotFoundError) as exc:
            raise unittest.SkipTest("명시적으로 빌드한 원시 라이브러리가 없습니다") from exc
        cls.collector_type = DualCollector

    def collector(self):
        collector = self.collector_type(self.config, source_mode="replay")
        self.addCleanup(collector.close)
        return collector

    def test_native_real_protocol_preserved(self):
        result = next(replay.replay_records([record()], self.collector()))
        self.assertEqual(len(result["observations"]), 1)
        observation = result["observations"][0]
        self.assertEqual(observation["tag_address"], 0x6E19)
        self.assertEqual(observation["raw_angle_deg"], -21)
        self.assertEqual(observation["range_m"], .57)
        self.assertEqual(observation["source_rx_monotonic_ns"], 0)
        self.assertFalse(observation["drive_authorized"])

    def test_fixture_contains_faults_gap_epoch_and_recovery(self):
        events = list(replay.replay_file(ROOT / "samples" / "dual_uwb.synthetic.jsonl", self.collector()))
        snapshots = [item["snapshot"] for item in events]
        self.assertTrue(any(s["sensors"]["uwb_left"]["state"] == "stale" for s in snapshots))
        self.assertTrue(any(s["sensors"]["uwb_left"]["state"] == "error" for s in snapshots))
        self.assertGreater(snapshots[-1]["sensors"]["uwb_left"]["stream_epoch"],
                           snapshots[0]["sensors"]["uwb_left"]["stream_epoch"])
        self.assertTrue(all(item["source_mode"] == "replay" and not item["drive_authorized"] for item in events))

    def test_checksum_fault_invalidates_only_affected_sensor_and_recovers(self):
        bad = bytearray(frame(11))
        bad[29] ^= 1
        records = [record(data_hex=frame(10).hex()),
                   record(at_ns=1, sensor_id="uwb_right", data_hex=frame(90).hex()),
                   record(at_ns=2, data_hex=bytes(bad).hex()),
                   record(at_ns=3, data_hex=frame(12).hex())]
        events = list(replay.replay_records(records, self.collector()))
        fault = events[2]["snapshot"]["sensors"]
        self.assertEqual(fault["uwb_left"]["latest"]["invalid_reason"], "bad_frame:checksum")
        self.assertTrue(fault["uwb_right"]["latest"]["valid"])
        self.assertTrue(events[3]["snapshot"]["sensors"]["uwb_left"]["latest"]["valid"])

    def test_duplicate_keeps_original_deadline_and_explicit_gap_expires(self):
        timeout_ns = self.config.timeout_ms * 1_000_000
        records = [record(data_hex=frame(10).hex()),
                   record(at_ns=timeout_ns - 1, data_hex=frame(10).hex()),
                   record("tick", timeout_ns + 1, sensor_id=None)]
        events = list(replay.replay_records(records, self.collector()))
        first = events[0]["observations"][0]
        duplicate = events[1]["observations"][0]
        self.assertEqual(duplicate["invalid_reason"], "duplicate_sequence")
        self.assertEqual(duplicate["valid_until_monotonic_ns"], first["valid_until_monotonic_ns"])
        self.assertFalse(events[2]["snapshot"]["sensors"]["uwb_left"]["latest"]["valid"])

    def test_reconnect_accepts_new_device_sequence_with_new_epoch(self):
        records = [record(data_hex=frame(100).hex()), record("disconnect", 1),
                   record("reconnect", 2), record(at_ns=3, data_hex=frame(0).hex())]
        events = list(replay.replay_records(records, self.collector()))
        first, last = events[0]["observations"][0], events[3]["observations"][0]
        self.assertTrue(last["valid"])
        self.assertEqual(last["stream_epoch"], first["stream_epoch"] + 1)
        self.assertGreater(last["sample_sequence"], first["sample_sequence"])

    def test_duplicate_reverse_wrap_unknown_tag_and_independent_sensors(self):
        records = [record(data_hex=frame(254).hex()),
                   record(at_ns=1, sensor_id="uwb_right", data_hex=frame(50).hex()),
                   record(at_ns=2, data_hex=frame(255).hex()),
                   record(at_ns=3, data_hex=frame(0).hex()),
                   record(at_ns=4, data_hex=frame(0).hex()),
                   record(at_ns=5, data_hex=frame(255).hex()),
                   record(at_ns=6, data_hex=frame(1, tag=0x1234).hex())]
        events = list(replay.replay_records(records, self.collector()))
        last = events[-1]["snapshot"]["sensors"]
        self.assertFalse(last["uwb_left"]["latest"]["valid"])
        self.assertTrue(last["uwb_right"]["latest"]["valid"])
        for index in (4, 5):
            self.assertFalse(events[index]["snapshot"]["sensors"]["uwb_left"]["latest"]["valid"])


if __name__ == "__main__":
    unittest.main()
