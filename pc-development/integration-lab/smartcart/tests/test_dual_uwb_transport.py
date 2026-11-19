"""실물 없이 실제 C 파서와 가상 포트로 수집·기록·종료 경계를 검증한다."""
from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import queue
import struct
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from smartcart_uwb import app
from smartcart_uwb.collector import DualCollector
from smartcart_uwb.config import config_from_dict, load_config
from smartcart_uwb.native import default_library_candidates
from smartcart_uwb.replay import read_records, replay_file
from smartcart_uwb.ros_adapter import ObservationQueue, observation_message
from smartcart_uwb.serial_reader import (OutputError, SerialReaders, canonical_port,
                                         device_identity, open_readonly_port)

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "config" / "dual_uwb.d5.example.json"
NATIVE_AVAILABLE = any(path.is_file() for path in default_library_candidates())


def frame(sequence=1, angle=-21, distance=57):
    payload = struct.pack("<BHiIHii3h", sequence, 0x6E19, angle, distance, 0, -91, -83, 0, 0, 0)
    checksum = 0
    for value in payload:
        checksum ^= value
    return b"\x2a\x1b" + payload + bytes((checksum, 0x23))


def assigned_config():
    payload = json.loads(EXAMPLE.read_text(encoding="utf-8"))
    for index, sensor_id in enumerate(("uwb_left", "uwb_right"), 21):
        payload["sensors"][sensor_id].update(port=f"COM{index}", device_identity_verified=True,
                                               expected_device_identity=sensor_id)
    return config_from_dict(payload)


class FakePort:
    def __init__(self, chunks=()):
        self.chunks = queue.Queue()
        for chunk in chunks:
            self.chunks.put(chunk)
        self.closed = False
        self.writes = 0

    def read(self, maximum):
        if self.closed:
            raise OSError("닫힌 가상 포트")
        try:
            value = self.chunks.get(timeout=.01)
        except queue.Empty:
            return b""
        if isinstance(value, Exception):
            raise value
        return value

    def write(self, _data):
        self.writes += 1
        raise AssertionError("읽기 전용 수신기의 송신 금지")

    def close(self):
        self.closed = True


class PortBoundaryTests(unittest.TestCase):
    def test_only_local_ports_and_alias_normalization(self):
        self.assertEqual(canonical_port("COM21"), canonical_port("\\\\.\\com21"))
        for port in ("socket://localhost:1", "rfc2217://host", "COM0", "relative", "COM21\n"):
            with self.subTest(port=port), self.assertRaises(ValueError):
                canonical_port(port)

    def test_identity_uses_serial_or_physical_location(self):
        info = SimpleNamespace(vid=0x0483, pid=0x5740, serial_number="abc", location="1-2")
        self.assertEqual(device_identity(info), "vid=0483;pid=5740;serial=abc;location=1-2")
        info.serial_number = None
        self.assertEqual(device_identity(info), "vid=0483;pid=5740;location=1-2")

    def test_serial_open_never_writes_and_sets_control_lines_before_open(self):
        events = []

        class Device:
            def __setattr__(self, name, value):
                events.append((name, value))
                object.__setattr__(self, name, value)

            def open(self):
                events.append(("open", self.port))

            def reset_input_buffer(self):
                events.append(("discard_old_input", None))

            def close(self):
                events.append(("close", None))

        device = Device()
        fake_serial = SimpleNamespace(Serial=lambda **kwargs: device)
        with patch.dict(sys.modules, {"serial": fake_serial}):
            self.assertIs(open_readonly_port("COM21"), device)
        self.assertLess(events.index(("dtr", False)), events.index(("open", "COM21")))
        self.assertLess(events.index(("rts", False)), events.index(("open", "COM21")))
        self.assertIn(("discard_old_input", None), events)

    def test_capture_is_exclusive_and_replayable(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as directory:
            path = Path(directory) / "capture.jsonl"
            event = {"schema": "smartcart.uwb_capture.v1", "event": "tick", "at_ns": 0,
                     "source": "recorded"}
            with app.CaptureWriter(path) as writer:
                writer.write(event)
            self.assertEqual(list(read_records(path)), [event])
            with self.assertRaises(FileExistsError):
                app.CaptureWriter(path)
            self.assertEqual(len(list(read_records(path))), 1)

    def test_capture_matches_replay_record_and_byte_limits(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as directory:
            event = {"schema": "smartcart.uwb_capture.v1", "event": "tick", "at_ns": 0,
                     "source": "recorded"}
            with app.CaptureWriter(Path(directory) / "capture.jsonl") as writer:
                with patch.object(app, "MAX_CAPTURE_RECORDS", 1):
                    writer.write(event)
                    with self.assertRaises(ValueError):
                        writer.write(event)
                with patch.object(app, "MAX_CAPTURE_BYTES", writer.size):
                    with self.assertRaises(ValueError):
                        writer.write(event)
                with self.assertRaises(ValueError):
                    writer.write({**event, "command": "금지"})

    def test_inspection_and_help_do_not_load_library_or_open_ports(self):
        with patch("ctypes.CDLL", side_effect=AssertionError("로딩 금지")), redirect_stdout(io.StringIO()):
            self.assertEqual(app.main(["inspect-config", "--config", str(EXAMPLE)]), 0)


@unittest.skipUnless(NATIVE_AVAILABLE, "명시적으로 빌드한 네이티브 라이브러리가 필요합니다")
class NativeTransportTests(unittest.TestCase):
    def wait_until(self, predicate):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.01)
        self.fail("가상 수집기 대기 시간 초과")

    def test_example_cannot_open_unassigned_ports(self):
        with DualCollector(load_config(EXAMPLE)) as collector:
            with self.assertRaises(ValueError):
                SerialReaders(collector, port_opener=lambda _: self.fail("개방 금지"))

    def test_identity_mismatch_never_opens_a_device(self):
        with DualCollector(assigned_config()) as collector:
            readers = SerialReaders(collector, identity_reader=lambda _: "wrong",
                                    port_opener=lambda _: self.fail("식별 불일치 개방 금지"), retry_seconds=.05)
            with readers:
                self.wait_until(lambda: collector.snapshot(time.monotonic_ns())["sensors"]["uwb_left"]["state"] == "error")
            self.assertFalse(collector.snapshot(time.monotonic_ns())["drive_authorized"])

    def test_explicit_left_only_never_opens_right(self):
        left = FakePort([frame()])
        opened = []
        def open_port(port):
            opened.append(port)
            if port != "COM21":
                self.fail("오른쪽 포트 개방 금지")
            return left
        with DualCollector(assigned_config()) as collector:
            collector.disconnected("uwb_right", reason="disabled_single_left")
            readers = SerialReaders(collector, sensor_ids=("uwb_left",), port_opener=open_port,
                                    identity_reader=lambda port: "uwb_left", retry_seconds=.05)
            with readers:
                self.wait_until(lambda: readers.tick()["sensors"]["uwb_left"]["state"] == "live")
                status = readers.tick()
                self.assertFalse(status["sensors"]["uwb_right"]["connected"])
                self.assertEqual(status["sensors"]["uwb_right"]["last_error"], "disabled_single_left")
                self.assertFalse(status["drive_authorized"])
            self.assertEqual(opened, ["COM21"])

    def test_fragmented_independent_streams_record_no_tx_and_replay(self):
        left = FakePort([frame()[:10], frame()[10:]])
        right = FakePort([frame(angle=30, distance=120)])
        events = []
        with DualCollector(assigned_config()) as collector:
            readers = SerialReaders(collector, port_opener=lambda port: left if port == "COM21" else right,
                                    identity_reader=lambda port: "uwb_left" if port == "COM21" else "uwb_right",
                                    capture_callback=events.append, retry_seconds=.05)
            with readers:
                self.wait_until(lambda: all(v["state"] == "live" for v in readers.tick()["sensors"].values()))
                latest = readers.tick()["sensors"]
                self.assertEqual(latest["uwb_left"]["latest"]["range_m"], .57)
                self.assertEqual(latest["uwb_right"]["latest"]["range_m"], 1.2)
            self.assertTrue(left.closed and right.closed)
            self.assertEqual(left.writes + right.writes, 0)
        self.assertEqual([e["at_ns"] for e in events], sorted(e["at_ns"] for e in events))
        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as directory:
            path = Path(directory) / "capture.jsonl"
            with app.CaptureWriter(path) as writer:
                for event in events:
                    writer.write(event)
            with DualCollector(assigned_config(), source_mode="replay") as replay_collector:
                result = list(replay_file(path, replay_collector))
            self.assertEqual(sum(len(x["observations"]) for x in result), 2)
            self.assertTrue(all(x["drive_authorized"] is False for x in result))

    def test_one_port_failure_reconnects_only_its_own_epoch(self):
        left = FakePort([frame()])
        bad_right, good_right = FakePort([OSError("가상 단절")]), FakePort([frame()])
        right_ports = iter([bad_right, good_right])
        with DualCollector(assigned_config()) as collector:
            readers = SerialReaders(collector, port_opener=lambda port: left if port == "COM21" else next(right_ports),
                                    identity_reader=lambda port: "uwb_left" if port == "COM21" else "uwb_right",
                                    retry_seconds=.05)
            with readers:
                self.wait_until(lambda: readers.tick()["sensors"]["uwb_right"]["state"] == "live")
                status = readers.tick()["sensors"]
                self.assertEqual(status["uwb_left"]["stream_epoch"], 1)
                self.assertEqual(status["uwb_right"]["stream_epoch"], 2)
            self.assertTrue(bad_right.closed and good_right.closed)

    def test_capture_failure_stops_all_workers_and_close_reports_failure(self):
        ports = [FakePort([frame()]), FakePort()]
        with DualCollector(assigned_config()) as collector:
            readers = SerialReaders(collector, port_opener=lambda port: ports[0] if port == "COM21" else ports[1],
                                    identity_reader=lambda port: "uwb_left" if port == "COM21" else "uwb_right",
                                    capture_callback=lambda _: (_ for _ in ()).throw(OSError("가상 디스크 오류")))
            readers.start()
            self.wait_until(readers.stop.is_set)
            with self.assertRaises(RuntimeError):
                readers.close()
            self.assertTrue(all(not worker.is_alive() for worker in readers.threads))

    def test_tick_holds_event_lock_through_snapshot(self):
        with DualCollector(assigned_config()) as collector:
            readers = SerialReaders(collector)
            original = collector.snapshot
            ownership = []
            def checked(now_ns):
                ownership.append(readers.event_lock._is_owned())
                return original(now_ns)
            with patch.object(collector, "snapshot", checked):
                readers.tick()
            self.assertEqual(ownership, [True])

    def test_start_failure_rolls_back_started_workers(self):
        created = []
        class Thread:
            def __init__(self, **_):
                self.index = len(created)
                self.joined = False
                created.append(self)
            def start(self):
                if self.index:
                    raise RuntimeError("가상 스레드 생성 실패")
            def join(self, timeout):
                self.joined = True
            def is_alive(self):
                return False
        with DualCollector(assigned_config()) as collector:
            readers = SerialReaders(collector)
            with patch("smartcart_uwb.serial_reader.threading.Thread", Thread), self.assertRaises(RuntimeError):
                readers.start()
            self.assertTrue(readers.stop.is_set() and created[0].joined)

    def test_ros_conversion_preserves_contract_and_expires_delayed_messages(self):
        with DualCollector(assigned_config()) as collector:
            value = collector.feed("uwb_left", frame(), now_ns=100)[0]
        message = observation_message(value, SimpleNamespace, now_ns=101)
        self.assertTrue(message.valid and message.frame_available)
        self.assertEqual(message.range_m, .57)
        self.assertAlmostEqual(message.bearing_rad, 21 * 3.141592653589793 / 180)
        self.assertFalse(message.device_timestamp_available or message.drive_authorized)
        self.assertFalse(observation_message(value, SimpleNamespace, now_ns=value["valid_until_monotonic_ns"]).valid)
        with self.assertRaises(ValueError):
            observation_message({**value, "source_mode": "replay"}, SimpleNamespace, now_ns=101)
        with self.assertRaises(ValueError):
            observation_message({**value, "drive_authorized": True}, SimpleNamespace, now_ns=101)
        definition = (ROOT.parent / "ros2/smartcart_uwb_interfaces/msg/UwbObservation.msg").read_text(encoding="utf-8")
        fields = {line.split()[1] for line in definition.splitlines() if line and not line.startswith("#")}
        self.assertEqual(fields, set(vars(message)))

    def test_ros_queue_overflow_invalidates_instead_of_renewing_old_data(self):
        with DualCollector(assigned_config()) as collector:
            value = collector.feed("uwb_left", frame(), now_ns=100)[0]
            pending = ObservationQueue(collector, capacity=1)
            pending.append(value)
            pending.append(value)
            self.assertEqual(pending.drain(), [])
            self.assertEqual(collector.snapshot(101)["sensors"]["uwb_left"]["state"], "error")
            collector.feed("uwb_left", frame(sequence=2), now_ns=102)
            self.assertEqual(collector.snapshot(103)["sensors"]["uwb_left"]["state"], "live")

    def test_ros_invalid_reason_survives_an_unavailable_deadline(self):
        data = bytearray(frame())
        data[29] ^= 1
        with DualCollector(assigned_config()) as collector:
            value = collector.feed("uwb_left", bytes(data), now_ns=100)[0]
        message = observation_message(value, SimpleNamespace, now_ns=101)
        self.assertFalse(message.valid or message.frame_available)
        self.assertEqual(message.invalid_reason, "bad_frame:checksum")

    def test_cli_tick_output_failure_returns_error_without_raw_data(self):
        class Readers:
            fatal_error = None
            stop = threading.Event()
            def __init__(self, *_, **__):
                pass
            def __enter__(self):
                return self
            def __exit__(self, *_):
                pass
            def tick(self):
                raise OutputError("가상 기록 오류")
        output, errors = io.StringIO(), io.StringIO()
        with patch.object(app, "SerialReaders", Readers), redirect_stdout(output), redirect_stderr(errors):
            result = app.main(["collect", "--config", str(EXAMPLE), "--seconds", ".1"])
        self.assertEqual(result, 2)
        self.assertNotIn("28185", output.getvalue() + errors.getvalue())


if __name__ == "__main__":
    unittest.main()
