"""UWB 입력·좌표·시간 초과의 장치 독립 회귀 테스트."""
import sys
import types
import unittest
from unittest.mock import patch

from smartcart_sensing.uwb import (
    MAX_LINE_BYTES, UwbStore, demo_loop, parse_diagnostic_line, serial_loop,
    synthetic_line, transform_position, validate_serial_port,
)

VALID_V2 = ("I (1234) C_KEY_DIAG: C_KEY_DIAG_V2,1234,42,28185,0,0,1,1,1,143,8,"
            "1430.0,1412.0,7.50,0.184,1.400,1.112,WELCOME,2,42,3")
VALID_V3 = ("C_KEY_DIAG_V3,1234,42,28185,0,0,1,1,1,143,8,-8123,-7544,"
            "1430.0,1412.0,7.50,0.184,1.400,1.112,WELCOME,2,42,3,1,9")


def change(column, value, text=VALID_V3):
    fields = text.split(",")
    fields[column] = str(value)
    return ",".join(fields)


class ParserTests(unittest.TestCase):
    def test_existing_v2_protocol(self):
        frame = parse_diagnostic_line(VALID_V2)
        self.assertEqual(frame["tag_address"], 0x6E19)
        self.assertEqual(frame["filtered_angle_deg"], 7.5)
        self.assertEqual(frame["first_path_power"], 0)
        self.assertFalse(frame["angle_sample_rejected"])

    def test_existing_v3_protocol(self):
        frame = parse_diagnostic_line(VALID_V3)
        self.assertEqual(frame["first_path_power"], -8123)
        self.assertEqual(frame["rx_level"], -7544)
        self.assertTrue(frame["angle_sample_rejected"])
        self.assertEqual(frame["angle_rejected_samples"], 9)
        self.assertEqual(frame["state"], "WELCOME")

    def test_boot_log_ignored(self):
        self.assertIsNone(parse_diagnostic_line("I (200) 시작 완료"))

    def test_bad_columns_and_new_versions_fail(self):
        for line in ("C_KEY_DIAG_V3,1,2", VALID_V3 + ",1", VALID_V3.replace("_V3", "_V9")):
            with self.subTest(line=line), self.assertRaises(ValueError):
                parse_diagnostic_line(line)

    def test_types_nonfinite_and_field_limits(self):
        cases = [(6, "true"), (7, "2"), (8, "-1"), (1, 4294967296), (2, 256),
                 (3, 65536), (4, -1), (5, 16), (11, -2147483649), (12, 2147483648),
                 (13, "nan"), (14, "inf"), (15, 180.1), (16, 31), (17, "-inf"),
                 (18, -1), (19, "<script>"), (23, 2), (24, 4294967296)]
        for column, value in cases:
            with self.subTest(column=column, value=value), self.assertRaises(ValueError):
                parse_diagnostic_line(change(column, value))

    def test_control_characters_and_oversized_line(self):
        for line in (VALID_V3.replace("WELCOME", "WELCOME\x1b"),
                     "x" * (MAX_LINE_BYTES + 1), VALID_V3.replace("1.400", "1.\n400")):
            with self.subTest(line=line[:20]), self.assertRaises(ValueError):
                parse_diagnostic_line(line)

    def test_firmware_zero_invalid_frame_accepted_for_status(self):
        line = "C_KEY_DIAG_V3,2000,0,0,255,0,0,0,0,0,0,0,0,0.0,0.0,0.00,0.000,0.000,0.000,NO_KEY,1,12,2,0,0"
        store = UwbStore()
        self.assertTrue(store.publish_line(line))
        snapshot = store.snapshot()
        self.assertEqual(snapshot["state"], "live")
        self.assertFalse(snapshot["measurement"]["valid"])
        self.assertIsNone(snapshot["measurement"]["position"])


class StoreTests(unittest.TestCase):
    def test_initial_and_live_and_rate(self):
        store = UwbStore()
        self.assertEqual(store.snapshot(10)["state"], "waiting")
        store.publish_line(VALID_V3, 10)
        store.publish_line(change(1, 1254), 10.02)
        snapshot = store.snapshot(10.02)
        self.assertEqual(snapshot["state"], "live")
        self.assertEqual(snapshot["receive_hz"], 50)
        self.assertTrue(snapshot["measurement"]["valid"])
        self.assertEqual(snapshot["measurement"]["position"], {"x": 1.4, "y": -.184, "frame_id": "base_link"})

    def test_timeout_removes_both_coordinate_frames(self):
        store = UwbStore()
        store.publish_line(VALID_V3, 10)
        snapshot = store.snapshot(10.501)
        self.assertEqual(snapshot["state"], "stale")
        self.assertFalse(snapshot["measurement"]["valid"])
        self.assertIsNone(snapshot["measurement"]["position"])
        self.assertIsNone(snapshot["measurement"]["uwb_position"])
        self.assertEqual(snapshot["measurement"]["firmware_state"], "WELCOME")

    def test_repeated_serial_frame_does_not_extend_freshness_or_clear_fault(self):
        store = UwbStore()
        self.assertTrue(store.publish_line(VALID_V3, 10))
        self.assertFalse(store.publish_line(VALID_V3, 10.4))
        self.assertEqual(store.snapshot(10.6)["state"], "stale")
        store.set_error("시리얼 연결 끊김")
        self.assertFalse(store.publish_line(VALID_V3, 10.7))
        self.assertEqual(store.snapshot(10.7)["state"], "error")
        self.assertTrue(store.publish_line(change(1, 1274), 10.8))
        self.assertEqual(store.snapshot(10.8)["state"], "live")

    def test_rejected_frame_immediately_hides_position_then_recovers(self):
        store = UwbStore()
        store.publish_line(VALID_V3, 10)
        self.assertFalse(store.publish_line(change(16, "nan"), 10.01))
        snapshot = store.snapshot(10.01)
        self.assertEqual(snapshot["state"], "error")
        self.assertIsNone(snapshot["measurement"]["position"])
        self.assertEqual(snapshot["rejected_frames"], 1)
        store.publish_line(change(1, 1274), 10.04)
        self.assertEqual(store.snapshot(10.04)["state"], "live")

    def test_invalid_id_flags_and_fault_never_return_position(self):
        for column, value in [(3, 1234), (4, 1), (5, 1), (6, 0), (7, 0), (8, 0),
                              (9, 2001), (14, 20001), (19, "FAULT"), (19, "INVALID_ID"), (19, "NO_KEY")]:
            with self.subTest(column=column, value=value):
                store = UwbStore()
                self.assertTrue(store.publish_line(change(column, value), 10))
                measurement = store.snapshot(10)["measurement"]
                self.assertFalse(measurement["valid"])
                self.assertIsNone(measurement["position"])

    def test_snapshot_cannot_mutate_store(self):
        store = UwbStore()
        store.publish_line(VALID_V3, 10)
        snapshot = store.snapshot(10)
        snapshot["measurement"]["position"]["x"] = 500
        snapshot["transform"]["offset_x"] = 20
        self.assertEqual(store.snapshot(10)["measurement"]["position"]["x"], 1.4)
        self.assertEqual(store.snapshot(10)["transform"]["offset_x"], 0)

    def test_demo_is_explicit_synthetic(self):
        store = UwbStore("demo")
        store.publish_line(synthetic_line(.1), 10)
        snapshot = store.snapshot(10)
        self.assertEqual(snapshot["state"], "synthetic")
        self.assertTrue(snapshot["synthetic"])
        live = UwbStore("serial")
        demo_loop(live)
        self.assertEqual(live.snapshot()["state"], "error")

    def test_constructor_rejects_unsafe_settings(self):
        for arguments in ({"source_mode": "unknown"}, {"stale_seconds": float("nan")},
                          {"stale_seconds": 0}, {"yaw_deg": 361}, {"offset_x": 11}):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                UwbStore(**arguments)


class TransformTests(unittest.TestCase):
    def test_forward_right_left(self):
        self.assertEqual(transform_position(0, 2), {"x": 2, "y": 0, "frame_id": "base_link"})
        self.assertEqual(transform_position(2, 0), {"x": 0, "y": -2, "frame_id": "base_link"})
        self.assertEqual(transform_position(-2, 0), {"x": 0, "y": 2, "frame_id": "base_link"})

    def test_yaw_and_offset_are_applied_after_axis_conversion(self):
        position = transform_position(0, 2, yaw_deg=90, offset_x=.2, offset_y=-.3)
        self.assertAlmostEqual(position["x"], .2)
        self.assertAlmostEqual(position["y"], 1.7)

    def test_nonfinite_and_unbounded_values_rejected(self):
        for arguments in ({"x_m": float("nan"), "y_m": 2},
                          {"x_m": 0, "y_m": 2, "offset_y": float("inf")},
                          {"x_m": True, "y_m": 2}):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                transform_position(**arguments)


class SerialTests(unittest.TestCase):
    def test_local_port_only(self):
        for port in ("COM21", "/dev/ttyUSB0", "/dev/serial/by-id/usb-ESP32-if00-port0"):
            self.assertEqual(validate_serial_port(port), port)
        for port in ("socket://127.0.0.1:1234", "/etc/passwd", "/dev/../etc/passwd", "COM0", "/dev/tty\nUSB0"):
            with self.subTest(port=port), self.assertRaises(ValueError):
                validate_serial_port(port)

    def test_serial_reads_fragmented_bounded_frames_without_writing(self):
        store = UwbStore()
        stream = [b"I (1) ", "부팅\n".encode(), b"x" * (MAX_LINE_BYTES + 10),
                  b"\n", VALID_V3[:60].encode(), VALID_V3[60:].encode() + b"\r\n"]
        connections = []

        class FakeSerial:
            def __init__(self, **kwargs):
                self.closed = False
                self.opened = False
                self.dtr = self.rts = True
                connections.append(self)

            @property
            def in_waiting(self):
                return len(stream[0]) if stream else 0

            def open(self):
                self.opened = True

            def read(self, limit):
                if not stream:
                    store.stop.set()
                    return b""
                chunk = stream.pop(0)
                if len(chunk) > limit:
                    stream.insert(0, chunk[limit:])
                return chunk[:limit]

            def write(self, data):
                raise AssertionError("센서 입력 수신기는 시리얼에 쓰면 안 됩니다")

            def close(self):
                self.closed = True

        fake_module = types.SimpleNamespace(Serial=FakeSerial, SerialException=OSError)
        with patch.dict(sys.modules, {"serial": fake_module}):
            serial_loop(store, "/dev/ttyUSB0")
        self.assertTrue(connections[0].closed)
        self.assertFalse(connections[0].dtr)
        self.assertFalse(connections[0].rts)
        self.assertEqual(store.snapshot()["rejected_frames"], 1)
        self.assertTrue(store.snapshot()["measurement"]["valid"])

    def test_missing_pyserial_reports_error_without_fallback(self):
        store = UwbStore()
        with patch.dict(sys.modules, {"serial": None}):
            serial_loop(store, "COM21")
        snapshot = store.snapshot()
        self.assertEqual(snapshot["state"], "error")
        self.assertFalse(snapshot["synthetic"])


if __name__ == "__main__":
    unittest.main()
