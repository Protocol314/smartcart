"""검수 CLI의 미완료 판정·원문 비노출·명시적 파일 저장과 원시 재생을 검사한다."""
import contextlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from smartcart_field import cli
from smartcart_field.identity import enumerate_inventory
from smartcart_field.reporting import write_report
from smartcart_uwb.native import default_library_candidates

ROOT = Path(__file__).resolve().parents[1]


def inventory():
    return enumerate_inventory(port_infos=[SimpleNamespace(device="COM101", serial_number="private-serial",
        hwid="private-hwid", vid=100, pid=200, location="1-2", manufacturer="test", product="test")])


class FieldCliTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=ROOT / "tests")
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)

    def run_cli(self, args):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = cli.main(args)
        return code, json.loads(out.getvalue()) if out.getvalue() else None, err.getvalue()

    def test_doctor_never_enumerates_or_opens_ports(self):
        with patch("smartcart_field.identity.enumerate_inventory", side_effect=AssertionError("장치 열거 금지")):
            code, value, _ = self.run_cli(["doctor"])
        self.assertEqual(code, 0)
        self.assertFalse(value["port_opened"])
        self.assertFalse(value["drive_authorized"])

    def test_require_pi4_does_not_pass_unavailable_environment(self):
        with patch.object(cli, "environment_report", return_value={"ready": False}):
            self.assertEqual(self.run_cli(["doctor", "--require-pi4"])[0], 3)

    def test_inventory_redacts_identifiers_and_never_saves_explicit_raw_display(self):
        output = self.directory / "inventory.json"
        with patch("smartcart_field.identity.enumerate_inventory", return_value=inventory()):
            code, value, stderr = self.run_cli(["inventory", "--show-identifiers", "--output", str(output)])
        self.assertEqual(code, 0)
        self.assertIn("private-serial", stderr)
        for text in (json.dumps(value), output.read_text(encoding="utf-8")):
            for raw in ("private-serial", "private-hwid", "COM101"):
                self.assertNotIn(raw, text)
        self.assertFalse(value["port_opened"])
        self.assertFalse(value["automatic_assignment"])

    def test_existing_report_fails_before_device_inventory(self):
        output = self.directory / "existing.json"
        output.write_text("원본", encoding="utf-8")
        with patch("smartcart_field.identity.enumerate_inventory") as enumerate_ports:
            code, value, _ = self.run_cli(["inventory", "--output", str(output)])
        self.assertEqual(code, 2)
        self.assertIsNone(value)
        self.assertEqual(output.read_text(encoding="utf-8"), "원본")
        enumerate_ports.assert_not_called()

    def test_default_cad_and_unassigned_devices_are_incomplete(self):
        with patch("smartcart_field.identity.enumerate_inventory", return_value=[]):
            code, value, _ = self.run_cli(["check"])
        self.assertEqual(code, 3)
        self.assertTrue(value["mounting"]["valid"])
        self.assertFalse(value["mounting"]["ready"])
        self.assertFalse(value["readiness"]["ready"])
        self.assertFalse(value["physical_acceptance"])
        self.assertFalse(value["firmware_or_calibration_changed"])
        self.assertEqual(set(value["config_sha256"]), {"mounting", "field", "uwb", "tracking"})

    def test_monitor_only_calls_subscriber_adapter_and_does_not_open_native(self):
        with patch("smartcart_field.identity.enumerate_inventory", return_value=[]), \
                patch("smartcart_field.ros_monitor.run_monitor") as adapter, \
                patch("smartcart_uwb.native.NativeParser", side_effect=AssertionError("포트·C 파서 생성 금지")):
            code, value, _ = self.run_cli(["monitor", "--seconds", "1"])
        self.assertEqual(code, 3)
        self.assertEqual(adapter.call_args.kwargs["duration_seconds"], 1)
        self.assertFalse(value["inspection_complete"])
        self.assertFalse(value["observation"]["completion"])
        self.assertTrue(all(item["status"] == "미시험" for item in value["observation"]["manual_unplug_tests"].values()))

    def test_bad_field_configuration_is_rejected_before_inventory(self):
        bad = self.directory / "bad.json"
        bad.write_text('{"schema":"a","schema":"b"}', encoding="utf-8")
        with patch("smartcart_field.identity.enumerate_inventory") as enumerate_ports:
            self.assertEqual(self.run_cli(["check", "--field-config", str(bad)])[0], 2)
        enumerate_ports.assert_not_called()

    def test_interrupted_monitor_returns_130_without_saving_completion(self):
        target = self.directory / "interrupted.json"
        with patch("smartcart_field.identity.enumerate_inventory", return_value=[]), \
                patch("smartcart_field.ros_monitor.run_monitor", side_effect=KeyboardInterrupt):
            code, value, _ = self.run_cli(["monitor", "--seconds", "1", "--output", str(target)])
        self.assertEqual(code, 130)
        self.assertIsNone(value)
        self.assertFalse(target.exists())

    def test_closed_marker_input_is_handled_without_accepting_a_declaration(self):
        monitor = SimpleNamespace(mark=lambda *_: self.fail("닫힌 입력으로 선언 수락"))
        with contextlib.redirect_stderr(io.StringIO()), patch.object(cli.select, "select", side_effect=ValueError):
            cli.marker_reader(monitor, threading.Event(), io.StringIO())

    @unittest.skipUnless(any(path.is_file() for path in default_library_candidates()), "원시 C 파서 빌드가 필요합니다")
    def test_actual_native_175_events_report_metadata_without_positions_or_identity(self):
        code, value, _ = self.run_cli(["replay", "--input", str(ROOT / "samples/field_acceptance.synthetic.jsonl")])
        self.assertEqual(code, 0)
        self.assertEqual(value["records"], 175)
        self.assertEqual(value["original_source"], "synthetic")
        self.assertFalse(value["inspection_complete"])
        self.assertFalse(value["observation"]["completion"])
        self.assertGreater(value["observation"]["statistics"]["pair_rx_skew_ms"]["count"], 0)
        forbidden = {"tag_address", "position_xy_m", "predicted_position_xy_m", "ranges", "range_m", "bearing_rad"}
        def check(obj):
            if isinstance(obj, dict):
                self.assertFalse(forbidden.intersection(obj))
                for item in obj.values():
                    check(item)
            elif isinstance(obj, list):
                for item in obj:
                    check(item)
        check(value)

    def test_empty_or_mixed_replay_never_creates_success_report(self):
        path, output = self.directory / "input.jsonl", self.directory / "report.json"
        for data in ("", '\n'.join(json.dumps({"schema": "smartcart.target_capture.v1", "event": "tick",
                   "at_ns": index, "source": source}) for index, source in enumerate(("synthetic", "recorded")))):
            path.write_text(data, encoding="utf-8")
            with patch("smartcart_uwb.native.NativeParser"):
                code, _, _ = self.run_cli(["replay", "--input", str(path), "--output", str(output)])
            self.assertEqual(code, 2)
            self.assertFalse(output.exists())

    def test_nonfinite_and_out_of_range_duration_rejected(self):
        for value in ("nan", "inf", "0", "3601"):
            with self.subTest(value=value), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                cli.build_parser().parse_args(["monitor", "--seconds", value])

    def test_report_writer_exclusive_bounded_and_private_on_linux(self):
        target = self.directory / "report.json"
        write_report(target, {"drive_authorized": False})
        with self.assertRaises(FileExistsError):
            write_report(target, {})
        with self.assertRaises(ValueError):
            write_report(self.directory / "large.json", {"large": "x" * (1024 * 1024)})
        if sys.platform.startswith("linux"):
            self.assertEqual(target.stat().st_mode & 0o777, 0o600)

    @unittest.skipUnless(sys.platform.startswith("linux"), "Linux 표준입력 중단 검사입니다")
    def test_partial_marker_line_is_interruptible_and_never_completes(self):
        read_fd, write_fd = os.pipe()
        self.addCleanup(os.close, write_fd)
        monitor, stop = SimpleNamespace(mark=lambda *_: self.fail("미완성 선언 수락")), threading.Event()
        with os.fdopen(read_fd, "rb", buffering=0) as source, contextlib.redirect_stderr(io.StringIO()):
            worker = threading.Thread(target=cli.marker_reader, args=(monitor, stop, source))
            worker.start()
            os.write(write_fd, b"unplug uwb_")
            time.sleep(.02)
            stop.set()
            worker.join(timeout=.5)
            self.assertFalse(worker.is_alive())

    @unittest.skipUnless(sys.platform.startswith("linux"), "Linux 표준입력 상한 검사입니다")
    def test_oversize_marker_discards_full_line_and_resynchronises(self):
        read_fd, write_fd = os.pipe()
        accepted, stop = [], threading.Event()
        monitor = SimpleNamespace(mark=lambda *args: accepted.append(args) or True)
        with os.fdopen(read_fd, "rb", buffering=0) as source, contextlib.redirect_stderr(io.StringIO()):
            worker = threading.Thread(target=cli.marker_reader, args=(monitor, stop, source))
            worker.start()
            os.write(write_fd, b"x" * 150 + b"unplug uwb_right\nreplug uwb_left\n")
            os.close(write_fd)
            worker.join(timeout=.5)
            self.assertFalse(worker.is_alive())
        self.assertEqual(len(accepted), 1)
        self.assertEqual(accepted[0][:2], ("uwb_left", "replug"))


if __name__ == "__main__":
    unittest.main()
