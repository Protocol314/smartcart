"""센서 없는 검증기의 실제 C 재생, 실패 전파·정리와 집계 저장 경계를 검사한다."""
from contextlib import ExitStack, redirect_stderr, redirect_stdout
from copy import deepcopy
import argparse
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import MagicMock, patch


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("smartcart_offline_verification", ROOT / "scripts" / "verify-offline.py")
offline = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(offline)


class OfflineVerificationTests(unittest.TestCase):
    def setUp(self):
        folder = ROOT.parent / "build" / "verification" / "offline-unit" / "tmp"
        folder.mkdir(parents=True, exist_ok=True)
        directory = tempfile.TemporaryDirectory(dir=folder)
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)

    def mock_checks(self):
        stack = ExitStack()
        stack.enter_context(patch.object(offline, "inspect_native", return_value=(Path("local.dll"), {"passed": True})))
        for name in ("inspect_field", "inspect_tracking", "inspect_http"):
            stack.enter_context(patch.object(offline, name, return_value={"passed": True}))
        return stack

    def test_time_limit_rejects_nonfinite_and_out_of_range_values(self):
        for value in ("nan", "inf", "0", "4.99", "60.01", "invalid"):
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                offline.seconds_value(value)
        self.assertEqual(offline.seconds_value("5"), 5)
        self.assertEqual(offline.seconds_value("60"), 60)

    def test_private_fields_are_rejected_at_every_depth(self):
        for name in offline.FORBIDDEN_REPORT_KEYS:
            with self.subTest(name=name), self.assertRaises(RuntimeError):
                offline.summary_only({"checks": [{"nested": {name: "PRIVATE"}}]})
        offline.summary_only({"checks": [{"records": 175, "passed": True}]})

    def test_real_native_field_and_tracking_checkpoints(self):
        library, native = offline.inspect_native()
        field = offline.inspect_field(library)
        tracking = offline.inspect_tracking(library)
        self.assertTrue(native["abi_loaded"])
        self.assertFalse(native["device_opened"])
        self.assertEqual((field["records"], tracking["records"]), (175, 175))
        self.assertEqual(tracking["checkpoints"], 41)
        self.assertGreater(field["id_mismatches"], 0)
        self.assertGreater(field["checksum_failures"], 0)
        offline.summary_only({"field": field, "tracking": tracking})

    def test_existing_output_is_rejected_before_any_check(self):
        destination = self.directory / "kept.json"
        destination.write_text("ORIGINAL", encoding="utf-8")
        with patch.object(offline, "inspect_native") as native, redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertEqual(offline.main(["--output", str(destination)]), 2)
        native.assert_not_called()
        self.assertEqual(destination.read_text(encoding="utf-8"), "ORIGINAL")

    def test_opt_in_report_has_only_aggregates_and_no_acceptance(self):
        destination = self.directory / "report.json"
        stdout = io.StringIO()
        with self.mock_checks(), redirect_stdout(stdout):
            self.assertEqual(offline.main(["--output", str(destination)]), 0)
        report = json.loads(destination.read_text(encoding="utf-8"))
        self.assertTrue(report["passed"])
        self.assertFalse(report["physical_acceptance"])
        self.assertFalse(report["drive_authorized"])
        self.assertFalse(report["device_opened"])
        self.assertEqual(report["original_source"], "synthetic")
        self.assertEqual(json.loads(stdout.getvalue()), report)
        offline.summary_only(report)

    def test_failure_never_echoes_native_path_or_marks_pass(self):
        stdout, stderr = io.StringIO(), io.StringIO()
        with patch.object(offline, "inspect_native", side_effect=OSError("PRIVATE_PATH_AND_TAG")), redirect_stdout(stdout), redirect_stderr(stderr):
            self.assertEqual(offline.main([]), 2)
        report = json.loads(stdout.getvalue())
        self.assertFalse(report["passed"])
        self.assertEqual(report["active_check"], "native")
        self.assertNotIn("PRIVATE_PATH_AND_TAG", stdout.getvalue() + stderr.getvalue())

    def test_child_failure_is_not_skipped_and_owned_child_is_reaped(self):
        child = MagicMock()
        child.poll.return_value = 7
        with patch.object(offline, "available_port", return_value=18764), patch.object(offline.subprocess, "Popen", return_value=child) as launch:
            with self.assertRaisesRegex(RuntimeError, "서버가 검사 도중"):
                offline.inspect_http(Path("local.dll"), 5)
        self.assertIs(launch.call_args.kwargs["shell"], False)
        child.terminate.assert_not_called()
        self.assertGreaterEqual(child.poll.call_count, 2)

    def test_connection_permission_error_still_terminates_child(self):
        child = MagicMock()
        child.poll.side_effect = [None, None, 1]
        with patch.object(offline, "available_port", return_value=18764), patch.object(offline.subprocess, "Popen", return_value=child), patch.object(offline, "request_json", side_effect=PermissionError("DENIED")):
            with self.assertRaises(PermissionError):
                offline.inspect_http(Path("local.dll"), 5)
        child.terminate.assert_called_once()
        child.wait.assert_called_once_with(timeout=3)

    def test_timeout_escalates_only_owned_process_to_kill(self):
        child = MagicMock()
        child.poll.side_effect = [None, -9]
        child.wait.side_effect = [subprocess.TimeoutExpired("owned", 3), -9]
        offline.stop_child(child)
        child.terminate.assert_called_once()
        child.kill.assert_called_once()
        self.assertEqual(child.wait.call_count, 2)

    def test_http_invalid_and_ended_views_cannot_keep_measurement_or_prediction(self):
        good = {"schema": "smartcart.tracking_view.v1", "mode": "replay", "source_mode": "replay",
                "drive_authorized": False, "provisional": True, "valid": False, "state": "ended",
                "result": {"track": {"position_xy_m": None, "velocity_xy_mps": None,
                                      "track_id": None, "predicted_position_xy_m": None}}}
        offline.validate_http_view(good)
        for key, value in (("position_xy_m", [1, 2]), ("velocity_xy_mps", [0, 0]),
                           ("track_id", 1), ("predicted_position_xy_m", [1, 2])):
            bad = deepcopy(good)
            bad["result"]["track"][key] = value
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                offline.validate_http_view(bad)
        bad = deepcopy(good)
        bad["drive_authorized"] = True
        with self.assertRaises(RuntimeError):
            offline.validate_http_view(bad)


if __name__ == "__main__":
    unittest.main()
