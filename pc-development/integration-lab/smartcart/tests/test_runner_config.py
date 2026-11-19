"""실행 준비는 미검증 값을 유지하고 명령·경로 경계를 넘어가지 않는다."""
from dataclasses import replace
from contextlib import nullcontext
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from smartcart_runner.app import build_parser, main
from smartcart_runner.config import default_config, initialize, load_config
from smartcart_runner.plan import build_plan, check_plan

ROOT = Path(__file__).resolve().parents[1]


class RunnerConfigTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="runner-", dir=ROOT / "tests")
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name) / "설정 공백"
        self.path = initialize(self.directory)

    def change(self, **values):
        data = json.loads(self.path.read_text(encoding="utf-8"))
        data.update(values)
        self.path.write_text(json.dumps(data), encoding="utf-8")

    def test_init_preserves_unverified_and_unknown_values(self):
        mounting = json.loads((self.directory / "mounting.json").read_text(encoding="utf-8"))
        field = json.loads((self.directory / "field.json").read_text(encoding="utf-8"))
        uwb = json.loads((self.directory / "uwb.json").read_text(encoding="utf-8"))
        self.assertTrue(all(item["port"] is None and not item["device_identity_verified"]
                            and not item["mounting_verified"] for item in mounting["sensors"].values()))
        self.assertFalse(field["origin_verified"])
        self.assertEqual(uwb["range_model"], "unknown")
        self.assertFalse(uwb["assumptions_verified"])

    def test_init_refuses_existing_directory_without_changing_files(self):
        before = {path.name: path.read_bytes() for path in self.directory.iterdir()}
        with self.assertRaises(FileExistsError):
            initialize(self.directory)
        self.assertEqual(before, {path.name: path.read_bytes() for path in self.directory.iterdir()})

    def test_relative_paths_follow_run_file_not_current_directory(self):
        config = load_config(self.path)
        self.assertEqual(config.mounting_config, self.directory / "mounting.json")

    def test_unknown_and_duplicate_keys_are_refused(self):
        self.change(command="anything")
        with self.assertRaises(ValueError):
            load_config(self.path)
        self.path.write_text('{"schema":"a","schema":"b"}', encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "중복"):
            load_config(self.path)

    def test_port_frame_and_remote_paths_are_bounded(self):
        cases = ({"http_port": True}, {"http_port": 65536}, {"lidar_port": "/dev/ttyUSB0;touch /tmp/x"},
                 {"lidar_frame_id": "laser --ros-args"}, {"lidar_frame_id": "x" * 65},
                 {"mounting_config": "https://example.test/settings"}, {"field_config": "bad\npath"})
        original = self.path.read_text(encoding="utf-8")
        for values in cases:
            with self.subTest(values=values):
                self.path.write_text(original, encoding="utf-8")
                self.change(**values)
                with self.assertRaises(ValueError):
                    load_config(self.path)

    def test_oversized_config_is_refused(self):
        self.path.write_text(" " * 65537, encoding="utf-8")
        with self.assertRaises(ValueError):
            load_config(self.path)

    def test_no_config_defaults_to_replay_without_serial_or_ros(self):
        args = build_parser().parse_args(["start"])
        self.assertEqual(args.mode, "replay")
        with patch("smartcart_field.identity.enumerate_inventory", side_effect=AssertionError("USB 열거 금지")):
            plan = build_plan(default_config())
        self.assertEqual(len(plan["commands"]), 1)
        command = plan["commands"][0]
        self.assertEqual(command["name"], "replay_gui")
        self.assertIn("--tracking-loop", command["argv"])
        self.assertIn("127.0.0.1", command["argv"])
        self.assertNotIn("--serial-port", command["argv"])
        self.assertFalse(plan["port_opened"])
        self.assertFalse(plan["drive_authorized"])

    def test_live_plan_marks_missing_devices_and_still_does_not_enumerate(self):
        with patch("smartcart_field.identity.enumerate_inventory", side_effect=AssertionError("USB 열거 금지")):
            plan = build_plan(load_config(self.path), "live")
        self.assertFalse(plan["configuration_ready"])
        self.assertIn("lidar_port_unassigned", plan["blockers"])
        self.assertEqual([item["name"] for item in plan["commands"]], ["lidar", "uwb", "tracking_gui"])
        self.assertIn("serial_baudrate:=460800", plan["commands"][0]["argv"])
        self.assertIn("frame_id:=lidar_scan", plan["commands"][0]["argv"])

    def test_external_driver_is_not_started_or_owned(self):
        plan = build_plan(replace(load_config(self.path), lidar_mode="external"), "live")
        self.assertEqual([item["name"] for item in plan["commands"]], ["uwb", "tracking_gui"])
        self.assertTrue(any("external_lidar" in warning for warning in plan["warnings"]))

    def test_lidar_frame_and_port_binding_mismatch_block_live(self):
        plan = build_plan(replace(load_config(self.path), lidar_frame_id="laser", lidar_port="/dev/ttyUSB9"), "live")
        self.assertIn("lidar_frame_mismatch", plan["blockers"])
        self.assertIn("lidar_port_binding_mismatch", plan["blockers"])

    def test_unknown_distance_model_is_warning_after_identity_assignment(self):
        mounting_path, field_path = self.directory / "mounting.json", self.directory / "field.json"
        mounting = json.loads(mounting_path.read_text(encoding="utf-8"))
        for index, device in enumerate(mounting["sensors"].values()):
            device.update(port="/dev/ttyACM" + str(index), expected_device_identity="serial=fake" + str(index),
                          device_identity_verified=True)
        field = json.loads(field_path.read_text(encoding="utf-8"))
        field["lidar_device"].update(port="/dev/ttyUSB0", expected_device_identity="serial=fake-lidar", device_identity_verified=True)
        mounting_path.write_text(json.dumps(mounting), encoding="utf-8")
        field_path.write_text(json.dumps(field), encoding="utf-8")
        self.change(lidar_port="/dev/ttyUSB0")
        plan = build_plan(load_config(self.path), "live")
        self.assertTrue(plan["configuration_ready"])
        self.assertTrue(plan["warnings"])
        self.assertFalse(plan["physical_acceptance"])

    def test_explicit_missing_config_is_not_silently_replaced(self):
        self.assertEqual(main(["--config", str(self.directory / "missing.json"), "plan"]), 2)

    def test_seconds_rejects_nonfinite_and_unbounded_values(self):
        for value in ("0", "-1", "nan", "inf", "3601"):
            with self.subTest(value=value), self.assertRaises(SystemExit):
                build_parser().parse_args(["start", "--seconds", value])

    def test_missing_native_library_makes_replay_check_fail_without_usb(self):
        with patch("smartcart_uwb.native.NativeParser", side_effect=FileNotFoundError("없음")), \
                patch("smartcart_field.identity.enumerate_inventory", side_effect=AssertionError("USB 금지")):
            report = check_plan(default_config(), build_plan(default_config()))
        self.assertFalse(report["ready"])
        self.assertIn("native_parser_unavailable", report["blockers"])

    def test_start_acquires_user_live_lock_only_for_live_mode(self):
        for mode, scopes in (("replay", ["http"]), ("live", ["http", "live"])):
            requested = []
            def lock(port, *, scope="http"):
                requested.append(scope)
                return nullcontext()
            with self.subTest(mode=mode), \
                    patch("smartcart_runner.app.InstanceLock", side_effect=lock), \
                    patch("smartcart_runner.app.check_plan", return_value={"ready": True}), \
                    patch("smartcart_runner.app.ensure_http_available"), \
                    patch("smartcart_runner.app.supervise", return_value=0):
                self.assertEqual(main(["start", "--mode", mode, "--seconds", ".1"]), 0)
                self.assertEqual(requested, scopes)


if __name__ == "__main__":
    unittest.main()
