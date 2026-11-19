"""잘못된 실행 설정과 실측/합성 분리를 확인한다."""
import json
from pathlib import Path
import subprocess
import sys
import unittest

from smartcart_sensing.app import build_parser, validate_args, web_root

ROOT = Path(__file__).resolve().parents[1]


class SensingAppTests(unittest.TestCase):
    def args(self, *options):
        return build_parser().parse_args(["serve", "--mode", "demo", *options])

    def test_default_is_loopback(self):
        args = validate_args(self.args())
        self.assertEqual(args.host, "127.0.0.1")
        self.assertEqual(args.uwb_timeout, .5)

    def test_explicit_lan_opt_in(self):
        with self.assertRaises(ValueError):
            validate_args(self.args("--host", "0.0.0.0"))
        self.assertEqual(validate_args(self.args("--host", "0.0.0.0", "--allow-lan")).host, "0.0.0.0")

    def test_bad_topics_ports_timeout_and_mount_are_rejected(self):
        for options in (("--topic", "../../scan"), ("--port", "0"),
                        ("--uwb-timeout", "1"), ("--lidar-timeout", "0"),
                        ("--uwb-x", "1000"), ("--host", "evil.invalid"),
                        ("--topic", "/smartcart/uwb")):
            with self.subTest(options=options), self.assertRaises(ValueError):
                validate_args(self.args(*options))

    def test_real_serial_mode_requires_a_local_device(self):
        for port in (None, "rfc2217://evil.invalid:1234", "/tmp/data", "/dev/../tmp/file"):
            args = build_parser().parse_args(["serve"] + ([] if port is None else ["--serial-port", port]))
            with self.subTest(port=port), self.assertRaises(ValueError):
                validate_args(args)
        args = build_parser().parse_args(["serve", "--serial-port", "/dev/serial/by-id/usb-ESP32"])
        self.assertEqual(validate_args(args).serial_port, "/dev/serial/by-id/usb-ESP32")

    def test_web_files_resolve_without_ros(self):
        self.assertTrue((web_root() / "js/sensing.js").is_file())

    def test_doctor_never_opens_hardware(self):
        result = subprocess.run([sys.executable, str(ROOT / "sensing.py"), "doctor"],
                                capture_output=True, text=True, encoding="utf-8", timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIs(json.loads(result.stdout)["hardware_opened"], False)

    def test_help_and_version_work_without_ros(self):
        result = subprocess.run([sys.executable, str(ROOT / "sensing.py"), "--version"],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0)
        self.assertEqual(result.stdout.strip(), "0.1.0")
