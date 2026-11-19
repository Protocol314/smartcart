"""PC에서 미래 Pi 포트를 점검하되 실물 식별이나 링크 확인을 대신하지 않는다."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from smartcart_uwb.config import canonical_port as config_port, config_from_dict
from smartcart_uwb.serial_reader import canonical_port as reader_port
from smartcart_field.config import DeviceBinding


ROOT = Path(__file__).resolve().parents[1]


class OfflinePortConfigTests(unittest.TestCase):
    def test_windows_pi_ports_are_lexical_and_never_resolved_on_pc(self):
        with patch("os.name", "nt"), patch("os.path.realpath", side_effect=AssertionError("PC 링크 조회 금지")):
            for canonical in (config_port, reader_port):
                self.assertEqual(canonical("/dev/serial/by-id/left"), "posix:/dev/serial/by-id/left")
                self.assertEqual(canonical("/dev/serial/by-id/../by-id/left"), canonical("/dev/serial/by-id/left"))
                for value in ("/dev/../etc/passwd", "/dev/serial\\left", "/dev/left\n"):
                    with self.assertRaises(ValueError):
                        canonical(value)

    def test_windows_can_validate_unverified_pi_configuration(self):
        payload = json.loads((ROOT / "config/dual_uwb.d5.example.json").read_text(encoding="utf-8"))
        payload["sensors"]["uwb_left"]["port"] = "/dev/serial/by-id/left"
        payload["sensors"]["uwb_right"]["port"] = "/dev/serial/by-id/right"
        with patch("os.name", "nt"):
            config = config_from_dict(payload)
            binding = DeviceBinding("/dev/serial/by-id/lidar", "", False)
        self.assertFalse(config.drive_authorized)
        self.assertFalse(config.sensors["uwb_left"].device_identity_verified)
        self.assertFalse(binding.device_identity_verified)

    def test_windows_duplicate_pi_aliases_are_rejected(self):
        payload = json.loads((ROOT / "config/dual_uwb.d5.example.json").read_text(encoding="utf-8"))
        payload["sensors"]["uwb_left"]["port"] = "/dev/serial/by-id/left"
        payload["sensors"]["uwb_right"]["port"] = "/dev/serial/by-id/../by-id/left"
        with patch("os.name", "nt"), self.assertRaises(ValueError):
            config_from_dict(payload)

    def test_native_linux_keeps_actual_link_resolution(self):
        with patch("os.name", "posix"), patch("os.path.isabs", return_value=True), \
                patch("os.path.normcase", side_effect=lambda value: value), \
                patch("os.path.realpath", return_value="/dev/ttyACM1"):
            self.assertEqual(config_port("/dev/serial/by-id/left"), "/dev/ttyACM1")
            self.assertEqual(reader_port("/dev/serial/by-id/left"), "/dev/ttyACM1")


if __name__ == "__main__":
    unittest.main()
