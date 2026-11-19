"""실물 포트 개방 없이 정확 배정·중복 장치·별칭·식별값 최소화를 검사한다."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from smartcart_field.config import DeviceBinding, FieldConfig
from smartcart_field.identity import check_identity, enumerate_inventory, redact_inventory
from smartcart_uwb.config import config_from_dict

ROOT = Path(__file__).resolve().parents[1]


def port_info(port, serial_number, location, *, vid=0x0483, pid=0x5740, hwid=None):
    return SimpleNamespace(device=port, serial_number=serial_number, location=location, vid=vid, pid=pid,
                           hwid=hwid or "USB HWID PRIVATE " + str(serial_number))


def default_port_infos():
    return [port_info("COM21", "SERIAL_LEFT_PRIVATE", "2-1"),
            port_info("COM22", "SERIAL_RIGHT_PRIVATE", "2-2"),
            port_info("COM23", "SERIAL_LIDAR_PRIVATE", "2-3")]


def assigned_configs(inventory):
    payload = json.loads((ROOT / "config" / "dual_uwb.d5.example.json").read_text(encoding="utf-8"))
    for role, item in zip(("uwb_left", "uwb_right"), inventory[:2]):
        payload["sensors"][role].update(port=item["port"], expected_device_identity=item["identity"],
                                         device_identity_verified=True)
    lidar = inventory[2]
    settings = replace(FieldConfig(), lidar_device=DeviceBinding(lidar["port"], lidar["identity"], True))
    return config_from_dict(payload), settings


class FieldIdentityTests(unittest.TestCase):
    def setUp(self):
        self.infos = default_port_infos()
        self.inventory = enumerate_inventory(port_infos=self.infos)
        self.mounting, self.settings = assigned_configs(self.inventory)

    def test_single_left_does_not_require_right_but_still_checks_lidar_and_left(self):
        only_left_lidar = [self.inventory[0], self.inventory[2]]
        report = check_identity(self.mounting, self.settings, only_left_lidar, single_left=True)
        self.assertTrue(report["ready"])
        self.assertEqual(set(report["roles"]), {"uwb_left", "lidar_center"})
        self.assertFalse(check_identity(self.mounting, self.settings, only_left_lidar)["ready"])
        self.assertFalse(check_identity(self.mounting, self.settings, [self.inventory[2]], single_left=True)["ready"])

    def test_default_enumeration_uses_only_list_ports_and_never_opens_devices(self):
        enumeration = Mock(return_value=self.infos)
        forbidden = Mock(side_effect=AssertionError("장치 개방 금지"))
        modules = {"serial": SimpleNamespace(Serial=forbidden),
                   "serial.tools": SimpleNamespace(list_ports=SimpleNamespace(comports=enumeration))}
        with patch.dict("sys.modules", modules), patch("builtins.open", forbidden):
            result = enumerate_inventory()
        enumeration.assert_called_once_with(include_links=False)
        forbidden.assert_not_called()
        self.assertEqual(len(result), 3)

    def test_pi_uart_link_metadata_does_not_block_usb_inventory(self):
        uart = port_info('/dev/ttyAMA0', None, None, vid=None, pid=None, hwid='fe201000.serial')
        alias = port_info('/dev/serial1', None, None, vid=None, pid=None, hwid='LINK=/dev/ttyAMA0 LINK=/dev/ttyAMA0')
        infos = [uart] + self.infos
        def comports(*, include_links=False):
            return infos + [alias] if include_links else infos
        modules = {'serial.tools': SimpleNamespace(list_ports=SimpleNamespace(comports=comports))}
        def realpath(port):
            return '/dev/ttyAMA0' if port == '/dev/serial1' else port
        with patch.dict('sys.modules', modules), patch('smartcart_uwb.serial_reader.os.name', 'posix'), \
                patch('smartcart_uwb.serial_reader.os.path.realpath', side_effect=realpath):
            self.assertEqual(len(enumerate_inventory()), 4)
            with self.assertRaises(ValueError):
                enumerate_inventory(port_infos=comports(include_links=True))

    def test_exact_three_role_mapping_is_ready_but_never_drive_authorized(self):
        report = check_identity(self.mounting, self.settings, self.inventory)
        self.assertTrue(report["ready"])
        self.assertTrue(all(role["matched"] for role in report["roles"].values()))
        self.assertFalse(report["port_opened"])
        self.assertEqual(report["lidar_owner"], "existing_driver")
        self.assertTrue(report["provisional"])
        self.assertFalse(report["drive_authorized"])

    def test_report_and_redacted_inventory_do_not_leak_serial_hwid_or_by_id(self):
        raw = json.dumps(self.inventory)
        self.assertIn("SERIAL_LEFT_PRIVATE", raw)
        report = check_identity(self.mounting, self.settings, self.inventory)
        wire = json.dumps([report, redact_inventory(self.inventory)])
        for secret in ("SERIAL_LEFT_PRIVATE", "SERIAL_RIGHT_PRIVATE", "SERIAL_LIDAR_PRIVATE", "COM21", "USB HWID PRIVATE"):
            self.assertNotIn(secret, wire)
        self.assertEqual(len(report["roles"]["uwb_left"]["device_fingerprint"]), 16)

    def test_missing_device_and_unassigned_lidar_are_not_ready(self):
        report = check_identity(self.mounting, self.settings, self.inventory[:2])
        self.assertFalse(report["ready"])
        self.assertIn("device_missing", report["roles"]["lidar_center"]["reasons"])
        report = check_identity(self.mounting, FieldConfig(), self.inventory)
        self.assertIn("port_unassigned", report["roles"]["lidar_center"]["reasons"])
        self.assertIn("identity_unassigned", report["roles"]["lidar_center"]["reasons"])

    def test_changed_serial_and_partial_identity_are_not_accepted(self):
        changed = default_port_infos()
        changed[0].serial_number = "DIFFERENT_PRIVATE"
        report = check_identity(self.mounting, self.settings, enumerate_inventory(port_infos=changed))
        self.assertIn("identity_mismatch", report["roles"]["uwb_left"]["reasons"])
        binding = self.settings.lidar_device
        partial = replace(self.settings, lidar_device=DeviceBinding(binding.port, binding.expected_device_identity[:12], True))
        self.assertFalse(check_identity(self.mounting, partial, self.inventory)["ready"])

    def test_correct_identity_does_not_promote_unverified_assignment(self):
        binding = self.settings.lidar_device
        settings = replace(self.settings, lidar_device=replace(binding, device_identity_verified=False))
        report = check_identity(self.mounting, settings, self.inventory)
        self.assertTrue(report["roles"]["lidar_center"]["matched"])
        self.assertFalse(report["roles"]["lidar_center"]["ready"])
        self.assertIn("identity_unverified", report["roles"]["lidar_center"]["reasons"])

    def test_windows_port_alias_is_grouped_and_cross_role_alias_is_rejected(self):
        infos = default_port_infos()
        alias = deepcopy(infos[0])
        alias.device = r"\\.\COM21"
        inventory = enumerate_inventory(port_infos=infos + [alias])
        self.assertEqual(len(inventory), 3)
        self.assertEqual(len(inventory[0]["aliases"]), 2)
        settings = replace(self.settings, lidar_device=DeviceBinding(alias.device, inventory[0]["identity"], True))
        report = check_identity(self.mounting, settings, inventory)
        self.assertFalse(report["ready"])
        self.assertIn("port_alias_conflict", report["roles"]["lidar_center"]["reasons"])

    def test_by_id_alias_is_resolved_to_the_listed_device_without_opening_it(self):
        infos = default_port_infos()
        infos[2].device = "/dev/ttyACM0"
        alias = "/dev/serial/by-id/PRIVATE_LIDAR_ALIAS"
        # 이 검사는 실제 Pi/Linux의 링크 해석이다. Windows 사전 점검은 링크를 해석하지 않는다.
        with patch("smartcart_uwb.serial_reader.os.name", "posix"), \
                patch("smartcart_uwb.serial_reader.os.path.realpath", side_effect=lambda port: "/dev/ttyACM0" if port == alias else port):
            inventory = enumerate_inventory(port_infos=infos)
            lidar = next(item for item in inventory if item["port"] == "/dev/ttyACM0")
            settings = replace(self.settings, lidar_device=DeviceBinding(alias, lidar["identity"], True))
            report = check_identity(self.mounting, settings, inventory)
        self.assertTrue(report["ready"])
        self.assertNotIn("PRIVATE_LIDAR_ALIAS", json.dumps(report))

    def test_multiple_interfaces_of_same_usb_serial_cannot_fill_two_roles(self):
        infos = default_port_infos()
        infos[0].serial_number = infos[1].serial_number = "SAME_USB_PRIVATE"
        infos[0].location, infos[1].location = "2-1:1.0", "2-1:1.1"
        inventory = enumerate_inventory(port_infos=infos)
        mounting, settings = assigned_configs(inventory)
        report = check_identity(mounting, settings, inventory)
        self.assertFalse(report["ready"])
        self.assertIn("same_physical_device", report["roles"]["uwb_left"]["reasons"])

    def test_duplicate_identity_on_another_port_is_not_unique(self):
        extra = deepcopy(self.infos[0])
        extra.device = "COM24"
        report = check_identity(self.mounting, self.settings, enumerate_inventory(port_infos=self.infos + [extra]))
        self.assertIn("identity_not_unique", report["roles"]["uwb_left"]["reasons"])
        self.assertFalse(report["ready"])

    def test_specific_hwid_without_usb_serial_is_compared_exactly(self):
        infos = [port_info("COM" + str(index), None, None, vid=None, pid=None, hwid="HWID_PRIVATE_" + str(index))
                 for index in (21, 22, 23)]
        inventory = enumerate_inventory(port_infos=infos)
        mounting, settings = assigned_configs(inventory)
        self.assertTrue(check_identity(mounting, settings, inventory)["ready"])
        self.assertNotIn("HWID_PRIVATE", json.dumps(redact_inventory(inventory)))

    def test_nonspecific_hwid_never_counts_as_verified_identity(self):
        infos = default_port_infos()
        infos[2] = port_info("COM23", None, None, vid=None, pid=None, hwid="n/a")
        inventory = enumerate_inventory(port_infos=infos)
        mounting, settings = assigned_configs(inventory)
        report = check_identity(mounting, settings, inventory)
        self.assertFalse(report["ready"])
        self.assertIn("identity_not_specific", report["roles"]["lidar_center"]["reasons"])

    def test_conflicting_metadata_for_same_alias_and_inventory_overflow_are_rejected(self):
        duplicate = deepcopy(self.infos[0])
        duplicate.serial_number = "CONFLICT"
        with self.assertRaises(ValueError):
            enumerate_inventory(port_infos=self.infos + [duplicate])
        with self.assertRaises(ValueError):
            enumerate_inventory(port_infos=[self.infos[0]] * 129)

    def test_invalid_inventory_metadata_and_tampered_canonical_path_are_rejected(self):
        for name, replacement in (("vid", True), ("serial_number", "bad\nvalue"), ("device", "https://bad.invalid")):
            with self.subTest(name=name):
                infos = default_port_infos()
                setattr(infos[0], name, replacement)
                with self.assertRaises(ValueError):
                    enumerate_inventory(port_infos=infos)
        inventory = deepcopy(self.inventory)
        inventory[0]["canonical_port"] = "COM99"
        with self.assertRaises(ValueError):
            check_identity(self.mounting, self.settings, inventory)


if __name__ == "__main__":
    unittest.main()
