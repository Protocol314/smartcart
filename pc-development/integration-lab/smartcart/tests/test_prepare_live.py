"""실물 포트를 열지 않고 명시한 시리얼 역할 배정과 백업 보존을 검사한다."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from smartcart_field.identity import enumerate_inventory

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('prepare_live', ROOT / 'scripts/prepare-live.py')
PREPARE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREPARE)


def inventory():
    return enumerate_inventory(port_infos=[
        SimpleNamespace(device='/dev/ttyACM0', vid=0x0483, pid=0x5740,
                        serial_number='left-test', location='1-3:1.0', hwid='left-test'),
        SimpleNamespace(device='/dev/ttyACM1', vid=0x0483, pid=0x5740,
                        serial_number='right-test', location='1-4:1.0', hwid='right-test'),
        SimpleNamespace(device='/dev/ttyUSB0', vid=0x10C4, pid=0xEA60,
                        serial_number='lidar-test', location='1-2', hwid='lidar-test'),
    ])


class PrepareLiveTests(unittest.TestCase):
    def test_current_identity_is_bound_and_old_files_are_backed_up(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            backup = PREPARE.prepare(root, ('left-test', 'right-test', 'lidar-test'),
                                     0xB16E, inventory=inventory())
            mounting = json.loads((root / 'local-config/mounting.json').read_text())
            field = json.loads((root / 'local-config/field.json').read_text())
            uwb = json.loads((root / 'local-config/uwb.json').read_text())
            run = json.loads((root / 'local-config/run.json').read_text())
            self.assertEqual(mounting['allowed_tag_address'], 0xB16E)
            self.assertIn('serial=left-test', mounting['sensors']['uwb_left']['expected_device_identity'])
            self.assertIn('serial=right-test', mounting['sensors']['uwb_right']['expected_device_identity'])
            self.assertTrue(field['lidar_device']['device_identity_verified'])
            self.assertFalse(mounting['sensors']['uwb_left']['mounting_verified'])
            self.assertEqual(uwb['range_model'], 'unknown')
            self.assertFalse(uwb['assumptions_verified'])
            self.assertEqual(run['lidar_mode'], 'managed')
            before = json.loads((backup / 'mounting.json').read_text())
            self.assertFalse(before['sensors']['uwb_left']['device_identity_verified'])

    def test_missing_or_wrong_hardware_never_creates_configuration(self):
        for values in (('missing', 'right-test', 'lidar-test'),
                       ('left-test', 'left-test', 'lidar-test')):
            with self.subTest(values=values), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                with self.assertRaises(ValueError):
                    PREPARE.prepare(root, values, 0xB16E, inventory=inventory())
                self.assertFalse((root / 'local-config').exists())

    def test_invalid_tag_never_creates_configuration(self):
        for tag in (True, -1, 65536):
            with self.subTest(tag=tag), tempfile.TemporaryDirectory() as temporary:
                with self.assertRaises(ValueError):
                    PREPARE.prepare(Path(temporary), ('left-test', 'right-test', 'lidar-test'),
                                    tag, inventory=inventory())
                self.assertFalse((Path(temporary) / 'local-config').exists())


if __name__ == '__main__':
    unittest.main()
