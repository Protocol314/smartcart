"""GPIO 없이 원격 화면의 가상 명령과 정지·역전 경계를 검증한다."""
import ast
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/motor_remote_virtual.py'


class VirtualRemoteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('motor_remote_virtual', SCRIPT)
        cls.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'aiohttp': types.SimpleNamespace(web=object())}):
            spec.loader.exec_module(cls.module)

    def test_wiring_is_documented_without_gpio_imports(self):
        tree = ast.parse(SCRIPT.read_text(encoding='utf-8'))
        modules = [n.module or '' for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
        modules += [alias.name for n in ast.walk(tree) if isinstance(n, ast.Import) for alias in n.names]
        self.assertFalse(any('gpio' in module.lower() for module in modules))
        self.assertEqual(self.module.Motors.pins, ((17, 27), (23, 24)))
        backend = self.module.Motors()
        backend.write([43, -43])
        self.assertEqual(backend.values, [43, -43])
        backend.stop()
        self.assertEqual(backend.values, [0, 0])

    def test_forward_reverse_and_tank_turn_are_bounded(self):
        target = self.module.target
        self.assertEqual(target(1, 0, 43), [43, 43])
        self.assertEqual(target(-1, 0, 43), [-43, -43])
        self.assertEqual(target(0, 1, 43), [-43, 43])
        self.assertEqual(target(0, -1, 43), [43, -43])
        self.assertTrue(all(abs(v) <= 43 for v in target(1, 1, 999)))

    def test_reversal_wait_and_stop(self):
        drive = self.module.Drive()
        for i in range(40):
            drive.step([43, 43], .04, i * .04)
        self.assertTrue(all(v > 0 for v in drive.value))
        drive.stop(2.0)
        self.assertEqual(drive.step([-43, -43], .04, 2.2), [0, 0])
        self.assertTrue(all(v < 0 for v in drive.step([-43, -43], .04, 2.6)))

    def test_localhost_only(self):
        source = SCRIPT.read_text(encoding='utf-8')
        self.assertIn('host="127.0.0.1"', source)
        self.assertNotIn('host="0.0.0.0"', source)
        self.assertIn('web.HTTPForbidden()', source)
