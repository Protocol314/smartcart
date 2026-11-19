"""LAN 가상 서버의 주소·인증서·GPIO 미접근 경계를 검사한다."""
import ast
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/motor_remote_virtual_lan.py'


class VirtualLanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('virtual_lan', SCRIPT)
        cls.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'aiohttp': types.SimpleNamespace(web=object())}):
            spec.loader.exec_module(cls.module)

    def test_no_gpio_and_explicit_private_ipv4_only(self):
        tree = ast.parse(SCRIPT.read_text(encoding='utf-8'))
        imports = [n.module or '' for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
        imports += [a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names]
        self.assertFalse(any('gpio' in name.lower() for name in imports))
        self.assertEqual(self.module.validated_address('192.168.43.212'), '192.168.43.212')
        for value in ('0.0.0.0', '127.0.0.1', '8.8.8.8', '::1', 'bad'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.module.validated_address(value)

    def test_tls_and_websocket_guards_present(self):
        source = SCRIPT.read_text(encoding='utf-8')
        self.assertIn('ssl_context=context', source)
        self.assertIn('TLSVersion.TLSv1_2', source)
        self.assertIn('secrets.compare_digest', source)
        self.assertIn('web.HTTPTooManyRequests()', source)
        self.assertIn('origin != f"https://{BIND_IP}:8443"', source)
        self.assertEqual(self.module.Motors.pins, ((17, 27), (23, 24)))

    def test_certificate_generation_and_no_overwrite(self):
        openssl = shutil.which('openssl')
        fallback = Path(r'C:\Program Files\Git\mingw64\bin\openssl.exe')
        if not openssl and not fallback.exists():
            self.skipTest('OpenSSL 없음')
        with tempfile.TemporaryDirectory() as temporary:
            env_path = str(fallback.parent) + os.pathsep + os.environ.get('PATH', '') if not openssl else os.environ['PATH']
            with patch.dict(os.environ, {'PATH': env_path}):
                files = self.module.make_certificates('192.168.43.212', Path(temporary) / 'private')
                check = subprocess.run(['openssl', 'verify', '-CAfile', str(files['ca.crt']), '-verify_ip',
                                        '192.168.43.212', str(files['server.crt'])], capture_output=True, text=True)
                self.assertEqual(check.returncode, 0, check.stderr)
                wrong = subprocess.run(['openssl', 'verify', '-CAfile', str(files['ca.crt']), '-verify_ip',
                                        '192.168.43.213', str(files['server.crt'])], capture_output=True, text=True)
                self.assertNotEqual(wrong.returncode, 0)
                self.assertEqual(len(self.module.fingerprint(files['ca.crt'])), 64)
                with self.assertRaises(FileExistsError):
                    self.module.make_certificates('192.168.43.212', Path(temporary) / 'private')
