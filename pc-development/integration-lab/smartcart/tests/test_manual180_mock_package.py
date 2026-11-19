"""배포본에서 GPIO 경로가 빠지고 모의 제어만 동작하는지 검사한다."""
import ast
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / 'build/releases/smartcart-manual180-mock-20261117.zip'


class MockPackageTests(unittest.TestCase):
    def test_gpio_path_removed_and_mock_backend_runs(self):
        with tempfile.TemporaryDirectory(dir=ROOT / 'build') as tmp:
            with zipfile.ZipFile(PACKAGE) as archive:
                self.assertIsNone(archive.testzip())
                archive.extractall(tmp)
            app = Path(tmp) / 'smartcart'
            drive = (app / 'drive.py').read_text(encoding='utf-8')
            motor = (app / 'smartcart_motor/manual_remote.py').read_text(encoding='utf-8')
            ast.parse(drive, feature_version=(3, 10))
            ast.parse(motor, feature_version=(3, 10))
            self.assertNotIn('class RPiBackend', motor)
            self.assertNotIn('import RPi.GPIO', motor)
            self.assertIn('choices=("mock",)', drive)
            self.assertIn('type(backend) is not MockBackend', drive)
            self.assertIn('type(backend) is not MockBackend', motor)
            command = '''
from smartcart_motor.manual_remote import ManualDrive, MockBackend
backend = MockBackend()
drive = ManualDrive(backend)
assert drive.acquire('one')
assert drive.snapshot()['hardware'] is False
try:
    ManualDrive(object())
except ValueError:
    pass
else:
    raise AssertionError('모의 백엔드 이외 입력을 거부하지 못했습니다')
drive.close()
assert backend.values == (0, 0)
'''
            result = subprocess.run([sys.executable, '-c', command], cwd=app,
                                    text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            ui = subprocess.run(['node', 'tests/test_manual_controls.cjs'], cwd=app,
                                text=True, capture_output=True)
            self.assertEqual(ui.returncode, 0, ui.stdout + ui.stderr)


if __name__ == '__main__':
    unittest.main()
