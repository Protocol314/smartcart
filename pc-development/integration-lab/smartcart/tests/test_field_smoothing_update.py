"""기존 Pi 화면의 보존·백업·현지 수정 거부를 실제 설치기로 검사한다."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class SmoothingUpdateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('field_smoothing_update',
            ROOT / 'smartcart/scripts/build-field-smoothing-update.py')
        cls.builder = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.builder)

    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix='smartcart-smoothing-test-')
        self.addCleanup(directory.cleanup)
        work = Path(directory.name)
        self.root = work / 'pi'
        self.app = self.root / 'smartcart'
        self.app.mkdir(parents=True)
        (self.app / 'cart.py').write_text('# 기존 설치본\n', encoding='utf-8')
        self.old = self.builder.baseline()
        for relative, data in self.old.items():
            path = self.app / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        (self.app / 'local-config').mkdir()
        (self.app / 'local-config' / 'run.json').write_bytes(b'unchanged')
        self.installer = self.builder.build(work / 'release')

    def run_installer(self, *args):
        return subprocess.run([sys.executable, '-X', 'utf8', str(self.installer),
            '--root', str(self.root), *args], capture_output=True, text=True, encoding='utf-8', timeout=15)

    def test_check_apply_backup_and_repeat(self):
        check = self.run_installer()
        self.assertEqual(check.returncode, 0, check.stderr)
        self.assertFalse(list(self.root.glob('smartcart-smoothing-backup-*')))
        applied = self.run_installer('--apply')
        self.assertEqual(applied.returncode, 0, applied.stderr)
        backups = list(self.root.glob('smartcart-smoothing-backup-*'))
        self.assertEqual(len(backups), 1)
        for relative, old in self.old.items():
            self.assertEqual((backups[0] / relative).read_bytes(), old)
            self.assertEqual((self.app / relative).read_bytes(), (ROOT / 'smartcart' / relative).read_bytes())
        self.assertEqual((self.app / 'local-config' / 'run.json').read_bytes(), b'unchanged')
        repeated = self.run_installer('--apply')
        self.assertEqual(repeated.returncode, 0, repeated.stderr)
        self.assertIn('이미 같은 업데이트', repeated.stdout)

    def test_unrecognized_change_rejected_before_any_write(self):
        target = self.app / 'web/js/app.js'
        target.write_bytes('현지 사용자 수정'.encode('utf-8'))
        result = self.run_installer('--apply')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('알 수 없는 현지 수정', result.stderr)
        self.assertEqual(target.read_bytes(), '현지 사용자 수정'.encode('utf-8'))
        self.assertFalse(list(self.root.glob('smartcart-smoothing-backup-*')))


if __name__ == '__main__':
    unittest.main()
