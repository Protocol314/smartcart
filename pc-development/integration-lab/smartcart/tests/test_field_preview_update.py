"""이전 설치본·설정 보존과 재실행을 실제 설치기로 검증한다."""
import hashlib
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class FieldPreviewUpdateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('field_preview_update',
            ROOT / 'smartcart/scripts/build-field-preview-update.py')
        cls.builder = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.builder)

    def setUp(self):
        work = tempfile.TemporaryDirectory(prefix='smartcart-preview-test-')
        self.addCleanup(work.cleanup)
        self.work = Path(work.name)
        self.root = self.work / 'pi'
        self.app = self.root / 'smartcart'
        self.app.mkdir(parents=True)
        (self.app / 'cart.py').write_text('# 설치 여부 확인\n', encoding='utf-8')
        self.old = self.builder.baseline()
        for relative, data in self.old.items():
            path = self.app / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            if data is not None:
                path.write_bytes(data)
        (self.app / 'local-config').mkdir()
        (self.app / 'local-config' / 'run.json').write_bytes(b'unchanged')
        self.installer = self.builder.build(self.work / 'release')

    def run_installer(self, *args):
        return subprocess.run([sys.executable, '-X', 'utf8', str(self.installer),
                               '--root', str(self.root), *args],
                              text=True, encoding='utf-8', capture_output=True, timeout=15)

    def test_check_then_apply_backup_and_repeat(self):
        check = self.run_installer()
        self.assertEqual(check.returncode, 0, check.stderr)
        self.assertFalse(list(self.root.glob('smartcart-preview-backup-*')))
        applied = self.run_installer('--apply')
        self.assertEqual(applied.returncode, 0, applied.stderr)
        backups = list(self.root.glob('smartcart-preview-backup-*'))
        self.assertEqual(len(backups), 1)
        for relative, old in self.old.items():
            self.assertEqual((self.app / relative).read_bytes(), (ROOT / 'smartcart' / relative).read_bytes())
            if old is not None:
                self.assertEqual((backups[0] / relative).read_bytes(), old)
        self.assertEqual((self.app / 'local-config' / 'run.json').read_bytes(), b'unchanged')
        repeated = self.run_installer('--apply')
        self.assertEqual(repeated.returncode, 0, repeated.stderr)
        self.assertIn('이미 같은 업데이트', repeated.stdout)
        self.assertEqual(len(list(self.root.glob('smartcart-preview-backup-*'))), 1)

    def test_unknown_local_change_rejected_before_any_write(self):
        target = self.app / 'web/js/app.js'
        target.write_bytes(b'user modification')
        result = self.run_installer('--apply')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('알 수 없는 현지 수정', result.stderr)
        self.assertEqual(target.read_bytes(), b'user modification')
        self.assertFalse((self.app / 'web/js/field_preview.js').exists())
        self.assertFalse(list(self.root.glob('smartcart-preview-backup-*')))

    def test_checksum_and_stable_output(self):
        self.assertEqual(hashlib.sha256(self.installer.read_bytes()).hexdigest(),
                         self.installer.with_suffix('.py.sha256').read_text().split()[0])
        self.assertEqual(self.builder.build(self.installer.parent), self.installer)


if __name__ == '__main__':
    unittest.main()
