"""기존 설치본의 검증·백업·재실행·현지 수정 보존을 실제 설치기로 검사한다."""
import hashlib
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[2]


class DualUwbUpdateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('ui_update_builder', ROOT / 'smartcart/scripts/build-dual-uwb-update.py')
        cls.builder = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.builder)
        cls.build_directory = tempfile.TemporaryDirectory(prefix='smartcart-ui-build-')
        cls.addClassCleanup(cls.build_directory.cleanup)
        cls.installer = cls.builder.build(output_dir=cls.build_directory.name)

    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix='smartcart-ui-test-')
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.app = self.root / 'smartcart'
        self.app.mkdir()
        (self.app / 'cart.py').write_text('# 기존 설치본\n', encoding='utf-8')
        self.original = {}
        with zipfile.ZipFile(ROOT / 'build/releases/smartcart-d5-v2-pi4-arm64-v0.3.1.zip') as archive:
            for relative in self.builder.FILES:
                target = self.app / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                matches = [name for name in archive.namelist() if name.endswith('smartcart/' + relative)]
                self.original[relative] = archive.read(matches[0]) if matches else None
                if matches:
                    target.write_bytes(self.original[relative])
        local = self.app / 'local-config'
        local.mkdir()
        (local / 'run.json').write_bytes(b'unchanged local settings')

    def run_installer(self, *arguments):
        return subprocess.run([sys.executable, '-X', 'utf8', str(self.installer), '--root', str(self.root), *arguments],
                              capture_output=True, text=True, encoding='utf-8', timeout=10)

    def test_check_changes_nothing_and_apply_backs_up_exact_originals(self):
        checked = self.run_installer()
        self.assertEqual(checked.returncode, 0, checked.stdout + checked.stderr)
        self.assertEqual(list(self.root.glob('smartcart-ui-backup-*')), [])
        for relative, data in self.original.items():
            target = self.app / relative
            self.assertEqual(target.read_bytes() if target.exists() else None, data)
        applied = self.run_installer('--apply')
        self.assertEqual(applied.returncode, 0, applied.stdout + applied.stderr)
        backups = list(self.root.glob('smartcart-ui-backup-*'))
        self.assertEqual(len(backups), 1)
        for relative, data in self.original.items():
            self.assertEqual((self.app / relative).read_bytes(), (ROOT / 'smartcart' / relative).read_bytes())
            if data is not None:
                self.assertEqual((backups[0] / relative).read_bytes(), data)
        self.assertEqual((self.app / 'local-config/run.json').read_bytes(), b'unchanged local settings')
        repeated = self.run_installer('--apply')
        self.assertEqual(repeated.returncode, 0, repeated.stderr)
        self.assertIn('이미 같은 업데이트', repeated.stdout)
        self.assertEqual(len(list(self.root.glob('smartcart-ui-backup-*'))), 1)

    def test_unrecognized_local_change_is_rejected_before_any_write(self):
        target = self.app / 'web/js/app.js'
        target.write_bytes(b'local edit must survive')
        result = self.run_installer('--apply')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('알 수 없는 현지 수정', result.stderr)
        self.assertEqual(target.read_bytes(), b'local edit must survive')
        self.assertFalse((self.app / 'smartcart_sensing/dual_uwb.py').exists())
        self.assertEqual(list(self.root.glob('smartcart-ui-backup-*')), [])

    def test_generated_digest_matches_published_checksum(self):
        sha = hashlib.sha256(self.installer.read_bytes()).hexdigest()
        self.assertEqual(self.installer.with_suffix('.py.sha256').read_text().split()[0], sha)

    def test_previously_installed_nine_ui_files_are_preserved_when_adding_restart_fix(self):
        for relative in self.builder.FILES:
            if relative != 'smartcart_runner/supervisor.py':
                (self.app / relative).write_bytes((ROOT / 'smartcart' / relative).read_bytes())
        result = self.run_installer('--apply')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('변경 필요 1개', result.stdout)
        self.assertEqual((self.app / 'smartcart_runner/supervisor.py').read_bytes(),
                         (ROOT / 'smartcart/smartcart_runner/supervisor.py').read_bytes())
