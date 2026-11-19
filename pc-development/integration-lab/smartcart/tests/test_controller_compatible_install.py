"""기존 화면 위에 설치하고 재실행·현지 수정 보호를 확인한다."""
import ast
import base64
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zipfile
import zlib

ROOT = Path(__file__).resolve().parents[2]
RELEASE = ROOT / 'build/releases'


def payload(name):
    tree = ast.parse((RELEASE / name).read_text(encoding='utf-8'))
    data = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name)
                and n.targets[0].id == 'PAYLOAD')
    return json.loads(zlib.decompress(base64.b64decode(data)))


class InstallerTests(unittest.TestCase):
    def test_install_on_previous_single_left_screen(self):
        installer = RELEASE / 'smartcart-controller-layout-compatible-20261117-install.py'
        with tempfile.TemporaryDirectory(dir=ROOT / 'build') as tmp:
            with zipfile.ZipFile(RELEASE / 'smartcart-d5-v2-pi4-arm64-v0.3.1.zip') as z:
                z.extractall(tmp)
            root = Path(tmp) / 'SmartCart-D5-v2'
            app = root / 'smartcart'
            for name in ('smartcart-field-preview-20261103-install.py',
                         'smartcart-field-smoothing-20261103-install.py',
                         'smartcart-single-left-preview-20261117-install.py'):
                for path, item in payload(name).items():
                    if path in ('web/index.html', 'web/css/app.css'):
                        (app / path).write_text(item['text'], encoding='utf-8', newline='')
            self.assertFalse((app / 'web/js/virtual_controls.js').exists())
            args = [sys.executable, '-X', 'utf8', str(installer), '--root', str(root), '--apply']
            result = subprocess.run(args, capture_output=True, text=True, encoding='utf-8')
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertTrue((app / 'web/js/virtual_controls.js').is_file())

    def test_install_and_existing_change_rejection(self):
        installer = RELEASE / 'smartcart-controller-layout-compatible-20261117-install.py'
        with tempfile.TemporaryDirectory(dir=ROOT / 'build') as tmp:
            with zipfile.ZipFile(RELEASE / 'smartcart-d5-v2-pi4-arm64-v0.3.1.zip') as z:
                z.extractall(tmp)
            root = Path(tmp) / 'SmartCart-D5-v2'
            app = root / 'smartcart'
            for name in ('smartcart-field-preview-20261103-install.py',
                         'smartcart-field-smoothing-20261103-install.py',
                         'smartcart-single-left-preview-20261117-install.py',
                         'smartcart-integrated-virtual-20261117-install.py'):
                for path, item in payload(name).items():
                    if path in ('web/index.html', 'web/css/app.css', 'web/js/virtual_controls.js'):
                        (app / path).write_text(item['text'], encoding='utf-8', newline='')
            args = [sys.executable, '-X', 'utf8', str(installer), '--root', str(root), '--apply']
            result = subprocess.run(args, capture_output=True, text=True, encoding='utf-8')
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run(args, capture_output=True, text=True, encoding='utf-8')
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn('이미 같은 업데이트', result.stdout)
            target = app / 'web/index.html'
            target.write_text('현지 변경', encoding='utf-8')
            result = subprocess.run(args, capture_output=True, text=True, encoding='utf-8')
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(target.read_text(encoding='utf-8'), '현지 변경')
