"""Pi에 적용된 이전 파일을 재현해 새 설치기의 검사·백업·재실행을 검증한다."""
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
    value = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                 and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name)
                 and n.targets[0].id == 'PAYLOAD')
    return json.loads(zlib.decompress(base64.b64decode(value)))


class PreviewInstallerTests(unittest.TestCase):
    def test_existing_install_and_repeat(self):
        name = 'smartcart-single-left-preview-20261117-install.py'
        with tempfile.TemporaryDirectory(dir=ROOT / 'build') as directory:
            with zipfile.ZipFile(RELEASE / 'smartcart-d5-v2-pi4-arm64-v0.3.1.zip') as archive:
                archive.extractall(directory)
            root = Path(directory) / 'SmartCart-D5-v2'
            app = root / 'smartcart'
            for previous in ('smartcart-field-preview-20261103-install.py',
                             'smartcart-field-smoothing-20261103-install.py',
                             'smartcart-single-left-pi-identity-20261117-install.py'):
                for path, item in payload(previous).items():
                    target = app / path
                    target.parent.mkdir(parents=True, exist_ok=True)
                    target.write_text(item['text'], encoding='utf-8', newline='')
            config = app / 'local-config/uwb.json'
            config.parent.mkdir(exist_ok=True)
            config.write_text('{"range_model":"unknown"}', encoding='utf-8')
            for _ in range(2):
                run = subprocess.run([sys.executable, '-X', 'utf8', str(RELEASE / name), '--root', str(root), '--apply'],
                                     capture_output=True, text=True, encoding='utf-8')
                self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            self.assertIn('이미 같은 업데이트', run.stdout)
            self.assertEqual(config.read_text(encoding='utf-8'), '{"range_model":"unknown"}')
