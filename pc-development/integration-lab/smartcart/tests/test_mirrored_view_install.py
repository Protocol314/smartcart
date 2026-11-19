"""Pi 기준 화면에만 좌우반전 업데이트가 적용되는지 확인한다."""
import ast
import base64
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[2]


def previous_index():
    path = ROOT / 'build/releases/smartcart-controller-layout-compatible-20261117-install.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    encoded = next(ast.literal_eval(node.value) for node in tree.body
                   if isinstance(node, ast.Assign) and len(node.targets) == 1
                   and getattr(node.targets[0], 'id', '') == 'PAYLOAD')
    return json.loads(zlib.decompress(base64.b64decode(encoded)))['web/index.html']['text']


class MirrorInstallTest(unittest.TestCase):
    def test_previous_versions_backup_reapply_and_local_change_rejection(self):
        with tempfile.TemporaryDirectory(dir=ROOT / 'build') as temp:
            app = Path(temp) / 'smartcart'
            (app / 'web/js').mkdir(parents=True)
            (app / 'cart.py').write_text('', encoding='utf-8')
            for name in ('core.js', 'radar.js'):
                data = (ROOT / 'SmartCart_ManualDrive_180/smartcart/web/js' / name).read_bytes()
                (app / 'web/js' / name).write_bytes(data)
            (app / 'web/index.html').write_text(previous_index(), encoding='utf-8', newline='')
            installer = ROOT / 'build/releases/smartcart-mirrored-view-20261117-install.py'
            command = [sys.executable, '-X', 'utf8', str(installer), '--root', temp, '--apply']
            for _ in range(2):
                result = subprocess.run(command, text=True, encoding='utf-8', capture_output=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertTrue(list(Path(temp).glob('smartcart-mirrored-view-backup-*')))
            modified = app / 'web/js/core.js'
            modified.write_text('현지 수정', encoding='utf-8')
            result = subprocess.run(command, text=True, encoding='utf-8', capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(modified.read_text(encoding='utf-8'), '현지 수정')


if __name__ == '__main__':
    unittest.main()
