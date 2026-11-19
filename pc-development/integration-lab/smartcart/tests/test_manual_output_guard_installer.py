"""설치기는 원본 파일 확인과 모의 시험이 끝난 뒤에만 복사본을 갱신한다."""
import ast
import base64
import hashlib
import importlib.util
import io
import shutil
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]


class InstallerTest(unittest.TestCase):
    def test_verified_install_and_backup(self):
        old_installer = ROOT / 'build/releases/smartcart-manual-pivot-gpio-update-20261117.py'
        for node in ast.parse(old_installer.read_text(encoding='utf-8')).body:
            if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) and node.targets[0].id == 'PAYLOAD':
                archive = zipfile.ZipFile(io.BytesIO(base64.b64decode(ast.literal_eval(node.value))))
                old_js = archive.read('web/js/manual_controls.js')
                break
        else:
            self.fail('이전 표시 파일을 찾을 수 없습니다')
        installer = ROOT / 'build/releases/smartcart-manual-output-guard-20261118-install.py'
        spec = importlib.util.spec_from_file_location('output_guard_installer', installer)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            temp = Path(directory)
            project = temp / 'smartcart'
            shutil.copytree(ROOT / 'SmartCart_ManualDrive_180/smartcart', project)
            (project / 'web/js/manual_controls.js').write_bytes(old_js)
            motor = project / 'smartcart_motor/manual_remote.py'
            data = motor.read_bytes().replace(b'        self.applied_sequence = -1\r\n', b'')
            data = data.replace(b'                    self.applied_sequence = self.sequence\r\n', b'')
            data = data.replace(b', "applied_seq": self.applied_sequence', b'')
            motor.write_bytes(data)
            for name, expected in module.EXPECTED.items():
                self.assertEqual(module.digest(project / name), expected)
            module.ROOT = project
            with patch.object(Path, 'home', return_value=temp), patch.object(sys, 'argv', ['installer', '--apply']):
                module.main()
            expected = ROOT / 'SmartCart_ManualDrive_180/smartcart/web/js/manual_controls.js'
            self.assertEqual(module.digest(project / 'web/js/manual_controls.js'), hashlib.sha256(expected.read_bytes()).hexdigest())
            self.assertEqual(len(list(temp.glob('smartcart-manual-backup-*'))), 1)


if __name__ == '__main__':
    unittest.main()
