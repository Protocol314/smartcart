"""구동 감독기 배포 ZIP의 구성·재현성·덮어쓰기 거부를 검사한다."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import zipfile

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'package-drive-supervisor.py'
spec = importlib.util.spec_from_file_location('package_drive_supervisor', SCRIPT)
package = importlib.util.module_from_spec(spec)
spec.loader.exec_module(package)


class DriveSupervisorPackageTests(unittest.TestCase):
    def test_contents_are_reproducible_and_exclude_local_state(self):
        with tempfile.TemporaryDirectory() as directory:
            first, second = Path(directory) / 'a.zip', Path(directory) / 'b.zip'
            self.assertEqual(package.build(first), package.build(second))
            with self.assertRaises(FileExistsError):
                package.build(first)
            names = zipfile.ZipFile(first).namelist()
        prefix = package.NAME + '/'
        for required in ('smartcart_motor/drive_app.py', 'smartcart_motor/supervisor.py',
                         'smartcart_motor/safety_provider.py', 'smartcart_motor/drive_server.py',
                         'smartcart_motor/drive_inputs.py', 'web/index.html', 'web/js/drive_link.js',
                         'web/css/drive_link.css', 'scripts/start-drive-supervisor.sh', 'README.md'):
            self.assertIn(prefix + required, names)
        self.assertTrue(all(name.startswith(prefix) for name in names))
        for forbidden in ('__pycache__', 'local-config', '.pyc', 'tests/', '.key', '.crt'):
            self.assertFalse(any(forbidden in name for name in names), forbidden)


if __name__ == '__main__':
    unittest.main()
