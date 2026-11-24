import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE))
import server
spec=importlib.util.spec_from_file_location('real_v1_installer',HERE/'install.py')
installer=importlib.util.module_from_spec(spec)
spec.loader.exec_module(installer)

class InstallerTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.home=Path(self.temporary.name)
        self.root=self.home/'smartcart';self.root.mkdir()
        self.package=self.home/'package'
        shutil.copytree(HERE,self.package,ignore=shutil.ignore_patterns('__pycache__','*.pyc','SHA256.json'))
        manifest={p.relative_to(self.package).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in self.package.rglob('*') if p.is_file()}
        (self.package/'SHA256.json').write_text(json.dumps(manifest))
        for name in ('SmartCart_ManualDrive_180/smartcart/drive.py',
                     'SmartCart_ManualDrive_180/smartcart/smartcart_motor/manual_remote.py',
                     'smartcart-virtual-4digit.DlQ3Pf7U/virtual_lan_server.py'):
            p=self.root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('# unchanged dependency\n')
        for name in ('real.sh','virtual.sh','sensors.sh','cert.sh'):
            (self.root/name).write_text('previous '+name)
        base=SimpleNamespace(**{name:object() for name in ('ManualDrive','MockBackend','RPiBackend','make_app')})
        for item in (patch.object(installer,'__file__',str(self.package/'install.py')),
                     patch.object(installer.sys,'platform','linux'),
                     patch.object(Path,'home',return_value=self.home),
                     patch.object(installer.socket,'socket'),
                     patch.object(server,'load_base',return_value=base),
                     patch.object(server,'load_login',return_value={'make_guard':object()})):
            item.start();self.addCleanup(item.stop)

    def test_preserves_dependencies_and_archives_launchers(self):
        installer.main()
        self.assertTrue((self.root/'releases/real-uwb-v1/server.py').is_file())
        self.assertIn('real-uwb-v1/server.py',(self.root/'virtual.sh').read_text())
        backup=next((self.root/'archive').iterdir())
        self.assertEqual((backup/'virtual.sh').read_text(),'previous virtual.sh')
        self.assertEqual((self.root/'sensors.sh').read_text(),'previous sensors.sh')
        self.assertEqual((self.root/'cert.sh').read_text(),'previous cert.sh')
        with self.assertRaises(SystemExit):installer.main()

    def test_partial_activation_restores_original_launchers(self):
        original=Path.replace
        def fail_second(path,target):
            if path.name=='virtual.sh.real-v1-new' and target==self.root/'virtual.sh':
                raise OSError('simulated second launcher failure')
            return original(path,target)
        with patch.object(Path,'replace',new=fail_second):
            with self.assertRaises(OSError):installer.main()
        for name in ('real.sh','virtual.sh'):
            self.assertEqual((self.root/name).read_text(),'previous '+name)
        self.assertFalse((self.root/'releases/real-uwb-v1').exists())
        backup=next((self.root/'archive').iterdir())
        self.assertTrue((backup/'failed-release/server.py').exists())

    def test_tampered_payload_does_not_change_entry_points(self):
        (self.package/'runtime.py').write_text('# changed after signing manifest\n')
        with self.assertRaises(SystemExit):installer.main()
        self.assertEqual((self.root/'virtual.sh').read_text(),'previous virtual.sh')
        self.assertFalse((self.root/'archive').exists())

if __name__=='__main__':unittest.main()
