"""ROS·실물 없이 시작 스크립트의 환경 적용 순서와 실측 인수를 확인한다."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.name == 'posix' and shutil.which('bash'), 'Linux Bash 검사')
class LiveLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        base = Path(self.temporary.name)
        self.root = base / 'installation with spaces'
        self.cart = self.root / 'smartcart'
        scripts = self.cart / 'scripts'
        scripts.mkdir(parents=True)
        self.ros = base / 'ros-setup.bash'
        self.ros.write_text('export SC_ORDER=ros\n', encoding='utf-8')
        self.home = base / 'home'
        self.home.mkdir()
        for label, folder in (('old-driver', self.home / 'smartcart_ws'),
                              ('new-driver', self.home / 'smartcart_lidar_ws'),
                              ('integration', self.root / 'workspace')):
            target = folder / 'install/setup.bash'
            target.parent.mkdir(parents=True)
            target.write_text('export SC_ORDER="$SC_ORDER:' + label + '"\n', encoding='utf-8')
        (self.cart / 'local-config').mkdir()
        (self.cart / 'local-config/run.json').write_text('{}', encoding='utf-8')
        (self.cart / 'cart.py').write_text(
            'import json, os, sys\n'
            'print(json.dumps({"args":sys.argv[1:], "order":os.environ["SC_ORDER"], '
            '"domain":os.environ["ROS_DOMAIN_ID"], "local":os.environ["ROS_LOCALHOST_ONLY"]}))\n',
            encoding='utf-8')
        self.launcher = scripts / 'start-live.sh'
        source = (ROOT / 'scripts/start-live.sh').read_text(encoding='utf-8')
        self.launcher.write_text(source.replace('/opt/ros/humble/setup.bash', str(self.ros)), encoding='utf-8')
        self.environment = {**os.environ, 'HOME': str(self.home), 'ROS_DOMAIN_ID': '37'}

    def run_launcher(self, *args):
        return subprocess.run(['bash', str(self.launcher), *args], env=self.environment,
                              text=True, capture_output=True, timeout=5)

    def test_sources_environment_in_order_then_executes_live_with_fixed_config(self):
        result = self.run_launcher()
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout.splitlines()[-1])
        self.assertEqual(report['order'], 'ros:old-driver:new-driver:integration')
        self.assertEqual(report['domain'], '37')
        self.assertEqual(report['local'], '1')
        self.assertEqual(report['args'], ['--config', str(self.cart / 'local-config/run.json'),
                                          'start', '--mode', 'live'])

    def test_missing_config_never_executes_cart(self):
        (self.cart / 'local-config/run.json').unlink()
        result = self.run_launcher()
        self.assertEqual(result.returncode, 2)
        self.assertIn('실측 설정이 없습니다', result.stderr)

    def test_unexpected_argument_never_executes_cart(self):
        result = self.run_launcher('replay')
        self.assertEqual(result.returncode, 2)
        self.assertIn('인수를 받지 않습니다', result.stderr)


if __name__ == '__main__':
    unittest.main()
