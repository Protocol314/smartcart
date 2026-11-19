"""임시 런타임 ZIP의 추적 의존성과 기존 데모 시작을 장치·소켓 없이 확인한다."""

from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from test_deploy_sensing import installer, packager, ROOT


class TrackingReleaseTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="release-tracking-", dir=ROOT / "tests")
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)

    def test_new_runtime_files_are_allowed_without_native_or_local_configuration(self):
        for name in ("smartcart_tracking/view.py", "smartcart_tracking/ros_runtime.py",
                     "smartcart_uwb/estimator.py", "launch/tracking.launch.py", "web/js/tracking.js",
                     *installer.EXAMPLE_CONFIGS):
            with self.subTest(name=name):
                self.assertTrue(installer.allowed_path(name))
        for name in ("smartcart_tracking/.env.py", "smartcart_tracking/__pycache__/view.pyc",
                     "smartcart_uwb/lib/smartcart_uwb_parser.dll", "smartcart_uwb/lib/libsmartcart_uwb_parser.so",
                     "smartcart_uwb/parser.dll", "config/my-devices.json", "config/target_tracking.synthetic.json",
                     "samples/target_tracking.synthetic.jsonl", "recordings/scan.jsonl"):
            with self.subTest(name=name):
                self.assertFalse(installer.allowed_path(name))

    def test_temporary_zip_installs_and_starts_legacy_demo_with_tracking_off(self):
        archive = self.directory / "temporary-runtime.zip"
        digest, count = packager.build_release(ROOT, archive)
        names = installer.validate_archive(archive, digest)
        expected = {"smartcart_tracking/view.py", "smartcart_tracking/tracker.py", "smartcart_tracking/ros_runtime.py",
                    "smartcart_uwb/config.py", "smartcart_uwb/estimator.py", "launch/tracking.launch.py",
                    "web/js/tracking.js", *installer.EXAMPLE_CONFIGS}
        self.assertTrue(expected <= names.keys())
        self.assertEqual(count, len(names))
        self.assertFalse(any(name.endswith((".dll", ".so", ".dylib", ".pyc", ".jsonl")) for name in names))
        home = self.directory / "home"
        home.mkdir()
        with mock.patch.object(installer.sys, "platform", "linux"), \
                mock.patch.object(installer.os, "geteuid", return_value=1000, create=True), \
                mock.patch.object(installer.Path, "home", return_value=home):
            target = installer.install(archive, digest)
        # 별도 Python의 검색 경로에 추출본만 추가해 작업 저장소가 누락 모듈을 가리지 않게 한다.
        code = r'''
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from smartcart_sensing import app
from smartcart_uwb.config import load_config, example_config_path
from smartcart_uwb.estimation_config import load_estimation_config
from smartcart_tracking.config import load_tracking_config

root = Path(sys.argv[1]).resolve()
assert Path(app.__file__).resolve().is_relative_to(root)
assert load_config(example_config_path()).sensors.keys() == {"uwb_left", "uwb_right"}
assert load_estimation_config(root / "config/uwb_estimation.example.json").range_model == "unknown"
load_tracking_config(root / "config/target_tracking.example.json")
assert (app.web_root() / "js/tracking.js").is_file()

class NoSocketServer:
    def __init__(self, address, store, web, **kwargs):
        self.server_port, self.store = address[1], store
        assert store.tracking.snapshot()["source_mode"] == "off"
    def serve_forever(self, **kwargs):
        self.store.stop.set()
    def server_close(self):
        pass

app.SensingHTTPServer = NoSocketServer
assert app.main(["serve", "--mode", "demo"]) == 0
print("TEMPORARY_RELEASE_DEMO_OK")
'''
        result = subprocess.run([sys.executable, "-I", "-X", "utf8", "-c", code, str(target)],
                                cwd=target, capture_output=True, text=True, encoding="utf-8", timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("TEMPORARY_RELEASE_DEMO_OK", result.stdout)


if __name__ == "__main__":
    unittest.main()
