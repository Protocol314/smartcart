"""기존 배포본과 이전 패치를 복원한 임시 폴더에서 설치기를 시험한다."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[2]
RELEASE = ROOT / "build" / "releases"


class InstallerTests(unittest.TestCase):
    def test_baseline_update_and_idempotence(self):
        name = "smartcart-single-left-pi-identity-20261117-install.py"
        with tempfile.TemporaryDirectory(dir=ROOT / "build") as directory:
            with zipfile.ZipFile(RELEASE / "smartcart-d5-v2-pi4-arm64-v0.3.1.zip") as archive:
                archive.extractall(directory)
            root = Path(directory) / "SmartCart-D5-v2"
            # 미확인 기준 파일은 덮어쓰지 않는다.
            changed = root / "smartcart/web/js/app.js"
            changed.write_bytes(b"local change")
            failed = subprocess.run([sys.executable, "-X", "utf8", str(RELEASE / name), "--root", str(root), "--apply"],
                                    capture_output=True, text=True, encoding="utf-8")
            self.assertNotEqual(failed.returncode, 0)
            self.assertEqual(changed.read_bytes(), b"local change")
            # 모든 파일이 이미 새 버전인 경우 재적용은 아무것도 바꾸지 않는다.
            import importlib.util
            spec = importlib.util.spec_from_file_location("single_builder", ROOT / "smartcart/scripts/build-single-left-update.py")
            builder = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(builder)
            for relative in builder.FILES:
                target = root / "smartcart" / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes((ROOT / "smartcart" / relative).read_bytes())
            again = subprocess.run([sys.executable, "-X", "utf8", str(RELEASE / name), "--root", str(root), "--apply"],
                                   capture_output=True, text=True, encoding="utf-8")
            self.assertEqual(again.returncode, 0, again.stdout + again.stderr)
            self.assertIn("이미 같은 업데이트", again.stdout)
            self.assertTrue((root / "smartcart/scripts/start-single-left.sh").is_file())
