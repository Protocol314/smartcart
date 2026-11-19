"""실제 SSH·서비스 변경 없이 릴리스 경계와 셸 실행기를 검증한다."""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import os
import shutil
import stat
import subprocess
import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]


def load_module(name: str, relative: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


installer = load_module("test_sensing_installer", "deploy/install-sensing.py")
packager = load_module("test_sensing_packager", "scripts/package-sensing.py")
deployer = load_module("test_sensing_deployer", "scripts/deploy-sensing.py")


class ReleaseBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=ROOT / "tests")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def archive(self, files=None, extra=(), manifest_override=None):
        files = files or {"sensing.py": b"print('test')\n", "README-SENSING.md": b"test\n"}
        manifest = manifest_override or "".join(f"{hashlib.sha256(data).hexdigest()}  {name}\n" for name, data in files.items())
        path = self.root / "release.zip"
        with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as archive:
            for name, data in files.items():
                archive.writestr(installer.PREFIX + name, data)
            archive.writestr(installer.PREFIX + installer.MANIFEST, manifest)
            for name, data, mode in extra:
                info = zipfile.ZipInfo(installer.PREFIX + name)
                info.create_system = 3
                info.external_attr = mode << 16
                archive.writestr(info, data)
        return path, hashlib.sha256(path.read_bytes()).hexdigest()

    def test_valid_release_hashes(self):
        path, digest = self.archive()
        self.assertEqual(set(installer.validate_archive(path, digest)), {"sensing.py", "README-SENSING.md"})

    def test_rejects_archive_digest_mismatch(self):
        path, _ = self.archive()
        with self.assertRaises(ValueError):
            installer.validate_archive(path, "0" * 64)

    def test_rejects_content_digest_mismatch(self):
        path, digest = self.archive(manifest_override="0" * 64 + "  sensing.py\n" + "0" * 64 + "  README-SENSING.md\n")
        with self.assertRaises(ValueError):
            installer.validate_archive(path, digest)

    def test_rejects_paths_secrets_and_logs(self):
        for name in ("../escape", "web/../../escape.js", "web//duplicate.js", "web/./duplicate.js", "web/.env.js", "deploy/sensing.env", ".git/config", "recordings/scan.json", "history.txt"):
            with self.subTest(name=name):
                path, digest = self.archive(extra=[(name, b"x", stat.S_IFREG | 0o600)])
                with self.assertRaises(ValueError):
                    installer.validate_archive(path, digest)

    def test_rejects_symlinks_and_special_files(self):
        for mode in (stat.S_IFLNK | 0o777, stat.S_IFIFO | 0o600):
            path, digest = self.archive(extra=[("web/evil.js", b"/etc/passwd", mode)])
            with self.assertRaises(ValueError):
                installer.validate_archive(path, digest)

    def test_rejects_oversized_file_before_reading(self):
        path, digest = self.archive(files={"sensing.py": b"x" * 256, "README-SENSING.md": b"x"})
        with mock.patch.object(installer, "MAX_FILE", 200), self.assertRaises(ValueError):
            installer.validate_archive(path, digest)

    def test_rejects_excessive_entry_count(self):
        path, digest = self.archive()
        with mock.patch.object(installer, "MAX_FILES", 2), self.assertRaises(ValueError):
            installer.validate_archive(path, digest)

    def test_package_excludes_non_runtime_files_and_is_reproducible(self):
        project = self.root / "source"
        required = installer.FIXED | {name + "/__init__.py" for name in installer.PYTHON_PACKAGES} | {"web/index.html"}
        for name in required:
            path = project / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"test\n")
        shutil.copyfile(ROOT / "deploy/install-sensing.py", project / "deploy/install-sensing.py")
        for name in (".env", ".git/config", "recordings/data.json", "web/.env.js", "MANIFEST.sha256",
                     "smartcart_uwb/lib/smartcart_uwb_parser.dll", "smartcart_uwb/lib/libsmartcart_uwb_parser.so",
                     "config/my-devices.json", "config/target_tracking.synthetic.json"):
            path = project / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"exclude\n")
        first, second = self.root / "one.zip", self.root / "two.zip"
        digest, _ = packager.build_release(project, first)
        second_digest, _ = packager.build_release(project, second)
        self.assertEqual(digest, second_digest)
        names = installer.validate_archive(first, digest)
        self.assertNotIn("MANIFEST.sha256", names)
        self.assertNotIn(".env", names)
        self.assertIn("docs/CALIBRATION.md", names)
        self.assertTrue(installer.EXAMPLE_CONFIGS <= names.keys())
        self.assertTrue({name + "/__init__.py" for name in installer.PYTHON_PACKAGES} <= names.keys())
        self.assertFalse(any(name.endswith((".dll", ".so", ".dylib")) for name in names))
        self.assertNotIn("config/my-devices.json", names)
        self.assertNotIn("config/target_tracking.synthetic.json", names)
        bootstrap = first.with_name(first.stem + "-install.py")
        self.assertEqual(bootstrap.read_bytes(), (project / "deploy/install-sensing.py").read_bytes())
        checksums = first.with_name(first.name + ".sha256").read_text(encoding="ascii").splitlines()
        self.assertEqual(checksums, [digest + "  one.zip", hashlib.sha256(bootstrap.read_bytes()).hexdigest() + "  one-install.py"])
        with self.assertRaises((ValueError, FileExistsError)):
            packager.build_release(project, first)

    def test_dry_run_never_connects(self):
        path, digest = self.archive()
        with mock.patch.object(deployer.subprocess, "run", side_effect=AssertionError("network called")):
            deployer.deploy("capstone@192.0.2.1", 22, path, digest, dry_run=True)

    def test_install_new_folder_and_preserve_existing_projects(self):
        path, digest = self.archive()
        home = self.root / "home"
        home.mkdir()
        for directory in ("SmartCart-Radar", "smartcart_ws"):
            (home / directory).mkdir()
            (home / directory / "keep.txt").write_text("keep", encoding="utf-8")
        with mock.patch.object(installer.sys, "platform", "linux"), mock.patch.object(installer.os, "geteuid", return_value=1000, create=True), mock.patch.object(installer.Path, "home", return_value=home):
            target = installer.install(path, digest)
            self.assertEqual((target / "sensing.py").read_bytes(), b"print('test')\n")
            with self.assertRaises(ValueError):
                installer.install(path, digest)
        for directory in ("SmartCart-Radar", "smartcart_ws"):
            self.assertEqual((home / directory / "keep.txt").read_text(encoding="utf-8"), "keep")

    def test_failed_integrity_never_creates_install_target(self):
        path, _ = self.archive()
        home = self.root / "home"
        home.mkdir()
        with mock.patch.object(installer.sys, "platform", "linux"), mock.patch.object(installer.os, "geteuid", return_value=1000, create=True), mock.patch.object(installer.Path, "home", return_value=home):
            with self.assertRaises(ValueError):
                installer.install(path, "0" * 64)
        self.assertFalse((home / "SmartCart-Sensing").exists())

    def test_rejects_shell_injection_target_and_invalid_ports(self):
        for target in ("-oProxyCommand=evil", "user@host;touch pwn", "user@$(evil)", "user@host\nanything", "user@host:/tmp", "root@pi"):
            with self.subTest(target=target), self.assertRaises(argparse.ArgumentTypeError):
                deployer.target_value(target)
        for port in ("0", "65536", "22;id", "not-a-port"):
            with self.subTest(port=port), self.assertRaises(argparse.ArgumentTypeError):
                deployer.port_value(port)


class ShellLauncherTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        git_bash = Path("C:/Program Files/Git/usr/bin/bash.exe")
        cls.bash = str(git_bash) if os.name == "nt" and git_bash.is_file() else shutil.which("bash")
        if not cls.bash:
            raise unittest.SkipTest("Bash 실행 파일이 없는 환경입니다.")

    def test_bash_syntax(self):
        for name in ("start-sensing.sh", "doctor-sensing.sh"):
            result = subprocess.run([self.bash, "--noprofile", "--norc", "-n", (ROOT / "scripts" / name).as_posix()], capture_output=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stderr.decode("utf-8", errors="replace"))

    def test_launch_rejects_extra_arguments_before_loading_ros(self):
        result = subprocess.run([self.bash, "--noprofile", "--norc", (ROOT / "scripts/start-sensing.sh").as_posix(), "--host", "0.0.0.0"], capture_output=True, timeout=15)
        self.assertEqual(result.returncode, 2)
        self.assertIn("CLI", result.stderr.decode("utf-8", errors="replace"))

    def test_launch_rejects_unlisted_ros_version(self):
        env = dict(os.environ, SENSING_ROS_DISTRO="humble;exit 0")
        result = subprocess.run([self.bash, "--noprofile", "--norc", (ROOT / "scripts/start-sensing.sh").as_posix()], env=env, capture_output=True, timeout=15)
        self.assertEqual(result.returncode, 2)


if __name__ == "__main__":
    unittest.main()
