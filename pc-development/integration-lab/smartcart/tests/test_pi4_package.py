"""임시 Pi 4 ZIP의 경계·대상·해시를 검사한다. ARM 코드 실행·실물 설치 검사가 아니다."""
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import stat
import struct
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


installer = load("pi4_test_installer", ROOT / "deploy/install-pi4.py")
packager = load("pi4_test_packager", ROOT / "scripts/package-pi4.py")


def synthetic_elf(machine=183):
    """형식 검사만 위한 비실행 ELF 표본이다. 실제 ARM 라이브러리로 취급하지 않는다."""
    data = bytearray(256)
    data[:7] = b"\x7fELF\x02\x01\x01"
    struct.pack_into("<HHIQQQIHHHHHH", data, 16, 3, machine, 1, 0, 64, 0, 0, 64, 56, 2, 0, 0, 0)
    struct.pack_into("<IIQQQQQQ", data, 64, 1, 5, 0, 0, 0, 256, 256, 4096)
    struct.pack_into("<IIQQQQQQ", data, 120, 2, 6, 176, 176, 176, 16, 16, 8)
    return bytes(data)


class Pi4PackageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="pi4-package-", dir=ROOT / "tests")
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.repository = self.directory / "source"
        self.native = self.directory / "arm64-format-only.so"
        self.native.write_bytes(synthetic_elf())
        for name in installer.REQUIRED - {installer.NATIVE, installer.TARGET_FILE}:
            path = self.repository / ("smartcart/README-PI4.md" if name == "README-PI4.md" else name)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"test fixture\n")
        shutil.copyfile(ROOT / "deploy/install-pi4.py", self.repository / "smartcart/deploy/install-pi4.py")
        for name in ("dual_uwb.d5", "uwb_estimation", "target_tracking"):
            shutil.copyfile(ROOT / "config" / (name + ".example.json"),
                            self.repository / "smartcart/config" / (name + ".example.json"))
        shutil.copyfile(ROOT / "config/field_acceptance.example.json",
                        self.repository / "smartcart/config/field_acceptance.example.json")
        for name in installer.SYNTHETIC_CONFIGS:
            shutil.copyfile(ROOT / Path(name).relative_to("smartcart"), self.repository / name)
        for name in (installer.FIELD_SAMPLE, installer.TRACKING_SAMPLE):
            (self.repository / name).write_text(json.dumps({"schema": "smartcart.target_capture.v1",
                "event": "tick", "at_ns": 0, "source": "synthetic"}) + "\n", encoding="utf-8")
        (self.repository / installer.TRACKING_TRUTH).write_text(json.dumps({
            "schema": "smartcart.target_tracking_truth.v1", "source": "synthetic", "frame_id": "base_link",
            "tag_height_base_link_m": .9, "notes": "합성 패키지 검증 자료", "checkpoints": [{
                "label": "initial", "event_index": 0, "expected_valid": False, "same_track_as": None,
                "different_track_from": None, "candidate_min": None, "track_id_null": True,
                "expected_position_xy_m": None, "max_target_error_m": None}]}), encoding="utf-8")

    def build(self, name="bundle.zip"):
        path = self.directory / name
        digest, count = packager.build_release(self.repository, path, self.native, "0.3.0")
        return path, digest, count

    def files(self, path):
        with zipfile.ZipFile(path) as archive:
            return {item.filename[len(installer.PREFIX):]: archive.read(item) for item in archive.infolist()
                    if item.filename != installer.PREFIX + installer.MANIFEST}

    def archive(self, files, *, extra=(), prefix=None, manifest=None):
        path = self.directory / "mutated.zip"
        if manifest is None:
            manifest = "".join(hashlib.sha256(data).hexdigest() + "  " + name + "\n" for name, data in sorted(files.items())).encode("ascii")
        with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as archive:
            for name, data in files.items():
                archive.writestr((installer.PREFIX if prefix is None else prefix) + name, data)
            archive.writestr(installer.PREFIX + installer.MANIFEST, manifest)
            for name, data, mode in extra:
                item = zipfile.ZipInfo(installer.PREFIX + name)
                item.create_system = 3
                item.external_attr = mode << 16
                archive.writestr(item, data)
        return path, hashlib.sha256(path.read_bytes()).hexdigest()

    def test_reproducible_archive_has_field_interface_sources_arm_metadata_and_bootstrap(self):
        first, digest, count = self.build()
        second, second_digest, _ = self.build("second.zip")
        self.assertEqual(first.read_bytes(), second.read_bytes())
        self.assertEqual(digest, second_digest)
        hashes = installer.validate_archive(first, digest)
        self.assertEqual(len(hashes), count)
        self.assertTrue(installer.REQUIRED <= hashes.keys())
        files = self.files(first)
        metadata = installer.parse_json(files[installer.TARGET_FILE])
        self.assertEqual(metadata["release_version"], "0.3.0")
        self.assertEqual(metadata["native"]["machine"], 183)
        self.assertEqual(metadata["source_sha256"], {name: hashes[name] for name in installer.NATIVE_SOURCES})
        self.assertTrue(installer.CONFIGS <= hashes.keys())
        bootstrap = first.with_name(first.stem + "-install.py")
        self.assertEqual(bootstrap.read_bytes(), files["smartcart/deploy/install-pi4.py"])
        self.assertEqual(len(first.with_name(first.name + ".sha256").read_text(encoding="ascii").splitlines()), 2)
        with self.assertRaises(ValueError):
            self.build()

    def test_only_named_synthetic_sample_and_examples_are_allowed(self):
        forbidden = (".env", "keys/pi.pem", "smartcart/config/my-devices.json", "smartcart/recordings/a.jsonl",
                     "smartcart/local-config/run.json", "smartcart/samples/recorded.jsonl",
                     "smartcart/smartcart_uwb/lib/smartcart_uwb_parser.dll", "smartcart/tests/secret.py",
                     "smartcart/smartcart_field/__pycache__/app.pyc", "../escape", "smartcart/web/../../escape.js")
        for name in forbidden:
            with self.subTest(name=name):
                self.assertFalse(installer.allowed_path(name))
        for name in forbidden[:8]:
            path = self.repository / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("excluded fixture", encoding="utf-8")
        path, digest, _ = self.build()
        hashes = installer.validate_archive(path, digest)
        self.assertFalse(set(forbidden) & hashes.keys())
        self.assertEqual({name for name in hashes if name.endswith(".jsonl")},
                         {installer.FIELD_SAMPLE, installer.TRACKING_SAMPLE})
        self.assertTrue(installer.OFFLINE_FILES <= hashes.keys())
        self.assertIn("smartcart/README-OFFLINE.md", hashes)

    def test_offline_release_requires_runner_and_assets_but_legacy_archive_still_verifies(self):
        path, _, _ = self.build()
        files = self.files(path)
        for name in ("smartcart/cart.py", "smartcart/smartcart_runner/app.py", installer.TRACKING_SAMPLE):
            missing = dict(files)
            del missing[name]
            invalid, digest = self.archive(missing)
            with self.subTest(missing=name), self.assertRaises(ValueError):
                installer.validate_archive(invalid, digest)
        legacy = {name: value for name, value in files.items() if name not in installer.OFFLINE_FILES
                  and not name.startswith("smartcart/smartcart_runner/")}
        metadata = json.loads(legacy[installer.TARGET_FILE])
        metadata["release_version"] = "0.2.0"
        legacy[installer.TARGET_FILE] = json.dumps(metadata).encode("utf-8")
        old_path, old_digest = self.archive(legacy)
        self.assertEqual(set(installer.validate_archive(old_path, old_digest)), set(legacy))

    def test_tracking_synthetic_inputs_cannot_package_recordings_or_verified_assumptions(self):
        path = self.repository / installer.TRACKING_SAMPLE
        original = path.read_bytes()
        for content in (original.replace(b"synthetic", b"recorded"), b""):
            path.write_bytes(content)
            with self.subTest(content=content), self.assertRaises(ValueError):
                self.build()
        path.write_bytes(original)
        for name in installer.SYNTHETIC_CONFIGS:
            path = self.repository / name
            value = json.loads(path.read_text(encoding="utf-8"))
            value["assumptions_verified"] = True
            path.write_text(json.dumps(value), encoding="utf-8")
            with self.subTest(config=name), self.assertRaises(ValueError):
                self.build()
            value["assumptions_verified"] = False
            path.write_text(json.dumps(value), encoding="utf-8")

    def test_truth_cannot_relabel_recordings_or_reference_events_outside_bundled_sample(self):
        path = self.repository / installer.TRACKING_TRUTH
        original = json.loads(path.read_text(encoding="utf-8"))
        mutations = []
        for key, value in (("source", "recorded"), ("frame_id", "private_frame"), ("tag_height_base_link_m", 2)):
            truth = deepcopy(original)
            truth[key] = value
            mutations.append(truth)
        for key, value in (("event_index", 999), ("same_track_as", "missing"),
                           ("expected_position_xy_m", [100, 200]), ("extra_local_notes", "현장 메모")):
            truth = deepcopy(original)
            truth["checkpoints"][0][key] = value
            mutations.append(truth)
        for value in mutations:
            path.write_text(json.dumps(value), encoding="utf-8")
            with self.subTest(truth=value), self.assertRaises(ValueError):
                self.build()

    def test_local_ports_verified_assumptions_and_recorded_sample_are_not_packaged(self):
        paths = [self.repository / "smartcart/config/dual_uwb.d5.example.json",
                 self.repository / "smartcart/config/uwb_estimation.example.json",
                 self.repository / installer.FIELD_SAMPLE]
        originals = [path.read_bytes() for path in paths]
        mounting = json.loads(originals[0])
        mounting["sensors"]["uwb_left"]["port"] = "/dev/ttyACM0"
        uwb = json.loads(originals[1])
        uwb["assumptions_verified"] = True
        mutations = [json.dumps(mounting).encode(), json.dumps(uwb).encode(), originals[2].replace(b"synthetic", b"recorded")]
        for index, (path, content) in enumerate(zip(paths, mutations)):
            with self.subTest(index=index):
                path.write_bytes(content)
                with self.assertRaises(ValueError):
                    self.build("bad-example-" + str(index) + ".zip")
                path.write_bytes(originals[index])

    def test_wrong_platform_and_truncated_libraries_are_rejected(self):
        for value in (b"MZ" + bytes(300), synthetic_elf(62), synthetic_elf()[:100], b"\x7fELF\x01" + bytes(300)):
            with self.subTest(prefix=value[:20]):
                self.native.write_bytes(value)
                with self.assertRaises(ValueError):
                    self.build()

    def test_field_example_cannot_leak_lidar_binding_or_mounting_evidence(self):
        path = self.repository / "smartcart/config/field_acceptance.example.json"
        original = json.loads(path.read_text(encoding="utf-8"))
        cases = []
        for name, value in (("port", "/dev/serial/by-id/usb-PRIVATE"),
                            ("expected_device_identity", "vid=1234;pid=5678;serial=PRIVATE"),
                            ("device_identity_verified", True)):
            field = deepcopy(original)
            field["lidar_device"][name] = value
            cases.append(field)
        for name in ("origin_verified", "axes_verified", "range_model_verified", "tag_height_verified"):
            field = deepcopy(original)
            field[name] = True
            cases.append(field)
        for role in ("uwb_left", "uwb_right", "lidar_center"):
            for name, value in (("translation_verified", True), ("yaw_verified", True),
                                ("height_verified", True), ("level_verified", True),
                                ("reference_point", "scan_origin" if role == "lidar_center" else "rf_phase_center"),
                                ("evidence_note", "개인 현장 검수 기록")):
                field = deepcopy(original)
                field["mounting_evidence"][role][name] = value
                cases.append(field)
        for name, value in (("origin_verified", 0), ("extra_local_notes", "개인 메모")):
            field = deepcopy(original)
            field[name] = value
            cases.append(field)
        # ZIP을 쓰기 전 거부되는지 공개 빌드 경로로 모든 누출 변형을 검사한다.
        for index, field in enumerate(cases):
            path.write_text(json.dumps(field, ensure_ascii=False), encoding="utf-8")
            output = self.directory / ("field-leak-" + str(index) + ".zip")
            with self.subTest(case=index), self.assertRaises(ValueError):
                packager.build_release(self.repository, output, self.native, "0.2.0")
            self.assertFalse(output.exists())

    def test_mounting_lidar_verified_example_is_not_packaged(self):
        path = self.repository / "smartcart/config/dual_uwb.d5.example.json"
        value = json.loads(path.read_text(encoding="utf-8"))
        value["lidar"]["mounting_verified"] = True
        path.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaises(ValueError):
            self.build()

    def test_metadata_exact_types_target_and_hash_provenance_are_enforced(self):
        path, _, _ = self.build()
        original_files = self.files(path)
        original_meta = json.loads(original_files[installer.TARGET_FILE])
        mutations = []
        for name, value in (("architecture", "x86_64"), ("version", "24.04"), ("ros_distro", "jazzy")):
            metadata = deepcopy(original_meta)
            metadata["target"][name] = value
            mutations.append(metadata)
        for key in ("machine", "abi_version"):
            metadata = deepcopy(original_meta)
            metadata["native"][key] = float(metadata["native"][key])
            mutations.append(metadata)
        metadata = deepcopy(original_meta)
        metadata["safety"]["drive_authorized"] = 0
        mutations.append(metadata)
        metadata = deepcopy(original_meta)
        metadata["source_sha256"][next(iter(installer.NATIVE_SOURCES))] = "0" * 64
        mutations.append(metadata)
        for metadata in mutations:
            files = dict(original_files)
            files[installer.TARGET_FILE] = json.dumps(metadata).encode("utf-8")
            invalid, digest = self.archive(files)
            with self.subTest(metadata=metadata), self.assertRaises(ValueError):
                installer.validate_archive(invalid, digest)

    def test_invalid_digest_missing_file_content_corruption_and_limits_are_rejected(self):
        path, digest, _ = self.build()
        with self.assertRaises(ValueError):
            installer.validate_archive(path, "0" * 64)
        files = self.files(path)
        missing = dict(files)
        del missing["smartcart/field_audit.py"]
        invalid, invalid_digest = self.archive(missing)
        with self.assertRaises(ValueError):
            installer.validate_archive(invalid, invalid_digest)
        wrong = dict(files)
        wrong["README-PI4.md"] += b"changed"
        manifest = "".join(hashlib.sha256(data).hexdigest() + "  " + name + "\n" for name, data in files.items()).encode("ascii")
        invalid, invalid_digest = self.archive(wrong, manifest=manifest)
        with self.assertRaises(ValueError):
            installer.validate_archive(invalid, invalid_digest)
        for name, maximum in (("MAX_FILES", 3), ("MAX_EXPANDED", 100), ("MAX_ARCHIVE", 100), ("MAX_FILE", 100)):
            with self.subTest(limit=name), patch.object(installer, name, maximum), self.assertRaises(ValueError):
                installer.validate_archive(path, digest)

    def test_zip_path_duplicate_symlink_and_special_file_are_rejected(self):
        path, _, _ = self.build()
        files = self.files(path)
        for name, mode in (("smartcart/web/link.js", stat.S_IFLNK | 0o777),
                           ("smartcart/web/pipe.js", stat.S_IFIFO | 0o600),
                           ("../escape", stat.S_IFREG | 0o600),
                           ("smartcart/field_audit.py", stat.S_IFREG | 0o600)):
            invalid, digest = self.archive(files, extra=[(name, b"bad", mode)])
            with self.subTest(name=name), self.assertRaises(ValueError):
                installer.validate_archive(invalid, digest)

    def test_install_only_new_home_target_and_preserve_legacy_projects(self):
        path, digest, _ = self.build()
        home = self.directory / "home"
        (home / "SmartCart-Sensing").mkdir(parents=True)
        (home / "SmartCart-Sensing/keep.txt").write_text("preserved", encoding="utf-8")
        with patch.object(installer, "check_target_environment"), patch.object(installer.Path, "home", return_value=home):
            target = installer.install(path, digest)
            self.assertEqual(target, home / "SmartCart-D5-v2")
            self.assertEqual((target / installer.NATIVE).read_bytes(), synthetic_elf())
            with self.assertRaises(ValueError):
                installer.install(path, digest)
        self.assertEqual((home / "SmartCart-Sensing/keep.txt").read_text(encoding="utf-8"), "preserved")
        self.assertFalse((target / "workspace").exists())

    def test_environment_gate_requires_regular_arm64_ubuntu_22_user(self):
        with patch.object(installer.sys, "platform", "linux"), patch.object(installer.platform, "machine", return_value="aarch64"), \
                patch.object(installer.os, "geteuid", return_value=1000, create=True), \
                patch.object(installer.Path, "read_text", return_value='ID=ubuntu\nVERSION_ID="22.04"\n'):
            installer.check_target_environment()
            for target, attr, value in ((installer.sys, "platform", "win32"), (installer.platform, "machine", lambda: "x86_64"),
                                        (installer.os, "geteuid", lambda: 0)):
                with self.subTest(attr=attr), patch.object(target, attr, value), self.assertRaises(ValueError):
                    installer.check_target_environment()
            with patch.object(installer.Path, "read_text", return_value='ID=debian\nVERSION_ID="12"\n'), self.assertRaises(ValueError):
                installer.check_target_environment()

    def test_build_script_bash_syntax_and_argument_rejection_without_build(self):
        bash = Path("C:/Program Files/Git/usr/bin/bash.exe")
        executable = str(bash) if bash.is_file() else shutil.which("bash")
        if not executable:
            self.skipTest("Bash가 없어 빌드 스크립트의 구문 검사를 실행하지 못합니다")
        script = (ROOT / "scripts/build-pi4.sh").as_posix()
        syntax = subprocess.run([executable, "--noprofile", "--norc", "-n", script], capture_output=True, timeout=10)
        self.assertEqual(syntax.returncode, 0, syntax.stderr.decode("utf-8", errors="replace"))
        rejected = subprocess.run([executable, "--noprofile", "--norc", script, "--unapproved"], capture_output=True, timeout=10)
        self.assertEqual(rejected.returncode, 2)


if __name__ == "__main__":
    unittest.main()
