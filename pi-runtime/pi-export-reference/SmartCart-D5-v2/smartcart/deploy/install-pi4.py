#!/usr/bin/env python3
"""ARM64 Pi 4 묶음을 검증하고 일반 계정의 새 전용 폴더에만 설치한다."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shlex
import stat
import struct
import sys
import tempfile
import zipfile

PREFIX = "SmartCart-D5-v2/"
MANIFEST = "RELEASE.sha256"
TARGET_FILE = "TARGET.json"
NATIVE = "smartcart/smartcart_uwb/lib/libsmartcart_uwb_parser.so"
FIELD_SAMPLE = "smartcart/samples/field_acceptance.synthetic.jsonl"
TRACKING_SAMPLE = "smartcart/samples/target_tracking.synthetic.jsonl"
TRACKING_TRUTH = "smartcart/samples/target_tracking.truth.json"
MAX_ARCHIVE, MAX_EXPANDED, MAX_FILE = 32 * 1024 * 1024, 64 * 1024 * 1024, 10 * 1024 * 1024
MAX_FILES, MAX_MANIFEST = 384, 128 * 1024
PYTHON_PACKAGES = {"smartcart_radar", "smartcart_sensing", "smartcart_uwb", "smartcart_tracking", "smartcart_field",
                   "smartcart_runner"}
CONFIGS = {"smartcart/config/" + name for name in ("dual_uwb.d5.example.json", "uwb_estimation.example.json",
                                                  "target_tracking.example.json", "field_acceptance.example.json")}
SYNTHETIC_CONFIGS = {"smartcart/config/" + name for name in ("uwb_estimation.synthetic.json",
                                                            "target_tracking.synthetic.json")}
NATIVE_SOURCES = {"components/c_key_core/bu04_pdoa.c", "components/c_key_core/include/bu04_pdoa.h",
                  "smartcart/native/uwb_parser_bridge.c", "smartcart/native/uwb_parser_bridge.h"}
LEGACY_FIXED = {"README-PI4.md", "smartcart/LICENSE", "smartcart/README-SENSING.md", "smartcart/sensing.py",
         "smartcart/uwb_collect.py", "smartcart/target_replay.py", "smartcart/field_audit.py",
         "smartcart/package.xml", "smartcart/setup.py", "smartcart/setup.cfg", "smartcart/resource/smartcart_sensing",
         "smartcart/scripts/build-pi4.sh", "smartcart/scripts/build_uwb_native.py", "smartcart/deploy/install-pi4.py",
         "ros2/smartcart_uwb_interfaces/CMakeLists.txt", "ros2/smartcart_uwb_interfaces/package.xml",
         "ros2/smartcart_uwb_interfaces/msg/UwbObservation.msg", FIELD_SAMPLE} | CONFIGS | NATIVE_SOURCES
OFFLINE_FILES = {"smartcart/README-OFFLINE.md", "smartcart/cart.py", "smartcart/scripts/verify-offline.py", "smartcart/scripts/test-offline.bat",
                 "smartcart/scripts/test-offline.sh", "smartcart/scripts/start-cart.bat", "smartcart/scripts/start-cart.sh",
                 TRACKING_SAMPLE, TRACKING_TRUTH} | SYNTHETIC_CONFIGS
FIXED = LEGACY_FIXED | OFFLINE_FILES
LEGACY_REQUIRED = LEGACY_FIXED | {"smartcart/" + name + "/__init__.py" for name in PYTHON_PACKAGES - {"smartcart_runner"}} | {
    "smartcart/web/index.html", "smartcart/launch/tracking.launch.py", NATIVE, TARGET_FILE}
REQUIRED = LEGACY_REQUIRED | OFFLINE_FILES | {"smartcart/smartcart_runner/__init__.py", "smartcart/smartcart_runner/app.py"}
TARGET = {"os": "linux", "architecture": "aarch64", "distribution": "ubuntu", "version": "22.04", "ros_distro": "humble"}
SAFETY = {"drive_authorized": False, "automatic_device_open": False, "automatic_service_start": False}


def required_for_version(version):
    """기존 0.2.0 묶음도 검증하되 0.3.0부터 오프라인 실행 자료를 필수로 요구한다."""
    return REQUIRED if tuple(map(int, version.split("."))) >= (0, 3, 0) else LEGACY_REQUIRED


def allowed_path(name):
    if type(name) is not str or len(name) > 220 or not re.fullmatch(r"[A-Za-z0-9_./+-]+", name):
        return False
    parts = PurePosixPath(name).parts
    if not parts or PurePosixPath(name).as_posix() != name or any(part.startswith(".") for part in parts):
        return False
    if name in FIXED | {NATIVE, TARGET_FILE}:
        return True
    if len(parts) == 3 and parts[0] == "smartcart" and parts[1] in PYTHON_PACKAGES:
        return re.fullmatch(r"[a-z_][a-z0-9_]*\.py", parts[2]) is not None
    if len(parts) == 3 and parts[:2] == ("smartcart", "launch"):
        return re.fullmatch(r"[a-z_][a-z0-9_]*\.launch\.py", parts[2]) is not None
    return (len(parts) in (3, 4) and parts[:2] == ("smartcart", "web")
            and PurePosixPath(name).suffix in {".html", ".css", ".js", ".svg"})


def _unique(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError("중복 JSON 키를 허용하지 않습니다")
        value[key] = item
    return value


def _constant(_value):
    raise ValueError("비유한 JSON 상수를 허용하지 않습니다")


def parse_json(raw):
    if len(raw) > 65536:
        raise ValueError("대상 메타데이터 크기 상한 초과")
    try:
        return json.loads(raw.decode("utf-8"), object_pairs_hook=_unique, parse_constant=_constant)
    except (UnicodeError, RecursionError) as error:
        raise ValueError("대상 메타데이터 JSON이 잘못되었습니다") from error


def _exact(value, expected):
    if type(value) is not type(expected):
        return False
    if type(expected) is dict:
        return value.keys() == expected.keys() and all(_exact(value[key], item) for key, item in expected.items())
    return value == expected


def validate_metadata(value):
    fields = {"schema", "release_version", "target", "safety", "native", "source_sha256"}
    if (type(value) is not dict or value.keys() != fields or value["schema"] != "smartcart.pi4_package.v1"
            or type(value["release_version"]) is not str
            or not re.fullmatch(r"[0-9]{1,4}\.[0-9]{1,4}\.[0-9]{1,4}", value["release_version"])
            or not _exact(value["target"], TARGET) or not _exact(value["safety"], SAFETY)):
        raise ValueError("Pi 4 대상·버전·안전 메타데이터가 잘못되었습니다")
    native = value["native"]
    expected = {"path": NATIVE, "elf_class": 64, "endianness": "little", "machine": 183, "abi_version": 1}
    if (type(native) is not dict or native.keys() != expected.keys() | {"sha256"}
            or not all(_exact(native[key], item) for key, item in expected.items())
            or type(native["sha256"]) is not str or not re.fullmatch(r"[0-9a-f]{64}", native["sha256"])):
        raise ValueError("ARM64 네이티브 메타데이터가 잘못되었습니다")
    sources = value["source_sha256"]
    if (type(sources) is not dict or sources.keys() != NATIVE_SOURCES
            or any(type(item) is not str or not re.fullmatch(r"[0-9a-f]{64}", item) for item in sources.values())):
        raise ValueError("원본 파서 소스 해시 메타데이터가 잘못되었습니다")
    return value


def validate_elf(data):
    """ARM 코드를 실행하지 않고 ELF64 공유 라이브러리와 적재 범위를 검사한다."""
    if (type(data) is not bytes or not 64 <= len(data) <= MAX_FILE or data[:7] != b"\x7fELF\x02\x01\x01"
            or data[7] not in (0, 3)):
        raise ValueError("Linux ARM64 ELF64 little-endian 라이브러리가 필요합니다")
    kind, machine, version = struct.unpack_from("<HHI", data, 16)
    offset = struct.unpack_from("<Q", data, 32)[0]
    header_size, entry_size, count = struct.unpack_from("<HHH", data, 52)
    if (kind != 3 or machine != 183 or version != 1 or header_size != 64 or entry_size != 56
            or not 1 <= count <= 128 or offset < 64 or offset + count * entry_size > len(data)):
        raise ValueError("공유 ELF의 아키텍처·헤더·프로그램 표가 잘못되었습니다")
    kinds = set()
    for index in range(count):
        segment, _flags, start, _virtual, _physical, size, memory, _align = struct.unpack_from("<IIQQQQQQ", data, offset + index * entry_size)
        if start + size > len(data) or (segment == 1 and size > memory):
            raise ValueError("ELF 적재 영역이 파일 범위를 벗어납니다")
        kinds.add(segment)
    if not {1, 2} <= kinds:
        raise ValueError("ELF 적재·동적 연결 영역이 필요합니다")


def _manifest(raw):
    if len(raw) > MAX_MANIFEST:
        raise ValueError("내용 해시 목록 크기 상한 초과")
    result = {}
    for line in raw.decode("ascii").splitlines():
        match = re.fullmatch(r"([0-9a-f]{64})  ([A-Za-z0-9_./+-]+)", line)
        if not match or not allowed_path(match[2]) or match[2] in result:
            raise ValueError("내용 해시 목록 형식·경로가 잘못되었습니다")
        result[match[2]] = match[1]
    return result


def validate_archive(path, expected_sha256):
    path = Path(path)
    if type(expected_sha256) is not str or not re.fullmatch(r"[0-9a-f]{64}", expected_sha256):
        raise ValueError("전달받은 SHA-256 64자리 소문자 해시가 필요합니다")
    if path.is_symlink() or not path.is_file() or path.stat().st_size > MAX_ARCHIVE:
        raise ValueError("일반 ZIP 파일 또는 전체 크기 제한을 확인하십시오")
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(65536), b""):
            digest.update(chunk)
    if digest.hexdigest() != expected_sha256:
        raise ValueError("전달 ZIP 해시가 다릅니다")
    with zipfile.ZipFile(path) as archive:
        entries, names, folded = archive.infolist(), set(), set()
        if len(entries) > MAX_FILES or sum(item.file_size for item in entries) > MAX_EXPANDED:
            raise ValueError("ZIP 파일 수·전체 해제 크기 제한 초과")
        for item in entries:
            if not item.filename.startswith(PREFIX) or item.is_dir() or item.flag_bits & 1:
                raise ValueError("ZIP 경로·디렉터리·암호화 항목을 거부합니다")
            name, mode = item.filename[len(PREFIX):], item.external_attr >> 16
            if (name in names or name.casefold() in folded or (name != MANIFEST and not allowed_path(name))
                    or stat.S_IFMT(mode) not in (0, stat.S_IFREG) or item.file_size > MAX_FILE):
                raise ValueError("중복·비허용 경로·링크·특수 파일·큰 항목을 거부합니다")
            names.add(name)
            folded.add(name.casefold())
        if not LEGACY_REQUIRED | {MANIFEST} <= names:
            raise ValueError("Pi 4 묶음에 필수 파일이 빠졌습니다")
        if archive.getinfo(PREFIX + MANIFEST).file_size > MAX_MANIFEST:
            raise ValueError("내용 해시 목록 크기 상한 초과")
        if archive.getinfo(PREFIX + TARGET_FILE).file_size > 65536:
            raise ValueError("대상 메타데이터 크기 상한 초과")
        hashes = _manifest(archive.read(PREFIX + MANIFEST))
        if hashes.keys() != names - {MANIFEST}:
            raise ValueError("ZIP 내용과 해시 목록이 다릅니다")
        metadata = validate_metadata(parse_json(archive.read(PREFIX + TARGET_FILE)))
        if not required_for_version(metadata["release_version"]) <= names:
            raise ValueError("이 버전의 Pi 4 묶음에 오프라인 실행 필수 파일이 빠졌습니다")
        if metadata["native"]["sha256"] != hashes[NATIVE] or any(metadata["source_sha256"][name] != hashes[name] for name in NATIVE_SOURCES):
            raise ValueError("원본 소스·네이티브 해시와 대상 메타데이터가 다릅니다")
        for name, expected in hashes.items():
            digest, size = hashlib.sha256(), 0
            with archive.open(PREFIX + name) as source:
                for chunk in iter(lambda: source.read(65536), b""):
                    size += len(chunk)
                    if size > MAX_FILE:
                        raise ValueError("해제 파일 크기 제한 초과")
                    digest.update(chunk)
            if digest.hexdigest() != expected:
                raise ValueError("내부 파일 내용 해시가 다릅니다")
        validate_elf(archive.read(PREFIX + NATIVE))
        return hashes


def check_target_environment():
    if sys.platform != "linux" or platform.machine() != "aarch64" or os.geteuid() == 0:
        raise ValueError("Linux aarch64의 일반 사용자로 실행하십시오. sudo는 허용하지 않습니다")
    release = {}
    for line in Path("/etc/os-release").read_text(encoding="utf-8").splitlines():
        if line.startswith(("ID=", "VERSION_ID=")):
            key, value = line.split("=", 1)
            parsed = shlex.split(value)
            if len(parsed) != 1 or key in release:
                raise ValueError("OS 배포판 정보를 확인할 수 없습니다")
            release[key] = parsed[0]
    if release != {"ID": "ubuntu", "VERSION_ID": "22.04"}:
        raise ValueError("Ubuntu 22.04 ARM64 배포본만 설치할 수 있습니다")


def install(path, expected_sha256):
    check_target_environment()
    home = Path.home().resolve(strict=True)
    if home == Path("/") or not home.is_dir():
        raise ValueError("일반 계정 홈을 확인할 수 없습니다")
    target = home / "SmartCart-D5-v2"
    if target.exists() or target.is_symlink():
        raise ValueError("~/SmartCart-D5-v2가 이미 있습니다. 기존 폴더는 덮어쓰지 않습니다")
    hashes = validate_archive(path, expected_sha256)
    with tempfile.TemporaryDirectory(prefix=".smartcart-pi4-", dir=home) as directory:
        stage = Path(directory)
        with zipfile.ZipFile(path) as archive:
            for name in [*hashes, MANIFEST]:
                output = stage.joinpath(*PurePosixPath(name).parts)
                output.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
                digest, size = hashlib.sha256(), 0
                with archive.open(PREFIX + name) as source, output.open("xb") as destination:
                    for chunk in iter(lambda: source.read(65536), b""):
                        size += len(chunk)
                        if size > (MAX_MANIFEST if name == MANIFEST else MAX_FILE):
                            raise ValueError("추출 도중 크기 제한 초과")
                        digest.update(chunk)
                        destination.write(chunk)
                if name != MANIFEST and digest.hexdigest() != hashes[name]:
                    raise ValueError("추출 도중 내용 해시가 달라졌습니다")
                output.chmod(0o700 if name.endswith(".sh") else 0o600)
        if _manifest((stage / MANIFEST).read_bytes()) != hashes:
            raise ValueError("추출 도중 해시 목록의 내용이 달라졌습니다")
        # 최종 경로를 배타적으로 만든 뒤 검증된 새 파일만 이동한다.
        target.mkdir(mode=0o700)
        for child in stage.iterdir():
            child.rename(target / child.name)
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--sha256", required=True)
    parser.add_argument("--verify-only", action="store_true", help="대상 코드 실행·설치 없이 ZIP 내용만 검증")
    args = parser.parse_args()
    try:
        if args.verify_only:
            print(f"Pi 4 묶음 검증 완료: {len(validate_archive(args.archive, args.sha256))}개 파일. 설치하지 않았습니다")
        else:
            print(f"검증·설치 완료: {install(args.archive, args.sha256)}")
            print("apt·서비스·드라이버·장치는 실행하지 않았습니다. README-PI4.md를 읽으십시오")
    except (ValueError, OSError, zipfile.BadZipFile, UnicodeError) as error:
        print("Pi 4 설치 실패: " + str(error), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
