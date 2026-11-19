#!/usr/bin/env python3
"""수신한 릴리스를 검증하여 현재 계정의 새 전용 폴더에만 설치한다."""
from __future__ import annotations

import argparse
import hashlib
import os
import re
import stat
import sys
import tempfile
import zipfile
from pathlib import Path, PurePosixPath

PREFIX = "SmartCart-Sensing/"
MANIFEST = "RELEASE.sha256"
MAX_ARCHIVE = 32 * 1024 * 1024
MAX_EXPANDED = 64 * 1024 * 1024
MAX_FILE = 10 * 1024 * 1024
MAX_FILES = 256
PYTHON_PACKAGES = {"smartcart_radar", "smartcart_sensing", "smartcart_uwb", "smartcart_tracking", "smartcart_field"}
EXAMPLE_CONFIGS = {"config/dual_uwb.d5.example.json", "config/uwb_estimation.example.json",
                   "config/target_tracking.example.json", "config/field_acceptance.example.json"}
FIXED = {
    "LICENSE", "README-SENSING.md", "sensing.py",
    "package.xml", "setup.py", "setup.cfg", "resource/smartcart_sensing",
    "docs/CALIBRATION.md",
    "deploy/sensing.env.example", "deploy/smartcart-sensing.service",
    "deploy/install-sensing.py", "scripts/start-sensing.sh",
    "scripts/doctor-sensing.sh", "scripts/package-sensing.py",
    "scripts/deploy-sensing.py",
} | EXAMPLE_CONFIGS


def allowed_path(name: str) -> bool:
    """런타임 코드·자산의 제한된 위치만 허용한다."""
    if len(name) > 220 or "\\" in name or not re.fullmatch(r"[A-Za-z0-9_./+-]+", name):
        return False
    parts = PurePosixPath(name).parts
    if PurePosixPath(name).as_posix() != name or not parts or any(part in {".", ".."} or part.startswith(".") for part in parts):
        return False
    if name in FIXED:
        return True
    if len(parts) == 2 and parts[0] in PYTHON_PACKAGES:
        return re.fullmatch(r"[a-z_][a-z0-9_]*\.py", parts[1]) is not None
    if len(parts) == 2 and parts[0] == "launch":
        return re.fullmatch(r"[a-z_][a-z0-9_]*\.launch\.py", parts[1]) is not None
    return len(parts) in {2, 3} and parts[0] == "web" and PurePosixPath(name).suffix in {".html", ".css", ".js", ".svg"}


def validate_archive(path: Path, expected_sha256: str) -> dict[str, str]:
    """상한·경로·타입·내용 해시를 모두 검사하고 추출 전 파일 목록을 반환한다."""
    if not re.fullmatch(r"[0-9a-f]{64}", expected_sha256):
        raise ValueError("SHA-256은 소문자 16진수 64자리여야 합니다.")
    if path.is_symlink() or not path.is_file() or path.stat().st_size > MAX_ARCHIVE:
        raise ValueError("릴리스 ZIP이 일반 파일이 아니거나 32 MiB를 초과합니다.")
    digest = hashlib.sha256()
    with path.open("rb") as archive_file:
        for chunk in iter(lambda: archive_file.read(65536), b""):
            digest.update(chunk)
    if digest.hexdigest() != expected_sha256:
        raise ValueError("릴리스 ZIP의 SHA-256이 전달받은 값과 다릅니다.")
    with zipfile.ZipFile(path) as archive:
        entries = archive.infolist()
        if len(entries) > MAX_FILES or sum(item.file_size for item in entries) > MAX_EXPANDED:
            raise ValueError("릴리스 파일 수 또는 전체 해제 크기가 상한을 초과합니다.")
        names: set[str] = set()
        for item in entries:
            if not item.filename.startswith(PREFIX) or item.is_dir() or item.flag_bits & 1:
                raise ValueError("잘못된 ZIP 경로·디렉터리·암호화 항목입니다.")
            name = item.filename[len(PREFIX):]
            if name in names or (name != MANIFEST and not allowed_path(name)):
                raise ValueError("허용하지 않는 파일 또는 중복 경로가 있습니다.")
            mode = item.external_attr >> 16
            if stat.S_IFMT(mode) not in {0, stat.S_IFREG} or item.file_size > MAX_FILE:
                raise ValueError("심볼릭 링크·특수 파일 또는 너무 큰 파일은 허용하지 않습니다.")
            names.add(name)
        if MANIFEST not in names:
            raise ValueError("릴리스 내용 해시 목록이 없습니다.")
        manifest_item = archive.getinfo(PREFIX + MANIFEST)
        if manifest_item.file_size > 65536:
            raise ValueError("해시 목록이 너무 큽니다.")
        hashes: dict[str, str] = {}
        for line in archive.read(manifest_item).decode("ascii").splitlines():
            match = re.fullmatch(r"([0-9a-f]{64})  ([A-Za-z0-9_./+-]+)", line)
            if not match or not allowed_path(match[2]) or match[2] in hashes:
                raise ValueError("잘못된 릴리스 해시 목록입니다.")
            hashes[match[2]] = match[1]
        if set(hashes) != names - {MANIFEST} or not {"sensing.py", "README-SENSING.md"}.issubset(hashes):
            raise ValueError("릴리스 내용과 해시 목록이 일치하지 않습니다.")
        for name, expected in hashes.items():
            digest = hashlib.sha256()
            read_size = 0
            with archive.open(PREFIX + name) as source:
                for chunk in iter(lambda: source.read(65536), b""):
                    read_size += len(chunk)
                    if read_size > MAX_FILE:
                        raise ValueError("파일 해제 크기가 상한을 초과합니다.")
                    digest.update(chunk)
            if digest.hexdigest() != expected:
                raise ValueError("릴리스 내부 파일 해시가 일치하지 않습니다.")
        return hashes


def install(path: Path, expected_sha256: str) -> Path:
    if sys.platform != "linux" or os.geteuid() == 0:
        raise ValueError("이 설치기는 Pi Linux의 일반 SSH 사용자로 실행해야 합니다. sudo를 사용하지 마십시오.")
    home = Path.home().resolve()
    if home == Path("/"):
        raise ValueError("사용자 홈 경로를 확인할 수 없습니다.")
    target = home / "SmartCart-Sensing"
    if target.exists() or target.is_symlink():
        raise ValueError("~/SmartCart-Sensing이 이미 존재합니다. 기존 내용을 확인하고 별도 이름으로 보존한 뒤 재실행하십시오.")
    hashes = validate_archive(path, expected_sha256)
    # 새 폴더에서 전부 검증을 마친 후 대상 디렉터리를 배타적으로 생성한다.
    with tempfile.TemporaryDirectory(prefix=".smartcart-install-", dir=home) as temp:
        stage = Path(temp)
        with zipfile.ZipFile(path) as archive:
            for name in [*hashes, MANIFEST]:
                output = stage.joinpath(*PurePosixPath(name).parts)
                output.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
                digest = hashlib.sha256()
                read_size = 0
                with archive.open(PREFIX + name) as source, output.open("xb") as dest:
                    for chunk in iter(lambda: source.read(65536), b""):
                        read_size += len(chunk)
                        if read_size > (65536 if name == MANIFEST else MAX_FILE):
                            raise ValueError("해제 도중 파일이 크기 상한을 초과했습니다.")
                        digest.update(chunk)
                        dest.write(chunk)
                if name != MANIFEST and digest.hexdigest() != hashes[name]:
                    raise ValueError("해제 도중 내용이 바뀐 릴리스를 거부했습니다.")
                output.chmod(0o700 if output.suffix == ".sh" else 0o600)
        # mkdir는 동시 설치나 기존 폴더를 덮어쓰지 않고 실패한다.
        target.mkdir(mode=0o700)
        for child in stage.iterdir():
            child.rename(target / child.name)
    return target


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, required=True, help="전달받은 릴리스 ZIP")
    parser.add_argument("--sha256", required=True, help="송신 PC에서 확인한 ZIP SHA-256")
    args = parser.parse_args()
    try:
        target = install(args.archive, args.sha256)
    except (ValueError, OSError, zipfile.BadZipFile, UnicodeError) as exc:
        print(f"설치 실패: {exc}", file=sys.stderr)
        return 2
    print(f"검증 및 설치 완료: {target}")
    print("장치·드라이버·서비스는 시작하지 않았습니다. README-SENSING.md를 따라 실측 구성을 확인하십시오.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
