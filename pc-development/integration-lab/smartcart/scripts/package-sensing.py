#!/usr/bin/env python3
"""기록·설정·비밀정보 없이 Pi 런타임만 새 해시 목록과 함께 묶는다."""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import stat
import sys
import zipfile
from pathlib import Path


def load_installer(root: Path):
    spec = importlib.util.spec_from_file_location("sensing_release_installer", root / "deploy/install-sensing.py")
    if spec is None or spec.loader is None:
        raise ValueError("릴리스 검증기를 찾을 수 없습니다.")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_release(root: Path, output: Path) -> tuple[str, int]:
    installer = load_installer(root)
    required = installer.FIXED | {name + "/__init__.py" for name in installer.PYTHON_PACKAGES} | {"web/index.html"}
    files: dict[str, bytes] = {}
    # 비밀키·가상환경·기록 디렉터리를 탐색하지 않고 런타임 위치만 읽는다.
    candidates = {root / name for name in installer.FIXED}
    for directory in (*sorted(installer.PYTHON_PACKAGES), "launch"):
        candidates.update((root / directory).glob("*.py"))
    candidates.update((root / "web").rglob("*"))
    for path in sorted(candidates):
        relative = path.relative_to(root).as_posix()
        if not installer.allowed_path(relative):
            continue
        if not path.exists() and not path.is_symlink():
            continue
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"일반 파일이 아닌 배포 항목입니다: {relative}")
        if path.stat().st_size > installer.MAX_FILE:
            raise ValueError(f"배포 항목이 10 MiB를 초과합니다: {relative}")
        # 허용된 경로의 부모도 심볼릭 링크이면 패키지 밖 파일을 포함할 수 있다.
        if not path.resolve().is_relative_to(root.resolve()):
            raise ValueError("패키지 경계 밖 파일은 배포할 수 없습니다.")
        files[relative] = path.read_bytes()
    missing = required - files.keys()
    if missing:
        raise ValueError("필수 런타임 파일이 없습니다: " + ", ".join(sorted(missing)))
    if len(files) + 1 > installer.MAX_FILES or sum(map(len, files.values())) > installer.MAX_EXPANDED - 65536:
        raise ValueError("배포 파일 수 또는 크기가 상한을 초과합니다.")
    manifest = "".join(f"{hashlib.sha256(data).hexdigest()}  {name}\n" for name, data in sorted(files.items()))
    files[installer.MANIFEST] = manifest.encode("ascii")
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    # 기존 릴리스 파일을 덮어쓰지 않는다.
    checksum_path = output.with_name(output.name + ".sha256")
    bootstrap_path = output.with_name(output.stem + "-install.py")
    if output.suffix != ".zip":
        raise ValueError("배포 파일은 .zip 확장자를 사용하십시오.")
    if any(path.exists() or path.is_symlink() for path in (output, checksum_path, bootstrap_path)):
        raise ValueError("동일 이름의 ZIP·설치기·체크섬 파일이 이미 존재합니다.")
    with output.open("xb") as destination:
        with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
            for name, data in sorted(files.items()):
                item = zipfile.ZipInfo(installer.PREFIX + name, date_time=(2026, 1, 1, 0, 0, 0))
                item.create_system = 3
                item.external_attr = (stat.S_IFREG | (0o700 if name.endswith(".sh") else 0o600)) << 16
                item.compress_type = zipfile.ZIP_DEFLATED
                archive.writestr(item, data)
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    installer.validate_archive(output, digest)
    # ZIP을 먼저 풀지 않고도 복사된 릴리스를 안전하게 설치할 수 있게 한다.
    bootstrap = files["deploy/install-sensing.py"]
    with bootstrap_path.open("xb") as bootstrap_file:
        bootstrap_file.write(bootstrap)
    with checksum_path.open("x", encoding="ascii", newline="\n") as checksum:
        checksum.write(digest + "  " + output.name + "\n")
        checksum.write(hashlib.sha256(bootstrap).hexdigest() + "  " + bootstrap_path.name + "\n")
    return digest, len(files) - 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="새 ZIP 경로. build/verification/ 아래 권장")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    try:
        digest, count = build_release(root, args.output)
    except (ValueError, OSError, zipfile.BadZipFile) as exc:
        print(f"패키지 생성 실패: {exc}", file=sys.stderr)
        return 2
    print(f"릴리스: {args.output.resolve()}")
    print(f"파일: {count}개, SHA-256: {digest}")
    print(f"수동 복사용 설치기: {args.output.with_name(args.output.stem + '-install.py').resolve()}")
    print(f"ZIP·설치기 해시: {args.output.with_name(args.output.name + '.sha256').resolve()}")
    print("환경파일·기록·원본 업스트림 MANIFEST.sha256는 포함하지 않았습니다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
