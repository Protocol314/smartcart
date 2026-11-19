#!/usr/bin/env python3
"""검증된 ZIP을 키 기반 SSH로 전달하고 새 사용자 전용 폴더에만 설치한다."""
from __future__ import annotations

import argparse
import importlib.util
import re
import secrets
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path


def target_value(value: str) -> str:
    if not re.fullmatch(r"[a-z_][a-z0-9_-]{0,31}@[A-Za-z0-9][A-Za-z0-9.-]{0,252}", value):
        raise argparse.ArgumentTypeError("SSH 대상은 일반 사용자@IPv4주소 또는 사용자@호스트명 형식이어야 합니다.")
    if value.split("@", 1)[0] == "root":
        raise argparse.ArgumentTypeError("root 계정 대신 일반 SSH 사용자를 지정하십시오.")
    return value


def sha256_value(value: str) -> str:
    if not re.fullmatch(r"[0-9a-f]{64}", value):
        raise argparse.ArgumentTypeError("SHA-256은 소문자 16진수 64자리여야 합니다.")
    return value


def port_value(value: str) -> int:
    try:
        port = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("SSH 포트는 1~65535 정수여야 합니다.") from exc
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("SSH 포트는 1~65535 정수여야 합니다.")
    return port


def deploy(target: str, port: int, archive: Path, expected: str, *, dry_run: bool = False) -> None:
    root = Path(__file__).resolve().parents[1]
    installer_file = root / "deploy/install-sensing.py"
    spec = importlib.util.spec_from_file_location("sensing_release_installer", installer_file)
    if spec is None or spec.loader is None:
        raise ValueError("릴리스 검증기를 찾을 수 없습니다.")
    installer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(installer)
    installer.validate_archive(archive, expected)
    # 원격 셸에 삽입되는 문자열은 아래에서 생성한 16진 토큰·해시·고정 파일명뿐이다.
    token = secrets.token_hex(12)
    remote_archive = f"smartcart-sensing-{token}.zip"
    remote_installer = f"smartcart-sensing-install-{token}.py"
    common = ["-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=10"]
    scp_command = ["scp", "-B", "-P", str(port), *common, remote_archive, remote_installer, f"{target}:./"]
    remote_command = f"/usr/bin/python3 {remote_installer} --archive {remote_archive} --sha256 {expected}"
    ssh_command = ["ssh", "-p", str(port), *common, target, remote_command]
    if dry_run:
        print(f"검증 완료. 전송 예정 대상: {target}, SSH 포트: {port}")
        print("설치 위치: 현재 SSH 계정의 ~/SmartCart-Sensing (기존 폴더가 있으면 실패)")
        print("드라이런: SSH 연결·전송·설치·서비스 기동을 하지 않았습니다.")
        return
    if shutil.which("ssh") is None or shutil.which("scp") is None:
        raise ValueError("로컬 OpenSSH ssh/scp 실행 파일이 필요합니다.")
    with tempfile.TemporaryDirectory(prefix="smartcart-upload-") as temp:
        staging = Path(temp)
        shutil.copyfile(archive, staging / remote_archive)
        shutil.copyfile(installer_file, staging / remote_installer)
        # Python shell=True 및 문자열 셸 조립을 사용하지 않는다.
        subprocess.run(scp_command, cwd=staging, check=True, timeout=180)
        subprocess.run(ssh_command, check=True, timeout=180)
    print("원격 설치가 완료되었습니다. 장치나 서비스는 시작하지 않았습니다.")
    print(f"전달용 임시 파일: ~/{remote_archive}, ~/{remote_installer} (확인 후 직접 제거 가능)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", type=target_value, required=True, help="일반 SSH 사용자@현재 Pi 호스트")
    parser.add_argument("--ssh-port", type=port_value, default=22)
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--sha256", type=sha256_value, required=True)
    parser.add_argument("--dry-run", action="store_true", help="ZIP만 검증하며 네트워크에 연결하지 않음")
    args = parser.parse_args()
    try:
        deploy(args.target, args.ssh_port, args.archive, args.sha256, dry_run=args.dry_run)
    except (ValueError, OSError, UnicodeError, zipfile.BadZipFile, subprocess.SubprocessError) as exc:
        print(f"SSH 배포 실패: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
