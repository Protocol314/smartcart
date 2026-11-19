#!/usr/bin/env python3
"""기존 BU04 순수 C 원본과 얇은 ABI 연결부를 명시적으로만 빌드한다."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading


MAX_COMPILER_OUTPUT = 65536
COMPILE_TIMEOUT_SECONDS = 60
LIBRARY_FILENAME = ("smartcart_uwb_parser.dll" if sys.platform == "win32" else
                    "libsmartcart_uwb_parser.dylib" if sys.platform == "darwin" else
                    "libsmartcart_uwb_parser.so")


def _local_path(value: str) -> str:
    if (not isinstance(value, str) or not value or len(value) > 4096
            or any(ord(char) < 32 for char in value)
            or value.startswith(("\\\\", "//")) or "://" in value):
        raise ValueError("길이가 제한된 로컬 파일 경로를 지정하십시오")
    return value


def _compiler_path(value: str) -> Path:
    value = _local_path(value)
    found = shutil.which(value)
    if not found:
        raise ValueError("지정한 C 컴파일러를 찾지 못했습니다")
    compiler = Path(found).resolve(strict=True)
    if not compiler.is_file() or (sys.platform == "win32" and compiler.suffix.lower() != ".exe"):
        raise ValueError("직접 실행할 수 있는 C 컴파일러 파일이 필요합니다")
    return compiler


def _run_compiler(command: list[str]) -> str:
    """셸·표준입력을 사용하지 않으며 출력 상한 또는 시간 초과이면 컴파일러를 종료한다."""
    process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, shell=False)
    output = bytearray()
    overflow = threading.Event()

    def drain():
        assert process.stdout is not None
        while chunk := process.stdout.read(4096):
            remaining = MAX_COMPILER_OUTPUT - len(output)
            output.extend(chunk[:remaining])
            if len(chunk) > remaining:
                overflow.set()
                process.kill()
                break

    reader = threading.Thread(target=drain, daemon=True)
    reader.start()
    try:
        process.wait(timeout=COMPILE_TIMEOUT_SECONDS)
    except subprocess.TimeoutExpired as exc:
        process.kill()
        process.wait(timeout=5)
        raise RuntimeError("C 컴파일러 실행이 60초 상한을 초과했습니다") from exc
    finally:
        reader.join(timeout=2)
        if process.stdout is not None and not reader.is_alive():
            process.stdout.close()
    message = bytes(output).decode("utf-8", errors="replace")
    if reader.is_alive():
        raise RuntimeError("C 컴파일러 출력 스트림이 종료되지 않았습니다")
    if overflow.is_set():
        raise RuntimeError("C 컴파일러 출력이 65536바이트 상한을 초과했습니다")
    if process.returncode:
        raise RuntimeError("UWB 네이티브 빌드에 실패했습니다:\n" + message)
    return message


def build_native(cc: str, output: Path | str | None = None) -> Path:
    smartcart = Path(__file__).resolve().parents[1]
    repository = smartcart.parent
    core = repository / "components" / "c_key_core"
    sources = (smartcart / "native" / "uwb_parser_bridge.c", core / "bu04_pdoa.c")
    if not all(source.is_file() for source in sources):
        raise ValueError("저장소의 원본 BU04 파서와 ABI 연결부가 필요합니다")
    compiler = _compiler_path(cc)
    target = Path(_local_path(os.fspath(output))) if output is not None else repository / "tests" / "build" / "dual_uwb" / LIBRARY_FILENAME
    if target.is_symlink():
        raise ValueError("심볼릭 링크를 빌드 결과로 덮어쓸 수 없습니다")
    target = target.resolve()
    if target.suffix.lower() != Path(LIBRARY_FILENAME).suffix:
        raise ValueError("현재 플랫폼의 공유 라이브러리 확장자를 사용하십시오")
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=".uwb-build-", suffix=target.suffix, dir=target.parent)
    os.close(descriptor)
    temporary = Path(temporary_name)
    command = [str(compiler), "-std=c17", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic"]
    command += ["-dynamiclib"] if sys.platform == "darwin" else ["-shared"]
    if sys.platform != "win32":
        command += ["-fPIC", "-fvisibility=hidden"]
    command += ["-I", str(core / "include"), "-I", str(smartcart / "native"),
                *map(str, sources), "-o", str(temporary)]
    try:
        _run_compiler(command)
        if not temporary.is_file() or temporary.stat().st_size == 0:
            raise RuntimeError("C 컴파일러가 공유 라이브러리를 생성하지 않았습니다")
        os.replace(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)
    return target


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="gcc", help="C 컴파일러 실행 파일. 추가 셸 인수는 허용하지 않음")
    parser.add_argument("--output", type=Path, help="공유 라이브러리 경로. 기본값은 저장소 tests/build/dual_uwb")
    args = parser.parse_args(argv)
    try:
        result = build_native(args.cc, args.output)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"네이티브 빌드 오류: {exc}", file=sys.stderr)
        return 2
    print(f"네이티브 UWB 라이브러리: {result}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
