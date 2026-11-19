"""검수 집계를 명시한 새 로컬 파일에만 제한해서 저장한다."""
import json
import os
from pathlib import Path

MAX_REPORT_BYTES = 1024 * 1024


def encoded_report(value):
    raw = json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + "\n"
    if len(raw.encode("utf-8")) > MAX_REPORT_BYTES:
        raise ValueError("검수 집계가 1 MiB 상한을 넘었습니다")
    return raw


def write_report(path, value):
    """기존 파일·링크를 덮어쓰지 않고 기본 접근 권한을 현재 계정으로 제한한다."""
    text = os.fspath(path)
    if (not text or len(text) > 4096 or any(ord(char) < 32 for char in text)
            or text.startswith(("\\\\", "//")) or "://" in text):
        raise ValueError("명시적인 로컬 보고서 경로가 필요합니다")
    destination = Path(path)
    raw = encoded_report(value)
    descriptor = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as output:
        output.write(raw)
