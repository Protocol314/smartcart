"""원시 위치·태그를 기본 출력하지 않는 명시적 캡처 재생 명령."""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from .replay import ReplayError, replay_file
from .config import example_config_path

DEFAULT_CONFIG = example_config_path()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="BU04 좌우 캡처 읽기 전용 재생")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--native-library", type=Path)
    parser.add_argument("--output", type=Path, help="명시한 경우에만 관측 JSONL을 새 파일로 저장")
    parser.add_argument("--summary", action="store_true", help="기본 동작: 민감 관측 없는 집계만 출력")
    args = parser.parse_args(argv)
    collector = output = None
    try:
        from .config import load_config
        from .collector import DualCollector
        from .native import NativeParser
        config = load_config(args.config)
        factory = lambda: NativeParser(library_path=args.native_library)
        collector = DualCollector(config, parser_factory=factory, source_mode="replay")
        if args.output is not None:
            # 사용자 명시 경로만 쓰며 이전 실험 결과와 입력을 덮어쓰지 않는다.
            descriptor = os.open(args.output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            output = os.fdopen(descriptor, "w", encoding="utf-8", newline="\n")
        summary = {"schema": "smartcart.uwb_replay_summary.v1", "source_mode": "replay",
                   "drive_authorized": False, "records": 0, "observations": 0,
                   "first_at_ns": None, "last_at_ns": None,
                   "sources": {"synthetic": 0, "recorded": 0}}
        for event in replay_file(args.input, collector):
            summary["records"] += 1
            summary["observations"] += len(event["observations"])
            if summary["first_at_ns"] is None:
                summary["first_at_ns"] = event["at_ns"]
            summary["last_at_ns"] = event["at_ns"]
            summary["sources"][event["original_source"]] += 1
            if output is not None:
                output.write(json.dumps(event, ensure_ascii=False, allow_nan=False,
                                        separators=(",", ":")) + "\n")
        if output is not None:
            saved, output = output, None
            saved.close()
        print(json.dumps(summary, ensure_ascii=False, allow_nan=False))
        return 0
    except (ValueError, OSError, RuntimeError) as exc:
        message = str(exc) if isinstance(exc, ReplayError) else "설정·입력·원시 라이브러리·출력 경로를 확인하십시오"
        print(f"재생 실패: {message}", file=sys.stderr)
        return 2
    finally:
        if output is not None:
            try:
                output.close()
            except OSError:
                print("재생 출력 파일 종료에 실패했습니다", file=sys.stderr)
        if collector is not None:
            collector.close()


if __name__ == "__main__":
    raise SystemExit(main())
