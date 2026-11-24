"""위치·태그는 명시적 로컬 파일에만 저장하는 장치 없는 위치 재생 명령."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

from .config import example_config_path, load_config
from .estimation_config import load_estimation_config
from .estimation_replay import estimate_file
from .replay import MAX_FILE_BYTES


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="BU04 좌우 좌표·시간 정합과 태그 위치 재생; 주행 허가 없음")
    parser.add_argument("--config", type=Path, default=example_config_path())
    parser.add_argument("--estimation-config", type=Path, required=True,
                        help="거리 모델·태그 높이·오차와 시간 한도를 명시한 설정")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--native-library", type=Path)
    parser.add_argument("--output", type=Path, help="선택한 새 JSONL 파일에만 위치 결과 저장; 기존 파일 거부")
    parser.add_argument("--summary", action="store_true", help="기본 동작: 태그·좌표 없는 집계만 출력")
    args = parser.parse_args(argv)
    output = collector = None
    try:
        from .collector import DualCollector
        from .estimator import TagEstimator
        from .native import NativeParser
        config = load_config(args.config)
        settings = load_estimation_config(args.estimation_config)
        factory = lambda: NativeParser(library_path=args.native_library)
        collector = DualCollector(config, parser_factory=factory, source_mode="replay")
        estimator = TagEstimator(config, settings, source_mode="replay")
        if args.output is not None:
            descriptor = os.open(args.output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            output = os.fdopen(descriptor, "w", encoding="utf-8", newline="\n")
        summary = {"schema": "smartcart.uwb_estimation_replay_summary.v1",
                   "source_mode": "replay", "drive_authorized": False, "records": 0,
                   "estimates_created": 0, "valid_events": 0, "invalid_reasons": {},
                   "range_model": settings.range_model, "provisional": True,
                   "sources": {"synthetic": 0, "recorded": 0}}
        previous_sequence = 0
        output_bytes = 0
        for event in estimate_file(args.input, collector, estimator):
            summary["records"] += 1
            summary["sources"][event["original_source"]] += 1
            estimate = event["estimate"]
            sequence = estimate["estimate_sequence"]
            if sequence > previous_sequence:
                summary["estimates_created"] += sequence - previous_sequence
                previous_sequence = sequence
            if estimate["valid"]:
                summary["valid_events"] += 1
            else:
                reason = estimate["invalid_reason"]
                summary["invalid_reasons"][reason] = summary["invalid_reasons"].get(reason, 0) + 1
            if output is not None:
                line = json.dumps(event, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n"
                output_bytes += len(line.encode("utf-8"))
                if output_bytes > MAX_FILE_BYTES:
                    raise ValueError("위치 결과 출력 크기 상한을 넘었습니다")
                output.write(line)
        if output is not None:
            saved, output = output, None
            saved.close()
        print(json.dumps(summary, ensure_ascii=False, allow_nan=False))
        return 0
    except (ValueError, OSError, RuntimeError, ImportError):
        # 경로·태그·위치 등 입력 내용을 진단 로그로 복사하지 않는다.
        print("위치 재생 실패: 설정·입력·라이브러리·새 출력 경로와 크기 한도를 확인하십시오", file=sys.stderr)
        return 2
    finally:
        if output is not None:
            try:
                output.close()
            except OSError:
                print("위치 재생 출력 파일 종료에 실패했습니다", file=sys.stderr)
        if collector is not None:
            collector.close()


if __name__ == "__main__":
    raise SystemExit(main())
