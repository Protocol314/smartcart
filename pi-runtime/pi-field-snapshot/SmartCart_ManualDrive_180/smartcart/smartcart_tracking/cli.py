"""태그·좌표를 기본 출력하지 않는 대상 추적 재생 명령."""
import argparse
import json
import os
from pathlib import Path
import sys

from smartcart_uwb.config import example_config_path, load_config
from smartcart_uwb.estimation_config import load_estimation_config
from smartcart_uwb.native import NativeParser
from smartcart_uwb.replay import MAX_FILE_BYTES
from .config import load_tracking_config
from .pipeline import TrackingPipeline
from .replay import track_file


def main(argv=None):
    parser = argparse.ArgumentParser(description="LiDAR 클러스터·UWB 태그 대상 연관·추적의 읽기 전용 재생")
    parser.add_argument("--config", type=Path, default=example_config_path())
    parser.add_argument("--uwb-estimation-config", type=Path, required=True)
    parser.add_argument("--tracking-config", type=Path, required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--native-library", type=Path)
    parser.add_argument("--output", type=Path, help="선택한 새 로컬 파일에만 위치·추적 결과 저장")
    parser.add_argument("--summary", action="store_true", help="기본 동작: 좌표·태그 없는 집계만 출력")
    args = parser.parse_args(argv)
    output = None
    try:
        mounting = load_config(args.config)
        uwb_settings = load_estimation_config(args.uwb_estimation_config)
        settings = load_tracking_config(args.tracking_config)
        factory = lambda: NativeParser(args.native_library)
        with TrackingPipeline(mounting, uwb_settings, settings, parser_factory=factory) as pipeline:
            if args.output is not None:
                descriptor = os.open(args.output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
                output = os.fdopen(descriptor, "w", encoding="utf-8", newline="\n")
            summary = {"schema": "smartcart.target_replay_summary.v1", "source_mode": "replay",
                       "drive_authorized": False, "provisional": True, "records": 0,
                       "uwb_observations": 0, "valid_track_events": 0, "tracking_entries": 0,
                       "states": {}, "sources": {"synthetic": 0, "recorded": 0}}
            output_bytes = 0
            previous_state = None
            for event in track_file(args.input, pipeline):
                summary["records"] += 1
                summary["sources"][event["original_source"]] += 1
                summary["uwb_observations"] += event["uwb_observation_count"]
                track = event["track"]
                state = track["state"]
                summary["states"][state] = summary["states"].get(state, 0) + 1
                summary["valid_track_events"] += int(track["valid"])
                if state == "tracking" and previous_state != "tracking":
                    summary["tracking_entries"] += 1
                previous_state = state
                if output is not None:
                    line = json.dumps(event, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n"
                    output_bytes += len(line.encode("utf-8"))
                    if output_bytes > MAX_FILE_BYTES:
                        raise ValueError("추적 출력 파일 크기 상한을 넘었습니다")
                    output.write(line)
            if output is not None:
                saved, output = output, None
                saved.close()
            print(json.dumps(summary, ensure_ascii=False, allow_nan=False))
        return 0
    except (ValueError, TypeError, OSError, RuntimeError, ImportError):
        print("대상 추적 재생 실패: 설정·입력·라이브러리·새 출력 경로와 크기 상한을 확인하십시오", file=sys.stderr)
        return 2
    finally:
        if output is not None:
            try:
                output.close()
            except OSError:
                print("추적 출력 파일 종료에 실패했습니다", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
