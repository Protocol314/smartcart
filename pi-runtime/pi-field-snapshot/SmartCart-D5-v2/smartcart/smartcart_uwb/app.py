"""장치를 자동 선택하지 않는 읽기 전용 수집 명령행."""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

from .config import load_config
from .collector import DualCollector
from .native import NativeParser
from .serial_reader import OutputError, SerialReaders, list_data_ports

MAX_CAPTURE_BYTES = 100 * 1024 * 1024
MAX_CAPTURE_RECORDS = 100_000


class CaptureWriter:
    """명시적으로 선택한 파일에만 새 기록을 만들고 덮어쓰지 않는다."""

    def __init__(self, path: Path | str):
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        self.file = os.fdopen(descriptor, "w", encoding="utf-8", newline="\n")
        self.size = 0
        self.records = 0
        self.lock = threading.Lock()

    def write(self, event: dict) -> None:
        from .replay import validate_record
        validate_record(event)
        line = json.dumps(event, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n"
        encoded_size = len(line.encode("utf-8"))
        with self.lock:
            if self.size + encoded_size > MAX_CAPTURE_BYTES or self.records >= MAX_CAPTURE_RECORDS:
                raise ValueError("기록 파일 크기 또는 레코드 수 상한에 도달했습니다")
            self.file.write(line)
            self.size += encoded_size
            self.records += 1

    def close(self) -> None:
        try:
            self.file.close()
        except OSError as exc:
            raise RuntimeError("기록 파일을 완전히 저장하지 못했습니다") from exc

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def status_summary(snapshot: dict) -> dict:
    """태그·거리·위치를 자동 로그에 넣지 않는 상태 요약이다."""
    return {"schema": "smartcart.dual_uwb_summary.v1", "drive_authorized": False,
            "source_mode": snapshot["source_mode"],
            "sensors": {key: {"state": value["state"],
                               "stream_epoch": value["stream_epoch"]}
                        for key, value in snapshot["sensors"].items()}}


def _duration(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or not .1 <= result <= 86400:
        raise argparse.ArgumentTypeError("수집 시간은 0.1~86400초여야 합니다")
    return result


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if arguments and arguments[0] == "replay":
        from .replay_cli import main as replay_main
        return replay_main(arguments[1:])
    if arguments and arguments[0] == "estimate-replay":
        from .estimation_cli import main as estimate_main
        return estimate_main(arguments[1:])
    parser = argparse.ArgumentParser(description="좌우 BU04 읽기 전용 수집기; 주행 명령 없음")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("replay", help="원시 캡처의 관측·상태 재생")
    commands.add_parser("estimate-replay", help="원시 캡처의 좌표·시간 정합과 위치 추정 재생")
    commands.add_parser("list-ports", help="포트를 열지 않고 장치 식별자를 열거")
    inspect = commands.add_parser("inspect-config", help="장착·장치 배정 설정 검사")
    inspect.add_argument("--config", type=Path, required=True)
    collect = commands.add_parser("collect", help="명시적으로 배정한 두 데이터 USB 수신")
    collect.add_argument("--config", type=Path, required=True)
    collect.add_argument("--native-library", type=Path)
    collect.add_argument("--seconds", type=_duration)
    collect.add_argument("--record", type=Path, help="선택한 JSONL 파일에만 기록; 기존 파일 거부")
    options = parser.parse_args(arguments)
    recorder = None
    try:
        if options.command == "list-ports":
            print(json.dumps(list_data_ports(), ensure_ascii=False, allow_nan=False))
            return 0
        config = load_config(options.config)
        if options.command == "inspect-config":
            print(json.dumps({"valid_config": True, "drive_authorized": False,
                              "sensors": {key: {"frame_id": sensor.frame_id,
                                                "port_assigned": sensor.port is not None,
                                                "device_identity_verified": sensor.device_identity_verified,
                                                "mounting_verified": sensor.mounting_verified}
                                          for key, sensor in config.sensors.items()}},
                             ensure_ascii=False, allow_nan=False))
            return 0
        factory = lambda: NativeParser(options.native_library)
        with DualCollector(config, parser_factory=factory, source_mode="live") as collector:
            # 기록 파일을 만들기 전에 장치 배정과 네이티브 라이브러리를 검사한다.
            readers = SerialReaders(collector)
            if options.record is not None:
                recorder = CaptureWriter(options.record)
                readers.capture_callback = recorder.write
            deadline = None if options.seconds is None else time.monotonic() + options.seconds
            with readers:
                previous = None
                while not readers.stop.is_set() and (deadline is None or time.monotonic() < deadline):
                    summary = status_summary(readers.tick())
                    if summary != previous:
                        print(json.dumps(summary, ensure_ascii=False, allow_nan=False), flush=True)
                        previous = summary
                    readers.stop.wait(.1)
                if readers.fatal_error:
                    raise RuntimeError(readers.fatal_error)
        if recorder is not None:
            writer, recorder = recorder, None
            writer.close()
        return 0
    except KeyboardInterrupt:
        if recorder is not None:
            writer, recorder = recorder, None
            try:
                writer.close()
            except RuntimeError:
                print("기록 파일 종료에 실패했습니다", file=sys.stderr)
                return 2
        return 0
    except (OSError, ValueError, RuntimeError, ImportError, OutputError) as exc:
        # 명령행에만 간결한 진단을 반환하며 원시 자료와 태그를 출력하지 않는다.
        print(f"수집 실패: {exc}", file=sys.stderr)
        return 2
    finally:
        if recorder is not None:
            try:
                recorder.close()
            except RuntimeError:
                print("기록 파일 종료에 실패했습니다", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
