"""기본 재생과 명시적 실측의 사전 점검·프로세스 감독 명령행."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import signal
import sys
import threading

from .config import default_config, initialize, load_config
from .plan import build_plan, check_plan
from .supervisor import InstanceLock, ensure_http_available, supervise


def _seconds(text):
    try:
        value = float(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("실행 시간은 0초 초과·3600초 이하의 수여야 합니다") from error
    if not 0 < value <= 3600:
        raise argparse.ArgumentTypeError("실행 시간은 0초 초과·3600초 이하의 수여야 합니다")
    return value


def build_parser():
    parser = argparse.ArgumentParser(description="SmartCart 통합 실행 · 기본 합성 재생 · 구동 출력 없음")
    parser.add_argument("--config", type=Path, help="로컬 run.json. 미지정 시 합성 재생 기본 설정")
    sub = parser.add_subparsers(dest="command", required=True)
    init = sub.add_parser("init", help="새 디렉터리에 미검증 설정을 덮어쓰기 없이 준비")
    init.add_argument("--directory", type=Path, default=Path("local-config"))
    for name, description in (("plan", "장치를 열거나 실행하지 않고 고정 명령 계획 확인"),
                              ("check", "장치 미개방 상태로 라이브러리·실행 환경 확인"),
                              ("start", "확인한 구성만 실행하고 종료·실패 시 함께 정리")):
        command = sub.add_parser(name, help=description)
        command.add_argument("--config", type=Path, default=argparse.SUPPRESS,
                             help="로컬 run.json. 전역 인수와 같은 의미")
        command.add_argument("--mode", choices=("replay", "live"), default="replay")
        command.add_argument("--single-left", action="store_true", help="왼쪽 BU04 단독 구성")
        if name == "start":
            command.add_argument("--seconds", type=_seconds, help="웹 서버 기동 후 제한 시간만 실행. 미지정 시 Ctrl+C까지")
    return parser


def _print(report):
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), flush=True)


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        if args.command == "init":
            path = initialize(args.directory)
            _print({"schema": "smartcart.run_init.v1", "config": str(path),
                    "configuration_verified": False, "port_opened": False,
                    "physical_acceptance": False, "drive_authorized": False})
            return 0
        config = load_config(args.config) if args.config is not None else default_config()
        plan = build_plan(config, args.mode, single_left=args.single_left)
        if args.command == "plan":
            _print(plan)
            return 0 if plan["configuration_ready"] else 3
        report = check_plan(config, plan)
        if args.command == "check" or not report["ready"]:
            _print(report)
            return 0 if report["ready"] else 3
        stop = threading.Event()
        previous = {}
        def interrupted(signum, frame):
            stop.set()
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, interrupted)
        try:
            with ExitStack() as locks:
                locks.enter_context(InstanceLock(config.http_port))
                if args.mode == "live":
                    locks.enter_context(InstanceLock(config.http_port, scope="live"))
                ensure_http_available(config.http_port)
                return supervise(plan["commands"], port=config.http_port, duration=args.seconds, stop=stop)
        finally:
            for signum, handler in previous.items():
                signal.signal(signum, handler)
    except KeyboardInterrupt:
        print("통합 실행을 중단했습니다", file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, OSError, ImportError, UnicodeError, RecursionError) as error:
        print("통합 실행 오류: " + str(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
