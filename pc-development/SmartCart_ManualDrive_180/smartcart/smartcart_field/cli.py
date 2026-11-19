"""Pi 환경·장치 식별·장착·수신 상태를 변경 없이 확인하는 명령행."""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import select
import sys
import threading
import time

from smartcart_uwb.config import example_config_path, load_config
from smartcart_uwb.estimation_config import load_estimation_config
from smartcart_tracking.config import load_tracking_config
from .reporting import encoded_report, write_report


def _seconds(text):
    value = float(text)
    if not math.isfinite(value) or not 1 <= value <= 3600:
        raise argparse.ArgumentTypeError("검수 시간은 1~3600초의 유한한 수여야 합니다")
    return value


def build_parser():
    parser = argparse.ArgumentParser(description="SmartCart 센서 읽기 전용 검수 · 모터 출력 없음")
    sub = parser.add_subparsers(dest="command", required=True)
    doctor = sub.add_parser("doctor", help="장치를 열지 않고 Pi ARM64·ROS 실행 환경 확인")
    doctor.add_argument("--require-pi4", action="store_true")
    doctor.add_argument("--output", type=Path)
    inventory = sub.add_parser("inventory", help="포트를 열지 않는 세 센서 후보 열거")
    inventory.add_argument("--show-identifiers", action="store_true", help="원문 장치 식별자를 화면에만 명시적으로 표시")
    inventory.add_argument("--output", type=Path)
    for name, help_text in (("check", "장치 배정·장착 좌표·실측 증빙 확인"),
                            ("monitor", "기존 ROS 입력만 구독하여 시간차·단절·복구 관측"),
                            ("replay", "실물 없이 원시 캡처의 검수 메타데이터 재생")):
        command = sub.add_parser(name, help=help_text)
        command.add_argument("--mounting-config", type=Path)
        command.add_argument("--field-config", type=Path)
        command.add_argument("--uwb-config", type=Path)
        command.add_argument("--tracking-config", type=Path)
        command.add_argument("--output", type=Path, help="위치·태그·원문 식별자 없는 새 로컬 집계만 저장")
        if name == "monitor":
            command.add_argument("--seconds", type=_seconds, default=300)
            command.add_argument("--scan-topic", default="/scan")
            command.add_argument("--status-topic", default="/smartcart/uwb/dual/status")
            command.add_argument("--tracking-status-topic")
            command.add_argument("--interactive-markers", action="store_true",
                                 help="표준입력 unplug/replug 선언과 실제 관측 순서 대조")
            command.add_argument("--progress", action="store_true", help="위치 없는 상태 변경을 표준오류에 표시")
        if name == "replay":
            command.add_argument("--input", type=Path, required=True)
            command.add_argument("--native-library", type=Path)
    return parser


def environment_report():
    def available(name):
        try:
            return importlib.util.find_spec(name) is not None
        except (ImportError, ModuleNotFoundError, ValueError):
            return False
    os_info = {}
    release = Path("/etc/os-release")
    if release.is_file():
        for line in release.read_text(encoding="utf-8")[:4096].splitlines():
            if "=" in line:
                name, value = line.split("=", 1)
                if name in ("ID", "VERSION_ID"):
                    os_info[name] = value.strip('"')
    model_path = Path("/proc/device-tree/model")
    model = model_path.read_bytes()[:256].replace(b"\0", b"").decode("utf-8", errors="replace") if model_path.is_file() else ""
    dependencies = {name: available(name) for name in ("serial", "rclpy", "sensor_msgs", "std_msgs", "smartcart_uwb_interfaces")}
    target = (sys.platform.startswith("linux") and platform.machine().lower() == "aarch64"
              and os_info == {"ID": "ubuntu", "VERSION_ID": "22.04"})
    pi4 = "Raspberry Pi 4" in model
    return {"schema": "smartcart.field_environment.v1", "system": platform.system(),
            "architecture": platform.machine(), "python": platform.python_version(), "os": os_info,
            "target_platform_matches": target, "raspberry_pi4_detected": pi4,
            "ros_distro": os.environ.get("ROS_DISTRO"), "dependencies": dependencies,
            "ready": target and pi4 and os.environ.get("ROS_DISTRO") == "humble" and all(dependencies.values()),
            "port_opened": False, "drive_authorized": False,
            "notice": "환경 확인은 센서 실측·보정·구동 승인과 별개입니다"}


def load_settings(args):
    from .config import load_field_config
    directory = example_config_path().parent
    paths = {"mounting": args.mounting_config or directory / "dual_uwb.d5.example.json",
             "field": args.field_config or directory / "field_acceptance.example.json",
             "uwb": args.uwb_config or directory / "uwb_estimation.example.json",
             "tracking": args.tracking_config or directory / "target_tracking.example.json"}
    values = (load_config(paths["mounting"]), load_field_config(paths["field"]),
              load_estimation_config(paths["uwb"]), load_tracking_config(paths["tracking"]))
    hashes = {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in paths.items()}
    return values, hashes


def static_report(mounting, field, uwb):
    from .identity import enumerate_inventory, check_identity
    from .mounting import inspect_mounting, readiness_summary
    identity = check_identity(mounting, field, enumerate_inventory())
    geometry = inspect_mounting(mounting, field, uwb)
    return identity, geometry, readiness_summary(identity, geometry)


def marker_reader(monitor, stop, stream=sys.stdin):
    """파이프·터미널에서 제한된 수동 선언만 읽으며 포트·센서 설정을 쓰지 않는다."""
    print("분리 직전에 unplug uwb_left/uwb_right/lidar_center, 복구 직전에 replug 역할을 입력하십시오.", file=sys.stderr)
    pending, dropping = bytearray(), False
    while not stop.is_set():
        try:
            ready, _, _ = select.select([stream], [], [], .1)
            chunk = os.read(stream.fileno(), 256) if ready else None
        except (OSError, ValueError):
            print("표준입력이 닫혀 수동 선언을 더 받지 않습니다.", file=sys.stderr)
            return
        if not ready:
            continue
        if not chunk:
            return
        for byte in chunk:
            if byte != 10:
                if len(pending) >= 128 or byte > 127:
                    dropping = True
                if not dropping:
                    pending.append(byte)
                continue
            pieces = bytes(pending).decode("ascii").strip().split()
            pending.clear()
            invalid, dropping = dropping, False
            if invalid or len(pieces) != 2 or pieces[0] not in ("unplug", "replug"):
                print("수동 선언 형식: unplug 또는 replug, 공백, 센서 역할", file=sys.stderr)
                continue
            try:
                accepted = monitor.mark(pieces[1], pieces[0], time.monotonic_ns())
                print("수동 선언 기록" if accepted else "수동 선언 순서·현재 수신 상태를 확인하십시오", file=sys.stderr)
            except ValueError:
                print("센서 역할은 uwb_left·uwb_right·lidar_center 중 하나입니다", file=sys.stderr)


def _timing_summary(report):
    statistics, limits = report["statistics"], report["thresholds"]
    pairs = (("pair_rx_skew_ms", "max_pair_skew_ms"),
             ("lidar_pair_rx_skew_ms", "max_sensor_skew_ms"),
             ("lidar_scan_duration_ms", "max_scan_duration_ms"))
    checks = {name: {"observed": statistics[name]["count"] > 0,
                     "maximum_ms": statistics[name]["max_observed"], "limit_ms": limits[limit],
                     "within_limit": statistics[name]["count"] > 0
                         and statistics[name]["max_observed"] <= limits[limit]}
              for name, limit in pairs}
    return {"checks": checks, "all_observed_within_limits": all(item["within_limit"] for item in checks.values()),
            "basis": "HOST_RX와 ROS 도착 시각의 소프트웨어 관측이며 하드웨어 동기화 검증이 아님"}


def progress_reader(monitor, stop):
    previous = None
    while not stop.wait(.25):
        value = monitor.report()
        summary = {name: {"state": item["state"],
                          "manual_phase": value["manual_unplug_tests"][name]["phase"]}
                   for name, item in value["sensors"].items()}
        if summary != previous:
            print("검수 관측 " + json.dumps(summary, ensure_ascii=False), file=sys.stderr, flush=True)
            previous = summary


def execute(args):
    if args.command == "doctor":
        report = environment_report()
        return report, 0 if not args.require_pi4 or report["ready"] else 3
    if args.command == "inventory":
        from .identity import enumerate_inventory, redact_inventory
        inventory = enumerate_inventory()
        report = {"schema": "smartcart.field_inventory.v1", "inventory": redact_inventory(inventory),
                  "port_opened": False, "drive_authorized": False, "automatic_assignment": False}
        if args.show_identifiers:
            print("명시적으로 요청한 원문 식별자이며 보고서에는 저장하지 않습니다.", file=sys.stderr)
            print(json.dumps(inventory, ensure_ascii=False, allow_nan=False), file=sys.stderr)
        return report, 0
    (mounting, field, uwb, tracking), hashes = load_settings(args)
    report = {"schema": "smartcart.field_report.v1", "command": args.command,
              "config_sha256": hashes, "drive_authorized": False, "provisional": True,
              "physical_acceptance": False, "firmware_or_calibration_changed": False}
    if args.command in ("check", "monitor"):
        identity, geometry, readiness = static_report(mounting, field, uwb)
        report.update(identity=identity, mounting=geometry, readiness=readiness)
        if args.command == "check":
            report["inspection_complete"] = False
            return report, 0 if readiness["ready"] else 3
    from .monitor import FieldMonitor
    monitor = FieldMonitor(mounting, uwb, tracking, source_mode="replay" if args.command == "replay" else "live")
    if args.command == "replay":
        from .replay import inspect_replay
        from smartcart_uwb.native import NativeParser
        value = inspect_replay(mounting, monitor, args.input, parser_factory=lambda: NativeParser(args.native_library))
        report.update(source_mode="replay", original_source=value["original_source"], records=value["records"],
                      observation=value["report"], inspection_complete=False)
        report["timing"] = _timing_summary(value["report"])
        return report, 0
    from .ros_monitor import run_monitor
    stop = threading.Event()
    reader = None
    progress = None
    monitor.begin(time.monotonic_ns())
    if args.interactive_markers:
        if not sys.platform.startswith("linux"):
            raise ValueError("수동 선언 입력은 Linux 표준입력에서만 지원합니다")
        reader = threading.Thread(target=marker_reader, args=(monitor, stop), daemon=True)
        reader.start()
    if args.interactive_markers or args.progress:
        progress = threading.Thread(target=progress_reader, args=(monitor, stop), daemon=True)
        progress.start()
    try:
        run_monitor(monitor, duration_seconds=args.seconds, scan_topic=args.scan_topic,
                    status_topic=args.status_topic, tracking_status_topic=args.tracking_status_topic, stop=stop)
    finally:
        stop.set()
        if reader is not None:
            reader.join(timeout=1)
        if progress is not None:
            progress.join(timeout=1)
    observation = monitor.report()
    timing = _timing_summary(observation)
    complete = report["readiness"]["ready"] and observation["completion"] and timing["all_observed_within_limits"]
    report.update(source_mode="live", observation=observation, timing=timing, inspection_complete=bool(complete))
    return report, 0 if complete else 3


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        # 관측을 시작하기 전에 출력 대상 충돌을 거부한다.
        if args.output is not None and (args.output.exists() or args.output.is_symlink()):
            raise ValueError("기존 보고서는 덮어쓰지 않습니다")
        report, code = execute(args)
        if args.output is not None:
            write_report(args.output, report)
        print(encoded_report(report), end="")
        return code
    except KeyboardInterrupt:
        print("검수를 중단했습니다. 완료로 판정하지 않습니다.", file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, OSError, ImportError) as error:
        print("읽기 전용 검수 오류: " + str(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
