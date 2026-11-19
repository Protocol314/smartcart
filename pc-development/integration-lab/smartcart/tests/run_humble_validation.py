#!/usr/bin/env python3
"""실제 Linux/Humble 검사에서 환경 건너뛰기를 성공으로 계산하지 않는다."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import platform
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
REQUIRED = (
    "test_sensing_serial_runtime", "test_sensing_ros_runtime", "test_dual_uwb_ros_runtime",
    "test_tracking_ros.ActualRosTests", "test_tracking_ros_runtime", "test_dual_uwb_pty_runtime",
    "test_field_ros_runtime",
)


def preflight():
    if not sys.platform.startswith("linux") or os.environ.get("ROS_DISTRO") != "humble":
        raise RuntimeError("Linux에서 ROS 2 Humble과 새 설치 환경을 source해야 합니다")
    import serial
    import rclpy
    from sensor_msgs.msg import LaserScan
    from smartcart_uwb_interfaces.msg import UwbObservation
    from smartcart_uwb.native import NativeParser
    with NativeParser():
        pass
    return {"platform": platform.platform(), "python": platform.python_version(),
            "ros_distro": os.environ["ROS_DISTRO"], "pyserial": serial.VERSION,
            "rclpy_available": bool(rclpy), "generated_message_available": bool(UwbObservation),
            "scan_message_available": bool(LaserScan)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--full", action="store_true", help="필수 검사 대신 Python 전체 회귀 실행")
    parser.add_argument("--output", type=Path, help="새 로컬 JSON 집계만 저장")
    args = parser.parse_args(argv)
    if args.output is not None and args.output.exists():
        parser.error("기존 보고서는 덮어쓰지 않습니다")
    environment = preflight()
    suite = (unittest.defaultTestLoader.discover(str(ROOT / "tests")) if args.full else
             unittest.defaultTestLoader.loadTestsFromNames(REQUIRED))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    report = {"schema": "smartcart.humble_validation.v1", "environment": environment,
              "scope": "full" if args.full else "required_runtime", "tests_run": result.testsRun,
              "failures": len(result.failures), "errors": len(result.errors),
              "skipped": [{"test": str(test), "reason": reason} for test, reason in result.skipped],
              "passed": result.wasSuccessful() and not result.skipped and result.testsRun > 0,
              "inputs": "synthetic_and_linux_pty", "hardware_used": False, "drive_authorized": False}
    if args.output is not None:
        with args.output.open("x", encoding="utf-8") as destination:
            json.dump(report, destination, ensure_ascii=False, indent=2, allow_nan=False)
            destination.write("\n")
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
