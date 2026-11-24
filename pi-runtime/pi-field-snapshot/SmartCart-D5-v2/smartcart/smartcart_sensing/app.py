"""라즈베리파이 ROS 2 및 명시적 합성 모드의 통합 센싱 실행기."""
from __future__ import annotations

import argparse
import importlib.util
import ipaddress
import json
import math
import os
from pathlib import Path
import platform
import signal
import sys
import threading

from smartcart_radar.core import ScanStore
from smartcart_radar.sources import source_loop
from . import __version__
from .ros import ros_loop, validate_topic
from .server import SensingHTTPServer
from .service import SensingStore
from .uwb import UwbStore, demo_loop, serial_loop, validate_serial_port


def _finite(text):
    try:
        value = float(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("숫자를 입력하십시오") from exc
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError("유한한 숫자를 입력하십시오")
    return value


def build_parser():
    parser = argparse.ArgumentParser(description="SmartCart LiDAR·UWB 읽기 전용 ROS 2 통합 센싱")
    parser.add_argument("--version", action="version", version=__version__)
    sub = parser.add_subparsers(dest="command", required=True)
    serve = sub.add_parser("serve", help="통합 GUI와 센서 수신 실행")
    serve.add_argument("--mode", choices=("ros", "demo", "replay"), default="ros")
    serve.add_argument("--uwb-mode", choices=("serial", "ros", "off"), default="serial")
    serve.add_argument("--serial-port", help="ESP32의 /dev/serial/by-id/... 또는 COM 번호")
    serve.add_argument("--baud", type=int, choices=(9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600), default=115200)
    serve.add_argument("--topic", default="/scan", help="기존 LaserScan 토픽")
    serve.add_argument("--uwb-topic", default="/smartcart/uwb")
    serve.add_argument("--host", default="127.0.0.1", help="기본값은 SSH 터널용 루프백")
    serve.add_argument("--port", type=int, default=8765)
    serve.add_argument("--allow-lan", action="store_true", help="신뢰하는 LAN에서만 외부 IPv4 바인딩 허용")
    serve.add_argument("--lidar-timeout", type=_finite, default=1.5)
    serve.add_argument("--uwb-timeout", type=_finite, default=.5)
    serve.add_argument("--uwb-yaw", type=_finite, default=0, help="UWB 장착 yaw, ROS 왼쪽 양수(deg)")
    serve.add_argument("--uwb-x", type=_finite, default=0, help="UWB 원점의 카트 전방 위치(m)")
    serve.add_argument("--uwb-y", type=_finite, default=0, help="UWB 원점의 카트 좌측 위치(m)")
    serve.add_argument("--tracking-mode", choices=("off", "ros", "replay"), default="off")
    serve.add_argument("--single-left", action="store_true", help="왼쪽 BU04 단독 읽기 전용 구성")
    serve.add_argument("--tracking-mounting-config", type=Path)
    serve.add_argument("--tracking-uwb-config", type=Path)
    serve.add_argument("--tracking-config", type=Path)
    serve.add_argument("--tracking-native-library", type=Path)
    serve.add_argument("--tracking-status-topic", default="/smartcart/uwb/dual/status")
    serve.add_argument("--tracking-input", type=Path, help="명시적 원시 통합 캡처 JSONL")
    serve.add_argument("--tracking-speed", type=_finite, default=.25)
    serve.add_argument("--tracking-loop", action="store_true", help="재생 종료 뒤 새 세션으로 반복")
    serve.add_argument("--tracking-ros-replay", action="store_true", help="재생 전용 ROS 토픽에만 발행")
    doctor = sub.add_parser("doctor", help="장치를 열지 않고 ROS·시리얼 실행 환경 점검")
    doctor.add_argument("--require-live", action="store_true", help="실측 필수 의존성이 없으면 종료 코드 2")
    return parser


def validate_args(args):
    if args.command != "serve":
        return args
    if args.single_left and (args.mode != "ros" or args.tracking_mode != "ros"):
        raise ValueError("왼쪽 단독 구성은 ROS 실측 추적에서만 지원합니다")
    if args.host == "localhost":
        args.host = "127.0.0.1"
    try:
        address = ipaddress.IPv4Address(args.host)
    except ipaddress.AddressValueError as exc:
        raise ValueError("--host는 IPv4 주소 또는 localhost여야 합니다") from exc
    if address.is_multicast or (not address.is_loopback and not args.allow_lan):
        raise ValueError("외부 바인딩은 --allow-lan 명시가 필요합니다. 기본 SSH 터널을 권장합니다")
    if not 1 <= args.port <= 65535:
        raise ValueError("포트는 1~65535 범위여야 합니다")
    if not .1 <= args.lidar_timeout <= 10 or not .05 <= args.uwb_timeout <= .5:
        raise ValueError("LiDAR 시간 제한은 0.1~10초, UWB는 0.05~0.5초 범위여야 합니다")
    if abs(args.uwb_yaw) > 360 or abs(args.uwb_x) > 10 or abs(args.uwb_y) > 10:
        raise ValueError("UWB 장착 보정 범위를 벗어났습니다")
    validate_topic(args.topic)
    validate_topic(args.uwb_topic)
    validate_topic(args.tracking_status_topic)
    if args.topic == args.uwb_topic:
        raise ValueError("LiDAR와 UWB 토픽은 달라야 합니다")
    if not .05 <= args.tracking_speed <= 4:
        raise ValueError("추적 재생 속도는 0.05~4 범위여야 합니다")
    if args.mode == "replay":
        if args.tracking_mode == "ros" or args.tracking_input is None:
            raise ValueError("통합 재생에는 --tracking-input을 명시해야 합니다")
        args.tracking_mode, args.uwb_mode = "replay", "off"
    elif args.tracking_mode == "replay" or args.tracking_input is not None or args.tracking_loop or args.tracking_ros_replay:
        raise ValueError("추적 재생 옵션은 --mode replay에서만 사용합니다")
    if args.tracking_mode == "ros" and args.mode != "ros":
        raise ValueError("ROS 추적 입력을 합성 모드와 섞을 수 없습니다")
    if args.tracking_mode == "ros" and args.tracking_status_topic in {args.topic, args.uwb_topic}:
        raise ValueError("추적 상태 토픽은 기존 입력 토픽과 달라야 합니다")
    if args.mode == "ros" and args.uwb_mode == "serial":
        validate_serial_port(args.serial_port)
    return args


def web_root() -> Path:
    local = Path(__file__).resolve().parent.parent / "web"
    if (local / "index.html").is_file():
        return local
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory("smartcart_sensing")) / "web"
    except (ImportError, LookupError) as exc:
        raise RuntimeError("GUI 정적 파일을 찾지 못했습니다") from exc
    if not (installed / "index.html").is_file():
        raise RuntimeError("설치된 GUI 정적 파일이 없습니다")
    return installed


def doctor() -> dict:
    names = ("rclpy", "sensor_msgs", "std_msgs", "geometry_msgs", "diagnostic_msgs", "serial")
    modules = {name: importlib.util.find_spec(name) is not None for name in names}
    devices = []
    if modules["serial"]:
        from serial.tools import list_ports
        devices = [{"device": port.device, "description": (port.description or "")[:160]}
                   for port in list_ports.comports()][:32]
    return {"schema": "smartcart.doctor.v1", "version": __version__,
            "system": platform.system(), "architecture": platform.machine(),
            "python": platform.python_version(), "ros_distro": os.environ.get("ROS_DISTRO"),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
            "ros_localhost_only": os.environ.get("ROS_LOCALHOST_ONLY", "기존 환경값 없음"),
            "dependencies": modules, "live_dependencies_ready": all(modules.values()),
            "serial_devices": devices, "hardware_opened": False}


def serve(args, *, ros_args=None):
    validate_args(args)
    if args.mode == "ros" or args.tracking_ros_replay:
        prerequisites = doctor()
        required = ("rclpy", "sensor_msgs", "std_msgs", "geometry_msgs", "diagnostic_msgs")
        if not all(prerequisites["dependencies"][name] for name in required):
            raise RuntimeError("ROS 2 환경을 source한 Pi의 /usr/bin/python3로 실행하십시오")
        if args.mode == "ros" and args.uwb_mode == "serial" and not prerequisites["dependencies"]["serial"]:
            raise RuntimeError("실측 시리얼 수신에는 Pi의 python3-serial 패키지가 필요합니다")
        os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    lidar = ScanStore(args.mode, args.lidar_timeout)
    uwb = UwbStore("demo" if args.mode == "demo" else args.uwb_mode,
                   args.uwb_timeout, lidar.stop, args.uwb_yaw, args.uwb_x, args.uwb_y)
    from smartcart_tracking.view import TrackingStore
    mounting = uwb_settings = tracking_settings = None
    if args.tracking_mode != "off":
        from smartcart_uwb.config import example_config_path, load_config
        from smartcart_uwb.estimation_config import load_estimation_config
        from smartcart_tracking.config import load_tracking_config
        config_directory = example_config_path().parent
        mounting = load_config(args.tracking_mounting_config or example_config_path())
        uwb_settings = load_estimation_config(args.tracking_uwb_config or config_directory / "uwb_estimation.example.json")
        tracking_settings = load_tracking_config(args.tracking_config or config_directory / "target_tracking.example.json")
    tracking = TrackingStore(mounting, tracking_settings, mode=args.tracking_mode, stop=lidar.stop)
    dual = None
    if args.mode == 'ros' and args.tracking_mode == 'ros':
        from .dual_uwb import DualUwbStore
        dual = DualUwbStore(mounting, uwb_settings, stop=lidar.stop, single_left=args.single_left)
    store = SensingStore(lidar, uwb, args.mode, tracking=tracking, dual_uwb=dual)
    if args.mode == "demo":
        workers = [threading.Thread(target=source_loop, args=(lidar,), daemon=True),
                   threading.Thread(target=demo_loop, args=(uwb,), daemon=True)]
    elif args.mode == "replay":
        from smartcart_tracking.service_runtime import replay_loop
        from smartcart_uwb.native import NativeParser
        factory = lambda: NativeParser(args.tracking_native_library)
        # 서버를 열기 전에 명시한 원시 파서와 캡처 종류를 확인한다.
        from smartcart_tracking.replay import read_records
        with NativeParser(args.tracking_native_library):
            capture = read_records(args.tracking_input)
            try:
                next(capture)
            except StopIteration as exc:
                raise ValueError("재생 입력에 레코드가 없습니다.") from exc
            finally:
                capture.close()
        workers = [threading.Thread(target=replay_loop, args=(lidar, tracking, mounting, uwb_settings,
            tracking_settings, args.tracking_input), kwargs={"speed": args.tracking_speed,
            "loop": args.tracking_loop, "parser_factory": factory}, daemon=True)]
        if args.tracking_ros_replay:
            from smartcart_tracking.ros_runtime import tracking_replay_ros_loop
            workers.append(threading.Thread(target=tracking_replay_ros_loop, args=(tracking,),
                                            kwargs={"ros_args": ros_args}, daemon=True))
    else:
        workers = []
        if args.tracking_mode == "ros":
            from smartcart_tracking.ros_runtime import tracking_ros_loop
            tracking_kwargs = {"scan_topic": args.topic, "status_topic": args.tracking_status_topic,
                               "ros_args": ros_args, "dual_uwb_store": dual}
            if args.single_left:
                tracking_kwargs["single_left"] = True
            workers.append(threading.Thread(target=tracking_ros_loop,
                args=(lidar, tracking, mounting, uwb_settings, tracking_settings),
                kwargs=tracking_kwargs, daemon=True))
        if args.uwb_mode != "off" or args.tracking_mode == "off":
            workers.append(threading.Thread(target=ros_loop, args=(lidar, uwb), kwargs={
                "topic": args.topic, "uwb_topic": args.uwb_topic,
                "subscribe_scan": args.tracking_mode == "off", "subscribe_uwb": args.uwb_mode == "ros",
                "publish_uwb": args.uwb_mode == "serial", "ros_args": ros_args}, daemon=True))
        if args.uwb_mode == "serial":
            workers.append(threading.Thread(target=serial_loop, args=(uwb, args.serial_port, args.baud), daemon=True))
    server = SensingHTTPServer((args.host, args.port), store, web_root(), allow_lan=args.allow_lan)
    for worker in workers:
        worker.start()
    print(f"SmartCart Sensing {__version__} | 읽기 전용 | {args.mode}", flush=True)
    print(f"GUI: http://{args.host}:{server.server_port} | LiDAR 1/2Hz · UWB 최대 10Hz", flush=True)
    if args.mode == "demo":
        print("합성 데모입니다. 실제 LiDAR·UWB를 연결하거나 장치를 제어하지 않습니다.", flush=True)
    elif args.mode == "replay":
        print("통합 캡처 재생입니다. 실측 입력·구동 허가가 아닙니다.", flush=True)
    previous = None
    if threading.current_thread() is threading.main_thread():
        def terminate(_signum, _frame):
            store.stop.set()
            threading.Thread(target=server.shutdown, daemon=True).start()
        previous = signal.signal(signal.SIGTERM, terminate)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        store.stop.set()
        server.server_close()
        for worker in workers:
            worker.join(timeout=3)
        if previous is not None:
            signal.signal(signal.SIGTERM, previous)
    return 0


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    ros_args = []
    if "--ros-args" in argv:
        try:
            from rclpy.utilities import remove_ros_args
            ros_args = list(argv)
            argv = remove_ros_args(args=["smartcart-sensing"] + argv)[1:]
        except ImportError:
            print("ROS 실행 인수에는 rclpy가 필요합니다", file=sys.stderr)
            return 2
    args = build_parser().parse_args(argv)
    try:
        if args.command == "doctor":
            report = doctor()
            print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
            return 2 if args.require_live and not report["live_dependencies_ready"] else 0
        return serve(args, ros_args=ros_args)
    except (ValueError, RuntimeError, OSError) as exc:
        print(f"센싱 실행 오류: {exc}", file=sys.stderr)
        return 2
