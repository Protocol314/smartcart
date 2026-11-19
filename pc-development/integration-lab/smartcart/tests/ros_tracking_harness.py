#!/usr/bin/env python3
"""합성 캡처 → 실제 격리 DDS → 운영 추적 어댑터 → DDS 구독 → HTTP GUI 검증.

물리 포트·센서·모터는 열지 않는다. ROS/live는 운영 전송 계약을 시험하는 표기다.
화면은 상시 합성 배너를 표시하며 기본 기록은 없다. --output은 좌표 없는 새 집계만 저장한다.
컨테이너에서는 --container-bind와 Docker의 127.0.0.1:호스트포트:컨테이너포트를 함께 사용한다.
"""
from __future__ import annotations

import argparse
from array import array
from collections import Counter
from contextlib import ExitStack
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from urllib.parse import urlsplit
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

SCAN_TOPIC = "/smartcart/synthetic_validation/scan"
STATUS_TOPIC = "/smartcart/synthetic_validation/dual_status"
MAX_CAPTURE_BYTES = 16 * 1024 * 1024
MAX_CAPTURE_RECORDS = 5000
MAX_HTTP_BYTES = 512 * 1024
TRACK_STATES = {"waiting", "acquiring", "tracking", "coasting", "ambiguous", "lost", "error"}
BANNER = ("합성 검증 · 실제 ROS 경로 시험 · 실측 아님 · 모터 구동 불가. "
          "아래 ROS/live 표시는 전송 경로이며 입력은 전부 합성입니다.")
BANNER_CSS = """/* 합성 검증 하네스에서만 제공하는 고정 안내다. */
body { padding-top:92px; }
#syntheticValidationBanner { position:fixed; inset:0 0 auto; z-index:99999;
  box-sizing:border-box; min-height:76px; padding:12px 18px; color:#fff4cb;
  background:#312500; border-bottom:2px solid #d9b440; font:14px/1.5 sans-serif;
  overflow-wrap:anywhere; }
#syntheticValidationBanner strong { display:block; }
@media(max-width:600px) { body { padding-top:126px; }
  #syntheticValidationBanner { min-height:112px; font-size:13px; } }
"""


def finite_number(text, minimum, maximum):
    value = float(text)
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise argparse.ArgumentTypeError("허용 범위의 유한한 숫자가 필요합니다")
    return value


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--seconds", type=lambda value: finite_number(value, 1, 3600), default=120)
    parser.add_argument("--speed", type=lambda value: finite_number(value, .05, 4), default=1.0,
                        help="합성 입력 도착 간격 배율. 운영 호스트 시계와 TTL은 실제 속도를 유지한다")
    parser.add_argument("--loop", action="store_true", help="명시한 경우에만 새 UWB epoch로 합성 입력 반복")
    parser.add_argument("--domain-id", type=int, default=231, help="합성 시험 전용 DDS 도메인(215~232)")
    parser.add_argument("--container-bind", action="store_true",
                        help="컨테이너 안에서만 0.0.0.0 바인딩. Docker 호스트 포트는 127.0.0.1에만 공개")
    parser.add_argument("--input", type=Path, default=ROOT / "samples" / "target_tracking.synthetic.jsonl")
    parser.add_argument("--mounting-config", type=Path, default=ROOT / "config" / "dual_uwb.d5.example.json")
    parser.add_argument("--uwb-config", type=Path, default=ROOT / "config" / "uwb_estimation.synthetic.json")
    parser.add_argument("--tracking-config", type=Path, default=ROOT / "config" / "target_tracking.synthetic.json")
    parser.add_argument("--native-library", type=Path)
    parser.add_argument("--output", type=Path, help="선택한 새 JSON 파일에 좌표·태그·원시 입력 없는 집계 저장")
    parser.add_argument("--plan-only", action="store_true", help="ROS·소켓·파일 출력 없이 시험 경계만 확인")
    return parser


def synthetic_records(path):
    """운영 캡처와 섞지 않도록 합성 출처·시각·입력 크기를 전부 사전 검사한다."""
    from smartcart_tracking.replay import read_records
    path = Path(path)
    if path.stat().st_size > MAX_CAPTURE_BYTES:
        raise ValueError("합성 시험 입력은 16MiB 이하여야 합니다")
    records, previous = [], -1
    for record in read_records(path):
        if len(records) >= MAX_CAPTURE_RECORDS:
            raise ValueError("합성 시험 레코드는 5000개 이하여야 합니다")
        if record["source"] != "synthetic" or record["at_ns"] < previous:
            raise ValueError("비감소 시각의 합성 입력만 시험할 수 있습니다")
        previous = record["at_ns"]
        records.append(record)
    if not records or not any(record["event"] == "lidar_scan" for record in records):
        raise ValueError("합성 LiDAR 스캔이 포함된 입력이 필요합니다")
    return records


def banner_html(document):
    """사용자 값을 삽입하지 않고 기존 스크립트·스타일 CSP를 그대로 유지한다."""
    if "<body>" not in document or "</head>" not in document:
        raise ValueError("시험 GUI HTML 구조를 확인할 수 없습니다")
    document = document.replace("<title>SmartCart 통합 센싱</title>", "<title>합성 검증 · SmartCart ROS</title>", 1)
    document = document.replace("</head>", '<link rel="stylesheet" href="/synthetic-validation.css">\n</head>', 1)
    notice = ('<aside id="syntheticValidationBanner" role="note" aria-label="합성 검증 안내">'
              '<strong>합성 검증 · 실제 ROS 경로 시험 · 실측 아님 · 모터 구동 불가</strong>'
              '아래 ROS/live 표시는 전송 경로이며 입력은 전부 합성입니다.</aside>')
    return document.replace("<body>", "<body>\n" + notice, 1)


class Metrics:
    """좌표·식별자·입력 본문 대신 유한 개수와 상태 이름만 보관한다."""
    def __init__(self):
        self.lock = threading.RLock()
        self.counts = Counter()
        self.dds_states, self.http_states, self.errors = set(), set(), set()
        self.iterations = 0

    def count(self, name):
        with self.lock:
            self.counts[name] += 1

    def state(self, source, value):
        if value not in TRACK_STATES:
            raise ValueError("지원하지 않는 추적 상태입니다")
        with self.lock:
            (self.dds_states if source == "dds" else self.http_states).add(value)

    def error(self, code):
        with self.lock:
            self.errors.add(code)

    def summary(self):
        with self.lock:
            return {"original_source": "synthetic", "transport_source_mode": "live",
                    "drive_authorized": False, "physical_hardware_opened": False,
                    "iterations": self.iterations, "counts": dict(self.counts),
                    "dds_track_states_seen": sorted(self.dds_states),
                    "http_track_states_seen": sorted(self.http_states), "errors": sorted(self.errors)}


def create_http_server(address, store, metrics, *, container_bind=False):
    from smartcart_sensing.server import SensingHandler, SensingHTTPServer

    class SyntheticHandler(SensingHandler):
        def do_GET(self):
            path = urlsplit(self.path).path
            if path in ("/", "/index.html", "/synthetic-validation.css"):
                if path.endswith(".css"):
                    payload, content_type = BANNER_CSS.encode("utf-8"), "text/css; charset=utf-8"
                else:
                    document = (self.server.web_root / "index.html").read_text(encoding="utf-8")
                    payload, content_type = banner_html(document).encode("utf-8"), "text/html; charset=utf-8"
                self.send_response(200)
                self.write_headers(content_type, len(payload))
                self.wfile.write(payload)
                return
            if path == "/api/synthetic-validation":
                self.json_response(200, {"schema": "smartcart.synthetic_validation.v1", **metrics.summary()})
                return
            super().do_GET()

    server = SensingHTTPServer(address, store, ROOT / "web", allow_lan=container_bind)
    server.RequestHandlerClass = SyntheticHandler
    return server


def http_monitor(server, metrics, stop):
    """실제 HTTP 전송 뒤에도 출처·구동 금지와 위치 만료 조건이 유지되는지 본다."""
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with urlopen(base + "/", timeout=3) as response:
            html = response.read(MAX_HTTP_BYTES + 1)
            if (len(html) > MAX_HTTP_BYTES or "합성 검증".encode() not in html
                    or b"syntheticValidationBanner" not in html
                    or "script-src 'self'" not in response.headers.get("Content-Security-Policy", "")):
                raise ValueError("합성 안내 또는 CSP가 없습니다")
        metrics.count("http_banner_and_csp_verified")
        while not stop.is_set():
            with urlopen(base + "/api/tracking", timeout=3) as response:
                payload = response.read(MAX_HTTP_BYTES + 1)
            if len(payload) > MAX_HTTP_BYTES:
                raise ValueError("HTTP 추적 응답 상한 초과")
            value = json.loads(payload, parse_constant=lambda _: (_ for _ in ()).throw(ValueError()))
            if (value.get("schema") != "smartcart.tracking_view.v1" or value.get("mode") != "ros"
                    or value.get("source_mode") != "live" or value.get("drive_authorized") is not False
                    or value.get("provisional") is not True):
                raise ValueError("ROS 경로의 읽기 전용 표시 계약이 달라졌습니다")
            metrics.count("http_tracking_responses")
            result = value.get("result")
            if result is not None:
                track = result["track"]
                metrics.state("http", track["state"])
                if track["valid"]:
                    if not value["valid"] or value["remaining_ms"] <= 0:
                        raise ValueError("만료된 측정 위치가 HTTP에 남았습니다")
                    metrics.count("http_valid_targets")
                elif track["position_xy_m"] is not None or track["velocity_xy_mps"] is not None:
                    raise ValueError("무효 추적에 측정 위치가 남았습니다")
            stop.wait(.1)
    except Exception:
        metrics.error("http_contract_error")
        stop.set()


def run(args, records, metrics):
    # 도메인·검색 범위는 이 독립 시험 프로세스에만 적용한다.
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
    except ImportError as error:
        raise RuntimeError("ROS 2 Humble 환경을 source한 Linux Python이 필요합니다") from error

    from smartcart_radar.core import ScanStore
    from smartcart_sensing.service import SensingStore
    from smartcart_sensing.uwb import UwbStore
    from smartcart_tracking.config import load_tracking_config
    from smartcart_tracking.ros_runtime import LIVE_TOPICS, _result, decode_status, tracking_ros_loop
    from smartcart_tracking.view import TrackingStore
    from smartcart_uwb.collector import DualCollector
    from smartcart_uwb.config import load_config
    from smartcart_uwb.estimation_config import load_estimation_config
    from smartcart_uwb.native import NativeParser

    mounting = load_config(args.mounting_config)
    uwb_settings = load_estimation_config(args.uwb_config)
    settings = load_tracking_config(args.tracking_config)
    lidar = ScanStore("ros")
    stop = lidar.stop
    display = TrackingStore(mounting, settings, mode="ros", stop=stop)
    # GUI는 아래 운영 작업자의 직접 메모리가 아니라 실제 DDS status 구독 결과를 사용한다.
    runtime_store = TrackingStore(mounting, settings, mode="ros", stop=stop)
    store = SensingStore(lidar, UwbStore("off", stop=stop), mode="ros", tracking=display)
    context = node = executor = server = None
    workers = []
    with ExitStack() as stack:
        collector = stack.enter_context(DualCollector(mounting,
            parser_factory=lambda: NativeParser(args.native_library), source_mode="live"))
        try:
            context = Context()
            rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
            node = Node("smartcart_synthetic_validation_" + str(os.getpid()), context=context,
                        use_global_arguments=False, enable_rosout=False, start_parameter_services=False)
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
            scan_publisher = node.create_publisher(LaserScan, SCAN_TOPIC, qos)
            status_publisher = node.create_publisher(String, STATUS_TOPIC, qos)

            def receive(name, message):
                try:
                    value = decode_status(message.data)
                    metrics.count("dds_received_" + name)
                    if name == "status":
                        result = _result(value, "live")
                        display.publish(result, now_ns=time.monotonic_ns())
                        metrics.state("dds", result["track"]["state"])
                    else:
                        if value.get("drive_authorized") is not False or value.get("source_mode") != "live":
                            raise ValueError("실제 DDS 출력의 출처·구동 금지 계약이 다릅니다")
                        if name == "target" and value.get("valid"):
                            metrics.count("dds_valid_targets")
                except Exception:
                    display.set_error("synthetic_dds_contract_error")
                    metrics.error("dds_contract_error")
                    stop.set()

            subscriptions = [node.create_subscription(String, topic,
                lambda message, name=name: receive(name, message), qos) for name, topic in LIVE_TOPICS.items()]
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            production = threading.Thread(target=tracking_ros_loop,
                args=(lidar, runtime_store, mounting, uwb_settings, settings),
                kwargs={"scan_topic": SCAN_TOPIC, "status_topic": STATUS_TOPIC}, daemon=True)
            workers.append(production)
            production.start()
            discovery_deadline = time.monotonic() + 10
            while not stop.is_set() and time.monotonic() < discovery_deadline:
                executor.spin_once(timeout_sec=.02)
                if (node.count_subscribers(SCAN_TOPIC) and node.count_subscribers(STATUS_TOPIC)
                        and all(node.count_publishers(topic) for topic in LIVE_TOPICS.values())):
                    metrics.count("dds_discovery_verified")
                    break
            else:
                raise RuntimeError("격리 DDS 토픽 검색을 완료하지 못했습니다")
            if not subscriptions:
                raise RuntimeError("실제 DDS 출력 구독을 만들지 못했습니다")

            host = "0.0.0.0" if args.container_bind else "127.0.0.1"
            server = create_http_server((host, args.port), store, metrics, container_bind=args.container_bind)
            workers.extend((threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .05}, daemon=True),
                            threading.Thread(target=http_monitor, args=(server, metrics, stop), daemon=True)))
            for worker in workers[1:]:
                worker.start()
            print(BANNER, flush=True)
            print(f"CUA URL: http://127.0.0.1:{server.server_port}/ | DDS domain={args.domain_id}", flush=True)
            if args.container_bind:
                print("컨테이너 0.0.0.0 수신: Docker 호스트는 -p 127.0.0.1:포트:포트만 사용합니다.", flush=True)

            def publish_status():
                message = String()
                message.data = json.dumps(collector.snapshot(time.monotonic_ns()), ensure_ascii=False,
                                          allow_nan=False, separators=(",", ":"))
                status_publisher.publish(message)
                metrics.count("dds_published_status")

            def publish_record(record):
                event = record["event"]
                now = time.monotonic_ns()
                if event == "uwb_bytes":
                    collector.feed(record["sensor_id"], bytes.fromhex(record["data_hex"]), now,
                                   source_rx_ns=record["at_ns"])
                elif event == "uwb_disconnect":
                    collector.disconnected(record["sensor_id"], "synthetic_disconnect")
                elif event == "uwb_reconnect":
                    collector.reconnect(record["sensor_id"])
                elif event == "lidar_scan":
                    message = LaserScan()
                    message.header.frame_id = mounting.lidar.frame_id
                    message.header.stamp = node.get_clock().now().to_msg()
                    for name, value in record["scan"].items():
                        setattr(message, name, (array("f", (math.inf if item is None else float(item) for item in value))
                                               if name == "ranges" else float(value)))
                    message.intensities = array("f")
                    scan_publisher.publish(message)
                    metrics.count("dds_published_scan")
                # ROS LaserScan에 단절 메시지는 없다. 캡처의 LiDAR 단절은 도착 중지와 기존 TTL로 평가한다.
                publish_status()
                metrics.count("input_events")

            started = cycle_start = time.monotonic()
            deadline, index, next_cycle = started + args.seconds, 0, None
            metrics.iterations = 1
            while not stop.is_set() and context.ok() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.002)
                now = time.monotonic()
                if index < len(records):
                    if now - cycle_start >= records[index]["at_ns"] / args.speed / 1e9:
                        publish_record(records[index])
                        index += 1
                        if index == len(records):
                            for sensor_id in ("uwb_left", "uwb_right"):
                                collector.disconnected(sensor_id, "synthetic_capture_ended")
                            publish_status()
                            next_cycle = now + .75 if args.loop else None
                elif next_cycle is not None and now >= next_cycle:
                    # 같은 수집기 세션의 명시적 재연결로 epoch와 장치 순번 기준만 새로 시작한다.
                    for sensor_id in ("uwb_left", "uwb_right"):
                        collector.reconnect(sensor_id)
                    publish_status()
                    index, cycle_start, next_cycle = 0, now, None
                    with metrics.lock:
                        metrics.iterations += 1
                if runtime_store.snapshot()["state"] == "error":
                    raise RuntimeError("운영 ROS 추적 작업자가 오류 상태입니다")
            summary = metrics.summary()
            for name in ("clusters", "target", "status"):
                if not summary["counts"].get("dds_received_" + name):
                    metrics.error("missing_dds_" + name)
            if not summary["counts"].get("dds_valid_targets"):
                metrics.error("missing_dds_valid_target")
            if not summary["counts"].get("http_valid_targets"):
                metrics.error("missing_http_valid_target")
        finally:
            stop.set()
            if server is not None:
                # 시작 전에 초기화가 실패하면 serve_forever가 없어 shutdown이 기다릴 수 있다.
                if len(workers) > 1 and workers[1].ident is not None:
                    server.shutdown()
                server.server_close()
            for worker in workers:
                if worker.ident is not None:
                    worker.join(timeout=4)
                if worker.is_alive():
                    metrics.error("worker_not_stopped")
            for code, cleanup in (
                ("executor_cleanup_error", lambda: executor.shutdown(timeout_sec=1) if executor is not None else None),
                ("node_cleanup_error", lambda: node.destroy_node() if node is not None else None),
                ("context_cleanup_error", lambda: context.shutdown() if context is not None and context.ok() else None),
            ):
                try:
                    cleanup()
                except Exception:
                    metrics.error(code)


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535 or not 215 <= args.domain_id <= 232:
        parser.error("포트는 0~65535, 독립 시험 도메인은 215~232여야 합니다")
    if args.output is not None and args.output.exists():
        parser.error("기존 검사 집계 파일을 덮어쓰지 않습니다")
    if args.plan_only:
        print(json.dumps({"schema": "smartcart.ros_tracking_plan.v1", "original_source": "synthetic",
            "drive_authorized": False, "physical_hardware_opened": False, "browser_checks_run": False,
            "domain_id": args.domain_id, "input_topics": [SCAN_TOPIC, STATUS_TOPIC],
            "gui_feed": "real_dds_status_subscription", "notice": BANNER}, ensure_ascii=False))
        return 0
    metrics = Metrics()
    try:
        records = synthetic_records(args.input)
        run(args, records, metrics)
    except KeyboardInterrupt:
        metrics.count("user_requested_stop")
    except (ValueError, RuntimeError, OSError) as error:
        metrics.error("harness_" + type(error).__name__)
        print("합성 ROS 시험 실행 오류: " + str(error), file=sys.stderr)
    report = {"schema": "smartcart.ros_tracking_harness.v1", **metrics.summary(),
              "domain_id": args.domain_id, "browser_checks_run": False,
              "gui_feed": "real_dds_status_subscription",
              "lidar_disconnect_semantics": "arrival_silence_and_existing_ttl",
              "not_tested": ["브라우저 DOM·Canvas 시각 판정", "실물 센서·모터", "Pi ARM64 성능"]}
    if args.output is not None:
        try:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as handle:
                json.dump(report, handle, ensure_ascii=False, indent=2, allow_nan=False)
                handle.write("\n")
        except OSError:
            metrics.error("output_write_error")
            report["errors"] = metrics.summary()["errors"]
    print(json.dumps(report, ensure_ascii=False, allow_nan=False), flush=True)
    return int(bool(report["errors"]))


if __name__ == "__main__":
    raise SystemExit(main())
