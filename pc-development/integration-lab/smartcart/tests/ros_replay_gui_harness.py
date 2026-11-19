#!/usr/bin/env python3
"""합성 원시 재생 → 운영 CLI → 실제 replay DDS 3개 토픽·같은 HTTP GUI 검증.

Linux/ROS 2 Humble에서 실행한다. 물리 장치·모터를 열지 않으며 운영 GUI와 REPLAY
배지는 수정하지 않는다. --output에 지정한 새 파일에는 개수·상태·고정 오류 코드만 쓴다.
컨테이너의 --container-bind 사용 시 Docker 호스트 포트는 127.0.0.1에만 공개한다.
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
from urllib.error import URLError
from urllib.request import ProxyHandler, Request, build_opener

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
DOMAIN_ID = 232
MAX_HTTP_BYTES = 512 * 1024
MAX_CHILD_BYTES = 65536
TRACK_STATES = {"waiting", "acquiring", "tracking", "coasting", "ambiguous", "lost", "error"}
VIEW_STATES = {"waiting", "running", "stale", "ended", "error", "off"}


def duration(text):
    try:
        value = float(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("검사 시간은 1~600초의 유한한 수여야 합니다") from error
    if not math.isfinite(value) or not 1 <= value <= 600:
        raise argparse.ArgumentTypeError("검사 시간은 1~600초의 유한한 수여야 합니다")
    return value


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--seconds", type=duration, default=30)
    parser.add_argument("--container-bind", action="store_true",
                        help="컨테이너 자식만 0.0.0.0 수신. Docker 호스트는 127.0.0.1로만 공개")
    parser.add_argument("--input", type=Path, default=ROOT / "samples/target_tracking.synthetic.jsonl")
    parser.add_argument("--mounting-config", type=Path, default=ROOT / "config/dual_uwb.d5.example.json")
    parser.add_argument("--uwb-config", type=Path, default=ROOT / "config/uwb_estimation.synthetic.json")
    parser.add_argument("--tracking-config", type=Path, default=ROOT / "config/target_tracking.synthetic.json")
    parser.add_argument("--native-library", type=Path)
    parser.add_argument("--output", type=Path, help="좌표·식별자·원시 본문이 없는 새 JSON 집계 파일")
    parser.add_argument("--plan-only", action="store_true", help="ROS·소켓·자식 실행·출력 파일 없이 경계만 확인")
    return parser


def command(args):
    values = [sys.executable, "-X", "utf8", str(ROOT / "sensing.py"), "serve", "--mode", "replay",
              "--tracking-input", str(args.input.resolve()), "--tracking-ros-replay",
              "--tracking-speed", ".25", "--tracking-loop",
              "--tracking-mounting-config", str(args.mounting_config.resolve()),
              "--tracking-uwb-config", str(args.uwb_config.resolve()),
              "--tracking-config", str(args.tracking_config.resolve()),
              "--host", "0.0.0.0" if args.container_bind else "127.0.0.1", "--port", str(args.port)]
    if args.container_bind:
        values.append("--allow-lan")
    if args.native_library is not None:
        values.extend(("--tracking-native-library", str(args.native_library.resolve())))
    return values


class Metrics:
    """합성 대응 키는 작은 메모리에만 두고 보고서에는 개수와 상태만 남긴다."""
    def __init__(self):
        self.lock = threading.RLock()
        self.counts, self.errors = Counter(), set()
        self.states = {"dds": set(), "http": set(), "view": set()}
        self.recent = deque(maxlen=32)

    def count(self, name, amount=1):
        with self.lock:
            self.counts[name] += amount

    def error(self, code):
        with self.lock:
            self.errors.add(code)

    def state(self, source, value):
        if value not in (VIEW_STATES if source == "view" else TRACK_STATES):
            raise ValueError("알 수 없는 추적 상태입니다")
        with self.lock:
            self.states[source].add(value)

    def publisher_count(self, name, value):
        with self.lock:
            self.counts["max_publishers_" + name] = max(self.counts["max_publishers_" + name], value)

    def valid(self, source, track):
        key = (track["track_id"], track["track_sequence"], track["reference_monotonic_ns"])
        now = time.monotonic()
        with self.lock:
            self.counts[source + "_valid_targets"] += 1
            if source == "dds":
                self.recent.append((now, key))
            elif any(now - at < 1 and prior == key for at, prior in self.recent):
                self.counts["matching_dds_http_valid_targets"] += 1

    def report(self):
        with self.lock:
            return {"schema": "smartcart.ros_replay_gui_harness.v1", "original_source": "synthetic",
                    "source_mode": "replay", "drive_authorized": False,
                    "physical_hardware_opened": False, "browser_checks_run": False,
                    "counts": dict(self.counts), "states": {key: sorted(value) for key, value in self.states.items()},
                    "errors": sorted(self.errors)}


def parse_json(raw):
    from smartcart_uwb.config import _reject_constant, _unique_object
    return json.loads(raw, parse_constant=_reject_constant, object_pairs_hook=_unique_object)


def check_track(track):
    if (type(track) is not dict or track.get("schema") != "smartcart.target_track.v1"
            or track.get("source_mode") != "replay" or track.get("drive_authorized") is not False
            or track.get("provisional") is not True or type(track.get("valid")) is not bool):
        raise ValueError("재생 대상 출처·구동 금지 계약이 잘못되었습니다")
    if track["valid"]:
        position = track.get("position_xy_m")
        if (track.get("state") != "tracking" or type(position) is not list or len(position) != 2
                or any(type(value) not in (int, float) or not -100 <= value <= 100
                       or not math.isfinite(value) for value in position)):
            raise ValueError("유효 재생 대상의 위치가 잘못되었습니다")
        for name in ("track_id", "track_sequence", "reference_monotonic_ns"):
            if type(track.get(name)) is not int or track[name] < 1:
                raise ValueError("유효 재생 대상의 대응 키가 잘못되었습니다")
    elif track.get("position_xy_m") is not None or track.get("velocity_xy_mps") is not None:
        raise ValueError("무효 재생 대상에 측정 위치가 남아 있습니다")


def validate_synthetic_input(path):
    from smartcart_tracking.replay import read_records
    if not path.is_file() or path.stat().st_size > 16 * 1024 * 1024:
        raise ValueError("합성 입력은 16MiB 이하의 일반 파일이어야 합니다")
    count, previous, scans = 0, -1, 0
    for record in read_records(path):
        count += 1
        if count > 5000 or record["source"] != "synthetic" or record["at_ns"] < previous:
            raise ValueError("5000개 이하의 비감소 합성 레코드만 허용합니다")
        previous = record["at_ns"]
        scans += record["event"] == "lidar_scan"
    if not count or not scans:
        raise ValueError("비어 있지 않은 합성 LiDAR 입력이 필요합니다")
    return count


def http_monitor(args, metrics, stop):
    opener = build_opener(ProxyHandler({}))
    base = f"http://127.0.0.1:{args.port}"

    def get(path):
        with opener.open(Request(base + path, headers={"Origin": base}), timeout=1) as response:
            raw = response.read(MAX_HTTP_BYTES + 1)
            if len(raw) > MAX_HTTP_BYTES:
                raise ValueError("HTTP 응답 크기 상한을 넘었습니다")
            return raw, response.headers

    try:
        deadline = time.monotonic() + 10
        while not stop.is_set():
            try:
                document, headers = get("/")
                break
            except (OSError, URLError):
                metrics.count("http_startup_retries")
                if time.monotonic() >= deadline:
                    raise RuntimeError("운영 CLI HTTP 시작 시간 상한을 넘었습니다")
                stop.wait(.1)
        else:
            return
        script, _ = get("/js/tracking.js")
        app_script, _ = get("/js/app.js")
        if (b'id="trackingPanel"' not in document or b"js/tracking.js" not in document
                or b"syntheticValidationBanner" in document or "재생".encode("utf-8") not in script
                or b"REPLAY" not in app_script
                or "script-src 'self'" not in headers.get("Content-Security-Policy", "")):
            raise ValueError("운영 GUI의 추적 자산·REPLAY 배지·CSP를 확인하지 못했습니다")
        metrics.count("production_gui_assets_verified")
        while not stop.is_set():
            raw, _ = get("/api/tracking")
            envelope = parse_json(raw)
            if (type(envelope) is not dict or envelope.get("schema") != "smartcart.tracking_view.v1"
                    or envelope.get("mode") != "replay" or envelope.get("source_mode") != "replay"
                    or envelope.get("drive_authorized") is not False or envelope.get("provisional") is not True):
                raise ValueError("HTTP 추적이 같은 재생 출처가 아닙니다")
            metrics.count("http_tracking_responses")
            metrics.state("view", envelope.get("state"))
            if envelope["state"] == "error":
                raise RuntimeError("운영 CLI의 재생 작업자가 오류 상태입니다")
            result = envelope.get("result")
            if result is not None:
                for item in (result["uwb_estimate"], result["lidar_clusters"], result["track"]):
                    if item.get("source_mode") != "replay" or item.get("drive_authorized") is not False:
                        raise ValueError("HTTP 재생 결과에 다른 출처가 섞였습니다")
                track = result["track"]
                check_track(track)
                metrics.state("http", track["state"])
                if track["valid"]:
                    if envelope.get("valid") is not True or envelope.get("remaining_ms", 0) <= 0:
                        raise ValueError("만료된 재생 측정이 HTTP에 남았습니다")
                    metrics.valid("http", track)
            stop.wait(.1)
    except Exception as error:
        metrics.error("http_contract_error")
        print("재생 HTTP 검사 오류: " + str(error), file=sys.stderr, flush=True)
        stop.set()


def run(args, metrics):
    if not sys.platform.startswith("linux") or os.environ.get("ROS_DISTRO") != "humble":
        raise RuntimeError("source된 Linux ROS 2 Humble 환경이 필요합니다")
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from smartcart_tracking.ros_runtime import LIVE_TOPICS, REPLAY_TOPICS, _result, decode_status

    os.environ["ROS_DOMAIN_ID"], os.environ["ROS_LOCALHOST_ONLY"] = str(DOMAIN_ID), "1"
    metrics.count("validated_synthetic_records", validate_synthetic_input(args.input))
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", args.port))
    context = node = executor = child = None
    monitor = drain = None
    stop = threading.Event()
    child_output = bytearray()
    forbidden = {"live_" + name: topic for name, topic in LIVE_TOPICS.items()}
    forbidden.update(cmd_vel="/cmd_vel", cmd_vel_unstamped="/cmd_vel_unstamped", tf="/tf", tf_static="/tf_static")

    def graph_check():
        for name, topic in forbidden.items():
            count = node.count_publishers(topic)
            metrics.publisher_count(name, count)
            if count:
                metrics.error("forbidden_publisher_" + name)
                stop.set()
        metrics.count("forbidden_publisher_checks")

    try:
        context = Context()
        rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = Node("smartcart_production_replay_probe_" + str(os.getpid()), context=context,
                    use_global_arguments=False, enable_rosout=False, start_parameter_services=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)

        def receive(name, message):
            try:
                value = decode_status(message.data)
                if value.get("source_mode") != "replay" or value.get("drive_authorized") is not False:
                    raise ValueError("재생 DDS 출처·구동 금지 계약이 다릅니다")
                metrics.count("dds_received_" + name)
                if name == "status":
                    if value.get("schema") == "smartcart.tracking_view.v1":
                        if value.get("result") is not None or value.get("valid") is not False:
                            raise ValueError("좌표 없는 재생 상태가 잘못되었습니다")
                        if value.get("state") == "error":
                            raise RuntimeError("재생 ROS 작업자 오류가 전달됐습니다")
                        metrics.state("view", value.get("state"))
                    else:
                        result = _result(value, "replay")
                        check_track(result["track"])
                        metrics.state("dds", result["track"]["state"])
                        if result["track"]["valid"]:
                            metrics.count("dds_valid_full_status")
                elif name == "target":
                    check_track(value)
                    metrics.state("dds", value["state"])
                    if value["valid"]:
                        metrics.valid("dds", value)
                elif (value.get("schema") != "smartcart.lidar_clusters.v1"
                      or type(value.get("clusters")) is not list or len(value["clusters"]) > 128
                      or (value.get("valid") is False and value["clusters"])):
                    raise ValueError("재생 DDS 군집 계약이 잘못되었습니다")
            except Exception as error:
                metrics.error("dds_contract_error")
                print("재생 DDS 검사 오류: " + str(error), file=sys.stderr, flush=True)
                stop.set()

        for name, topic in REPLAY_TOPICS.items():
            node.create_subscription(String, topic, lambda message, name=name: receive(name, message), qos)
        graph_check()
        if stop.is_set():
            raise RuntimeError("격리 도메인에 금지 발행자가 이미 있습니다")
        child = subprocess.Popen(command(args), cwd=ROOT, env=dict(os.environ), stdin=subprocess.DEVNULL,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
        metrics.count("production_cli_started")

        def read_child():
            try:
                while chunk := child.stdout.read(4096):
                    remaining = MAX_CHILD_BYTES - len(child_output)
                    child_output.extend(chunk[:max(0, remaining)])
                    metrics.count("child_output_bytes", len(chunk))
                    if len(chunk) > remaining:
                        metrics.error("child_output_overflow")
                        stop.set()
                        return
            except OSError:
                metrics.error("child_output_read_error")
                stop.set()

        drain = threading.Thread(target=read_child, name="replay-cli-output", daemon=True)
        drain.start()
        monitor = threading.Thread(target=http_monitor, args=(args, metrics, stop), name="replay-http-monitor", daemon=True)
        monitor.start()
        print("합성 원시 재생 · 운영 CLI · 실제 replay DDS/HTTP 검사 · 실물·모터 없음", flush=True)
        print(f"CUA URL: http://127.0.0.1:{args.port}/ | REPLAY | DDS domain={DOMAIN_ID}", flush=True)
        deadline, next_graph = time.monotonic() + args.seconds, 0.0
        while not stop.is_set() and context.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if child.poll() is not None:
                metrics.error("production_cli_exited_early")
                break
            if time.monotonic() >= next_graph:
                graph_check()
                next_graph = time.monotonic() + .25
        if not context.ok():
            metrics.error("dds_context_stopped_early")
        report = metrics.report()
        for name in ("clusters", "target", "status"):
            if not report["counts"].get("dds_received_" + name):
                metrics.error("missing_dds_" + name)
        for name in ("dds_valid_targets", "dds_valid_full_status", "http_valid_targets",
                     "matching_dds_http_valid_targets", "production_gui_assets_verified"):
            if not report["counts"].get(name):
                metrics.error("missing_" + name)
    finally:
        stop.set()
        if monitor is not None:
            monitor.join(timeout=2)
            if monitor.is_alive():
                metrics.error("http_monitor_not_stopped")
        if child is not None:
            try:
                if child.poll() is None:
                    try:
                        child.send_signal(signal.SIGTERM)
                        metrics.count("production_cli_sigterm_sent")
                    except ProcessLookupError:
                        # poll 직후 자식이 끝난 경합도 wait로 회수하고 나머지 정리를 계속한다.
                        pass
                    try:
                        child.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        metrics.error("production_cli_sigterm_timeout")
                        child.kill()
                        child.wait(timeout=2)
            except Exception as error:
                metrics.error("production_cli_cleanup_error")
                print("운영 CLI 종료 오류: " + str(error), file=sys.stderr)
            if child.returncode != 0:
                metrics.error("production_cli_nonzero_exit")
            else:
                metrics.count("production_cli_graceful_exit")
        if drain is not None:
            drain.join(timeout=2)
            if drain.is_alive():
                metrics.error("child_output_reader_not_stopped")
        if child is not None and child.stdout is not None:
            child.stdout.close()
        if executor is not None and node is not None:
            try:
                deadline = time.monotonic() + 4
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.05)
                    if all(node.count_publishers(topic) == 0 for topic in REPLAY_TOPICS.values()):
                        metrics.count("replay_publishers_removed_after_exit")
                        break
                else:
                    metrics.error("replay_publishers_not_removed")
                graph_check()
            except Exception as error:
                metrics.error("dds_shutdown_check_error")
                print("재생 DDS 종료 검사 오류: " + str(error), file=sys.stderr)
        for name, cleanup in (("executor", lambda: executor.shutdown(timeout_sec=1) if executor is not None else None),
                              ("node", lambda: node.destroy_node() if node is not None else None),
                              ("context", lambda: context.shutdown() if context is not None and context.ok() else None)):
            try:
                cleanup()
            except Exception:
                metrics.error(name + "_cleanup_error")
        if metrics.report()["errors"] and child_output:
            print("운영 CLI의 제한된 진단 출력:\n" + child_output.decode("utf-8", errors="replace"), file=sys.stderr)


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("포트는 1~65535여야 합니다")
    if args.output is not None and (args.output.exists() or args.output.is_symlink()):
        parser.error("기존 집계 파일을 덮어쓰지 않습니다")
    if args.plan_only:
        print(json.dumps({"schema": "smartcart.ros_replay_gui_plan.v1", "original_source": "synthetic",
            "source_mode": "replay", "domain_id": DOMAIN_ID, "production_cli": True,
            "physical_hardware_opened": False, "drive_authorized": False, "gui_modified": False,
            "container_bind": args.container_bind, "seconds": args.seconds}, ensure_ascii=False))
        return 0
    metrics = Metrics()
    previous_sigterm = None
    if threading.current_thread() is threading.main_thread():
        def terminate(_signum, _frame):
            # 하네스 자체의 종료 요청도 자식·DDS의 finally 정리를 통과시킨다.
            raise KeyboardInterrupt
        previous_sigterm = signal.signal(signal.SIGTERM, terminate)
    try:
        run(args, metrics)
    except KeyboardInterrupt:
        metrics.error("harness_interrupted")
    except Exception as error:
        metrics.error("harness_" + type(error).__name__)
        print("재생 운영 CLI 검사 오류: " + str(error), file=sys.stderr)
    finally:
        if previous_sigterm is not None:
            signal.signal(signal.SIGTERM, previous_sigterm)
    report = metrics.report()
    if args.output is not None:
        try:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as handle:
                json.dump(report, handle, ensure_ascii=False, indent=2, allow_nan=False)
                handle.write("\n")
        except OSError as error:
            metrics.error("output_write_error")
            report = metrics.report()
            print("검사 집계 저장 오류: " + str(error), file=sys.stderr)
    print(json.dumps(report, ensure_ascii=False, allow_nan=False), flush=True)
    return int(bool(report["errors"]))


if __name__ == "__main__":
    raise SystemExit(main())
