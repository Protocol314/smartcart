#!/usr/bin/env python3
"""실물·ROS 없이 C 파서, 합성 고장·복구, 실제 재생 HTTP와 종료를 검사한다."""
from __future__ import annotations

import argparse
from collections import Counter
import http.client
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from smartcart_field.reporting import encoded_report, write_report


FORBIDDEN_REPORT_KEYS = frozenset({
    "position_xy_m", "velocity_xy_mps", "predicted_position_xy_m", "tag_address",
    "track_id", "ranges", "data_hex", "centroid_xy_m", "translation_m",
    "port", "expected_device_identity", "result", "lidar_transform",
})


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def summary_only(value):
    """저장 직전 깊은 구조 전체를 검사해 위치·식별자·원시 입력의 혼입을 거부한다."""
    if isinstance(value, dict):
        require(not FORBIDDEN_REPORT_KEYS.intersection(value), "집계에 저장 금지 필드가 있습니다")
        for item in value.values():
            summary_only(item)
    elif isinstance(value, list):
        for item in value:
            summary_only(item)


def seconds_value(value):
    try:
        number = float(value)
    except (ValueError, TypeError) as error:
        raise argparse.ArgumentTypeError("확인 시간은 5~60초여야 합니다") from error
    if not math.isfinite(number) or not 5 <= number <= 60:
        raise argparse.ArgumentTypeError("확인 시간은 5~60초의 유한한 수여야 합니다")
    return number


def inspect_native(library=None):
    from smartcart_uwb.native import NativeParser
    with NativeParser(library) as parser:
        # 실제 라이브러리를 로드하고 ABI를 확인한다. 포트는 열지 않는다.
        require(parser.accepted_frames == 0 and parser.rejected_frames == 0,
                "C 파서 초기 상태가 잘못되었습니다")
        path = parser.library_path
    return path, {"passed": True, "abi_loaded": True, "device_opened": False}


def inspect_field(library):
    from smartcart_field.cli import build_parser, execute
    args = build_parser().parse_args([
        "replay", "--input", str(ROOT / "samples" / "field_acceptance.synthetic.jsonl"),
        "--native-library", str(library),
    ])
    value, code = execute(args)
    require(code == 0 and value["original_source"] == "synthetic" and value["records"] == 175,
            "현장 합성 캡처의 출처·레코드 수가 다릅니다")
    require(value["source_mode"] == "replay" and value["drive_authorized"] is False
            and value["physical_acceptance"] is False and value["inspection_complete"] is False,
            "재생 자료를 실물 검수·구동 허가로 처리했습니다")
    sensors = value["observation"]["sensors"]
    require(all(item["state"] == "fresh" for item in sensors.values()), "합성 단절 뒤 복구가 누락됐습니다")
    left, right, lidar = (sensors[name]["counts"] for name in ("uwb_left", "uwb_right", "lidar_center"))
    require(left.get("explicit_disconnects", 0) > 0 and left.get("freshness_losses", 0) > 0,
            "좌측 UWB 합성 단절을 감지하지 못했습니다")
    require(right.get("id_mismatches", 0) > 0 and right.get("checksum_failures", 0) > 0
            and right.get("recoveries", 0) >= 2, "우측 UWB 합성 오류·복구를 감지하지 못했습니다")
    require(lidar.get("stale_transitions", 0) > 0 and lidar.get("recoveries", 0) > 0,
            "LiDAR 합성 시간 초과·복구를 감지하지 못했습니다")
    return {"passed": True, "records": value["records"], "all_sensors_recovered": True,
            "id_mismatches": right["id_mismatches"], "checksum_failures": right["checksum_failures"],
            "disconnects": left["explicit_disconnects"], "lidar_timeouts": lidar["stale_transitions"]}


def inspect_tracking(library):
    from smartcart_uwb.config import load_config
    from smartcart_uwb.estimation_config import load_estimation_config
    from smartcart_uwb.native import NativeParser
    from smartcart_tracking.config import load_tracking_config
    from smartcart_tracking.pipeline import TrackingPipeline
    from smartcart_tracking.replay import track_file
    mounting = load_config(ROOT / "config" / "dual_uwb.d5.example.json")
    uwb = load_estimation_config(ROOT / "config" / "uwb_estimation.synthetic.json")
    settings = load_tracking_config(ROOT / "config" / "target_tracking.synthetic.json")
    with TrackingPipeline(mounting, uwb, settings, parser_factory=lambda: NativeParser(library)) as pipeline:
        events = list(track_file(ROOT / "samples" / "target_tracking.synthetic.jsonl", pipeline))
    truth = json.loads((ROOT / "samples" / "target_tracking.truth.json").read_text(encoding="utf-8"))
    require(truth["source"] == "synthetic" and bool(events), "합성 추적 기준 자료가 없습니다")
    states = Counter()
    for event in events:
        require(event["original_source"] == "synthetic" and event["source_mode"] == "replay"
                and event["drive_authorized"] is False, "추적 재생 출처·구동 허가가 잘못되었습니다")
        for name in ("track", "uwb_estimate", "lidar_clusters"):
            require(event[name]["drive_authorized"] is False and event[name]["source_mode"] == "replay",
                    "추적 하위 결과의 출처·구동 허가가 잘못되었습니다")
        track = event["track"]
        states[track["state"]] += 1
        if not track["valid"]:
            require(track["position_xy_m"] is None and track["velocity_xy_mps"] is None,
                    "무효 추적에 이전 측정 위치가 남았습니다")
    identifiers = {}
    for expected in truth["checkpoints"]:
        track = events[expected["event_index"]]["track"]
        require(track["valid"] is expected["expected_valid"], "합성 추적 체크포인트의 유효성이 다릅니다")
        if expected["track_id_null"]:
            require(track["track_id"] is None, "고장·단절 후 대상 식별자가 남았습니다")
        if expected["same_track_as"]:
            require(track["track_id"] == identifiers[expected["same_track_as"]], "짧은 가림 뒤 대상이 바뀌었습니다")
        if expected["different_track_from"]:
            require(track["track_id"] != identifiers[expected["different_track_from"]], "단절 후 과거 대상 식별자가 재사용됐습니다")
        if expected["candidate_min"] is not None:
            require(track["candidate_count"] >= expected["candidate_min"], "모호한 합성 후보가 누락됐습니다")
        if expected["expected_valid"]:
            require(math.dist(track["position_xy_m"], expected["expected_position_xy_m"])
                    <= expected["max_target_error_m"], "합성 추적 오차가 기준을 넘었습니다")
        identifiers[expected["label"]] = track["track_id"]
    require(states["tracking"] > 0 and states["ambiguous"] > 0 and states["coasting"] > 0,
            "합성 추적·모호성·가림 상태가 빠졌습니다")
    return {"passed": True, "records": len(events), "checkpoints": len(truth["checkpoints"]),
            "states": dict(sorted(states.items())), "invalid_measurements_cleared": True}


def available_port():
    with socket.socket() as handle:
        handle.bind(("127.0.0.1", 0))
        return handle.getsockname()[1]


def request_json(port, path):
    # 프록시·리디렉션 없이 검사에서 직접 시작한 루프백 서버만 조회한다.
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=.75)
    try:
        connection.request("GET", path)
        response = connection.getresponse()
        require(response.status == 200, "재생 HTTP 응답이 정상이 아닙니다")
        require(response.getheader("X-Content-Type-Options") == "nosniff", "HTTP 보안 헤더가 누락됐습니다")
        raw = response.read(1024 * 1024 + 1)
        require(len(raw) <= 1024 * 1024, "HTTP 응답 크기 상한을 넘었습니다")
        return json.loads(raw)
    finally:
        connection.close()


def stop_child(process):
    """이 검사가 소유한 서버만 종료하며 시간 초과 시 강제 종료한다."""
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)
    require(process.poll() is not None, "재생 자식 프로세스가 종료되지 않았습니다")


def validate_http_view(value):
    require(value.get("schema") == "smartcart.tracking_view.v1" and value.get("mode") == "replay"
            and value.get("source_mode") == "replay" and value.get("drive_authorized") is False
            and value.get("provisional") is True, "HTTP 추적 출처·구동 허가가 잘못되었습니다")
    if not value["valid"] and value.get("result") is not None:
        require(value["result"]["track"]["position_xy_m"] is None
                and value["result"]["track"]["velocity_xy_mps"] is None,
                "무효 HTTP 추적에 측정 위치가 남았습니다")
    if value.get("state") == "ended":
        require(value["valid"] is False and value.get("result") is not None, "재생 종료 상태가 잘못되었습니다")
        track = value["result"]["track"]
        require(track["track_id"] is None and track["predicted_position_xy_m"] is None,
                "재생 종료 후 대상 식별자·예측이 남았습니다")


def inspect_http(library, seconds):
    port = available_port()
    command = [sys.executable, "-X", "utf8", str(ROOT / "sensing.py"), "serve", "--mode", "replay",
               "--tracking-input", str(ROOT / "samples" / "target_tracking.synthetic.jsonl"),
               "--tracking-mounting-config", str(ROOT / "config" / "dual_uwb.d5.example.json"),
               "--tracking-uwb-config", str(ROOT / "config" / "uwb_estimation.synthetic.json"),
               "--tracking-config", str(ROOT / "config" / "target_tracking.synthetic.json"),
               "--tracking-native-library", str(library), "--tracking-speed", "1",
               "--host", "127.0.0.1", "--port", str(port)]
    environment = dict(os.environ, PYTHONUTF8="1", ROS_LOCALHOST_ONLY="1")
    process = subprocess.Popen(command, cwd=ROOT, env=environment, stdin=subprocess.DEVNULL,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, shell=False)
    valid_seen = ended_seen = health_seen = False
    requests = 0
    sequence = -1
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            require(process.poll() is None, "재생 HTTP 서버가 검사 도중 종료됐습니다")
            try:
                if not health_seen:
                    health = request_json(port, "/api/health")
                    summary_only(health)
                    require(health["sensing"]["mode"] == "replay" and health["sensing"]["synthetic"] is True,
                            "HTTP 상태가 합성 재생으로 표시되지 않았습니다")
                    health_seen = True
                value = request_json(port, "/api/tracking")
            except (ConnectionError, TimeoutError, http.client.HTTPException):
                time.sleep(.05)
                continue
            validate_http_view(value)
            require(value["sequence"] >= sequence, "HTTP 표시 순번이 역행했습니다")
            sequence = value["sequence"]
            requests += 1
            valid_seen |= value["valid"] is True
            ended_seen |= value["state"] == "ended"
            if ended_seen:
                break
            time.sleep(.04)
        require(health_seen and valid_seen and ended_seen,
                "제한 시간 내 HTTP 상태·유효 추적·재생 종료를 모두 확인하지 못했습니다")
    finally:
        stop_child(process)
    with socket.socket() as handle:
        handle.settimeout(.5)
        require(handle.connect_ex(("127.0.0.1", port)) != 0, "종료 후 HTTP 포트가 열려 있습니다")
    return {"passed": True, "health_checked": health_seen, "responses": requests,
            "valid_tracking_seen": valid_seen, "end_clears_target": ended_seen,
            "child_exit_observed": True, "listener_closed": True}


def build_parser():
    parser = argparse.ArgumentParser(description="Pi·센서 없는 SmartCart 사전 검증 · 장치 개방 없음")
    parser.add_argument("--seconds", type=seconds_value, default=8,
                        help="재생 HTTP 확인 제한 시간 5~60초, 기본 8초")
    parser.add_argument("--native-library", type=Path, help="현재 PC 플랫폼에 맞게 빌드한 C 파서")
    parser.add_argument("--output", type=Path, help="집계만 저장할 새 로컬 JSON 파일")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    report = {"schema": "smartcart.offline_verification.v1", "source_mode": "replay",
              "original_source": "synthetic", "device_opened": False, "drive_authorized": False,
              "physical_acceptance": False, "passed": False, "checks": {}}
    try:
        if args.output is not None and (args.output.exists() or args.output.is_symlink()):
            raise ValueError("기존 보고서는 덮어쓰지 않습니다")
        report["active_check"] = "native"
        library, report["checks"]["native"] = inspect_native(args.native_library)
        report["active_check"] = "field_replay"
        report["checks"]["field_replay"] = inspect_field(library)
        report["active_check"] = "tracking_replay"
        report["checks"]["tracking_replay"] = inspect_tracking(library)
        report["active_check"] = "http_replay"
        report["checks"]["http_replay"] = inspect_http(library, args.seconds)
        del report["active_check"]
        report["passed"] = True
        summary_only(report)
        if args.output is not None:
            write_report(args.output, report)
        print(encoded_report(report), end="")
        return 0
    except KeyboardInterrupt:
        print("사전 검증을 중단했습니다. 완료로 판정하지 않습니다.", file=sys.stderr)
        return 130
    except (OSError, ValueError, TypeError, RuntimeError, ImportError, KeyError, subprocess.SubprocessError) as error:
        # 라이브러리·입력 경로와 합성 좌표를 오류 보고서로 복사하지 않는다.
        report["error_type"] = type(error).__name__
        summary_only(report)
        print(encoded_report(report), end="")
        print("사전 검증 실패: 현재 플랫폼 C 파서와 합성 자료, 루프백 실행 환경을 확인하십시오.", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
